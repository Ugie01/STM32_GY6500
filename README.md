## MPU-6500 (GY-6500) IMU 센서 드라이버 (STM32 HAL 기반)

### 1. 개요

본 라이브러리는 HAL 라이브러리 기반 **STM32 마이크로컨트롤러**에서 **MPU-6500 6축 IMU(관성 측정 장치) 센서**를 사용하기 위한 라이브러리 입니다.

---

## 2. 개발 환경

* **MCU**: `STM32 Series (STM32F407vet6)`
* **IDE**: `STM32CubeIDE 1.19.0`
* **Core Library**: `STM32 HAL Library`
* **Sensor**: `GY6500`
* **Interface**: `I2C`

---

### 3. 핵심 기능 (자세한 함수 설명은 6번 참고)

* **STM32 HAL 라이브러리 호환**: STM32CubeMX로 생성된 프로젝트에 쉽게 통합 가능
* **I2C 통신**: I2C를 이용한 센서 데이터 통신
* **다양한 데이터 수집**:
    * 3축 가속도 (g 단위)
    * 3축 각속도 (dps, degrees per second 단위)
    * 온도 (°C 단위)
* **자세 각도 계산**:
    * 가속도계와 자이로스코프 데이터를 융합하는 **상보 필터(Complementary Filter)** 적용
    * 안정적인 **Roll** 및 **Pitch** 각도 계산
    * (지자계 센서 없이) 자이로스코프 적분을 통한 **Yaw** 각도 계산 - 드리프트 존재
* **센서 설정 기능**:
    * 가속도계 및 자이로스코프의 측정 범위 설정
    * 디지털 저역 통과 필터(DLPF) 설정으로 노이즈 제거
    * 각 센서 축 및 온도 센서 개별 활성화/비활성화
* **캘리브레이션**: 정지 상태에서의 센서 오프셋(offset)을 계산하여 측정 정확도 향상

---

### 4. 사용 방법

1.  **하드웨어 연결**: STM32 보드의 I2C 핀(SCL, SDA)을 MPU-6500 모듈에 연결하고, VCC와 GND를 연결
2.  **STM32CubeMX 설정**:
    * 사용할 I2C 활성화
    * `printf` 출력을 위해 사용할 UART 활성화
3.  **프로젝트에 파일 추가**:
    * `gy6500.c` 파일을 프로젝트의 `Src` 폴더에 추가
    * `gy6500.h` 파일을 프로젝트의 `Inc` 폴더에 추가
4.  **`main.c`에 코드 작성**:
    * `extern I2C_HandleTypeDef hi2c2` 처럼 활성화 한 I2C 적용
    * `gy6500.h` 파일을 `#include`
    * `main` 함수 내에서 `GyData` 구조체 변수를 선언
    * `Gy6500_init()` 함수를 호출하여 센서를 초기화
    * `while(1)` 루프 안에서 센서 데이터를 읽고 자세 각도를 계산하는 함수를 주기적으로 호출

---

### 5. 예제 코드

#### 5.1. 기본 사용 (자세 각도 계산)

`main.c`의 `while(1)` 루프에서 다음 코드를 실행하여 Roll, Pitch, Yaw 값을 지속적으로 계산하고 출력

```c
/* main.c */

#include "gy6500.h"
#include <stdio.h>

// ... (main 함수 앞부분 생략)

int main(void)
{
    // ... (HAL_Init, SystemClock_Config, 주변장치 초기화)

    GyData gy6500_1;

    // 센서 초기화
    if (Gy6500_init(&gy6500_1, &hi2c2, DEV_ADDR) == HAL_OK) {
        printf("센서 연결 완료\r\n");
    } else {
        printf("센서 연결 실패\r\n");
        Error_Handler();
    }

    HAL_Delay(1000); // 센서 안정화 대기

    // 시간차(dt) 계산을 위한 변수
    uint32_t last_tick = HAL_GetTick();
    uint32_t current_tick;
    float dt;

    while (1)
    {
        // 시간 변화량(dt) 계산
        current_tick = HAL_GetTick();
        dt = (float)(current_tick - last_tick) / 1000.0f; // 초 단위로 변환
        last_tick = current_tick;

        // 모든 센서 데이터 읽기
        getAllData(&gy6500_1);

        // 상보 필터를 이용해 각도 계산
        calculateAngles(&gy6500_1, dt);

        // 계산된 각도 출력 (받고싶은 형식으로 변경)
        printf("c,%.2f,%.2f,%.2f\r\n", gy6500_1.roll, gy6500_1.pitch, gy6500_1.yaw);

        HAL_Delay(10); // 적절한 딜레이
    }
}
```

---

### 6. 함수 설명

#### 6.1. 초기화 및 탐색

* `void Scanning_I2C()`
    * I2C 버스에 연결된 모든 장치를 스캔하여 주소를 출력한다. 센서가 정상적으로 연결되었는지 확인할 때 유용
* `HAL_StatusTypeDef Gy6500_init(GyData *dev, I2C_HandleTypeDef *i2c_handle, uint8_t address)`
    * `GyData` 구조체와 I2C 핸들을 받아 센서를 초기화한다.
    * 센서의 `WHO_AM_I` 레지스터를 읽어 MPU-6500이 맞는지 확인하고, 절전 모드를 해제하여 데이터 수집을 준비

#### 6.2. 데이터 수집

* `HAL_StatusTypeDef getAllData(GyData *dev)`
    * 가속도, 온도, 자이로스코프의 모든 원시 데이터를 한 번에 `get`
    * 읽어온 16비트 정수 값을 물리 단위(g, °C, dps)로 변환하여 `GyData` 구조체에 저장
* `HAL_StatusTypeDef getAllDataOffset(GyData *dev)`
    * `getAllData`와 유사하지만, 사전에 계산된 `오프셋 값을 적용`하여 보정된 데이터를 `GyData` 구조체에 저장
* `HAL_StatusTypeDef getAccelerometer(GyData *dev)`
    * 가속도계(X, Y, Z축) 데이터만 `get`
* `HAL_StatusTypeDef getGyroscope(GyData *dev)`
    * 자이로스코프(X, Y, Z축) 데이터만 `get`
* `HAL_StatusTypeDef getTemp(GyData *dev)`
    * 온도 센서 데이터만 `get`

#### 6.3. 센서 설정

* `HAL_StatusTypeDef setSensorState(GyData *dev, uint8_t set_data)`
    * `PWR_MGMT_2` 레지스터를 제어하여 가속도 및 자이로스코프의 각 축을 개별적으로 활성화 또는 비활성화
* `HAL_StatusTypeDef setTempState(GyData *dev, bool set_data)`
    * `PWR_MGMT_1` 레지스터를 제어하여 온도 센서를 활성화 또는 비활성화
* `HAL_StatusTypeDef setGyroRange(GyData *dev, gyro_range_t range)`
    * 자이로스코프의 최대 측정 범위를 설정. (±250, ±500, ±1000, ±2000 dps)
* `HAL_StatusTypeDef setAccelRange(GyData *dev, accel_range_t range)`
    * 가속도계의 최대 측정 범위를 설정 (±2g, ±4g, ±8g, ±16g)
* `HAL_StatusTypeDef setAccelDLPF(GyData *dev, accel_dlpf_t mode)`
    * 가속도계의 디지털 저역 통과 필터(DLPF) 대역폭을 설정
* `HAL_StatusTypeDef setGyroDLPF(GyData *dev, gyro_dlpf_t mode)`
    * 자이로스코프의 디지털 저역 통과 필터(DLPF) 대역폭을 설정
* `HAL_StatusTypeDef resetAccelDLPF(GyData *dev)` / `HAL_StatusTypeDef resetGyroDLPF(GyData *dev)`
    * 설정된 DLPF를 비활성화/리셋

#### 6.4. 캘리브레이션 및 자세 계산

* `void calibrateGyro(GyData *dev, uint16_t samples)` / `void calibrateAccel(GyData *dev, uint16_t samples)`
    * 센서가 수평 및 정지 상태일 때 지정된 `samples` 수만큼 데이터를 평균내어 오프셋을 계산
    * 이 오프셋 값은 이후 측정에서 오차를 보정하는 데 사용
* `void calculateAngles(GyData *dev, float dt)`
    * `dt`(시간 변화량)를 이용하여 자세 각도를 계산
    * **Roll/Pitch**: 가속도계 값으로 계산한 절대 각도와 자이로스코프 값을 적분한 각도를 상보 필터로 융합하여 계산 (단기적인 자이로의 정확성과 장기적인 가속도계의 안정성을 모두 취함)
    * **Yaw**: Z축 자이로스코프 값을 적분하여 계산 (드리프트 오차 누적 가능성 있음)

---

### 7. 주의사항
Yaw 드리프트: 이 라이브러리는 지자계 센서가 없어 Z축 자이로 오차가 누적되어 **Yaw 각도가 틀어지는 현상(Drift)**이 발생 - 정확한 방위각이 필요하다면 9축 센서 사용을 권장

캘리브레이션: 정확한 측정을 위해, 프로그램 시작 시 센서가 평평하고 움직이지 않는 상태에서 캘리브레이션 함수를 반드시 호출

I2C 주소: MPU-6500 모듈의 AD0 핀 설정에 따라 I2C 주소는 0x68 또는 0x69가 됨. gy6500.h의 DEV_ADDR 값을 실제 환경에 맞게 확인

민감도 상수: 측정 범위를 변경(setGyroRange, setAccelRange)하면 데이터시트를 참고하여 gy6500.c의 GYRO_SENSITIVITY와 ACCEL_SENSITIVITY 상수도 함께 수정

---

### 8. 추가 설명

#### 8.1 성능 테스트 코드
아래 코드는 각 함수들이나 코드들의 성능을 테스트하기 유용한 코드입니다.
```c
/* main.c */

#include "gy6500.h"
#include <stdio.h>

// ... (main 함수 앞부분 생략)

int main(void)
{
// ... (HAL_Init, SystemClock_Config, 주변장치 초기화)

//   DWT 초기화 및 사이클 카운터 활성화
   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    while (1)
    {
        DWT->CYCCNT = 0; // 카운터 초기화
//        시간 측정 시작 (성능 테스트)
       uint32_t start_cycle = DWT->CYCCNT;
       setSensorState(&gy6500_1, ALL_SENSORS_ON);
       setTempState(&gy6500_1, TEMP_DIS);

//        ------------ 여기에 성능 테스트 할 코드 추가 ------------

//       시간 측정 종료
       uint32_t end_cycle = DWT->CYCCNT;
       uint32_t elapsed_cycles = end_cycle - start_cycle;

//        실행 시간 계산 (SystemCoreClock은 시스템 클럭 주파수, 예: 168MHz)
//        시간(us) = (소요 사이클 수 * 1,000,000) / 시스템 클럭 주파수
       double elapsed_us = (double) elapsed_cycles * 1000000.0 / SystemCoreClock;
//        결과 출력 (printf나 SWV로 확인)
       printf("소요 사이클: %lu cycles\r\n", elapsed_cycles);
       printf("실행 시간: %.3f us\r\n", elapsed_us);
    }
}
```