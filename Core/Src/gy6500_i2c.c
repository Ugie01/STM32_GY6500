/*
 * gy6500.c
 *
 *  Created on: Sep 14, 2025
 *      Author: ugie01
 */

/*
 * HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
 * @brief  지정한 I2C 주소에 장치가 응답하는지 확인하는 함수 (장치 존재 유무 체크)
 * @param  hi2c: 사용할 I2C 포트 핸들 (예: &hi2c1)
 * @param  DevAddress: 데이터를 읽어올 슬레이브 장치의 7비트 I2C 주소 (예: 0x68), 사용할땐 주소 << 1 을 해야함
 * @param  Trials: 응답이 없을 경우 재시도할 횟수
 * @param  Timeout: 통신이 완료될 때까지 기다릴 최대 시간 (밀리초 단위). HAL_MAX_DELAY를 사용하면 무한정 대기
 * @retval HAL_StatusTypeDef: 장치가 응답하면 HAL_OK, 응답이 없으면 HAL_ERROR 또는 HAL_TIMEOUT을 반환
 */

/*
 * HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
 * @brief  I2C 장치 내부의 특정 메모리 주소(레지스터)에서부터 지정된 크기의 데이터를 읽어오는 함수
 * @brief  '주소 쓰기' 동작과 '데이터 읽기' 동작이 하나로 합쳐져 있어 센서 데이터 읽기에 매우 편리
 * @param  hi2c: 사용할 I2C 포트 핸들 (예: &hi2c1)
 * @param  DevAddress: 데이터를 읽어올 슬레이브 장치의 7비트 I2C 주소 (예: 0x68), 사용할땐 주소 << 1 을 해야함
 * @param  MemAddress: 읽기를 시작할 장치 내부의 메모리(레지스터) 시작 주소 (예: WHO_AM_I 레지스터 주소 0x75)
 * @param  MemAddSize: 메모리(레지스터) 주소 자체의 크기. 보통 8비트 주소를 사용하므로 I2C_MEMADD_SIZE_8BIT를 사용
 * @param  pData: 읽어온 데이터를 저장할 버퍼(배열)의 포인터
 * @param  Size: 읽어올 데이터의 크기 (바이트 단위)
 * @param  Timeout: 통신이 완료될 때까지 기다릴 최대 시간 (밀리초 단위). HAL_MAX_DELAY를 사용하면 무한정 대기
 * @retval HAL_StatusTypeDef: 장치가 응답하고 쓰기가 완료되면 HAL_OK, 응답이 없으면 HAL_ERROR 또는 HAL_TIMEOUT을 반환
 */

/*
 * HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
 * @brief  I2C 장치 내부의 특정 메모리 주소(레지스터)에 지정된 크기의 데이터를 쓰는 함수
 * @brief  주로 센서의 설정 값을 변경하거나 특정 동작을 명령할 때 사용
 * @param  hi2c: 사용할 I2C 포트 핸들 (예: &hi2c1)
 * @param  DevAddress: 데이터를 쓸 슬레이브 장치의 7비트 I2C 주소 (예: 0x68), 사용할땐 주소 << 1 을 해야함
 * @param  MemAddress: 쓰기를 시작할 장치 내부의 메모리(레지스터) 시작 주소 (예: PWR_MGMT_1 레지스터 주소 0x6B)
 * @param  MemAddSize: 메모리(레지스터) 주소 자체의 크기. 보통 8비트 주소를 사용하므로 I2C_MEMADD_SIZE_8BIT를 사용
 * @param  pData: 장치에 쓸 데이터가 담겨있는 버퍼(배열)의 포인터
 * @param  Size: 쓸 데이터의 크기 (바이트 단위)
 * @param  Timeout: 통신이 완료될 때까지 기다릴 최대 시간 (밀리초 단위). HAL_MAX_DELAY를 사용하면 무한정 대기
 * @retval HAL_StatusTypeDef: 장치가 응답하고 쓰기가 완료되면 HAL_OK, 응답이 없으면 HAL_ERROR 또는 HAL_TIMEOUT을 반환
 */

#include <gy6500_i2c.h>

// 현재 센서 설정에 맞는 민감도(Sensitivity) 값을 여기에 입력
// 자이로  (±250dps): 131.0
// 가속도 (±2g)    : 16384.0
const float GYRO_SENSITIVITY = 131.0f;
const float ACCEL_SENSITIVITY = 16384.0f;
int16_t ONE_G_RAW = 16384;
const uint32_t TIME_OUT = 10;

void Scanning_I2C()
{
    printf("Scanning I2C bus...\r\n");

    for (uint8_t address = 1; address < 128; address++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t) (address << 1), 2, 10) == HAL_OK)
        {
            printf("I2C device found at address 0x%X\r\n", address);
        }
    }
}

HAL_StatusTypeDef Gy6500_init(GyData *dev, I2C_HandleTypeDef *i2c_handle, uint8_t address)
{
    uint8_t who_am_i_value = 0;
    dev->address = address;
    dev->hi2c = i2c_handle;

    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->address, 2, 100) != HAL_OK)
        return HAL_ERROR; // MPU-6500 Not Found.

    HAL_I2C_Mem_Read(dev->hi2c, dev->address, WHO_AM_I, 1, &who_am_i_value, 1, TIME_OUT);
    if (who_am_i_value != 0x70)
        return HAL_ERROR; // Device found, but it's not MPU-6500.

    uint8_t write = 0x01;
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->address, PWR_MGMT_1, 1, &write, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(100); // 클럭이 안정화될 때까지 잠시 대기
    return HAL_OK; // MPU-6500 Found
}

HAL_StatusTypeDef getAllData(GyData *dev)
{
    uint8_t raw_data[14];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, ACCEL_XOUT_H, 1, raw_data, 14, TIME_OUT);
    if (status == HAL_OK)
    {
        dev->accel_x = (float) ((int16_t) ((raw_data[0] << 8) | raw_data[1])) / ACCEL_SENSITIVITY;
        dev->accel_y = (float) ((int16_t) ((raw_data[2] << 8) | raw_data[3])) / ACCEL_SENSITIVITY;
        dev->accel_z = (float) ((int16_t) ((raw_data[4] << 8) | raw_data[5])) / ACCEL_SENSITIVITY;

        dev->temp = ((float) ((int16_t) ((raw_data[6] << 8) | raw_data[7])) / 340.0f) + 36.53f;

        dev->gyro_x = (float) ((int16_t) ((raw_data[8] << 8) | raw_data[9])) / GYRO_SENSITIVITY;
        dev->gyro_y = (float) ((int16_t) ((raw_data[10] << 8) | raw_data[11])) / GYRO_SENSITIVITY;
        dev->gyro_z = (float) ((int16_t) ((raw_data[12] << 8) | raw_data[13])) / GYRO_SENSITIVITY;
    }
    return status;
}

HAL_StatusTypeDef getAllDataOffset(GyData *dev)
{
    uint8_t raw_data[14];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, ACCEL_XOUT_H, 1, raw_data, 14, 100);
    if (status == HAL_OK)
    {
        dev->accel_x = ((float)((int16_t)((raw_data[0] << 8) | raw_data[1])) - dev->accel_offset_x) / ACCEL_SENSITIVITY;
        dev->accel_y = ((float)((int16_t)((raw_data[2] << 8) | raw_data[3])) - dev->accel_offset_y) / ACCEL_SENSITIVITY;
        dev->accel_z = ((float)((int16_t)((raw_data[4] << 8) | raw_data[5])) - dev->accel_offset_z) / ACCEL_SENSITIVITY;

        dev->temp = ((float)((int16_t)((raw_data[6] << 8) | raw_data[7])) / 340.0f) + 36.53f;

        dev->gyro_x = ((float)((int16_t)((raw_data[8] << 8) | raw_data[9])) - dev->gyro_offset_x) / GYRO_SENSITIVITY;
        dev->gyro_y = ((float)((int16_t)((raw_data[10] << 8) | raw_data[11])) - dev->gyro_offset_y) / GYRO_SENSITIVITY;
        dev->gyro_z = ((float)((int16_t)((raw_data[12] << 8) | raw_data[13])) - dev->gyro_offset_z) / GYRO_SENSITIVITY;
    }

    return status;
}


HAL_StatusTypeDef getAccelerometer(GyData *dev)
{
    uint8_t raw_data[6];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, ACCEL_XOUT_H, 1, raw_data, 6, TIME_OUT);
    if (status == HAL_OK)
    {
        dev->accel_x = (float) ((int16_t) ((raw_data[0] << 8) | raw_data[1])) / ACCEL_SENSITIVITY;
        dev->accel_y = (float) ((int16_t) ((raw_data[2] << 8) | raw_data[3])) / ACCEL_SENSITIVITY;
        dev->accel_z = (float) ((int16_t) ((raw_data[4] << 8) | raw_data[5])) / ACCEL_SENSITIVITY;
    }
    return status;
}

HAL_StatusTypeDef getGyroscope(GyData *dev)
{
    uint8_t raw_data[6];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, GYRO_XOUT_H, 1, raw_data, 6, TIME_OUT);
    if (status == HAL_OK)
    {
        dev->gyro_x = (float) ((int16_t) ((raw_data[0] << 8) | raw_data[1])) / GYRO_SENSITIVITY;
        dev->gyro_y = (float) ((int16_t) ((raw_data[2] << 8) | raw_data[3])) / GYRO_SENSITIVITY;
        dev->gyro_z = (float) ((int16_t) ((raw_data[4] << 8) | raw_data[5])) / GYRO_SENSITIVITY;
    }
    return status;
}

HAL_StatusTypeDef getTemp(GyData *dev)
{
    uint8_t raw_data[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, TEMP_OUT_H, 1, raw_data, 2, TIME_OUT);
    if (status == HAL_OK)
    {
        dev->temp = ((float) ((int16_t) ((raw_data[0] << 8) | raw_data[1])) / 340.0f) + 36.53f;
    }
    return status;
}

HAL_StatusTypeDef setSensorState(GyData *dev, uint8_t set_data)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, PWR_MGMT_2, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & 0xC0) | set_data; // 1100 0000 과 0011 1111로 7~8 비트 보호
    dev->state = (dev->state & 0x40) | set_data;	// dev의 센서 state 변경

    HAL_Delay(20); // 안정화될 때까지 잠시 대기
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, PWR_MGMT_2, 1, &reg_data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef setTempState(GyData *dev, bool set_data)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, PWR_MGMT_1, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & 0xF7) | (!set_data << 3);
    dev->state = (dev->state & 0xBF) | (!set_data << 6);	// dev의 센서 state 변경

    HAL_Delay(20); // 안정화될 때까지 잠시 대기
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, PWR_MGMT_1, 1, &reg_data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef setGyroRange(GyData *dev, gyro_range_t range)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, GYRO_CONFIG, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & ~0x18) | (range << 3);

    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, GYRO_CONFIG, 1, &reg_data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef setAccelRange(GyData *dev, accel_range_t range)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, ACCEL_CONFIG, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & ~0x18) | (range << 3);

    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, ACCEL_CONFIG, 1, &reg_data, 1, HAL_MAX_DELAY);
}

// 가속도 센서 DLPF 적용 함수
HAL_StatusTypeDef setAccelDLPF(GyData *dev, accel_dlpf_t mode)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, ACCEL_CONFIG, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & ~0x07) | mode;
//    reg_data = (reg_data & ~0x08) | 0x00;

    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, ACCEL_CONFIG, 1, &reg_data, 1, HAL_MAX_DELAY);
}

// 자이로 및 온도 센서 DLPF 적용 함수
HAL_StatusTypeDef setGyroDLPF(GyData *dev, gyro_dlpf_t mode)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, GYRO_CONFIG, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

//    reg_data = (reg_data & ~0x03) | 0x00;
//    status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, GYRO_CONFIG, 1, &reg_data, 1, HAL_MAX_DELAY);
//    if (status != HAL_OK)
//        return status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, CONFIG, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & ~0x07) | mode;

    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, CONFIG, 1, &reg_data, 1, HAL_MAX_DELAY);
}

// 가속도 센서 DLPF 리셋 함수
HAL_StatusTypeDef resetAccelDLPF(GyData *dev)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, ACCEL_CONFIG_2, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & ~0x08) | 0x08;

    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, ACCEL_CONFIG_2, 1, &reg_data, 1, HAL_MAX_DELAY);
}

// 자이로 및 온도 센서 DLPF 리셋 함수
HAL_StatusTypeDef resetGyroDLPF(GyData *dev)
{
    uint8_t reg_data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, GYRO_CONFIG, 1, &reg_data, 1, TIME_OUT);
    if (status != HAL_OK)
        return status;

    reg_data = (reg_data & ~0x03) | 0x01;

    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, GYRO_CONFIG, 1, &reg_data, 1, HAL_MAX_DELAY);
}

void calibrateGyro(GyData *dev, uint16_t samples)
{
    int32_t gx = 0, gy = 0, gz = 0;
    uint8_t raw_data[6];

    for (uint16_t i = 0; i < samples; i++)
    {
        HAL_I2C_Mem_Read(dev->hi2c, dev->address, GYRO_XOUT_H, 1, raw_data, 6, 100);
        gx += (int16_t)((raw_data[0] << 8) | raw_data[1]);
        gy += (int16_t)((raw_data[2] << 8) | raw_data[3]);
        gz += (int16_t)((raw_data[4] << 8) | raw_data[5]);
        HAL_Delay(2);
    }

    dev->gyro_offset_x = (float)gx / samples;
    dev->gyro_offset_y = (float)gy / samples;
    dev->gyro_offset_z = (float)gz / samples;
}

void calibrateAccel(GyData *dev, uint16_t samples)
{
    int32_t ax = 0, ay = 0, az = 0;
    uint8_t raw_data[6];

    for (uint16_t i = 0; i < samples; i++)
    {
        HAL_I2C_Mem_Read(dev->hi2c, dev->address, ACCEL_XOUT_H, 1, raw_data, 6, 100);
        ax += (int16_t)((raw_data[0] << 8) | raw_data[1]);
        ay += (int16_t)((raw_data[2] << 8) | raw_data[3]);
        az += (int16_t)((raw_data[4] << 8) | raw_data[5]) - ONE_G_RAW; // 중력 보정
        HAL_Delay(2);
    }

    dev->accel_offset_x = (float)ax / samples;
    dev->accel_offset_y = (float)ay / samples;
    dev->accel_offset_z = (float)az / samples;
}

void calculateAngles(GyData *dev, float dt)
{
    // 가속도 센서로 현재 각도 추정 (저주파 성분)
    // atan2f 함수 덕분에 Roll은 -180~180°, Pitch는 -90~90° 표준 범위로 이미 계산
    float accel_roll = atan2f(dev->accel_y, dev->accel_z) * RADIAN_TO_DEGREE;
    float accel_pitch = atan2f(-dev->accel_x, sqrtf(dev->accel_y * dev->accel_y + dev->accel_z * dev->accel_z)) * RADIAN_TO_DEGREE;

    // 자이로 센서로 각도 변화량 예측 (고주파 성분)
    float gyro_predicted_roll = dev->roll + dev->gyro_x * dt;
    float gyro_predicted_pitch = dev->pitch + dev->gyro_y * dt;

    // 상보 필터를 이용해 Roll, Pitch 오차 보정
    dev->roll = ALPHA * gyro_predicted_roll + (1.0f - ALPHA) * accel_roll;
    dev->pitch = ALPHA * gyro_predicted_pitch + (1.0f - ALPHA) * accel_pitch;

    // 자이로 센서 값을 이용한 Yaw 계산 (주의: 드리프트 오차 누적됨!)
    dev->yaw += dev->gyro_z * dt;

    // Yaw 값이 -180 ~ 180도 범위를 유지하도록 정규화
    while (dev->yaw > 180.0f) {
        dev->yaw -= 360.0f;
    }
    while (dev->yaw <= -180.0f) {
        dev->yaw += 360.0f;
    }
//    printf("roll,%.2f,%.2f,%.2f\r\n", accel_roll, gyro_predicted_roll, dev->roll);
//    printf("pitch,%.2f,%.2f,%.2f\r\n", accel_pitch, gyro_predicted_pitch, dev->pitch);
//    printf("yaw,%.2f\r\n", dev->yaw);
}
