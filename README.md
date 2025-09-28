STM32 기반 MPU-6500 IMU 센서 드라이버
STM32 HAL 라이브러리를 사용하여 MPU-6500 (GY-6500 모듈) 6축 IMU 센서를 제어하는 드라이버 코드입니다.

주요 기능
센서 초기화: I2C 통신을 통해 MPU-6500을 초기화하고 WHO_AM_I 레지스터를 확인하여 연결 상태 검증.

데이터 수집: 가속도(Accel), 자이로(Gyro), 온도(Temp) 데이터의 Raw 값을 수집.

센서 캘리브레이션: 측정 데이터의 영점 오차(Zero-bias) 보정을 위한 오프셋 계산.

측정 범위 설정: 가속도계와 자이로스코프의 측정 범위(Range)를 동적으로 설정.

디지털 저역 통과 필터(DLPF): 노이즈 감소를 위한 DLPF 대역폭 설정.

자세 추정: 상보 필터(Complementary Filter)를 이용해 Roll, Pitch, Yaw 각도 계산.

개발 환경
MCU: STM32 Series

Sensor: MPU-6500 (GY-6500 Module)

IDE: STM32CubeIDE

Library: STM32 HAL Library

MPU-6500 IMU Sensor Driver for STM32
This is a driver for the MPU-6500 6-axis IMU sensor (GY-6500 module), implemented using the STM32 HAL library.

Features
Sensor Initialization: Initializes the MPU-6500 via I2C and verifies the connection by checking the WHO_AM_I register.

Data Acquisition: Reads raw data for accelerometer, gyroscope, and temperature.

Sensor Calibration: Calculates offsets to correct for zero-bias errors in measurements.

Configurable Range: Dynamically sets the measurement range for the accelerometer and gyroscope.

Digital Low-Pass Filter (DLPF): Configures the DLPF bandwidth to reduce noise.

Attitude Estimation: Calculates Roll, Pitch, and Yaw angles using a complementary filter.

Environment
MCU: STM32 Series

Sensor: MPU-6500 (GY-6500 Module)

IDE: STM32CubeIDE

Library: STM32 HAL Library