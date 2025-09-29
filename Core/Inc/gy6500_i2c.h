/*
 * gy6500.h
 *
 *  Created on: Sep 14, 2025
 *      Author: ugie01
 */

#ifndef INC_GY6500_I2C_H_
#define INC_GY6500_I2C_H_

#include "main.h"
#include "i2c.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ALPHA 0.98f
#define RADIAN_TO_DEGREE (180.0f / M_PI)
#define DEGREE_TO_RADIAN (M_PI / 180.0f)

// 디바이스 주소
#define DEV_ADDR 		(0x68 << 1)

// 레지스터 관련 주소 (데이터시트 참고)
#define WHO_AM_I 		0x75
#define ACCEL_XOUT_H  	0x3B
#define GYRO_XOUT_H   	0x43
#define TEMP_OUT_H    	0x41
#define PWR_MGMT_1		0x6B
#define PWR_MGMT_2		0x6C
#define GYRO_CONFIG 	0x1B
#define ACCEL_CONFIG	0x1C
#define CONFIG     	 	0x1A
#define GYRO_CONFIG	   	0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG_2   0x1D
#define XG_OFFSET_H    0x13
#define XG_OFFSET_L    0x14
#define YG_OFFSET_H    0x15
#define YG_OFFSET_L    0x16
#define ZG_OFFSET_H    0x17
#define ZG_OFFSET_L    0x18
#define XA_OFFSET_H    0x77
#define XA_OFFSET_L    0x78
#define YA_OFFSET_H    0x7A
#define YA_OFFSET_L    0x7B
#define ZA_OFFSET_H    0x7D
#define ZA_OFFSET_L    0x7E

// 센서 데이터 ON/OFF
#define ACCEL_X_DIS (1 << 5) // 0b00100000
#define ACCEL_Y_DIS (1 << 4) // 0b00010000
#define ACCEL_Z_DIS (1 << 3) // 0b00001000
#define GYRO_X_DIS  (1 << 2) // 0b00000100
#define GYRO_Y_DIS  (1 << 1) // 0b00000010
#define GYRO_Z_DIS  (1 << 0) // 0b00000001
#define TEMP_ON     true     // 0b00001000
#define TEMP_DIS    false    // 0b00001000

// 전체 센서 데이터 ON/OFF
#define ALL_SENSORS_ON  (0x00)
#define ALL_GYRO_DIS    (GYRO_X_DIS | GYRO_Y_DIS | GYRO_Z_DIS)
#define ALL_ACCEL_DIS   (ACCEL_X_DIS | ACCEL_Y_DIS | ACCEL_Z_DIS)

typedef struct _GyData
{
    // 사용할 I2C 포트
    I2C_HandleTypeDef *hi2c;

    // 사용할 슬레이브 주소
    uint8_t address;

    // 센서 데이터 변수
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temp;

    // 계산된 자세 데이터
    float roll, pitch, yaw;

    // 오프셋 값
    float accel_offset_x, accel_offset_y, accel_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;

    // 6bit - temp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z - 0bit     --->     0 활성화 , 1 비활성화
    uint8_t state;
} GyData;

// 자이로스코프 측정 범위   ±250dps(00), ±500dps (01), ±1000dps(10), ±2000dps(11)
typedef enum
{
    GYRO_RANGE_250_DPS = 0, // ±250 도/s - 기본
    GYRO_RANGE_500_DPS = 1, // ±500 도/s
    GYRO_RANGE_1000_DPS = 2, // ±1000 도/s
    GYRO_RANGE_2000_DPS = 3  // ±2000 도/s
} gyro_range_t;

// 가속도계 측정 범위   ±2g (00), ±4g (01), ±8g (10), ±16g (11)
typedef enum
{
    ACCEL_RANGE_2G = 0, // ±2g - 기본
    ACCEL_RANGE_4G = 1, // ±4g
    ACCEL_RANGE_8G = 2, // ±8g
    ACCEL_RANGE_16G = 3  // ±16g
} accel_range_t;

// 자이로 디지털 저역 통과 필터 + 온도센서도 포함
typedef enum
{
    GYRO_DLPF_BW_250_HZ = 0, // 250 Hz
    GYRO_DLPF_BW_184_HZ = 1, // 184 Hz
    GYRO_DLPF_BW_92_HZ = 2, // 92 Hz
    GYRO_DLPF_BW_41_HZ = 3, // 41 Hz
    GYRO_DLPF_BW_20_HZ = 4, // 20 Hz
    GYRO_DLPF_BW_10_HZ = 5, // 10 Hz
    GYRO_DLPF_BW_5_HZ = 6  // 5 Hz
} gyro_dlpf_t;

// 가속도 디지털 저역 통과 필터
typedef enum
{
    ACCEL_DLPF_BW_460_HZ = 0, // 460 Hz
    ACCEL_DLPF_BW_184_HZ = 1, // 184 Hz
    ACCEL_DLPF_BW_92_HZ = 2, // 92 Hz
    ACCEL_DLPF_BW_41_HZ = 3, // 41 Hz
    ACCEL_DLPF_BW_20_HZ = 4, // 20 Hz
    ACCEL_DLPF_BW_10_HZ = 5, // 10 Hz
    ACCEL_DLPF_BW_5_HZ = 6  // 5 Hz
} accel_dlpf_t;


void Scanning_I2C();
HAL_StatusTypeDef Gy6500_init(GyData *dev, I2C_HandleTypeDef *i2c_handle, uint8_t address);
HAL_StatusTypeDef getAllData(GyData *dev);
HAL_StatusTypeDef getAllDataOffset(GyData *dev);
HAL_StatusTypeDef getAccelerometer(GyData *dev);
HAL_StatusTypeDef getGyroscope(GyData *dev);
HAL_StatusTypeDef getTemp(GyData *dev);
HAL_StatusTypeDef setSensorState(GyData *dev, uint8_t setData);
HAL_StatusTypeDef setTempState(GyData *dev, bool set_data);
HAL_StatusTypeDef setGyroRange(GyData *dev, gyro_range_t range);
HAL_StatusTypeDef setAccelRange(GyData *dev, accel_range_t range);
HAL_StatusTypeDef setAccelDLPF(GyData *dev, accel_dlpf_t mode);
HAL_StatusTypeDef setGyroDLPF(GyData *dev, gyro_dlpf_t mode);
HAL_StatusTypeDef resetAccelDLPF(GyData *dev);
HAL_StatusTypeDef resetGyroDLPF(GyData *dev);
void calibrateGyro(GyData *dev, uint16_t samples);
void calibrateAccel(GyData *dev, uint16_t samples);
void calculateAngles(GyData *dev, float dt);

#endif /* INC_GY6500_I2C_H_ */
