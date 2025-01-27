#ifndef MPU6050_H
#define MPU6050_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>

// Configure o MPU6050 aqui
#define MPU6050_ACEL_CONFIG_REG MPU6050_RANGE_8_G
#define MPU6050_GYRO_CONFIG_REG MPU6050_RANGE_500_DEG
#define MPU6050_BAND_CONFIG_REG MPU6050_BAND_21_HZ

struct MPU6050Values {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;
};

void mpu6050_setup();
MPU6050Values get_mpu6050_value();

#endif // MPU6050_H