/*
 * mpu6050.h
 *
 *  Created on: Oct 24, 2023
 *      Author: meric
 */

#ifndef HWDRIVERS_MPU6050_H_
#define HWDRIVERS_MPU6050_H_



#endif /* HWDRIVERS_MPU6050_H_ */
// MPU6050 structure
#include <stdint.h>
#include "stm32f4xx_hal.h"
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    //double KalmanAngleX;
    //double KalmanAngleY;
} MPU6050_t;

// Kalman structure
/*
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;*/
extern I2C_HandleTypeDef hi2c1;
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

//double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
