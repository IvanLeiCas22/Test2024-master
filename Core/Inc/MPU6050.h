#include "stm32f1xx_hal.h"

void MPU6050_init(void); //Initialize the MPU 
void MPU6050_Read_Accel (int16_t *Ax, int16_t *Ay, int16_t *Az); //Read MPU Accelerator
void MPU6050_Read_Gyro (int16_t *Gx, int16_t *Gy, int16_t *Gz); //Read MPU Gyroscope
