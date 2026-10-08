#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Raw sensor readings, exactly as the MPU-6050 reports them. */
typedef struct
{
    int16_t ax, ay, az; /* accelerometer: 16384 counts = 1 g  (range +/-2 g)       */
    int16_t temp;       /* temperature, raw                                        */
    int16_t gx, gy, gz; /* gyroscope: 65.5 counts = 1 deg/s   (range +/-500 deg/s) */
} mpu6050_raw_t;

/* Wakes the sensor and configures it. Stores the WHO_AM_I value in *who_am_i. */
HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);

/* Reads accelerometer, temperature and gyroscope in one 14-byte burst. */
HAL_StatusTypeDef mpu6050_read(I2C_HandleTypeDef *hi2c, mpu6050_raw_t *out);

#endif /* MPU6050_H */