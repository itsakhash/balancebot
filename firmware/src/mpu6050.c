#include "mpu6050.h"

#define MPU_ADDR (0x68 << 1) /* 7-bit address 0x68 (AD0 low), shifted for HAL */
#define I2C_TIMEOUT_MS 10

#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                              who_am_i, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Wake from sleep and use the gyro's clock, which is more stable than the internal one. */
    status = write_reg(hi2c, REG_PWR_MGMT_1, 0x01);
    if (status != HAL_OK)
        return status;
    HAL_Delay(100);

    /* Low-pass filter about 44 Hz: smooths vibration noise, adds a few ms of delay. */
    status = write_reg(hi2c, REG_CONFIG, 0x03);
    if (status != HAL_OK)
        return status;

    /* Gyro range +/-500 deg/s. */
    status = write_reg(hi2c, REG_GYRO_CONFIG, 0x08);
    if (status != HAL_OK)
        return status;

    /* Accelerometer range +/-2 g. */
    return write_reg(hi2c, REG_ACCEL_CONFIG, 0x00);
}

HAL_StatusTypeDef mpu6050_read(I2C_HandleTypeDef *hi2c, mpu6050_raw_t *out)
{
    uint8_t buf[14];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, REG_ACCEL_XOUT_H,
                                                I2C_MEMADD_SIZE_8BIT, buf, 14,
                                                I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Each value arrives as two bytes, high byte first. */
    out->ax = (int16_t)((buf[0] << 8) | buf[1]);
    out->ay = (int16_t)((buf[2] << 8) | buf[3]);
    out->az = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp = (int16_t)((buf[6] << 8) | buf[7]);
    out->gx = (int16_t)((buf[8] << 8) | buf[9]);
    out->gy = (int16_t)((buf[10] << 8) | buf[11]);
    out->gz = (int16_t)((buf[12] << 8) | buf[13]);
    return HAL_OK;
}