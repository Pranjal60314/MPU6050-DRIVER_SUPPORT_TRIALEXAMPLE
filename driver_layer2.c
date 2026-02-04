#include <stdint.h>
#include "FreeRTOS-master\Demo\CORTEX_A2F200_IAR_and_Keil\MicroSemi_Code\drivers\I2C\i2c.h"
#include "my_own_drivers/driver_Pranjal.h"
#include "my_own_drivers/define.h"

int mpu6050_who_am_I(uint8_t *who_am_i)
{
    return mpu6050_read_reg(MPU6050_RA_WHO_AM_I, who_am_i);
}

int mpu6050_device_reset()
{
    return mpu6050_write_bits(PWR_MGMT_1, 0x80/*10000000*/, 0x80); // PWR_MGMT_1 register, DEVICE_RESET bit
}

int mpu6050_sleep_enable()
{
    return mpu6050_write_bits(PWR_MGMT_1, 0x40/*01000000*/, 0x40); // PWR_MGMT_1 register, SLEEP bit
}

int mpu6050_sleep_disable()
{
    return mpu6050_write_bits(PWR_MGMT_1, 0x40/*01000000*/, 0x00); // PWR_MGMT_1 register, SLEEP bit
}

int mpu6050_set_clock_source(uint8_t clock_source)
{
    if (clock_source > 7)
        return -1; // Invalid clock source
    return mpu6050_write_bits(PWR_MGMT_1, 0x07/*00000111*/, clock_source); // PWR_MGMT_1 register, CLKSEL bits
}

int mpu6050_set_sample_rate(uint8_t sample_rate_div)
{
    return mpu6050_write_reg(SMPLRT_DIV, sample_rate_div); // SMPLRT_DIV register
}

int mpu6050_set_dlpf(uint8_t dlpf_cfg)
{
    return mpu6050_write_bits(CONFIG, 0x07/*00000111*/, dlpf_cfg); // CONFIG register, DLPF_CFG bits
}

int mpu6050_set_accel_config(uint8_t accel_config)
{
    return mpu6050_write_bits(ACCEL_CONFIG, 0x18/*00011000*/, accel_config); // ACCEL_CONFIG register, AFS_SEL bits
}

int mpu6050_set_gyro_config(uint8_t gyro_config)
{
    return mpu6050_write_bits(GYRO_CONFIG, 0x18/*00011000*/, gyro_config); // GYRO_CONFIG register, FS_SEL bits
}

int mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    int ret;

    ret = mpu6050_read_regs(ACCEL_XOUT_H, buf, 6);
    if (ret != 0)
        return ret;

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);

    return 0;
}


int mpu6050_read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t buf[6];
    int ret;

    ret = mpu6050_read_regs(GYRO_XOUT_H, buf, 6);
    if (ret != 0)
        return ret;

    *gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
    *gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
    *gyro_z = (int16_t)((buf[4] << 8) | buf[5]);

    return 0;
}

int mpu6050_read_temp(int16_t *temp)
{
    uint8_t buf[2];
    int ret;

    ret = mpu6050_read_regs(TEMP_OUT_H, buf, 2);
    if (ret != 0)
        return ret;

    *temp = (int16_t)((buf[0] << 8) | buf[1]);

    return 0;
}



int mpu6050_self_test_basic(void)
{
    uint8_t who_am_i;
    int ret;

    ret = mpu6050_read_reg(WHO_AM_I, &who_am_i);
    if (ret != 0)
        return ret;

    if (who_am_i != 0x68)
        return -1;

    return 0;
}
