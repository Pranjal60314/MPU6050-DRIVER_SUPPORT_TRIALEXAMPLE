#define MPU6050_I2C_ADDR            0x68   //assuming address pin is low

#include "define.h"


#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS-master\Demo\CORTEX_A2F200_IAR_and_Keil\MicroSemi_Code\drivers\I2C\i2c.h"



int mpu6050_write_reg(uint8_t reg , uint8_t value)
{
    uint8_t tx_buf[2];

    tx_buf[0] = reg;
    tx_buf[1] = value;

    /*void MSS_I2C_write
(
	mss_i2c_instance_t * this_i2c,: g_mss_i2c0 and g_mss_i2c1
	uint8_t serial_addr,          ;serial address of the target I2C device
	const uint8_t * write_buffer,:buffer holding the data to be written to the target I2C device
	uint16_t write_size,             ;Number of bytes held in the write_buffer to be written to the target I2C device
    uint8_t options                ;indicate if the I2C bus should be released on completion of the write transaction
);*/
    return MSS_I2C_write(&g_mss_i2c0, MPU6050_I2C_ADDR, tx_buf, 2, 0);
}

int mpu6050_read_reg(uint8_t reg, uint8_t *value)
{
    int ret;

    // Write register address so that the sensor knows which register to read
    ret = MSS_I2C_write(&g_mss_i2c0, MPU6050_I2C_ADDR, &reg, 1, 0);
    if (ret != 0)
        return ret;

    // Read data and return it
    return MSS_I2C_read(&g_mss_i2c0, MPU6050_I2C_ADDR, value, 1);
}

int mpu6050_write_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t tmp;
    int ret;

    /* Read current register value */
    ret = mpu6050_read_reg(reg, &tmp);
    if (ret != 0)
        return ret;

    /* Clear bits defined by mask */
    tmp &= ~mask;

    /* Set new bits (masked) */
    tmp |= (value & mask);

    /* Write back */
    return mpu6050_write_reg(reg, tmp);
}

int mpu6050_init(void)
{
    int ret;    
    ret = mpu6050_write_bits(PWR_MGMT_1, 0x20/*0010 0000*/, 0x00); // Clear sleep bit
    if (ret != 0)
        return ret;

    //Set clock source to PLL with X axis gyroscope reference
    ret = mpu6050_write_bits(PWR_MGMT_1, 0x07/*0000 0111*/, 0x00); // CLKSEL = 0 8Mhz internal oscillator
    if (ret != 0)
        return ret;

    ret = mpu6050_write_reg(SMPLRT_DIV, 0x07); // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) set to 1kHz
    if (ret != 0)
        return ret;

    // Set accelerometer configuration to +/- 2g
    ret = mpu6050_write_bits(ACCEL_CONFIG, 0x18/*00011000*/, 0x00); // AFS_SEL = 0 (+/- 2g)
    if (ret != 0)
        return ret;

    // Set gyroscope configuration to +/- 250 degrees/sec
    ret = mpu6050_write_bits(GYRO_CONFIG, 0x18/*00011000*/, 0x00); // FS_SEL = 0 (+/- 250 deg/sec)
    if (ret != 0)
        return ret;

    return 0;
}

int mpu6050_set_field(uint8_t reg, uint8_t start_bit, uint8_t length, uint8_t value)
{
    uint8_t mask;
    if (length == 0 || length > 8 || start_bit > 7)
        return -1; // Invalid parameters
    mask = ((1 << length) - 1) << (start_bit - length + 1);
    value <<= (start_bit - length + 1);
    return mpu6050_write_bits(reg, mask, value);
}
