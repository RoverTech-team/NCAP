#include "imu.h"

#include <stdio.h>

#include "main.h"
#include "i2c1.h"

// Accel/gyro die.
#define REG_WHO_AM_I     0x0Fu
#define REG_CTRL1_G      0x10u
#define REG_OUT_X_L_G    0x18u
#define REG_CTRL5_XL     0x1Fu
#define REG_CTRL6_XL     0x20u
#define REG_CTRL8        0x22u
#define REG_OUT_X_L_XL   0x28u
#define WHO_AM_I_AG      0x68u

// Magnetometer die.
#define REG_WHO_AM_I_M   0x0Fu
#define REG_CTRL1_M      0x20u
#define REG_CTRL2_M      0x21u
#define REG_CTRL3_M      0x22u
#define REG_OUT_X_L_M    0x28u
#define WHO_AM_I_M       0x3Du

#define AUTO_INC         0x80u

// +/-2 g accel: 0.061 mg/LSB. +/-245 dps gyro: 8.75 mdps/LSB.
// +/-4 gauss mag: 0.14 mgauss/LSB.
#define ACCEL_LSB_TO_MS2  (0.061f * 9.80665f / 1000.0f)
#define GYRO_LSB_TO_RADS  (0.00875f * 3.14159265f / 180.0f)
#define MAG_LSB_TO_T      (0.14e-3f * 1e-4f)

static int16_t to_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool imu_init(void)
{
    // Under multi-machine load the simulated I2C bus answers slowly; retry a
    // few times before declaring the sensor missing.
    uint8_t who = 0;
    bool present = false;
    for (int attempt = 0; attempt < 4 && !present; attempt++)
    {
        if (attempt > 0)
        {
            HAL_Delay(50);
        }
        present = i2c1_read_regs(IMU_AG_ADDR, REG_WHO_AM_I, &who, 1u) && who == WHO_AM_I_AG;
    }
    if (!present)
    {
        printf("F4: imu WHO_AM_I=%02X (want 68)\r\n", who);
        return false;
    }
    printf("F4: imu accel/gyro present\r\n");

    // Gyro: 119 Hz, +/-245 dps.
    if (!i2c1_write_reg(IMU_AG_ADDR, REG_CTRL1_G, 0x60u))
    {
        printf("F4: imu CTRL1_G write failed\r\n");
        return false;
    }
    // Accel XYZ enable.
    if (!i2c1_write_reg(IMU_AG_ADDR, REG_CTRL5_XL, 0x38u))
    {
        printf("F4: imu CTRL5_XL write failed\r\n");
        return false;
    }
    // Accel: 50 Hz, +/-2 g.
    if (!i2c1_write_reg(IMU_AG_ADDR, REG_CTRL6_XL, 0x60u))
    {
        printf("F4: imu CTRL6_XL write failed\r\n");
        return false;
    }
    // Keep register auto-increment on (default, but make it explicit).
    if (!i2c1_write_reg(IMU_AG_ADDR, REG_CTRL8, 0x04u))
    {
        printf("F4: imu CTRL8 write failed\r\n");
        return false;
    }

    if (!i2c1_read_regs(IMU_MAG_ADDR, REG_WHO_AM_I_M, &who, 1u) || who != WHO_AM_I_M)
    {
        printf("F4: mag WHO_AM_I=%02X (want 3D) - continuing without mag\r\n", who);
        return true;
    }
    printf("F4: imu magnetometer present\r\n");

    // Mag: ultra-high performance, 10 Hz, +/-4 gauss, continuous.
    if (!i2c1_write_reg(IMU_MAG_ADDR, REG_CTRL1_M, 0x30u))
    {
        return false;
    }
    if (!i2c1_write_reg(IMU_MAG_ADDR, REG_CTRL2_M, 0x00u))
    {
        return false;
    }
    return i2c1_write_reg(IMU_MAG_ADDR, REG_CTRL3_M, 0x00u);
}

// Renode's STM32F1_I2C aborts on multi-byte master reads (unhandled
// register-read path), so each output byte is fetched with its own
// single-byte transaction. Slower, but every transaction here is a sequence
// already proven to work during WHO_AM_I bring-up.
static bool read_block(uint8_t dev, uint8_t base, uint8_t *dst, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        if (!i2c1_read_regs(dev, (uint8_t)(base + i), &dst[i], 1u))
        {
            return false;
        }
    }
    return true;
}

bool imu_read(imu_sample_t *out)
{
    uint8_t b[6];

    if (!read_block(IMU_AG_ADDR, REG_OUT_X_L_G, b, 6u))
    {
        return false;
    }
    int16_t gx = to_i16(&b[0]);
    int16_t gy = to_i16(&b[2]);
    int16_t gz = to_i16(&b[4]);

    if (!read_block(IMU_AG_ADDR, REG_OUT_X_L_XL, b, 6u))
    {
        return false;
    }
    int16_t ax = to_i16(&b[0]);
    int16_t ay = to_i16(&b[2]);
    int16_t az = to_i16(&b[4]);

    out->gx = (float)gx * GYRO_LSB_TO_RADS;
    out->gy = (float)gy * GYRO_LSB_TO_RADS;
    out->gz = (float)gz * GYRO_LSB_TO_RADS;
    out->ax = (float)ax * ACCEL_LSB_TO_MS2;
    out->ay = (float)ay * ACCEL_LSB_TO_MS2;
    out->az = (float)az * ACCEL_LSB_TO_MS2;

    out->mag_valid = false;
    if (read_block(IMU_MAG_ADDR, REG_OUT_X_L_M, b, 6u))
    {
        out->mx = (float)to_i16(&b[0]) * MAG_LSB_TO_T;
        out->my = (float)to_i16(&b[2]) * MAG_LSB_TO_T;
        out->mz = (float)to_i16(&b[4]) * MAG_LSB_TO_T;
        out->mag_valid = true;
    }
    return true;
}