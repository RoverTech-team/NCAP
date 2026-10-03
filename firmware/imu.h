// ST LSM9DS1 9-axis IMU over I2C1 (accel/gyro die at 0x6B, mag die at 0x1E).
//
// Only the bring-up subset is implemented: WHO_AM_I check, a fixed
// +/-2 g accel / +/-245 dps gyro / continuous mag configuration, and burst
// reads of the output registers. Raw values are converted to SI units on the
// way out so publishers can fill sensor_msgs directly.

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_AG_ADDR   0x6Bu
#define IMU_MAG_ADDR  0x1Eu

typedef struct
{
    float ax, ay, az;   // m/s^2
    float gx, gy, gz;   // rad/s
    float mx, my, mz;   // Tesla
    bool mag_valid;
} imu_sample_t;

bool imu_init(void);
bool imu_read(imu_sample_t *out);

#endif