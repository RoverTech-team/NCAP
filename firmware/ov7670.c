#include "ov7670.h"

#include <stdio.h>

#include "main.h"
#include "i2c1.h"

#define REG_PID            0x0Au
#define REG_VER            0x0Bu
#define REG_COM3           0x0Cu
#define REG_COM7           0x12u
#define REG_CLKRC          0x11u
#define REG_COM14          0x3Eu
#define REG_TSLB           0x3Au
#define REG_COM15          0x40u
#define REG_COM17          0x42u
#define REG_FRAME_ROW_H    0xF0u
#define REG_FRAME_ROW_L    0xF1u
#define REG_FRAME_ROW_CNT  0xF2u
#define REG_FRAME_DATA     0xF3u

#define PID_EXPECT         0x76u
#define VER_EXPECT         0x73u

// QVGA YUV422 under the model's simplified COM7 decoding
// (bit4 = QVGA, bits[1:0] = 00 = YUV422).
#define COM7_QVGA_YUV      0x10u

// Output scaling, per the OV7670 datasheet:
//   COM3[3] Scale enable, COM3[2] DCW enable (horizontal /2),
//   COM14[3] Manual scaling enable, COM14[2:0] vertical divider.
// QVGA 320x240 with COM3=0x0C, COM14=0x09 becomes QQVGA 160x120, which is what
// makes a full YUV422 frame (2 bytes/pixel) fit the F446's 128 KB of RAM.
#define COM3_SCALE_DCW     0x0Cu
#define COM14_QQVGA        0x09u

// Ask the sensor for the geometry the rest of the firmware was built for.
#ifndef OV7670_OUT_WIDTH
#define OV7670_OUT_WIDTH   VIDEO_WIDTH
#endif
#ifndef OV7670_OUT_HEIGHT
#define OV7670_OUT_HEIGHT  VIDEO_HEIGHT
#endif

bool ov7670_init(void)
{
    uint8_t pid = 0, ver = 0;
    if (!i2c1_read_regs(OV7670_ADDR, REG_PID, &pid, 1u) || pid != PID_EXPECT)
    {
        printf("F4: ov7670 PID=%02X (want 76)\r\n", pid);
        return false;
    }
    if (!i2c1_read_regs(OV7670_ADDR, REG_VER, &ver, 1u) || ver != VER_EXPECT)
    {
        printf("F4: ov7670 VER=%02X (want 73)\r\n", ver);
        return false;
    }
    printf("F4: ov7670 PID/VER ok\r\n");

    // Reset, then fixed QVGA/YUV422 configuration.
    if (!i2c1_write_reg(OV7670_ADDR, REG_COM7, 0x80u))
    {
        printf("F4: ov7670 reset failed\r\n");
        return false;
    }
    HAL_Delay(5);
    if (!i2c1_write_reg(OV7670_ADDR, REG_COM7, COM7_QVGA_YUV))
    {
        printf("F4: ov7670 COM7 write failed\r\n");
        return false;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_CLKRC, 0x01u))
    {
        return false;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_COM15, 0x00u))
    {
        return false;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_COM17, 0x00u))
    {
        return false;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_TSLB, 0x04u))
    {
        return false;
    }

    // Downscale to the requested output geometry when it is smaller than
    // QVGA. DCW halves the width, COM14's manual divider halves the height.
    uint8_t com3 = 0x00u;
    uint8_t com14 = 0x00u;
    if ((uint32_t)OV7670_OUT_WIDTH <= 160u)
    {
        com3 = COM3_SCALE_DCW;
    }
    if ((uint32_t)OV7670_OUT_HEIGHT <= 120u)
    {
        com14 = COM14_QQVGA;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_COM3, com3) ||
        !i2c1_write_reg(OV7670_ADDR, REG_COM14, com14))
    {
        printf("F4: ov7670 scaling write failed\r\n");
        return false;
    }

    uint8_t back = 0;
    if (!i2c1_read_regs(OV7670_ADDR, REG_COM7, &back, 1u) || back != COM7_QVGA_YUV)
    {
        printf("F4: ov7670 COM7 readback=%02X\r\n", back);
        return false;
    }
    printf("F4: ov7670 QVGA YUV422 configured, out %ux%u (COM3=%02X COM14=%02X)\r\n",
           (unsigned)OV7670_OUT_WIDTH, (unsigned)OV7670_OUT_HEIGHT,
           (unsigned)com3, (unsigned)com14);
    return true;
}

bool ov7670_read_luma_rows(uint16_t start_row, uint8_t count,
                            uint8_t *buf, uint16_t width)
{
    if (buf == NULL || count == 0u)
    {
        return false;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_FRAME_ROW_H, (uint8_t)(start_row >> 8)))
    {
        return false;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_FRAME_ROW_L, (uint8_t)(start_row & 0xFFu)))
    {
        return false;
    }
    if (!i2c1_write_reg(OV7670_ADDR, REG_FRAME_ROW_CNT, count))
    {
        return false;
    }
    uint16_t total = (uint16_t)count * width;
    uint16_t done = 0;
    while (done < total)
    {
        uint16_t chunk = total - done;
        if (chunk > 255u)
        {
            chunk = 255u;
        }
        if (!i2c1_read_regs(OV7670_ADDR, REG_FRAME_DATA, &buf[done], (uint8_t)chunk))
        {
            return false;
        }
        done += chunk;
    }
    return true;
}