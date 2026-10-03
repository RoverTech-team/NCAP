// OmniVision OV7670 VGA camera - SCCB register model with a synthetic scene.
//
// Fidelity scope (deliberate):
//   Real: SCCB register interface (PID/VER, COM7/CLKRC/COM3/COM14/COM15/COM17/
//   TSLB plus windowing), QVGA YUV422 output layout, and a deterministic
//   synthetic scene (vertical ramp + walking row band + sweeping bar) whose
//   luma matches firmware/video_udp.c's synth_line() pixel-for-pixel, so the
//   two can be cross-checked.
//   Stand-in: pixel readout. Real silicon streams pixels over a parallel bus
//   (PCLK/VSYNC/HREF/D0-D7). Until a parallel-capture path exists in firmware,
//   requested luma rows are fetched through test registers 0xF0-0xF3 over
//   SCCB. The bytes are identical to what the bus would carry (Y of YUYV);
//   only the transfer mechanism differs.
//   Simplified: COM7 bit decoding covers reset/format/resolution for the
//   standard init values; full datasheet bit semantics are future work.
//   No AEC/AGC/AWB, no exposure model.
//
// Default state after reset: QVGA (320x240) YUV422, matching this project's
// streaming configuration.

using System;

using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class OV7670 : II2CPeripheral
    {
        public OV7670()
        {
            Reset();
        }

        public void Reset()
        {
            lock (sync)
            {
                regs[REG_COM7] = 0x10;   // QVGA YUV
                regs[REG_CLKRC] = 0x01;
                regs[REG_COM3] = 0x00;
                regs[REG_COM14] = 0x00;
                regs[REG_COM15] = 0x00;
                regs[REG_COM17] = 0x00;
                regs[REG_TSLB] = 0x04;   // YUYV order
                ApplyConfig();
                frameSeq = 0;
                reqRow = 0;
                reqCount = 0;
                readCursor = 0;
                regAddr = 0;
                expectAddr = true;
            }
        }

        public void FinishTransmission()
        {
            lock (sync)
            {
                expectAddr = true;
            }
        }

        public void Write(byte[] data)
        {
            lock (sync)
            {
                foreach (var b in data)
                {
                    WriteByteLocked(b);
                }
            }
        }

        private void WriteByteLocked(byte b)
        {
            // Called with sync already held.
            if (expectAddr)
                {
                    regAddr = b;
                    expectAddr = false;
                    return;
                }
                WriteRegister(regAddr, b);
                // SCCB auto-increments for sequential access, except the data
                // FIFO which advances its own cursor on read instead.
                if (regAddr != REG_FRAME_DATA)
                {
                    regAddr++;
                }
        }

        public byte[] Read(int count = 1)
        {
            lock (sync)
            {
                var outBytes = new byte[count];
                for (var i = 0; i < count; i++)
                {
                    outBytes[i] = ReadRegister(regAddr);
                    if (regAddr == REG_FRAME_DATA)
                    {
                        readCursor++;
                    }
                    else
                    {
                        regAddr++;
                    }
                }
                return outBytes;
            }
        }

        // ---- configuration derived from registers ----

        public int Width { get; private set; } = 320;
        public int Height { get; private set; } = 240;
        public bool IsYuv422 { get; private set; } = true;

        private void ApplyConfig()
        {
            byte com7 = regs[REG_COM7];
            // Bit 4 selects QVGA; otherwise VGA. Bits[1:0] select the output
            // format (00 = YUV422, 10 = RGB565). This covers the standard init
            // values; exotic combinations fall back to QVGA YUV422.
            bool qvga = (com7 & 0x10) != 0;
            var fmt = com7 & 0x03;
            if (qvga)
            {
                Width = 320;
                Height = 240;
            }
            else
            {
                Width = 640;
                Height = 480;
            }
            IsYuv422 = fmt != 0x02;
        }

        private void WriteRegister(byte addr, byte value)
        {
            // PID/VER are read-only identification registers.
            if (addr == REG_PID || addr == REG_VER)
            {
                return;
            }
            regs[addr] = value;
            if (addr == REG_COM7)
            {
                if ((value & 0x80) != 0)
                {
                    // SCCB reset: restore defaults.
                    Reset();
                    return;
                }
                ApplyConfig();
            }
            if (addr == REG_FRAME_ROW_H || addr == REG_FRAME_ROW_L || addr == REG_FRAME_ROW_COUNT)
            {
                reqRow = (ushort)((regs[REG_FRAME_ROW_H] << 8) | regs[REG_FRAME_ROW_L]);
                reqCount = regs[REG_FRAME_ROW_COUNT];
                readCursor = 0;
            }
        }

        private byte ReadRegister(byte addr)
        {
            if (addr == REG_PID)
            {
                return 0x76;
            }
            if (addr == REG_VER)
            {
                return 0x73;
            }
            if (addr == REG_FRAME_DATA)
            {
                return LumaAtCursor();
            }
            return regs[addr];
        }

        private byte LumaAtCursor()
        {
            uint row = (uint)(reqRow + readCursor / Width);
            uint col = (uint)(readCursor % Width);
            if (row >= (uint)Height)
            {
                return 0;
            }
            // A new frame starts whenever row 0 is (re)requested.
            if (reqRow == 0 && readCursor == 0)
            {
                frameSeq++;
            }
            return Luma(row, col, frameSeq);
        }

        private byte Luma(uint row, uint col, uint seq)
        {
            // Identical to firmware synth_line() so model output and firmware
            // output can be diffed directly.
            uint v = (row * 255u) / (uint)Height;
            uint bandY = (seq * 5u) % (uint)Height;
            if (row >= bandY && row < bandY + 12u)
            {
                v = (v + 90u) & 0xFFu;
            }
            uint barX = (seq * 8u) % (uint)Width;
            uint dx = col > barX ? col - barX : barX - col;
            if (dx < 10u)
            {
                v = 255u;
            }
            return (byte)v;
        }

        private const byte REG_PID = 0x0A;
        private const byte REG_VER = 0x0B;
        private const byte REG_COM3 = 0x0C;
        private const byte REG_COM7 = 0x12;
        private const byte REG_CLKRC = 0x11;
        private const byte REG_TSLB = 0x3A;
        private const byte REG_COM14 = 0x3E;
        private const byte REG_COM15 = 0x40;
        private const byte REG_COM17 = 0x42;
        private const byte REG_FRAME_ROW_H = 0xF0;
        private const byte REG_FRAME_ROW_L = 0xF1;
        private const byte REG_FRAME_ROW_COUNT = 0xF2;
        private const byte REG_FRAME_DATA = 0xF3;

        private readonly object sync = new object();
        private readonly byte[] regs = new byte[256];
        private byte regAddr;
        private bool expectAddr;
        private ushort reqRow;
        private byte reqCount;
        private int readCursor;
        private uint frameSeq;
    }
}