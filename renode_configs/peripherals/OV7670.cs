// OmniVision OV7670 VGA camera - SCCB register model with a synthetic scene.
//
// Fidelity scope (deliberate):
//   Real: SCCB register interface (PID/VER, COM7/CLKRC/COM3/COM14/COM15/COM17/
//   TSLB plus windowing), QVGA YUV422 output layout, and a deterministic
//   synthetic scene (vertical ramp + walking row band + sweeping bar) whose
//   luma matches firmware/video_udp.c's synth_line() pixel-for-pixel, so the
//   two can be cross-checked.
//   Stand-in: none for pixel readout any more - the DVP parallel bus below is
//   the real transfer path. The fictional 0xF0-0xF3 test registers are NOT in
//   the OV7670 register map (datasheet Table 5 stops at 0xC9); they are kept
//   only so the older firmware path keeps working for A/B comparison and must
//   not be used for real hardware.
//   Simplified: COM7 bit decoding covers reset/format/resolution for the
//   standard init values; full datasheet bit semantics are future work.
//   No AEC/AGC/AWB, no exposure model.
//
// DVP output (PCLK/VSYNC/HREF/D0-D7) is driven by a ClockEntry so the pixel
// clock advances in simulated time. Data lines are updated before each PCLK
// rising edge, matching the datasheet tPDV/tSU budget (5 ns / 15 ns), so a
// receiver sampling on the rising edge sees a stable byte.
//
// Default state after reset: QVGA (320x240) YUV422, matching this project's
// streaming configuration.

using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class OV7670 : II2CPeripheral, IGPIOReceiver
    {
        // DVP receiver input indices, mirrored by Dcmi.cs.
        public const int GpioStreamEnable = 0;

        // PCLK must match the geometry. For YUV422 the bus carries 2 bytes per
        // pixel, so a 30 fps frame rate needs Width*Height*2*30 bytes/s:
        //   QQVGA 160x120 -> ~1.15 MHz      QVGA 320x240 -> ~4.6 MHz
        // Driving a much higher rate than the output geometry warrants only
        // makes Renode schedule edges that no real sensor would produce, and
        // each edge is a GPIO notification, so it burns wall-clock time and
        // starves the CPU. This default suits QQVGA; VGA builds want ~4.6 MHz.
        public const int DefaultPclkHz = 1_200_000;

        public OV7670(IMachine machine) : this(machine, DefaultPclkHz)
        {
        }

        public OV7670(IMachine machine, int pclkHz)
        {
            // ClockEntry takes a period in TimeInterval ticks plus the tick
            // rate; one tick is 1 ns, so an 8 MHz PCLK is a 125-tick period.
            // AddClockEntry is required - constructing a ClockEntry does not
            // schedule it.
            var period = TimeInterval.TicksPerSecond / (ulong)pclkHz;
            pclk = new ClockEntry(period, TimeInterval.TicksPerSecond, OnPclk,
                                  machine, "OV7670 PCLK");
            machine.ClockSource.AddClockEntry(pclk);
            PCLK = new GPIO();
            VSYNC = new GPIO();
            HREF = new GPIO();
            dataLines = new GPIO[8];
            for (var i = 0; i < 8; i++)
            {
                dataLines[i] = new GPIO();
            }
            Reset();
        }

        public GPIO PCLK { get; }
        public GPIO VSYNC { get; }
        public GPIO HREF { get; }
        public GPIO D0 { get { return dataLines[0]; } }
        public GPIO D1 { get { return dataLines[1]; } }
        public GPIO D2 { get { return dataLines[2]; } }
        public GPIO D3 { get { return dataLines[3]; } }
        public GPIO D4 { get { return dataLines[4]; } }
        public GPIO D5 { get { return dataLines[5]; } }
        public GPIO D6 { get { return dataLines[6]; } }
        public GPIO D7 { get { return dataLines[7]; } }
        public GPIO[] Data => dataLines;

        // The capture engine (Dcmi.cs) pulls StreamEnable high when it is armed,
        // so an un-captured sensor does not burn simulation time clocking a
        // bus nobody is listening to.
        public void OnGPIO(int number, bool value)
        {
            if (number == GpioStreamEnable)
            {
                streaming = value;
            }
        }

        public bool IsStreaming => streaming;

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
                // Rewind the DVP engine so a sensor reset restarts the frame.
                row = 0;
                col = 0;
                lineByte = 0;
                blankLines = 0;
                PCLK?.Set(false);
                HREF?.Set(false);
                VSYNC?.Set(false);
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

            // Downscaling of a pre-defined output mode (datasheet: COM3[3]
            // "Scale enable", COM3[2] "DCW enable", COM14[3] "Manual scaling
            // enable for pre-defined resolution modes such as CIF, QCIF, and
            // QVGA", COM14[2:0] the divider). DCW halves the width; the manual
            // divider halves the height per step. COM3=0x0C with COM14=0x09
            // therefore turns QVGA into QQVGA 160x120.
            if ((regs[REG_COM3] & 0x08) != 0)
            {
                if ((regs[REG_COM3] & 0x04) != 0)
                {
                    Width /= 2;
                }
                if ((regs[REG_COM14] & 0x08) != 0)
                {
                    var div = 1u << (regs[REG_COM14] & 0x07);
                    if (div > 0u)
                    {
                        Height /= (int)div;
                    }
                }
            }
            Logger.Log(LogLevel.Warning, "OV7670 geometry {0}x{1} yuv422={2} COM7={3:X2} COM3={4:X2} COM14={5:X2}",
                        Width, Height, IsYuv422, regs[REG_COM7], regs[REG_COM3], regs[REG_COM14]);
            if (Width < 16 || Height < 16)
            {
                Width = qvga ? 320 : 640;
                Height = qvga ? 240 : 480;
            }
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
            if (addr == REG_COM3 || addr == REG_COM14)
            {
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

        // ---- DVP frame engine ----

        // Driven at the PCLK rate; one call emits one byte period. VSYNC marks
        // the frame boundary, HREF marks active line data, and the byte stream
        // is YUYV: Y0 Cb Y1 Cr Y2 Cb Y3 Cr ... so Width*2 bytes per line.
        private void OnPclk()
        {
            if (!streaming)
            {
                return;
            }

            // Vertical blanking. A real sensor holds VSYNC across a real
            // blanking interval; holding it for a single pixel clock (as this
            // model used to) leaves the receiver almost no virtual time to
            // re-arm between the frame ending and the next one starting, and
            // Renode does not necessarily schedule the CPU in between. Emitting
            // nothing for a few blanking lines is both more faithful and gives
            // the frame interrupt room to be taken.
            if (blankLines > 0)
            {
                blankLines--;
                HREF.Set(false);
                if (blankLines == 0)
                {
                    // Blank over: the next frame starts here.
                    VSYNC.Set(false);
                    row = 0;
                    col = 0;
                    lineByte = 0;
                }
                return;
            }

            if (lineByte == 0)
            {
                HREF.Set(true);
            }

            // YUYV: Y0 Cb Y1 Cr ... Luma lands on even columns and the shared
            // chroma on odd ones. Chroma is a plausible mid-level so a receiver
            // that forgets to de-interleave still sees a varying stream rather
            // than a tell-tale constant.
            // YUYV is 2 bytes per PIXEL: Y0 Cb Y1 Cr Y2 Cb Y3 Cr ...
            // 'col' counts bytes across the line, so the pixel index is col/2
            // and even bytes are luma. Emitting one byte per pixel would halve
            // the frame size and desynchronise the sensor from the DMA slot
            // the firmware sizes from VIDEO_WIDTH*VIDEO_HEIGHT*2.
            var b = ((col & 1) == 0) ? Luma((uint)row, (uint)(col >> 1), frameSeq)
                                     : (byte)128;

            DriveData(b);
            PCLK.Set(true);
            PCLK.Set(false);

            col++;
            lineByte++;
            if (col >= BytesPerLine)
            {
                col = 0;
                lineByte = 0;
                HREF.Set(false);
                row++;
                if (row >= Height)
                {
                    frameSeq++;
                    VSYNC.Set(true);
                    blankLines = BlankLines;
                }
            }
        }

        // Vertical blanking lines between frames. Needs to be long enough in
        // virtual time for the receiver's frame interrupt to be serviced.
        private const int BlankLines = 8;

        // YUV422 and RGB565 both put two bytes on the bus per pixel.
        private int BytesPerLine => Width * 2;

        private void DriveData(byte value)
        {
            for (var i = 0; i < 8; i++)
            {
                dataLines[i].Set((value & (1 << i)) != 0);
            }
        }

        private readonly ClockEntry pclk;
        private readonly GPIO[] dataLines;
        private bool streaming;
        private int row;
        private int col;
        private int lineByte;
        private int blankLines;

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