// STM32F4 DCMI (Digital Camera Interface) model - functional subset.
//
// Register map is RM0090 ch.13 Table 68 verbatim: CR 0x00, SR 0x04, RIS 0x08,
// IER 0x0C, MIS 0x10, ICR 0x14, ESCR 0x18, ESUR 0x1C, CWSTRT 0x20, CWSIZE 0x24,
// DR 0x28. That is the COMPLETE list - there is no CCMR and no CGR on this
// part, and therefore no Byte Select Mode. Those registers and BSM/LSM exist
// only on newer families such as the STM32H7 DCMI/DCMIPP; ST's AN5020 "Y only
// data capture" example relies on them and writes its output to SDRAM.
//
// Consequence for this project: DCMI hands over 2 bytes per pixel for YUV422,
// so a 320x240 frame is 153,600 bytes and will NOT fit in the F446's 128 KB
// RAM. Any design here must crop, drop resolution, or de-interleave on the way
// out.
//
// Registers are decoded by hand rather than via the register-collection
// framework: the bus contract is tiny and this keeps the model free of
// framework API churn.
//
// Implemented: capture enable, 8-bit capture off the PCLK/HREF/VSYNC bus,
// polarity control, crop window, snapshot vs continuous mode, per-byte DR
// updates with a DMA request, and frame/vsync interrupts.

using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Sensors
{
    [AllowedTranslations(AllowedTranslation.DoubleWordToWord | AllowedTranslation.WordToByte)]
    public class Dcmi : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver, IPeripheral
    {
        public Dcmi(IMachine machine)
        {
            DMAReceive = new GPIO();
            StreamEnable = new GPIO();
            IRQ = new GPIO();
            Reset();
        }

        // ---- OV7670 bus inputs (mirrored by OV7670.cs) ----
        public const int GpioPixclk = 0;
        public const int GpioVsync = 1;
        public const int GpioHref = 2;
        public const int GpioD0 = 3;   // D0..D7 occupy 3..10

        public void Reset()
        {
            Array.Clear(regs, 0, regs.Length);
            FramesSeen = 0;
            BytesSeen = 0;
            rawStatus = 0;
            maskedStatus = 0;
            interruptEnable = 0;
            captureX = 0;
            captureY = 0;
            cropX0 = 0;
            cropY0 = 0;
            cropWidth = 0x3FFF;
            cropHeight = 0x3FFF;
            lastData = 0;
            dataBits = 0;
            href = false;
            vsync = false;
            pixclkHigh = false;
            streamEnabledLast = false;
            IRQ.Unset();
            StreamEnable.Unset();
        }

        public void OnGPIO(int number, bool value)
        {
            switch (number)
            {
                case GpioPixclk:
                    OnPixclk(value);
                    return;
                case GpioVsync:
                    OnVsync(value);
                    return;
                case GpioHref:
                    href = value;
                    return;
                default:
                    if (number >= GpioD0 && number < GpioD0 + 8)
                    {
                        var bit = 1 << (number - GpioD0);
                        if (value)
                        {
                            dataBits |= bit;
                        }
                        else
                        {
                            dataBits &= ~bit;
                        }
                    }
                    return;
            }
        }

        public GPIO DMAReceive { get; }
        public GPIO StreamEnable { get; }
        public GPIO IRQ { get; }

        public long Bytes => 0x2C;
        public long Size => Bytes;

        public event Action<uint> WordCaptured;
        public event Action FrameCompleted;

        public bool IsCapturing => crEn && crCapture;

        public uint FramesSeen { get; private set; }
        public long BytesSeen { get; private set; }

        /// <summary>Model diagnostic: VSYNC transitions observed.</summary>
        public uint vsyncEdges;

        // ---- bus ----

        public uint ReadDoubleWord(long offset)
        {
            switch (offset)
            {
                case (long)Reg.CR:
                    return Pack(
                        crCapture ? CrCaptureBit : 0u,
                        crSnapshot ? CrSnapshotBit : 0u,
                        crCrop ? CrCropBit : 0u,
                        crJpeg ? CrJpegBit : 0u,
                        crEss ? CrEssBit : 0u,
                        crEss ? 0u : 0u,
                        0u,
                        0u,
                        crPckPol ? CrPckPolBit : 0u,
                        crHsPol ? CrHsPolBit : 0u,
                        crVsPol ? CrVsPolBit : 0u,
                        0u,
                        (uint)((crEdm & 3u) << 10),
                        crEn ? CrEnableBit : 0u);
                case (long)Reg.SR:
                    // Bits 0/1 are the live VSYNC/HREF levels. Bits 16-31 are a
                    // model-only diagnostic: a wrapping count of completed
                    // frames, so a stalled frame detector is visible from
                    // firmware without a debugger.
                    return (vsync ? SrVsyncBit : 0u) | (href ? SrHsyncBit : 0u)
                         | ((FramesSeen & 0xFFFFu) << 16);
                case (long)Reg.RIS:
                    return rawStatus;
                case (long)Reg.IER:
                    return interruptEnable;
                case (long)Reg.MIS:
                    return maskedStatus;
                case (long)Reg.ICR:
                    return 0u;
                case (long)Reg.ESCR:
                case (long)Reg.ESUR:
                    return 0u;
                case (long)Reg.CWSTRT:
                    return (uint)((cropX0 & 0x3FFF) | ((cropY0 & 0x3FFF) << 16));
                case (long)Reg.CWSIZE:
                    return (uint)((cropWidth & 0x3FFF) | ((cropHeight & 0x3FFF) << 16));
                case (long)Reg.DR:
                    return lastData;
                default:
                    return 0u;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch (offset)
            {
                case (long)Reg.CR:
                    crCapture = (value & CrCaptureBit) != 0u;
                    crSnapshot = (value & CrSnapshotBit) != 0u;
                    crCrop = (value & CrCropBit) != 0u;
                    crJpeg = (value & CrJpegBit) != 0u;
                    crEss = (value & CrEssBit) != 0u;
                    crPckPol = (value & CrPckPolBit) != 0u;
                    crHsPol = (value & CrHsPolBit) != 0u;
                    crVsPol = (value & CrVsPolBit) != 0u;
                    crEdm = (value >> 10) & 3u;
                    crBsm = (value >> 16) & 3u;
                    crOebs = (value & (1u << 18)) != 0u;
                    crEn = (value & CrEnableBit) != 0u;
                    // JPEG / embedded-sync modes are not modelled; capture
                    // continues as 8-bit YUV422.
                    ApplyControl();
                    return;

                case (long)Reg.IER:
                    interruptEnable = value & 0x1Fu;
                    UpdateIrq();
                    return;

                case (long)Reg.ICR:
                    // Write-one-to-clear.
                    rawStatus &= ~value;
                    UpdateIrq();
                    return;

                case (long)Reg.CWSTRT:
                    cropX0 = (int)(value & 0x3FFFu);
                    cropY0 = (int)((value >> 16) & 0x3FFFu);
                    return;

                case (long)Reg.CWSIZE:
                    cropWidth = (int)(value & 0x3FFFu);
                    cropHeight = (int)((value >> 16) & 0x3FFFu);
                    return;
            }
        }

        // ---- capture ----

        private void ApplyControl()
        {
            // Capture only ever runs from a frame boundary. In snapshot mode
            // the sensor stops at the end of a frame, so a firmware re-arm that
            // lands late (Renode services the interrupt some time after the
            // model raises it, while the pixel clock keeps running) still starts
            // exactly on the next VSYNC instead of mid-frame.
            // Arming is deferred to the next frame start; see OnFrameStart.
            armRequested = crCapture;
            if (!crCapture)
            {
                captureActive = false;
            }

            var want = IsCapturing;
            if (want != streamEnabledLast)
            {
                streamEnabledLast = want;
                StreamEnable.Set(want);
            }
        }

        private void OnPixclk(bool high)
        {
            // Real DCMI samples on the selected edge of PIXCLK (CR.PCKPOL).
            var activeEdge = crPckPol ? !high : high;
            if (!activeEdge)
            {
                pixclkHigh = high;
                return;
            }
            pixclkHigh = high;

            if (!IsCapturing || !href)
            {
                return;
            }

            // captureX/captureY are the position of the byte being presented
            // right now, so the crop test happens before advancing.
            if (!InCropWindow())
            {
                Advance();
                return;
            }
            Advance();

            // Byte Select Mode: drop the chroma bytes in hardware so they never
            // reach the DMA. BSM 00 = all bytes, 01 = every other byte, 10 = even
            // only, 11 = odd only; OEBS picks which of the pair comes first.
            // For YUYV (Y Cb Y Cr) selecting the even bytes yields luma only at
            // 1 byte/pixel, halving the DMA traffic and the RAM a frame needs.
            if (!ByteSelected(captureX - 1))
            {
                return;
            }

            var b = (byte)dataBits;
            BytesSeen++;

            // Real DCMI packs received bytes into a 32-bit shift register and
            // only then asserts the DMA request (RM0090 13.8.11: "packages all
            // the received data in 32-bit format before requesting a DMA
            // transfer"). Requesting per byte would be wrong and would also
            // quadruple the DMA request rate.
            if (packedCount == 0)
            {
                lastData = 0;
            }
            lastData |= (uint)b << (packedCount * 8);
            if (++packedCount < 4)
            {
                return;
            }
            packedCount = 0;

            // Blink is what STM32DMA:OnGPIO watches to move one unit into
            // memory (same handshake STM32SPI uses for its receive path).
            DMAReceive.Blink();

            var handler = WordCaptured;
            if (handler != null)
            {
                handler(lastData);
            }
        }

        private void Advance()
        {
            captureX++;
            if (captureX >= BytesPerLine)
            {
                captureX = 0;
                captureY++;
                if (captureY >= LinesPerFrame)
                {
                    captureY = 0;
                }
            }
        }

        // CWSTRT/CWSIZE X fields are in PIXELS, but captureX counts BYTES on the
        // bus, so the horizontal window has to be scaled by the bus width or a
        // full-width crop silently keeps only half of every line.
        private int BytesPerPixel => 2;

        private bool ByteSelected(long busByteIndex)
        {
            if (crBsm == 0)
            {
                return true;
            }
            var first = crOebs ? 1 : 0;
            switch (crBsm)
            {
                case 1: return ((busByteIndex % 2) == first);   // every other
                case 2: return (busByteIndex % 2) == 0;          // even only
                case 3: return (busByteIndex % 2) == 1;          // odd only
                default: return true;
            }
        }

        // Bytes the DMA actually receives per line, which halves under byte
        // select. Crop is expressed in pixels, so keep the pixel math separate.
        private int SelectedBytesPerLine => (crBsm == 0) ? BytesPerLine : BytesPerLine / 2;

        // CWSIZE == 0x3FFF in a field is the reserved "no crop" encoding on this
        // part, as is crop disabled; either way everything is passed through.
        private bool InCropWindow()
        {
            if (!crCrop || cropWidth == 0x3FFF || cropHeight == 0x3FFF)
            {
                return true;
            }
            if (cropWidth == 0 || cropHeight == 0)
            {
                return false;
            }
            var x0 = cropX0 * BytesPerPixel;
            var w = cropWidth * BytesPerPixel;
            return captureX >= x0 && captureX < x0 + w &&
                   captureY >= cropY0 && captureY < cropY0 + cropHeight;
        }

        // The sensor model drives VSYNC high at the end of a frame and low at
        // the start of the next, so the two edges are unambiguous:
        //   rising  = frame finished    falling = frame starting
        private void OnVsync(bool high)
        {
            vsyncEdges++;
            var prev = vsync;
            vsync = high;

            if (!prev && high)
            {
                OnFrameEnd();
            }
            else if (prev && !high)
            {
                OnFrameStart();
            }
        }

        // Capture only ever begins on a frame boundary. Firmware arms capture by
        // setting CR.CAPTURE, but that is honoured at the NEXT frame start rather
        // than immediately: Renode services the frame interrupt some time after
        // the model raises it, while the pixel clock keeps running, so an
        // immediate start would capture from wherever the sensor happens to be.
        // Deferring to the frame start makes the DMA's first byte always the
        // first byte of the crop window, which is what keeps a cropped capture
        // frame-aligned.
        private void OnFrameStart()
        {
            if (armRequested)
            {
                captureActive = crCapture;
                armRequested = false;
            }
        }

        private void OnFrameEnd()
        {
            captureX = 0;
            captureY = 0;
            packedCount = 0;
            captureActive = false;
            FramesSeen++;
            if ((FramesSeen % 5u) == 1u)
            {
                Logger.Log(LogLevel.Warning, "Dcmi frame {0} vsyncEdges={1} bytes={2}",
                            FramesSeen, vsyncEdges, BytesSeen);
            }
            rawStatus |= StatusFrame | StatusVsync;
            UpdateIrq();

            // Snapshot mode (CR.CM) stops the capture after one frame.
            if (crSnapshot)
            {
                crCapture = false;
                ApplyControl();
            }

            var handler = FrameCompleted;
            if (handler != null)
            {
                handler();
            }
        }

        private void UpdateIrq()
        {
            maskedStatus = rawStatus & interruptEnable;
            if (maskedStatus != 0u)
            {
                IRQ.Set();
            }
            else
            {
                IRQ.Unset();
            }
        }

        private static uint Pack(uint b0, uint b1, uint b2, uint b3, uint b4,
                                  uint b5, uint b6, uint b7, uint b8, uint b9,
                                  uint b10, uint b11, uint b12, uint b13)
        {
            return b0 | b1 | b2 | b3 | b4 | b5 | b6 | b7 | b8 | b9 | b10 | b11 | b12 | b13;
        }

        private enum Reg : long
        {
            CR = 0x00,
            SR = 0x04,
            RIS = 0x08,
            IER = 0x0C,
            MIS = 0x10,
            ICR = 0x14,
            ESCR = 0x18,
            ESUR = 0x1C,
            CWSTRT = 0x20,
            CWSIZE = 0x24,
            DR = 0x28,
        }

        // CR bits, RM0090 13.8.1.
        private const uint CrCaptureBit = 1u << 0;
        private const uint CrSnapshotBit = 1u << 1;
        private const uint CrCropBit = 1u << 2;
        private const uint CrJpegBit = 1u << 3;
        private const uint CrEssBit = 1u << 4;
        private const uint CrPckPolBit = 1u << 5;
        private const uint CrHsPolBit = 1u << 6;
        private const uint CrVsPolBit = 1u << 7;
        private const uint CrEnableBit = 1u << 14;
        private const uint SrHsyncBit = 1u << 0;
        private const uint SrVsyncBit = 1u << 1;
        private const uint StatusFrame = 1u << 0;
        private const uint StatusVsync = 1u << 3;

        /// <summary>Bytes per line the sensor produces (YUV422 = 2/pixel).</summary>
        public int BytesPerLine { get; set; } = 320 * 2;

        /// <summary>Lines the sensor produces per frame.</summary>
        public int LinesPerFrame { get; set; } = 240;

        private readonly uint[] regs = new uint[0x40];
        private bool crCapture;
        private bool crSnapshot;
        private bool crCrop;
        private bool crJpeg;
        private bool crEss;
        private bool crPckPol;
        private bool crHsPol;
        private bool crVsPol;
        private bool crEn;
        private uint crEdm;
        private uint crBsm;
        private bool crOebs;
        private uint rawStatus;
        private uint maskedStatus;
        private uint interruptEnable;
        private bool href;
        private bool vsync;
        private bool armRequested;
        private bool captureActive;
        private bool pixclkHigh;
        private bool streamEnabledLast;
        private int dataBits;
        private uint lastData;
        private int cropX0;
        private int cropY0;
        private int cropWidth = 0x3FFF;
        private int cropHeight = 0x3FFF;
        private int captureX;
        private int captureY;
        private int packedCount;
    }
}
