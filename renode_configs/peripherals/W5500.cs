// W5500 hardwired TCP/IP SPI Ethernet controller model for Renode.
// Implements Antmicro.Renode.Peripherals.SPI.ISPIPeripheral.
// SPI frame (datasheet ch.2): Offset[15:8], Offset[7:0], Control[BSB4:0|RWB|OM1:0], Data...
//   BSB 0x00      : common registers (0x0000-0x0039)
//   BSB 1+4*N     : socket N registers (N=0..7)
//   BSB 2+4*N     : socket N TX buffer (16KB)
//   BSB 3+4*N     : socket N RX buffer (16KB)
//   RWB: 0=read, 1=write. OM: 00=VDM (variable, CS-framed), 01/10/11=FDM 1/2/4 bytes.
//
// Load in Renode with:  include @renode_configs/peripherals/W5500.cs
// Attach in .repl with: w5500: Network.W5500 @ spi1  (+  w5500 -> gpioPortB@1 for INTn, active low)
//
// Two operating modes:
//   LoopbackMode=true (default): SEND copies TX buffer into own RX buffer and raises RECV.
//     Lets unmodified W5500 firmware (VERSIONR/SIR/Sn_SR/Sn_TX_FSR/ping-pong) pass with no host network.
//   EnableHostSockets=true: TCP/UDP sockets are bridged to real host .NET sockets (TCP client/server, UDP).
//     Loopback still applies when no peer is connected, so tests stay deterministic.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Network;
using Antmicro.Renode.Peripherals.Network;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Network
{
    // Input 0: SCSn (active low) - delimits Variable-Length Data mode frames.
    // Input 1: RSTn (active low) - hardware reset.
    [GPIO(NumberOfInputs = 2)]
    public class W5500 : ISPIPeripheral, IGPIOReceiver, IMACInterface
    {
        public W5500()
        {
            IRQ = new GPIO();
            IRQ.Set(true); // INTn idle high (active low)
            Reset();
        }

        public void OnGPIO(int number, bool value)
        {
            lock (sync)
            {
                switch (number)
                {
                    case 0: // SCSn, active low
                        if (csActive == value)
                        {
                            if (!value)
                            {
                                // CSn asserted: start a new SPI frame.
                                spiPhase = SpiPhase.OffsetHigh;
                                spiFixedRemaining = 0;
                            }
                            csActive = value;
                        }
                        break;
                    case 1: // RSTn, active low
                        if (!value)
                        {
                            Reset();
                        }
                        break;
                }
            }
        }

        // Settable from .repl, e.g.  LoopbackMode: true
        public bool LoopbackMode { get; set; } = true;
        public bool EnableHostSockets { get; set; } = false;
        public bool Verbose { get; set; } = false;

        // When true, UDP socket traffic is emitted as real Ethernet frames
        // (IMACInterface) instead of being handed to host sockets. Attach the
        // model to a Renode EthernetSwitch (+ TAP) and the agent sees a stable,
        // real client IP/MAC - the same topology as the production HIL setup.
        public bool UseEthernet { get; set; } = false;

        public GPIO IRQ { get; private set; }

        public MACAddress MAC
        {
            get { return mac; }
            set { mac = value; }
        }

        public event Action<EthernetFrame> FrameReady;

        public void Reset()
        {
            lock (sync)
            {
                spiPhase = SpiPhase.OffsetHigh;
                spiOffset = 0;
                spiBsb = 0;
                spiIsWrite = false;
                spiFixedRemaining = 0;

                common = new byte[CommonSize];
                common[(int)CommonOff.RTR0] = 0x07; common[(int)CommonOff.RTR1] = 0xD0;
                common[(int)CommonOff.RCR] = 0x08;
                common[(int)CommonOff.PHYCFGR] = 0xB8; // LINK on, 100M, full duplex
                common[(int)CommonOff.VERSIONR] = 0x04;

                sockets = new SocketState[SocketCount];
                for (var i = 0; i < SocketCount; i++)
                {
                    sockets[i] = new SocketState();
                }
                RefreshInterrupt();
            }
        }

        public void FinishTransmission()
        {
            lock (sync)
            {
                spiPhase = SpiPhase.OffsetHigh;
                spiFixedRemaining = 0;
                csActive = false;
            }
        }

        public byte Transmit(byte data)
        {
            lock (sync)
            {
                switch (spiPhase)
                {
                    case SpiPhase.OffsetHigh:
                        spiOffset = (ushort)(data << 8);
                        spiPhase = SpiPhase.OffsetLow;
                        return 0;
                    case SpiPhase.OffsetLow:
                        spiOffset |= data;
                        spiPhase = SpiPhase.Control;
                        return 0;
                    case SpiPhase.Control:
                        spiBsb = (byte)((data >> 3) & 0x1F);
                        spiIsWrite = ((data >> 2) & 0x1) == 1;
                        var om = data & 0x03;
                        spiFixedRemaining = om == 0 ? -1 : (om == 1 ? 1 : (om == 2 ? 2 : 4));
                        spiPhase = SpiPhase.Data;
                        if (Verbose)
                        {
                            this.Log(LogLevel.Debug, "W5500 header off=0x{0:X4} bsb=0x{1:X2} {2} om={3}",
                                spiOffset, spiBsb, spiIsWrite ? "W" : "R", om);
                        }
                        return 0;
                    case SpiPhase.Data:
                    default:
                        byte ret;
                        if (spiIsWrite)
                        {
                            WriteByte(spiBsb, spiOffset, data);
                            ret = 0;
                        }
                        else
                        {
                            ret = ReadByte(spiBsb, spiOffset);
                        }
                        // Auto-increment offset in VDM; in FDM the master should deassert CS after N bytes.
                        spiOffset++;
                        if (spiFixedRemaining > 0)
                        {
                            spiFixedRemaining--;
                            if (spiFixedRemaining == 0)
                            {
                                // Stay in Data but further bytes are out-of-frame; reset to header on next CS.
                                // Keep phase so streaming masters don't desync, but count is exhausted.
                            }
                        }
                        return ret;
                }
            }
        }

        // ---- address decode ----

        private byte ReadByte(byte bsb, ushort offset)
        {
            if (bsb == 0x00)
            {
                return ReadCommon(offset);
            }
            int sock = BsbToSocket(bsb);
            int kind = BsbToKind(bsb); // 0=reg,1=tx,2=rx (relative to 1+4N base)
            if (sock < 0 || sock >= SocketCount)
            {
                this.Log(LogLevel.Warning, "W5500 read from invalid BSB 0x{0:X2} off 0x{1:X4}", bsb, offset);
                return 0;
            }
            if (kind == 0)
            {
                return ReadSocketReg(sock, offset);
            }
            if (kind == 1)
            {
                var s = sockets[sock];
                int eff = EffectiveTxSize(sock);
                return s.TxBuf[offset % eff];
            }
            int reff = EffectiveRxSize(sock);
            return sockets[sock].RxBuf[offset % reff];
        }

        private void WriteByte(byte bsb, ushort offset, byte value)
        {
            if (bsb == 0x00)
            {
                WriteCommon(offset, value);
                return;
            }
            int sock = BsbToSocket(bsb);
            int kind = BsbToKind(bsb);
            if (sock < 0 || sock >= SocketCount)
            {
                this.Log(LogLevel.Warning, "W5500 write to invalid BSB 0x{0:X2} off 0x{1:X4}", bsb, offset);
                return;
            }
            if (kind == 0)
            {
                WriteSocketReg(sock, offset, value);
                return;
            }
            if (kind == 1)
            {
                var s = sockets[sock];
                int eff = EffectiveTxSize(sock);
                s.TxBuf[offset % eff] = value;
                if (Verbose)
                {
                    s.TxWriteCount++;
                    if (s.TxWriteCount <= 16 || (s.TxWriteCount % 512) == 0)
                    {
                        this.Log(LogLevel.Debug, "W5500 s{0} TXBUF[{1}]={2:X2} (write #{3})",
                            sock, offset % eff, value, s.TxWriteCount);
                    }
                }
                return;
            }
            // RX buffer is read-only from SPI master; ignore writes.
            this.Log(LogLevel.Warning, "W5500 write to socket{0} RX buffer ignored", sock);
        }

        private static int BsbToSocket(byte bsb)
        {
            if (bsb == 0) return -1;
            return (bsb - 1) / 4;
        }

        private static int BsbToKind(byte bsb)
        {
            return (bsb - 1) % 4 == 0 ? 0 : ((bsb - 1) % 4 == 1 ? 1 : 2);
            // BSB layout per socket N: reg=1+4N, tx=2+4N, rx=3+4N. (bsb-1)%4: 0=reg,1=tx,2=rx
        }

        // ---- common registers ----

        private enum CommonOff
        {
            MR = 0x0000,
            GAR0 = 0x0001,
            SUBR0 = 0x0005,
            SHAR0 = 0x0009,
            SIPR0 = 0x000F,
            INTLEVEL0 = 0x0013,
            IR = 0x0015,
            IMR = 0x0016,
            SIR = 0x0017,
            SIMR = 0x0018,
            RTR0 = 0x0019,
            RTR1 = 0x001A,
            RCR = 0x001B,
            PHYCFGR = 0x002E,
            VERSIONR = 0x0039,
        }
        private const int CommonSize = 0x40;

        private byte ReadCommon(ushort offset)
        {
            if (offset == (int)CommonOff.SIR)
            {
                return ComputeSIR();
            }
            if (offset == (int)CommonOff.VERSIONR)
            {
                return 0x04;
            }
            if (offset == (int)CommonOff.PHYCFGR)
            {
                return common[offset]; // 0xB8: link up
            }
            if (offset < CommonSize)
            {
                return common[offset];
            }
            return 0;
        }

        private void WriteCommon(ushort offset, byte value)
        {
            if (offset == (int)CommonOff.MR && (value & 0x80) != 0)
            {
                this.Log(LogLevel.Info, "W5500 software reset");
                // Preserve SPI framing state; reset registers/buffers.
                var loop = LoopbackMode; var host = EnableHostSockets; var verb = Verbose;
                Reset();
                LoopbackMode = loop; EnableHostSockets = host; Verbose = verb;
                return;
            }
            if (offset == (int)CommonOff.SIR || offset == (int)CommonOff.VERSIONR)
            {
                return; // read-only
            }
            if (offset == (int)CommonOff.IR)
            {
                // Write-1-to-clear.
                common[offset] &= (byte)~value;
                RefreshInterrupt();
                return;
            }
            if (offset < CommonSize)
            {
                common[offset] = value;
            }
        }

        private byte ComputeSIR()
        {
            byte sir = 0;
            for (var i = 0; i < SocketCount; i++)
            {
                if (sockets[i].Regs[(int)SockOff.IR] != 0)
                {
                    sir |= (byte)(1 << i);
                }
            }
            return sir;
        }

        // ---- socket registers ----

        private enum SockOff
        {
            MR = 0x0000,
            CR = 0x0001,
            IR = 0x0002,
            SR = 0x0003,
            PORT0 = 0x0004,
            PORT1 = 0x0005,
            DHAR0 = 0x0006,
            DIPR0 = 0x000C,
            DPORT0 = 0x0010,
            DPORT1 = 0x0011,
            MSSR0 = 0x0012,
            MSSR1 = 0x0013,
            TOS = 0x0015,
            TTL = 0x0016,
            RXBUF_SIZE = 0x001E,
            TXBUF_SIZE = 0x001F,
            TX_FSR0 = 0x0020,
            TX_FSR1 = 0x0021,
            TX_RD0 = 0x0022,
            TX_RD1 = 0x0023,
            TX_WR0 = 0x0024,
            TX_WR1 = 0x0025,
            RX_RSR0 = 0x0026,
            RX_RSR1 = 0x0027,
            RX_RD0 = 0x0028,
            RX_RD1 = 0x0029,
            RX_WR0 = 0x002A,
            RX_WR1 = 0x002B,
            IMR = 0x002C,
            FRAG0 = 0x002D,
            FRAG1 = 0x002E,
            KPALVTR = 0x002F,
        }

        // Sn_MR protocols
        private const byte P_CLOSE = 0x00, P_TCP = 0x01, P_UDP = 0x02, P_MACRAW = 0x04;
        // Sn_CR commands
        private const byte C_OPEN = 0x01, C_LISTEN = 0x02, C_CONNECT = 0x04, C_DISCON = 0x08,
            C_CLOSE = 0x10, C_SEND = 0x20, C_SEND_MAC = 0x21, C_SEND_KEEP = 0x22, C_RECV = 0x40;
        // Sn_IR bits
        private const byte IR_SENDOK = 0x10, IR_TIMEOUT = 0x08, IR_RECV = 0x04, IR_DISCON = 0x02, IR_CON = 0x01;
        // Sn_SR states
        private const byte S_CLOSED = 0x00, S_INIT = 0x13, S_LISTEN = 0x14, S_SYNSENT = 0x15,
            S_SYNRECV = 0x16, S_ESTABLISHED = 0x17, S_FINWAIT = 0x18, S_CLOSING = 0x1A,
            S_TIMEWAIT = 0x1B, S_CLOSEWAIT = 0x1C, S_LASTACK = 0x1D, S_UDP = 0x22, S_MACRAW = 0x02;

        private sealed class SocketState
        {
            public readonly byte[] Regs = new byte[0x30];
            public int TxWriteCount;
            public readonly byte[] TxBuf = new byte[16384];
            public readonly byte[] RxBuf = new byte[16384];
            public TcpClient Tcp;
            public TcpListener Listener;
            public NetworkStream Stream;
            public UdpClient Udp;
            public CancellationTokenSource Cts;
            public readonly object Lock = new object();

            public SocketState()
            {
                Regs[(int)SockOff.TXBUF_SIZE] = 2;
                Regs[(int)SockOff.RXBUF_SIZE] = 2;
                Regs[(int)SockOff.MSSR0] = 0x05; Regs[(int)SockOff.MSSR1] = 0x36; // 1336
                Regs[(int)SockOff.TTL] = 0x80;
                Regs[(int)SockOff.SR] = S_CLOSED;
            }
        }

        private int EffectiveTxSize(int s)
        {
            var kb = sockets[s].Regs[(int)SockOff.TXBUF_SIZE];
            return kb == 0 ? 16384 : Math.Min(16, (int)kb) * 1024;
        }

        private int EffectiveRxSize(int s)
        {
            var kb = sockets[s].Regs[(int)SockOff.RXBUF_SIZE];
            return kb == 0 ? 16384 : Math.Min(16, (int)kb) * 1024;
        }

        private static ushort GetU16(byte[] r, SockOff hi, SockOff lo)
        {
            return (ushort)((r[(int)hi] << 8) | r[(int)lo]);
        }

        private static void SetU16(byte[] r, SockOff hi, SockOff lo, ushort v)
        {
            r[(int)hi] = (byte)(v >> 8);
            r[(int)lo] = (byte)(v & 0xFF);
        }

        private byte ReadSocketReg(int s, ushort offset)
        {
            var st = sockets[s];
            switch (offset)
            {
                case (int)SockOff.TX_FSR0:
                case (int)SockOff.TX_FSR1:
                case (int)SockOff.RX_RSR0:
                case (int)SockOff.RX_RSR1:
                    {
                        ushort v = offset <= (int)SockOff.TX_FSR1 ? ComputeTxFsr(s) : ComputeRxRsr(s);
                        return offset % 2 == 0 ? (byte)(v >> 8) : (byte)(v & 0xFF);
                    }
                case (int)SockOff.CR:
                    return 0x00; // auto-cleared
                default:
                    if (offset < st.Regs.Length) return st.Regs[offset];
                    return 0;
            }
        }

        private void WriteSocketReg(int s, ushort offset, byte value)
        {
            var st = sockets[s];
            if (offset == (int)SockOff.SR || offset == (int)SockOff.TX_FSR0 || offset == (int)SockOff.TX_FSR1
                || offset == (int)SockOff.RX_RSR0 || offset == (int)SockOff.RX_RSR1
                || offset == (int)SockOff.TX_RD0 || offset == (int)SockOff.TX_RD1
                || offset == (int)SockOff.RX_WR0 || offset == (int)SockOff.RX_WR1)
            {
                return; // read-only
            }
            if (offset == (int)SockOff.IR)
            {
                st.Regs[offset] &= (byte)~value; // write-1-to-clear
                RefreshInterrupt();
                return;
            }
            if (offset == (int)SockOff.CR)
            {
                ExecuteCommand(s, value);
                return;
            }
            if (offset < st.Regs.Length)
            {
                st.Regs[offset] = value;
            }
        }

        private ushort ComputeTxFsr(int s)
        {
            var st = sockets[s];
            ushort wr = GetU16(st.Regs, SockOff.TX_WR0, SockOff.TX_WR1);
            ushort rd = GetU16(st.Regs, SockOff.TX_RD0, SockOff.TX_RD1);
            int eff = EffectiveTxSize(s);
            int used = (wr - rd) & 0xFFFF;
            if (used > eff) used = eff;
            return (ushort)(eff - used);
        }

        private ushort ComputeRxRsr(int s)
        {
            var st = sockets[s];
            ushort wr = GetU16(st.Regs, SockOff.RX_WR0, SockOff.RX_WR1);
            ushort rd = GetU16(st.Regs, SockOff.RX_RD0, SockOff.RX_RD1);
            return (ushort)((wr - rd) & 0xFFFF);
        }

        // ---- socket commands ----

        private void ExecuteCommand(int s, byte cmd)
        {
            var st = sockets[s];
            byte proto = (byte)(st.Regs[(int)SockOff.MR] & 0x0F);
            if (Verbose) this.Log(LogLevel.Debug, "W5500 s{0} CMD 0x{1:X2} proto {2} sr 0x{3:X2}", s, cmd, proto, st.Regs[(int)SockOff.SR]);
            switch (cmd)
            {
                case C_OPEN:
                    OpenSocket(s, proto);
                    break;
                case C_LISTEN:
                    if (st.Regs[(int)SockOff.SR] == S_INIT)
                    {
                        st.Regs[(int)SockOff.SR] = S_LISTEN;
                        StartTcpServer(s);
                    }
                    break;
                case C_CONNECT:
                    TcpConnect(s);
                    break;
                case C_DISCON:
                    st.Regs[(int)SockOff.IR] |= IR_DISCON;
                    CloseTransport(s);
                    st.Regs[(int)SockOff.SR] = S_CLOSED;
                    RefreshInterrupt();
                    break;
                case C_CLOSE:
                    CloseTransport(s);
                    st.Regs[(int)SockOff.SR] = S_CLOSED;
                    st.Regs[(int)SockOff.IR] = 0;
                    RefreshInterrupt();
                    break;
                case C_SEND:
                case C_SEND_MAC:
                case C_SEND_KEEP:
                    SendSocket(s);
                    break;
                case C_RECV:
                    // Host has already advanced RX_RD; clear RECV flag when drained.
                    if (ComputeRxRsr(s) == 0)
                    {
                        st.Regs[(int)SockOff.IR] &= unchecked((byte)~IR_RECV);
                        RefreshInterrupt();
                    }
                    break;
                default:
                    this.Log(LogLevel.Warning, "W5500 s{0} unknown command 0x{1:X2}", s, cmd);
                    break;
            }
        }

        private void OpenSocket(int s, byte proto)
        {
            var st = sockets[s];
            CloseTransport(s);
            SetU16(st.Regs, SockOff.TX_RD0, SockOff.TX_RD1, 0);
            SetU16(st.Regs, SockOff.TX_WR0, SockOff.TX_WR1, 0);
            SetU16(st.Regs, SockOff.RX_RD0, SockOff.RX_RD1, 0);
            SetU16(st.Regs, SockOff.RX_WR0, SockOff.RX_WR1, 0);
            st.Regs[(int)SockOff.IR] = 0;
            if (proto == P_TCP) st.Regs[(int)SockOff.SR] = S_INIT;
            else if (proto == P_UDP) { st.Regs[(int)SockOff.SR] = S_UDP; OpenUdp(s); }
            else if (proto == P_MACRAW) st.Regs[(int)SockOff.SR] = S_MACRAW;
            else st.Regs[(int)SockOff.SR] = S_CLOSED;
            RefreshInterrupt();
        }

        private void SendSocket(int s)
        {
            var st = sockets[s];
            byte proto = (byte)(st.Regs[(int)SockOff.MR] & 0x0F);
            ushort rd = GetU16(st.Regs, SockOff.TX_RD0, SockOff.TX_RD1);
            ushort wr = GetU16(st.Regs, SockOff.TX_WR0, SockOff.TX_WR1);
            int len = (wr - rd) & 0xFFFF;
            int eff = EffectiveTxSize(s);
            if (len > eff) len = eff;
            var data = new byte[len];
            for (var i = 0; i < len; i++)
            {
                data[i] = st.TxBuf[(rd + i) % eff];
            }
            if (Verbose && proto == P_TCP)
            {
                var dbg = new System.Text.StringBuilder();
                for (var d = 0; d < Math.Min(len, 8); d++)
                {
                    dbg.Append(data[d].ToString("X2") + " ");
                }
                this.Log(LogLevel.Debug, "W5500 s{0} TCP send rd={1} wr={2} len={3} eff={4} first=[{5}]",
                    s, rd, wr, len, eff, dbg.ToString().Trim());
            }
            bool sent = false;
            if (proto == P_UDP)
            {
                if (UseEthernet)
                {
                    sent = SendUdpOverEthernet(s, data);
                }
                else
                {
                    sent = UdpSend(s, data);
                }
            }
            else if (proto == P_TCP)
            {
                sent = TcpSend(s, data);
            }
            else
            {
                this.Log(LogLevel.Debug, "W5500 s{0} MACRAW send {1}B dropped", s, len);
            }
            if (!sent && LoopbackMode && len > 0)
            {
                if (proto == P_UDP)
                {
                    // ioLibrary recvfrom() expects the W5500 UDP RX format:
                    // DIPR(4) + DPORT(2) + LEN(2) + payload. Loop the packet
                    // back as if received from the destination we sent to.
                    var frame = new byte[8 + len];
                    frame[0] = st.Regs[(int)SockOff.DIPR0];
                    frame[1] = st.Regs[(int)SockOff.DIPR0 + 1];
                    frame[2] = st.Regs[(int)SockOff.DIPR0 + 2];
                    frame[3] = st.Regs[(int)SockOff.DIPR0 + 3];
                    frame[4] = st.Regs[(int)SockOff.DPORT0];
                    frame[5] = st.Regs[(int)SockOff.DPORT1];
                    frame[6] = (byte)(len >> 8);
                    frame[7] = (byte)(len & 0xFF);
                    Array.Copy(data, 0, frame, 8, len);
                    InjectRx(s, frame, null);
                }
                else
                {
                    InjectRx(s, data, null);
                }
            }
            // SEND consumes TX buffer.
            SetU16(st.Regs, SockOff.TX_RD0, SockOff.TX_RD1, wr);
            st.Regs[(int)SockOff.IR] |= IR_SENDOK;
            RefreshInterrupt();
        }

        // ---- host-socket bridge (optional) ----

        // ================= Ethernet (ARP + IPv4 + UDP) =================

        private const int EthHeaderLen = 14;
        private const int Ipv4HeaderLen = 20;
        private const int UdpHeaderLen = 8;
        private const ushort EtherTypeIPv4 = 0x0800;
        private const ushort EtherTypeArp = 0x0806;
        private const byte ProtocolUdp = 17;

        private MACAddress mac = MACAddress.Parse("00:F4:57:00:00:01");
        private readonly Dictionary<string, byte[]> arpCache = new Dictionary<string, byte[]>();
        private readonly List<PendingDatagram> pendingDatagrams = new List<PendingDatagram>();

        private class PendingDatagram
        {
            public int Socket;
            public byte[] DestIp;
            public ushort DestPort;
            public byte[] Payload;
        }

        private static string IpKey(byte[] ip) => string.Join(".", ip);

        private byte[] CurrentMacBytes
        {
            get
            {
                // The firmware programs SHAR; prefer it so the model reflects reality.
                var shar = new byte[6];
                for (var i = 0; i < 6; i++)
                {
                    shar[i] = common[(int)CommonOff.SHAR0 + i];
                }
                return shar[0] == 0 && shar[1] == 0 && shar[2] == 0 ? mac.Bytes : shar;
            }
        }

        private byte[] LocalIp()
        {
            var ip = new byte[4];
            for (var i = 0; i < 4; i++)
            {
                ip[i] = common[(int)CommonOff.SIPR0 + i];
            }
            return ip;
        }

        private byte[] SubnetMask()
        {
            var m = new byte[4];
            for (var i = 0; i < 4; i++)
            {
                m[i] = common[(int)CommonOff.SUBR0 + i];
            }
            return m;
        }

        private byte[] GatewayIp()
        {
            var g = new byte[4];
            for (var i = 0; i < 4; i++)
            {
                g[i] = common[(int)CommonOff.GAR0 + i];
            }
            return g;
        }

        private static bool SameSubnet(byte[] a, byte[] b, byte[] mask)
        {
            for (var i = 0; i < 4; i++)
            {
                if ((a[i] & mask[i]) != (b[i] & mask[i]))
                {
                    return false;
                }
            }
            return true;
        }

        private static ushort Checksum(byte[] data, int offset, int length)
        {
            uint sum = 0;
            for (var i = 0; i + 1 < length; i += 2)
            {
                sum += (uint)((data[offset + i] << 8) | data[offset + i + 1]);
            }
            if ((length & 1) != 0)
            {
                sum += (uint)(data[offset + length - 1] << 8);
            }
            while ((sum >> 16) != 0)
            {
                sum = (sum & 0xFFFF) + (sum >> 16);
            }
            return (ushort)~sum;
        }

        private bool SendUdpOverEthernet(int s, byte[] payload)
        {
            var st = sockets[s];
            var destIp = new byte[4];
            for (var i = 0; i < 4; i++)
            {
                destIp[i] = st.Regs[(int)SockOff.DIPR0 + i];
            }
            int dport = (st.Regs[(int)SockOff.DPORT0] << 8) | st.Regs[(int)SockOff.DPORT1];

            var mask = SubnetMask();
            var local = LocalIp();
            var target = SameSubnet(local, destIp, mask) ? destIp : GatewayIp();

            byte[] destMac;
            var key = IpKey(target);
            if (!arpCache.TryGetValue(key, out destMac))
            {
                pendingDatagrams.Add(new PendingDatagram
                {
                    Socket = s, DestIp = destIp, DestPort = (ushort)dport, Payload = payload,
                });
                SendArpRequest(target);
                this.Log(LogLevel.Debug, "W5500: ARP request queued for {0}, {1} datagram(s) pending",
                    key, pendingDatagrams.Count);
                return true;
            }

            EmitUdpFrame(destMac, destIp, (ushort)dport, local, payload, s);
            return true;
        }

        private void SendArpRequest(byte[] targetIp)
        {
            var srcMac = CurrentMacBytes;
            var srcIp = LocalIp();
            var frame = new byte[42];
            // Ethernet header: broadcast
            for (var i = 0; i < 6; i++) { frame[i] = 0xFF; }
            Array.Copy(srcMac, 0, frame, 6, 6);
            frame[12] = (byte)(EtherTypeArp >> 8); frame[13] = (byte)(EtherTypeArp & 0xFF);
            // ARP: request for IPv4
            frame[14] = 0x00; frame[15] = 0x01;   // htype = Ethernet
            frame[16] = 0x08; frame[17] = 0x00;   // ptype = IPv4
            frame[18] = 0x06;                     // hlen
            frame[19] = 0x04;                     // plen
            frame[20] = 0x00; frame[21] = 0x01;   // oper = request
            Array.Copy(srcMac, 0, frame, 22, 6);
            Array.Copy(srcIp, 0, frame, 28, 4);
            for (var i = 0; i < 6; i++) { frame[32 + i] = 0x00; }   // target hardware address
            Array.Copy(targetIp, 0, frame, 38, 4);                  // target protocol address
            EmitFrame(frame);
        }

        private void EmitUdpFrame(byte[] destMac, byte[] destIp, ushort destPort,
                                   byte[] srcIp, byte[] payload, int socketIndex)
        {
            var srcMac = CurrentMacBytes;
            var frame = new byte[EthHeaderLen + Ipv4HeaderLen + UdpHeaderLen + payload.Length];

            Array.Copy(destMac, 0, frame, 0, 6);
            Array.Copy(srcMac, 0, frame, 6, 6);
            frame[12] = (byte)(EtherTypeIPv4 >> 8); frame[13] = (byte)(EtherTypeIPv4 & 0xFF);

            var ip = EthHeaderLen;
            frame[ip + 0] = 0x45;                                  // IPv4, IHL=5
            frame[ip + 1] = 0x00;                                  // DSCP/ECN
            var ipLen = Ipv4HeaderLen + UdpHeaderLen + payload.Length;
            frame[ip + 2] = (byte)(ipLen >> 8);
            frame[ip + 3] = (byte)(ipLen & 0xFF);
            frame[ip + 4] = 0x00; frame[ip + 5] = 0x00;           // identification
            frame[ip + 6] = 0x40;                                  // DF
            frame[ip + 7] = 0x00;                                  // reserved
            frame[ip + 8] = 64;                                    // TTL
            frame[ip + 9] = ProtocolUdp;
            frame[ip + 10] = 0x00;                                 // checksum placeholder
            Array.Copy(srcIp, 0, frame, ip + 12, 4);
            Array.Copy(destIp, 0, frame, ip + 16, 4);
            var ipSum = Checksum(frame, ip, Ipv4HeaderLen);
            frame[ip + 10] = (byte)(ipSum >> 8);
            frame[ip + 11] = (byte)(ipSum & 0xFF);

            var udp = ip + Ipv4HeaderLen;
            int srcPort = SocketPort(socketIndex);
            frame[udp + 0] = (byte)(srcPort >> 8);
            frame[udp + 1] = (byte)(srcPort & 0xFF);
            frame[udp + 2] = (byte)(destPort >> 8);
            frame[udp + 3] = (byte)(destPort & 0xFF);
            var udpLen = UdpHeaderLen + payload.Length;
            frame[udp + 4] = (byte)(udpLen >> 8);
            frame[udp + 5] = (byte)(udpLen & 0xFF);
            frame[udp + 6] = 0x00;                                 // checksum 0 == unused (legal for IPv4 UDP)
            frame[udp + 7] = 0x00;
            Array.Copy(payload, 0, frame, udp + UdpHeaderLen, payload.Length);

            EmitFrame(frame);
        }

        private int SocketPort(int s)
        {
            var st = sockets[s];
            return (st.Regs[(int)SockOff.PORT0] << 8) | st.Regs[(int)SockOff.PORT1];
        }

        private void EmitFrame(byte[] raw)
        {
            EthernetFrame frame;
            if (!EthernetFrame.TryCreateEthernetFrame(raw, CRCMode.Add, out frame))
            {
                this.Log(LogLevel.Warning, "W5500: failed to build frame ({0} bytes)", raw.Length);
                return;
            }
            FrameReady?.Invoke(frame);
        }

        public void ReceiveFrame(EthernetFrame frame)
        {
            lock (sync)
            {
                var data = frame.Bytes;
                if (data.Length < EthHeaderLen + 28)
                {
                    return;
                }

                var ethType = (ushort)((data[12] << 8) | data[13]);
                var destMac = new byte[6];
                Array.Copy(data, 0, destMac, 0, 6);
                var srcMac = new byte[6];
                Array.Copy(data, 6, srcMac, 0, 6);
                var srcIp = new byte[4];
                var arpSenderIpOffset = ethType == EtherTypeArp ? 28 : 26;
                Array.Copy(data, arpSenderIpOffset, srcIp, 0, 4);

                if (ethType == EtherTypeArp)
                {
                    HandleArp(data, srcMac, srcIp);
                    return;
                }
                if (ethType != EtherTypeIPv4)
                {
                    return;
                }

                var ipOff = EthHeaderLen;
                var proto = data[ipOff + 9];
                if (proto != ProtocolUdp)
                {
                    return;
                }
                var destIp = new byte[4];
                Array.Copy(data, ipOff + 16, destIp, 0, 4);
                var myIp = LocalIp();
                if (!destIp.SequenceEqual(myIp))
                {
                    return;
                }
                var myMac = CurrentMacBytes;
                var broadcast = destMac.All(b => b == 0xFF);
                if (!broadcast && !destMac.SequenceEqual(myMac))
                {
                    return;
                }

                var udpOff = ipOff + Ipv4HeaderLen;
                var dport = (ushort)((data[udpOff + 2] << 8) | data[udpOff + 3]);
                var udpLen = (data[udpOff + 4] << 8) | data[udpOff + 5];
                if (udpLen < UdpHeaderLen)
                {
                    return;
                }
                var payloadLen = Math.Min(udpLen - UdpHeaderLen, data.Length - (udpOff + UdpHeaderLen));

                var sport = (ushort)((data[udpOff + 0] << 8) | data[udpOff + 1]);
                DeliverUdpToSocket(srcIp, sport, dport, data, udpOff + UdpHeaderLen, payloadLen);
            }
        }

        private void DeliverUdpToSocket(byte[] peerIp, ushort peerPort, ushort dport,
                                        byte[] data, int offset, int length)
        {
            for (var s = 0; s < SocketCount; s++)
            {
                if ((sockets[s].Regs[(int)SockOff.MR] & 0x0F) != P_UDP)
                {
                    continue;
                }
                if (SocketPort(s) != dport)
                {
                    continue;
                }
                // W5500 UDP RX layout: peer IP(4) + peer port(2) + length(2) + data
                var payload = new byte[8 + length];
                Array.Copy(peerIp, 0, payload, 0, 4);
                payload[4] = (byte)(peerPort >> 8);
                payload[5] = (byte)(peerPort & 0xFF);
                payload[6] = (byte)(length >> 8);
                payload[7] = (byte)(length & 0xFF);
                Array.Copy(data, offset, payload, 8, length);
                InjectRx(s, payload, null);
                return;
            }
            this.Log(LogLevel.Debug, "W5500: no UDP socket bound to port {0}, dropping", dport);
        }

        private void HandleArp(byte[] data, byte[] srcMac, byte[] srcIp)
        {
            var oper = (ushort)((data[20] << 8) | data[21]);
            var targetIp = new byte[4];
            Array.Copy(data, 38, targetIp, 0, 4);

            if (oper == 2)
            {
                // reply: learn sender
                arpCache[IpKey(srcIp)] = srcMac;
                FlushPendingArp();
                return;
            }
            if (oper != 1)
            {
                return;
            }

            var myIp = LocalIp();
            if (!targetIp.SequenceEqual(myIp))
            {
                return;
            }

            var myMac = CurrentMacBytes;
            var reply = new byte[42];
            for (var i = 0; i < 6; i++) { reply[i] = 0xFF; }
            Array.Copy(myMac, 0, reply, 6, 6);
            reply[12] = (byte)(EtherTypeArp >> 8); reply[13] = (byte)(EtherTypeArp & 0xFF);
            reply[14] = 0x00; reply[15] = 0x01;
            reply[16] = 0x08; reply[17] = 0x00;
            reply[18] = 0x06;
            reply[19] = 0x04;
            reply[20] = 0x00; reply[21] = 0x02;   // oper = reply
            Array.Copy(myMac, 0, reply, 22, 6);
            Array.Copy(myIp, 0, reply, 28, 4);
            Array.Copy(srcMac, 0, reply, 32, 6);
            Array.Copy(srcIp, 0, reply, 38, 4);
            EmitFrame(reply);
        }

        private void FlushPendingArp()
        {
            if (pendingDatagrams.Count == 0)
            {
                return;
            }
            var stillPending = new List<PendingDatagram>();
            foreach (var pd in pendingDatagrams)
            {
                var mask = SubnetMask();
                var local = LocalIp();
                var target = SameSubnet(local, pd.DestIp, mask) ? pd.DestIp : GatewayIp();
                byte[] destMac;
                if (arpCache.TryGetValue(IpKey(target), out destMac))
                {
                    EmitUdpFrame(destMac, pd.DestIp, pd.DestPort, local, pd.Payload, pd.Socket);
                }
                else
                {
                    stillPending.Add(pd);
                }
            }
            pendingDatagrams.Clear();
            pendingDatagrams.AddRange(stillPending);
        }

        // ================= host-socket bridge =================

        private void OpenUdp(int s)
        {
            if (!EnableHostSockets) return;
            try
            {
                CloseTransport(s);
                var st = sockets[s];
                int port = GetU16(st.Regs, SockOff.PORT0, SockOff.PORT1);
                st.Udp = new UdpClient(port == 0 ? 0 : port);
                st.Cts = new CancellationTokenSource();
                var tok = st.Cts.Token;
                Task.Run(() => UdpReceiveLoop(s, tok));
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Warning, "W5500 s{0} UDP open failed: {1}", s, e.Message);
            }
        }

        private bool UdpSend(int s, byte[] data)
        {
            if (!EnableHostSockets) return false;
            try
            {
                var st = sockets[s];
                if (st.Udp == null) return false;
                var ip = new IPAddress(new byte[] { st.Regs[(int)SockOff.DIPR0], st.Regs[(int)SockOff.DIPR0 + 1], st.Regs[(int)SockOff.DIPR0 + 2], st.Regs[(int)SockOff.DIPR0 + 3] });
                int port = (st.Regs[(int)SockOff.DPORT0] << 8) | st.Regs[(int)SockOff.DPORT1];
                st.Udp.Send(data, data.Length, new IPEndPoint(ip, port));
                return true;
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Warning, "W5500 s{0} UDP send failed: {1}", s, e.Message);
                return false;
            }
        }

        private void UdpReceiveLoop(int s, CancellationToken tok)
        {
            var st = sockets[s];
            while (!tok.IsCancellationRequested)
            {
                try
                {
                    var ep = new IPEndPoint(IPAddress.Any, 0);
                    var data = st.Udp.Receive(ref ep);
                    // W5500 UDP RX format: DIPR(4) DPORT(2) LEN(2) + payload.
                    var frame = new byte[8 + data.Length];
                    var b = ep.Address.GetAddressBytes();
                    Array.Copy(b, 0, frame, 0, 4);
                    frame[4] = (byte)(ep.Port >> 8); frame[5] = (byte)(ep.Port & 0xFF);
                    frame[6] = (byte)(data.Length >> 8); frame[7] = (byte)(data.Length & 0xFF);
                    Array.Copy(data, 0, frame, 8, data.Length);
                    lock (sync) InjectRx(s, frame, null);
                }
                catch { if (tok.IsCancellationRequested) break; Thread.Sleep(10); }
            }
        }

        private void StartTcpServer(int s)
        {
            if (!EnableHostSockets) return;
            try
            {
                var st = sockets[s];
                int port = GetU16(st.Regs, SockOff.PORT0, SockOff.PORT1);
                st.Listener = new TcpListener(IPAddress.Any, port == 0 ? 0 : port);
                st.Listener.Start();
                st.Cts = new CancellationTokenSource();
                var tok = st.Cts.Token;
                Task.Run(() =>
                {
                    try
                    {
                        var client = st.Listener.AcceptTcpClient();
                        lock (sync)
                        {
                            st.Tcp = client;
                            st.Stream = client.GetStream();
                            st.Regs[(int)SockOff.SR] = S_ESTABLISHED;
                            st.Regs[(int)SockOff.IR] |= IR_CON;
                            RefreshInterrupt();
                        }
                        Task.Run(() => TcpReceiveLoop(s, tok));
                    }
                    catch { }
                }, tok);
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Warning, "W5500 s{0} TCP listen failed: {1}", s, e.Message);
            }
        }

        private void TcpConnect(int s)
        {
            var st = sockets[s];
            st.Regs[(int)SockOff.SR] = S_SYNSENT;
            if (!EnableHostSockets || !LoopbackMode)
            {
                if (!EnableHostSockets && LoopbackMode)
                {
                    st.Regs[(int)SockOff.SR] = S_ESTABLISHED;
                    st.Regs[(int)SockOff.IR] |= IR_CON;
                    RefreshInterrupt();
                    return;
                }
            }
            try
            {
                var ip = new IPAddress(new byte[] { st.Regs[(int)SockOff.DIPR0], st.Regs[(int)SockOff.DIPR0 + 1], st.Regs[(int)SockOff.DIPR0 + 2], st.Regs[(int)SockOff.DIPR0 + 3] });
                int port = (st.Regs[(int)SockOff.DPORT0] << 8) | st.Regs[(int)SockOff.DPORT1];
                var cts = new CancellationTokenSource();
                st.Cts = cts;
                Task.Run(() =>
                {
                    try
                    {
                        var c = new TcpClient();
                        if (c.ConnectAsync(ip, port).Wait(2000))
                        {
                            lock (sync)
                            {
                                st.Tcp = c;
                                st.Stream = c.GetStream();
                                st.Regs[(int)SockOff.SR] = S_ESTABLISHED;
                                st.Regs[(int)SockOff.IR] |= IR_CON;
                                RefreshInterrupt();
                            }
                            Task.Run(() => TcpReceiveLoop(s, cts.Token));
                        }
                        else
                        {
                            lock (sync)
                            {
                                if (LoopbackMode)
                                {
                                    st.Regs[(int)SockOff.SR] = S_ESTABLISHED;
                                    st.Regs[(int)SockOff.IR] |= IR_CON;
                                }
                                else
                                {
                                    st.Regs[(int)SockOff.SR] = S_CLOSED;
                                    st.Regs[(int)SockOff.IR] |= IR_TIMEOUT;
                                }
                                RefreshInterrupt();
                            }
                        }
                    }
                    catch
                    {
                        lock (sync)
                        {
                            if (LoopbackMode)
                            {
                                st.Regs[(int)SockOff.SR] = S_ESTABLISHED;
                                st.Regs[(int)SockOff.IR] |= IR_CON;
                            }
                            else
                            {
                                st.Regs[(int)SockOff.SR] = S_CLOSED;
                                st.Regs[(int)SockOff.IR] |= IR_TIMEOUT;
                            }
                            RefreshInterrupt();
                        }
                    }
                });
            }
            catch
            {
                st.Regs[(int)SockOff.SR] = LoopbackMode ? S_ESTABLISHED : S_CLOSED;
                if (LoopbackMode) st.Regs[(int)SockOff.IR] |= IR_CON;
                else st.Regs[(int)SockOff.IR] |= IR_TIMEOUT;
                RefreshInterrupt();
            }
        }

        private bool TcpSend(int s, byte[] data)
        {
            var st = sockets[s];
            try
            {
                if (st.Stream != null && st.Tcp != null && st.Tcp.Connected)
                {
                    st.Stream.Write(data, 0, data.Length);
                    return true;
                }
            }
            catch (Exception e)
            {
                this.Log(LogLevel.Warning, "W5500 s{0} TCP send failed: {1}", s, e.Message);
            }
            return false;
        }

        private void TcpReceiveLoop(int s, CancellationToken tok)
        {
            var st = sockets[s];
            var tmp = new byte[2048];
            while (!tok.IsCancellationRequested)
            {
                try
                {
                    int n = st.Stream.Read(tmp, 0, tmp.Length);
                    if (n <= 0) break;
                    var data = new byte[n];
                    Array.Copy(tmp, data, n);
                    lock (sync) InjectRx(s, data, null);
                }
                catch { break; }
            }
            lock (sync)
            {
                st.Regs[(int)SockOff.IR] |= IR_DISCON;
                if (st.Regs[(int)SockOff.SR] == S_ESTABLISHED) st.Regs[(int)SockOff.SR] = S_CLOSEWAIT;
                RefreshInterrupt();
            }
        }

        private void CloseTransport(int s)
        {
            var st = sockets[s];
            try { st.Cts?.Cancel(); } catch { }
            try { st.Stream?.Close(); } catch { }
            try { st.Tcp?.Close(); } catch { }
            try { st.Listener?.Stop(); } catch { }
            try { st.Udp?.Close(); } catch { }
            st.Stream = null; st.Tcp = null; st.Listener = null; st.Udp = null; st.Cts = null;
        }

        // Copy payload into RX buffer at RX_WR, advance RX_WR, raise RECV.
        private void InjectRx(int s, byte[] data, byte[] _unused)
        {
            var st = sockets[s];
            int eff = EffectiveRxSize(s);
            ushort wr = GetU16(st.Regs, SockOff.RX_WR0, SockOff.RX_WR1);
            ushort rsr = ComputeRxRsr(s);
            if (rsr + data.Length > eff)
            {
                this.Log(LogLevel.Warning, "W5500 s{0} RX overflow, dropping {1}B", s, data.Length);
                return;
            }
            for (var i = 0; i < data.Length; i++)
            {
                st.RxBuf[(wr + i) % eff] = data[i];
            }
            SetU16(st.Regs, SockOff.RX_WR0, SockOff.RX_WR1, (ushort)(wr + data.Length));
            st.Regs[(int)SockOff.IR] |= IR_RECV;
            RefreshInterrupt();
        }

        private void RefreshInterrupt()
        {
            byte sir = ComputeSIR();
            byte simr = common[(int)CommonOff.SIMR];
            bool any = false;
            for (var i = 0; i < SocketCount; i++)
            {
                byte snIr = sockets[i].Regs[(int)SockOff.IR];
                byte snImr = sockets[i].Regs[(int)SockOff.IMR];
                // W5500 default IMR after reset is 0xFF? ioLibrary expects 0xFF to get interrupts.
                // Treat IMR==0 as "no mask programmed yet" -> still report via SIR so polling works,
                // but only assert INTn when SIMR+Sn_IMR enable it or IMR is default.
                if ((sir & (1 << i)) != 0 && (simr & (1 << i)) != 0 && ((snIr & snImr) != 0 || snImr == 0))
                {
                    any = true;
                }
            }
            // INTn active low.
            IRQ.Set(!any);
        }

        private const int SocketCount = 8;

        private enum SpiPhase { OffsetHigh, OffsetLow, Control, Data }

        private readonly object sync = new object();
        private SpiPhase spiPhase = SpiPhase.OffsetHigh;
        private ushort spiOffset;
        private byte spiBsb;
        private bool spiIsWrite;
        private int spiFixedRemaining = -1;
        private bool csActive;

        private byte[] common = new byte[CommonSize];
        private SocketState[] sockets = new SocketState[SocketCount];
    }
}
