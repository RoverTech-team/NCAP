using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Antmicro.Renode.Peripherals.Network;

public static class Program
{
    static int failures = 0;

    static void Check(bool cond, string name)
    {
        Console.WriteLine((cond ? "PASS " : "FAIL ") + name);
        if (!cond) failures++;
    }

    // SCSn/RSTn are active low: 1 = asserted.
    const byte CSn = 0;
    const byte RSTn = 1;

    static byte[] Frame(W5500 w, ushort offset, byte bsb, bool write, byte[] data)
    {
        byte control = (byte)((bsb << 3) | (write ? 0x04 : 0x00)); // OM=00 VDM
        w.OnGPIO(CSn, true);   // CSn low: frame start
        w.Transmit((byte)(offset >> 8));
        w.Transmit((byte)(offset & 0xFF));
        w.Transmit(control);
        var Out = new byte[data.Length];
        for (int i = 0; i < data.Length; i++) Out[i] = w.Transmit(data[i]);
        w.OnGPIO(CSn, false);  // CSn high: frame end
        return Out;
    }

    static byte Read(W5500 w, ushort offset, byte bsb) => Frame(w, offset, bsb, false, new byte[1])[0];
    static void Write(W5500 w, ushort offset, byte bsb, byte val) => Frame(w, offset, bsb, true, new byte[] { val });
    static void WriteBuf(W5500 w, ushort offset, byte bsb, byte[] payload) => Frame(w, offset, bsb, true, payload);
    static byte[] ReadBuf(W5500 w, ushort offset, byte bsb, int len) => Frame(w, offset, bsb, false, new byte[len]);
    static void WriteReg(W5500 w, int sock, ushort off, byte v) => Write(w, off, (byte)(1 + 4 * sock), v);
    static byte ReadReg(W5500 w, int sock, ushort off) => Read(w, off, (byte)(1 + 4 * sock));
    static ushort ReadU16(W5500 w, int sock, ushort hi)
    {
        return (ushort)((ReadReg(w, sock, hi) << 8) | ReadReg(w, sock, (ushort)(hi + 1)));
    }

    public static int Main()
    {
        var w = new W5500();

        // 1. VERSIONR must read 0x04 (common block BSB=0, offset 0x0039)
        Check(Read(w, 0x0039, 0x00) == 0x04, "VERSIONR==0x04");

        // 2. Common R/W: GAR0
        Write(w, 0x0001, 0x00, 0xC0);
        Check(Read(w, 0x0001, 0x00) == 0xC0, "GAR0 write/read");

        // 3. Socket0 OPEN TCP -> SR==SOCK_INIT (0x13)
        WriteReg(w, 0, 0x0000, 0x01); // Sn_MR = TCP
        WriteReg(w, 0, 0x0001, 0x01); // Sn_CR = OPEN
        Check(ReadReg(w, 0, 0x0003) == 0x13, "socket0 OPEN TCP -> INIT");
        Check(ReadU16(w, 0, 0x0020) == 2048, "TX_FSR==2048 after OPEN");

        // 4. TCP loopback SEND: raw payload echoed
        WriteBuf(w, 0x0000, 0x02, new byte[] { 0x48, 0x69 });
        WriteReg(w, 0, 0x0024, 0x00); WriteReg(w, 0, 0x0025, 0x02); // TX_WR = 2
        WriteReg(w, 0, 0x0001, 0x20); // SEND
        Check((ReadReg(w, 0, 0x0002) & 0x10) != 0, "Sn_IR SEND_OK set");
        Check(ReadU16(w, 0, 0x0026) == 2, "TCP RX_RSR==2 after loopback");
        var rx = ReadBuf(w, 0x0000, 0x03, 2);
        Check(rx[0] == 0x48 && rx[1] == 0x69, "TCP loopback payload matches");

        // 5. RECV clears RECV flag when drained
        WriteReg(w, 0, 0x0028, 0x00); WriteReg(w, 0, 0x0029, 0x02); // RX_RD = 2
        WriteReg(w, 0, 0x0001, 0x40); // RECV
        Check((ReadReg(w, 0, 0x0002) & 0x04) == 0, "Sn_IR RECV cleared after drain");

        // 6. CLOSE -> SR==CLOSED
        WriteReg(w, 0, 0x0001, 0x10);
        Check(ReadReg(w, 0, 0x0003) == 0x00, "socket0 CLOSE -> CLOSED");

        // 7. UDP OPEN -> SR==SOCK_UDP (0x22), UDP loopback carries 8-byte header
        WriteReg(w, 0, 0x0000, 0x02); // Sn_MR = UDP
        WriteReg(w, 0, 0x0001, 0x01); // OPEN
        Check(ReadReg(w, 0, 0x0003) == 0x22, "socket0 OPEN UDP -> UDP");
        WriteReg(w, 0, 0x000C, 192); WriteReg(w, 0, 0x000D, 168);
        WriteReg(w, 0, 0x000E, 0); WriteReg(w, 0, 0x000F, 1); // DIPR=192.168.0.1
        WriteReg(w, 0, 0x0010, 0x22); WriteReg(w, 0, 0x0011, 0xB8); // DPORT=8888
        WriteBuf(w, 0x0000, 0x02, new byte[] { 0x48, 0x69 });
        WriteReg(w, 0, 0x0024, 0x00); WriteReg(w, 0, 0x0025, 0x02); // TX_WR = 2
        WriteReg(w, 0, 0x0001, 0x20); // SEND
        Check(ReadU16(w, 0, 0x0026) == 10, "UDP RX_RSR==10 (8 hdr + 2)");
        var uhdr = ReadBuf(w, 0x0000, 0x03, 10);
        bool hdrOk = uhdr[0] == 192 && uhdr[1] == 168 && uhdr[2] == 0 && uhdr[3] == 1
            && uhdr[4] == 0x22 && uhdr[5] == 0xB8 && uhdr[6] == 0x00 && uhdr[7] == 0x02
            && uhdr[8] == 0x48 && uhdr[9] == 0x69;
        Check(hdrOk, "UDP loopback header (peer/port/len) + payload parse");

        // 8. Host bridge: SEND to a real UDP echo server, RX parses sender header
        var wb = new W5500 { LoopbackMode = false, EnableHostSockets = true };
        var echoCts = new CancellationTokenSource();
        Task.Run(() =>
        {
            var srv = new UdpClient(19988);
            try
            {
                while (!echoCts.Token.IsCancellationRequested)
                {
                    var ep = new IPEndPoint(IPAddress.Any, 0);
                    var d = srv.Receive(ref ep);
                    srv.Send(d, d.Length, ep);
                }
            }
            catch { }
            finally { srv.Close(); }
        });
        Thread.Sleep(200);
        WriteReg(wb, 1, 0x0000, 0x02); // socket1 UDP
        WriteReg(wb, 1, 0x0001, 0x01); // OPEN
        Check(ReadReg(wb, 1, 0x0003) == 0x22, "bridge socket OPEN UDP");
        WriteReg(wb, 1, 0x000C, 127); WriteReg(wb, 1, 0x000D, 0);
        WriteReg(wb, 1, 0x000E, 0); WriteReg(wb, 1, 0x000F, 1); // DIPR=127.0.0.1
        WriteReg(wb, 1, 0x0010, 0x4E); WriteReg(wb, 1, 0x0011, 0x14); // DPORT=19988 (0x4E14)
        WriteBuf(wb, 0x0000, 0x06, new byte[] { 0x70, 0x69, 0x6E, 0x67 }); // s1 TX buf BSB=2+4*1=6
        WriteReg(wb, 1, 0x0024, 0x00); WriteReg(wb, 1, 0x0025, 0x04); // TX_WR=4
        WriteReg(wb, 1, 0x0001, 0x20); // SEND
        Thread.Sleep(1000);
        bool gotRsr = ReadU16(wb, 1, 0x0026) == 12;
        Check(gotRsr, "bridge RX_RSR==12 (echo reply received)");
        if (gotRsr)
        {
            var b = ReadBuf(wb, 0x0000, 0x07, 12); // s1 RX buf BSB=3+4*1=7
            bool ok = b[0] == 127 && b[1] == 0 && b[2] == 0 && b[3] == 1
                && b[4] == 0x4E && b[5] == 0x14 && b[6] == 0x00 && b[7] == 0x04
                && b[8] == 0x70 && b[9] == 0x69 && b[10] == 0x6E && b[11] == 0x67;
            Check(ok, "bridge recvfrom header (127.0.0.1:19988/len) + payload");
        }
        else
        {
            Check(false, "bridge recvfrom header (127.0.0.1:19988/len) + payload");
        }
        echoCts.Cancel();

        // 9. CSn framing: two frames back-to-back without FinishTransmission()
        //    (this is how STM32SPI drives a real peripheral).
        var wcs = new W5500();
        byte first = 0, second = 0;
        wcs.OnGPIO(CSn, true);
        wcs.Transmit(0x00); wcs.Transmit(0x39); wcs.Transmit(0x00); first = wcs.Transmit(0x00);
        wcs.OnGPIO(CSn, false);
        Check(first == 0x04, "CSn framed read VERSIONR==0x04");
        wcs.OnGPIO(CSn, true);
        wcs.Transmit(0x00); wcs.Transmit(0x39); wcs.Transmit(0x00); second = wcs.Transmit(0x00);
        wcs.OnGPIO(CSn, false);
        Check(second == 0x04, "second CSn framed read VERSIONR==0x04 (no desync)");

        // 10. RSTn (active low) clears state
        var wrst = new W5500();
        WriteReg(wrst, 0, 0x0000, 0x02);   // Sn_MR = UDP
        WriteReg(wrst, 0, 0x0001, 0x01);   // OPEN
        Check(ReadReg(wrst, 0, 0x0003) == 0x22, "socket0 OPEN UDP before reset");
        wrst.OnGPIO(RSTn, false);          // RSTn low
        Check(ReadReg(wrst, 0, 0x0003) == 0x00, "RSTn resets socket0 to CLOSED");

        // 11. FDM (OM=01) fixed 1-byte frame works without CS toggling
        var wfdm = new W5500();
        wfdm.Transmit(0x00); wfdm.Transmit(0x39); wfdm.Transmit(0x01); // OM=01, RWB=0, BSB=0
        Check(wfdm.Transmit(0x00) == 0x04, "FDM 1-byte VERSIONR==0x04");

        EthernetTests.Run(Check, Write, WriteBuf, ReadBuf, ReadReg, WriteReg, ReadU16);

        Console.WriteLine(failures == 0 ? "ALL TESTS PASSED" : failures + " FAILURES");
        return failures;
    }
}
