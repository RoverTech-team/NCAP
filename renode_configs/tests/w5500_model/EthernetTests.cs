// Tests for the W5500 Ethernet mode (IMACInterface): the model terminates
// ARP/IPv4/UDP itself and emits real frames, so the micro-ROS agent sees a
// stable client MAC/IP instead of host-socket/docker-proxy churn.
//
// No root and no TAP required - these exercise the frame TX/RX path in-process.

using System;
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Network;
using Antmicro.Renode.Peripherals.Network;

internal static class EthernetTests
{
    private const byte EthHeaderLen = 14;
    private const byte Ipv4HeaderLen = 20;

    private static readonly byte[] HostMac = new byte[] { 0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
    private static readonly byte[] HostIp = new byte[] { 192, 168, 0, 213 };
    private static readonly byte[] OurIp = new byte[] { 192, 168, 0, 10 };

    public static void Run(
        Action<bool, string> check,
        Action<W5500, ushort, byte, byte> Write,
        Action<W5500, ushort, byte, byte[]> WriteBuf,
        Func<W5500, ushort, byte, int, byte[]> ReadBuf,
        Func<W5500, int, ushort, byte> ReadReg,
        Action<W5500, int, ushort, byte> WriteReg,
        Func<W5500, int, ushort, ushort> ReadU16)
    {
        var w = new W5500 { UseEthernet = true, LoopbackMode = false, Verbose = true };

        var sent = new List<EthernetFrame>();
        w.FrameReady += f => sent.Add(f);

        // Program the same values w5500_hw_init() writes: SHAR, SIPR, SUBR, GAR
        byte[] myMac = new byte[] { 0x00, 0xF4, 0x57, 0x00, 0x00, 0x01 };
        WriteBuf(w, 0x0009, 0x00, myMac);                 // SHAR
        WriteBuf(w, 0x000F, 0x00, OurIp);                // SIPR
        WriteBuf(w, 0x0005, 0x00, new byte[] { 255, 255, 255, 0 }); // SUBR
        WriteBuf(w, 0x0001, 0x00, new byte[] { 192, 168, 0, 1 });  // GAR

        // Socket0: UDP, PORT=5000, dest = agent
        WriteReg(w, 0, 0x0000, 0x02);                    // Sn_MR = UDP
        WriteReg(w, 0, 0x0004, 0x00); WriteReg(w, 0, 0x0005, 0xF4); // Sn_PORT = 0x00F4 = 244
        WriteBuf(w, 0x000C, 0x01, HostIp);               // Sn_DIPR = agent IP
        WriteReg(w, 0, 0x0010, 0x22); WriteReg(w, 0, 0x0011, 0xB8); // Sn_DPORT = 8888
        WriteReg(w, 0, 0x0001, 0x01);                    // Sn_CR = OPEN
        check(ReadReg(w, 0, 0x0003) == 0x22, "eth: socket0 OPEN UDP");

        // sendto a payload -> model must ARP first (cache is cold)
        var payload = new byte[] { 0x81, 0x00, 0x00, 0x00, 0x04, 0x01, 0x0B };
        WriteBuf(w, 0x0000, 0x02, payload);
        WriteReg(w, 0, 0x0024, 0x00); WriteReg(w, 0, 0x0025, (byte)payload.Length); // TX_WR
        WriteReg(w, 0, 0x0001, 0x20);                    // SEND

        check(sent.Count == 1 && sent[0].Bytes[12] == 0x08 && sent[0].Bytes[13] == 0x06,
            "eth: cold cache emits an ARP request, not UDP");
        check(sent.Count > 0 && sent[0].Bytes.Take(6).All(b => b == 0xFF),
            "eth: ARP request is broadcast");
        check(sent.Count > 0 && sent[0].Bytes[20] == 0x00 && sent[0].Bytes[21] == 0x01,
            "eth: ARP oper == request");
        check(sent.Count > 0 && sent[0].Bytes.Skip(38).Take(4).SequenceEqual(HostIp),
            "eth: ARP asks for the agent IP");

        // Reply to that ARP request so the cache fills, and the queued datagram flushes.
        w.ReceiveFrame(MakeFrame(BuildArpReply(HostMac, HostIp, myMac, OurIp)));
        check(sent.Count == 2, "eth: ARP reply flushes the queued datagram");

        var udp = sent.Count > 1 ? sent[1].Bytes : new byte[0];
        check(udp.Length > EthHeaderLen + Ipv4HeaderLen && udp[12] == 0x08 && udp[13] == 0x00,
            "eth: queued datagram emits IPv4 UDP");
        check(udp.Skip(0).Take(6).SequenceEqual(HostMac), "eth: frame dest MAC == agent MAC");
        check(udp.Skip(6).Take(6).SequenceEqual(myMac), "eth: frame src MAC == SHAR");
        check(udp.Skip(26).Take(4).SequenceEqual(OurIp), "eth: IP src == SIPR");
        check(udp.Skip(30).Take(4).SequenceEqual(HostIp), "eth: IP dst == Sn_DIPR");

        var ipOff = EthHeaderLen;
        check(IpChecksumOk(udp, ipOff, Ipv4HeaderLen), "eth: IPv4 header checksum valid");
        check(udp[ipOff + 9] == 17, "eth: IP protocol == UDP");

        var udpOff = ipOff + Ipv4HeaderLen;
        var dport = (ushort)((udp[udpOff + 2] << 8) | udp[udpOff + 3]);
        var sport = (ushort)((udp[udpOff + 0] << 8) | udp[udpOff + 1]);
        var udpLen = (udp[udpOff + 4] << 8) | udp[udpOff + 5];
        check(dport == 8888, "eth: UDP dest port == Sn_DPORT");
        check(sport == 244, "eth: UDP source port == Sn_PORT");
        check(udpLen == 8 + payload.Length, "eth: UDP length field == 8 + payload");
        check(udp.Skip(udpOff + 8).Take(payload.Length).SequenceEqual(payload),
            "eth: UDP payload matches what firmware wrote");

        // Inbound UDP from the agent -> must land in the W5500 RX buffer with
        // the peer-IP / peer-port / length prefix that recvfrom expects.
        var agentPayload = new byte[] { 0x81, 0x00, 0x00, 0x00, 0x04, 0x01, 0x0B, 0x00, 0x00, 0x00, 0x58 };
        w.ReceiveFrame(MakeFrame(BuildUdpFrame(myMac, OurIp, HostMac, HostIp, 8888, 244, agentPayload)));

        var rsr = ReadU16(w, 0, 0x0026);
        check(rsr == 8 + agentPayload.Length, "eth: RX_RSR == 8 + datagram after inbound frame");
        var rx = ReadBuf(w, 0x0000, 0x03, 8 + agentPayload.Length);
        check(rx[0] == 192 && rx[1] == 168 && rx[2] == 0 && rx[3] == 213,
            "eth: recvfrom prefix carries the peer IP");
        check(((rx[4] << 8) | rx[5]) == 8888, "eth: recvfrom prefix carries the peer port");
        check(((rx[6] << 8) | rx[7]) == agentPayload.Length, "eth: recvfrom prefix carries length");
        check(rx.Skip(8).SequenceEqual(agentPayload), "eth: inbound payload delivered intact");

        // ARP for our IP must be answered with our MAC
        var before = sent.Count;
        // The agent asks who owns 192.168.0.10
        var req = BuildArp(HostMac, HostIp, Broadcast, OurIp, ZeroMac, 1);
        w.ReceiveFrame(MakeFrame(req));
        check(sent.Count == before + 1, "eth: ARP request for our IP is answered");
        if (sent.Count > before)
        {
            var rep = sent[before].Bytes;
            check(rep.Skip(6).Take(6).SequenceEqual(myMac), "eth: ARP reply source MAC == SHAR");
            check(rep.Skip(28).Take(4).SequenceEqual(OurIp), "eth: ARP reply target IP == SIPR");
            check(rep.Skip(32).Take(6).SequenceEqual(HostMac), "eth: ARP reply echoes the requester MAC");
        }

        // Traffic for a different IP must be ignored, not crash or corrupt RX
        var rsrBefore = ReadU16(w, 0, 0x0026);
        w.ReceiveFrame(MakeFrame(BuildUdpFrame(myMac, new byte[] { 10, 0, 0, 9 }, HostMac, HostIp, 8888, 244, new byte[] { 1, 2, 3 })));
        check(ReadU16(w, 0, 0x0026) == rsrBefore, "eth: frame for a foreign IP is ignored");
    }

    private static EthernetFrame MakeFrame(byte[] raw)
    {
        EthernetFrame f;
        EthernetFrame.TryCreateEthernetFrame(raw, CRCMode.Add, out f);
        return f;
    }

    private static byte[] BuildArp(byte[] srcMac, byte[] srcIp, byte[] dstMac, byte[] dstIp,
                                   byte[] targetHardwareAddress, ushort oper)
    {
        var b = new byte[42];
        Array.Copy(dstMac, 0, b, 0, 6);
        Array.Copy(srcMac, 0, b, 6, 6);
        b[12] = 0x08; b[13] = 0x06;
        b[14] = 0x00; b[15] = 0x01; b[16] = 0x08; b[17] = 0x00; b[18] = 0x06; b[19] = 0x04;
        b[20] = (byte)(oper >> 8); b[21] = (byte)(oper & 0xFF);
        Array.Copy(srcMac, 0, b, 22, 6);
        Array.Copy(srcIp, 0, b, 28, 4);
        Array.Copy(targetHardwareAddress, 0, b, 32, 6);
        Array.Copy(dstIp, 0, b, 38, 4);
        return b;
    }

    private static readonly byte[] Broadcast = new byte[] { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    private static readonly byte[] ZeroMac = new byte[6];

    private static byte[] BuildArpRequest(byte[] srcMac, byte[] srcIp, byte[] dstMac, byte[] dstIp)
        => BuildArp(srcMac, srcIp, Broadcast, dstIp, ZeroMac, 1);

    // Sent BY the agent: "peerIp lives at peerMac".
    private static byte[] BuildArpReply(byte[] peerMac, byte[] peerIp, byte[] requesterMac, byte[] requesterIp)
        => BuildArp(peerMac, peerIp, Broadcast, requesterIp, requesterMac, 2);

    private static byte[] BuildUdpFrame(byte[] dstMac, byte[] dstIp, byte[] srcMac, byte[] srcIp,
                                       ushort srcPort, ushort dstPort, byte[] payload)
    {
        var frame = new byte[EthHeaderLen + Ipv4HeaderLen + 8 + payload.Length];
        Array.Copy(dstMac, 0, frame, 0, 6);
        Array.Copy(srcMac, 0, frame, 6, 6);
        frame[12] = 0x08; frame[13] = 0x00;
        var ip = EthHeaderLen;
        frame[ip + 0] = 0x45;
        var ipLen = Ipv4HeaderLen + 8 + payload.Length;
        frame[ip + 2] = (byte)(ipLen >> 8); frame[ip + 3] = (byte)(ipLen & 0xFF);
        frame[ip + 6] = 0x40; frame[ip + 8] = 64; frame[ip + 9] = 17;
        Array.Copy(srcIp, 0, frame, ip + 12, 4);
        Array.Copy(dstIp, 0, frame, ip + 16, 4);
        frame[ip + 10] = (byte)(Checksum(frame, ip, Ipv4HeaderLen) >> 8);
        frame[ip + 11] = (byte)(Checksum(frame, ip, Ipv4HeaderLen) & 0xFF);
        var udp = ip + Ipv4HeaderLen;
        frame[udp + 0] = (byte)(srcPort >> 8); frame[udp + 1] = (byte)(srcPort & 0xFF);
        frame[udp + 2] = (byte)(dstPort >> 8); frame[udp + 3] = (byte)(dstPort & 0xFF);
        var ul = 8 + payload.Length;
        frame[udp + 4] = (byte)(ul >> 8); frame[udp + 5] = (byte)(ul & 0xFF);
        Array.Copy(payload, 0, frame, udp + 8, payload.Length);
        return frame;
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

    private static bool IpChecksumOk(byte[] frame, int offset, int length)
    {
        // The frame the model emitted includes an Ethernet CRC at the end; ignore it.
        uint sum = 0;
        for (var i = 0; i + 1 < length; i += 2)
        {
            sum += (uint)((frame[offset + i] << 8) | frame[offset + i + 1]);
        }
        while ((sum >> 16) != 0)
        {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
        return (ushort)sum == 0xFFFF;
    }
}