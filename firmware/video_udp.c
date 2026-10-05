#include "video_udp.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "socket.h"

#ifndef NCAP_OV7670
#define NCAP_OV7670 0
#endif
// NCAP_DVP=1 streams real sensor pixels captured over the DVP parallel bus
// through DCMI. NCAP_OV7670's SCCB "test register" readout is a Renode-only
// fiction (those addresses are absent from the OV7670 register map) and is
// never used here.
#ifndef NCAP_DVP
#define NCAP_DVP 0
#endif
#if NCAP_OV7670
#include "ov7670.h"
#endif
#if NCAP_DVP
#include "ov7670_dvp.h"
// DCMI crop can shorten the captured window, so the frame height the stream
// advertises and iterates over is not necessarily the sensor's output height.
#define VIDEO_H  ((uint32_t)DVP_STREAM_H)
#else
#define VIDEO_H  ((uint32_t)VIDEO_HEIGHT)
#endif

// ---- tunables (all overridable from the Makefile) --------------------------

#ifndef VIDEO_WIDTH
#define VIDEO_WIDTH     320u
#endif
#ifndef VIDEO_HEIGHT
#define VIDEO_HEIGHT    240u
#endif
#ifndef VIDEO_HOST_IP
#define VIDEO_HOST_IP   "127.0.0.1"
#endif
#ifndef VIDEO_PORT
#define VIDEO_PORT      18080u
#endif
#ifndef VIDEO_SRC_PORT
#define VIDEO_SRC_PORT  5000u
#endif
#ifndef VIDEO_FPS
#define VIDEO_FPS        24u
#endif
// Deprecated: VIDEO_FPS_MS used to mean "idle delay after a frame", which
// made the real frame period send_time + VIDEO_FPS_MS. A target rate in
// VIDEO_FPS is now used instead, so the send cost cannot silently eat the
// budget. Kept only so stale Makefile overrides still compile.
#ifndef VIDEO_FPS_MS
#define VIDEO_FPS_MS    200u
#endif
// Per-frame printf costs ~3.5 ms at 115200 baud, which is 8% of a 41 ms
// period. Log at most this often.
#ifndef VIDEO_LOG_EVERY
#define VIDEO_LOG_EVERY  30u
#endif
// Report the achieved frame rate once per simulated second. Useful on target
// and in Renode, where the host-observed rate is bounded by emulation speed
// rather than by the firmware.
#ifndef VIDEO_FPS_REPORT
#define VIDEO_FPS_REPORT 0
#endif
// Socket TX buffer in KB (valid: 1,2,4,8,16). At 2 KB only one 1300 B
// datagram fits, so sendto() blocks in the Sn_TX_FSR spin for the W5500 to
// drain and SPI and Ethernet serialise instead of overlapping.
#ifndef VIDEO_TX_BUF_KB
#define VIDEO_TX_BUF_KB  4u
#endif
#ifndef VIDEO_SOCK
#define VIDEO_SOCK      1u
#endif

// Sn_SR socket states (W5500 datasheet). ioLibrary's headers in this tree do
// not export them. A UDP socket reports S_UDP (0x22) once open - NOT S_INIT,
// which is only used by TCP.
#define V_SOCK_UDP    0x22u

// Keep each datagram comfortably inside the W5500 MTU.
#define VIDEO_MTU_BUDGET   1400u

#define HDR_LEN            20u
#define META_MAGIC0        'V'
#define META_MAGIC1        'F'
#define META_MAGIC2        'M'
#define META_MAGIC3        '0'
#define FRAME_MAGIC0       'V'
#define FRAME_MAGIC1       'F'
#define FRAME_MAGIC2       'R'
#define FRAME_MAGIC3       '0'

// One row block must fit: hdr + rows*width <= MTU_BUDGET.
#define ROWS_PER_DGRAM  ((uint32_t)((VIDEO_MTU_BUDGET - HDR_LEN) / VIDEO_WIDTH) > 0u \
                          ? (uint32_t)((VIDEO_MTU_BUDGET - HDR_LEN) / VIDEO_WIDTH) : 1u)
#define MAX_BLOCK_BYTES  (uint32_t)(ROWS_PER_DGRAM * VIDEO_WIDTH)

static uint8_t vhost_ip[4];
static uint8_t v_hdr[HDR_LEN];
static uint8_t v_block[MAX_BLOCK_BYTES];
static uint8_t v_dgram[HDR_LEN + MAX_BLOCK_BYTES];
static uint32_t v_seq;
static uint32_t v_row;
static uint32_t v_dgrams;
#if NCAP_OV7670
static bool v_sensor;
#endif
#if NCAP_DVP
// Luma frame currently being drained, plus the row within it.
static const uint8_t *v_frame;
#endif

static void put_u16_be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static void put_u32_be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static bool parse_ip(const char *s, uint8_t out[4])
{
    if (s == NULL)
    {
        return false;
    }
    for (int i = 0; i < 4; i++)
    {
        unsigned val = 0;
        int digits = 0;
        while (*s >= '0' && *s <= '9')
        {
            val = val * 10u + (unsigned)(*s - '0');
            s++;
            digits++;
        }
        if (digits == 0 || val > 255u)
        {
            return false;
        }
        out[i] = (uint8_t)val;
        if (i < 3)
        {
            if (*s != '.')
            {
                return false;
            }
            s++;
        }
    }
    return *s == '\0';
}

// One synthetic luma row: vertical ramp, a bright row band that walks down the
// frame, and a bright column bar that sweeps left to right.
#if !NCAP_DVP
static void synth_line(uint8_t *line, uint32_t seq, uint32_t y)
{
    const uint32_t bar_x = (seq * 8u) % VIDEO_WIDTH;
    const uint32_t band_y = (seq * 5u) % VIDEO_HEIGHT;

    for (uint32_t x = 0; x < VIDEO_WIDTH; x++)
    {
        uint32_t v = (y * 255u) / VIDEO_HEIGHT;
        if (y >= band_y && y < band_y + 12u)
        {
            v = (v + 90u) & 0xFFu;
        }
        const uint32_t dx = (x > bar_x) ? (x - bar_x) : (bar_x - x);
        if (dx < 10u)
        {
            v = 255u;
        }
        line[x] = (uint8_t)v;
    }
}
#endif

void video_udp_init(void)
{
    if (!parse_ip(VIDEO_HOST_IP, vhost_ip))
    {
        printf("F4: video bad host ip\r\n");
        while (1) { }
    }
    printf("F4: video udp sock=%u %ux%u mono8 -> %d.%d.%d.%d:%u (src %u, %u rows/dgram)\r\n",
           (unsigned)VIDEO_SOCK, (unsigned)VIDEO_WIDTH, (unsigned)VIDEO_H,
           vhost_ip[0], vhost_ip[1], vhost_ip[2], vhost_ip[3],
           (unsigned)VIDEO_PORT, (unsigned)VIDEO_SRC_PORT, (unsigned)ROWS_PER_DGRAM);

    // Must be programmed while the socket is still closed: it sizes the
    // socket's slice of the W5500's shared 16 KB pool.
    setSn_TXBUF_SIZE(VIDEO_SOCK, (uint8_t)VIDEO_TX_BUF_KB);
    setSn_RXBUF_SIZE(VIDEO_SOCK, 1u);

    int8_t sn = socket(VIDEO_SOCK, Sn_MR_UDP, (uint16_t)VIDEO_SRC_PORT, 0);
    if (sn != (int8_t)VIDEO_SOCK)
    {
        printf("F4: video socket failed\r\n");
        while (1) { }
    }
    uint32_t guard = 0;
    while (getSn_SR(VIDEO_SOCK) != V_SOCK_UDP)
    {
        HAL_Delay(1);
        if (++guard > 5000u)
        {
            printf("F4: video SOCK_UDP timeout sr=%02X\r\n", getSn_SR(VIDEO_SOCK));
            while (1) { }
        }
    }

    // sendto() writes Sn_DIPR itself from the address argument, so there is no
    // destination IP to program here.

    // Meta datagram carrying an HTTP-style preamble, kept for parity with the
    // earlier TCP design and for human inspection of the stream parameters.
    {
        static uint8_t meta[128];
        int mn = snprintf((char *)meta, sizeof(meta),
                          "HTTP/1.1 200 OK\r\n"
                          "Content-Type: application/octet-stream\r\n"
                          "X-Frame-Width: %u\r\n"
                          "X-Frame-Height: %u\r\n"
                          "X-Encoding: mono8\r\n"
                          "\r\n",
                          (unsigned)VIDEO_WIDTH, (unsigned)VIDEO_HEIGHT);
        if (mn < 0)
        {
            mn = 0;
        }
        v_block[0] = META_MAGIC0;
        v_block[1] = META_MAGIC1;
        v_block[2] = META_MAGIC2;
        v_block[3] = META_MAGIC3;
        memcpy(&v_block[4], meta, (uint32_t)mn);
        sendto(VIDEO_SOCK, v_block, (uint16_t)(4u + mn), vhost_ip, (uint16_t)VIDEO_PORT);
        printf("F4: video meta sent (%d B)\r\n", mn + 4);
    }

    v_hdr[0] = FRAME_MAGIC0;
    v_hdr[1] = FRAME_MAGIC1;
    v_hdr[2] = FRAME_MAGIC2;
    v_hdr[3] = FRAME_MAGIC3;
    put_u16_be(&v_hdr[8], (uint16_t)VIDEO_WIDTH);
    put_u16_be(&v_hdr[10], (uint16_t)VIDEO_H);
    v_seq = 0;
    v_row = 0;
    v_dgrams = 0;
#if NCAP_DVP
    v_frame = NULL;
    if (ov7670_dvp_init())
    {
        printf("F4: video source: ov7670 over DVP/DCMI\r\n");
    }
    else
    {
        printf("F4: dvp init FAILED, falling back to synthetic\r\n");
    }
#elif NCAP_OV7670
    v_sensor = ov7670_init();
    printf("F4: video source: %s\r\n", v_sensor ? "ov7670 (SCCB fiction)" : "synthetic (sensor missing)");
#endif
}

// Emits one row block. Returns true when a frame completes.
bool video_udp_step(void)
{
#if NCAP_DVP
    // Fetch a new capture ONLY at a frame boundary, i.e. once the previous
    // frame has been fully drained. Calling take_frame() on every step let
    // write_slot wrap the ring back onto the slot currently being streamed
    // while the DMA was overwriting it, which is a race between the capture
    // engine and the transmitter. Serialising the two removes the hazard.
    if (v_row == 0)
    {
        const uint8_t *fresh;
        if (!ov7670_dvp_take_frame(&fresh))
        {
            // Nothing captured yet: not a frame boundary, caller keeps spinning.
            return false;
        }
        v_frame = fresh;
        v_seq++;
        put_u32_be(&v_hdr[4], v_seq);
    }

    uint32_t rows = VIDEO_H - v_row;
    if (rows > ROWS_PER_DGRAM)
    {
        rows = ROWS_PER_DGRAM;
    }
    uint32_t off = rows * VIDEO_WIDTH;
    memcpy(v_block, v_frame + ((size_t)v_row * VIDEO_WIDTH), (size_t)off);
#else
    if (v_row == 0)
    {
        v_seq++;
        put_u32_be(&v_hdr[4], v_seq);
    }

    uint32_t rows = VIDEO_H - v_row;
    if (rows > ROWS_PER_DGRAM)
    {
        rows = ROWS_PER_DGRAM;
    }

    uint32_t off = 0;
#if NCAP_OV7670
    if (v_sensor)
    {
        if (!ov7670_read_luma_rows((uint16_t)v_row, (uint8_t)rows, v_block, VIDEO_WIDTH))
        {
            printf("F4: video sensor fetch failed, falling back to synth\r\n");
            v_sensor = false;
        }
        else
        {
            off = rows * VIDEO_WIDTH;
        }
    }
    if (!v_sensor)
#endif
    {
        for (uint32_t r = 0; r < rows; r++)
        {
            synth_line(&v_block[off], v_seq, v_row + r);
            off += VIDEO_WIDTH;
        }
    }
#endif

    put_u16_be(&v_hdr[12], (uint16_t)v_row);
    put_u16_be(&v_hdr[14], (uint16_t)rows);
    put_u32_be(&v_hdr[16], off);

    // Header and payload travel together in ONE datagram so the receiver
    // needs no pairing logic and a lost datagram loses whole row blocks.
    memcpy(v_dgram, v_hdr, HDR_LEN);
    memcpy(v_dgram + HDR_LEN, v_block, off);
    sendto(VIDEO_SOCK, v_dgram, (uint16_t)(HDR_LEN + off), vhost_ip, (uint16_t)VIDEO_PORT);
    v_dgrams += 1;

    v_row += rows;
    if (v_row >= VIDEO_H)
    {
        v_row = 0;
        if (VIDEO_LOG_EVERY && (v_seq % (uint32_t)VIDEO_LOG_EVERY) == 0u)
        {
            printf("F4: video frame %u sent (%u dgrams total)\r\n",
                   (unsigned)v_seq, (unsigned)v_dgrams);
        }
        return true;
    }
    return false;
}

// Deadline-paced: VIDEO_FPS is a real target rate, not an idle delay.
// Accumulating 1000 and draining whole VIDEO_FPS units gives an average
// period of 1000/VIDEO_FPS ms (41/42 ms alternating at 24 fps) without
// integer-division drift. If a frame overruns its deadline the debt is
// dropped rather than accumulated, so a late frame cannot cascade.
void video_udp_serve(void)
{
    video_udp_init();
    uint32_t acc = 0u;
    uint32_t next = HAL_GetTick();
#if VIDEO_FPS_REPORT
    uint32_t rep_frames = 0u;
    uint32_t rep_start = next;
#endif
    for (;;)
    {
        while (!video_udp_step()) { }

#if VIDEO_FPS_REPORT
        rep_frames++;
        {
            uint32_t now = HAL_GetTick() - rep_start;
            if (now >= 1000u)
            {
                printf("F4: video rate %u.%02u fps (%u frames / %u ms)\r\n",
                       (unsigned)(rep_frames * 1000u / now),
                       (unsigned)((rep_frames * 1000u % now) * 100u / now),
                       (unsigned)rep_frames, (unsigned)now);
                rep_frames = 0u;
                rep_start += now;
            }
        }
#endif

        acc += 1000u;
        uint32_t period = 0u;
        while (acc >= (uint32_t)VIDEO_FPS)
        {
            acc -= (uint32_t)VIDEO_FPS;
            period++;
        }
        next += period;
        uint32_t now = HAL_GetTick();
        int32_t slack = (int32_t)(next - now);
        if (slack > 0)
        {
            HAL_Delay((uint32_t)slack);
        }
        else
        {
            next = now;
        }
    }
}
