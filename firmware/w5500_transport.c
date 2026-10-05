// micro-ROS custom transport over the WIZnet W5500 (SPI hardwired TCP/IP).
// Bare-metal build: no RTOS, so read() polls RX_RSR against a HAL_GetTick deadline.
// Under NCAP_FREERTOS=1 the poll yields through ncap_cpu_relax() instead - see
// the comment on that hook below. The file itself stays RTOS-agnostic.

#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>

#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "socket.h"
#include "wizchip_conf.h"

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

#define W5500_TRANSPORT_SOCK   0
// Client source port. MUST differ from W5500_AGENT_PORT when the agent is
// reached over the same host: the model's host socket binds this port, so an
// identical agent port makes the socket deliver our own datagram back to us
// instead of reaching the agent (self-send). In production the client and the
// agent are different hosts, so both may use 8888.
#ifndef W5500_TRANSPORT_PORT
#define W5500_TRANSPORT_PORT   5000
#endif
#ifndef W5500_AGENT_PORT
#define W5500_AGENT_PORT       8888
#endif
#define W5500_MTU              1472

static uint8_t w5500_agent_ip[4] = {127, 0, 0, 1};
static bool w5500_opened = false;

// Cooperative yield point for the RX busy-poll below.
//
// The poll spins on getSn_RX_RSR() with no blocking call, so on bare metal
// (NCAP_FREERTOS=0) it must spin: there is nothing else to run. Under
// NCAP_FREERTOS=1 the very same spin inside the executor task would starve
// every lower-priority task - including the idle task the CPU-load metric is
// derived from, which would then read a permanent 100% load - so main.c
// redefines this as vTaskDelay(1).
//
// vTaskDelay and not taskYIELD: taskYIELD only lets a task of equal or higher
// priority run, so the sampler and the idle task would still be locked out.
// vTaskDelay blocks the calling task, which is what actually reopens the CPU.
//
// Weak by default: the no-op below keeps this file linkable on its own and
// keeps the bare-metal personality byte-for-byte identical.
__attribute__((weak)) void ncap_cpu_relax(void) { }

// --- hex-dump tracing of every XRCE datagram (build with -DW5500_TDEBUG=1) ---
// Off by default: a running session emits several datagrams per second and the
// UART cannot keep up, which would itself perturb timing.
#ifndef W5500_TDEBUG
#define W5500_TDEBUG 0
#endif

#if W5500_TDEBUG
#define W5500_DBG_MAX      48
#define W5500_DBG_MAXB     80

static uint32_t w5500_dbg_tx = 0;
static uint32_t w5500_dbg_rx = 0;
static uint32_t w5500_dbg_rx_empty = 0;

static void w5500_dbg_hex(const char *tag, const uint8_t *buf, size_t len)
{
    printf("%s len=%u:", tag, (unsigned)len);
    for (size_t i = 0; i < len && i < W5500_DBG_MAXB; i++)
    {
        printf(" %02X", buf[i]);
    }
    if (len > W5500_DBG_MAXB)
    {
        printf(" ...");
    }
    printf("\r\n");
}
#else
#define w5500_dbg_hex(tag, buf, len)   do { } while (0)
#endif

static bool w5500_parse_ip(const char *s, uint8_t out[4])
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

bool cubemx_transport_open_w5500(struct uxrCustomTransport *transport)
{
    if (!w5500_parse_ip((const char *)transport->args, w5500_agent_ip))
    {
        printf("F4: w5500-open-bad-ip\r\n");
        return false;
    }
    if (w5500_opened)
    {
        close(W5500_TRANSPORT_SOCK);
        w5500_opened = false;
    }
    int8_t sn = socket(W5500_TRANSPORT_SOCK, Sn_MR_UDP, W5500_TRANSPORT_PORT, 0);
    if (sn != W5500_TRANSPORT_SOCK)
    {
        printf("F4: w5500-socket-failed\r\n");
        return false;
    }
    w5500_opened = true;
    printf("F4: transport-open agent=%d.%d.%d.%d:%d\r\n",
           w5500_agent_ip[0], w5500_agent_ip[1], w5500_agent_ip[2], w5500_agent_ip[3],
           W5500_AGENT_PORT);
    return true;
}

bool cubemx_transport_close_w5500(struct uxrCustomTransport *transport)
{
    (void)transport;
    if (w5500_opened)
    {
        close(W5500_TRANSPORT_SOCK);
        w5500_opened = false;
    }
    return true;
}

size_t cubemx_transport_write_w5500(struct uxrCustomTransport *transport,
                                    const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;
    if (!w5500_opened)
    {
        if (err) *err = 1;
        return 0;
    }
    if (len > W5500_MTU)
    {
        if (err) *err = 1;
        return 0;
    }
#if W5500_TDEBUG
    if (w5500_dbg_tx < W5500_DBG_MAX)
    {
        w5500_dbg_tx++;
        printf("F4: tx#%u\r\n", (unsigned)w5500_dbg_tx);
        w5500_dbg_hex("F4: TX", (const uint8_t *)buf, len);
    }
#endif
    int32_t ret = sendto(W5500_TRANSPORT_SOCK, (uint8_t *)buf, (uint16_t)len,
                         w5500_agent_ip, W5500_AGENT_PORT);
    if (ret <= 0)
    {
        printf("F4: w5500-sendto-failed %ld\r\n", (long)ret);
        if (err) *err = 1;
        return 0;
    }
    if (err) *err = 0;
    return (size_t)ret;
}

size_t cubemx_transport_read_w5500(struct uxrCustomTransport *transport, uint8_t *buf,
                                   size_t len, int timeout, uint8_t *err)
{
    (void)transport;
    if (!w5500_opened)
    {
        if (err) *err = 1;
        return 0;
    }
    uint32_t start = HAL_GetTick();
    uint32_t spins = 0;
    while (getSn_RX_RSR(W5500_TRANSPORT_SOCK) == 0)
    {
        spins++;
        ncap_cpu_relax();
        if ((int32_t)(HAL_GetTick() - start) >= timeout)
        {
#if W5500_TDEBUG
            if (w5500_dbg_rx_empty < W5500_DBG_MAX)
            {
                w5500_dbg_rx_empty++;
                printf("F4: rx#%u EMPTY timeout=%d cap=%u spins=%u rsr=%u\r\n",
                       (unsigned)w5500_dbg_rx_empty, timeout, (unsigned)len,
                       (unsigned)spins,
                       (unsigned)getSn_RX_RSR(W5500_TRANSPORT_SOCK));
            }
#endif
            if (err) *err = 0;   /* timeout is not an error */
            return 0;
        }
    }
    uint8_t peer_ip[4];
    uint16_t peer_port = 0;
#if W5500_TDEBUG
    bool dbg_rx_call = (w5500_dbg_rx < W5500_DBG_MAX);
    if (dbg_rx_call)
    {
        w5500_dbg_rx++;
        printf("F4: rx#%u DATA timeout=%d cap=%u spins=%u rsr=%u\r\n",
               (unsigned)w5500_dbg_rx, timeout, (unsigned)len, (unsigned)spins,
               (unsigned)getSn_RX_RSR(W5500_TRANSPORT_SOCK));
    }
#endif
    int32_t ret = recvfrom(W5500_TRANSPORT_SOCK, buf,
                           (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len), peer_ip, &peer_port);
    if (ret < 0)
    {
        printf("F4: w5500-recvfrom-failed\r\n");
        if (err) *err = 1;
        return 0;
    }
#if W5500_TDEBUG
    if (dbg_rx_call)
    {
        printf("F4: rx#%u from %d.%d.%d.%d:%d\r\n", (unsigned)w5500_dbg_rx,
               peer_ip[0], peer_ip[1], peer_ip[2], peer_ip[3], (int)peer_port);
        w5500_dbg_hex("F4: RX", (const uint8_t *)buf, (size_t)ret);
    }
#endif
    if (err) *err = 0;
    return (size_t)ret;
}

#endif
