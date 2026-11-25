// NCAP STM32F4 + W5500 — micro-ROS client (bare metal, custom UDP transport).
// Session runs against the micro-ROS agent (Docker, UDP :8888) through the
// W5500 model in Renode.
//
// Uses the API of the bundled libmicroros.a:
//   rmw_uros_options_set_custom_transport() + rclc_support_init_with_options()
//   rclc_publisher_init_default() / rclc_executor_spin()

#include "main.h"

#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/node.h>
#include <rclc/publisher.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw/rmw.h>
#include <rmw_microros/custom_transport.h>
#include <rmw_microros/init_options.h>
#include <rcl/init_options.h>
#include <uxr/client/client.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

const rosidl_message_type_support_t *
rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__std_msgs__msg__Int32(void);
const rosidl_message_type_support_t *
rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__sensor_msgs__msg__Imu(void);
// Present in the prebuilt libmicroros.a (verified with nm), declared the same
// way as the two above. Only needed when the sysusage module is compiled in;
// sysusage.h has its own copy, this keeps the set visible in one place.
const rosidl_message_type_support_t *
rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticArray(void);

#include "wizchip_conf.h"
#include "socket.h"

#include "i2c_bus.h"
#include "imu.h"
#include "ov7670.h"

#include "video_udp.h"

#include "sysusage.h"

// 1 = stream synthetic grayscale luma over UDP (see video_udp.c)
// 0 = micro-ROS client publishing std_msgs/Int32 (default)
#ifndef NCAP_VIDEO_UDP
#define NCAP_VIDEO_UDP 0
#endif

// 1 = build the FreeRTOS kernel and hand the main loop to vTaskStartScheduler().
// 0 = the historic bare-metal super-loop (default).
#ifndef NCAP_FREERTOS
#define NCAP_FREERTOS 0
#endif

#if NCAP_FREERTOS == 1
#include "FreeRTOS.h"
#include "task.h"

/* Turn the W5500 RX busy-poll into a cooperative yield point. See the
   ncap_cpu_relax comment in w5500_transport.c: without this the executor task
   spins on getSn_RX_RSR() and locks out every lower-priority task, which pins
   the idle-tick counter at zero and makes cpu_load_pct read a permanent 100%.
   vTaskDelay rather than taskYIELD because only vTaskDelay reopens the CPU to
   strictly lower priorities (the sampler and the idle task). */
/* Strong, deliberately: the transport's fallback is __attribute__((weak)), and
   two weak definitions would be resolved by link order alone - i.e. by luck.
   Strong here makes "the scheduler yields" the guaranteed winner whenever
   NCAP_FREERTOS=1, and lets the transport keep its weak no-op for bare metal. */
void ncap_cpu_relax(void)
{
    vTaskDelay(1);
}
#endif

extern UART_HandleTypeDef huart2;

uint8_t w5500_version(void);
void w5500_hw_init(const uint8_t mac[6], const uint8_t ip[4],
                   const uint8_t mask[4], const uint8_t gw[4]);

bool cubemx_transport_open_w5500(struct uxrCustomTransport *transport);
bool cubemx_transport_close_w5500(struct uxrCustomTransport *transport);
size_t cubemx_transport_write_w5500(struct uxrCustomTransport *transport,
                                    const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read_w5500(struct uxrCustomTransport *transport, uint8_t *buf,
                                   size_t len, int timeout, uint8_t *err);

// Agent address. Must be the published host port (127.0.0.1:8888): Docker
// Desktop container IPs are inside the VM and not routable from macOS.
// The client source port (CLIENT_PORT) must be fixed so docker-proxy can
// deliver the agent's reply back to this socket.
// Host LAN address running the micro-ROS agent (published 8888/udp).
// Must NOT be 127.0.0.1 when the client port is also 8888: the model's host
// socket would then send to itself. Override at build time:
//   make -C firmware MICRO_AGENT_IP=192.168.0.213
#ifndef MICRO_AGENT_IP
#define MICRO_AGENT_IP "192.168.0.213"
#endif
#define AGENT_IP   MICRO_AGENT_IP
#define TOPIC      "heartbeat"
// 0 = legacy single-node personality (exact historic names/ports/keys).
// >0 = multi-node personality: suffixed names/topics and a unique client key.
#ifndef NODE_ID
#define NODE_ID 0
#endif
#define PUB_PERIOD 1000u

#define ROS_TOPIC_TYPE  rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__std_msgs__msg__Int32()
#define ROS_QOS         rmw_qos_profile_default

static void uart_puts(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 300);
}

static void uart_put_u32(uint32_t v)
{
    char b[12];
    int i = 0;
    if (v == 0) { b[i++] = '0'; }
    while (v != 0 && i < 10) { b[i++] = (char)('0' + (v % 10u)); v /= 10u; }
    for (int j = i - 1; j >= 0; j--)
    {
        char c = b[j];
        HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 100);
    }
}

/* ---- static memory pool for the bare-metal RMW ---- */
#define POOL_SIZE (40 * 1024)
static uint8_t pool[POOL_SIZE] __attribute__((aligned(8)));
static size_t pool_used;

static void *pool_alloc(size_t size, void *state)
{
    (void)state;
    size = (size + 7u) & ~((size_t)7u);
    if (pool_used + size > POOL_SIZE)
    {
        uart_puts("pool-exhausted\r\n");
        return NULL;
    }
    void *p = &pool[pool_used];
    pool_used += size;
    return p;
}
static void pool_deallocate(void *pointer, void *state) { (void)pointer; (void)state; }
static void *pool_reallocate(void *pointer, size_t size, void *state)
{
    (void)pointer;
    return pool_alloc(size, state);
}
static void *pool_zero_alloc(size_t n, size_t size, void *state)
{
    void *p = pool_alloc(n * size, state);
    if (p != NULL) { memset(p, 0, n * size); }
    return p;
}

static rcl_allocator_t allocator;

static rclc_support_t support;
static rclc_executor_t executor;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_publisher_t imu_pub;
static rcl_timer_t timer;
static std_msgs__msg__Int32 msg;
static sensor_msgs__msg__Imu imu_msg;
static uint32_t counter;
static bool imu_ready;
#if NCAP_SYSUSAGE == 1
static bool sysusage_publish_failed;
#endif

#if NCAP_FREERTOS == 1
/* ---- FreeRTOS application callbacks -------------------------------------
   configSUPPORT_STATIC_ALLOCATION is 1, so the kernel asks the application -
   not the heap - for the idle task memory, and configUSE_IDLE_HOOK is 1 so it
   also demands vApplicationIdleHook(). Nothing in this tree supplies them:
   cmsis_os2.c, which carries weak fallbacks, is deliberately excluded from SRC,
   and Core/Src/freertos.c is dead CubeMX robot code that is not built. Without
   the two functions below the NCAP_FREERTOS=1 link dies with
   "undefined reference to vApplicationGetIdleTaskMemory / vApplicationIdleHook".

   vApplicationGetTimerTaskMemory is deliberately NOT here: configUSE_TIMERS is
   0 (nothing calls xTimerCreate, and the daemon cost ~1356 B of .bss for no
   gain), so timers.c never references it. Re-adding it is the first thing to do
   if configUSE_TIMERS ever goes back to 1.

   Static, not heap: the F446 has 128 KB of RAM of which ~70 KB is already
   data+bss in the full-feature build, and sysusage's own task is static too, so
   nothing here draws on configTOTAL_HEAP_SIZE. */
static StaticTask_t idle_task_tcb;
static StackType_t  idle_task_stack[configMINIMAL_STACK_SIZE];

/* Idle-tick counter, the CPU-load primitive the plan calls for: sample it and
   uxTaskGetTickCount() from any thread and
       idle_pct  = 100 * (idle_now - idle_then) / (ticks_now - ticks_then)
       load_pct  = 100 - idle_pct
   Deliberately non-static and volatile: sysusage.c samples it from its own task
   and must not be able to optimise the read away. Written only by the idle
   hook below, so the delta is the one-word increment and needs no lock. */
volatile uint32_t ncap_idle_ticks;

/* Weak on purpose: sysusage.c also defines vApplicationIdleHook, because the
   idle-tick counter is the CPU-load primitive its sampler reads - and it must
   keep a reference to the symbol so --gc-sections does not discard it. Both
   definitions are genuinely required:

     - sysusage.c is absent from the NCAP_SYSUSAGE=0 NCAP_FREERTOS=1 row, so
       without the weak copy below that row cannot link at all;
     - with both rows on, sysusage.c's strong definition is what the linker
       keeps and the weak copy is discarded silently.

   Weak (rather than deleting either side) is what lets all four matrix rows
   link. Same pattern as the __WEAK vApplicationIdleHook in
   CMSIS_RTOS_V2/cmsis_os2.c, which this build excludes on purpose. */
__attribute__((weak)) void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, uint32_t *size)
{
    *tcb = &idle_task_tcb;
    *stack = idle_task_stack;
    *size = (uint32_t)configMINIMAL_STACK_SIZE;
}

/* MUST stay trivial: it runs at idle-task priority with scheduler locks held, so
   anything slow here is invisible CPU time that the load metric reports as busy.
   One increment, no printf, no HAL, no rclc. Note that in the full-feature build
   sysusage.c's strong definition is the one that actually runs; this fallback
   exists only to keep the scheduler linkable when sysusage is compiled out. */
__attribute__((weak)) void vApplicationIdleHook(void)
{
    ncap_idle_ticks++;
}

/* ---- the rclc executor task ----------------------------------------------
   Under NCAP_FREERTOS the scheduler owns the loop, so something has to spin the
   executor or the 1 Hz timer_callback() never fires and *nothing* is published -
   not the heartbeat, not the IMU, not /diagnostics. This task is that something.

   Only this task ever calls rclc. A second executor on the same XRCE client
   would interleave datagrams on the one UDP session the W5500 socket owns and
   corrupt it, which is exactly why sysusage.c samples from its own task and
   hands the result over instead of publishing itself. See docs/plan-sysusage.md.

   Bounded spin_some() rather than the blocking spin(): rclc_executor_spin()
   never returns, and even one trip through it parks the task inside the
   transport's RX deadline loop for the whole timeout. The vTaskDelay(1) between
   slices guarantees a blocking point per tick even if the transport read
   returns immediately. */
#define RCL_TASK_PRIO        2
#define RCL_TASK_STACK       (256u)
#define RCL_SLICE_NS         (5ULL * 1000ULL * 1000ULL)

static StaticTask_t rcl_task_tcb;
static StackType_t  rcl_task_stack[RCL_TASK_STACK];

static void rcl_task(void *arg)
{
    (void)arg;
    for (;;)
    {
        rclc_executor_spin_some(&executor, RCL_SLICE_NS);
        vTaskDelay(1);
    }
}

static void rcl_task_start(void)
{
    /* V10.3.1 order is (fn, name, depth, params, prio, stack, tcb) - NOT the
       CubeMX/CMSIS-v2 order, and it returns a TaskHandle_t, not a BaseType_t. */
    (void)xTaskCreateStatic(rcl_task, "rcl", RCL_TASK_STACK, NULL,
                            RCL_TASK_PRIO, rcl_task_stack, &rcl_task_tcb);
}
#endif /* NCAP_FREERTOS == 1 */

static void timer_callback(rcl_timer_t *tm, int64_t ts)
{
    (void)tm; (void)ts;
    msg.data = (int32_t)++counter;
    if (rcl_publish(&publisher, &msg, NULL) == RCL_RET_OK)
    {
        uart_puts("publish ");
        uart_put_u32(counter);
        uart_puts(" ok\r\n");
    }
    if (imu_ready)
    {
        imu_sample_t smp;
        if (imu_read(&smp))
        {
            imu_msg.angular_velocity.x = smp.gx;
            imu_msg.angular_velocity.y = smp.gy;
            imu_msg.angular_velocity.z = smp.gz;
            imu_msg.linear_acceleration.x = smp.ax;
            imu_msg.linear_acceleration.y = smp.ay;
            imu_msg.linear_acceleration.z = smp.az;
            if (rcl_publish(&imu_pub, &imu_msg, NULL) == RCL_RET_OK)
            {
                uart_puts("imu publish ok\r\n");
                if (1)
                {
                    // nano.specs has no %f; print milli-units as integers.
                    printf("F4: imu a=(%d,%d,%d) ug g=(%d,%d,%d) mdps\r\n",
                           (int)(smp.ax * 1000000.0f / 9.80665f),
                           (int)(smp.ay * 1000000.0f / 9.80665f),
                           (int)(smp.az * 1000000.0f / 9.80665f),
                           (int)(smp.gx * 1000.0f * 180.0f / 3.14159265f),
                           (int)(smp.gy * 1000.0f * 180.0f / 3.14159265f),
                           (int)(smp.gz * 1000.0f * 180.0f / 3.14159265f));
                }
            }
            else
            {
                uart_puts("imu publish err\r\n");
            }
        }
        else
        {
            uart_puts("imu read err\r\n");
        }
    }
    else
    {
        uart_puts("publish err\r\n");
    }

#if NCAP_SYSUSAGE == 1
    /* Telemetry rides the heartbeat on purpose: timer_callback() is driven by
       the 1 Hz rcl_timer (PUB_PERIOD), so this fires exactly once per heartbeat
       with no extra rate limiting - a second decimation would only desynchronise
       /diagnostics from the heartbeat it is meant to explain. The publish cost
       is one preallocated DiagnosticArray serialised on the same socket the
       heartbeat already uses, so it cannot stall the heartbeat by construction.

       Everything sysusage.c cannot see is stamped here, on the rclc executor's
       own thread - never from the sampler task, which must not touch rclc
       (single XRCE session). video_fps_x100 stays 0 (= n/a): frame rate is
       owned by video_udp.c, which is not linked into the default personality. */
    sysusage_extra_t extra;
    extra.uptime_ms      = HAL_GetTick();
    extra.pool_size      = (uint32_t)POOL_SIZE;
    extra.pool_used      = (uint32_t)pool_used;
    extra.video_fps_x100 = 0u;
    sysusage_set_extra(&extra);
    if (sysusage_publish() != RCL_RET_OK)
    {
        /* Announce once only, then stay quiet: a telemetry failure must never
           degrade or stall the heartbeat, so no retry and no park. */
        if (!sysusage_publish_failed)
        {
            sysusage_publish_failed = true;
            uart_puts("sysusage publish err - heartbeat only\r\n");
        }
    }
#endif /* NCAP_SYSUSAGE == 1 */
}

#if NCAP_PIN_TEST == 1
// DVP wiring self-test. Each data pin is sampled twice, once with the internal
// pull-up and once with the pull-down. A pin driven by the sensor reads much
// the same both ways; a wire on GND stays low, a disconnected wire follows the
// pull. HS/VS/PLK are checked for edges instead. Results are printed one line
// per pin so scripts/test_camera_pins.py can parse them.
typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    char port_char;
    uint8_t pin_no;
    uint8_t is_data;
} dvp_pin_desc_t;

static const dvp_pin_desc_t dvp_pin_map[] = {
    {"D0",  GPIOC, GPIO_PIN_6,  'C', 6,  1},
    {"D1",  GPIOC, GPIO_PIN_7,  'C', 7,  1},
    {"D2",  GPIOC, GPIO_PIN_8,  'C', 8,  1},
    {"D3",  GPIOC, GPIO_PIN_9,  'C', 9,  1},
    {"D4",  GPIOC, GPIO_PIN_11, 'C', 11, 1},
    {"D5",  GPIOB, GPIO_PIN_6,  'B', 6,  1},
    {"D6",  GPIOB, GPIO_PIN_8,  'B', 8,  1},
    {"D7",  GPIOB, GPIO_PIN_9,  'B', 9,  1},
    {"HS",  GPIOA, GPIO_PIN_4,  'A', 4,  0},
    {"VS",  GPIOB, GPIO_PIN_7,  'B', 7,  0},
    {"PLK", GPIOA, GPIO_PIN_6,  'A', 6,  0},
};

static void pin_sample(const dvp_pin_desc_t *p, uint32_t pull, uint32_t ms,
                       uint32_t *high, uint32_t *edges, uint32_t *samples)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = pull;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = p->pin;
    HAL_GPIO_Init(p->port, &g);

    uint32_t prev = (p->port->IDR & p->pin) ? 1u : 0u;
    uint32_t h = 0, e = 0, n = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms)
    {
        uint32_t v = (p->port->IDR & p->pin) ? 1u : 0u;
        h += v;
        e += (v ^ prev);
        prev = v;
        n++;
    }
    *high = h;
    *edges = e;
    *samples = (n != 0u) ? n : 1u;
}

static int dvp_pin_test_run(void)
{
    int failures = 0;
    printf("PIN_TEST begin\r\n");
    for (unsigned i = 0; i < (unsigned)(sizeof(dvp_pin_map) / sizeof(dvp_pin_map[0])); i++)
    {
        const dvp_pin_desc_t *p = &dvp_pin_map[i];
        uint32_t hu, eu, nu, hd, ed, nd;
        // VSYNC is ~5 Hz, so sync lines need a much longer window than the
        // data pins to catch at least one pulse.
        uint32_t ms = p->is_data ? 50u : 400u;
        pin_sample(p, GPIO_PULLUP, ms, &hu, &eu, &nu);
        pin_sample(p, GPIO_PULLDOWN, ms, &hd, &ed, &nd);
        uint32_t up = (hu * 100u) / nu;
        uint32_t dn = (hd * 100u) / nd;
        const char *verdict;

        if (!p->is_data)
        {
            verdict = (eu > 0u) ? "OK" : "NO SIGNAL";
        }
        else if (up <= 5u && dn <= 5u)
        {
            verdict = "STUCK LOW";
        }
        else if (up >= 95u && dn >= 95u)
        {
            verdict = "STUCK HIGH";
        }
        else
        {
            int diff = (int)up - (int)dn;
            if (diff < 0) { diff = -diff; }
            verdict = (diff >= 60) ? "OPEN" : "OK";
        }

        if (strcmp(verdict, "OK") != 0)
        {
            failures++;
        }
        printf("PIN %-3s P%c%-2u up=%2lu%% dn=%2lu%% edges=%lu %s\r\n",
               p->name, p->port_char, (unsigned)p->pin_no,
               (unsigned long)up, (unsigned long)dn,
               (unsigned long)(eu + ed), verdict);
    }
    printf("PIN_TEST RESULT: %s (%d bad)\r\n", (failures != 0) ? "FAIL" : "PASS", failures);
    return failures;
}
#endif

#if NCAP_CAM_TEST == 1
// Connectivity test: sample the data pins with pull-up and with pull-down.
// A pin driven by the sensor reads the same duty both ways; a floating pin
// follows the pull (0% vs 100%). HS/VS/PLK are sampled without pull so they
// are not disturbed.
static uint32_t data_high_percent(uint32_t pull)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = pull;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);

    uint32_t hi = 0, n = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 100u)
    {
        uint32_t b = GPIOB->IDR, c = GPIOC->IDR;
        hi += ((c & (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11)) != 0u) +
              ((b & (GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9)) != 0u);
        n++;
    }
    return (hi * 100u) / (n ? n : 1u);
}

static void cam_pin_activity(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = GPIO_PIN_4 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);
    uint32_t hs = 0, vs = 0;
    uint32_t pa = GPIOA->IDR, pb = GPIOB->IDR;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 100u)
    {
        uint32_t a = GPIOA->IDR, b = GPIOB->IDR;
        if ((a ^ pa) & GPIO_PIN_4) hs++;
        if ((b ^ pb) & GPIO_PIN_7) vs++;
        pa = a; pb = b;
    }
    uint32_t up = data_high_percent(GPIO_PULLUP);
    uint32_t dn = data_high_percent(GPIO_PULLDOWN);
    printf("CAM: HS edges=%lu VS edges=%lu | data high%% pullup=%lu pulldown=%lu\r\n",
           (unsigned long)hs, (unsigned long)vs, (unsigned long)up, (unsigned long)dn);
}
#endif

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    // NUCLEO-F446RE: SPI2 carries the W5500. SPI1 is not initialised because
    // its PA6/PA4 pins are DCMI_PIXCLK/DCMI_HSYNC on this board.
    MX_SPI2_Init();
    MX_USART2_UART_Init();

    uart_puts("\r\nNCAP micro-ROS over W5500\r\n");

#if NCAP_PIN_TEST == 1
    uart_puts("PIN TEST: DVP wiring self-test (no W5500 required)\r\n");
    i2c_bus_init();
    if (!ov7670_init())
    {
        uart_puts("PIN TEST: SCCB init failed - check sensor power and XCLK\r\n");
    }
    for (;;)
    {
        (void)dvp_pin_test_run();
        HAL_Delay(5000);
    }
#endif

#if NCAP_CAM_TEST == 1
    // Camera-only bring-up personality: no W5500, no micro-ROS. Re-probe SCCB
    // every 2 s until the sensor answers, so wiring can be changed live.
    uart_puts("CAM TEST: SCCB probe loop (no W5500 required)\r\n");
    i2c_bus_init();
    for (;;)
    {
        static bool pin_reported = false;
        if (ov7670_init() && !pin_reported)
        {
            cam_pin_activity();
            pin_reported = true;
        }
        HAL_Delay(2000);
    }
#endif

    static uint8_t mac[6]  = {0x00, 0xF4, 0x57, 0x00, 0x00, 0x01};
    static uint8_t ip[4]   = {192, 168, 0, 10};
    mac[5] = (uint8_t)(0x01u + (unsigned)NODE_ID);
    ip[3] = (uint8_t)(10u + (unsigned)NODE_ID);
    static const uint8_t mask[4] = {255, 255, 255, 0};
    static const uint8_t gw[4]   = {192, 168, 0, 1};

    // A cold 5 V rail or a slow module regulator can leave VERSIONR invalid at
    // first boot; retry the whole reset + configure sequence a few times.
    uint8_t ver = 0;
    for (unsigned attempt = 0; attempt < 3u; attempt++)
    {
        w5500_hw_init(mac, ip, mask, gw);
        ver = w5500_version();
        if (ver == 0x04u)
        {
            break;
        }
        HAL_Delay(100);
    }

    if (ver == 0x04)
    {
        uart_puts("W5500 ok (VERSIONR=04)\r\n");
    }
    else
    {
        // 0xFF: MISO idle/not driven (no power, no CS, or chip absent)
        // 0x00: MISO stuck low (short, swapped MOSI/MISO, or no clock)
        printf("W5500 FAIL (VERSIONR=%02X)\r\n", ver);
        while (1) { }
    }

    i2c_bus_init();
    imu_ready = imu_init();
    uart_puts(imu_ready ? "imu init ok\r\n" : "imu init skipped\r\n");

#if NCAP_VIDEO_UDP == 1
    // Bulk video goes out as a UDP stream instead of DDS: no full-frame buffer,
    // no micro-ROS fragmentation, no host-side rmw_microxrcedds needed, and no
    // docker-proxy involvement. Control/telemetry can still use micro-ROS in the
    // default build.
    video_udp_serve();
    while (1) { }
#endif

    allocator.allocate = pool_alloc;
    allocator.deallocate = pool_deallocate;
    allocator.reallocate = pool_reallocate;
    allocator.zero_allocate = pool_zero_alloc;

    node = rcl_get_zero_initialized_node();
    publisher = rcl_get_zero_initialized_publisher();
    timer = rcl_get_zero_initialized_timer();

    // Documented micro-ROS custom-transport pattern: set the global transport
    // first, then hand initialised rcl options to rclc_support_init_with_options().
    if (rmw_uros_set_custom_transport(false, (void *)AGENT_IP,
            cubemx_transport_open_w5500, cubemx_transport_close_w5500,
            cubemx_transport_write_w5500, cubemx_transport_read_w5500) != RMW_RET_OK)
    {
        uart_puts("transport-set FAILED\r\n");
        while (1) { }
    }
    uart_puts("transport-set ok\r\n");

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK)
    {
        uart_puts("init-options FAILED\r\n");
        while (1) { }
    }

#if NODE_ID > 0
    {
        rmw_init_options_t *rmw_opts = rcl_init_options_get_rmw_init_options(&init_options);
        rmw_uros_options_set_client_key(0xC0FFEE00u + (uint32_t)NODE_ID, rmw_opts);
    }
#endif
    rcl_ret_t sup_ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (sup_ret != RCL_RET_OK)
    {
        uart_puts("support-init FAILED\r\n");
        printf("F4: support-init ret=%d\r\n", (int)sup_ret);
        while (1) { }
    }
    uart_puts("support-init ok\r\n");

    char node_name[24];
    char hb_topic[24];
    char imu_topic[24];
#if NODE_ID == 0
    strcpy(node_name, "ncap_f4");
    strcpy(hb_topic, "heartbeat");
    strcpy(imu_topic, "imu/data");
#else
    snprintf(node_name, sizeof node_name, "ncap_f4_%u", (unsigned)NODE_ID);
    snprintf(hb_topic, sizeof hb_topic, "heartbeat_%u", (unsigned)NODE_ID);
    snprintf(imu_topic, sizeof imu_topic, "imu/data_%u", (unsigned)NODE_ID);
#endif
    printf("F4: node %s key %08X\r\n", node_name,
           (unsigned)(0xC0FFEE00u + (unsigned)NODE_ID));
    rcl_ret_t node_ret = rclc_node_init_default(&node, node_name, "", &support);
    if (node_ret != RCL_RET_OK)
    {
        uart_puts("node-init FAILED\r\n");
        printf("F4: node-init ret=%d\r\n", (int)node_ret);
        while (1) { }
    }

#if NCAP_SYSUSAGE == 1
    /* Optional publisher: on failure print one line and carry on with the
       heartbeat only - same degrade-not-park decision as the IMU publisher
       below. Parking here would take the node down over telemetry. */
    if (!sysusage_publisher_init(&support, &node, &allocator, NULL))
    {
        uart_puts("sysusage publisher unavailable\r\n");
        printf("F4: sysusage publisher unavailable\r\n");
    }
    else
    {
        uart_puts("sysusage publisher ok\r\n");
    }
#endif /* NCAP_SYSUSAGE == 1 */

    rcl_ret_t pub_ret = rclc_publisher_init_default(&publisher, &node, ROS_TOPIC_TYPE, hb_topic);
    if (pub_ret != RCL_RET_OK)
    {
        uart_puts("publisher-init FAILED\r\n");
        printf("F4: publisher-init ret=%d\r\n", (int)pub_ret);
        while (1) { }
    }
    uart_puts("publisher-init ok\r\n");

    if (imu_ready)
    {
        const rosidl_message_type_support_t * imu_ts =
            rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__sensor_msgs__msg__Imu();
        rcl_ret_t imu_pub_ret = rclc_publisher_init_default(&imu_pub, &node, imu_ts, imu_topic);
        if (imu_pub_ret != RCL_RET_OK)
        {
            uart_puts("imu publisher FAILED - heartbeat only\r\n");
            imu_ready = false;
        }
        else
        {
            uart_puts("imu publisher ok\r\n");
        }
    }

    rcl_ret_t ex_ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ex_ret != RCL_RET_OK)
    {
        uart_puts("executor-init FAILED\r\n");
        printf("F4: executor-init ret=%d pool_used=%u/%u\r\n", (int)ex_ret,
               (unsigned)pool_used, (unsigned)POOL_SIZE);
        while (1) { }
    }
    // The timer must be fully initialised BEFORE it is registered with the
    // executor: rclc_executor_add_timer() dereferences the timer handle to fill
    // in its index, and registering a zero-initialised rcl_timer_t leaves the
    // executor holding a stale handle once init overwrites the struct.
    rcl_ret_t timer_ret = rclc_timer_init_default(&timer, &support,
                                                  (uint64_t)PUB_PERIOD * 1000000ULL, timer_callback);
    if (timer_ret != RCL_RET_OK)
    {
        uart_puts("timer-init FAILED\r\n");
        printf("F4: timer-init ret=%d\r\n", (int)timer_ret);
        while (1) { }
    }
    rcl_ret_t add_ret = rclc_executor_add_timer(&executor, &timer);
    if (add_ret != RCL_RET_OK)
    {
        uart_puts("timer-add FAILED\r\n");
        printf("F4: timer-add ret=%d\r\n", (int)add_ret);
        while (1) { }
    }
    uart_puts("executor ok, spinning\r\n");

#if NCAP_FREERTOS == 1
    /* FreeRTOS owns the loop from here. There is deliberately NO bare-metal
       fallback below this point: the two spin loops at the bottom of main()
       never return, so a spin loop placed after vTaskStartScheduler() would be
       dead code.

       The executor moves into its own task (rcl_task, above) because a scheduler
       that is never spun publishes nothing at all. Task creation order matters:
       everything must exist before xPortStartFirstTask() picks a task to run,
       so both the rcl task and the sysusage sampler are created here, on the
       main() stack and before the scheduler starts.

       Priorities, lowest to highest (configMAX_PRIORITIES is 8, plenty):
         0  idle task        - the CPU-load metric's numerator
         1  sysusage sampler - only when NCAP_SYSUSAGE=1
         2  rcl task         - only non-idle task above the sampler
       The rcl task must be strictly above the sampler, and must block rather
       than spin, or the sampler's periodic sample is starved by the transport's
       RX poll. Both properties come from ncap_cpu_relax() above. */
    rcl_task_start();
#if NCAP_SYSUSAGE == 1
    (void)sysusage_task_start();
#endif
    uart_puts("freertos scheduler starting\r\n");
    vTaskStartScheduler();
    /* Not reached: xPortStartScheduler() does not return once the scheduler is
       running. Kept as an explicit park rather than a silent fall-through. */
    uart_puts("scheduler RETURNED - fault\r\n");
    while (1) { }
#endif /* NCAP_FREERTOS == 1 */

/* ---- bare-metal personalities: the only live loops when NCAP_FREERTOS == 0 ---- */
#if NCAP_VIDEO_UDP == 2
    // Combined run: micro-ROS owns the control plane (heartbeat on DDS) and the
    // video stream runs on its own W5500 socket. rclc_executor_spin() never
    // returns, so this uses bounded spin_some() slices and emits one video row
    // block per slice - cooperative multitasking on bare metal.
    video_udp_init();
    uart_puts("combined ok, spinning + streaming\r\n");
    for (;;)
    {
        rclc_executor_spin_some(&executor, 5ULL * 1000ULL * 1000ULL);
        video_udp_step();
    }
#else
    for (;;)
    {
        rclc_executor_spin(&executor);
    }
#endif
}
