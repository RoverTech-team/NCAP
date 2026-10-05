// NCAP STM32F4 — modular system-usage reporting over micro XRCE-DDS.
//
// See sysusage.h for the (frozen) public API and docs/plan-sysusage.md for the
// design. Hard constraints this file lives by:
//
//   * zero heap. Every rosidl_runtime_c__String buffer is a static char[] and
//     every *__{Sequence}.data points at a static array with capacity set to
//     that array's true length. libmicroros.a must never be handed a pointer
//     it could try to free().
//   * no printf floating point. The firmware links -specs=nano.specs, which has
//     no %f and no %l/%ll, so every fixed-point value is scaled by 100 and
//     printed as an integer (the *_x100 convention).
//   * one rclc caller. Nothing here touches rclc/rcl/rmw except
//     sysusage_publisher_init() and sysusage_publish().
//   * the whole file is empty when NCAP_SYSUSAGE == 0.

#include "sysusage.h"

#if NCAP_SYSUSAGE == 1

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"                          // HAL_GetTick()
#include <rmw_microros/time_sync.h>

// main.c declares this the same way for sensor_msgs/Imu (main.c:30-33).
const rosidl_message_type_support_t *
rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticArray(void);

#if NCAP_FREERTOS == 1
// Defensive: the Makefile passes -DNCAP_FREERTOS=$(NCAP_FREERTOS); these
// defaults let this file compile standalone (scratch builds, IDE projects).
#include "FreeRTOS.h"
#include "task.h"
#endif

/* ------------------------------------------------------------------ *
 * Static budget. Every byte below lands in .bss (nothing is
 * initialised at load time; the buffers are filled at first publish).
 * Numbers are the measured sizeof() on arm-none-eabi-gcc for
 * -mthumb -mcpu=cortex-m4 (size_t == 4, StackType_t == 4).
 * ------------------------------------------------------------------ */

/* frame_id of the DiagnosticArray header: "ncap_f4" or "ncap_f4_<id>". */
#define SU_FRAME_ID_LEN    16u

/* DiagnosticStatus "f4_system". */
#define SU_SYS_NAME_LEN    12u   /* "f4_system" + NUL                        */
#define SU_SYS_MSG_LEN     32u   /* longest level message                    */
#define SU_HW_ID_LEN       16u   /* "ncap_f4" / "ncap_f4_255" + NUL           */
#define SU_SYS_KV_MAX      16u   /* 13 used; 16 leaves headroom, not RAM 2x  */
#define SU_SYS_KEY_LEN     26u   /* longest key "heap_min_ever_free_bytes"+NUL */
#define SU_SYS_VAL_LEN     16u   /* longest value: 10-digit u32 + NUL        */

/* DiagnosticStatus "f4_tasks" (RTOS builds only). */
#define SU_TSK_NAME_LEN    12u   /* "f4_tasks" + NUL                         */
#define SU_TSK_MSG_LEN     32u
#define SU_TSK_KV_MAX      12u   /* one KeyValue per reported task          */
#define SU_TSK_KEY_LEN     20u   /* configMAX_TASK_NAME_LEN is 16, +room    */
#define SU_TSK_VAL_LEN     24u   /* "R stk=65535 rt=100%" (19) + NUL        */

/* Tasks reported in "f4_tasks". */
#define SU_MAX_TASKS       12u

#if NCAP_FREERTOS == 1
#define SU_STATUS_MAX      2u
#else
#define SU_STATUS_MAX      1u
#endif

/* Sampler task period: 4 samples per publish period, floor of 50 ms. */
#define SU_SAMPLE_MS       ((SYSUSAGE_PERIOD_MS / 4u) > 50u ? (SYSUSAGE_PERIOD_MS / 4u) : 50u)

/* ---- string buffers (static char[], zero-initialised => .bss) ---- */
static char su_frame_id[SU_FRAME_ID_LEN];                     /*   16 B */

static char su_sys_name[SU_SYS_NAME_LEN];                     /*   12 B */
static char su_sys_msg[SU_SYS_MSG_LEN];                       /*   32 B */
static char su_hw_id[SU_HW_ID_LEN];                           /*   16 B */
static char su_sys_key[SU_SYS_KV_MAX][SU_SYS_KEY_LEN];        /*  416 B */
static char su_sys_val[SU_SYS_KV_MAX][SU_SYS_VAL_LEN];        /*  256 B */

#if NCAP_FREERTOS == 1
static char su_tsk_name[SU_TSK_NAME_LEN];                     /*   12 B */
static char su_tsk_msg[SU_TSK_MSG_LEN];                       /*   32 B */
static char su_tsk_key[SU_TSK_KV_MAX][SU_TSK_KEY_LEN];        /*  240 B */
static char su_tsk_val[SU_TSK_KV_MAX][SU_TSK_VAL_LEN];        /*  288 B */
#endif

/* ---- message structs (static => .bss) ---- */
/* DiagnosticArray 32 B, KeyValue 24 B each, DiagnosticStatus 52 B each. */
static diagnostic_msgs__msg__DiagnosticArray su_msg;                        /*  32 B */
static diagnostic_msgs__msg__DiagnosticStatus su_status[SU_STATUS_MAX];     /* 104 B */
static diagnostic_msgs__msg__KeyValue su_sys_kv[SU_SYS_KV_MAX];            /* 384 B */
#if NCAP_FREERTOS == 1
static diagnostic_msgs__msg__KeyValue su_tsk_kv[SU_TSK_KV_MAX];            /* 288 B */
#endif

/* ---- module state ---- */
static rcl_publisher_t su_pub;
static bool su_pub_ready;
static bool su_published;
static sysusage_extra_t su_extra;
static uint32_t su_seq;
static uint32_t su_publish_fails;

/* ================================================================== *
 * Bounded string helpers. nano.specs: integer conversions only.
 * ================================================================== */

/* Bound the payload of *s to src (always NUL-terminated) and fix s->size. */
static void su_str_setn(rosidl_runtime_c__String *s, const char *src)
{
    size_t i = 0u;

    if (s == NULL || s->data == NULL || s->capacity == 0u)
    {
        return;
    }
    while ((i + 1u) < s->capacity && src[i] != '\0')
    {
        s->data[i] = src[i];
        i++;
    }
    s->data[i] = '\0';
    s->size = i;
}

/* Literal convenience wrapper. */
static void su_str_set(rosidl_runtime_c__String *s, const char *src)
{
    su_str_setn(s, src);
}

/* snprintf straight into *s. The bound IS the buffer's capacity, so the result
   can never overrun; vsnprintf always NUL-terminates within it. */
static void su_str_setf(rosidl_runtime_c__String *s, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (s == NULL || s->data == NULL || s->capacity == 0u)
    {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(s->data, s->capacity, fmt, ap);
    va_end(ap);

    if (n < 0)
    {
        s->data[0] = '\0';
        s->size = 0u;
        return;
    }
    /* strlen, not the vsnprintf return: the return is the would-be length and
       would report a truncated value larger than what actually landed. */
    s->size = strlen(s->data);
}

/* hundredths -> "A.BB". No %f anywhere: integer conversions only. */
static void su_str_set_fixed(rosidl_runtime_c__String *s, uint32_t hundredths)
{
    su_str_setf(s, "%u.%02u", (unsigned)(hundredths / 100u), (unsigned)(hundredths % 100u));
}

/* Wire a static char[] into a rosidl_runtime_c__String. */
#define SU_BIND(buf, field)              \
    do                                   \
    {                                    \
        (field).data = (buf);            \
        (field).size = 0u;               \
        (field).capacity = sizeof(buf);  \
    } while (0)

/* ================================================================== *
 * Snapshot plumbing
 * ================================================================== */

/* Scalars sampled by the task, or inline when there is no task. Small enough
   (16 B) to pass by pointer from either personality. */
typedef struct
{
    uint32_t uptime_ms;
    uint32_t cpu_load_x100;        /* 0..10000, RTOS builds only            */
    uint32_t heap_free;            /* xPortGetFreeHeapSize, RTOS only        */
    uint32_t heap_min_ever_free;   /* xPortGetMinimumEverFreeHeapSize       */
} su_sample_t;

#if NCAP_FREERTOS == 1

/* What the "f4_tasks" status formats from. Pointed at the snapshot slot the
   publisher currently holds; there is no equivalent in the no-task personality,
   which does not compile this block or emit the "f4_tasks" status at all.
   su_view_total_runtime is only consumed when configGENERATE_RUN_TIME_STATS
   makes ulRunTimeCounter meaningful (it does not in this tree). */
static const TaskStatus_t *su_view_tasks;
static uint32_t su_view_task_count;
static uint32_t su_view_total_runtime;

/* ------------------------------------------------------------------ *
 * CPU load. The idle task runs only when nothing else is ready, so
 *   load_pct = 100 - idle_picks / tick_delta * 100.
 * ------------------------------------------------------------------ */
static volatile uint32_t su_idle_picks;
static volatile uint32_t su_load_x100;
static uint32_t su_prev_idle;
static uint32_t su_prev_tick;

/* configUSE_IDLE_HOOK is 1 in the shipped build (Makefile/FreeRTOSConfig.h), so
   the kernel calls this from the idle task. The definition is unconditional so
   the module compiles even if the config still has the hook disabled.

   Keepalive rationale: the symbol has external linkage, so -Wall cannot flag it
   as an unused function. su_idle_hook_ref is a file-scope constant pointer that
   forces a relocation the linker must resolve, which also survives
   --gc-sections in a build where configUSE_IDLE_HOOK is still 0 (stale config,
   or a standalone scratch compile of this file). A const-pointer table is
   preferred over __attribute__((used)) because the attribute is opaque to most
   static analysers and to -fkeep-style section GC. 4 B of .rodata. */
void vApplicationIdleHook(void);
static void (*const su_idle_hook_ref)(void) = vApplicationIdleHook;

void vApplicationIdleHook(void)
{
    su_idle_picks++;
}

/* Fold the idle-pick delta since the previous sample into su_load_x100. */
static void su_load_update(void)
{
    uint32_t idle = su_idle_picks;
    uint32_t tick = (uint32_t)xTaskGetTickCount();
    uint32_t d_idle = idle - su_prev_idle;
    uint32_t d_tick = tick - su_prev_tick;
    uint32_t idle_x100;

    su_prev_idle = idle;
    su_prev_tick = tick;

    if (d_tick == 0u)
    {
        /* No time elapsed since the last sample: report fully loaded rather
           than divide by zero. */
        su_load_x100 = 10000u;
        return;
    }
    if (d_idle > d_tick)
    {
        d_idle = d_tick;   /* the idle task cannot run more than once per tick */
    }
    idle_x100 = (d_idle * 10000u) / d_tick;
    if (idle_x100 > 10000u)
    {
        idle_x100 = 10000u;
    }
    su_load_x100 = 10000u - idle_x100;
}

/* ------------------------------------------------------------------ *
 * Double-buffered snapshot handoff.
 *
 * Two slots. The sampler claims a free slot with an O(1) index decision under a
 * short critical section, fills it with no scheduler interaction at all, then
 * publishes the index with a second short critical section. The publisher
 * picks up the ready index with a short critical section, formats out of it,
 * and clears the busy flag with a last short critical section.
 *
 * The flag is cleared only AFTER formatting, so a slot the publisher is reading
 * is never one the sampler picks. No critical section is ever held across
 * rclc, SPI or W5500 access, and no FreeRTOS queue (heap) is involved.
 * ------------------------------------------------------------------ */
typedef struct
{
    su_sample_t s;
    uint32_t total_runtime;
    uint32_t task_count;
    TaskStatus_t tasks[SU_MAX_TASKS];
} su_snap_t;

static su_snap_t su_slot[2];
static volatile uint8_t su_slot_busy[2];
static volatile uint8_t su_slot_ready;
static volatile uint8_t su_slot_ready_valid;

/* Sampler task: statically allocated so it costs no configTOTAL_HEAP_SIZE. */
#define SU_TASK_STACK_WORDS 256u                      /* 1024 B */
#define SU_TASK_PRIO        (tskIDLE_PRIORITY + 1)
static StackType_t su_task_stack[SU_TASK_STACK_WORDS];
static StaticTask_t su_task_tcb;

static su_snap_t *su_slot_claim(void)
{
    uint8_t i;

    taskENTER_CRITICAL();
    i = (su_slot_busy[0] != 0u) ? 1u : 0u;
    if (su_slot_busy[i] != 0u)
    {
        taskEXIT_CRITICAL();
        return NULL;   /* both slots in flight: skip this round */
    }
    su_slot_busy[i] = 1u;
    taskEXIT_CRITICAL();

    return &su_slot[i];
}

static void su_slot_commit(const su_snap_t *dst)
{
    uint8_t i = (uint8_t)(dst - &su_slot[0]);

    taskENTER_CRITICAL();
    su_slot_ready = i;
    su_slot_ready_valid = 1u;
    taskEXIT_CRITICAL();
}

static su_snap_t *su_slot_fetch(void)
{
    uint8_t i;

    taskENTER_CRITICAL();
    i = (su_slot_ready_valid != 0u) ? su_slot_ready : 0u;
    taskEXIT_CRITICAL();

    return &su_slot[i];
}

static void su_slot_release(su_snap_t *src)
{
    uint8_t i = (uint8_t)(src - &su_slot[0]);

    taskENTER_CRITICAL();
    su_slot_busy[i] = 0u;
    taskEXIT_CRITICAL();
}

/* The sampler never calls rclc/rcl/rmw. It only reads kernel state. */
static void su_sampler_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        su_snap_t *dst = su_slot_claim();

        if (dst != NULL)
        {
            UBaseType_t want;
            UBaseType_t got;

            su_load_update();

            dst->s.uptime_ms = HAL_GetTick();
            dst->s.cpu_load_x100 = su_load_x100;
            dst->s.heap_free = (uint32_t)xPortGetFreeHeapSize();
            dst->s.heap_min_ever_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();

            /* Clamp with uxTaskGetNumberOfTasks(): uxTaskGetSystemState fills
               NOTHING and returns 0 if the array is smaller than the task
               count, so asking for 12 unconditionally would silently report
               nothing. */
            want = uxTaskGetNumberOfTasks();
            if (want > (UBaseType_t)SU_MAX_TASKS)
            {
                want = (UBaseType_t)SU_MAX_TASKS;
            }
            if (want == 0u)
            {
                got = 0u;
            }
            else
            {
                /* configUSE_TRACE_FACILITY is 1, so this symbol exists. It
                   suspends the scheduler internally: task context only, never
                   an ISR. */
                got = uxTaskGetSystemState(dst->tasks, want, &dst->total_runtime);
                if (got > (UBaseType_t)SU_MAX_TASKS)
                {
                    got = (UBaseType_t)SU_MAX_TASKS;
                }
            }
            dst->task_count = (uint32_t)got;

            su_slot_commit(dst);
        }

        vTaskDelay(pdMS_TO_TICKS(SU_SAMPLE_MS));
    }
}

#endif /* NCAP_FREERTOS == 1 */

/* ================================================================== *
 * Public API
 * ================================================================== */

bool sysusage_publisher_init(rclc_support_t *support,
                             rcl_node_t *node,
                             rcl_allocator_t *allocator,
                             const char *topic_name)
{
    const rosidl_message_type_support_t *ts;
    rcl_ret_t ret;

    /* Frozen signature: rclc_publisher_init_default() takes the node, not the
       support, so support is part of the contract but unused here. */
    (void)support;

    if (node == NULL || allocator == NULL)
    {
        return false;
    }
    if (topic_name == NULL)
    {
        topic_name = "diagnostics";
    }

    ts = rosidl_typesupport_microxrcedds_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticArray();
    if (ts == NULL)
    {
        printf("F4: sysusage type-support missing\r\n");
        return false;
    }

    su_pub = rcl_get_zero_initialized_publisher();
    ret = rclc_publisher_init_default(&su_pub, node, ts, topic_name);
    if (ret != RCL_RET_OK)
    {
        /* Never park: the caller decides the degraded path. */
        printf("F4: sysusage publisher-init ret=%d\r\n", (int)ret);
        su_pub_ready = false;
        return false;
    }

    /* Bind every static buffer into the message once, so the publish path never
       re-wires anything and can never allocate. */
    SU_BIND(su_frame_id, su_msg.header.frame_id);
    su_msg.header.stamp.sec = 0;
    su_msg.header.stamp.nanosec = 0u;
#if defined(NODE_ID) && (NODE_ID > 0)
    su_str_setf(&su_msg.header.frame_id, "ncap_f4_%u", (unsigned)NODE_ID);
#else
    su_str_set(&su_msg.header.frame_id, "ncap_f4");
#endif

    su_msg.status.data = su_status;
    su_msg.status.size = 0u;
    su_msg.status.capacity = SU_STATUS_MAX;

    SU_BIND(su_sys_name, su_status[0].name);
    SU_BIND(su_sys_msg, su_status[0].message);
    SU_BIND(su_hw_id, su_status[0].hardware_id);
    /* Same value as frame_id: hardware_id is what a ROS consumer keys on to tell
       nodes apart, and an empty string made every /diagnostics array look like it
       came from the same anonymous board. su_status[1] shares this buffer. */
    su_str_setn(&su_status[0].hardware_id, su_msg.header.frame_id.data);
    su_status[0].values.data = su_sys_kv;
    su_status[0].values.size = 0u;
    su_status[0].values.capacity = SU_SYS_KV_MAX;
    {
        uint32_t i;
        for (i = 0u; i < SU_SYS_KV_MAX; i++)
        {
            SU_BIND(su_sys_key[i], su_sys_kv[i].key);
            SU_BIND(su_sys_val[i], su_sys_kv[i].value);
        }
    }

#if NCAP_FREERTOS == 1
    SU_BIND(su_tsk_name, su_status[1].name);
    SU_BIND(su_tsk_msg, su_status[1].message);
    /* Share status[0]'s hardware_id (same static buffer, never written again). */
    su_status[1].hardware_id = su_status[0].hardware_id;
    su_status[1].values.data = su_tsk_kv;
    su_status[1].values.size = 0u;
    su_status[1].values.capacity = SU_TSK_KV_MAX;
    {
        uint32_t i;
        for (i = 0u; i < SU_TSK_KV_MAX; i++)
        {
            SU_BIND(su_tsk_key[i], su_tsk_kv[i].key);
            SU_BIND(su_tsk_val[i], su_tsk_kv[i].value);
        }
    }
#endif

    su_pub_ready = true;
    return true;
}

void sysusage_set_extra(const sysusage_extra_t *extra)
{
    if (extra == NULL)
    {
        memset(&su_extra, 0, sizeof(su_extra));
        return;
    }
    su_extra = *extra;
}

bool sysusage_task_start(void)
{
#if NCAP_FREERTOS == 1
    /* V10.3.1's xTaskCreateStatic() returns a TaskHandle_t, not a BaseType_t
       (task.h:445-453), so the check is against NULL. */
    TaskHandle_t handle;

    (void) &su_idle_hook_ref;   /* see the keepalive note at its definition */

    su_prev_idle = su_idle_picks;
    su_prev_tick = (uint32_t)xTaskGetTickCount();

    handle = xTaskCreateStatic(su_sampler_task,
                               "sysusage",
                               (uint32_t)SU_TASK_STACK_WORDS,
                               NULL,
                               (UBaseType_t)SU_TASK_PRIO,
                               su_task_stack,
                               &su_task_tcb);
    if (handle == NULL)
    {
        printf("F4: sysusage task create failed\r\n");
        return false;
    }
    return true;
#else
    /* Not an error: sysusage_publish() samples inline in this personality. */
    return false;
#endif
}

const diagnostic_msgs__msg__DiagnosticArray *sysusage_last(void)
{
    return su_published ? &su_msg : NULL;
}

/* ================================================================== *
 * Formatting
 * ================================================================== */

/* OK / WARN / ERROR from real thresholds. */
static uint8_t su_level(uint32_t cpu_x100,
                        uint32_t heap_free,
                        uint32_t heap_min_ever_free,
                        uint32_t pool_used,
                        uint32_t pool_size,
                        const char **msg)
{
#if NCAP_FREERTOS == 1
    if (heap_min_ever_free < 512u)
    {
        *msg = "rtos heap nearly exhausted";
        return diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    }
#endif
    if (pool_size != 0u && (pool_used * 10000u) / pool_size >= 9000u)
    {
        *msg = "rcl pool nearly exhausted";
        return diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    }
#if NCAP_FREERTOS == 1
    if (cpu_x100 >= 8000u)
    {
        *msg = "cpu load above 80%";
        return diagnostic_msgs__msg__DiagnosticStatus__WARN;
    }
    if (heap_free < 1024u)
    {
        *msg = "rtos heap low";
        return diagnostic_msgs__msg__DiagnosticStatus__WARN;
    }
#endif
    *msg = "ok";
    return diagnostic_msgs__msg__DiagnosticStatus__OK;
}

/* Small state-name letter for the f4_tasks values. */
#if NCAP_FREERTOS == 1
static char su_state_char(eTaskState st)
{
    switch (st)
    {
    case eRunning:   return 'X';
    case eReady:     return 'R';
    case eBlocked:   return 'B';
    case eSuspended: return 'S';
    case eDeleted:   return 'D';
    default:         return '?';
    }
}
#endif

static void su_stamp(void)
{
    /* Verified present in the prebuilt libmicroros.a:
     *   T rmw_uros_epoch_nanos / rmw_uros_epoch_millis / rmw_uros_epoch_synchronized
     * (librmw_microxrcedds-time_sync.c.obj). Unlike the IMU publish at
     * main.c:163-169, the stamp IS set here. rmw_uros_epoch_nanos() returns the
     * session time (boot-relative) even before NTP sync; rmw_uros_epoch_
     * synchronized() says whether the epoch offset has been applied yet. */
    int64_t ns = rmw_uros_epoch_nanos();

    if (ns > 0)
    {
        su_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
        su_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
        return;
    }

    {
        int64_t ms = rmw_uros_epoch_millis();
        if (ms > 0)
        {
            su_msg.header.stamp.sec = (int32_t)(ms / 1000LL);
            su_msg.header.stamp.nanosec = (uint32_t)((ms % 1000LL) * 1000000LL);
            return;
        }
    }

    /* No XRCE session time at all: fall back to the HAL tick so the host still
       gets a monotonically increasing stamp instead of zeros. */
    {
        uint32_t tick = HAL_GetTick();
        su_msg.header.stamp.sec = (int32_t)(tick / 1000u);
        su_msg.header.stamp.nanosec = (tick % 1000u) * 1000000u;
    }
}

/* --- "f4_system" --- */
static uint32_t su_format_system(const su_sample_t *s)
{
    diagnostic_msgs__msg__DiagnosticStatus *st = &su_status[0];
    const char *msg = "ok";
    uint32_t n = 0u;
    uint32_t pool_free_pct_x100 = 0u;

    if (su_extra.pool_size != 0u)
    {
        uint32_t free_bytes = (su_extra.pool_used <= su_extra.pool_size)
                              ? (su_extra.pool_size - su_extra.pool_used) : 0u;
        pool_free_pct_x100 = (free_bytes * 10000u) / su_extra.pool_size;
    }

    st->level = su_level(s->cpu_load_x100, s->heap_free, s->heap_min_ever_free,
                         su_extra.pool_used, su_extra.pool_size, &msg);
    su_str_set(&st->name, "f4_system");
    su_str_set(&st->message, msg);
    /* Idempotent: st->hardware_id is bound to su_hw_id in
       sysusage_publisher_init() and written once there. */
    su_str_setn(&st->hardware_id, su_hw_id);
    st->values.size = 0u;   /* reset every publish; size is the wire count */

    su_str_set(&su_sys_kv[n].key, "build");
#if NCAP_FREERTOS == 1
    su_str_set(&su_sys_kv[n].value, "freertos");
#else
    su_str_set(&su_sys_kv[n].value, "bare-metal");
#endif
    n++;

    su_str_set(&su_sys_kv[n].key, "period_ms");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)SYSUSAGE_PERIOD_MS);
    n++;

    su_str_set(&su_sys_kv[n].key, "uptime_ms");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)s->uptime_ms);
    n++;

    /* This module's own publish counter, NOT main.c's heartbeat `counter` (that
       is file-static in main.c and unreachable from here). su_seq is the number
       of DiagnosticArrays published before this one, so it starts at 0. */
    su_str_set(&su_sys_kv[n].key, "seq");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)su_seq);
    n++;

#if NCAP_FREERTOS == 1
    su_str_set(&su_sys_kv[n].key, "cpu_load_pct");
    su_str_set_fixed(&su_sys_kv[n].value, s->cpu_load_x100);
    n++;

    su_str_set(&su_sys_kv[n].key, "heap_free_bytes");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)s->heap_free);
    n++;

    su_str_set(&su_sys_kv[n].key, "heap_min_ever_free_bytes");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)s->heap_min_ever_free);
    n++;
#endif

    su_str_set(&su_sys_kv[n].key, "rcl_pool_used");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)su_extra.pool_used);
    n++;

    su_str_set(&su_sys_kv[n].key, "rcl_pool_size");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)su_extra.pool_size);
    n++;

    su_str_set(&su_sys_kv[n].key, "rcl_pool_free_pct");
    su_str_set_fixed(&su_sys_kv[n].value, pool_free_pct_x100);
    n++;

    if (su_extra.video_fps_x100 != 0u)
    {
        su_str_set(&su_sys_kv[n].key, "video_fps_x100");
        su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)su_extra.video_fps_x100);
        n++;
    }

    su_str_set(&su_sys_kv[n].key, "time_sync");
    su_str_set(&su_sys_kv[n].value,
               rmw_uros_epoch_synchronized() ? "ntp" : "session");
    n++;

    su_str_set(&su_sys_kv[n].key, "publish_fails");
    su_str_setf(&su_sys_kv[n].value, "%u", (unsigned)su_publish_fails);
    n++;

    if (n > SU_SYS_KV_MAX)
    {
        n = SU_SYS_KV_MAX;   /* unreachable with the fixed key list above */
    }
    st->values.size = n;
    return n;
}

#if NCAP_FREERTOS == 1
/* --- "f4_tasks" --- */
static void su_format_tasks(void)
{
    diagnostic_msgs__msg__DiagnosticStatus *st = &su_status[1];
    uint32_t n = 0u;
    uint32_t i;
    uint8_t worst = diagnostic_msgs__msg__DiagnosticStatus__OK;
    const char *msg = "ok";

    st->level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    su_str_set(&st->name, "f4_tasks");
    st->values.size = 0u;

#if (configGENERATE_RUN_TIME_STATS != 1)
    /* No run-time counter is wired up in this build, so ulRunTimeCounter is
       meaningless and the per-task runtime share cannot be computed. */
    (void) su_view_total_runtime;
#endif

    for (i = 0u; i < su_view_task_count && n < SU_TSK_KV_MAX; i++)
    {
        const TaskStatus_t *t = &su_view_tasks[i];

        su_str_setn(&su_tsk_kv[n].key, (t->pcTaskName != NULL) ? t->pcTaskName : "?");

#if (configGENERATE_RUN_TIME_STATS == 1)
        {
            /* Only valid when a run-time counter is wired up; the vendored
               FreeRTOSConfig.h does not define one, so this stays compiled out
               and the value below reads "n/a". */
            uint32_t rt_pct_x100 = 0u;
            if (su_view_total_runtime != 0u)
            {
                rt_pct_x100 = (t->ulRunTimeCounter * 10000u) / su_view_total_runtime;
            }
            su_str_setf(&su_tsk_kv[n].value, "%c stk=%u rt=%u.%02u",
                        su_state_char(t->eCurrentState),
                        (unsigned)t->usStackHighWaterMark,
                        (unsigned)(rt_pct_x100 / 100u),
                        (unsigned)(rt_pct_x100 % 100u));
        }
#else
        su_str_setf(&su_tsk_kv[n].value, "%c stk=%u rt=n/a",
                    su_state_char(t->eCurrentState),
                    (unsigned)t->usStackHighWaterMark);
#endif

        /* A high-water mark at or below 2 words is an overflow in progress. */
        if (t->usStackHighWaterMark <= 2u)
        {
            worst = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
            msg = "task stack margin exhausted";
        }
        else if (worst == diagnostic_msgs__msg__DiagnosticStatus__OK &&
                 t->usStackHighWaterMark <= 8u)
        {
            worst = diagnostic_msgs__msg__DiagnosticStatus__WARN;
            msg = "task stack margin low";
        }
        n++;
    }

    su_str_set(&st->message, (n == 0u) ? "no tasks sampled" : msg);
    su_str_setn(&st->hardware_id, su_hw_id);
    st->level = (n == 0u) ? diagnostic_msgs__msg__DiagnosticStatus__STALE : worst;
    st->values.size = n;
}
#endif /* NCAP_FREERTOS == 1 */

/* ================================================================== *
 * Publish. Executor thread only. The single rclc caller in this module.
 * ================================================================== */
rcl_ret_t sysusage_publish(void)
{
    su_sample_t s;
    rcl_ret_t ret;

    if (!su_pub_ready)
    {
        return RCL_RET_ERROR;
    }

    memset(&s, 0, sizeof(s));

#if NCAP_FREERTOS == 1
    {
        /* Short critical section: index read only. */
        su_snap_t *snap = su_slot_fetch();

        s = snap->s;
        su_view_tasks = snap->tasks;
        su_view_task_count = snap->task_count;
        su_view_total_runtime = snap->total_runtime;

        (void) su_format_system(&s);
        su_format_tasks();

        /* Short critical section: flag clear only. Formatted data lives in the
           module's own buffers, so nothing below can race the sampler. */
        su_slot_release(snap);

        su_msg.status.size = 2u;
    }
#else
    /* No scheduler: sample inline. HAL_GetTick() is the only clock available,
       so cpu_load_pct / heap_* keys are omitted rather than faked. */
    s.uptime_ms = HAL_GetTick();
    (void) su_format_system(&s);
    su_msg.status.size = 1u;
#endif

    su_stamp();
    su_seq++;

    ret = rcl_publish(&su_pub, &su_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        su_publish_fails++;
        return ret;
    }

    su_published = true;
    return RCL_RET_OK;
}

#endif /* NCAP_SYSUSAGE == 1 */
