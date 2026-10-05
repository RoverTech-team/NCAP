// NCAP STM32F4 — modular system-usage reporting (docs/plan-sysusage.md).
//
// One self-contained module that owns its publisher, its message buffers and
// (when a scheduler exists) its own statically allocated sampler task.
//
// Two personalities, one code path:
//   NCAP_SYSUSAGE=1 NCAP_FREERTOS=1 -> a task samples into a double buffer,
//        sysusage_publish() flips the index on the executor thread.
//   NCAP_SYSUSAGE=1 NCAP_FREERTOS=0 -> no task; sysusage_publish() samples
//        inline from HAL_GetTick().
//
// When NCAP_SYSUSAGE == 0 this header is an empty translation unit and
// sysusage.c emits no code, no symbols and no storage.

#ifndef NCAP_SYSUSAGE_H
#define NCAP_SYSUSAGE_H

#include <stdbool.h>
#include <stdint.h>

#ifndef NCAP_SYSUSAGE
#define NCAP_SYSUSAGE 0
#endif

#if NCAP_SYSUSAGE == 1

// Devensive defaults so this header is self-contained even when the Makefile
// has not been taught about a macro yet. A -D on the command line always wins.
#ifndef NCAP_FREERTOS
#define NCAP_FREERTOS 0
#endif

#include <rcl/allocator.h>
#include <rcl/publisher.h>
#include <rclc/node.h>
// NOTE: the plan's frozen snippet lists <rclc/support.h>. That header does not
// exist in this vendored rclc (verified: firmware/microros/include/rclc/ has no
// support.h). rclc_support_t is declared in <rclc/types.h>, pulled in by
// <rclc/init.h>, so we include those instead. The published API below is
// otherwise verbatim.
#include <rclc/types.h>
#include <rclc/init.h>
#include <rclc/publisher.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>

#ifndef SYSUSAGE_PERIOD_MS
#define SYSUSAGE_PERIOD_MS 1000u
#endif

/* Create the /diagnostics publisher. Call once, after rclc_support_init and
   rclc_node_init_default, before rclc_executor_init is fine either way.
   topic_name NULL => "diagnostics". Returns false on failure; caller should
   keep running heartbeat-only rather than park. */
bool sysusage_publisher_init(rclc_support_t *support,
                             rcl_node_t *node,
                             rcl_allocator_t *allocator,
                             const char *topic_name);

/* Facts main.c knows and sysusage.c cannot discover. Pass NULL to clear. */
typedef struct {
    uint32_t uptime_ms;      /* HAL_GetTick() snapshot at publish time */
    uint32_t pool_size;      /* static RMW pool bytes (main.c POOL_SIZE) */
    uint32_t pool_used;      /* main.c pool_used */
    uint32_t video_fps_x100; /* achieved emulated video fps x100, 0 = n/a */
} sysusage_extra_t;
void sysusage_set_extra(const sysusage_extra_t *extra);

/* Start the sampler task. Must be called BEFORE vTaskStartScheduler().
   Returns true if a task is running. Returns false (not an error) when
   NCAP_FREERTOS == 0: sysusage_publish() then samples inline instead. */
bool sysusage_task_start(void);

/* Fill + publish one DiagnosticArray. Call ONLY from the rclc executor's
   thread (the existing timer callback). Never from a task. */
rcl_ret_t sysusage_publish(void);

/* Last published message, for tests/debug. NULL before first publish. */
const diagnostic_msgs__msg__DiagnosticArray *sysusage_last(void);

#endif /* NCAP_SYSUSAGE == 1 */
#endif /* NCAP_SYSUSAGE_H */
