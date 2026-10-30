#ifndef ODOMETER_BENCH_H
#define ODOMETER_BENCH_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * Lifted-chassis odometer bench test.
 *
 * The test owns USART3 exclusively. Do not start the legacy USART3 RX DMA
 * while this test is enabled.
 */

typedef enum {
    ODOM_BENCH_BASELINE = 0,
    ODOM_BENCH_WAIT_START,
    ODOM_BENCH_MOVE_OUT,
    ODOM_BENCH_HOLD_OUT,
    ODOM_BENCH_MOVE_BACK,
    ODOM_BENCH_DONE,
    ODOM_BENCH_ERROR
} OdometerBench_State_t;

typedef enum {
    ODOM_BENCH_ERR_NONE = 0,
    ODOM_BENCH_ERR_BASELINE_TIMEOUT,
    ODOM_BENCH_ERR_FEEDBACK_LOST,
    ODOM_BENCH_ERR_MOTION_TIMEOUT,
    ODOM_BENCH_ERR_IMU_TIMEOUT
} OdometerBench_Error_t;

typedef struct {
    uint8_t address;
    uint8_t fresh;
    uint8_t miss_streak;
    uint8_t reserved;
    int64_t raw_count;
    int64_t logical_count;
    int64_t delta_count;
    float rpm;
} OdometerBench_Wheel_t;

typedef struct {
    OdometerBench_State_t state;
    OdometerBench_Error_t error;
    uint32_t cycle_count;
    uint32_t overrun_count;
    uint32_t bus_time_ms;
    uint32_t bus_time_max_ms;
    uint8_t fresh_mask;
    uint8_t baseline_ok;
    uint8_t settled_cycles;
    uint8_t stop_sent;
    uint8_t imu_online;
    uint8_t fusion_ready;
    uint8_t probe_address;
    uint8_t probe_pass_count;
    uint16_t probe_reply_mask;
    uint32_t imu_frame_count;
    uint32_t imu_age_ms;
    OdometerBench_Wheel_t wheel[4];
    int64_t probe_raw_count[8];
    float x_mm;
    float y_mm;
    float yaw_deg;              /* fused yaw */
    float wheel_yaw_deg;
    float imu_yaw_deg;
    float vx_mm_s;
    float vy_mm_s;
    float wz_deg_s;
    float target_x_mm;
    float heading_target_deg;
    float heading_error_deg;
    float command_x_rpm;
    float command_z_rpm;
    float position_error_mm;
} OdometerBench_Debug_t;

/* Add this symbol to the Keil Watch window while testing. */
extern volatile OdometerBench_Debug_t g_odom_bench;

void OdometerBench_Init(void);
void OdometerBench_Task(void);
void OdometerBench_EmergencyStop(void);

#endif
