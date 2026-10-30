#include "odometer_bench.h"
#include "chassis_direction.h"

#include "Emm_V5.h"
#include "hwt101.h"
#include "usart.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---------------- Parameters to verify on the real chassis ---------------- */
#define ODOM_TEST_PERIOD_MS                 20U
#define ODOM_TEST_QUERY_TIMEOUT_MS           4U
#define ODOM_TEST_INTERFRAME_GUARD_MS         1U
#define ODOM_TEST_BASELINE_TIMEOUT_MS     5000U
#define ODOM_TEST_START_DELAY_MS          1500U
#define ODOM_TEST_HOLD_MS                 1000U
#define ODOM_TEST_MOTION_TIMEOUT_MS      12000U
#define ODOM_TEST_FEEDBACK_LOST_CYCLES       3U
#define ODOM_TEST_BASELINE_GOOD_CYCLES       5U
#define ODOM_TEST_SETTLED_CYCLES             8U

#define ODOM_ENCODER_COUNTS_PER_REV     65536.0f
#define ODOM_WHEEL_DIAMETER_MM             77.0f
#define ODOM_HALF_LENGTH_PLUS_WIDTH_MM    168.9f

#define ODOM_TEST_DISTANCE_MM             2100.0f
#define ODOM_TEST_MAX_RPM                  150.0f
#define ODOM_TEST_RAMP_RPM_PER_S           300.0f
#define ODOM_TEST_BRAKE_DECEL_MM_S2        1100.0f
#define ODOM_TEST_MIN_APPROACH_RPM            3.0f
#define ODOM_TEST_POSITION_TOLERANCE_MM      2.0f
#define ODOM_TEST_SPEED_TOLERANCE_MM_S       5.0f
#define ODOM_IMU_TIMEOUT_MS                 200U
#define ODOM_IMU_WHEEL_ALPHA                  0.85f
#define ODOM_IMU_YAW_SIGN                     1.0f
#define ODOM_TEST_HEADING_TARGET_DEG           0.0f
#define ODOM_TEST_HEADING_KP_RPM_PER_DEG       1.5f
#define ODOM_TEST_HEADING_KD_RPM_PER_DEG_S     0.08f
#define ODOM_TEST_HEADING_MAX_RPM             25.0f
#define ODOM_TEST_HEADING_RAMP_RPM_PER_S      150.0f
#define ODOM_TEST_HEADING_TOLERANCE_DEG         1.0f
#define ODOM_TEST_YAW_RATE_TOLERANCE_DEG_S      3.0f

/* 独立测试也共用地址映射；本文件原有方向/控制标定仍独立于正式导航。 */
static const uint8_t s_motor_address[4] = CHASSIS_MOTOR_ADDRESS_LIST;

/*
 * Existing chassis code treats dir=1 as positive logical motor motion.
 * The 0x36 protocol encodes dir=1 using a negative sign, so invert all four
 * encoder values to keep feedback in the same logical coordinates as commands.
 * If a wheel's logical_count decreases under a positive command, change only
 * that wheel's entry to +1 after checking the wiring/address mapping.
 */
static const int8_t s_encoder_sign[4] = {-1, -1, -1, -1};

volatile OdometerBench_Debug_t g_odom_bench;

static uint32_t s_next_cycle_ms;
static uint32_t s_state_start_ms;
static uint32_t s_init_ms;
static uint8_t s_baseline_good_cycles;
static uint8_t s_probe_address = 1U;
static int64_t s_zero_count[4];
static int64_t s_last_count[4];
static uint32_t s_last_count_ms[4];
static HWT101_Snapshot_t s_imu_snapshot;
static float s_imu_zero_deg;
static float s_fused_yaw_deg;
static float s_last_wheel_yaw_deg;

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float SlewFloat(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return current + delta;
}

static float WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static void DrainMotorRx(void)
{
    volatile uint32_t dummy;

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
        dummy = huart3.Instance->DR;
        (void)dummy;
    }
}

/* Read one matching [addr, 0x36, sign, count(4), 0x6B] response. */
static uint8_t ReadPositionOnce(uint8_t address, int64_t *position_count)
{
    uint8_t query[3] = {address, 0x36U, 0x6BU};
    uint8_t frame[8];
    uint8_t byte;
    uint8_t index = 0U;
    uint32_t start_ms;

    if (position_count == NULL) return 0U;

    DrainMotorRx();
    if (HAL_UART_Transmit(&huart3, query, sizeof(query), 2U) != HAL_OK) {
        return 0U;
    }

    start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_ms) <
           ODOM_TEST_QUERY_TIMEOUT_MS) {
        if (HAL_UART_Receive(&huart3, &byte, 1U, 1U) != HAL_OK) {
            continue;
        }

        if (index == 0U) {
            if (byte == address) {
                frame[index++] = byte;
            }
            continue;
        }
        if (index == 1U) {
            if (byte == 0x36U) {
                frame[index++] = byte;
            } else {
                index = (byte == address) ? 1U : 0U;
            }
            continue;
        }

        frame[index++] = byte;
        if (index == sizeof(frame)) {
            uint32_t magnitude;
            if (frame[7] != 0x6BU || frame[2] > 1U) {
                index = 0U;
                continue;
            }
            magnitude = ((uint32_t)frame[3] << 24) |
                        ((uint32_t)frame[4] << 16) |
                        ((uint32_t)frame[5] << 8)  |
                        ((uint32_t)frame[6]);
            *position_count = (frame[2] != 0U)
                            ? -(int64_t)magnitude
                            :  (int64_t)magnitude;
            /*
             * IDLE/RX complete only means the last byte arrived. The shared
             * motor bus still needs a short release/turnaround guard before
             * addressing the next driver. Without this, every query directly
             * following a successful reply can be lost (observed as 0x05).
             */
            HAL_Delay(ODOM_TEST_INTERFRAME_GUARD_MS);
            return 1U;
        }
    }
    return 0U;
}

static void SendOneLogicalRpm(uint8_t address, float logical_rpm)
{
    uint8_t direction;
    uint16_t magnitude;

    logical_rpm = ClampFloat(logical_rpm,
                             -ODOM_TEST_MAX_RPM, ODOM_TEST_MAX_RPM);
    direction = (logical_rpm >= 0.0f) ? 1U : 0U;
    magnitude = (uint16_t)(fabsf(logical_rpm) + 0.5f);
    Emm_V5_Vel_Control(address, direction, magnitude, 0U, true);
    HAL_Delay(1U);
}

/* Inverse of the mixer already used by motor_Omnidirectional_set(). */
static void SendBodyRpm(float x_rpm, float z_rpm)
{
    float motor_rpm[4] = {
         x_rpm + z_rpm,
        -x_rpm + z_rpm,
        -x_rpm + z_rpm,
         x_rpm + z_rpm
    };
    float max_abs = 0.0f;
    float scale = 1.0f;
    uint8_t i;

    for (i = 0U; i < 4U; ++i) {
        float value_abs = fabsf(motor_rpm[i]);
        if (value_abs > max_abs) max_abs = value_abs;
    }
    if (max_abs > ODOM_TEST_MAX_RPM) {
        scale = ODOM_TEST_MAX_RPM / max_abs;
    }
    for (i = 0U; i < 4U; ++i) {
        SendOneLogicalRpm(s_motor_address[i], motor_rpm[i] * scale);
    }
    Emm_V5_Synchronous_motion(0U);
}

void OdometerBench_EmergencyStop(void)
{
    Emm_V5_Stop_Now(0U, false);
    g_odom_bench.command_x_rpm = 0.0f;
    g_odom_bench.command_z_rpm = 0.0f;
    g_odom_bench.stop_sent = 1U;
}

static void EnterState(OdometerBench_State_t state, uint32_t now_ms)
{
    g_odom_bench.state = state;
    g_odom_bench.settled_cycles = 0U;
    s_state_start_ms = now_ms;
}

static void EnterError(OdometerBench_Error_t error)
{
    g_odom_bench.error = error;
    g_odom_bench.state = ODOM_BENCH_ERROR;
    if (g_odom_bench.stop_sent == 0U) {
        OdometerBench_EmergencyStop();
    }
}

static uint8_t PollAllWheels(uint32_t now_ms)
{
    uint8_t i;
    uint8_t fresh_mask = 0U;

    for (i = 0U; i < 4U; ++i) {
        int64_t raw_count;
        g_odom_bench.wheel[i].fresh = 0U;

        if (ReadPositionOnce(s_motor_address[i], &raw_count) != 0U) {
            int64_t logical_count = raw_count * (int64_t)s_encoder_sign[i];
            uint32_t dt_ms = now_ms - s_last_count_ms[i];

            g_odom_bench.wheel[i].raw_count = raw_count;
            g_odom_bench.wheel[i].logical_count = logical_count;
            g_odom_bench.wheel[i].fresh = 1U;
            g_odom_bench.wheel[i].miss_streak = 0U;
            fresh_mask |= (uint8_t)(1U << i);

            if (s_last_count_ms[i] != 0U && dt_ms != 0U) {
                float sample_rpm =
                    (float)(logical_count - s_last_count[i]) * 60000.0f /
                    (ODOM_ENCODER_COUNTS_PER_REV * (float)dt_ms);
                g_odom_bench.wheel[i].rpm =
                    0.70f * g_odom_bench.wheel[i].rpm + 0.30f * sample_rpm;
            }
            s_last_count[i] = logical_count;
            s_last_count_ms[i] = now_ms;
        } else {
            if (g_odom_bench.wheel[i].miss_streak < 255U) {
                g_odom_bench.wheel[i].miss_streak++;
            }
        }
    }

    g_odom_bench.fresh_mask = fresh_mask;
    return (fresh_mask == 0x0FU) ? 1U : 0U;
}

/*
 * During baseline only, probe one address per cycle. The sticky reply mask uses
 * bit 0 for address 1 ... bit 7 for address 8. Motors are never commanded by
 * this probe, so it is safe to leave running while the chassis is lifted.
 */
static void ProbeOneMotorAddress(void)
{
    int64_t raw_count;
    uint8_t address = s_probe_address;

    g_odom_bench.probe_address = address;
    if (ReadPositionOnce(address, &raw_count) != 0U) {
        g_odom_bench.probe_reply_mask |= (uint16_t)(1U << (address - 1U));
        g_odom_bench.probe_raw_count[address - 1U] = raw_count;
    }

    s_probe_address++;
    if (s_probe_address > 8U) {
        s_probe_address = 1U;
        if (g_odom_bench.probe_pass_count < 255U) {
            g_odom_bench.probe_pass_count++;
        }
    }
}

static void UpdateImuStatus(uint32_t now_ms)
{
    HWT101_GetSnapshot(&s_imu_snapshot);
    g_odom_bench.imu_frame_count = s_imu_snapshot.frame_count;

    if (s_imu_snapshot.frame_count == 0U) {
        g_odom_bench.imu_age_ms = 0xFFFFFFFFU;
        g_odom_bench.imu_online = 0U;
    } else {
        g_odom_bench.imu_age_ms = now_ms - s_imu_snapshot.last_update_ms;
        g_odom_bench.imu_online =
            (g_odom_bench.imu_age_ms <= ODOM_IMU_TIMEOUT_MS) ? 1U : 0U;
    }
}

static void UpdateKinematics(void)
{
    float wheel_mm[4];
    float wheel_rpm[4];
    float mm_per_count =
        ((float)M_PI * ODOM_WHEEL_DIAMETER_MM) /
        ODOM_ENCODER_COUNTS_PER_REV;
    float rpm_to_mm_s =
        ((float)M_PI * ODOM_WHEEL_DIAMETER_MM) / 60.0f;
    float z_mm;
    float z_mm_s;
    uint8_t i;

    for (i = 0U; i < 4U; ++i) {
        int64_t delta = g_odom_bench.wheel[i].logical_count - s_zero_count[i];
        g_odom_bench.wheel[i].delta_count = delta;
        wheel_mm[i] = (float)delta * mm_per_count;
        wheel_rpm[i] = g_odom_bench.wheel[i].rpm;
    }

    g_odom_bench.x_mm =
        (wheel_mm[0] - wheel_mm[1] - wheel_mm[2] + wheel_mm[3]) * 0.25f;
    g_odom_bench.y_mm =
        (wheel_mm[0] + wheel_mm[1] - wheel_mm[2] - wheel_mm[3]) * 0.25f;
    z_mm =
        (wheel_mm[0] + wheel_mm[1] + wheel_mm[2] + wheel_mm[3]) * 0.25f;
    g_odom_bench.wheel_yaw_deg = WrapDeg(
        z_mm / ODOM_HALF_LENGTH_PLUS_WIDTH_MM * 180.0f / (float)M_PI);

    g_odom_bench.vx_mm_s =
        (wheel_rpm[0] - wheel_rpm[1] - wheel_rpm[2] + wheel_rpm[3]) *
        0.25f * rpm_to_mm_s;
    g_odom_bench.vy_mm_s =
        (wheel_rpm[0] + wheel_rpm[1] - wheel_rpm[2] - wheel_rpm[3]) *
        0.25f * rpm_to_mm_s;
    z_mm_s =
        (wheel_rpm[0] + wheel_rpm[1] + wheel_rpm[2] + wheel_rpm[3]) *
        0.25f * rpm_to_mm_s;
    g_odom_bench.wz_deg_s =
        z_mm_s / ODOM_HALF_LENGTH_PLUS_WIDTH_MM * 180.0f / (float)M_PI;

    {
        float wheel_delta_deg =
            WrapDeg(g_odom_bench.wheel_yaw_deg - s_last_wheel_yaw_deg);
        float predicted_deg = WrapDeg(s_fused_yaw_deg + wheel_delta_deg);

        g_odom_bench.imu_yaw_deg = WrapDeg(
            ODOM_IMU_YAW_SIGN * (s_imu_snapshot.yaw_deg - s_imu_zero_deg));
        if (g_odom_bench.imu_online != 0U) {
            float correction_deg =
                WrapDeg(g_odom_bench.imu_yaw_deg - predicted_deg);
            s_fused_yaw_deg = WrapDeg(
                predicted_deg + (1.0f - ODOM_IMU_WHEEL_ALPHA) *
                correction_deg);
        } else {
            s_fused_yaw_deg = predicted_deg;
        }
        s_last_wheel_yaw_deg = g_odom_bench.wheel_yaw_deg;
        g_odom_bench.yaw_deg = s_fused_yaw_deg;
        g_odom_bench.heading_error_deg = WrapDeg(
            ODOM_TEST_HEADING_TARGET_DEG - g_odom_bench.yaw_deg);
    }
}

static uint8_t FeedbackLost(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; ++i) {
        if (g_odom_bench.wheel[i].miss_streak >=
            ODOM_TEST_FEEDBACK_LOST_CYCLES) {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t MotionStep(float target_x_mm, uint8_t all_fresh)
{
    float desired_rpm = 0.0f;
    float desired_z_rpm = 0.0f;
    float ramp_step = ODOM_TEST_RAMP_RPM_PER_S *
                      ((float)ODOM_TEST_PERIOD_MS / 1000.0f);
    float heading_ramp_step = ODOM_TEST_HEADING_RAMP_RPM_PER_S *
                              ((float)ODOM_TEST_PERIOD_MS / 1000.0f);
    float error = target_x_mm - g_odom_bench.x_mm;

    g_odom_bench.target_x_mm = target_x_mm;
    g_odom_bench.position_error_mm = error;

    if (all_fresh != 0U) {
        float remaining_mm = fabsf(error) - ODOM_TEST_POSITION_TOLERANCE_MM;

        if (remaining_mm > 0.0f) {
            float profile_mm_s =
                sqrtf(2.0f * ODOM_TEST_BRAKE_DECEL_MM_S2 * remaining_mm);
            float profile_rpm =
                profile_mm_s * 60.0f /
                ((float)M_PI * ODOM_WHEEL_DIAMETER_MM);

            profile_rpm = ClampFloat(profile_rpm,
                                     ODOM_TEST_MIN_APPROACH_RPM,
                                     ODOM_TEST_MAX_RPM);
            desired_rpm = (error >= 0.0f) ? profile_rpm : -profile_rpm;
        }

        if (g_odom_bench.fusion_ready != 0U) {
            desired_z_rpm =
                ODOM_TEST_HEADING_KP_RPM_PER_DEG *
                g_odom_bench.heading_error_deg -
                ODOM_TEST_HEADING_KD_RPM_PER_DEG_S *
                g_odom_bench.wz_deg_s;
            desired_z_rpm = ClampFloat(desired_z_rpm,
                                       -ODOM_TEST_HEADING_MAX_RPM,
                                       ODOM_TEST_HEADING_MAX_RPM);
        }
    }
    g_odom_bench.command_x_rpm =
        SlewFloat(g_odom_bench.command_x_rpm, desired_rpm, ramp_step);
    g_odom_bench.command_z_rpm =
        SlewFloat(g_odom_bench.command_z_rpm,
                  desired_z_rpm, heading_ramp_step);
    SendBodyRpm(g_odom_bench.command_x_rpm,
                g_odom_bench.command_z_rpm);

    if (all_fresh != 0U &&
        fabsf(error) <= ODOM_TEST_POSITION_TOLERANCE_MM &&
        fabsf(g_odom_bench.vx_mm_s) <= ODOM_TEST_SPEED_TOLERANCE_MM_S &&
        fabsf(g_odom_bench.heading_error_deg) <=
            ODOM_TEST_HEADING_TOLERANCE_DEG &&
        fabsf(g_odom_bench.wz_deg_s) <=
            ODOM_TEST_YAW_RATE_TOLERANCE_DEG_S) {
        if (g_odom_bench.settled_cycles < 255U) {
            g_odom_bench.settled_cycles++;
        }
    } else {
        g_odom_bench.settled_cycles = 0U;
    }
    return (g_odom_bench.settled_cycles >= ODOM_TEST_SETTLED_CYCLES) ? 1U : 0U;
}

void OdometerBench_Init(void)
{
    uint8_t i;

    memset((void *)&g_odom_bench, 0, sizeof(g_odom_bench));
    for (i = 0U; i < 4U; ++i) {
        g_odom_bench.wheel[i].address = s_motor_address[i];
    }

    g_odom_bench.state = ODOM_BENCH_BASELINE;
    g_odom_bench.error = ODOM_BENCH_ERR_NONE;
    g_odom_bench.heading_target_deg = ODOM_TEST_HEADING_TARGET_DEG;
    s_baseline_good_cycles = 0U;
    s_fused_yaw_deg = 0.0f;
    s_last_wheel_yaw_deg = 0.0f;
    s_init_ms = HAL_GetTick();
    s_state_start_ms = s_init_ms;
    s_next_cycle_ms = s_init_ms;

    /* Make sure no legacy DMA reception is holding USART3. */
    (void)HAL_UART_DMAStop(&huart3);
    DrainMotorRx();

    /* A controller reset must not leave a previous speed command running. */
    Emm_V5_Stop_Now(0U, false);
    g_odom_bench.stop_sent = 1U;
    HAL_Delay(2U);

    /* Drivers normally retain enable state, but explicit enable is repeatable. */
    Emm_V5_En_Control(0U, true, false);
    HAL_Delay(2U);
}

void OdometerBench_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t bus_start_ms;
    uint8_t all_fresh;
    uint8_t i;

    if ((int32_t)(now_ms - s_next_cycle_ms) < 0) {
        return;
    }
    if ((uint32_t)(now_ms - s_next_cycle_ms) > ODOM_TEST_PERIOD_MS) {
        g_odom_bench.overrun_count++;
        s_next_cycle_ms = now_ms + ODOM_TEST_PERIOD_MS;
    } else {
        s_next_cycle_ms += ODOM_TEST_PERIOD_MS;
    }

    bus_start_ms = HAL_GetTick();
    all_fresh = PollAllWheels(now_ms);
    UpdateImuStatus(now_ms);
    if (g_odom_bench.baseline_ok == 0U) {
        ProbeOneMotorAddress();
    }
    g_odom_bench.bus_time_ms = HAL_GetTick() - bus_start_ms;
    if (g_odom_bench.bus_time_ms > g_odom_bench.bus_time_max_ms) {
        g_odom_bench.bus_time_max_ms = g_odom_bench.bus_time_ms;
    }
    g_odom_bench.cycle_count++;

    if (g_odom_bench.baseline_ok == 0U) {
        if (all_fresh != 0U && g_odom_bench.imu_online != 0U) {
            if (++s_baseline_good_cycles >= ODOM_TEST_BASELINE_GOOD_CYCLES) {
                for (i = 0U; i < 4U; ++i) {
                    s_zero_count[i] = g_odom_bench.wheel[i].logical_count;
                }
                s_imu_zero_deg = s_imu_snapshot.yaw_deg;
                s_fused_yaw_deg = 0.0f;
                s_last_wheel_yaw_deg = 0.0f;
                g_odom_bench.fusion_ready = 1U;
                g_odom_bench.baseline_ok = 1U;
                EnterState(ODOM_BENCH_WAIT_START, now_ms);
            }
        } else {
            s_baseline_good_cycles = 0U;
        }

        if ((uint32_t)(now_ms - s_init_ms) >
            ODOM_TEST_BASELINE_TIMEOUT_MS) {
            EnterError((g_odom_bench.imu_online == 0U)
                       ? ODOM_BENCH_ERR_IMU_TIMEOUT
                       : ODOM_BENCH_ERR_BASELINE_TIMEOUT);
        }
        return;
    }

    UpdateKinematics();

    if (FeedbackLost() != 0U) {
        EnterError(ODOM_BENCH_ERR_FEEDBACK_LOST);
        return;
    }
    /* HWT101 short dropout is not fatal after initial alignment. The fusion
     * predictor continues from wheel yaw and automatically resumes IMU
     * correction when valid angle frames return. */

    switch (g_odom_bench.state) {
    case ODOM_BENCH_WAIT_START:
        if ((uint32_t)(now_ms - s_state_start_ms) >=
            ODOM_TEST_START_DELAY_MS) {
            g_odom_bench.stop_sent = 0U;
            EnterState(ODOM_BENCH_MOVE_OUT, now_ms);
        }
        break;

    case ODOM_BENCH_MOVE_OUT:
        if (MotionStep(ODOM_TEST_DISTANCE_MM, all_fresh) != 0U) {
            OdometerBench_EmergencyStop();
            EnterState(ODOM_BENCH_HOLD_OUT, now_ms);
        } else if ((uint32_t)(now_ms - s_state_start_ms) >
                   ODOM_TEST_MOTION_TIMEOUT_MS) {
            EnterError(ODOM_BENCH_ERR_MOTION_TIMEOUT);
        }
        break;

    case ODOM_BENCH_HOLD_OUT:
        if ((uint32_t)(now_ms - s_state_start_ms) >= ODOM_TEST_HOLD_MS) {
            g_odom_bench.stop_sent = 0U;
            EnterState(ODOM_BENCH_MOVE_BACK, now_ms);
        }
        break;

    case ODOM_BENCH_MOVE_BACK:
        if (MotionStep(0.0f, all_fresh) != 0U) {
            OdometerBench_EmergencyStop();
            EnterState(ODOM_BENCH_DONE, now_ms);
        } else if ((uint32_t)(now_ms - s_state_start_ms) >
                   ODOM_TEST_MOTION_TIMEOUT_MS) {
            EnterError(ODOM_BENCH_ERR_MOTION_TIMEOUT);
        }
        break;

    case ODOM_BENCH_DONE:
    case ODOM_BENCH_ERROR:
    default:
        break;
    }
}
