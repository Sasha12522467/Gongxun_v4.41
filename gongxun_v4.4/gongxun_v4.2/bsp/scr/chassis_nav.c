#include "chassis_nav.h"
#include "chassis_direction.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 地图原点在场地右上角，车辆初始停在300x300启停区1的中心。 */
#define CHASSIS_INITIAL_X_MM 150.0f
#define CHASSIS_INITIAL_Y_MM 150.0f

/* 到点后收完平移速度指令再宣布完成，避免DONE时直接截断运动。 */
#define NAV_STOP_COMMAND_MM_S 1.0f

typedef enum {
    NAV_MODE_NONE = 0,
    NAV_MODE_GOTO_XY,
    NAV_MODE_TURN
} ChassisNav_Mode_t;

typedef struct {
    ChassisNav_Config_t config;
    ChassisNav_State_t state;
    ChassisNav_Mode_t mode;
    float move_max_rpm;
    int64_t last_logical_count[CHASSIS_NAV_WHEEL_COUNT];
    float fused_yaw_deg;
    float imu_offset_deg;
    uint8_t initialized;
    uint8_t imu_initialized;
} ChassisNav_Context_t;

static ChassisNav_Context_t s_nav;

static float Nav_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float Nav_Slew(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return current + delta;
}

static float Nav_WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/* 距离是到容差边界的剩余距离，不是到目标中心的距离。
 * 全程使用同一个固定减速度A：v=sqrt(2*A*d)。
 * 不分高速/低速制动段，也不再按低速阈值减轻制动力。
 */
static float Nav_BrakeProfile(float remaining_mm)
{
    float a = s_nav.config.brake_deceleration_mm_s2;
    if (remaining_mm <= 0.0f) return 0.0f;
    return sqrtf(2.0f * a * remaining_mm);
}

/* 加速和制动分别限幅：加速用RPM/s，制动用固定mm/s²换算为RPM步长。
 * 制动不再受起步加速度上限限制，也不因当前速度降低而变柔。
 * 矢量限幅避免斜行时两个轴叠加超限；故障/急停仍立即停止。
 */
static void Nav_SlewTranslation(float vx, float vy, float dt_s)
{
    float current = hypotf(s_nav.state.command_vx_rpm,
                           s_nav.state.command_vy_rpm);
    float target = hypotf(vx, vy);
    float dx = vx - s_nav.state.command_vx_rpm;
    float dy = vy - s_nav.state.command_vy_rpm;
    float delta = hypotf(dx, dy);
    float step = s_nav.config.acceleration_rpm_s * dt_s;
    if (target < current) {
        float mm_per_rpm = (float)M_PI * s_nav.config.wheel_diameter_mm / 60.0f;
        step = s_nav.config.brake_deceleration_mm_s2 * dt_s / mm_per_rpm;
    }
    if (delta > step) {
        dx *= step / delta;
        dy *= step / delta;
    }
    s_nav.state.command_vx_rpm += dx;
    s_nav.state.command_vy_rpm += dy;
}

static uint8_t Nav_ConfigValid(const ChassisNav_Config_t *config)
{
    uint8_t i;
    if (config == NULL ||
        !isfinite(config->wheel_diameter_mm) ||
        !isfinite(config->encoder_counts_per_rev) ||
        !isfinite(config->half_length_plus_width_mm) ||
        !isfinite(config->max_wheel_rpm) ||
        !isfinite(config->acceleration_rpm_s) ||
        !isfinite(config->brake_deceleration_mm_s2) ||
        config->wheel_diameter_mm <= 0.0f ||
        config->encoder_counts_per_rev <= 0.0f ||
        config->half_length_plus_width_mm <= 0.0f ||
        config->max_wheel_rpm <= 0.0f ||
        config->acceleration_rpm_s <= 0.0f ||
        config->brake_deceleration_mm_s2 <= 0.0f ||
        config->settled_cycles_required == 0U ||
        config->encoder_lost_cycles == 0U) {
        return 0U;
    }
    for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
        if (config->encoder_sign[i] != 1 &&
            config->encoder_sign[i] != -1) {
            return 0U;
        }
    }
    return 1U;
}

void ChassisNav_GetDefaultConfig(ChassisNav_Config_t *config)
{
    uint8_t i;
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));

    config->wheel_diameter_mm = 77.0f;
    config->encoder_counts_per_rev = 65536.0f;
    config->half_length_plus_width_mm = 168.9f;
    for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
        config->encoder_sign[i] = -1;
    }

    /* 跑图提速：150 RPM；加速度230 RPM/s，约0.65秒升至最高轮速。
     * 制动全程固定1000 mm/s²，不再使用低速柔和段。
     * 制动距离由v²/(2*A)决定；实车若打滑/冲过目标，应降低A。
     */
    config->max_wheel_rpm = 190.0f;
    config->acceleration_rpm_s = 280.0f;
    config->brake_deceleration_mm_s2 = 1100.0f;
    config->brake_soft_speed_mm_s = 0.0f; /* 仅兼容旧配置，不参与控制。 */
    config->min_approach_rpm = 4.0f;      //还没进入位置容差时，平移接近速度的最低值，防止速度太低而迟迟走不到位
		
    config->position_tolerance_mm = 4.0f;   //当前位置到目标的直线距离≤2mm，不是X、Y各自±2mm
    config->speed_tolerance_mm_s = 1.0f;    //实际平移速度≤5mm/s
		
    config->heading_kp_rpm_per_deg = 1.5f;   //比例系数：每差1°，产生1.5 RPM的旋转修正量
    config->heading_kd_rpm_per_deg_s = 0.08f;  //角速度阻尼系数，用当前旋转速度抑制过冲
    config->heading_max_rpm = 25.0f;            //旋转速度分量的限幅；不是车体每分钟转25圈
		
    config->heading_tolerance_deg = 1.0f;    //航向误差绝对值≤1°
    config->yaw_rate_tolerance_deg_s = 3.0f;  //实际旋转角速度绝对值≤3°/s
    config->fusion_wheel_alpha = 0.85f;       //控制编码器预测航向和IMU航向的融合比例：
    config->settled_cycles_required = 8U;     //连续8个更新周期满足到位条件，才宣布完成
    /*
     * 位置模式同步启动后的短时间内，驱动器可能延迟累计位置应答。
     * 20 ms控制周期下允许连续丢失10次（约200 ms），避免把瞬时忙碌
     * 误判为永久掉线；超过该时间仍无完整四轮反馈则进入安全故障。
     */
    config->encoder_lost_cycles = 10U;
}

bool ChassisNav_Init(const ChassisNav_Config_t *config,
                     const ChassisNav_Feedback_t *initial_feedback)
{
    uint8_t i;
    if (Nav_ConfigValid(config) == 0U || initial_feedback == NULL ||
        initial_feedback->encoder_valid_mask !=
        CHASSIS_NAV_ENCODER_ALL_VALID) {
        return false;
    }

    memset(&s_nav, 0, sizeof(s_nav));
    s_nav.config = *config;
    s_nav.move_max_rpm = config->max_wheel_rpm;
    s_nav.state.pose.x_mm = CHASSIS_INITIAL_X_MM;
    s_nav.state.pose.y_mm = CHASSIS_INITIAL_Y_MM;
    for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
        s_nav.last_logical_count[i] =
            initial_feedback->motor_count[i] *
            (int64_t)s_nav.config.encoder_sign[i];
    }

    if (initial_feedback->imu_valid != 0U &&
        isfinite(initial_feedback->imu_yaw_deg)) {
        s_nav.imu_offset_deg = Nav_WrapDeg(initial_feedback->imu_yaw_deg);
        s_nav.imu_initialized = 1U;
        s_nav.state.imu_fused = 1U;
    }

    s_nav.state.status = CHASSIS_NAV_IDLE;
    s_nav.state.fault = CHASSIS_NAV_FAULT_NONE;
    s_nav.initialized = 1U;
    return true;
}

static void Nav_SetTarget(float x_mm, float y_mm, float yaw_deg,
                          ChassisNav_Mode_t mode)
{
    s_nav.state.target_x_mm = x_mm;
    s_nav.state.target_y_mm = y_mm;
    s_nav.state.target_yaw_deg = Nav_WrapDeg(yaw_deg);
    s_nav.state.settled_cycles = 0U;
    s_nav.state.status = CHASSIS_NAV_RUNNING;
    s_nav.state.fault = CHASSIS_NAV_FAULT_NONE;
    s_nav.mode = mode;
    s_nav.move_max_rpm = s_nav.config.max_wheel_rpm;
    s_nav.state.transit_mode = 0U;
}

bool ChassisNav_GotoXY(float x_mm, float y_mm, float final_yaw_deg)
{
    if (s_nav.initialized == 0U || !isfinite(x_mm) || !isfinite(y_mm) ||
        !isfinite(final_yaw_deg)) return false;
    Nav_SetTarget(x_mm, y_mm, final_yaw_deg, NAV_MODE_GOTO_XY);
    return true;
}

bool ChassisNav_MoveRelativeWorld(float dx_mm, float dy_mm,
                                  float final_yaw_deg)
{
    if (s_nav.initialized == 0U || !isfinite(dx_mm) || !isfinite(dy_mm) ||
        !isfinite(final_yaw_deg)) return false;
    return ChassisNav_GotoXY(s_nav.state.pose.x_mm + dx_mm,
                             s_nav.state.pose.y_mm + dy_mm,
                             final_yaw_deg);
}

bool ChassisNav_GotoXYLimited(float x_mm, float y_mm, float final_yaw_deg,
                              float max_rpm)
{
    if (!isfinite(max_rpm) || max_rpm <= 0.0f) return false;
    if (!ChassisNav_GotoXY(x_mm, y_mm, final_yaw_deg)) return false;
    s_nav.move_max_rpm = fminf(max_rpm, s_nav.config.max_wheel_rpm);
    return true;
}

bool ChassisNav_MoveRelativeBody(float forward_mm, float left_mm,
                                 float final_yaw_deg)
{
    float yaw_rad;
    float dx_world;
    float dy_world;
    if (s_nav.initialized == 0U || !isfinite(forward_mm) ||
        !isfinite(left_mm) || !isfinite(final_yaw_deg)) return false;

    yaw_rad = s_nav.state.pose.yaw_deg * ((float)M_PI / 180.0f);
    /*
     * 地图原点在右上角，X+向左、Y+向下；车辆初始朝Y+。
     * yaw=0时：车体前进对应地图Y+，车体左移对应地图X-。
     */
    dx_world = -forward_mm * sinf(yaw_rad) -
                left_mm * cosf(yaw_rad);
    dy_world =  forward_mm * cosf(yaw_rad) -
                left_mm * sinf(yaw_rad);
    return ChassisNav_MoveRelativeWorld(dx_world, dy_world, final_yaw_deg);
}

bool ChassisNav_TurnTo(float yaw_deg)
{
    if (s_nav.initialized == 0U || !isfinite(yaw_deg)) return false;
    Nav_SetTarget(s_nav.state.pose.x_mm, s_nav.state.pose.y_mm,
                  yaw_deg, NAV_MODE_TURN);
    return true;
}

void ChassisNav_SetEncoderFaultSuppressed(bool suppressed)
{
    s_nav.state.encoder_fault_suppressed = suppressed ? 1U : 0U;
    /* 恢复保护时从新周期重新计数，避免沿用粗转期间的历史丢帧。 */
    if (!suppressed) {
        s_nav.state.encoder_miss_streak = 0U;
    }
}

void ChassisNav_SetTransitMode(bool enabled)
{
    s_nav.state.transit_mode = enabled ? 1U : 0U;
    s_nav.state.settled_cycles = 0U;
}

void ChassisNav_Stop(void)
{
    if (s_nav.initialized == 0U) return;
    s_nav.mode = NAV_MODE_NONE;
    s_nav.state.command_vx_rpm = 0.0f;
    s_nav.state.command_vy_rpm = 0.0f;
    s_nav.state.command_wz_rpm = 0.0f;
    s_nav.state.status = CHASSIS_NAV_IDLE;
    s_nav.state.settled_cycles = 0U;
}

static void Nav_SetFault(ChassisNav_Fault_t fault)
{
    s_nav.state.fault = fault;
    s_nav.state.status = CHASSIS_NAV_FAULT;
    s_nav.mode = NAV_MODE_NONE;
    s_nav.state.command_vx_rpm = 0.0f;
    s_nav.state.command_vy_rpm = 0.0f;
    s_nav.state.command_wz_rpm = 0.0f;
}

static void Nav_UpdateOdometry(const ChassisNav_Feedback_t *feedback,
                               float dt_s)
{
    float wheel_delta_mm[CHASSIS_NAV_WHEEL_COUNT];
    float mm_per_count =
        ((float)M_PI * s_nav.config.wheel_diameter_mm) /
        s_nav.config.encoder_counts_per_rev;
    float dx_body;
    float dy_body;
    float dtheta_deg;
    float previous_yaw_deg = s_nav.fused_yaw_deg;
    float predicted_yaw_deg;
    float yaw_mid_rad;
    uint8_t i;

    for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
        int64_t logical_count = feedback->motor_count[i] *
                                (int64_t)s_nav.config.encoder_sign[i];
        int64_t delta_count = logical_count - s_nav.last_logical_count[i];
        s_nav.last_logical_count[i] = logical_count;
        wheel_delta_mm[i] = (float)delta_count * mm_per_count;
    }

    dx_body = (wheel_delta_mm[0] - wheel_delta_mm[1] -
               wheel_delta_mm[2] + wheel_delta_mm[3]) * 0.25f;
    dy_body = (wheel_delta_mm[0] + wheel_delta_mm[1] -
               wheel_delta_mm[2] - wheel_delta_mm[3]) * 0.25f;
    /* 与输出混控使用同一方向校准，确保实车前进时里程计也记为前进。
     * 编码器原始协议符号不改，转角仍由四轮和量计算。
     */
    dx_body *= (float)CHASSIS_FORWARD_SIGN;
    dy_body *= (float)CHASSIS_LATERAL_SIGN;
    dtheta_deg =
        (wheel_delta_mm[0] + wheel_delta_mm[1] +
         wheel_delta_mm[2] + wheel_delta_mm[3]) * 0.25f /
        s_nav.config.half_length_plus_width_mm *
        (180.0f / (float)M_PI);

    s_nav.state.wheel_yaw_deg =
        Nav_WrapDeg(s_nav.state.wheel_yaw_deg + dtheta_deg);
    predicted_yaw_deg = Nav_WrapDeg(previous_yaw_deg + dtheta_deg);

    if (feedback->imu_valid != 0U && isfinite(feedback->imu_yaw_deg)) {
        if (s_nav.imu_initialized == 0U) {
            s_nav.imu_offset_deg =
                Nav_WrapDeg(feedback->imu_yaw_deg - predicted_yaw_deg);
            s_nav.imu_initialized = 1U;
        }
        s_nav.state.imu_yaw_deg =
            Nav_WrapDeg(feedback->imu_yaw_deg - s_nav.imu_offset_deg);
        s_nav.fused_yaw_deg = Nav_WrapDeg(
            predicted_yaw_deg +
            (1.0f - s_nav.config.fusion_wheel_alpha) *
            Nav_WrapDeg(s_nav.state.imu_yaw_deg - predicted_yaw_deg));
        s_nav.state.imu_fused = 1U;
    } else {
        s_nav.fused_yaw_deg = predicted_yaw_deg;
        s_nav.state.imu_fused = 0U;
    }

    yaw_mid_rad = Nav_WrapDeg(previous_yaw_deg +
        0.5f * Nav_WrapDeg(s_nav.fused_yaw_deg - previous_yaw_deg)) *
        ((float)M_PI / 180.0f);
    /*
     * 机体增量转换到地图坐标。
     * yaw=0时，前进增加地图Y，向左移动减小地图X。
     */
    s_nav.state.pose.x_mm +=
        -dx_body * sinf(yaw_mid_rad) - dy_body * cosf(yaw_mid_rad);
    s_nav.state.pose.y_mm +=
         dx_body * cosf(yaw_mid_rad) - dy_body * sinf(yaw_mid_rad);
    s_nav.state.pose.yaw_deg = s_nav.fused_yaw_deg;

    s_nav.state.velocity.vx_mm_s = dx_body / dt_s;
    s_nav.state.velocity.vy_mm_s = dy_body / dt_s;
    s_nav.state.velocity.wz_deg_s =
        Nav_WrapDeg(s_nav.fused_yaw_deg - previous_yaw_deg) / dt_s;
}

static void Nav_MixAndLimit(float vx_rpm, float vy_rpm, float wz_rpm,
                            ChassisNav_Output_t *output)
{
    float max_abs = 0.0f;
    float scale = 1.0f;
    uint8_t i;

    /* 仅修正平移基向量；wz保持不变，避免把转向方向一起反掉。 */
    vx_rpm *= (float)CHASSIS_FORWARD_SIGN;
    vy_rpm *= (float)CHASSIS_LATERAL_SIGN;
    output->wheel_rpm[0] =  vx_rpm + vy_rpm + wz_rpm;
    output->wheel_rpm[1] = -vx_rpm + vy_rpm + wz_rpm;
    output->wheel_rpm[2] = -vx_rpm - vy_rpm + wz_rpm;
    output->wheel_rpm[3] =  vx_rpm - vy_rpm + wz_rpm;

    for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
        float value_abs = fabsf(output->wheel_rpm[i]);
        if (value_abs > max_abs) max_abs = value_abs;
    }
    if (max_abs > s_nav.move_max_rpm) {
        scale = s_nav.move_max_rpm / max_abs;
    }
    for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
        output->wheel_rpm[i] *= scale;
    }
}

static void Nav_RunController(float dt_s, ChassisNav_Output_t *output)
{
    float desired_vx_rpm = 0.0f;
    float desired_vy_rpm = 0.0f;
    float desired_wz_rpm;
    float dx_world = s_nav.state.target_x_mm - s_nav.state.pose.x_mm;
    float dy_world = s_nav.state.target_y_mm - s_nav.state.pose.y_mm;
    float distance_mm = sqrtf(dx_world * dx_world + dy_world * dy_world);
    float heading_error_deg =
        Nav_WrapDeg(s_nav.state.target_yaw_deg - s_nav.state.pose.yaw_deg);
    float slew_step = s_nav.config.acceleration_rpm_s * dt_s;
    float speed_mm_s = sqrtf(
        s_nav.state.velocity.vx_mm_s * s_nav.state.velocity.vx_mm_s +
        s_nav.state.velocity.vy_mm_s * s_nav.state.velocity.vy_mm_s);
    uint8_t position_ok;
    uint8_t heading_ok;
    float position_tolerance = s_nav.state.transit_mode ?
        CHASSIS_NAV_TRANSIT_POSITION_MM : s_nav.config.position_tolerance_mm;
    float heading_tolerance = s_nav.state.transit_mode ?
        CHASSIS_NAV_TRANSIT_HEADING_DEG : s_nav.config.heading_tolerance_deg;
    float speed_tolerance = s_nav.state.transit_mode ?
        CHASSIS_NAV_TRANSIT_SPEED_MM_S : s_nav.config.speed_tolerance_mm_s;
    float yaw_rate_tolerance = s_nav.state.transit_mode ?
        CHASSIS_NAV_TRANSIT_YAW_RATE_DEG_S : s_nav.config.yaw_rate_tolerance_deg_s;
    uint8_t stable_required = s_nav.state.transit_mode ?
        CHASSIS_NAV_TRANSIT_SETTLED_CYCLES : s_nav.config.settled_cycles_required;

    s_nav.state.position_error_mm = distance_mm;
    s_nav.state.heading_error_deg = heading_error_deg;

    if (s_nav.mode == NAV_MODE_GOTO_XY &&
        distance_mm > position_tolerance) {
        float remaining_mm =
            distance_mm - position_tolerance;
        float profile_mm_s = Nav_BrakeProfile(remaining_mm);
        float profile_rpm = profile_mm_s * 60.0f /
            ((float)M_PI * s_nav.config.wheel_diameter_mm);
        float yaw_rad = s_nav.state.pose.yaw_deg *
                        ((float)M_PI / 180.0f);
        /* 地图误差投影到机体前进/左移方向。 */
        float ex_body = -sinf(yaw_rad) * dx_world +
                         cosf(yaw_rad) * dy_world;
        float ey_body = -cosf(yaw_rad) * dx_world -
                         sinf(yaw_rad) * dy_world;

        profile_rpm = Nav_Clamp(profile_rpm,
                                s_nav.config.min_approach_rpm,
                                s_nav.move_max_rpm);
        desired_vx_rpm = profile_rpm * ex_body / distance_mm;
        desired_vy_rpm = profile_rpm * ey_body / distance_mm;
    }

    desired_wz_rpm =
        s_nav.config.heading_kp_rpm_per_deg * heading_error_deg -
        s_nav.config.heading_kd_rpm_per_deg_s *
        s_nav.state.velocity.wz_deg_s;
    desired_wz_rpm = Nav_Clamp(desired_wz_rpm,
                               -s_nav.config.heading_max_rpm,
                               s_nav.config.heading_max_rpm);

    Nav_SlewTranslation(desired_vx_rpm, desired_vy_rpm, dt_s);
    s_nav.state.command_wz_rpm = Nav_Slew(
        s_nav.state.command_wz_rpm, desired_wz_rpm, slew_step);

    Nav_MixAndLimit(s_nav.state.command_vx_rpm,
                    s_nav.state.command_vy_rpm,
                    s_nav.state.command_wz_rpm,
                    output);

    position_ok = ((s_nav.mode == NAV_MODE_TURN ||
                   distance_mm <= position_tolerance) &&
                  speed_mm_s <= speed_tolerance &&
                   /* 到容差后按固定减速度收完平移速度，再宣布完成。
                    * 航向仍按原判据，粗转/PD切换不受本次修改影响。
                   */
                  hypotf(s_nav.state.command_vx_rpm,
                         s_nav.state.command_vy_rpm) *
                      ((float)M_PI * s_nav.config.wheel_diameter_mm / 60.0f)
                      <= NAV_STOP_COMMAND_MM_S) ? 1U : 0U;
    heading_ok =
        (fabsf(heading_error_deg) <= heading_tolerance &&
         fabsf(s_nav.state.velocity.wz_deg_s) <=
             yaw_rate_tolerance) ? 1U : 0U;

    if (position_ok != 0U && heading_ok != 0U) {
        if (s_nav.state.settled_cycles < 255U) {
            s_nav.state.settled_cycles++;
        }
    } else {
        s_nav.state.settled_cycles = 0U;
    }

    if (s_nav.state.settled_cycles >= stable_required) {
        uint8_t i;
        s_nav.state.status = CHASSIS_NAV_DONE;
        s_nav.mode = NAV_MODE_NONE;
        s_nav.state.command_vx_rpm = 0.0f;
        s_nav.state.command_vy_rpm = 0.0f;
        s_nav.state.command_wz_rpm = 0.0f;
        for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
            output->wheel_rpm[i] = 0.0f;
        }
    }
}

void ChassisNav_Update(const ChassisNav_Feedback_t *feedback,
                       float dt_s,
                       ChassisNav_Output_t *output)
{
    uint8_t i;
    if (output == NULL) return;
    memset(output, 0, sizeof(*output));

    if (s_nav.initialized == 0U || feedback == NULL) {
        output->status = CHASSIS_NAV_FAULT;
        output->fault = CHASSIS_NAV_FAULT_BAD_DT;
        return;
    }
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.100f) {
        Nav_SetFault(CHASSIS_NAV_FAULT_BAD_DT);
    }

    if (feedback->encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID &&
        s_nav.state.status != CHASSIS_NAV_FAULT) {
        s_nav.state.encoder_miss_streak = 0U;
        Nav_UpdateOdometry(feedback, dt_s);
    } else if (s_nav.state.status != CHASSIS_NAV_FAULT) {
        if (s_nav.state.encoder_miss_streak < 255U) {
            s_nav.state.encoder_miss_streak++;
        }
        if (s_nav.state.encoder_fault_suppressed == 0U &&
            s_nav.state.encoder_miss_streak >=
            s_nav.config.encoder_lost_cycles) {
            Nav_SetFault(CHASSIS_NAV_FAULT_ENCODER_LOST);
        }
    }

    if (s_nav.state.status == CHASSIS_NAV_RUNNING &&
        feedback->encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID) {
        Nav_RunController(dt_s, output);
    } else {
        for (i = 0U; i < CHASSIS_NAV_WHEEL_COUNT; ++i) {
            output->wheel_rpm[i] = 0.0f;
        }
    }

    output->status = s_nav.state.status;
    output->fault = s_nav.state.fault;
}

void ChassisNav_CorrectPose(float x_mm, float y_mm, float yaw_deg,
                            float current_imu_yaw_deg, uint8_t imu_valid)
{
    if (s_nav.initialized == 0U || !isfinite(x_mm) || !isfinite(y_mm) ||
        !isfinite(yaw_deg)) return;

    s_nav.state.pose.x_mm = x_mm;
    s_nav.state.pose.y_mm = y_mm;
    s_nav.state.pose.yaw_deg = Nav_WrapDeg(yaw_deg);
    s_nav.fused_yaw_deg = s_nav.state.pose.yaw_deg;
    s_nav.state.wheel_yaw_deg = s_nav.state.pose.yaw_deg;
    if (imu_valid != 0U && isfinite(current_imu_yaw_deg)) {
        s_nav.imu_offset_deg =
            Nav_WrapDeg(current_imu_yaw_deg - s_nav.fused_yaw_deg);
        s_nav.imu_initialized = 1U;
    }
}

void ChassisNav_GetState(ChassisNav_State_t *state)
{
    if (state != NULL) *state = s_nav.state;
}

bool ChassisNav_IsFinished(void)
{
    return s_nav.state.status == CHASSIS_NAV_DONE;
}

bool ChassisNav_HasFault(void)
{
    return s_nav.state.status == CHASSIS_NAV_FAULT;
}
