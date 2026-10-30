#ifndef CHASSIS_NAV_H
#define CHASSIS_NAV_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_NAV_WHEEL_COUNT 4U
#define CHASSIS_NAV_ENCODER_ALL_VALID 0x0FU

/* 普通路径衔接容差；工位对接/最终停车仍用config中的严格参数。
 * 小角度和位置残差交给下一段GotoXY的航向/位置闭环继续修正。
 */
#define CHASSIS_NAV_TRANSIT_POSITION_MM       8.0f
#define CHASSIS_NAV_TRANSIT_HEADING_DEG        3.0f
#define CHASSIS_NAV_TRANSIT_SPEED_MM_S        40.0f
#define CHASSIS_NAV_TRANSIT_YAW_RATE_DEG_S    15.0f
#define CHASSIS_NAV_TRANSIT_SETTLED_CYCLES       2U

/* 导航状态：接口全部为非阻塞式，RUNNING期间持续调用Update。 */
typedef enum {
    CHASSIS_NAV_IDLE = 0,
    CHASSIS_NAV_RUNNING,
    CHASSIS_NAV_DONE,
    CHASSIS_NAV_FAULT
} ChassisNav_Status_t;

/* 故障码：编码器连续丢帧或调用周期异常时进入FAULT。 */
typedef enum {
    CHASSIS_NAV_FAULT_NONE = 0,
    CHASSIS_NAV_FAULT_ENCODER_LOST,
    CHASSIS_NAV_FAULT_BAD_DT
} ChassisNav_Fault_t;

/*
 * 底盘标定与控制参数。
 * 长度单位统一为mm，角度单位统一为deg，速度输出单位为电机RPM。
 * encoder_sign用于把驱动器协议正负号转换为底盘逻辑轮方向。
 */
typedef struct {
    float wheel_diameter_mm;
    float encoder_counts_per_rev;
    /* 轮中心几何参数(轴距+轮距)/2，不是车体外形尺寸之和的一半。 */
    float half_length_plus_width_mm;
    int8_t encoder_sign[CHASSIS_NAV_WHEEL_COUNT];

    float max_wheel_rpm;
    float acceleration_rpm_s;       /* 起步/加速上限，单位RPM/s。 */
    float brake_deceleration_mm_s2; /* 固定平移减速度，单位mm/s²；越大刹得越快。 */
    /* 兼容旧工程的保留字段，建议置0；现已取消柔和段，此值不参与控制。 */
    float brake_soft_speed_mm_s;
    float min_approach_rpm;

    float position_tolerance_mm;
    float speed_tolerance_mm_s;
    float heading_kp_rpm_per_deg;
    float heading_kd_rpm_per_deg_s;
    float heading_max_rpm;
    float heading_tolerance_deg;
    float yaw_rate_tolerance_deg_s;
    float fusion_wheel_alpha;

    uint8_t settled_cycles_required;
    uint8_t encoder_lost_cycles;
} ChassisNav_Config_t;

/*
 * 单周期反馈输入。
 * motor_count[]顺序固定为逻辑轮[m0,m1,m2,m3]，不是驱动器地址升序。
 * Runtime按chassis_direction.h映射回读：正常[2,3,4,5]，对角互换[4,5,2,3]。
 * 位置值为0x36应答解析出的“有符号原始累计计数”，这里尚未乘encoder_sign。
 * encoder_valid_mask的bit0~3分别表示四轮本周期是否回读成功。
 */
typedef struct {
    int64_t motor_count[CHASSIS_NAV_WHEEL_COUNT];
    uint8_t encoder_valid_mask;
    float imu_yaw_deg;
    uint8_t imu_valid;
} ChassisNav_Feedback_t;

/*
 * 地图坐标位姿：右上角停车区1为原点，向左为X+，向下为Y+。
 * 车辆在起点朝下时航向为0度，逆时针转动为正角度。
 */
typedef struct {
    float x_mm;
    float y_mm;
    float yaw_deg;
} ChassisNav_Pose_t;

/* 机体坐标速度：前进vx为正、车体左移vy为正、逆时针wz为正。 */
typedef struct {
    float vx_mm_s;
    float vy_mm_s;
    float wz_deg_s;
} ChassisNav_Velocity_t;

/* 控制输出：wheel_rpm[]为安装方向修正后的四轮逻辑转速。 */
typedef struct {
    float wheel_rpm[CHASSIS_NAV_WHEEL_COUNT];
    ChassisNav_Status_t status;
    ChassisNav_Fault_t fault;
} ChassisNav_Output_t;

/* 调试快照，可放入Keil Watch观察，不应由外部直接修改。 */
typedef struct {
    ChassisNav_Pose_t pose;
    ChassisNav_Velocity_t velocity;
    float wheel_yaw_deg;
    float imu_yaw_deg;
    float target_x_mm;
    float target_y_mm;
    float target_yaw_deg;
    float position_error_mm;
    float heading_error_deg;
    float command_vx_rpm;
    float command_vy_rpm;
    float command_wz_rpm;
    uint8_t imu_fused;
    uint8_t encoder_fault_suppressed;
    uint8_t encoder_miss_streak;
    uint8_t settled_cycles;
    uint8_t transit_mode; /* 1=路径快速衔接容差；0=严格停车/对接。 */
    ChassisNav_Status_t status;
    ChassisNav_Fault_t fault;
} ChassisNav_State_t;

/* 填入当前架空测试使用的默认参数，实车落地后需要重新标定轮径等参数。 */
void ChassisNav_GetDefaultConfig(ChassisNav_Config_t *config);

/* 建立编码器软件零点；初始化时必须保证四轮反馈全部有效。 */
bool ChassisNav_Init(const ChassisNav_Config_t *config,
                     const ChassisNav_Feedback_t *initial_feedback);

/* 前往地图绝对坐标目标，同时独立控制最终航向。 */
bool ChassisNav_GotoXY(float x_mm, float y_mm, float final_yaw_deg);

/* 本次移动单独限速；后续普通GotoXY/TurnTo自动恢复默认限速。 */
bool ChassisNav_GotoXYLimited(float x_mm, float y_mm, float final_yaw_deg,
                              float max_rpm);

/* 按地图坐标给相对位移；dx向左、dy向下，final_yaw_deg为绝对航向。 */
bool ChassisNav_MoveRelativeWorld(float dx_mm, float dy_mm,
                                  float final_yaw_deg);

/* 按当前车体坐标给相对位移：forward为前进，left为向左。 */
bool ChassisNav_MoveRelativeBody(float forward_mm, float left_mm,
                                 float final_yaw_deg);

/* 原地转到绝对世界航向。 */
bool ChassisNav_TurnTo(float yaw_deg);

/* 仅由电机任务在设置本次目标之后调用；下一次设目标自动恢复严格模式。 */
void ChassisNav_SetTransitMode(bool enabled);

/*
 * 固定脉冲位置模式执行期间，驱动器可能暂时不回复累计位置查询。
 * suppressed=true时仍尝试回读和更新里程计，但不因连续丢帧进入故障；
 * 退出粗转、进入速度闭环前必须恢复为false。
 */
void ChassisNav_SetEncoderFaultSuppressed(bool suppressed);

/* 取消当前动作；下一次Update输出四轮零速。 */
void ChassisNav_Stop(void);

/*
 * 建议每20 ms调用一次：更新里程计、融合HWT101、执行目标控制，输出四轮RPM。
 * IMU短时无效时自动退化为纯轮式航向；恢复有效后自动重新融合。
 * 禁止用目标RPM代替丢失的编码器反馈。
 */
void ChassisNav_Update(const ChassisNav_Feedback_t *feedback,
                       float dt_s,
                       ChassisNav_Output_t *output);

/* 视觉定位校正世界位姿，不清除电机累计位置，也不中断当前编码器增量。 */
void ChassisNav_CorrectPose(float x_mm, float y_mm, float yaw_deg,
                            float current_imu_yaw_deg, uint8_t imu_valid);

/* 原子性由调用者保证；裸机工程通常从主循环读取即可。 */
void ChassisNav_GetState(ChassisNav_State_t *state);
bool ChassisNav_IsFinished(void);
bool ChassisNav_HasFault(void);

#endif
