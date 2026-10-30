#include "action.h"
#include "main.h"
#include "maixcam.h"
#include "actionInit.h"
#include "tim.h"
#include "math.h"
#include "path_plan.h"
#include "cmsis_os.h"
#include "QR_code.h"
#include "chassis_runtime.h"
#include <string.h>

int min=1,max=11;
int x_cha,y_cha;

volatile uint8_t mat_1=0,mat_2=0,mat_3=0;
uint8_t yanse1=0,yanse2=0;

/* number决定工位圆环位置，slot决定车上的存料槽；机械动作参数保持原值。 */
void PutMaterialFromSlot(int number, uint8_t slot);
void PutStorageMaterialFromSlot(int number, uint8_t slot);
void GrabMaterialToSlot(int number, uint8_t slot);
void PutMaterial2FromSlot(int number, uint8_t slot);

uint8_t ColorToCameraMode(uint8_t color)
{
    switch (color)
    {
        case 1: return 3; // 红色：颜色1 → 模式3
        case 4: return 4; // 绿色：颜色4 → 模式4
        case 3: return 5; // 蓝色：颜色3 → 模式5
        case 5: return 6; // 黑色：颜色5 → 模式6
        case 2: return 7; // 黄色：颜色2 → 模式7
        case 6: return 8; // 浅蓝：颜色6 → 模式8
        default: return 0; // 无效颜色，不发送模式
    }
}

StorageStack_State_t g_storage_stack;

static uint8_t StorageStack_OrderValid(const uint8_t order[3], uint8_t max_id)
{
    uint8_t i;
    if (order == NULL) return 0U;
    for (i = 0U; i < 3U; ++i) {
        if (order[i] < 1U || order[i] > max_id) return 0U;
    }
    return (order[0] != order[1] && order[0] != order[2] &&
            order[1] != order[2]) ? 1U : 0U;
}

bool StorageStack_Configure(const uint8_t first_colors[3],
                            const uint8_t first_rings[3],
                            const uint8_t second_colors[3])
{
    uint8_t i, j;
    memset(&g_storage_stack, 0, sizeof(g_storage_stack));
    g_storage_stack.position_result = VISUAL_POSITION_TIMEOUT; /* 尚未定位。 */
    if (!StorageStack_OrderValid(first_colors, 6U) ||
        !StorageStack_OrderValid(first_rings, 3U) ||
        !StorageStack_OrderValid(second_colors, 6U)) {
        g_storage_stack.fault = STORAGE_STACK_FAULT_BAD_SCAN;
        return false;
    }

    for (i = 0U; i < 3U; ++i) {
        g_storage_stack.first_colors[i] = first_colors[i];
        g_storage_stack.first_rings[i] = first_rings[i];
        g_storage_stack.second_colors[i] = second_colors[i];
        /* 直接根据第1/2组扫码：ring_color[A2]=A1，B/C同理。
         * 不依赖会被第二圈抓取覆盖的mat_1/2/3。
         */
        g_storage_stack.ring_color[first_rings[i]] = first_colors[i];
    }
    for (i = 0U; i < 3U; ++i) {
        for (j = 0U; j < 3U; ++j) {
            if (second_colors[i] == first_colors[j]) {
                g_storage_stack.target_ring[i] = first_rings[j];
                break;
            }
        }
        if (j == 3U) {
            g_storage_stack.fault = STORAGE_STACK_FAULT_BAD_SCAN;
            return false; /* 两圈颜色集合不同，不能盲目码垛。 */
        }
    }
    g_storage_stack.camera_mode =
        ColorToCameraMode(g_storage_stack.ring_color[2]);
    g_storage_stack.configured = 1U;
    return true;
}

/* 只由主任务调用；底盘/摄像头串口中断和任务2仍照常调度。
 * 保留原坐标、航向、机构参数和收臂等待；定位成功后才允许抓放。
 */
static bool StorageStack_RunLayer(uint8_t second_layer)
{
    bool query_was_enabled;
    uint8_t modeX;
    if (g_storage_stack.fault != STORAGE_STACK_FAULT_NONE) return false;
    if (g_storage_stack.configured == 0U) {
        g_storage_stack.fault = STORAGE_STACK_FAULT_BAD_SCAN;
        return false;
    }
    if (second_layer != 0U && g_storage_stack.first_layer_done == 0U) {
        g_storage_stack.fault = STORAGE_STACK_FAULT_NO_FIRST_LAYER;
        return false;
    }
    /* 主流程重入不能把已完成的同一层再次放一遍。 */
    if ((second_layer == 0U && g_storage_stack.first_layer_done != 0U) ||
        (second_layer != 0U && g_storage_stack.second_layer_done != 0U)) return true;

    query_was_enabled = ChassisRuntime_IsEncoderQueryEnabled();
    modeX = second_layer != 0U ? g_storage_stack.camera_mode : 2U;
    Maixcam_setmode(modeX);
    ser_set(-69, 1000);
    osDelay(700U);
    g_storage_stack.position_attempted = 1U;
    if (second_layer != 0U) {
        /* 模式只识别圆环2已有物料的颜色；定位参数沿用用户验证值。 */
        g_storage_stack.position_result =
            VisualPosition_Run(72U, 83U, 0.0f, 1U, 8000U);  //码垛定位点
    } else {
        g_storage_stack.position_result =
            VisualPosition_Run(141U,127U,0.0f,1U,8000U);  //暂存区第一层定位点，采用最新测试值。
    }
    if (g_storage_stack.position_result != VISUAL_POSITION_OK) {
        /* VisualPosition_Run退出已停车；失败只收臂，不执行物料抓放。 */
        g_storage_stack.fault = second_layer != 0U ?
            STORAGE_STACK_FAULT_SECOND_POSITION : STORAGE_STACK_FAULT_FIRST_POSITION;
        ser_set(-145, 1000);
        osDelay(700U);
        return false;
    }

    ChassisRuntime_SetEncoderQueryEnabled(false);
    if (second_layer != 0U) {
        /* 动作仍按槽1/2/3执行，目标圆环已按各槽颜色匹配第一层。 */
        Storage_Second(g_storage_stack.target_ring[0],
                       g_storage_stack.target_ring[1],
                       g_storage_stack.target_ring[2]);
    } else {
        Storage_First(g_storage_stack.first_rings[0],
                      g_storage_stack.first_rings[1],
                      g_storage_stack.first_rings[2]);
    }
    ChassisRuntime_SetEncoderQueryEnabled(query_was_enabled);
    ser_set(-145, 1000);
    osDelay(700U);
    /* 完成标志表示机构动作函数已返回，不代表有传感器确认抓放成功。 */
    if (second_layer != 0U) g_storage_stack.second_layer_done = 1U;
    else g_storage_stack.first_layer_done = 1U;
    return true;
}

bool StorageStack_RunFirst(void)
{
    return StorageStack_RunLayer(0U);
}

bool StorageStack_RunSecond(void)
{
    return StorageStack_RunLayer(1U);
}

#define VISUAL_POSITION_FRAME_TIMEOUT_MS       300U
#define VISUAL_POSITION_STABLE_FRAMES            3U
#define VISUAL_POSITION_LOOP_DELAY_MS             5U
#define VISUAL_POSITION_ACCELERATION             10U

//*****************************************************************************************视觉定位函数****************************

/* 所有视觉接口共用地图角/IMU原始角换算；只在这个底层转换一次。 */
static float VisualPosition_WrapDeg(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}


/* 查询关闭后Runtime快照不再更新，但同周期的原始角与地图IMU角之差
 * 仍是最后一次有效对齐的固定偏移。使用nav.imu_yaw_deg而非融合pose.yaw，
 * 避免把轮式航向与IMU的瞬时融合残差误当作零点偏移。
 */
static uint8_t VisualPosition_MapToRawYaw(float map_yaw_deg, float *raw_yaw_deg)
{
    ChassisRuntime_State_t chassis;
    ChassisRuntime_GetState(&chassis);
    if (raw_yaw_deg == NULL || !isfinite(map_yaw_deg) ||
        chassis.nav_initialized == 0U || chassis.imu_valid == 0U ||
        chassis.nav.imu_fused == 0U || !isfinite(chassis.imu_yaw_deg) ||
        !isfinite(chassis.nav.imu_yaw_deg)) return 0U;

    *raw_yaw_deg = VisualPosition_WrapDeg(map_yaw_deg +
        VisualPosition_WrapDeg(chassis.imu_yaw_deg - chassis.nav.imu_yaw_deg));
    return 1U;
}

/* 保留零输出，只对非零误差施加最小速度和最大速度限制。 */
static int VisualPosition_LimitSpeed(int error, int max_speed)
{
    if (error > max_speed) return max_speed;
    if (error < -max_speed) return -max_speed;
    if (error > 0 && error < min) return min;
    if (error < 0 && error > -min) return -min;
    return error;
}


/**
 * @brief 使用视觉中心坐标微调车体位置，并在运动中保持目标航向。
 * @note
 *   1. 函数会在需要时自动关闭任务2的底盘查询，避免两个任务抢USART3；
 *   2. 若调用前查询已经关闭，函数结束后仍保持关闭，便于继续执行机构动作；
 *   3. 只有连续收到3个处于容差内的新视觉帧才判定成功；
 *   4. 视觉数据超过300 ms未更新时立即停车，直到恢复或总超时。
 *   5. 全部接口传地图航向；关闭查询前读取零点偏移，循环中检查实时IMU。
 */
static VisualPosition_Result_t VisualPosition_RunWithMax(uint16_t target_x,
                                           uint16_t target_y,
                                           float target_yaw_deg,
                                           uint8_t tolerance,
                                           uint32_t timeout_ms,
                                           int max_speed)
{
    uint32_t start_ms;
    uint32_t last_processed_frame = 0U;
    uint8_t stable_frames = 0U;
    uint8_t stopped = 0U;
    bool query_was_enabled;
    float raw_target_yaw_deg;
    VisualPosition_Result_t result = VISUAL_POSITION_TIMEOUT;

    if (tolerance == 0U || timeout_ms == 0U ||
        !VisualPosition_MapToRawYaw(target_yaw_deg, &raw_target_yaw_deg)) {
        return VISUAL_POSITION_BAD_PARAMETER;
    }

    query_was_enabled = ChassisRuntime_IsEncoderQueryEnabled();
    if (query_was_enabled) {
        ChassisRuntime_SetEncoderQueryEnabled(false);
        /* 等任务2跨过一个完整周期，确认共享串口已经让出。 */
        osDelay(20U);
    }

    start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t frame_count = maixcam_frame_count;
        uint16_t measured_x = YuanData[0];
        uint16_t measured_y = YuanData[1];
        int x_error;
        int y_error;
        uint8_t x_ok;
        uint8_t y_ok;
        HWT101_Snapshot_t imu;

        /* 编码器查询关闭不影响串口6接收；不能用停止刷新的Runtime快照
         * 判断当前角度是否在线。所有工位都检查原始IMU的实时帧时效。
         */
        HWT101_GetSnapshot(&imu);
        if (imu.frame_count == 0U || !isfinite(imu.yaw_deg) ||
            (uint32_t)(now_ms - imu.last_update_ms) > VISUAL_POSITION_IMU_TIMEOUT_MS) {
            stable_frames = 0U;
            if (stopped == 0U) {
                Emm_V5_Stop_Now(0U, false);
                stopped = 1U;
            }
            osDelay(VISUAL_POSITION_LOOP_DELAY_MS);
            continue;
        }

        /* 没有视觉帧或数据已经过期：只停车等待，不使用旧坐标盲走。 */
        if (frame_count == 0U ||
            (uint32_t)(now_ms - maixcam_last_update_ms) >
                VISUAL_POSITION_FRAME_TIMEOUT_MS) {
            stable_frames = 0U;
            if (stopped == 0U) {
                Emm_V5_Stop_Now(0U, false);
                stopped = 1U;
            }
            osDelay(VISUAL_POSITION_LOOP_DELAY_MS);
            continue;
        }

        x_error = (int)target_x - (int)measured_x;
        y_error = (int)measured_y - (int)target_y;
        x_cha = x_error;
        y_cha = y_error;
        x_ok = (x_error >= -(int)tolerance &&
                x_error <=  (int)tolerance) ? 1U : 0U;
        y_ok = (y_error >= -(int)tolerance &&
                y_error <=  (int)tolerance) ? 1U : 0U;

        if (x_ok != 0U && y_ok != 0U) {
            if (stopped == 0U) {
                Emm_V5_Stop_Now(0U, false);
                stopped = 1U;
            }
            /* 同一个视觉帧不能被循环重复计数。 */
            if (frame_count != last_processed_frame) {
                last_processed_frame = frame_count;
                if (stable_frames < 255U) stable_frames++;
                if (stable_frames >= VISUAL_POSITION_STABLE_FRAMES) {
                    result = VISUAL_POSITION_OK;
                    break;
                }
            }
        } else {
            stable_frames = 0U;
            last_processed_frame = frame_count;
            x_cha = (x_ok != 0U) ? 0 : VisualPosition_LimitSpeed(x_error, max_speed);
            y_cha = (y_ok != 0U) ? 0 : VisualPosition_LimitSpeed(y_error, max_speed);
            /* 此低层电机接口仍使用IMU原始航向，禁止再次加起始摆车角。 */
            motor_Omnidirectional_set(x_cha, y_cha, raw_target_yaw_deg,
                                      VISUAL_POSITION_ACCELERATION);
            stopped = 0U;
        }

        osDelay(VISUAL_POSITION_LOOP_DELAY_MS);
    }

    Emm_V5_Stop_Now(0U, false);
    osDelay(20U);
    if (query_was_enabled) {
        ChassisRuntime_SetEncoderQueryEnabled(true);
        /* 让高优先级任务先回读一次累计编码器，把本次微调写入里程计。 */
        osDelay(20U);
    }
    return result;
}

/* 两个接口都传地图航向，只在独立平移限速上不同。 */
VisualPosition_Result_t VisualPosition_Run(uint16_t target_x,
                                           uint16_t target_y,
                                           float target_yaw_deg,
                                           uint8_t tolerance,
                                           uint32_t timeout_ms)
{
    return VisualPosition_RunWithMax(target_x, target_y, target_yaw_deg,
                                     tolerance, timeout_ms, max);
}

VisualPosition_Result_t VisualPosition_RunRawMaterial(uint16_t target_x,
                                                      uint16_t target_y,
                                                      float target_yaw_deg,
                                                      uint8_t tolerance,
                                                      uint32_t timeout_ms)
{
    return VisualPosition_RunWithMax(target_x, target_y, target_yaw_deg,
        tolerance, timeout_ms, RAW_GRAB_POSITION_MAX_RPM);
}
//*****************************************************************************************视觉定位函数****************************


//******************************************************************************************原料区域的函数************************
/* 新原料区处理集中在action中；旧task_judge/GrabMaterialsInOrder仅保留兼容。
 * 仅最后一抓跳过robot_task.c的回中间动作；共享ZDT_set/servo_set不变。
 */
volatile uint16_t raw_grab_test_layout = 0U;

/* 按左/中/右调用用户原来封装的抓取动作，slot为车上存料槽1/2/3。
 * 电机脉冲、舵机、夹爪和动作等待均以robot_task.c里的grab_*为准。
 */
void RawGrab_GrabPosition(RawGrab_Position_t position, uint8_t slot)
{
    switch (position) {
    case RAW_GRAB_LEFT:   grab_left(slot);   break;
    case RAW_GRAB_MIDDLE: grab_middle(slot); break;
    case RAW_GRAB_RIGHT:  grab_right(slot);  break;
    default: break;
    }
}

/* 旧排列已由定位或上一抓确认，不再等物料转回旧位置。
 * 总流程传入下一120度位次中“仍可见的物料颜色和位置”。
 * 本函数只等模式切换后的新帧确认该新位置，不以颜色消失作为到位依据。
 * 无角度传感器：这是已知转盘顺序下的视觉换位推断，不是机械角度测量。
 */
void RawGrab_WaitColorAt(uint8_t color, RawGrab_Position_t position)
{
    uint16_t target_x;
    uint16_t target_y;
    int32_t tolerance = RAW_GRAB_TOLERANCE_PX;
    uint32_t baseline;

    switch (position) {
    case RAW_GRAB_LEFT:
        target_x = RAW_GRAB_LEFT_X_PX;
        target_y = RAW_GRAB_LEFT_Y_PX;
        break;
    case RAW_GRAB_MIDDLE:
        target_x = RAW_GRAB_EXTENDED_MIDDLE_X_PX;
        target_y = RAW_GRAB_EXTENDED_MIDDLE_Y_PX;
        break;
    case RAW_GRAB_RIGHT:
        target_x = RAW_GRAB_RIGHT_X_PX;
        target_y = RAW_GRAB_RIGHT_Y_PX;
        break;
    default: return;
    }
    Maixcam_setmode(ColorToCameraMode(color));
    baseline = maixcam_frame_count;
    for (;;) {
        uint32_t frame = maixcam_frame_count;
        uint8_t measured_color = *(volatile uint8_t *)&WuLiaoData;
        uint8_t command = *(volatile uint8_t *)&move_maix;
        uint16_t x = YuanData[0], y = YuanData[1];
        uint32_t update_ms = maixcam_last_update_ms;
        int32_t dx = (int32_t)x - target_x;
        int32_t dy = (int32_t)y - target_y;
        if (frame == maixcam_frame_count && frame != 0U && frame != baseline &&
            (uint32_t)(HAL_GetTick() - update_ms) <= RAW_GRAB_FRAME_TIMEOUT_MS &&
            command == 0xB1U && measured_color == color &&
            dx >= -tolerance && dx <= tolerance &&
            dy >= -tolerance && dy <= tolerance) {
            return;
        }
        osDelay(RAW_GRAB_POLL_MS);
    }
}

uint8_t RawGrab_Run(uint8_t a, uint8_t b, uint8_t c)
{
    uint8_t colors[3] = {a, b, c};
    uint8_t position[3], layout[3];
    uint8_t i, second_position, reference_position, reference_index;
    uint8_t third_position;
    uint8_t result = 0U;
    bool query_was_enabled;

    material_last = 0U;
    raw_grab_test_layout = 0U;
    yanse1 = yanse2 = 0U;
    mat_1 = mat_2 = mat_3 = 0U;
    if (a < 1U || a > 6U || b < 1U || b > 6U || c < 1U || c > 6U ||
        a == b || a == c || b == c) goto finish;

    Maixcam_setmode(1U);
    osDelay(100U);
    ser_set(-69, 800);
    osDelay(810U);
    /* 起初只能看中间，依次记录两个不同任务颜色；判断直接写在这里。 */
    for (i = 0U; i < 2U; ++i) {
        uint32_t baseline = maixcam_frame_count;
        for (;;) {
            uint32_t frame = maixcam_frame_count;
            uint8_t color = *(volatile uint8_t *)&WuLiaoData;
            uint8_t command = *(volatile uint8_t *)&move_maix;
            uint32_t update_ms = maixcam_last_update_ms;
            if (frame == maixcam_frame_count && frame != 0U && frame != baseline &&
                (uint32_t)(HAL_GetTick() - update_ms) <= RAW_GRAB_FRAME_TIMEOUT_MS &&
                command == 0xB1U && (color == a || color == b || color == c) &&
                (i == 0U || color != yanse1)) {
                if (i == 0U) yanse1 = color;
                else yanse2 = color;
                break;
            }
            osDelay(RAW_GRAB_POLL_MS);
        }
    }
    /* 第二次看到的颜色在中间；第一次看到的已转过120度。
     * 方向由宏决定，剩余颜色的位置由二维码三个颜色直接补出。
     * position[0/1/2]分别是a/b/c的位置，不再需要六种排列查表。
     */
    for (i = 0U; i < 3U; ++i) {
        if (colors[i] == yanse2) position[i] = RAW_GRAB_MIDDLE;
        else if (colors[i] == yanse1)
            position[i] = (RAW_GRAB_MIDDLE + RAW_GRAB_ROTATION_STEP) % 3U;
        else position[i] = 2U -
            (RAW_GRAB_MIDDLE + RAW_GRAB_ROTATION_STEP) % 3U;
        layout[position[i]] = i + 1U;
    }
    raw_grab_test_layout = layout[0] * 100U + layout[1] * 10U + layout[2];

    /* 此时摄像头能看中间：定位第二次看到的颜色，再按排列抓a。 */
    query_was_enabled = ChassisRuntime_IsEncoderQueryEnabled();
    Maixcam_setmode(ColorToCameraMode(yanse2));
    if (VisualPosition_RunRawMaterial(RAW_GRAB_MIDDLE_X_PX, RAW_GRAB_MIDDLE_Y_PX,
            RAW_GRAB_POSITION_YAW_DEG, RAW_GRAB_POSITION_TOLERANCE_PX,
            RAW_GRAB_POSITION_TIMEOUT_MS) != VISUAL_POSITION_OK ||
        *(volatile uint8_t *)&WuLiaoData != yanse2) goto finish;

    ChassisRuntime_SetEncoderQueryEnabled(false);
    osDelay(20U);
    RawGrab_GrabPosition((RawGrab_Position_t)position[0], 1U);
    mat_1 = a;

    /* 第二抓：直接按已知排列计算下一次120度的位置，不重新等旧状态。
     * b换位后在左右就识别b；b换到中间就直接发c的模式，看c的侧边新位置。
     * 不能先发b模式去等它回旧侧，否则b已经在中间时会一直收不到坐标。
     */
    second_position = (position[1] + RAW_GRAB_ROTATION_STEP) % 3U;
    reference_index = second_position == RAW_GRAB_MIDDLE ? 2U : 1U;
    reference_position = (position[reference_index] + RAW_GRAB_ROTATION_STEP) % 3U;
    RawGrab_WaitColorAt(colors[reference_index], (RawGrab_Position_t)reference_position);
    /* 新位次已被视觉确认，同步更新仍在转盘上的b/c位置，后续不再用旧排列。 */
    position[1] = second_position;
    position[2] = (position[2] + RAW_GRAB_ROTATION_STEP) % 3U;
    RawGrab_GrabPosition((RawGrab_Position_t)position[1], 2U);
    mat_2 = b;
    /* 第三抓：从刚确认的当前排列再算一次120度，不固定要求回左或回右。
     * 312/321的c下一位在中间，识别伸长后的中点(70,148)后中抓。
     * 其它排列保留原侧边等待规则，不靠颜色消失盲抓，也不重新等旧位置。
     */
    third_position = (position[2] + RAW_GRAB_ROTATION_STEP) % 3U;
    if (third_position == RAW_GRAB_MIDDLE &&
        raw_grab_test_layout != 312U && raw_grab_test_layout != 321U)
        third_position = (third_position + RAW_GRAB_ROTATION_STEP) % 3U;
    RawGrab_WaitColorAt(c, (RawGrab_Position_t)third_position);
    position[2] = third_position;
    material_last = 1U; /* Last raw grab: skip the return-to-middle tail. */
    RawGrab_GrabPosition((RawGrab_Position_t)position[2], 3U);
    mat_3 = c;
    ChassisRuntime_SetEncoderQueryEnabled(query_was_enabled);
    result = 1U;
finish:
    material_last = 0U;
    /* 不论成功或定位失败，退出原料处理前都先收臂；失败由任务锁存等待。 */
	   ZDT_set(6,1300);
    ser_set(-145, 1000);
    osDelay(700U);
    return result;
}

//********************************************************************原料区域的函数****************************


//********************************************************************暂存区域和加工区域的动作函数******************
/* 第一圈加工：槽1->圆环a、槽2->圆环b、槽3->圆环c，取回原槽。 */
void Processing_First(uint8_t a, uint8_t b, uint8_t c)
{
    material_last = 0U;
    PutMaterialFromSlot(a, 1U);
    PutMaterialFromSlot(b, 2U);
    material_last = 1U; /* Last placement: skip return to middle. */
    PutMaterialFromSlot(c, 3U);
    material_last = 0U;

    /* 用户接口：若加工需要完成信号，在此判断完成后再取回。 */
    GrabMaterialToSlot(a, 1U);
    GrabMaterialToSlot(b, 2U);
    material_last = 1U; /* Last retrieval: skip return to middle. */
    GrabMaterialToSlot(c, 3U);
    material_last = 0U;
	
	  ZDT_set(1,0);
	  osDelay(100);
}

/* 第二圈加工：与第一圈独立，方便分别调整任务。 */
void Processing_Second(uint8_t a, uint8_t b, uint8_t c)
{
    material_last = 0U;
    PutMaterialFromSlot(a, 1U);
    PutMaterialFromSlot(b, 2U);
    material_last = 1U; /* Last placement: skip return to middle. */
    PutMaterialFromSlot(c, 3U);
    material_last = 0U;

    /* 用户接口：第二圈加工完成判断放在这里。 */
    GrabMaterialToSlot(a, 1U);
    GrabMaterialToSlot(b, 2U);
    material_last = 1U; /* Last retrieval: skip return to middle. */
    GrabMaterialToSlot(c, 3U);
    material_last = 0U;
	
	  ZDT_set(1,0);
		osDelay(100);
}

/* 第一圈暂存：地面第一层放置，不能使用叠放高度。 */
void Storage_First(uint8_t a, uint8_t b, uint8_t c)
{
    material_last = 0U;
    PutStorageMaterialFromSlot(a, 1U);
    PutStorageMaterialFromSlot(b, 2U);
    material_last = 1U; /* Last placement: skip return to middle. */
    PutStorageMaterialFromSlot(c, 3U);
    material_last = 0U;
}

/* 第二圈暂存：沿用put_material2的第二层叠放动作。 */
void Storage_Second(uint8_t a, uint8_t b, uint8_t c)
{
    material_last = 0U;
    PutMaterial2FromSlot(a, 1U);
    PutMaterial2FromSlot(b, 2U);
    material_last = 1U; /* Last placement: skip return to middle. */
    PutMaterial2FromSlot(c, 3U);
    material_last = 0U;
}

void put_material(int number)
{
    /* 兼容旧接口：原先圆环号和槽号相同，调用行为保持不变。 */
    PutMaterialFromSlot(number, (uint8_t)number);
}

void PutMaterialFromSlot(int number, uint8_t slot)
{
	if(number==2)
	{
			Plate(slot);
		    ser_set(-145,750);
		    HAL_Delay(400);
	     ZDT_set(6,400);
	     HAL_Delay(100);	
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(200);
		    ser_set(-69,800);
		    HAL_Delay(400);
		    ZDT_set(6,3000);
		    HAL_Delay(100);
		    ZDT_set(1,8800);
		    HAL_Delay(850);
		
      servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
				if (material_last == 0U)
			 {
			     ser_set(-70,800);
			     HAL_Delay(600);
			 }
	}
    if(number==1)
	{
		Plate(slot);
		ser_set(-145,800);
		HAL_Delay(500);
		
	     ZDT_set(6,400);
	     HAL_Delay(200);		
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(400);
		
            ser_set(-100,550);
		    HAL_Delay(400);
		    ZDT_set(6,4800);
		    HAL_Delay(100);		
		    ZDT_set(1,8800);
		    HAL_Delay(700);
            servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
			
			 if (material_last == 0U)
			 {
			     ser_set(-70,800);
			     HAL_Delay(600);
			 }
	}
    if(number==3)
	{
		Plate(slot);
		ser_set(-145,800);
		HAL_Delay(500);
		
		ZDT_set(6,400);
	     HAL_Delay(200);		
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(400);
		    ser_set(-38,1100);
		 HAL_Delay(800);
		    ZDT_set(6,5800);
		    HAL_Delay(400);		
		    ZDT_set(1,8800);
		    HAL_Delay(900);
		
		
    servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
			 if (material_last == 0U)
			 {
			     ser_set(-70,500);
			     HAL_Delay(500);
			 }
	}

}

/* 暂存区第一层独立放置：初始参数与粗加工相同，后续只在此函数调试。 */
void PutStorageMaterialFromSlot(int number, uint8_t slot)
{
	if(number==2)
	{
			Plate(slot);
		    ser_set(-145,750);
		    HAL_Delay(400);
	     ZDT_set(6,400);
	     HAL_Delay(100);	
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(200);
		    ser_set(-69,800);
		    HAL_Delay(400);
		    ZDT_set(6,3000);
		    HAL_Delay(100);
		    ZDT_set(1,8850);
		    HAL_Delay(850);
		
      servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
	}
    if(number==1)
	{
		Plate(slot);
		ser_set(-145,800);
		HAL_Delay(500);
		
	     ZDT_set(6,400);
	     HAL_Delay(200);		
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(400);
		
       ser_set(-100,550);
		 HAL_Delay(400);
		    ZDT_set(6,4800);
		    HAL_Delay(100);		
		    ZDT_set(1,8800);
		    HAL_Delay(700);
      servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
			
			 if (material_last == 0U)
			 {
			     ser_set(-70,800);
			     HAL_Delay(600);
			 }
	}
    if(number==3)
	{
		Plate(slot);
		ser_set(-145,800);
		HAL_Delay(500);
		
		ZDT_set(6,400);
	     HAL_Delay(200);		
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(400);
		    ser_set(-38,1100);
		 HAL_Delay(800);
		    ZDT_set(6,5980);
		    HAL_Delay(400);		
		    ZDT_set(1,8800);
		    HAL_Delay(900);
		
		
            servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
			 if (material_last == 0U)
			 {
			     ser_set(-70,500);
			     HAL_Delay(500);
			 }
	}

}

void grab_material(int number)
{
    /* 兼容旧接口：仍将指定圆环的物料放回同号槽。 */
    GrabMaterialToSlot(number, (uint8_t)number);
}

void GrabMaterialToSlot(int number, uint8_t slot)
{
	if(number==2)
	{
		    ser_set(-69,800);
		    HAL_Delay(600);//转回圆环角度
		    Plate(slot);
		
		   ZDT_set2(1,8800);
			HAL_Delay(10);
			ZDT_set2(6,3000);
		    HAL_Delay(800);
		
				servo_set(1);
		
		   ZDT_set2(1,1000);
			HAL_Delay(10);
			ZDT_set2(6,400);
		  HAL_Delay(600);
		    ser_set(-145,800);
		    HAL_Delay(500);//回到转盘处
				
		
		   ZDT_set2(1,3800);
			HAL_Delay(10);
			ZDT_set2(6,400);
		  HAL_Delay(500);
			servo_set(2);
				
			ZDT_set2(1,2000);
			HAL_Delay(100);
			ZDT_set2(6,1300);
		  HAL_Delay(200);
			
		    if (material_last == 0U)
		    {
		        ser_set(-69,800);
		        HAL_Delay(500);
		    }
	}
    if(number==1)
	{
		    ser_set(-101,800);
		    HAL_Delay(600);//转回圆环角度
		    Plate(slot);
		
		   ZDT_set2(1,8800);
			HAL_Delay(10);
			ZDT_set2(6,4800);
		    HAL_Delay(800);
		
				servo_set(1);
		
		   ZDT_set2(1,1000);
			HAL_Delay(400);
			ZDT_set2(6,400);
		  HAL_Delay(600);
		    ser_set(-145,800);
		    HAL_Delay(500);//回到转盘处
				
		
		   ZDT_set2(1,3800);
			HAL_Delay(10);
			ZDT_set2(6,400);
		  HAL_Delay(500);
			servo_set(2);
				
			ZDT_set2(1,2000);
			HAL_Delay(100);
			ZDT_set2(6,1300);
		  HAL_Delay(200);
			
		    if (material_last == 0U)
		    {
		        ser_set(-69,800);
		        HAL_Delay(500);
		    }
	}
    if(number==3)
	{
		    ser_set(-38,800);
		    HAL_Delay(600);//转回圆环角度
		    Plate(slot);
		
		   ZDT_set2(1,8800);
			HAL_Delay(10);
			ZDT_set2(6,5800);
		    HAL_Delay(800);
		
				servo_set(1);
		
		   ZDT_set2(1,1000);
			HAL_Delay(10);
			ZDT_set2(6,400);
		  HAL_Delay(600);
		    ser_set(-145,800);
		    HAL_Delay(500);//回到转盘处
				
		
		   ZDT_set2(1,3800);
			HAL_Delay(10);
			ZDT_set2(6,400);
		  HAL_Delay(500);
			servo_set(2);
				
			ZDT_set2(1,2000);
			HAL_Delay(100);
			ZDT_set2(6,1300);
		  HAL_Delay(200);
			
		    if (material_last == 0U)
		    {
		        ser_set(-69,800);
		        HAL_Delay(500);
		    }
	}
}

void put_material2(int number)
{
    /* 兼容旧接口，第二层高度仍沿用原来的动作参数。 */
    PutMaterial2FromSlot(number, (uint8_t)number);
}

void PutMaterial2FromSlot(int number, uint8_t slot)
{
	if(number==2)
	{
			Plate(slot);
		    ser_set(-145,750);
		    HAL_Delay(400);
	     ZDT_set(6,400);
	     HAL_Delay(100);	
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(200);
		    ser_set(-69,800);
		    HAL_Delay(400);
		    ZDT_set(6,3200);
		    HAL_Delay(100);
		    ZDT_set(1,5500);
		    HAL_Delay(850);
		
            servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
	}
    if(number==1)
	{
		Plate(slot);
		ser_set(-145,800);
		HAL_Delay(500);
		
	     ZDT_set(6,400);
	     HAL_Delay(200);		
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(400);
		
       ser_set(-99,550);
		 HAL_Delay(400);
		    ZDT_set(6,5000);
		    HAL_Delay(300);		
		    ZDT_set(1,5500);
		    HAL_Delay(700);
       servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
			
			 if (material_last == 0U)
			 {
			     ser_set(-70,800);
			     HAL_Delay(600);
			 }
	}
    if(number==3)
	{
		Plate(slot);
		ser_set(-145,800);
		HAL_Delay(500);
		
		ZDT_set(6,400);
	     HAL_Delay(200);		
		   ZDT_set(1,3800);
		   HAL_Delay(400);
		   servo_set(1);
	      ZDT_set(1,2000);
		    HAL_Delay(400);
		    ser_set(-38,1100);
		 HAL_Delay(800);
		    ZDT_set(6,6000);
		    HAL_Delay(400);		
		    ZDT_set(1,5500);
		    HAL_Delay(900);
		
		
    servo_set(2);
		
			ZDT_set(1,0);
			HAL_Delay(10);
			ZDT_set(6,1300);
			HAL_Delay(600);
			 if (material_last == 0U)
			 {
			     ser_set(-70,500);
			     HAL_Delay(500);
			 }
	}
}
//********************************************************************原料区域和加工区域的函数******************

