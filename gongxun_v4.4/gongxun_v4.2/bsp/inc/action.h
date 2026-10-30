#ifndef __ACTION_H__
#define __ACTION_H__

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "chassic.h"
#include "maixcam.h"
#include "Emm_V5.h"
#include "path_plan.h"
#include "FSUS.h"
#include "robot_task.h"
//坐标的宏定义
#define RED_fhang_x       86
#define RED_fhang_y       92
 
#define GREEN_fhang_x     86
#define GREEN_fhang_y     92

#define BLUE_fhang_x      86
#define BLUE_fhang_y      92

#define Clockwise_90           -90
#define Clockwise_180          -180
#define Clockwise_270          -270
#define CounterClockwise_90    90
#define CounterClockwise_180   180
#define CounterClockwise_270   270

#define left 0
#define left_2 1
#define right 2
#define right_2 3


void Limit1(int* Source, int Limit_Min, int Limit_Max);
float positioning_pid(float Drive, float hope);

typedef enum {
    VISUAL_POSITION_OK = 0,
    VISUAL_POSITION_TIMEOUT,
    VISUAL_POSITION_BAD_PARAMETER
} VisualPosition_Result_t;

/*
 * 使用MaixCam图像坐标进行车体微调。
 * 所有定位接口的target_yaw_deg统一为地图绝对航向，不是IMU原始读数，
 * 也不是相对起始摆放的角度。启动CorrectPose建立的IMU偏移会自动换算。
 * 起始角改0/±90/180度后，原料-90、加工+90、暂存0度不需要跟着加减。
 * 必须先完成底盘初始化及有效IMU对齐；缺少参考返回BAD_PARAMETER。
 * timeout_ms防止丢图死循环；关闭编码器查询时仍直接读取串口6最新IMU。
 */
#define VISUAL_POSITION_IMU_TIMEOUT_MS 200U /* 全部视觉定位的IMU帧时效，毫秒；过期停车等待。 */
VisualPosition_Result_t VisualPosition_Run(uint16_t target_x,
                                           uint16_t target_y,
                                           float target_yaw_deg,
                                           uint8_t tolerance,
                                           uint32_t timeout_ms);
/* 原料区独立限速定位；target_yaw_deg使用地图航向，内部适配IMU原始零点。 */
VisualPosition_Result_t VisualPosition_RunRawMaterial(uint16_t target_x,
                                                      uint16_t target_y,
                                                      float target_yaw_deg,
                                                      uint8_t tolerance,
                                                      uint32_t timeout_ms);

/* 原料区：第一次看中间定位，第一抓按排列；伸臂后第三抓支持中间视觉。
 * 按已确认排列计算下一次120度换位，不再重新等物料回旧位置。
 * 第二件若换到中间，直接识别第三件作参照；确认后同步更新剩余物料位置。
 * 312/321最后一件直接等伸长后的中点再中抓；其它排列保持原侧边规则。
 * 不修改加工/暂存区动作，没有GRAB模式开关。
 */

/* 相机图像像素坐标，非地图mm；改坐标需重新标定对应抓取位置。 */
#define RAW_GRAB_LEFT_X_PX              25U
#define RAW_GRAB_LEFT_Y_PX              44U
#define RAW_GRAB_MIDDLE_X_PX            69U
#define RAW_GRAB_MIDDLE_Y_PX           101U
/* 抓过物料伸长后使用的中点；与上面的首次车辆定位点独立。 */
#define RAW_GRAB_EXTENDED_MIDDLE_X_PX   70U
#define RAW_GRAB_EXTENDED_MIDDLE_Y_PX  148U
#define RAW_GRAB_RIGHT_X_PX            105U
#define RAW_GRAB_RIGHT_Y_PX             52U
/* 左/右及伸长后的中点触发共用窗口，X/Y分别±此值，单位像素。
 * 越大越早触发，也越容易抓偏；伸长中点使用独立标定坐标。
 */
#define RAW_GRAB_TOLERANCE_PX            18U
#define RAW_GRAB_POSITION_YAW_DEG      -90.0f /* 原料工位地图航向，和起始摆车角独立。 */
#define RAW_GRAB_POSITION_TOLERANCE_PX   1U   /* 车辆定位精度，不是抓取窗口。 */
#define RAW_GRAB_POSITION_TIMEOUT_MS 8000U   /* 车辆定位总超时，失败不抓取。 */
#define RAW_GRAB_POSITION_MAX_RPM       17   /* 原料视觉平移分量限速，不改其它工位。 */
#define RAW_GRAB_FRAME_TIMEOUT_MS      300U  /* 帧最大年龄，不是等颜色的总超时。 */
#define RAW_GRAB_POLL_MS                 5U  /* 等待时让出CPU，不是固定抓取延时。 */
/* 每次120度的位置变化，下标为左0、中1、右2：新位置=(旧位置+此值)%3。
 * 2：左->右、右->中、中->左（沿用原工程方向）；
 * 1：左->中、中->右、右->左。转盘反转时只修改这里，不能凭观察先后猜方向。
 */
#ifndef RAW_GRAB_ROTATION_STEP
#define RAW_GRAB_ROTATION_STEP           2U
#endif
#if RAW_GRAB_ROTATION_STEP != 1U && RAW_GRAB_ROTATION_STEP != 2U
#error RAW_GRAB_ROTATION_STEP_must_be_1_or_2
#endif
/* 机械动作恢复调用robot_task.c中的grab_left/middle/right，机构参数在那里改。 */

/* 首次定位时的排列：1=a、2=b、3=c，百/十/个位分别是左/中/右。
 * 例如任务为红/黑/浅蓝，213表示左黑、中红、右浅蓝，不是颜色编号213。
 */
typedef enum {
    RAW_GRAB_LEFT = 0,
    RAW_GRAB_MIDDLE,
    RAW_GRAB_RIGHT
} RawGrab_Position_t;

extern volatile uint16_t raw_grab_test_layout;
/* 原料只保留三个公开函数，均无static作用域限制。
 * Run：观察两个颜色->定位->按a/b/c抓取->收臂，总流程直接调用它。
 * WaitColorAt：识别下一位中可见的参照颜色，等新帧坐标确认换位，不重新等旧点。
 * position支持LEFT/MIDDLE/RIGHT；MIDDLE为伸长后的中点，非车辆初始定位点。
 * GrabPosition：调用robot_task.c原有grab_left/middle/right，不重复写机械指令。
 */
void RawGrab_WaitColorAt(uint8_t color, RawGrab_Position_t position);
void RawGrab_GrabPosition(RawGrab_Position_t position, uint8_t slot);
/* 阻塞式共用处理入口：参数为实际颜色1~6；只表示动作执行完，不含抓牢检测。
 * 无目标颜色时持续osDelay等待；参数/车辆定位失败返回0，成功返回1。
 * 包含定位、查询开关管理和退出收臂；任务也可直接调用本函数。
 */
uint8_t RawGrab_Run(uint8_t a, uint8_t b, uint8_t c);
void task_judge(uint8_t a, uint8_t b, uint8_t c);
/* 旧固定等待参数，仅保留兼容/备查；当前抓取窗口判断不再使用此值。 */
#define RAW_MATERIAL_NEXT_GRAB_DELAY_MS 3200U

/* 旧GrabMaterialsInOrder接口专用窗口；新RawGrab_Run使用上面的RAW_GRAB参数。
 * 原料抓取窗口：图像像素坐标，不是地图mm；和车辆视觉定位参数独立。
 * 三件均需颜色匹配，并且X/Y分别进入目标±容差，不再额外等固定时间。
 * 容差0表示要求坐标完全相等；调这些值不会修改VisualPosition_Run参数。
 */
#define RAW_MATERIAL_GRAB_TARGET_X_PX       60U
#define RAW_MATERIAL_GRAB_TARGET_Y_PX      100U
#define RAW_MATERIAL_GRAB_TOLERANCE_PX       5U
#define RAW_MATERIAL_GRAB_FRAME_TIMEOUT_MS 300U /* 拒绝超过此时间的视觉数据。 */
#define RAW_MATERIAL_GRAB_POLL_MS            5U /* 不满足时让出CPU，不是抓取等待。 */

typedef enum {
    RAW_MATERIAL_GRAB_WAITING = 0,
    RAW_MATERIAL_GRAB_EXECUTING,
    RAW_MATERIAL_GRAB_DONE
} RawMaterial_GrabStatus_t;

/* 原料抓取调试快照；Watch查看，不要通过修改此结构体调整参数。 */
typedef struct {
    RawMaterial_GrabStatus_t status;
    uint8_t slot;             /* 当前存入的槽1/2/3。 */
    uint8_t target_color;
    uint8_t measured_color;
    uint8_t fresh;            /* 1=本件开始等待后收到的新帧，且没有过期。 */
    uint8_t ready;            /* 1=本件已匹配，已触发/正在执行抓取。 */
    uint16_t target_x_px;
    uint16_t target_y_px;
    uint16_t tolerance_px;
    uint16_t measured_x_px;
    uint16_t measured_y_px;
    int32_t error_x_px;       /* 实测X减目标X。 */
    int32_t error_y_px;       /* 实测Y减目标Y。 */
    uint32_t frame_count;
    uint32_t frame_age_ms;
} RawMaterial_GrabState_t;

extern RawMaterial_GrabState_t g_raw_grab;

/* 等待新视觉帧满足颜色及坐标窗口，依次存入车上1/2/3号槽。
 * 每次机构动作结束后重新等新帧，不使用机构动作前的旧数据抓下一件。
 * 原有机构动作内部的运动等待不变；这只是抓取触发条件。
 */
void GrabMaterialsInOrder(uint8_t a, uint8_t b, uint8_t c);

/*
 * 加工/暂存两圈动作入口；原料两圈均直接调用上面的RawGrab_Run。
 * 原料a/b/c是颜色编号1~6，依次抓取并存入车上1/2/3号槽；
 * 加工/暂存a/b/c是圆环位置编号1~3，分别对应车上1/2/3号槽的物料。
 * 原料接口使用RawGrab_Run并收臂，返回1才允许离站；失败返回0。
 * 加工/暂存接口仅执行机构动作，定位、查询开关和收臂由主任务负责。
 * 颜色编号、车上槽号、工位圆环编号是三套含义，不能混用。
 */
void Processing_First(uint8_t a, uint8_t b, uint8_t c);
void Processing_Second(uint8_t a, uint8_t b, uint8_t c);
/* 暂存第一层单件动作：number为圆环1/2/3，slot为车上存料槽1/2/3。 */
void PutStorageMaterialFromSlot(int number, uint8_t slot);
void Storage_First(uint8_t a, uint8_t b, uint8_t c);
void Storage_Second(uint8_t a, uint8_t b, uint8_t c);

/* 同色码垛状态：只使用扫码快照，和mat_1/2/3独立；可加入Keil Watch。 */
typedef enum {
    STORAGE_STACK_FAULT_NONE = 0,
    STORAGE_STACK_FAULT_BAD_SCAN,       /* 编号非法/重复，或两圈颜色集合不同。 */
    STORAGE_STACK_FAULT_NO_FIRST_LAYER, /* 尚未执行完第一层，禁止第二层码垛。 */
    STORAGE_STACK_FAULT_FIRST_POSITION, /* 第一圈暂存定位未成功。 */
    STORAGE_STACK_FAULT_SECOND_POSITION /* 第二圈物料定位未成功。 */
} StorageStack_Fault_t;

typedef struct {
    StorageStack_Fault_t fault;
    VisualPosition_Result_t position_result;
    uint8_t configured;
    uint8_t first_layer_done;
    uint8_t second_layer_done;
    uint8_t position_attempted;
    uint8_t first_colors[3]; /* 第1组：槽1/2/3的第一圈颜色。 */
    uint8_t first_rings[3];  /* 第2组：槽1/2/3第一层放到的圆环。 */
    uint8_t second_colors[3]; /* 第3组：槽1/2/3的第二圈颜色。 */
    uint8_t ring_color[4];  /* 下标1~3为第一层圆环颜色，下标0不用。 */
    uint8_t target_ring[3]; /* 第二圈槽1/2/3按同色匹配的目标圆环。 */
    uint8_t camera_mode;    /* 识别圆环2物料的模式3~8，不是圆环位置号。 */
} StorageStack_State_t;

extern StorageStack_State_t g_storage_stack;

/* 扫码后调用一次；第4组仍只用于第二圈加工，不参与暂存同色匹配。
 * 两圈颜色必须为同一组三种不同颜色，顺序可以不同。
 */
bool StorageStack_Configure(const uint8_t first_colors[3],
                            const uint8_t first_rings[3],
                            const uint8_t second_colors[3]);
/* 包含原定位/查询开关/收臂流程；成功返回true才允许继续路径。
 * 第一圈识别地面圆环并按第2组放置；第二圈识别圆环2物料后同色叠放。
 * 失败锁存fault，后续调用不重复动作；排除故障后复位重跑。
 */
bool StorageStack_RunFirst(void);
bool StorageStack_RunSecond(void);

void positioning_angle(float current_angle, float target_angle);
void go_map(const path_plan_result_t *pp, int stage);
void grab_material(int number);
/* 当前车上1/2/3号槽对应的颜色编号，抓取动作执行后分别更新。 */
extern volatile uint8_t mat_1,mat_2,mat_3;
void put_material2(int number);
void put_material(int number);

#endif
