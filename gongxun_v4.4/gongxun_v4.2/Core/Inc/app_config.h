#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/*
 * 雷达开关（后续只改这里即可恢复）：
 * 0：当前未接雷达，不初始化UART5、不发扫描指令；
 *    按地图无新增障碍运行A*，保留固定地图约束和原有转向代价。
 * 1：接回雷达后，恢复UART5初始化、启动扫描和障碍绕行规划。
 * 无雷达测试前必须确认路线确实没有新增障碍，不能自动躲避未知物体。
 */
#ifndef APP_RADAR_ENABLED
#define APP_RADAR_ENABLED 1
#endif

/* 任务数据来源：1=手动测试，不等待扫码；0=恢复实际扫码。
 * 手动模式仍会行驶到扫码区，再按下列四组任务继续完整流程。
 * 只跳过扫码等待，不跳过后续视觉定位、抓取和码垛动作。
 */
#ifndef APP_MANUAL_TASK_ENABLED
#define APP_MANUAL_TASK_ENABLED 0
#endif

/* 每组是三位十进制数，三位不能重复；修改后重新编译、烧录并复位。
 * 第1/3组：两圈取料颜色，各位范围1~6，两组必须是相同颜色的排列。
 * 第2/4组：两圈加工圆环，各位范围1~3；第一圈暂存沿用第2组，
 * 第二圈暂存自动按颜色匹配第一层，不直接使用第4组放置顺序。
 * 当前123+123+123+123：两圈均取颜色1/2/3，加工圆环均为1/2/3。
 */
#define APP_MANUAL_TASK_GROUP1 123U
#define APP_MANUAL_TASK_GROUP2 123U
#define APP_MANUAL_TASK_GROUP3 123U
#define APP_MANUAL_TASK_GROUP4 123U

/* 主流程选区/准备状态，可在Watch查看g_app_start。按键只在WAIT_KEY读取。 */
typedef enum {
    APP_START_WAIT_KEY = 0,
    APP_START_WAIT_FEEDBACK,
    APP_START_WAIT_POSE,
    APP_START_PLANNING,
    APP_START_RUNNING,
    APP_START_DONE,
    APP_START_ERROR
} AppStart_Status_t;

typedef enum {
    APP_START_FAULT_NONE = 0,
    APP_START_FAULT_BAD_ZONE,
    APP_START_FAULT_FEEDBACK_TIMEOUT,
    APP_START_FAULT_CHASSIS,
    APP_START_FAULT_POSE_QUEUE,
    APP_START_FAULT_POSE_TIMEOUT,
    APP_START_FAULT_PLAN,
    APP_START_FAULT_FOLLOWER
} AppStart_Fault_t;

typedef struct {
    AppStart_Status_t status;
    AppStart_Fault_t fault;
    uint8_t selected_zone; /* 0=未按键；1=PD12；2=PD13，上电只选择一次。 */
    uint32_t pose_sequence; /* 必须等电机任务completed_sequence匹配才允许规划。 */
} AppStart_State_t;

extern AppStart_State_t g_app_start;

#endif /* APP_CONFIG_H */
