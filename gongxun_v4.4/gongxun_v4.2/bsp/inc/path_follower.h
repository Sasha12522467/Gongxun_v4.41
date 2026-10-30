#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

#include "path_plan.h"

typedef enum {
    PATH_FOLLOWER_IDLE = 0,
    PATH_FOLLOWER_WAIT_CHASSIS,
    PATH_FOLLOWER_TURNING,
    PATH_FOLLOWER_WAIT_FORWARD,
    PATH_FOLLOWER_MOVING_FORWARD,
    PATH_FOLLOWER_DOCKING_TURN,
    PATH_FOLLOWER_WAIT_ACTION,
    PATH_FOLLOWER_DONE,
    PATH_FOLLOWER_FAULT,
    PATH_FOLLOWER_EXIT_WAIT_FEEDBACK, /* 动作后等待有效回读，补计视觉微调位移。 */
    PATH_FOLLOWER_EXIT_MOVING,        /* 保持工位航向，全向退到通道安全点。 */
    PATH_FOLLOWER_START_EXIT_MOVING,  /* 保持配置的起始航向驶入场内，尚不允许路径转向。 */
    PATH_FOLLOWER_PARKING_TURN,       /* 2区：在右下安全节点转到停车航向。 */
    PATH_FOLLOWER_PARKING_MOVING      /* 2区：保持停车航向全向入库，禁止库内自旋。 */
} PathFollower_Status_t;

typedef enum {
    PATH_FOLLOWER_FAULT_NONE = 0,
    PATH_FOLLOWER_FAULT_BAD_PLAN,
    PATH_FOLLOWER_FAULT_BAD_EDGE,
    PATH_FOLLOWER_FAULT_CHASSIS,
    PATH_FOLLOWER_FAULT_QUEUE
} PathFollower_Fault_t;

/* 联合控制调试快照，可直接加入Keil Watch。stage/edge下标均从0开始。 */
typedef struct {
    PathFollower_Status_t status;
    PathFollower_Fault_t fault;
    uint8_t stage_index;
    uint8_t edge_index;
    uint8_t completed_stage;
    uint8_t turn_skipped; /* 本条边已在航向容差内，直接前进修正角度。 */
    uint8_t startup_exit_done; /* 1=起点已驶入安全区，才允许执行第一条边。 */
    uint8_t start_zone; /* 本次执行锁定的启停区：1=PD12，2=PD13。 */
    int16_t signed_edge;
    uint16_t completed_waypoints;
    uint16_t total_waypoints;
    uint16_t skipped_turns; /* 本次任务累计省去的独立转向次数。 */
    uint32_t active_sequence;
    float target_x_mm;
    float target_y_mm;
    float target_yaw_deg;
} PathFollower_State_t;

extern PathFollower_State_t g_path_follower;

/* 复制规划结果并准备执行；不会阻塞，也不会立即越过未就绪的底盘。 */
bool PathFollower_Start(const path_plan_result_t *plan);

/* 主任务周期调用：明显换向先旋转；已在容差内直接前进并修正航向。 */
void PathFollower_Process(void);

/* 工位动作完成后调用；原料/加工/暂存区先保持工位航向全向离站，再进入下一段。 */
bool PathFollower_Continue(void);

/* 停止当前路径并向底盘发送停止命令。 */
void PathFollower_Stop(void);

#endif /* PATH_FOLLOWER_H */
