#ifndef RADAR_STARTUP_H
#define RADAR_STARTUP_H

#include <stdbool.h>
#include <stdint.h>

#include "path_plan.h"
#include "radar_v6.h"

/*
 * 雷达只在系统启动阶段工作：连接 -> 选区 -> 方格扫描 -> A*路径规划。
 * 所有接口均由主流程任务调用，不需要额外创建雷达任务。
 */
typedef enum {
    RADAR_STARTUP_IDLE = 0,
    RADAR_STARTUP_WAIT_READY,
    RADAR_STARTUP_SCANNING,
    RADAR_STARTUP_PLANNING,
    RADAR_STARTUP_DONE,
    RADAR_STARTUP_ERROR
} RadarStartup_Status_t;

typedef enum {
    RADAR_STARTUP_FAULT_NONE = 0,
    RADAR_STARTUP_FAULT_BAD_PARAMETER,
    RADAR_STARTUP_FAULT_REQUEST_REJECTED,
    RADAR_STARTUP_FAULT_RADAR,
    RADAR_STARTUP_FAULT_TIMEOUT,
    RADAR_STARTUP_FAULT_NO_PATH
} RadarStartup_Fault_t;

/* 可直接加入Keil Watch，观察雷达启动和规划过程。 */
typedef struct {
    RadarStartup_Status_t status;
    RadarStartup_Fault_t fault;
    uint8_t node_id;
    uint8_t zone;
    uint8_t scan_requested;
    uint8_t grid_received;
    uint8_t grid_count;
    uint8_t path_all_valid;
    uint32_t start_ms;
    uint32_t timeout_ms;
    uint32_t scan_sequence;
    radar_client_request_result_t last_request;
    radar_client_error_t last_radar_error;
    radar_client_state_t radar_state;
    uint8_t grid_ids[RADAR_CLIENT_MAX_GRID_IDS];
    path_plan_result_t path;
} RadarStartup_State_t;

extern RadarStartup_State_t g_radar_startup;

/*
 * 开始一次启动扫描。
 * node_id通常为1；zone必须为RADAR_CLIENT_ZONE_1或ZONE_2；
 * timeout_ms为整个连接、扫描和规划流程的总超时。
 */
bool RadarStartup_Start(uint8_t node_id, uint8_t zone, uint32_t timeout_ms);

/* 无雷达测试：不初始化通信、不扫描，直接按无新增障碍生成8段A*路径。
 * 清空旧障碍，保留DONE/ERROR及GetPath接口，便于原主流程继续执行。
 * 返回true表示全部规划成功，不代表已经执行或探测过真实障碍。
 */
bool RadarStartup_PlanWithoutRadar(void);

/* 主流程任务每5 ms调用一次，直到Finished返回true。 */
void RadarStartup_Process(void);

bool RadarStartup_IsFinished(void);
bool RadarStartup_IsSuccessful(void);

/* 仅在RadarStartup_IsSuccessful()为true时返回有效路径。 */
const path_plan_result_t *RadarStartup_GetPath(void);

#endif /* RADAR_STARTUP_H */
