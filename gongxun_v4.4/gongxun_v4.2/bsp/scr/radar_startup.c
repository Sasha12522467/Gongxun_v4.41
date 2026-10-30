#include "radar_startup.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define RADAR_STARTUP_DEFAULT_TIMEOUT_MS 20000U

RadarStartup_State_t g_radar_startup;

/* 回调由RadarClient_Poll()触发，因此和主流程处在同一个任务上下文。 */
static void RadarStartup_OnState(const radar_client_state_t *state)
{
    if (state != NULL) {
        g_radar_startup.radar_state = *state;
    }
}

static void RadarStartup_OnGrid(const radar_client_grid_result_t *grid)
{
    uint8_t index;

    if (grid == NULL || grid->count > RADAR_CLIENT_MAX_GRID_IDS) {
        return;
    }

    g_radar_startup.scan_sequence = grid->scan_sequence;
    g_radar_startup.grid_count = grid->count;
    for (index = 0U; index < grid->count; ++index) {
        g_radar_startup.grid_ids[index] = grid->grid_ids[index];
    }
    g_radar_startup.grid_received = 1U;
}

static void RadarStartup_OnError(const radar_client_error_t *error)
{
    if (error == NULL) {
        return;
    }

    g_radar_startup.last_radar_error = *error;

    /*
     * 建链和选区阶段由雷达驱动自动重试，最终由总超时兜底。
     * 扫描事务已经开始后发生协议、设备或链路错误则立即停止启动流程。
     */
    if (g_radar_startup.status == RADAR_STARTUP_SCANNING &&
        error->operation == RADAR_CLIENT_OPERATION_GRID_SCAN) {
        g_radar_startup.fault = RADAR_STARTUP_FAULT_RADAR;
        g_radar_startup.status = RADAR_STARTUP_ERROR;
    }
}

static void RadarStartup_RunPathPlan(void)
{
    int grid_ids[RADAR_CLIENT_MAX_GRID_IDS];
    uint8_t index;

    for (index = 0U; index < g_radar_startup.grid_count; ++index) {
        grid_ids[index] = (int)g_radar_startup.grid_ids[index];
    }

    path_plan_run(grid_ids, (int)g_radar_startup.grid_count,
                  &g_radar_startup.path);

    g_radar_startup.path_all_valid = 1U;
    for (index = 0U; index < PP_MAX_STAGES; ++index) {
        if (g_radar_startup.path.ok[index] == 0) {
            g_radar_startup.path_all_valid = 0U;
            break;
        }
    }

    if (g_radar_startup.path_all_valid != 0U) {
        g_radar_startup.status = RADAR_STARTUP_DONE;
    } else {
        /* 规划失败时保留path内容，便于在Watch中查看具体失败段。 */
        g_radar_startup.fault = RADAR_STARTUP_FAULT_NO_PATH;
        g_radar_startup.status = RADAR_STARTUP_ERROR;
    }
}

bool RadarStartup_PlanWithoutRadar(void)
{
    /* 清空上次障碍和雷达状态，不伪造雷达已连接或已收到扫描结果。 */
    memset(&g_radar_startup, 0, sizeof(g_radar_startup));
    g_radar_startup.start_ms = HAL_GetTick();
    g_radar_startup.status = RADAR_STARTUP_PLANNING;
    g_radar_startup.zone = path_plan_get_start_zone(); /* 选区信息，不表示雷达通信成功。 */

    /* grid_count=0：原地图约束仍有效，只是不添加雷达扫描的封锁道路。 */
    RadarStartup_RunPathPlan();
    return RadarStartup_IsSuccessful();
}

bool RadarStartup_Start(uint8_t node_id, uint8_t zone, uint32_t timeout_ms)
{
    radar_client_callbacks_t callbacks;
    radar_client_request_result_t request;

    memset(&g_radar_startup, 0, sizeof(g_radar_startup));
    memset(&callbacks, 0, sizeof(callbacks));

    g_radar_startup.node_id = node_id;
    g_radar_startup.zone = zone;
    g_radar_startup.timeout_ms =
        (timeout_ms == 0U) ? RADAR_STARTUP_DEFAULT_TIMEOUT_MS : timeout_ms;
    g_radar_startup.start_ms = HAL_GetTick();
    g_radar_startup.last_request = RADAR_CLIENT_REQUEST_ACCEPTED;

    if (node_id < 1U || node_id > 254U ||
        (zone != RADAR_CLIENT_ZONE_1 && zone != RADAR_CLIENT_ZONE_2)) {
        g_radar_startup.fault = RADAR_STARTUP_FAULT_BAD_PARAMETER;
        g_radar_startup.status = RADAR_STARTUP_ERROR;
        return false;
    }

    callbacks.state = RadarStartup_OnState;
    callbacks.grid = RadarStartup_OnGrid;
    callbacks.error = RadarStartup_OnError;

    g_radar_startup.status = RADAR_STARTUP_WAIT_READY;
    RadarClient_Init(&callbacks, node_id);

    /* 未建链时该请求会先保存目标区域，HELLO成功后自动完成选区。 */
    request = RadarClient_RequestSelectZone(zone);
    g_radar_startup.last_request = request;
    if (request != RADAR_CLIENT_REQUEST_ACCEPTED) {
        g_radar_startup.fault = RADAR_STARTUP_FAULT_REQUEST_REJECTED;
        g_radar_startup.status = RADAR_STARTUP_ERROR;
        return false;
    }

    return true;
}

void RadarStartup_Process(void)
{
    uint32_t now_ms;

    if (g_radar_startup.status == RADAR_STARTUP_IDLE ||
        g_radar_startup.status == RADAR_STARTUP_DONE ||
        g_radar_startup.status == RADAR_STARTUP_ERROR) {
        return;
    }

    /* 必须持续Poll，协议收包、握手、超时和用户回调都从这里执行。 */
    RadarClient_Poll();

    if (g_radar_startup.status == RADAR_STARTUP_ERROR) {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_radar_startup.start_ms) >=
        g_radar_startup.timeout_ms) {
        g_radar_startup.fault = RADAR_STARTUP_FAULT_TIMEOUT;
        g_radar_startup.status = RADAR_STARTUP_ERROR;
        return;
    }

    if (g_radar_startup.status == RADAR_STARTUP_WAIT_READY) {
        if (RadarClient_IsReady() && !RadarClient_IsBusy()) {
            radar_client_request_result_t request =
                RadarClient_RequestGridScan();
            g_radar_startup.last_request = request;

            if (request == RADAR_CLIENT_REQUEST_ACCEPTED) {
                g_radar_startup.scan_requested = 1U;
                g_radar_startup.status = RADAR_STARTUP_SCANNING;
            } else if (request == RADAR_CLIENT_REQUEST_BAD_PARAMETER) {
                g_radar_startup.fault = RADAR_STARTUP_FAULT_REQUEST_REJECTED;
                g_radar_startup.status = RADAR_STARTUP_ERROR;
            }
            /* NOT_READY、BUSY或QUEUE_FULL留到下一周期自动重试。 */
        }
        return;
    }

    if (g_radar_startup.status == RADAR_STARTUP_SCANNING &&
        g_radar_startup.grid_received != 0U &&
        !RadarClient_IsBusy()) {
        /*
         * GRID事件可能早于扫描完成状态到达，所以必须等事务真正空闲后
         * 才停止Poll并开始规划。
         */
        g_radar_startup.status = RADAR_STARTUP_PLANNING;
        RadarStartup_RunPathPlan();
    }
}

bool RadarStartup_IsFinished(void)
{
    return g_radar_startup.status == RADAR_STARTUP_DONE ||
           g_radar_startup.status == RADAR_STARTUP_ERROR;
}

bool RadarStartup_IsSuccessful(void)
{
    return g_radar_startup.status == RADAR_STARTUP_DONE;
}

const path_plan_result_t *RadarStartup_GetPath(void)
{
    return RadarStartup_IsSuccessful() ? &g_radar_startup.path : NULL;
}
