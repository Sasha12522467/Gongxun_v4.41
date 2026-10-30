/**
  ******************************************************************************
  * @file    path_plan.c
  * @brief   场地路径规划（A*，3x3 节点图，8 段固定任务）
  *          由「完整路径规划.cpp」移植为纯 C。
  *
  *  地图原点在右上角停车区1，X轴向左为正，Y轴向下为正。
  *  地图原点为场地右上角，坐标取车辆中心。X轴向左、Y轴向下：
  *      (300,350)   0=1区安全点       (275,1200)  1=右侧节点   (275,2060)  2=右下节点
  *      (1200,300)  4=原料区        (1200,1200) 5=中心节点   (1200,2000) 3=精加工区
  *      (2060,320)  7=左上节点      (2050,1200) 6=暂存区     (2060,2060) 8=左下安全节点
  *
  *  边1~12连接相邻节点，代价=曼哈顿路程+转向等效路程。
  *  障碍通过“封锁边”实现绕行。
  *
  *  用法：
  *      把雷达障碍 grid_id 经 grid_to_edges() 映射成边编号，
  *      再 path_plan_run() 一次性算出 8 段路径。
  ******************************************************************************
  */
#include "path_plan.h"
#include <stdlib.h>   /* abs */
#include <float.h>    /* FLT_MAX */
#include <math.h>
#include <string.h>

typedef struct {
    int id;
    int x;
    int y;
} Node_t;

typedef struct {
    int id;
    int from_node;
    int to_node;
    int cost;
    int is_blocked;
    int direction; /* 1=正向，-1=反向 */
} Edge_t;

static Node_t nodes[PP_MAX_NODES];
static Edge_t edges[PP_MAX_EDGES];
static int num_edges = 0;

/* 每条有向边代表一种到达朝向，另设起始/终止状态。
 * 相同节点从不同方向到达不能合并，否则可能丢掉后续转弯更少的路线。
 * 静态工作区避免增加RTOS任务栈；只允许主流程串行调用规划接口。
 */
#define PP_START_STATE (2 * PP_MAX_EDGES)
#define PP_GOAL_STATE  (PP_START_STATE + 1)
#define PP_SEARCH_STATES (PP_GOAL_STATE + 1)
typedef struct {
    float g;
    float f;
    float yaw;
    int node;
    int parent;
    unsigned char closed;
} SearchState_t;
static SearchState_t search_states[PP_SEARCH_STATES];

int path_plan_stage_docking_yaw(int stage, float *yaw_deg)
{
    if (yaw_deg == NULL) return 0;
    switch (stage) {
    case 0: case 3: case 6: *yaw_deg = 0.0f; return 1;
    case 1: case 4: *yaw_deg = -90.0f; return 1;
    case 2: case 5: *yaw_deg = 90.0f; return 1;
    case 7:
        if (path_plan_get_start_profile()->zone == PP_START_ZONE_2 &&
            path_plan_get_start_profile()->final_parking_required != 0U) {
            *yaw_deg = path_plan_get_start_profile()->park_yaw_deg;
            return 1;
        }
        return 0;
    default: return 0;
    }
}

static float edge_yaw(int from_node, int to_node)
{
    return atan2f(-(float)(nodes[to_node].x - nodes[from_node].x),
                   (float)(nodes[to_node].y - nodes[from_node].y)) *
           (180.0f / 3.14159265358979323846f);
}

static float turn_angle(float from_yaw, float to_yaw)
{
    float angle = fabsf(to_yaw - from_yaw);
    while (angle > 180.0f) angle = fabsf(angle - 360.0f);
    return angle > PP_TURN_IGNORE_DEG ? angle : 0.0f;
}

static float turn_cost(float from_yaw, float to_yaw)
{
    float angle = turn_angle(from_yaw, to_yaw);
    return angle > 0.0f ? PP_TURN_START_COST_MM +
                         PP_TURN_90_COST_MM * angle / 90.0f : 0.0f;
}

static int node_cost(int from_node, int to_node)
{
    return abs(nodes[to_node].x - nodes[from_node].x) +
           abs(nodes[to_node].y - nodes[from_node].y);
}

/* 8段工位顺序固定；只有第0段起点和第7段终点随启停区选择切换。 */
static int stages[PP_MAX_STAGES][2] = {
    {0, 1},  /* 段1: 所选启停区的接入节点 -> 扫码节点 */
    {1, 4},  /* 段2: 中维节点 -> 原料区 */
    {4, 3},  /* 段3: 原料区   -> 精加工区 */
    {3, 6},  /* 段4: 精加工区 -> 仓储区 */
    {6, 4},  /* 段5: 仓储区   -> 原料区 */
    {4, 3},  /* 段6: 原料区   -> 精加工区 */
    {3, 6},  /* 段7: 精加工区 -> 仓储区 */
    {6, 0}   /* 段8: 仓储区   -> 所选启停区的接入节点 */
};

/* 1区node0固定为场内安全接入点；停车中心只用于起始/最终停车。
 * 2区仍通过现有右下节点2接入，不移动node0到2区，障碍映射保持不变。
 */
static const PathPlan_StartProfile_t start_profiles[2] = {
    {PP_START_ZONE_1, 0,
     PP_ZONE1_CENTER_X_MM, PP_ZONE1_CENTER_Y_MM, (float)PP_START_YAW_DEG,
     PP_START_EXIT_X_MM, PP_START_EXIT_Y_MM, PP_START_EXIT_YAW_DEG,
     PP_ZONE1_CENTER_X_MM, PP_ZONE1_CENTER_Y_MM, 0.0f, 1U},
    {PP_START_ZONE_2, 2,
     PP_ZONE2_INITIAL_X_MM, PP_ZONE2_INITIAL_Y_MM, PP_ZONE2_INITIAL_YAW_DEG,
     PP_ZONE2_EXIT_X_MM, PP_ZONE2_EXIT_Y_MM, PP_ZONE2_EXIT_YAW_DEG,
     PP_ZONE2_PARK_X_MM, PP_ZONE2_PARK_Y_MM, PP_ZONE2_PARK_YAW_DEG, 1U}
};
static uint8_t selected_start_zone = PP_START_ZONE_1;

int path_plan_select_start_zone(uint8_t zone)
{
    const PathPlan_StartProfile_t *profile;
    if (zone != PP_START_ZONE_1 && zone != PP_START_ZONE_2) return 0;
    selected_start_zone = zone;
    profile = path_plan_get_start_profile();
    stages[0][0] = profile->route_node;
    stages[PP_MAX_STAGES - 1][1] = profile->route_node;
    return 1;
}

uint8_t path_plan_get_start_zone(void)
{
    return selected_start_zone;
}

const PathPlan_StartProfile_t *path_plan_get_start_profile(void)
{
    return &start_profiles[selected_start_zone - 1U];
}

static void initialize_graph(void)
{
    nodes[0].id = 0;
    nodes[0].x = PP_START_EXIT_X_MM;
    nodes[0].y = PP_START_EXIT_Y_MM; /* 1区安全接入点，中途经过不驶入停车中心。 */
    nodes[1].id = 1; nodes[1].x = 300;  nodes[1].y = 1240; /* 右侧通道中心 */
    nodes[2].id = 2; nodes[2].x = 275;  nodes[2].y = 2060; /* 右下通道中心 */
    nodes[3].id = 3; nodes[3].x = 1200; nodes[3].y = 2030; /* 加工区避让点 */
    nodes[4].id = 4; nodes[4].x = 1230; nodes[4].y = 340;  /* 原料区避让点 */
    nodes[5].id = 5; nodes[5].x = 1200; nodes[5].y = 1200; /* 场地中心 */
    nodes[6].id = 6; nodes[6].x = 2000; nodes[6].y = 1200; /* 暂存区避让点 */
    nodes[7].id = 7; nodes[7].x = 2000; nodes[7].y = 320;  /* 左上通道中心 */
    /* 左下角没有工位；为300 mm车体和里程计误差保留约100 mm附加余量。 */
    nodes[8].id = 8; nodes[8].x = 2060; nodes[8].y = 2060; /* 左下安全节点 */

    edges[0].id = 1;  edges[0].from_node = 0; edges[0].to_node = 1; edges[0].cost = node_cost(0, 1); edges[0].is_blocked = 0; edges[0].direction = 1;
    edges[1].id = 2;  edges[1].from_node = 1; edges[1].to_node = 2; edges[1].cost = node_cost(1, 2); edges[1].is_blocked = 0; edges[1].direction = 1;
    edges[2].id = 3;  edges[2].from_node = 0; edges[2].to_node = 4; edges[2].cost = node_cost(0, 4); edges[2].is_blocked = 0; edges[2].direction = 1;
    edges[3].id = 4;  edges[3].from_node = 1; edges[3].to_node = 5; edges[3].cost = node_cost(1, 5); edges[3].is_blocked = 0; edges[3].direction = 1;
    edges[4].id = 5;  edges[4].from_node = 2; edges[4].to_node = 3; edges[4].cost = node_cost(2, 3); edges[4].is_blocked = 0; edges[4].direction = 1;
    edges[5].id = 6;  edges[5].from_node = 4; edges[5].to_node = 5; edges[5].cost = node_cost(4, 5); edges[5].is_blocked = 0; edges[5].direction = 1;
    edges[6].id = 7;  edges[6].from_node = 5; edges[6].to_node = 3; edges[6].cost = node_cost(5, 3); edges[6].is_blocked = 0; edges[6].direction = 1;
    edges[7].id = 8;  edges[7].from_node = 4; edges[7].to_node = 7; edges[7].cost = node_cost(4, 7); edges[7].is_blocked = 0; edges[7].direction = 1;
    edges[8].id = 9;  edges[8].from_node = 5; edges[8].to_node = 6; edges[8].cost = node_cost(5, 6); edges[8].is_blocked = 0; edges[8].direction = 1;
    edges[9].id = 10; edges[9].from_node = 3; edges[9].to_node = 8; edges[9].cost = node_cost(3, 8); edges[9].is_blocked = 0; edges[9].direction = 1;
    edges[10].id = 11; edges[10].from_node = 7; edges[10].to_node = 6; edges[10].cost = node_cost(7, 6); edges[10].is_blocked = 0; edges[10].direction = 1;
    edges[11].id = 12; edges[11].from_node = 6; edges[11].to_node = 8; edges[11].cost = node_cost(6, 8); edges[11].is_blocked = 0; edges[11].direction = 1;

    num_edges = 12;
}

void path_plan_init(void)
{
    initialize_graph();
}

/* 把障碍边全部封锁 */
static void update_edge_status_by_id(const int *obstacle_edges, int num_obstacles)
{
    int i, j;
    for (i = 0; i < num_edges; i++) edges[i].is_blocked = 0;
    for (i = 0; i < num_obstacles; i++) {
        int edge_id = obstacle_edges[i];
        for (j = 0; j < num_edges; j++) {
            if (edges[j].id == edge_id) { edges[j].is_blocked = 1; break; }
        }
    }
}

/* 起步规划临时使用离站坐标；恢复后node0仍是安全点，不是停车中心。
 * 同步边长和朝向，让少转向成本与实际起步路线一致，不改变障碍映射。
 */
static void set_start_coordinates(int node, int x_mm, int y_mm)
{
    int i;
    nodes[node].x = x_mm;
    nodes[node].y = y_mm;
    for (i = 0; i < num_edges; ++i)
        edges[i].cost = node_cost(edges[i].from_node, edges[i].to_node);
}

static void relax_state(int current, int next, float cost, float heuristic)
{
    float tentative = search_states[current].g + cost;
    if (tentative < search_states[next].g) {
        search_states[next].g = tentative;
        search_states[next].f = tentative + heuristic;
        search_states[next].parent = current;
        search_states[next].closed = 0U;
    }
}

/* A*在有向边状态上搜索。距离启发式不含转向，仍是总代价的下界。
 * 到达终点后还要经过虚拟终止状态，计入工位对接转向再比较路线。
 * 否则先到终点的路线可能需要大幅对接旋转，实际反而更慢。
 */
static int a_star(int stage, path_plan_result_t *result)
{
    int start_node = stages[stage][0];
    int end_node = stages[stage][1];
    float start_yaw = path_plan_get_start_profile()->exit_yaw_deg;
    float docking_yaw = 0.0f;
    int has_docking = path_plan_stage_docking_yaw(stage, &docking_yaw);
    int i;
    if (stage > 0) (void)path_plan_stage_docking_yaw(stage - 1, &start_yaw);

    for (i = 0; i < PP_SEARCH_STATES; ++i) {
        search_states[i].g = FLT_MAX;
        search_states[i].f = FLT_MAX;
        search_states[i].parent = -1;
        search_states[i].closed = 0U;
    }
    for (i = 0; i < num_edges; ++i) {
        search_states[2 * i].node = edges[i].to_node;
        search_states[2 * i].yaw = edge_yaw(edges[i].from_node, edges[i].to_node);
        search_states[2 * i + 1].node = edges[i].from_node;
        search_states[2 * i + 1].yaw = edge_yaw(edges[i].to_node, edges[i].from_node);
    }
    search_states[PP_START_STATE].node = start_node;
    search_states[PP_START_STATE].yaw = start_yaw;
    search_states[PP_START_STATE].g = 0.0f;
    search_states[PP_START_STATE].f = (float)node_cost(start_node, end_node);

    while (1) {
        int current = -1;
        float min_f = FLT_MAX;
        for (i = 0; i < PP_SEARCH_STATES; ++i) {
            if (!search_states[i].closed && search_states[i].f < min_f) {
                current = i;
                min_f = search_states[i].f;
            }
        }
        if (current < 0) return 0;
        if (current == PP_GOAL_STATE) {
            int count = 0;
            float previous_yaw = start_yaw;
            current = search_states[current].parent;
            while (current != PP_START_STATE) {
                int edge_index = current / 2;
                if (current < 0 || count >= PP_MAX_PATH_LEN) return 0;
                result->edge_path[stage][count++] = (current % 2 == 0 ? 1 : -1) *
                                                  edges[edge_index].id;
                current = search_states[current].parent;
            }
            for (i = 0; i < count / 2; ++i) {
                int temp = result->edge_path[stage][i];
                result->edge_path[stage][i] = result->edge_path[stage][count - 1 - i];
                result->edge_path[stage][count - 1 - i] = temp;
            }
            result->length[stage] = count;
            result->total_cost[stage] = search_states[PP_GOAL_STATE].g;
            for (i = 0; i <= count; ++i) {
                float angle, yaw;
                if (i == count) {
                    if (!has_docking) break;
                    yaw = docking_yaw;
                } else {
                    int signed_edge = result->edge_path[stage][i];
                    int index = abs(signed_edge) - 1;
                    yaw = search_states[2 * index + (signed_edge < 0 ? 1 : 0)].yaw;
                    result->distance_mm[stage] += edges[index].cost;
                }
                angle = turn_angle(previous_yaw, yaw);
                if (angle > 0.0f) result->turn_count[stage]++;
                result->turn_degrees[stage] += angle;
                previous_yaw = yaw;
            }
            return 1;
        }
        search_states[current].closed = 1U;
        if (search_states[current].node == end_node) {
            relax_state(current, PP_GOAL_STATE,
                        has_docking ? turn_cost(search_states[current].yaw, docking_yaw) : 0.0f,
                        0.0f);
            continue;
        }
        for (i = 0; i < num_edges; ++i) {
            int next;
            if (edges[i].is_blocked) continue;
            if (edges[i].from_node == search_states[current].node) next = 2 * i;
            else if (edges[i].to_node == search_states[current].node) next = 2 * i + 1;
            else continue;
            relax_state(current, next, (float)edges[i].cost +
                        turn_cost(search_states[current].yaw, search_states[next].yaw),
                        (float)node_cost(search_states[next].node, end_node));
        }
    }
}

/* 雷达障碍 grid_id -> 要封锁的边编号（映射表由现场标定提供） */
int grid_to_edges(int grid_id, int edges_out[2])
{
    switch (grid_id) {
        case 1:  case 2:  edges_out[0] = 2;  return 1;
        case 3:           edges_out[0] = 1;  return 1;
        case 5:           edges_out[0] = 3;  return 1;
        case 13:          edges_out[0] = 6;  return 1;
        case 11:          edges_out[0] = 7;  return 1;
        case 15:          edges_out[0] = 8;  return 1;
        case 24:          edges_out[0] = 11; edges_out[1] = 8;  return 2;
        case 23:          edges_out[0] = 11; return 1;
        case 21:          edges_out[0] = 12; return 1;
        case 20:          edges_out[0] = 12; edges_out[1] = 10; return 2;
        case 19:          edges_out[0] = 10; return 1;
        case 17:          edges_out[0] = 9;  return 1;
        case 7:           edges_out[0] = 4;  return 1;
        case 9:           edges_out[0] = 5;  return 1;
        default:          return 0;   /* 该方格不封锁任何边，或 grid_id 越界 */
    }
}

void path_plan_run(const int *grid_ids, int grid_count, path_plan_result_t *result)
{
    int obstacle_edges[PP_MAX_OBSTACLE_EDGES];
    int num_obstacle_edges = 0;
    int i;
    const PathPlan_StartProfile_t *profile = path_plan_get_start_profile();

    if (result == NULL) return;

    memset(result, 0, sizeof(*result));
    result->start_zone = profile->zone;
    if (grid_ids == NULL) grid_count = 0;

    initialize_graph();

    /* 把每个障碍 grid_id 映射成边，收集并去重 */
    for (i = 0; i < grid_count; i++) {
        int e[2];
        int n = grid_to_edges(grid_ids[i], e);
        int k;
        for (k = 0; k < n; k++) {
            int dup = 0, m;
            for (m = 0; m < num_obstacle_edges; m++) {
                if (obstacle_edges[m] == e[k]) { dup = 1; break; }
            }
            if (!dup && num_obstacle_edges < PP_MAX_OBSTACLE_EDGES)
                obstacle_edges[num_obstacle_edges++] = e[k];
        }
    }

    update_edge_status_by_id(obstacle_edges, num_obstacle_edges);

    /* 逐段规划，结果存进 8 个数组 */
    for (i = 0; i < PP_MAX_STAGES; i++) {
        int route_node = profile->route_node;
        int initial_x = nodes[route_node].x;
        int initial_y = nodes[route_node].y;
        int exit_distance = 0;
        if (i == 0) {
            exit_distance = (int)(fabsf(profile->exit_x_mm - profile->initial_x_mm) +
                                  fabsf(profile->exit_y_mm - profile->initial_y_mm));
            set_start_coordinates(route_node, (int)profile->exit_x_mm,
                                  (int)profile->exit_y_mm);
        }
        result->ok[i] = a_star(i, result);
        if (i == 0) {
            if (result->ok[i]) {
                /* 固定航向平移不计转弯，距离统计仍统一使用曼哈顿距离。 */
                result->distance_mm[i] += exit_distance;
                result->total_cost[i] += (float)exit_distance;
            }
            set_start_coordinates(route_node, initial_x, initial_y);
        }
        if (i == PP_MAX_STAGES - 1 && result->ok[i] &&
            profile->final_parking_required != 0U) {
            /* 1区保持node0到达航向入库；2区仍先在node2完成停车转向。
             * 固定回库段不新增道路、不改变grid映射，只追加距离统计；
             * 安全节点的对接转向已由A*的虚拟终止状态计入总代价。
             */
            int parking_distance = (int)(fabsf(profile->park_x_mm - initial_x) +
                                         fabsf(profile->park_y_mm - initial_y));
            result->distance_mm[i] += parking_distance;
            result->total_cost[i] += (float)parking_distance;
        }
        if (!result->ok[i]) result->length[i] = 0;
    }
}

int path_plan_edge_points(int signed_edge,
                          float *start_x_mm, float *start_y_mm,
                          float *target_x_mm, float *target_y_mm)
{
    int edge_id = signed_edge < 0 ? -signed_edge : signed_edge;
    int i;

    if (signed_edge == 0 || start_x_mm == NULL || start_y_mm == NULL ||
        target_x_mm == NULL || target_y_mm == NULL) return 0;
    if (num_edges == 0) initialize_graph();

    for (i = 0; i < num_edges; i++) {
        if (edges[i].id == edge_id) {
            int start_node = signed_edge > 0 ?
                edges[i].from_node : edges[i].to_node;
            int target_node = signed_edge > 0 ?
                edges[i].to_node : edges[i].from_node;
            *start_x_mm = (float)nodes[start_node].x;
            *start_y_mm = (float)nodes[start_node].y;
            *target_x_mm = (float)nodes[target_node].x;
            *target_y_mm = (float)nodes[target_node].y;
            return 1;
        }
    }
    return 0;
}

int path_plan_edge_target(int signed_edge, float *x_mm, float *y_mm)
{
    float start_x;
    float start_y;
    return path_plan_edge_points(signed_edge, &start_x, &start_y,
                                 x_mm, y_mm);
}
