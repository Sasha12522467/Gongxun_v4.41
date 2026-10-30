/**
  ******************************************************************************
  * @file    path_plan.h
  * @brief   场地路径规划（A*，3x3 节点图，8 段固定任务）
  *          由「完整路径规划.cpp」移植为纯 C，供 Keil/MDK 直接使用。
  *          地图原点为右上角停车区1，X轴向左，Y轴向下。
  ******************************************************************************
  */
#ifndef PATH_PLAN_H
#define PATH_PLAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PP_MAX_NODES          20   /* 最大节点数 */
#define PP_MAX_EDGES          30   /* 最大边数 */
#define PP_MAX_STAGES         8    /* 固定 8 段任务 */
#define PP_MAX_PATH_LEN       PP_MAX_EDGES   /* 单段路径最大长度 */
#define PP_MAX_OBSTACLE_EDGES 12   /* 最多可封锁的边数（12 条边全封锁为上限） */

/* 两个启停区共用右上角原点的全局地图，不镜像工位或障碍编号。 */
#define PP_FIELD_SIZE_MM      2400
/* 初始0度时的车体外形：X方向275mm，Y方向300mm，随航向一起旋转。 */
#define PP_ROBOT_SIZE_X_MM      275
#define PP_ROBOT_SIZE_Y_MM      300
/* 兼容旧的单值接口，取较长边；实际包络计算应使用上面的两个尺寸。 */
#define PP_ROBOT_SIZE_MM        PP_ROBOT_SIZE_Y_MM

/* 唯一起始摆车航向配置，单位度；只在规划/启动前修改，重新编译并复位生效。
 * 从上方看，相对现在车头朝地图Y+的0度摆法：
 *   0：当前默认；90：逆时针转90度；-90：顺时针转90度；180/-180：反向。
 * 只填上述整数。两启停区的初始IMU对齐、保持航向离站和A*初始朝向
 * 自动跟随；地图坐标、工位工作朝向、现有回库流程不旋转。
 * 起步禁止在蓝块内先转回0度，必须保持此角度移入安全点后再转向。
 */
#ifndef PP_START_YAW_DEG
#define PP_START_YAW_DEG          90
#endif
#if (PP_START_YAW_DEG != 0) && (PP_START_YAW_DEG != 90) && \
    (PP_START_YAW_DEG != -90) && (PP_START_YAW_DEG != 180) && \
    (PP_START_YAW_DEG != -180)
#error PP_START_YAW_DEG_must_be_0_90_minus90_180_or_minus180
#endif

/* 1区道路接入/途经安全点，也是起步离站点，车辆中心坐标，单位mm。
 * 起步保持PP_START_YAW_DEG移入场内后再转向；中途绕行也使用这里。
 * 275x300车体自旋半径约203.5mm，目标距两侧边界300/350mm。
 * 停车中心仅用于起始/最终停车，最终从安全点保持到达航向平移入库。
 */
#define PP_START_EXIT_X_MM      300
#define PP_START_EXIT_Y_MM      350
#define PP_START_EXIT_YAW_DEG     ((float)PP_START_YAW_DEG) /* 跟随初始摆车，不单独改。 */

#define PP_ZONE1_CENTER_X_MM      150
#define PP_ZONE1_CENTER_Y_MM      150

/* 2号启停区：初始/离站航向跟随PP_START_YAW_DEG；回库仍保持原来的0度。
 * 先保持航向全向移到离站点，再允许转向；返回时先在右下节点转正，
 * 然后保持0度全向入库。中心(150,2250)是用户指定的蓝块正中心，
 * 300mm车长与场地下边没有几何余量，必须实车确认误差，禁止在中心自旋。
 */
#define PP_START_ZONE_1             1U
#define PP_START_ZONE_2             2U
#define PP_ZONE2_INITIAL_X_MM     150
#define PP_ZONE2_INITIAL_Y_MM    2250
#define PP_ZONE2_INITIAL_YAW_DEG    ((float)PP_START_YAW_DEG) /* 起始IMU对齐目标。 */
#define PP_ZONE2_EXIT_X_MM        300
#define PP_ZONE2_EXIT_Y_MM       2050
#define PP_ZONE2_EXIT_YAW_DEG       ((float)PP_START_YAW_DEG) /* 先保持摆车角离站。 */
#define PP_ZONE2_PARK_X_MM        150
#define PP_ZONE2_PARK_Y_MM       2250
#define PP_ZONE2_PARK_YAW_DEG       0.0f

/* 选区后规划、初始位姿、离站和回库共用这份配置。
 * route_node是A*接入的安全道路节点：1区为0(离站点)，2区为右下节点2。
 * final_parking_required=1：A*返回安全节点后，还需保持停车航向全向入库。
 */
typedef struct {
    uint8_t zone;
    int route_node;
    float initial_x_mm;
    float initial_y_mm;
    float initial_yaw_deg;
    float exit_x_mm;
    float exit_y_mm;
    float exit_yaw_deg;
    float park_x_mm;
    float park_y_mm;
    float park_yaw_deg;
    uint8_t final_parking_required;
} PathPlan_StartProfile_t;

/* 路线代价=路程+转向等效路程。90度转向默认增加400+100=500mm代价。
 * 调大更偏向少转弯，调小更偏向短距离；两项设0恢复只比较距离。
 * 小角度残差按路径航向容差忽略，避免把近似直线算成一次明显转弯。
 */
#define PP_TURN_90_COST_MM       400.0f
#define PP_TURN_START_COST_MM    100.0f
#define PP_TURN_IGNORE_DEG         5.0f /* 和路径执行的航向容差同步。 */

/* 一次规划的输出：8 段路径，每段一个数组 */
typedef struct {
    int edge_path[PP_MAX_STAGES][PP_MAX_PATH_LEN]; /* 每段路径的边序列（带符号方向） */
    int length[PP_MAX_STAGES];                     /* 每段路径长度（边的个数） */
    int ok[PP_MAX_STAGES];                         /* 1=找到路径，0=无路可走 */
    /* Watch统计包含出发转向和工位对接转向；规划距离沿用曼哈顿距离。 */
    int distance_mm[PP_MAX_STAGES];
    int turn_count[PP_MAX_STAGES];
    float turn_degrees[PP_MAX_STAGES];
    float total_cost[PP_MAX_STAGES];
    uint8_t start_zone; /* 本次规划所属启停区；执行器检查，防止路线与选区不一致。 */
} path_plan_result_t;

/* 默认1区。只能在开始规划/执行前选择，行驶期间不可重新选区。
 * 合法1/2返回1；无效值返回0并保留原选择。不改变节点/道路/障碍映射。
 */
int path_plan_select_start_zone(uint8_t zone);
uint8_t path_plan_get_start_zone(void);
const PathPlan_StartProfile_t *path_plan_get_start_profile(void);

/* 初始化图（path_plan_run 内部会自动调用，一般不用手动调） */
void path_plan_init(void);

/* 把雷达的障碍 grid_id(0~24) 映射为要封锁的边编号(1~12)。
   返回边个数 0/1/2，结果写到 edges_out。 */
int grid_to_edges(int grid_id, int edges_out[2]);

/* 输入：障碍 grid_id 数组 + 个数；输出：8 段路径结果（存 8 个数组） */
void path_plan_run(const int *grid_ids, int grid_count, path_plan_result_t *result);

/* 规划和执行共用工位朝向；stage从0开始。
 * 第7段：1区保持安全点到达航向入库；2区仍先在安全节点调整到停车朝向。
 */
int path_plan_stage_docking_yaw(int stage, float *yaw_deg);

/* 把带符号边编号转换成该行驶方向的起点和终点坐标，成功返回1。 */
int path_plan_edge_points(int signed_edge,
                          float *start_x_mm, float *start_y_mm,
                          float *target_x_mm, float *target_y_mm);

/* 只取得该行驶方向的终点绝对坐标，成功返回1。 */
int path_plan_edge_target(int signed_edge, float *x_mm, float *y_mm);

#ifdef __cplusplus
}
#endif

#endif /* PATH_PLAN_H */
