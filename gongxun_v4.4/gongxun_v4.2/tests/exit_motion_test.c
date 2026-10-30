/* Host regression: gcc -Ibsp/inc tests/exit_motion_test.c
 * bsp/scr/path_follower.c bsp/scr/path_plan.c bsp/scr/chassis_nav.c -lm
 * Runtime UART calls are mocked; this test never sends motor commands.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "path_follower.h"
#include "chassis_runtime.h"

ChassisRuntime_State_t g_chassis_runtime;
static unsigned moves, turns, stops;
static unsigned forwards, transit_forwards;
static float issued_x, issued_y, issued_yaw, issued_limit;

bool ChassisRuntime_GotoXYLimited(float x, float y, float yaw,
                                  float limit, uint32_t *sequence)
{
    moves++;
    issued_x = x; issued_y = y; issued_yaw = yaw; issued_limit = limit;
    *sequence = 123U;
    return true;
}
bool ChassisRuntime_GotoXYLimitedTransit(float x, float y, float yaw,
                                         float limit, uint32_t *sequence)
{
    return ChassisRuntime_GotoXYLimited(x, y, yaw, limit, sequence);
}
bool ChassisRuntime_GotoXY(float x, float y, float yaw, uint32_t *sequence)
{
    (void)x; (void)y; (void)yaw; forwards++; *sequence = 124U; return true;
}
bool ChassisRuntime_TurnTo(float yaw, uint32_t *sequence)
{
    (void)yaw; turns++; *sequence = 125U; return true;
}
bool ChassisRuntime_TurnToTransit(float yaw, uint32_t *sequence)
{
    return ChassisRuntime_TurnTo(yaw, sequence);
}
bool ChassisRuntime_GotoXYTransit(float x, float y, float yaw, uint32_t *sequence)
{
    transit_forwards++;
    return ChassisRuntime_GotoXY(x, y, yaw, sequence);
}
void ChassisRuntime_GetState(ChassisRuntime_State_t *state)
{
    *state = g_chassis_runtime;
}
bool ChassisRuntime_Stop(void) { stops++; return true; }
void ChassisRuntime_SetEncoderQueryEnabled(bool enabled)
{
    g_chassis_runtime.encoder_query_enabled = enabled;
}

static void TestExit(unsigned stage, float x, float y, float yaw)
{
    path_plan_result_t plan;
    path_plan_run(NULL, 0, &plan);
    memset(&g_chassis_runtime, 0, sizeof(g_chassis_runtime));
    moves = turns = stops = 0U;
    assert(PathFollower_Start(&plan));
    g_path_follower.stage_index = (uint8_t)stage;
    g_path_follower.completed_stage = (uint8_t)stage;
    g_path_follower.status = PATH_FOLLOWER_WAIT_ACTION;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.nav_initialized = 1U;
    assert(PathFollower_Continue());
    assert(g_chassis_runtime.encoder_query_enabled == 1U);
    assert(g_path_follower.stage_index == stage);
    PathFollower_Process();
    assert(moves == 0U && turns == 0U);
    g_chassis_runtime.cycle_count = 2U;
    PathFollower_Process(); /* Invalid feedback must not start departure. */
    assert(moves == 0U);
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    PathFollower_Process();
    assert(moves == 1U && turns == 0U);
    assert(issued_x == x && issued_y == y && issued_yaw == yaw);
    assert(issued_limit == 60.0f);
    assert(g_path_follower.status == PATH_FOLLOWER_EXIT_MOVING);
    PathFollower_Process();
    assert(g_path_follower.stage_index == stage);
    g_chassis_runtime.completed_sequence = 123U;
    PathFollower_Process();
    assert(g_path_follower.stage_index == stage + 1U);
    assert(g_path_follower.completed_waypoints == 0U);
    PathFollower_Process();
    assert(turns == 1U && moves == 1U);
}

static void TestSpeedLimit(void)
{
    ChassisNav_Config_t config;
    ChassisNav_Feedback_t feedback = {0};
    ChassisNav_Output_t output;
    unsigned cycle, wheel;
    ChassisNav_GetDefaultConfig(&config);
    feedback.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    assert(ChassisNav_Init(&config, &feedback));
    assert(!ChassisNav_GotoXYLimited(1000, 1000, 0, 0));
    assert(ChassisNav_GotoXYLimited(1000, 1000, 0, 30));
    for (cycle = 0; cycle < 100U; cycle++) {
        ChassisNav_Update(&feedback, 0.02f, &output);
        for (wheel = 0; wheel < 4U; wheel++) {
            assert(fabsf(output.wheel_rpm[wheel]) <= 30.001f);
        }
    }
    assert(ChassisNav_GotoXY(1000, 1000, 0));
    for (cycle = 0; cycle < 100U; cycle++)
        ChassisNav_Update(&feedback, 0.02f, &output);
    assert(fabsf(output.wheel_rpm[0]) > 30.0f ||
           fabsf(output.wheel_rpm[1]) > 30.0f);
    assert(fabsf(output.wheel_rpm[0]) <= config.max_wheel_rpm + 0.001f);
}

static void InitHandoff(path_plan_result_t *plan)
{
    path_plan_run(NULL, 0, plan);
    assert(PathFollower_Start(plan));
    memset(&g_chassis_runtime, 0, sizeof(g_chassis_runtime));
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.nav_initialized = 1U;
    g_chassis_runtime.encoder_query_enabled = 1U;
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    g_path_follower.stage_index = 2U; /* 原料到加工的两条直线边，航向0。 */
    forwards = transit_forwards = turns = stops = 0U;
}

static void TestHandoff(void)
{
    path_plan_result_t plan;
    float sx, sy, tx, ty;
    InitHandoff(&plan);
    g_chassis_runtime.nav.pose.yaw_deg = CHASSIS_NAV_TRANSIT_HEADING_DEG - .1f;
    PathFollower_Process();
    assert(turns == 0 && forwards == 1 && transit_forwards == 1);
    assert(g_path_follower.turn_skipped == 1 && g_path_follower.skipped_turns == 1);
    assert(g_path_follower.status == PATH_FOLLOWER_MOVING_FORWARD);

    InitHandoff(&plan);
    g_chassis_runtime.nav.pose.yaw_deg = CHASSIS_NAV_TRANSIT_HEADING_DEG + .1f;
    PathFollower_Process();
    assert(turns == 1 && forwards == 0);
    g_chassis_runtime.completed_sequence = 999U;
    PathFollower_Process();
    assert(forwards == 0); /* Wrong sequence cannot release forward. */
    g_chassis_runtime.completed_sequence = g_path_follower.active_sequence;
    PathFollower_Process();
    assert(forwards == 1 && g_path_follower.status == PATH_FOLLOWER_MOVING_FORWARD);

    InitHandoff(&plan);
    g_chassis_runtime.encoder_valid_mask = 0;
    PathFollower_Process();
    assert(turns == 1 && forwards == 0);
    InitHandoff(&plan);
    g_chassis_runtime.encoder_query_enabled = 0;
    PathFollower_Process();
    assert(turns == 1 && forwards == 0);
    InitHandoff(&plan);
    g_chassis_runtime.nav.velocity.wz_deg_s = CHASSIS_NAV_TRANSIT_YAW_RATE_DEG_S + 1;
    PathFollower_Process();
    assert(turns == 1 && forwards == 0);
    InitHandoff(&plan);
    g_chassis_runtime.nav.velocity.vx_mm_s = CHASSIS_NAV_TRANSIT_SPEED_MM_S + 1;
    PathFollower_Process();
    assert(turns == 1 && forwards == 0);

    InitHandoff(&plan);
    g_path_follower.status = PATH_FOLLOWER_MOVING_FORWARD;
    g_path_follower.active_sequence = g_chassis_runtime.completed_sequence = 124U;
    PathFollower_Process(); /* Same call prepares next straight edge. */
    assert(g_path_follower.edge_index == 1 && forwards == 1 && turns == 0);
    assert(g_path_follower.status == PATH_FOLLOWER_MOVING_FORWARD);
    PathFollower_Process(); /* Final waypoint must still issue strict docking turn. */
    assert(turns == 1 && g_path_follower.status == PATH_FOLLOWER_DOCKING_TURN);

    InitHandoff(&plan);
    g_path_follower.stage_index = PP_MAX_STAGES - 1;
    g_path_follower.edge_index = (uint8_t)(plan.length[PP_MAX_STAGES - 1] - 1);
    assert(path_plan_edge_points(plan.edge_path[PP_MAX_STAGES - 1][g_path_follower.edge_index],
                                 &sx, &sy, &tx, &ty));
    g_chassis_runtime.nav.pose.yaw_deg = atan2f(-(tx - sx), ty - sy) *
                                        180.0f / 3.14159265358979323846f;
    PathFollower_Process();
    assert(forwards == 1 && transit_forwards == 0 && turns == 0);
    puts("PASS: aligned skip, same-call handoff, feedback/motion guards, strict dock/final park");
}

static void InitStartup(path_plan_result_t *plan)
{
    path_plan_run(NULL, 0, plan);
    assert(PathFollower_Start(plan));
    memset(&g_chassis_runtime, 0, sizeof(g_chassis_runtime));
    g_chassis_runtime.status = CHASSIS_RUNTIME_IDLE;
    g_chassis_runtime.nav_initialized = 1U;
    g_chassis_runtime.encoder_query_enabled = 1U;
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    g_chassis_runtime.nav.pose.x_mm = 150;
    g_chassis_runtime.nav.pose.y_mm = 150;
    moves = turns = stops = forwards = transit_forwards = 0;
}

static void TestStartup(void)
{
    path_plan_result_t plan;
    InitStartup(&plan);
    g_chassis_runtime.encoder_valid_mask = 0;
    PathFollower_Process();
    assert(moves == 0 && turns == 0 && forwards == 0);
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    PathFollower_Process();
    assert(moves == 1 && turns == 0 && forwards == 0);
    assert(issued_x == PP_START_EXIT_X_MM && issued_y == PP_START_EXIT_Y_MM);
    assert(issued_yaw == 0 && issued_limit == 80);
    assert(g_path_follower.status == PATH_FOLLOWER_START_EXIT_MOVING);
    g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING;
    g_chassis_runtime.completed_sequence = 999;
    PathFollower_Process();
    assert(turns == 0 && forwards == 0 && g_path_follower.startup_exit_done == 0);
    g_chassis_runtime.completed_sequence = g_path_follower.active_sequence;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.nav.pose.x_mm = PP_START_EXIT_X_MM;
    g_chassis_runtime.nav.pose.y_mm = PP_START_EXIT_Y_MM;
    PathFollower_Process();
    assert(g_path_follower.startup_exit_done == 1 && g_path_follower.stage_index == 0);
    assert(g_path_follower.completed_waypoints == 0 && forwards == 1 && turns == 0);
    /* First leg now points almost straight down from300/350, not from150/150. */
    assert(fabsf(g_path_follower.target_yaw_deg) < CHASSIS_NAV_TRANSIT_HEADING_DEG);

    InitStartup(&plan);
    PathFollower_Process();
    g_chassis_runtime.completed_sequence = g_path_follower.active_sequence;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    PathFollower_Process(); /* Wrong position despite matching sequence must stop. */
    assert(g_path_follower.status == PATH_FOLLOWER_FAULT && turns == 0 && stops == 1);

    InitStartup(&plan);
    PathFollower_Process();
    g_chassis_runtime.cycle_count = 400;
    PathFollower_Process();
    assert(g_path_follower.status == PATH_FOLLOWER_FAULT && stops == 1);

    InitStartup(&plan);
    { /* Edge1 blocked: leave first, then turn onto the upper route. */
        int obstacle = 3;
        path_plan_run(&obstacle, 1, &plan);
        assert(PathFollower_Start(&plan));
    }
    PathFollower_Process();
    assert(moves == 1 && turns == 0);
    g_chassis_runtime.completed_sequence = g_path_follower.active_sequence;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.nav.pose.x_mm = PP_START_EXIT_X_MM;
    g_chassis_runtime.nav.pose.y_mm = PP_START_EXIT_Y_MM;
    PathFollower_Process();
    assert(turns == 1 && forwards == 0 && g_path_follower.startup_exit_done == 1);
    assert(g_path_follower.target_yaw_deg < -90);
    puts("PASS: startup exit before turn, fresh feedback, safe-position/timeout guards, upper detour");
}

/* Nominal map clearance, including20mm endpoint uncertainty; physical slip is not modeled. */
static void TestStartupClearance(void)
{
    float half_x = PP_ROBOT_SIZE_X_MM * .5f;
    float half_y = PP_ROBOT_SIZE_Y_MM * .5f;
    float radius = hypotf(half_x, half_y);
    float tolerance = CHASSIS_NAV_TRANSIT_POSITION_MM;
    float top_leg_yaw = atan2f(-(1200.0f - PP_START_EXIT_X_MM),
                              300.0f - PP_START_EXIT_Y_MM);
    float top_leg_half_y = half_x * fabsf(sinf(top_leg_yaw)) +
                           half_y * fabsf(cosf(top_leg_yaw));
    float x, y;
    unsigned i;
    assert(PP_START_EXIT_X_MM - tolerance > radius);
    assert(PP_START_EXIT_Y_MM - tolerance > radius);
    assert(hypotf(550 - PP_START_EXIT_X_MM, 550 - PP_START_EXIT_Y_MM) > radius + tolerance);
    /* 起步不转向，沿初始275x300矩形车体扫过的区域均在场内。 */
    for (i = 0; i <= 100; i++) {
        x = 150 + (PP_START_EXIT_X_MM - 150) * i / 100.0f;
        y = 150 + (PP_START_EXIT_Y_MM - 150) * i / 100.0f;
        assert(x - half_x >= 0 && y - half_y >= 0);
        assert(x + half_x < 550);
    }
    /* 右侧后续通道允许旋转包络；上侧绕行的实际方形包络在黄色区上方。 */
    assert(300 + tolerance + radius < 550);
    assert(275 - tolerance - radius > 0);
    assert(PP_START_EXIT_Y_MM + tolerance + top_leg_half_y < 550);
    assert(path_plan_edge_target(-1, &x, &y));
    assert(x == 150 && y == 150); /* Final parking center remains unchanged. */
    puts("PASS: startup translation/rotation/first-leg map clearance, final parking150/150");
}

int main(void)
{
    TestExit(2U, 1200, 2000, 90);
    TestExit(5U, 1200, 2000, 90);
    TestExit(3U, 2050, 1200, 0);
    TestExit(6U, 2050, 1200, 0);
    TestExit(2U, 1200, 2000, 90);
    g_path_follower.status = PATH_FOLLOWER_WAIT_ACTION;
    g_path_follower.completed_stage = 3U;
    assert(PathFollower_Continue());
    g_chassis_runtime.encoder_valid_mask = 0U;
    g_chassis_runtime.cycle_count += 50U;
    PathFollower_Process();
    assert(g_path_follower.status == PATH_FOLLOWER_FAULT && stops == 1U);
    TestSpeedLimit();
    TestHandoff();
    TestStartup();
    TestStartupClearance();
    puts("PASS: both laps departure ordering, feedback timeout, speed limit/reset");
    return 0;
}
