/* Host regression (Runtime UART is mocked; no motor commands are sent):
 * gcc -std=c99 -Wall -Wextra -Werror -Ibsp/inc
 * tests/start_zone_motion_test.c bsp/scr/path_follower.c
 * bsp/scr/path_plan.c -lm -o start_zone_motion_test.exe
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "path_follower.h"
#include "chassis_runtime.h"

#define TEST_PI 3.14159265358979323846f
#define TEST_COMMAND_CAPACITY 256U
#define TEST_START_EXIT_RPM 80.0f
#define TEST_PARK_RPM 60.0f
#define TEST_ZONE2_SAFE_X_MM 275.0f
#define TEST_ZONE2_SAFE_Y_MM 2060.0f

typedef enum {
    TEST_MOVE_STRICT,
    TEST_MOVE_TRANSIT,
    TEST_MOVE_LIMITED_STRICT,
    TEST_MOVE_LIMITED_TRANSIT,
    TEST_TURN_STRICT,
    TEST_TURN_TRANSIT
} TestCommandKind_t;

typedef struct {
    TestCommandKind_t kind;
    float x;
    float y;
    float yaw;
    float limit;
    float from_x;
    float from_y;
    uint32_t sequence;
} TestCommand_t;

ChassisRuntime_State_t g_chassis_runtime;
static TestCommand_t commands[TEST_COMMAND_CAPACITY];
static unsigned command_count;
static unsigned stop_count;
static uint32_t next_sequence;

static bool TestIssue(TestCommandKind_t kind, float x, float y, float yaw,
                      float limit, uint32_t *sequence)
{
    TestCommand_t *command;
    assert(command_count < TEST_COMMAND_CAPACITY);
    assert(sequence != NULL);
    command = &commands[command_count++];
    command->kind = kind;
    command->x = x;
    command->y = y;
    command->yaw = yaw;
    command->limit = limit;
    command->from_x = g_chassis_runtime.nav.pose.x_mm;
    command->from_y = g_chassis_runtime.nav.pose.y_mm;
    command->sequence = next_sequence++;
    *sequence = command->sequence;
    g_chassis_runtime.active_sequence = *sequence;
    g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING;
    return true;
}

bool ChassisRuntime_GotoXY(float x, float y, float yaw, uint32_t *sequence)
{
    return TestIssue(TEST_MOVE_STRICT, x, y, yaw, 0.0f, sequence);
}

bool ChassisRuntime_GotoXYTransit(float x, float y, float yaw,
                                 uint32_t *sequence)
{
    return TestIssue(TEST_MOVE_TRANSIT, x, y, yaw, 0.0f, sequence);
}

bool ChassisRuntime_GotoXYLimited(float x, float y, float yaw, float limit,
                                 uint32_t *sequence)
{
    return TestIssue(TEST_MOVE_LIMITED_STRICT, x, y, yaw, limit, sequence);
}

bool ChassisRuntime_GotoXYLimitedTransit(float x, float y, float yaw,
                                        float limit, uint32_t *sequence)
{
    return TestIssue(TEST_MOVE_LIMITED_TRANSIT, x, y, yaw, limit, sequence);
}

bool ChassisRuntime_TurnTo(float yaw, uint32_t *sequence)
{
    return TestIssue(TEST_TURN_STRICT, g_chassis_runtime.nav.pose.x_mm,
                     g_chassis_runtime.nav.pose.y_mm, yaw, 0.0f, sequence);
}

bool ChassisRuntime_TurnToTransit(float yaw, uint32_t *sequence)
{
    return TestIssue(TEST_TURN_TRANSIT, g_chassis_runtime.nav.pose.x_mm,
                     g_chassis_runtime.nav.pose.y_mm, yaw, 0.0f, sequence);
}

void ChassisRuntime_GetState(ChassisRuntime_State_t *state)
{
    *state = g_chassis_runtime;
}

bool ChassisRuntime_Stop(void)
{
    stop_count++;
    return true;
}

void ChassisRuntime_SetEncoderQueryEnabled(bool enabled)
{
    g_chassis_runtime.encoder_query_enabled = enabled ? 1U : 0U;
}

static void TestResetRuntime(void)
{
    memset(&g_chassis_runtime, 0, sizeof(g_chassis_runtime));
    memset(commands, 0, sizeof(commands));
    command_count = 0U;
    stop_count = 0U;
    next_sequence = 1U;
    g_chassis_runtime.status = CHASSIS_RUNTIME_IDLE;
    g_chassis_runtime.nav_initialized = 1U;
    g_chassis_runtime.encoder_query_enabled = 1U;
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
}

static unsigned TestTotalWaypoints(const path_plan_result_t *plan)
{
    unsigned stage;
    unsigned total = 0U;
    for (stage = 0U; stage < PP_MAX_STAGES; stage++) {
        assert(plan->ok[stage] != 0);
        assert(plan->length[stage] > 0);
        total += (unsigned)plan->length[stage];
    }
    return total;
}

static void TestStart(uint8_t zone, path_plan_result_t *plan)
{
    const PathPlan_StartProfile_t *profile;
    assert(path_plan_select_start_zone(zone));
    path_plan_run(NULL, 0, plan);
    assert(plan->start_zone == zone);
    TestResetRuntime();
    profile = path_plan_get_start_profile();
    g_chassis_runtime.nav.pose.x_mm = profile->initial_x_mm;
    g_chassis_runtime.nav.pose.y_mm = profile->initial_y_mm;
    g_chassis_runtime.nav.pose.yaw_deg = profile->initial_yaw_deg;
    assert(PathFollower_Start(plan));
    assert(g_path_follower.start_zone == zone);
    assert(g_path_follower.total_waypoints == TestTotalWaypoints(plan));
    assert(g_path_follower.completed_waypoints == 0U);
}

static const TestCommand_t *TestLastCommand(void)
{
    assert(command_count != 0U);
    return &commands[command_count - 1U];
}

static void TestCompleteLastCommand(void)
{
    const TestCommand_t *command = TestLastCommand();
    g_chassis_runtime.completed_sequence = command->sequence;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.nav.pose.x_mm = command->x;
    g_chassis_runtime.nav.pose.y_mm = command->y;
    g_chassis_runtime.nav.pose.yaw_deg = command->yaw;
    g_chassis_runtime.nav.velocity.vx_mm_s = 0.0f;
    g_chassis_runtime.nav.velocity.vy_mm_s = 0.0f;
    g_chassis_runtime.nav.velocity.wz_deg_s = 0.0f;
}

static void TestProfilesAndRoutes(void)
{
    path_plan_result_t plan;
    const PathPlan_StartProfile_t *profile;
    float sx, sy, tx, ty, yaw;
    uint8_t zone;

    for (zone = PP_START_ZONE_1; zone <= PP_START_ZONE_2; zone++) {
        assert(path_plan_select_start_zone(zone));
        assert(path_plan_get_start_zone() == zone);
        profile = path_plan_get_start_profile();
        assert(profile != NULL && profile->zone == zone);
        assert(profile->initial_x_mm == 150.0f);
        assert(profile->initial_y_mm == (zone == PP_START_ZONE_1 ? 150.0f : 2250.0f));
        assert(profile->initial_yaw_deg == (float)PP_START_YAW_DEG);
        assert(profile->exit_x_mm == 300.0f);
        assert(profile->exit_y_mm == (zone == PP_START_ZONE_1 ? 350.0f : 2050.0f));
        assert(profile->exit_yaw_deg == (float)PP_START_YAW_DEG);
        assert(profile->park_x_mm == profile->initial_x_mm);
        assert(profile->park_y_mm == profile->initial_y_mm);
        assert(profile->park_yaw_deg == 0.0f);
        assert(profile->route_node == (zone == PP_START_ZONE_1 ? 0 : 2));
        assert(profile->final_parking_required == (zone == PP_START_ZONE_1 ? 0U : 1U));
        assert(!path_plan_select_start_zone(0U));
        assert(!path_plan_select_start_zone(3U));
        assert(path_plan_get_start_zone() == zone);

        path_plan_run(NULL, 0, &plan);
        assert(plan.start_zone == zone);
        (void)TestTotalWaypoints(&plan);
        assert(path_plan_edge_points(plan.edge_path[0][0], &sx, &sy, &tx, &ty));
        assert(sx == (zone == PP_START_ZONE_1 ? 150.0f : TEST_ZONE2_SAFE_X_MM));
        assert(sy == (zone == PP_START_ZONE_1 ? 150.0f : TEST_ZONE2_SAFE_Y_MM));
        assert(path_plan_edge_target(plan.edge_path[7][plan.length[7] - 1], &tx, &ty));
        assert(tx == (zone == PP_START_ZONE_1 ? 150.0f : TEST_ZONE2_SAFE_X_MM));
        assert(ty == (zone == PP_START_ZONE_1 ? 150.0f : TEST_ZONE2_SAFE_Y_MM));
        assert(path_plan_stage_docking_yaw(7, &yaw) == (zone == PP_START_ZONE_1 ? 0 : 1));
        if (zone == PP_START_ZONE_2) assert(yaw == 0.0f);
    }
    puts("PASS: both start profiles, route endpoints, invalid selection preserved");
}

static void TestPlanSelectionMismatch(void)
{
    path_plan_result_t plan;
    uint8_t zone;
    for (zone = PP_START_ZONE_1; zone <= PP_START_ZONE_2; zone++) {
        assert(path_plan_select_start_zone(zone));
        path_plan_run(NULL, 0, &plan);
        assert(path_plan_select_start_zone(zone == PP_START_ZONE_1 ? PP_START_ZONE_2 : PP_START_ZONE_1));
        TestResetRuntime();
        assert(!PathFollower_Start(&plan));
        assert(g_path_follower.status == PATH_FOLLOWER_FAULT);
        assert(g_path_follower.fault == PATH_FOLLOWER_FAULT_BAD_PLAN);
        assert(command_count == 0U && stop_count == 1U);
    }
    puts("PASS: plan start_zone must match selected zone before execution");
}

static void TestStartup(uint8_t zone)
{
    path_plan_result_t plan;
    PathPlan_StartProfile_t profile;
    const TestCommand_t *command;
    uint32_t sequence;

    TestStart(zone, &plan);
    profile = *path_plan_get_start_profile();
    g_chassis_runtime.nav_initialized = 0U;
    PathFollower_Process();
    assert(command_count == 0U);
    g_chassis_runtime.nav_initialized = 1U;
    g_chassis_runtime.encoder_query_enabled = 0U;
    PathFollower_Process();
    assert(command_count == 0U);
    g_chassis_runtime.encoder_query_enabled = 1U;
    g_chassis_runtime.encoder_valid_mask = 0U;
    PathFollower_Process();
    assert(command_count == 0U);
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    PathFollower_Process();
    assert(command_count == 1U);
    command = TestLastCommand();
    assert(command->kind == TEST_MOVE_LIMITED_TRANSIT);
    assert(command->x == profile.exit_x_mm && command->y == profile.exit_y_mm);
    assert(command->yaw == profile.exit_yaw_deg && command->limit == TEST_START_EXIT_RPM);
    assert(command->from_x == profile.initial_x_mm && command->from_y == profile.initial_y_mm);
    assert(g_path_follower.status == PATH_FOLLOWER_START_EXIT_MOVING);
    assert(g_path_follower.stage_index == 0U && g_path_follower.edge_index == 0U);
    assert(g_path_follower.completed_waypoints == 0U);
    assert(!PathFollower_Continue());
    sequence = command->sequence;

    g_chassis_runtime.nav.pose.x_mm = profile.exit_x_mm;
    g_chassis_runtime.nav.pose.y_mm = profile.exit_y_mm;
    g_chassis_runtime.completed_sequence = sequence + 1000U;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    PathFollower_Process();
    assert(command_count == 1U && g_path_follower.startup_exit_done == 0U);
    assert(g_path_follower.status == PATH_FOLLOWER_START_EXIT_MOVING);
    g_chassis_runtime.completed_sequence = sequence;
    g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING;
    PathFollower_Process();
    assert(command_count == 1U && g_path_follower.startup_exit_done == 0U);
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.encoder_valid_mask = 0U;
    PathFollower_Process();
    assert(command_count == 1U && g_path_follower.startup_exit_done == 0U);

    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    PathFollower_Process();
    assert(g_path_follower.startup_exit_done == 1U);
    assert(g_path_follower.stage_index == 0U && g_path_follower.edge_index == 0U);
    assert(g_path_follower.completed_waypoints == 0U);
    assert(command_count == 2U);
    command = TestLastCommand();
    assert(command->kind == TEST_MOVE_TRANSIT || command->kind == TEST_TURN_TRANSIT);
    assert(command->from_x == profile.exit_x_mm && command->from_y == profile.exit_y_mm);
    printf("PASS: zone %u startup holds yaw %d, feedback/sequence gates, no waypoint increment\n",
           (unsigned)zone, PP_START_YAW_DEG);
}

static void TestStartupSafetyGuard(uint8_t zone)
{
    path_plan_result_t plan;
    unsigned invalid;
    for (invalid = 0U; invalid < 3U; invalid++) {
        TestStart(zone, &plan);
        PathFollower_Process();
        TestCompleteLastCommand();
        if (invalid == 0U) {
            g_chassis_runtime.nav.pose.x_mm += CHASSIS_NAV_TRANSIT_POSITION_MM + 0.5f;
        } else if (invalid == 1U) {
            g_chassis_runtime.nav.pose.x_mm = NAN;
        } else {
            g_chassis_runtime.nav.pose.y_mm = NAN;
        }
        PathFollower_Process();
        assert(g_path_follower.status == PATH_FOLLOWER_FAULT);
        assert(g_path_follower.fault == PATH_FOLLOWER_FAULT_CHASSIS);
        assert(g_path_follower.startup_exit_done == 0U);
        assert(command_count == 1U && stop_count == 1U);
        assert(g_path_follower.completed_waypoints == 0U);
    }
    TestStart(zone, &plan);
    PathFollower_Process();
    g_chassis_runtime.cycle_count = 400U;
    PathFollower_Process();
    assert(g_path_follower.status == PATH_FOLLOWER_FAULT);
    assert(command_count == 1U && stop_count == 1U);
    printf("PASS: zone %u startup safe-position, nonfinite feedback and timeout guards\n",
           (unsigned)zone);
}

static void TestProfileSnapshot(void)
{
    path_plan_result_t plan;
    TestStart(PP_START_ZONE_2, &plan);
    assert(path_plan_select_start_zone(PP_START_ZONE_1));
    PathFollower_Process();
    assert(g_path_follower.start_zone == PP_START_ZONE_2);
    assert(TestLastCommand()->x == PP_ZONE2_EXIT_X_MM);
    assert(TestLastCommand()->y == PP_ZONE2_EXIT_Y_MM);
    assert(TestLastCommand()->yaw == (float)PP_START_YAW_DEG);
    puts("PASS: startup uses the profile captured at PathFollower_Start");
}

static void TestPrepareFinalRoad(uint8_t zone, path_plan_result_t *plan)
{
    float sx, sy, tx, ty;
    int edge;
    TestStart(zone, plan);
    g_path_follower.stage_index = 7U;
    g_path_follower.edge_index = (uint8_t)(plan->length[7] - 1);
    g_path_follower.completed_stage = 6U;
    g_path_follower.completed_waypoints = (uint16_t)(g_path_follower.total_waypoints - 1U);
    g_path_follower.startup_exit_done = 1U;
    edge = plan->edge_path[7][g_path_follower.edge_index];
    assert(path_plan_edge_points(edge, &sx, &sy, &tx, &ty));
    g_chassis_runtime.nav.pose.x_mm = sx;
    g_chassis_runtime.nav.pose.y_mm = sy;
    g_chassis_runtime.nav.pose.yaw_deg = atan2f(-(tx - sx), ty - sy) * 180.0f / TEST_PI;
    PathFollower_Process();
    assert(command_count == 1U && TestLastCommand()->kind == TEST_MOVE_STRICT);
    assert(g_path_follower.status == PATH_FOLLOWER_MOVING_FORWARD);
    assert(TestLastCommand()->x == tx && TestLastCommand()->y == ty);
}

static void TestFinalSafePositionGuard(void)
{
    path_plan_result_t plan;
    unsigned invalid;
    for (invalid = 0U; invalid < 7U; invalid++) {
        TestPrepareFinalRoad(PP_START_ZONE_2, &plan);
        TestCompleteLastCommand();
        switch (invalid) {
        case 0U: g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING; break;
        case 1U: g_chassis_runtime.nav_initialized = 0U; break;
        case 2U: g_chassis_runtime.encoder_query_enabled = 0U; break;
        case 3U: g_chassis_runtime.encoder_valid_mask = 0U; break;
        case 4U:
            g_chassis_runtime.nav.pose.x_mm += CHASSIS_NAV_TRANSIT_POSITION_MM + 0.5f;
            break;
        case 5U: g_chassis_runtime.nav.pose.x_mm = NAN; break;
        default: g_chassis_runtime.nav.pose.y_mm = NAN; break;
        }
        PathFollower_Process();
        assert(g_path_follower.status == PATH_FOLLOWER_FAULT);
        assert(g_path_follower.fault == PATH_FOLLOWER_FAULT_CHASSIS);
        assert(command_count == 1U && stop_count == 1U);
        assert(g_path_follower.completed_waypoints == g_path_follower.total_waypoints);
    }
    puts("PASS: zone 2 refuses parking turn before DONE/valid feedback/safe finite position");
}

static void TestParkingFeedbackGate(PathFollower_Status_t status,
                                    unsigned expected_commands)
{
    TestCompleteLastCommand();
    g_chassis_runtime.status = CHASSIS_RUNTIME_RUNNING;
    PathFollower_Process();
    assert(g_path_follower.status == status && command_count == expected_commands);
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.nav_initialized = 0U;
    PathFollower_Process();
    assert(g_path_follower.status == status && command_count == expected_commands);
    g_chassis_runtime.nav_initialized = 1U;
    g_chassis_runtime.encoder_query_enabled = 0U;
    PathFollower_Process();
    assert(g_path_follower.status == status && command_count == expected_commands);
    g_chassis_runtime.encoder_query_enabled = 1U;
    g_chassis_runtime.encoder_valid_mask = 0U;
    PathFollower_Process();
    assert(g_path_follower.status == status && command_count == expected_commands);
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    assert(g_path_follower.completed_waypoints == g_path_follower.total_waypoints);
}

static void TestFinalParkingZone2(void)
{
    path_plan_result_t plan;
    const TestCommand_t *command;
    uint16_t total;
    uint32_t sequence;

    TestPrepareFinalRoad(PP_START_ZONE_2, &plan);
    total = g_path_follower.total_waypoints;
    assert(TestLastCommand()->x == TEST_ZONE2_SAFE_X_MM);
    assert(TestLastCommand()->y == TEST_ZONE2_SAFE_Y_MM);
    sequence = TestLastCommand()->sequence;
    g_chassis_runtime.completed_sequence = sequence + 1000U;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    PathFollower_Process();
    assert(g_path_follower.completed_waypoints == total - 1U);
    assert(command_count == 1U);

    /* Selection changes cannot replace the profile already owned by this run. */
    assert(path_plan_select_start_zone(PP_START_ZONE_1));
    TestCompleteLastCommand();
    PathFollower_Process();
    assert(command_count == 2U);
    assert(g_path_follower.status == PATH_FOLLOWER_PARKING_TURN);
    assert(g_path_follower.stage_index == 7U);
    assert(g_path_follower.completed_waypoints == total);
    command = TestLastCommand();
    assert(command->kind == TEST_TURN_STRICT && command->yaw == 0.0f);
    assert(command->from_x == TEST_ZONE2_SAFE_X_MM);
    assert(command->from_y == TEST_ZONE2_SAFE_Y_MM);
    assert(!PathFollower_Continue());

    sequence = command->sequence;
    g_chassis_runtime.completed_sequence = sequence + 1000U;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    PathFollower_Process();
    assert(command_count == 2U && g_path_follower.status == PATH_FOLLOWER_PARKING_TURN);
    assert(g_path_follower.completed_waypoints == total);
    TestParkingFeedbackGate(PATH_FOLLOWER_PARKING_TURN, 2U);
    PathFollower_Process();
    assert(command_count == 3U);
    assert(g_path_follower.status == PATH_FOLLOWER_PARKING_MOVING);
    command = TestLastCommand();
    assert(command->kind == TEST_MOVE_LIMITED_STRICT);
    assert(command->x == 150.0f && command->y == 2250.0f);
    assert(command->yaw == 0.0f && command->limit == TEST_PARK_RPM);
    assert(command->from_x == TEST_ZONE2_SAFE_X_MM && command->from_y == TEST_ZONE2_SAFE_Y_MM);
    assert(g_chassis_runtime.nav.pose.yaw_deg == 0.0f);
    assert(g_path_follower.completed_waypoints == total);
    assert(!PathFollower_Continue());

    sequence = command->sequence;
    g_chassis_runtime.completed_sequence = sequence + 1000U;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    PathFollower_Process();
    assert(command_count == 3U && g_path_follower.status == PATH_FOLLOWER_PARKING_MOVING);
    assert(g_path_follower.completed_waypoints == total);
    TestParkingFeedbackGate(PATH_FOLLOWER_PARKING_MOVING, 3U);
    PathFollower_Process();
    assert(g_path_follower.status == PATH_FOLLOWER_WAIT_ACTION);
    assert(g_path_follower.completed_stage == 7U && g_path_follower.stage_index == 7U);
    assert(g_path_follower.completed_waypoints == total);
    assert(PathFollower_Continue());
    assert(g_path_follower.status == PATH_FOLLOWER_DONE);
    PathFollower_Process();
    assert(command_count == 3U && stop_count == 0U);
    assert(g_path_follower.completed_waypoints == total);
    puts("PASS: zone 2 safe-node strict turn, strict yaw-0 parking at 60 rpm, sequence gates, DONE");
}

static void TestFinalParkingZone1Unchanged(void)
{
    path_plan_result_t plan;
    TestPrepareFinalRoad(PP_START_ZONE_1, &plan);
    assert(TestLastCommand()->x == 150.0f && TestLastCommand()->y == 150.0f);
    TestCompleteLastCommand();
    PathFollower_Process();
    assert(g_path_follower.status == PATH_FOLLOWER_WAIT_ACTION);
    assert(g_path_follower.completed_stage == 7U);
    assert(g_path_follower.completed_waypoints == g_path_follower.total_waypoints);
    assert(command_count == 1U);
    assert(PathFollower_Continue());
    assert(g_path_follower.status == PATH_FOLLOWER_DONE);
    assert(command_count == 1U && stop_count == 0U);
    puts("PASS: zone 1 final road keeps its original direct parking behavior");
}

static void TestRawDeparture(uint8_t zone, uint8_t stage)
{
    path_plan_result_t plan;
    unsigned commands_before;
    uint32_t departure_sequence;
    const TestCommand_t *command;
    float sx, sy, tx, ty, expected_yaw;
    TestStart(zone, &plan);
    g_path_follower.stage_index = stage;
    g_path_follower.completed_stage = stage;
    g_path_follower.status = PATH_FOLLOWER_WAIT_ACTION;
    g_chassis_runtime.status = CHASSIS_RUNTIME_DONE;
    g_chassis_runtime.nav.pose.x_mm = 1280.0f;
    g_chassis_runtime.nav.pose.y_mm = 180.0f;
    g_chassis_runtime.nav.pose.yaw_deg = -90.0f;
    g_chassis_runtime.encoder_query_enabled = 0U;
    g_chassis_runtime.encoder_valid_mask = 0U;
    g_chassis_runtime.cycle_count = 100U;
    assert(PathFollower_Continue());
    assert(g_chassis_runtime.encoder_query_enabled == 1U);
    assert(g_path_follower.status == PATH_FOLLOWER_EXIT_WAIT_FEEDBACK);
    assert(g_path_follower.stage_index == stage && command_count == 0U);
    PathFollower_Process();
    assert(command_count == 0U);
    g_chassis_runtime.cycle_count = 102U;
    PathFollower_Process();
    assert(command_count == 0U); /* Still no valid four-wheel feedback. */
    g_chassis_runtime.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    PathFollower_Process();
    assert(command_count == 1U && g_path_follower.status == PATH_FOLLOWER_EXIT_MOVING);
    command = TestLastCommand();
    assert(command->kind == TEST_MOVE_LIMITED_STRICT);
    assert(command->x == 1200.0f && command->y == 300.0f);
    assert(command->yaw == -90.0f && command->limit == 60.0f);
    departure_sequence = command->sequence;
    g_chassis_runtime.completed_sequence = departure_sequence + 1000U;
    PathFollower_Process();
    assert(command_count == 1U && g_path_follower.stage_index == stage);
    TestCompleteLastCommand();
    PathFollower_Process();
    assert(g_path_follower.stage_index == stage + 1U && command_count == 2U);
    assert(g_path_follower.completed_waypoints == 0U);
    assert(path_plan_edge_points(plan.edge_path[stage + 1U][0], &sx, &sy, &tx, &ty));
    expected_yaw = atan2f(-(tx - 1200.0f), ty - 300.0f) * 180.0f / TEST_PI;
    command = TestLastCommand();
    assert(command->kind == TEST_TURN_TRANSIT);
    assert(fabsf(command->yaw - expected_yaw) < .001f);
    assert(command->from_x == 1200.0f && command->from_y == 300.0f);
    commands_before = command_count;
    PathFollower_Process();
    assert(command_count == commands_before); /* No resubmission while turning. */
    printf("PASS: zone %u raw stage %u holds -90 to (1200,300) at 60 rpm before next turn\n", zone, stage);
}

static void TestCompleteRoute(uint8_t zone)
{
    path_plan_result_t plan;
    unsigned cycle, index;
    unsigned road_moves = 0U;
    unsigned startup_moves = 0U;
    unsigned strict_limited_moves = 0U;
    TestStart(zone, &plan);
    for (cycle = 0U; cycle < 1000U && g_path_follower.status != PATH_FOLLOWER_DONE; cycle++) {
        g_chassis_runtime.cycle_count++;
        assert(g_path_follower.status != PATH_FOLLOWER_FAULT);
        if (g_path_follower.status == PATH_FOLLOWER_WAIT_ACTION) {
            assert(PathFollower_Continue());
        } else if (g_chassis_runtime.status == CHASSIS_RUNTIME_RUNNING) {
            TestCompleteLastCommand();
        }
        PathFollower_Process();
        assert(g_path_follower.completed_waypoints <= g_path_follower.total_waypoints);
    }
    assert(cycle < 1000U && g_path_follower.status == PATH_FOLLOWER_DONE);
    assert(g_path_follower.completed_waypoints == TestTotalWaypoints(&plan));
    assert(g_path_follower.completed_waypoints == g_path_follower.total_waypoints);
    assert(stop_count == 0U);
    for (index = 0U; index < command_count; index++) {
        if (commands[index].kind == TEST_MOVE_STRICT || commands[index].kind == TEST_MOVE_TRANSIT)
            road_moves++;
        else if (commands[index].kind == TEST_MOVE_LIMITED_TRANSIT)
            startup_moves++;
        else if (commands[index].kind == TEST_MOVE_LIMITED_STRICT)
            strict_limited_moves++;
    }
    assert(road_moves == TestTotalWaypoints(&plan));
    assert(startup_moves == 1U);
    assert(strict_limited_moves == (zone == PP_START_ZONE_1 ? 6U : 7U));
    assert(g_chassis_runtime.nav.pose.x_mm == 150.0f);
    assert(g_chassis_runtime.nav.pose.y_mm == (zone == PP_START_ZONE_1 ? 150.0f : 2250.0f));
    if (zone == PP_START_ZONE_2) assert(g_chassis_runtime.nav.pose.yaw_deg == 0.0f);
    printf("PASS: zone %u full 8-stage route counts only %u road waypoints\n",
           (unsigned)zone, road_moves);
}

static float TestRectMaxY(float center_y, float yaw_deg)
{
    float angle = yaw_deg * TEST_PI / 180.0f;
    return center_y + PP_ROBOT_SIZE_X_MM * 0.5f * fabsf(sinf(angle)) +
                      PP_ROBOT_SIZE_Y_MM * 0.5f * fabsf(cosf(angle));
}

static void TestRectangleInsideField(float x, float y, float yaw_deg)
{
    float angle = yaw_deg * TEST_PI / 180.0f;
    float half_x = PP_ROBOT_SIZE_X_MM * 0.5f;
    float half_y = PP_ROBOT_SIZE_Y_MM * 0.5f;
    int sign_x, sign_y;
    for (sign_x = -1; sign_x <= 1; sign_x += 2) {
        for (sign_y = -1; sign_y <= 1; sign_y += 2) {
            float corner_x = x + sign_x * half_x * cosf(angle) - sign_y * half_y * sinf(angle);
            float corner_y = y + sign_x * half_x * sinf(angle) + sign_y * half_y * cosf(angle);
            /* Numerical tolerance only, NOT a physical safety margin. */
            assert(corner_x >= -0.001f && corner_x <= PP_FIELD_SIZE_MM + 0.001f);
            assert(corner_y >= -0.001f && corner_y <= PP_FIELD_SIZE_MM + 0.001f);
        }
    }
}

static void TestRotationEnvelope(float x, float y)
{
    float radius = hypotf(PP_ROBOT_SIZE_X_MM * 0.5f, PP_ROBOT_SIZE_Y_MM * 0.5f);
    float protected_radius = radius + CHASSIS_NAV_TRANSIT_POSITION_MM;
    /* The nearest corner of the yellow forbidden region is (550,1850). */
    assert(x < 550.0f && y > 1850.0f);
    assert(x - protected_radius > 0.0f && x + protected_radius < PP_FIELD_SIZE_MM);
    assert(y - protected_radius > 0.0f && y + protected_radius < PP_FIELD_SIZE_MM);
    assert(hypotf(550.0f - x, y - 1850.0f) > protected_radius);
    assert(radius > 203.48f && radius < 203.50f);
}

static void TestZone2SweptGeometry(void)
{
    unsigned sample;
    float x, y, t;
    float half_x = PP_ROBOT_SIZE_X_MM * 0.5f;
    float start_angle = (float)PP_START_YAW_DEG * TEST_PI / 180.0f;
    float start_half_x = half_x * fabsf(cosf(start_angle)) +
                        PP_ROBOT_SIZE_Y_MM * 0.5f * fabsf(sinf(start_angle));
    float yaw_error_max_y;
    float position_error_max_y;
    assert(PP_FIELD_SIZE_MM == 2400);
    assert(PP_ROBOT_SIZE_X_MM == 275 && PP_ROBOT_SIZE_Y_MM == 300);

    for (sample = 0U; sample <= 1000U; sample++) {
        t = (float)sample / 1000.0f;
        x = 150.0f + (300.0f - 150.0f) * t;
        y = 2250.0f + (2050.0f - 2250.0f) * t;
        TestRectangleInsideField(x, y, (float)PP_START_YAW_DEG);
        assert(x + start_half_x < 550.0f);
    }
    TestRotationEnvelope(300.0f, 2050.0f);
    TestRotationEnvelope(TEST_ZONE2_SAFE_X_MM, TEST_ZONE2_SAFE_Y_MM);
    for (sample = 0U; sample <= 1000U; sample++) {
        t = (float)sample / 1000.0f;
        x = TEST_ZONE2_SAFE_X_MM + (150.0f - TEST_ZONE2_SAFE_X_MM) * t;
        y = TEST_ZONE2_SAFE_Y_MM + (2250.0f - TEST_ZONE2_SAFE_Y_MM) * t;
        TestRectangleInsideField(x, y, 0.0f);
        assert(x + half_x < 550.0f);
    }
    /* Exact center and exact yaw have ZERO lower-bound clearance. Do not
     * expand this parking rectangle by the 20 mm transit tolerance.
     */
    assert(TestRectMaxY(2250.0f, 0.0f) == PP_FIELD_SIZE_MM);
    yaw_error_max_y = TestRectMaxY(2250.0f, 1.0f);
    position_error_max_y = TestRectMaxY(2252.0f, 0.0f);
    assert(yaw_error_max_y > PP_FIELD_SIZE_MM);
    assert(position_error_max_y > PP_FIELD_SIZE_MM);
    printf("PASS: zone 2 yaw-0 swept rectangles and safe rotation envelopes; "
           "center parking clearance=0 mm, 1 deg yaw exceeds by %.3f mm, "
           "2 mm downward error exceeds by %.3f mm\n",
           yaw_error_max_y - PP_FIELD_SIZE_MM, position_error_max_y - PP_FIELD_SIZE_MM);
}

int main(void)
{
    TestProfilesAndRoutes();
    TestPlanSelectionMismatch();
    TestStartup(PP_START_ZONE_1);
    TestStartup(PP_START_ZONE_2);
    TestStartupSafetyGuard(PP_START_ZONE_1);
    TestStartupSafetyGuard(PP_START_ZONE_2);
    TestProfileSnapshot();
    TestFinalSafePositionGuard();
    TestFinalParkingZone2();
    TestFinalParkingZone1Unchanged();
    TestRawDeparture(PP_START_ZONE_1, 1U);
    TestRawDeparture(PP_START_ZONE_1, 4U);
    TestRawDeparture(PP_START_ZONE_2, 1U);
    TestRawDeparture(PP_START_ZONE_2, 4U);
    TestCompleteRoute(PP_START_ZONE_1);
    TestCompleteRoute(PP_START_ZONE_2);
    TestZone2SweptGeometry();
    puts("PASS: start-zone motion regression");
    return 0;
}
