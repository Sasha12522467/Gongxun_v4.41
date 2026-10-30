/* Host mocks; actual navigation, visual controller and imu_pid are compiled below. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "chassis_runtime.h"

typedef struct {
    float yaw_deg;
    uint32_t frame_count, last_update_ms;
} HWT101_Snapshot_t;
volatile float yaw;
float kp = -2.3f, kd = -.1f;
volatile uint16_t YuanData[2];
volatile uint32_t maixcam_frame_count, maixcam_last_update_ms;
int min = 1, max = 11, x_cha, y_cha;
static ChassisRuntime_State_t chassis;
static uint32_t now_ms;
static unsigned moves, stops, scenario;
static bool query_enabled;
static float expected_raw_target;
static int expected_limit;

static float Wrap(float value)
{
    while (value > 180) value -= 360;
    while (value <= -180) value += 360;
    return value;
}

uint32_t HAL_GetTick(void) { return now_ms; }
bool ChassisRuntime_IsEncoderQueryEnabled(void) { return query_enabled; }
void ChassisRuntime_SetEncoderQueryEnabled(bool value) { query_enabled = value; }
void ChassisRuntime_GetState(ChassisRuntime_State_t *state) { *state = chassis; }
void Emm_V5_Stop_Now(uint8_t address, bool sync)
{
    assert(address == 0 && !sync);
    stops++;
}
void HWT101_GetSnapshot(HWT101_Snapshot_t *imu)
{
    imu->yaw_deg = yaw;
    imu->frame_count = scenario == 4 ? 0 : 1;
    imu->last_update_ms = now_ms - (scenario == 1 ? 201 : 0);
}
void osDelay(uint32_t ms)
{
    now_ms += ms;
    if (scenario == 3) return;
    maixcam_last_update_ms = now_ms;
    maixcam_frame_count++;
    if (moves >= 3 && scenario != 2) {
        YuanData[0] = 60;
        YuanData[1] = 100;
    }
}
float imu_pid(float hope);
void motor_Omnidirectional_set(int x, int y, float target, uint16_t acceleration)
{
    assert(!query_enabled && acceleration == 10);
    assert(abs(x) <= expected_limit && abs(y) <= expected_limit);
    assert(fabsf(Wrap(target - expected_raw_target)) < .001f);
    /* Actual imu_pid must see a 2-degree error, not an extra starting angle. */
    assert(fabsf(imu_pid(target)) < 8.0f);
    moves++;
}

/* ACTUAL_VISUAL_AND_PID */

static void Reset(float start_yaw, float initial_raw, float target_yaw,
                   bool query_initial, bool raw_interface)
{
    ChassisNav_Config_t config;
    ChassisNav_Feedback_t feedback = {0};
    ChassisNav_Output_t output;
    ChassisNav_GetDefaultConfig(&config);
    feedback.encoder_valid_mask = 15;
    feedback.imu_valid = 1;
    feedback.imu_yaw_deg = initial_raw;
    assert(ChassisNav_Init(&config, &feedback));
    ChassisNav_CorrectPose(0, 0, start_yaw, initial_raw, 1);
    expected_raw_target = Wrap(initial_raw + target_yaw - start_yaw);
    yaw = Wrap(expected_raw_target + 2.0f);
    feedback.imu_yaw_deg = yaw;
    ChassisNav_Update(&feedback, .02f, &output);
    memset(&chassis, 0, sizeof(chassis));
    chassis.nav_initialized = chassis.imu_valid = 1;
    chassis.imu_yaw_deg = yaw;
    ChassisNav_GetState(&chassis.nav);
    /* The real complementary filter has not converged to the station angle.
     * This catches use of fused pose.yaw instead of corrected IMU reference. */
    now_ms = 1000;
    moves = stops = scenario = 0;
    query_enabled = query_initial;
    maixcam_frame_count = 1;
    maixcam_last_update_ms = now_ms;
    YuanData[0] = 100;
    YuanData[1] = 100;
    expected_limit = raw_interface ? 17 : 11;
}

static VisualPosition_Result_t Locate(bool raw_interface, float target, uint32_t timeout)
{
    return raw_interface ? VisualPosition_RunRawMaterial(60, 100, target, 1, timeout) :
                           VisualPosition_Run(60, 100, target, 1, timeout);
}

int main(void)
{
    static const float starts[] = {0, 90, -90, 180, -180};
    static const float initial_readings[] = {0, 76.4f, 179, -179};
    static const float stations[] = {-90, 90, 0};
    unsigned a, b, c, q, api, cases = 0;
    for (a = 0; a < 5; ++a) for (b = 0; b < 4; ++b)
    for (c = 0; c < 3; ++c) for (q = 0; q < 2; ++q)
    for (api = 0; api < 2; ++api) {
        Reset(starts[a], initial_readings[b], stations[c], q != 0, api != 0);
        assert(Locate(api != 0, stations[c], 100) == VISUAL_POSITION_OK);
        assert(moves == 3 && stops && query_enabled == (q != 0));
        cases++;
    }
    for (api = 0; api < 2; ++api) {
        for (q = 0; q < 2; ++q) for (a = 1; a <= 4; ++a) {
            Reset(90, 179, -90, q != 0, api != 0);
            scenario = a;
            if (a == 3) maixcam_last_update_ms = now_ms - 301;
            assert(Locate(api != 0, -90, 40) == VISUAL_POSITION_TIMEOUT);
            assert(stops && query_enabled == (q != 0));
            if (a != 2) assert(moves == 0);
        }
        for (a = 0; a < 5; ++a) {
            Reset(90, 0, -90, true, api != 0);
            if (a == 0) chassis.nav_initialized = 0;
            if (a == 1) chassis.imu_valid = 0;
            if (a == 2) chassis.nav.imu_fused = 0;
            if (a == 3) chassis.imu_yaw_deg = NAN;
            if (a == 4) chassis.nav.imu_yaw_deg = NAN;
            assert(Locate(api != 0, -90, 100) == VISUAL_POSITION_BAD_PARAMETER);
            assert(moves == 0 && query_enabled);
        }
        /* A later map-reference correction must replace the old starting offset. */
        Reset(90, 0, 0, true, api != 0);
        {
            ChassisNav_Feedback_t feedback = {0};
            ChassisNav_Output_t output;
            feedback.encoder_valid_mask = 15;
            feedback.imu_valid = 1;
            feedback.imu_yaw_deg = yaw;
            ChassisNav_CorrectPose(0, 0, 45, yaw, 1);
            expected_raw_target = Wrap(yaw - 45);
            yaw = Wrap(expected_raw_target + 2);
            feedback.imu_yaw_deg = yaw;
            ChassisNav_Update(&feedback, .02f, &output);
            chassis.imu_yaw_deg = yaw;
            ChassisNav_GetState(&chassis.nav);
        }
        assert(Locate(api != 0, 0, 100) == VISUAL_POSITION_OK);
        Reset(0, 0, 0, true, api != 0);
        assert(Locate(api != 0, NAN, 100) == VISUAL_POSITION_BAD_PARAMETER);
    }
    printf("PASS: %u real-nav reference cases; both visual APIs, all start angles, stations, arbitrary raw zeros, query-off operation, reference recorrection, stale/zero IMU and +/-180 PID wrap.\n", cases);
    return 0;
}
