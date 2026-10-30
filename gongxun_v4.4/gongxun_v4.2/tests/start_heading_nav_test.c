/* Host-only regression, no UART or physical motor commands.
 * gcc -std=c99 -Wall -Wextra -Werror -Ibsp/inc -DPP_START_YAW_DEG=90
 * tests/start_heading_nav_test.c bsp/scr/chassis_nav.c bsp/scr/path_plan.c -lm
 * Repeat with 0, 90, -90, 180 and -180. Do not add this file to Keil firmware.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "chassis_nav.h"
#include "path_plan.h"

static float TestWrap(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

static void TestImuAlignment(const PathPlan_StartProfile_t *profile)
{
    const float raw_angles[] = {-179.0f, 0.0f, 76.4f, 179.0f};
    unsigned sample;
    for (sample = 0U; sample < sizeof(raw_angles) / sizeof(raw_angles[0]); sample++) {
        ChassisNav_Config_t config;
        ChassisNav_Feedback_t feedback;
        ChassisNav_Output_t output;
        ChassisNav_State_t state;
        float raw = raw_angles[sample];
        memset(&feedback, 0, sizeof(feedback));
        ChassisNav_GetDefaultConfig(&config);
        feedback.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
        feedback.imu_valid = 1U;
        feedback.imu_yaw_deg = raw;
        assert(ChassisNav_Init(&config, &feedback));
        /* Same API used by the real startup's queued pose correction. */
        ChassisNav_CorrectPose(profile->initial_x_mm, profile->initial_y_mm,
                               profile->initial_yaw_deg, raw, 1U);
        ChassisNav_Update(&feedback, 0.02f, &output);
        ChassisNav_GetState(&state);
        assert(state.imu_fused == 1U);
        assert(fabsf(state.pose.x_mm - profile->initial_x_mm) < 0.001f);
        assert(fabsf(state.pose.y_mm - profile->initial_y_mm) < 0.001f);
        assert(fabsf(TestWrap(state.pose.yaw_deg - profile->initial_yaw_deg)) < 0.001f);
        assert(fabsf(TestWrap(state.imu_yaw_deg - profile->initial_yaw_deg)) < 0.001f);

        /* Wrapped raw angles must still correct in the same positive direction. */
        feedback.imu_yaw_deg = TestWrap(raw + 7.0f);
        ChassisNav_Update(&feedback, 0.02f, &output);
        ChassisNav_GetState(&state);
        assert(fabsf(TestWrap(state.pose.yaw_deg - profile->initial_yaw_deg) -
                     7.0f * (1.0f - config.fusion_wheel_alpha)) < 0.002f);
    }
}

static void TestDeparture(const PathPlan_StartProfile_t *profile)
{
    ChassisNav_Config_t config;
    ChassisNav_Feedback_t feedback;
    ChassisNav_Output_t output;
    ChassisNav_State_t state;
    double counts[CHASSIS_NAV_WHEEL_COUNT] = {0};
    unsigned tick, wheel;
    memset(&feedback, 0, sizeof(feedback));
    memset(&output, 0, sizeof(output));
    ChassisNav_GetDefaultConfig(&config);
    feedback.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    feedback.imu_valid = 1U;
    feedback.imu_yaw_deg = 137.4f; /* Deliberately not zero or the map yaw. */
    assert(ChassisNav_Init(&config, &feedback));
    ChassisNav_CorrectPose(profile->initial_x_mm, profile->initial_y_mm,
                           profile->initial_yaw_deg, feedback.imu_yaw_deg, 1U);
    assert(ChassisNav_GotoXYLimited(profile->exit_x_mm, profile->exit_y_mm,
                                   profile->exit_yaw_deg, 80.0f));
    ChassisNav_SetTransitMode(true);
    for (tick = 0U; tick < 1000U; tick++) {
        /* Ideal wheels and quantized cumulative counts, not a slip model. */
        for (wheel = 0U; wheel < CHASSIS_NAV_WHEEL_COUNT; wheel++) {
            counts[wheel] += output.wheel_rpm[wheel] *
                config.encoder_counts_per_rev * 0.02 / 60.0 * config.encoder_sign[wheel];
            feedback.motor_count[wheel] = (int64_t)llround(counts[wheel]);
        }
        ChassisNav_Update(&feedback, 0.02f, &output);
        ChassisNav_GetState(&state);
        assert(output.status != CHASSIS_NAV_FAULT);
        assert(fabsf(TestWrap(state.pose.yaw_deg - profile->initial_yaw_deg)) < 0.1f);
        for (wheel = 0U; wheel < CHASSIS_NAV_WHEEL_COUNT; wheel++) {
            assert(fabsf(output.wheel_rpm[wheel]) <= 80.001f);
        }
        if (output.status == CHASSIS_NAV_DONE) break;
    }
    assert(tick < 1000U);
    assert(state.position_error_mm <= CHASSIS_NAV_TRANSIT_POSITION_MM + 0.01f);
    assert(state.transit_mode == 1U && state.imu_fused == 1U);
    printf("PASS: zone %u yaw %d, raw IMU 137.4 -> map yaw, departure to (%.0f,%.0f), %.2fs, residual %.2fmm\n",
           (unsigned)profile->zone, PP_START_YAW_DEG, profile->exit_x_mm,
           profile->exit_y_mm, tick * 0.02, state.position_error_mm);
}

static void TestProfilesAndDocking(void)
{
    static const float work_yaws[7] = {0.0f, -90.0f, 90.0f, 0.0f, -90.0f, 90.0f, 0.0f};
    uint8_t zone;
    for (zone = PP_START_ZONE_1; zone <= PP_START_ZONE_2; zone++) {
        const PathPlan_StartProfile_t *profile;
        path_plan_result_t plan;
        unsigned stage;
        float yaw;
        assert(path_plan_select_start_zone(zone));
        profile = path_plan_get_start_profile();
        assert(profile->initial_yaw_deg == (float)PP_START_YAW_DEG);
        assert(profile->exit_yaw_deg == profile->initial_yaw_deg);
        assert(profile->park_yaw_deg == 0.0f);
        assert(profile->initial_x_mm == 150.0f && profile->park_x_mm == 150.0f);
        assert(profile->initial_y_mm == (zone == PP_START_ZONE_1 ? 150.0f : 2250.0f));
        assert(profile->park_y_mm == profile->initial_y_mm);
        assert(profile->exit_x_mm == 300.0f);
        assert(profile->exit_y_mm == (zone == PP_START_ZONE_1 ? 350.0f : 2050.0f));
        path_plan_run(NULL, 0, &plan);
        assert(plan.start_zone == zone);
        for (stage = 0U; stage < PP_MAX_STAGES; stage++) assert(plan.ok[stage]);
        for (stage = 0U; stage < 7U; stage++) {
            assert(path_plan_stage_docking_yaw((int)stage, &yaw));
            assert(yaw == work_yaws[stage]);
        }
        assert(path_plan_stage_docking_yaw(7, &yaw) == (zone == PP_START_ZONE_2));
        if (zone == PP_START_ZONE_2) assert(yaw == 0.0f);
        TestImuAlignment(profile);
        TestDeparture(profile);
    }
    assert(path_plan_select_start_zone(PP_START_ZONE_1));
}

int main(void)
{
    TestProfilesAndDocking();
    puts("PASS: configurable initial yaw, arbitrary raw IMU zero, wrapped fusion, same map/exits, work docking and parking unchanged");
    return 0;
}
