/* Host-only white-box tests; compile this file alone, NOT into Keil firmware. */
#include <assert.h>
#include <stdio.h>
#include "../bsp/scr/chassis_nav.c"

static void TestEnvelope(void)
{
    ChassisNav_Config_t config;
    ChassisNav_Feedback_t feedback = {0};
    float prev = 0;
    unsigned i;
    ChassisNav_GetDefaultConfig(&config);
    feedback.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    assert(ChassisNav_Init(&config, &feedback));
    assert(config.brake_deceleration_mm_s2 == 1000.0f);
    assert(config.brake_soft_speed_mm_s == 0.0f);
    assert(Nav_BrakeProfile(-1) == 0);
    assert(Nav_BrakeProfile(0) == 0);
    for (i = 1; i < 10000; i++) {
        float d = i * .1f;
        float v = Nav_BrakeProfile(d);
        assert(v >= prev && isfinite(v));
        assert(fabsf(v - sqrtf(2 * config.brake_deceleration_mm_s2 * d)) < .001f);
        prev = v;
    }
    /* An old caller setting the deprecated field must not re-enable soft braking. */
    s_nav.config.brake_soft_speed_mm_s = 100;
    assert(fabsf(Nav_BrakeProfile(10) - sqrtf(20000)) < .001f);
}

static void TestFixedBrakeSlew(void)
{
    float factor = (float)M_PI * s_nav.config.wheel_diameter_mm / 60;
    float high_a, low_a, diagonal_a;
    s_nav.state.command_vx_rpm = 200 / factor;
    s_nav.state.command_vy_rpm = 0;
    Nav_SlewTranslation(0, 0, .02f);
    high_a = (200 - s_nav.state.command_vx_rpm * factor) / .02f;
    s_nav.state.command_vx_rpm = 80 / factor;
    Nav_SlewTranslation(0, 0, .02f);
    low_a = (80 - s_nav.state.command_vx_rpm * factor) / .02f;
    assert(fabsf(high_a - s_nav.config.brake_deceleration_mm_s2) < .1f);
    assert(fabsf(low_a - high_a) < .1f); /* No low-speed taper. */
    assert(high_a > s_nav.config.acceleration_rpm_s * factor);

    s_nav.state.command_vx_rpm = 60 / factor;
    s_nav.state.command_vy_rpm = 80 / factor;
    Nav_SlewTranslation(0, 0, .02f);
    diagonal_a = (100 - hypotf(s_nav.state.command_vx_rpm,
                              s_nav.state.command_vy_rpm) * factor) / .02f;
    assert(fabsf(diagonal_a - high_a) < .1f); /* Vector brake, not per-axis. */
    assert(fabsf(s_nav.state.command_vx_rpm / s_nav.state.command_vy_rpm - .75f) < .001f);

    s_nav.state.command_vx_rpm = -200 / factor;
    s_nav.state.command_vy_rpm = 0;
    Nav_SlewTranslation(0, 0, .02f);
    assert(fabsf(s_nav.state.command_vx_rpm * factor + 180) < .001f);

    s_nav.state.command_vx_rpm = 10 / factor;
    Nav_SlewTranslation(0, 0, .02f);
    assert(fabsf(s_nav.state.command_vx_rpm) < .001f); /* Last step cannot reverse. */
    Nav_SlewTranslation(100 / factor, 0, .02f);
    assert(fabsf(s_nav.state.command_vx_rpm -
                 s_nav.config.acceleration_rpm_s * .02f) < .001f);
}

/* Ideal wheels plus optional first-order speed lag, encoder quantization included.
 * This validates software convergence only, not real friction/slip/clearance.
 */
static void TestMove(float dx, float dy, int transit, float lag_s, float cap)
{
    ChassisNav_Config_t config;
    ChassisNav_Feedback_t feedback = {0};
    ChassisNav_Output_t output = {0};
    ChassisNav_State_t state;
    double counts[4] = {0};
    float wheel[4] = {0};
    float tolerance, previous_command = 0;
    unsigned tick, i;
    ChassisNav_GetDefaultConfig(&config);
    feedback.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    assert(ChassisNav_Init(&config, &feedback));
    assert(ChassisNav_GotoXYLimited(150 + dx, 150 + dy, 0, cap));
    ChassisNav_SetTransitMode(transit != 0);
    tolerance = transit ? CHASSIS_NAV_TRANSIT_POSITION_MM : config.position_tolerance_mm;
    for (tick = 0; tick < 1500; tick++) {
        for (i = 0; i < 4; i++) {
            wheel[i] += (output.wheel_rpm[i] - wheel[i]) * .02f / (.02f + lag_s);
            counts[i] += wheel[i] * config.encoder_counts_per_rev * .02 / 60 *
                         config.encoder_sign[i];
            feedback.motor_count[i] = (int64_t)llround(counts[i]);
        }
        ChassisNav_Update(&feedback, .02f, &output);
        ChassisNav_GetState(&state);
        assert(output.status != CHASSIS_NAV_FAULT);
        for (i = 0; i < 4; i++) assert(fabsf(output.wheel_rpm[i]) <= cap + .001f);
        if (output.status == CHASSIS_NAV_DONE) break;
        previous_command = hypotf(state.command_vx_rpm, state.command_vy_rpm) *
                           (float)M_PI * config.wheel_diameter_mm / 60;
    }
    assert(tick < 1500);
    assert(state.position_error_mm <= tolerance + .01f);
    assert(previous_command <= NAV_STOP_COMMAND_MM_S + .01f);
    printf("PASS move (%g,%g), transit=%d, lag=%g: %.2fs, residual=%.2fmm\n",
           dx, dy, transit, lag_s, tick * .02, state.position_error_mm);
}

int main(void)
{
    ChassisNav_Config_t config;
    ChassisNav_GetDefaultConfig(&config);
    TestEnvelope();
    TestFixedBrakeSlew();
    TestMove(0, 1000, 0, 0, config.max_wheel_rpm);
    TestMove(0, 1000, 1, .08f, config.max_wheel_rpm);
    TestMove(500, 500, 1, .08f, config.max_wheel_rpm);
    TestMove(0, -1000, 0, .08f, config.max_wheel_rpm);
    TestMove(0, 300, 0, .08f, 60);
    TestMove(150, 200, 1, .08f, 80); /* 起步固定0度全向移到300/350。 */
    TestMove(0, 20, 0, .08f, config.max_wheel_rpm);
    TestMove(0, CHASSIS_NAV_TRANSIT_POSITION_MM + 1, 1, 0, config.max_wheel_rpm);
    puts("PASS: fixed-deceleration envelope, equal high/low-speed braking, independent acceleration, finite stop, wheel caps");
    return 0;
}
