/* Host-only white-box direction tests; no UART or physical motor operations. */
#include <assert.h>
#include <stdio.h>
#include "../bsp/scr/chassis_nav.c"

static void Init(ChassisNav_Feedback_t *feedback, float yaw)
{
    ChassisNav_Config_t config;
    memset(feedback, 0, sizeof(*feedback));
    feedback->encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    ChassisNav_GetDefaultConfig(&config);
    assert(ChassisNav_Init(&config, feedback));
    ChassisNav_CorrectPose(0.0f, 0.0f, yaw, 0.0f, 0U);
}

static void CheckMix(void)
{
    ChassisNav_Feedback_t feedback;
    ChassisNav_Output_t output;
    float f = (float)CHASSIS_FORWARD_SIGN;
    float l = (float)CHASSIS_LATERAL_SIGN;
    Init(&feedback, 0);

    Nav_MixAndLimit(10, 0, 0, &output);
    assert(output.wheel_rpm[0] == 10*f && output.wheel_rpm[1] == -10*f);
    assert(output.wheel_rpm[2] == -10*f && output.wheel_rpm[3] == 10*f);
    Nav_MixAndLimit(-10, 0, 0, &output);
    assert(output.wheel_rpm[0] == -10*f && output.wheel_rpm[1] == 10*f);
    assert(output.wheel_rpm[2] == 10*f && output.wheel_rpm[3] == -10*f);

    Nav_MixAndLimit(0, 10, 0, &output);
    assert(output.wheel_rpm[0] == 10*l && output.wheel_rpm[1] == 10*l);
    assert(output.wheel_rpm[2] == -10*l && output.wheel_rpm[3] == -10*l);
    Nav_MixAndLimit(0, 0, 10, &output);
    assert(output.wheel_rpm[0] == 10 && output.wheel_rpm[1] == 10);
    assert(output.wheel_rpm[2] == 10 && output.wheel_rpm[3] == 10);

    /* Independent inverse: mixed translation and rotation must remain decoupled. */
    Nav_MixAndLimit(10, 20, 3, &output);
    assert(fabsf((output.wheel_rpm[0]-output.wheel_rpm[1]-
                  output.wheel_rpm[2]+output.wheel_rpm[3])*0.25f*f - 10) < .001f);
    assert(fabsf((output.wheel_rpm[0]+output.wheel_rpm[1]-
                  output.wheel_rpm[2]-output.wheel_rpm[3])*0.25f*l - 20) < .001f);
    assert(fabsf((output.wheel_rpm[0]+output.wheel_rpm[1]+
                  output.wheel_rpm[2]+output.wheel_rpm[3])*0.25f - 3) < .001f);
}

static void CheckForwardFeedback(float yaw, int forward)
{
    ChassisNav_Feedback_t feedback;
    ChassisNav_Output_t output;
    float yaw_rad = yaw * (float)M_PI / 180;
    float distance;
    int64_t count = 1024 * forward * CHASSIS_FORWARD_SIGN;
    Init(&feedback, yaw);
    distance = 1024.0f * (float)M_PI * s_nav.config.wheel_diameter_mm /
               s_nav.config.encoder_counts_per_rev;
    /* Encoder protocol sign=-1. New forward uses raw [+,-,-,+]. */
    feedback.motor_count[0] = -count;
    feedback.motor_count[1] = count;
    feedback.motor_count[2] = count;
    feedback.motor_count[3] = -count;
    ChassisNav_Update(&feedback, .02f, &output);
    assert(fabsf(s_nav.state.pose.x_mm + forward*distance*sinf(yaw_rad)) < .001f);
    assert(fabsf(s_nav.state.pose.y_mm - forward*distance*cosf(yaw_rad)) < .001f);
    assert(fabsf(s_nav.state.velocity.vx_mm_s - forward*distance/.02f) < .01f);
    assert(fabsf(s_nav.state.velocity.vy_mm_s) < .001f);
    assert(fabsf(s_nav.state.velocity.wz_deg_s) < .001f);
}

static void CheckLateralAndTurn(void)
{
    ChassisNav_Feedback_t feedback;
    ChassisNav_Output_t output;
    unsigned i;
    Init(&feedback, 0);
    feedback.motor_count[0] = -1024 * CHASSIS_LATERAL_SIGN;
    feedback.motor_count[1] = -1024 * CHASSIS_LATERAL_SIGN;
    feedback.motor_count[2] = 1024 * CHASSIS_LATERAL_SIGN;
    feedback.motor_count[3] = 1024 * CHASSIS_LATERAL_SIGN;
    ChassisNav_Update(&feedback, .02f, &output);
    assert(s_nav.state.pose.x_mm < 0 && fabsf(s_nav.state.pose.y_mm) < .001f);
    assert(s_nav.state.velocity.vy_mm_s > 0);

    Init(&feedback, 0);
    for (i = 0; i < 4; i++) feedback.motor_count[i] = -1024;
    ChassisNav_Update(&feedback, .02f, &output);
    assert(s_nav.state.wheel_yaw_deg > 0 && s_nav.state.velocity.wz_deg_s > 0);
    assert(fabsf(s_nav.state.pose.x_mm) < .001f && fabsf(s_nav.state.pose.y_mm) < .001f);
}

int main(void)
{
    CheckMix();
    CheckForwardFeedback(0, 1);
    CheckForwardFeedback(0, -1);
    CheckForwardFeedback(90, 1);
    CheckForwardFeedback(-90, 1);
    CheckForwardFeedback(180, 1);
    CheckLateralAndTurn();
    puts("PASS: forward/reverse wheel signs and encoder feedback, all headings, lateral unchanged, turn unchanged, mixed inverse");
    return 0;
}
