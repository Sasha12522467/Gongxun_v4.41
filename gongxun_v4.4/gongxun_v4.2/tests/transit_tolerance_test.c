/* Host test: link with bsp/scr/chassis_nav.c, no motor hardware needed. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "chassis_nav.h"
#include "chassis_direction.h"

static ChassisNav_Feedback_t feedback;
static ChassisNav_Output_t output;
static ChassisNav_State_t state;

static void Init(void)
{
    ChassisNav_Config_t config;
    ChassisNav_GetDefaultConfig(&config);
    memset(&feedback, 0, sizeof(feedback));
    feedback.encoder_valid_mask = CHASSIS_NAV_ENCODER_ALL_VALID;
    assert(ChassisNav_Init(&config, &feedback));
}
static void Step(unsigned cycles)
{
    while (cycles--) ChassisNav_Update(&feedback, 0.02f, &output);
    ChassisNav_GetState(&state);
}

int main(void)
{
    unsigned i;
    Init();
    assert(ChassisNav_TurnTo(2.0f));
    ChassisNav_SetTransitMode(true);
    Step(1);
    assert(output.status == CHASSIS_NAV_RUNNING);
    Step(1);
    assert(output.status == CHASSIS_NAV_DONE); /* Two-degree residual accepted. */

    assert(ChassisNav_TurnTo(CHASSIS_NAV_TRANSIT_HEADING_DEG - 0.1f));
    ChassisNav_SetTransitMode(true);
    Step(2);
    assert(output.status == CHASSIS_NAV_DONE); /* New slightly wider transit band. */

    assert(ChassisNav_TurnTo(2.0f)); /* Docking target resets to strict. */
    Step(10);
    assert(state.transit_mode == 0 && output.status == CHASSIS_NAV_RUNNING);
    assert(ChassisNav_TurnTo(0.0f));
    Step(7);
    assert(output.status == CHASSIS_NAV_RUNNING);
    Step(1);
    assert(output.status == CHASSIS_NAV_DONE);

    Init();
    assert(ChassisNav_GotoXY(160, 150, 0));
    ChassisNav_SetTransitMode(true);
    Step(2);
    assert(output.status == CHASSIS_NAV_DONE); /* 10 mm residual. */
    assert(ChassisNav_GotoXY(150 + CHASSIS_NAV_TRANSIT_POSITION_MM - 1, 150, 0));
    ChassisNav_SetTransitMode(true);
    Step(2);
    assert(output.status == CHASSIS_NAV_DONE); /* Inside the transit position band. */
    assert(ChassisNav_GotoXY(160, 150, 0));
    Step(10);
    assert(output.status == CHASSIS_NAV_RUNNING && state.transit_mode == 0);

    Init();
    assert(ChassisNav_TurnTo(CHASSIS_NAV_TRANSIT_HEADING_DEG + 1.0f));
    ChassisNav_SetTransitMode(true);
    Step(10);
    assert(output.status == CHASSIS_NAV_RUNNING); /* Outside angle tolerance. */

    Init();
    assert(ChassisNav_TurnTo(0));
    ChassisNav_SetTransitMode(true);
    for (i = 0; i < 2; i++) {
        unsigned wheel;
        for (wheel = 0; wheel < 4; wheel++) feedback.motor_count[wheel] -= 500;
        Step(1);
        assert(state.velocity.wz_deg_s > CHASSIS_NAV_TRANSIT_YAW_RATE_DEG_S);
        assert(state.settled_cycles == 0 && output.status == CHASSIS_NAV_RUNNING);
    }

    Init();
    assert(ChassisNav_GotoXY(160, 150, 0));
    ChassisNav_SetTransitMode(true);
    for (i = 0; i < 2; i++) {
        feedback.motor_count[0] -= 500 * CHASSIS_FORWARD_SIGN;
        feedback.motor_count[1] += 500 * CHASSIS_FORWARD_SIGN;
        feedback.motor_count[2] += 500 * CHASSIS_FORWARD_SIGN;
        feedback.motor_count[3] -= 500 * CHASSIS_FORWARD_SIGN;
        Step(1);
        assert(state.position_error_mm < CHASSIS_NAV_TRANSIT_POSITION_MM);
        assert(state.velocity.vx_mm_s > CHASSIS_NAV_TRANSIT_SPEED_MM_S);
        assert(state.settled_cycles == 0 && output.status == CHASSIS_NAV_RUNNING);
    }

    Init();
    assert(ChassisNav_TurnTo(2));
    ChassisNav_SetTransitMode(true);
    feedback.encoder_valid_mask = 0;
    Step(10);
    assert(output.status == CHASSIS_NAV_FAULT); /* No completion on stale feedback. */
    puts("PASS: transit residuals, strict reset, speed/rate guards, encoder loss");
    return 0;
}
