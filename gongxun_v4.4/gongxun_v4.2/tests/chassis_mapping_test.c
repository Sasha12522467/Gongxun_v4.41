/* Host test prefix. The runner inserts unchanged production helpers at marker. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "chassis_direction.h"
#include "chassis_runtime.h"

ChassisRuntime_State_t g_chassis_runtime;
static uint32_t s_coarse_guard_ms, s_turn_phase_start_ms;
static int64_t raw_by_address[8];
static uint8_t query_address[4], missing_address;
static unsigned query_count, command_count, sync_count;
typedef struct {
    uint8_t address, direction, acceleration;
    uint16_t rpm;
    uint32_t pulse;
} TestMotorCommand;
static TestMotorCommand sent[4];

#if CHASSIS_DIAGONAL_SWAP
static const uint8_t expected[4] = {4, 5, 2, 3};
#else
static const uint8_t expected[4] = {2, 3, 4, 5};
#endif

/* UART is not exercised: only verify which address each logical wheel queries. */
static uint8_t ChassisRuntime_ReadPosition(uint8_t address, int64_t *count)
{
    assert(query_count < 4);
    query_address[query_count++] = address;
    if (address == missing_address) return 0;
    *count = raw_by_address[address];
    return 1;
}

static void Emm_V5_Vel_Control(uint8_t address, uint8_t direction,
                               uint16_t rpm, uint8_t acceleration, bool sync)
{
    assert(command_count < 4 && sync);
    assert(address >= 2 && address <= 5);
    sent[command_count].address = address;
    sent[command_count].direction = direction;
    sent[command_count].rpm = rpm;
    sent[command_count].acceleration = acceleration;
    command_count++;
}

static void Emm_V5_Pos_Control(uint8_t address, uint8_t direction,
                               uint16_t rpm, uint8_t acceleration,
                               uint32_t pulse, bool absolute, bool sync)
{
    assert(!absolute);
    Emm_V5_Vel_Control(address, direction, rpm, acceleration, sync);
    sent[command_count - 1].pulse = pulse;
}

static void Emm_V5_Synchronous_motion(uint8_t address)
{
    assert(address == 0);
    sync_count++;
}

static uint32_t HAL_GetTick(void) { return 500U; }
static void osDelay(uint32_t ms) { (void)ms; }
static void HAL_Delay(uint32_t ms) { (void)ms; }
static float imu_pid(float yaw) { return yaw; }

/* ACTUAL_PRODUCTION */

static void ResetCommands(void)
{
    command_count = sync_count = 0;
    memset(sent, 0, sizeof(sent));
}

static void CheckCommands(const float rpm[4])
{
    unsigned i;
    assert(command_count == 4 && sync_count == 1);
    for (i = 0; i < 4; ++i) {
        assert(sent[i].address == expected[i]);
        assert(sent[i].direction == (rpm[i] >= 0 ? 1 : 0));
        assert(sent[i].rpm == (uint16_t)fabsf(rpm[i]));
    }
}

int main(void)
{
    const float mixed[4] = {11, -22, 33, -44};
    const float old_forward[4] = {30, -30, -30, 30};
    const float visual[4] = {
        10 * CHASSIS_FORWARD_SIGN + 20 * CHASSIS_LATERAL_SIGN + 3,
       -10 * CHASSIS_FORWARD_SIGN + 20 * CHASSIS_LATERAL_SIGN + 3,
       -10 * CHASSIS_FORWARD_SIGN - 20 * CHASSIS_LATERAL_SIGN + 3,
        10 * CHASSIS_FORWARD_SIGN - 20 * CHASSIS_LATERAL_SIGN + 3};
    ChassisNav_Config_t config;
    ChassisNav_Feedback_t feedback = {0};
    ChassisNav_Output_t output;
    ChassisNav_State_t state;
    unsigned i;
    assert(memcmp(s_motor_address, expected, 4) == 0);

    ResetCommands();
    ChassisRuntime_SendWheelRpm(mixed);
    CheckCommands(mixed);

    for (i = 0; i < 4; ++i) raw_by_address[expected[i]] = -(int64_t)(100 + i);
    assert(ChassisRuntime_ReadAllPositions() == 0x0F);
    for (i = 0; i < 4; ++i) {
        assert(query_address[i] == expected[i]);
        assert(g_chassis_runtime.motor_count[i] == -(int64_t)(100 + i));
    }
    query_count = 0;
    missing_address = expected[2];
    assert(ChassisRuntime_ReadAllPositions() == 0x0B); /* Logical bit2 missing. */
    missing_address = 0;

    ChassisNav_GetDefaultConfig(&config);
    feedback.encoder_valid_mask = 0x0F;
    assert(ChassisNav_Init(&config, &feedback));
    ChassisNav_CorrectPose(0, 0, 0, 0, 0);
    for (i = 0; i < 4; ++i) {
        int64_t logical = (i == 0 || i == 3) ? 1024 : -1024;
        raw_by_address[expected[i]] = -logical * CHASSIS_FORWARD_SIGN;
    }
    query_count = 0;
    feedback.encoder_valid_mask = ChassisRuntime_ReadAllPositions();
    memcpy(feedback.motor_count, g_chassis_runtime.motor_count,
           sizeof(feedback.motor_count));
    ChassisNav_Update(&feedback, .02f, &output);
    ChassisNav_GetState(&state);
    assert(state.pose.y_mm > 0 && fabsf(state.pose.x_mm) < .001f);
    assert(fabsf(state.velocity.wz_deg_s) < .001f);

    for (i = 0; i < 2; ++i) {
        unsigned j;
        ResetCommands();
        ChassisRuntime_SendCoarseTurn(i == 0 ? 45.0f : -45.0f);
        assert(command_count == 4 && sync_count == 1);
        for (j = 0; j < 4; ++j) {
            assert(sent[j].address == expected[j]);
            assert(sent[j].direction == (i == 0 ? 1 : 0));
            assert(sent[j].pulse == 1914 && sent[j].rpm == 120);
        }
    }

    ResetCommands();
    motor_Omnidirectional_set(10, 20, 3, 0);
    CheckCommands(visual);
    ResetCommands();
    MotorSetVel(1, 30, 0);
    CheckCommands(old_forward);
    ResetCommands();
    MotorSetPulse(500, 2, 30, 0);
    CheckCommands(old_forward);
    for (i = 0; i < 4; ++i) assert(sent[i].pulse == (i % 2 == 0 ? 500U : 0U));
    printf("chassis mapping=%d: output, feedback, odometry, coarse turn and visual PASS\n",
           CHASSIS_DIAGONAL_SWAP);
    return 0;
}
