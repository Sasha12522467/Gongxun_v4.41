/* Host-only camera/RTOS/actuator mocks. Runner appends actual production code.
 * No hardware is accessed. Do not add this file to Keil.
 */
uint8_t WuLiaoData;
volatile uint8_t mat_1, mat_2, mat_3;
volatile uint16_t YuanData[2];
volatile uint32_t maixcam_frame_count, maixcam_last_update_ms;
static uint8_t order[3];
static unsigned grabs, polls[3];
static uint32_t now_ms, start_ms[3], finish_ms[3], initial_ms;
static int scenario;

void GrabMaterialsInOrder(uint8_t a, uint8_t b, uint8_t c);
uint32_t HAL_GetTick(void) { return now_ms; }

int osDelay(uint32_t duration_ms)
{
    uint32_t age = 0U;
    assert(grabs < 3U && duration_ms == RAW_MATERIAL_GRAB_POLL_MS);
    assert(g_raw_grab.status == RAW_MATERIAL_GRAB_WAITING && !g_raw_grab.ready);
    polls[grabs]++;
    assert(polls[grabs] < 10U); /* Incorrect gates must not hang the test. */
    now_ms += duration_ms;
    WuLiaoData = order[grabs];
    YuanData[0] = RAW_MATERIAL_GRAB_TARGET_X_PX;
    YuanData[1] = RAW_MATERIAL_GRAB_TARGET_Y_PX;

    if (polls[grabs] == 1U) {
        if (scenario == 1) WuLiaoData = order[grabs] == 1U ? 2U : 1U;
        if (scenario == 2) YuanData[0] += RAW_MATERIAL_GRAB_TOLERANCE_PX + 1U;
        if (scenario == 3) YuanData[1] += RAW_MATERIAL_GRAB_TOLERANCE_PX + 1U;
        if (scenario == 4) age = RAW_MATERIAL_GRAB_FRAME_TIMEOUT_MS + 1U;
        if (scenario == 5) {
            maixcam_last_update_ms = now_ms;
            return 0; /* Correct values, but NO new frame: must not grab. */
        }
        if (scenario == 6) {
            maixcam_frame_count = 0U;
            maixcam_last_update_ms = now_ms;
            return 0; /* Zero frame count is not a received frame. */
        }
        if (scenario == 7) {
            YuanData[0] -= RAW_MATERIAL_GRAB_TOLERANCE_PX;
            YuanData[1] -= RAW_MATERIAL_GRAB_TOLERANCE_PX;
        }
        if (scenario == 8) {
            YuanData[0] += RAW_MATERIAL_GRAB_TOLERANCE_PX;
            YuanData[1] += RAW_MATERIAL_GRAB_TOLERANCE_PX;
        }
        if (scenario == 9) { YuanData[0] = 65535U; YuanData[1] = 0U; }
        if (scenario == 10) age = RAW_MATERIAL_GRAB_FRAME_TIMEOUT_MS;
        if (scenario == 12) maixcam_frame_count = UINT32_MAX;
    }
    maixcam_last_update_ms = now_ms - age;
    maixcam_frame_count++;
    return 0;
}

void grab_middle(int slot)
{
    unsigned i = grabs;
    assert(i < 3U && slot == (int)i + 1);
    assert(g_raw_grab.status == RAW_MATERIAL_GRAB_EXECUTING);
    assert(g_raw_grab.ready && g_raw_grab.fresh && g_raw_grab.slot == i + 1U);
    assert(g_raw_grab.target_color == order[i] && g_raw_grab.measured_color == order[i]);
    assert(WuLiaoData == order[i] && g_raw_grab.frame_count == maixcam_frame_count);
    assert(g_raw_grab.frame_age_ms <= RAW_MATERIAL_GRAB_FRAME_TIMEOUT_MS);
    assert(g_raw_grab.error_x_px >= -(int32_t)RAW_MATERIAL_GRAB_TOLERANCE_PX &&
           g_raw_grab.error_x_px <= (int32_t)RAW_MATERIAL_GRAB_TOLERANCE_PX);
    assert(g_raw_grab.error_y_px >= -(int32_t)RAW_MATERIAL_GRAB_TOLERANCE_PX &&
           g_raw_grab.error_y_px <= (int32_t)RAW_MATERIAL_GRAB_TOLERANCE_PX);
    assert((i == 0U && mat_1 == 0U && mat_2 == 0U && mat_3 == 0U) ||
           (i == 1U && mat_1 == order[0] && mat_2 == 0U && mat_3 == 0U) ||
           (i == 2U && mat_1 == order[0] && mat_2 == order[1] && mat_3 == 0U));
    start_ms[i] = now_ms;
    now_ms += 1000U; /* Existing mechanical action duration is mocked, unchanged. */
    finish_ms[i] = now_ms;
    grabs++;
    /* Next object already looks correct at action end, but next loop must wait
     * for a NEW frame rather than consume this action-period image. */
    if (grabs < 3U) {
        WuLiaoData = order[grabs];
        YuanData[0] = RAW_MATERIAL_GRAB_TARGET_X_PX;
        YuanData[1] = RAW_MATERIAL_GRAB_TARGET_Y_PX;
        maixcam_last_update_ms = now_ms;
        maixcam_frame_count++;
    }
}

static void RunCase(int test_scenario, uint8_t a, uint8_t b, uint8_t c)
{
    unsigned i;
    scenario = test_scenario;
    order[0] = a; order[1] = b; order[2] = c;
    grabs = 0U;
    now_ms = initial_ms = scenario == 11 ? UINT32_MAX - 2U : 1000U;
    for (i = 0U; i < 3U; ++i) polls[i] = start_ms[i] = finish_ms[i] = 0U;
    mat_1 = mat_2 = mat_3 = 6U;
    /* A previously received perfectly aligned frame must not trigger object1. */
    maixcam_frame_count = 42U;
    maixcam_last_update_ms = now_ms;
    WuLiaoData = a;
    YuanData[0] = RAW_MATERIAL_GRAB_TARGET_X_PX;
    YuanData[1] = RAW_MATERIAL_GRAB_TARGET_Y_PX;
    GrabMaterialsInOrder(a, b, c);
    assert(grabs == 3U && mat_1 == a && mat_2 == b && mat_3 == c);
    assert(g_raw_grab.status == RAW_MATERIAL_GRAB_DONE && !g_raw_grab.ready && !g_raw_grab.fresh);
    assert(g_raw_grab.target_x_px == RAW_MATERIAL_GRAB_TARGET_X_PX);
    assert(g_raw_grab.target_y_px == RAW_MATERIAL_GRAB_TARGET_Y_PX);
    assert(g_raw_grab.tolerance_px == RAW_MATERIAL_GRAB_TOLERANCE_PX);
    for (i = 0U; i < 3U; ++i) {
        unsigned required_polls = (scenario >= 1 && scenario <= 6) ||
                                  scenario == 9 || scenario == 12 ? 2U : 1U;
        uint32_t previous = i == 0U ? initial_ms : finish_ms[i - 1U];
        assert(polls[i] == required_polls);
        assert((uint32_t)(start_ms[i] - previous) == required_polls * RAW_MATERIAL_GRAB_POLL_MS);
    }
}

int main(void)
{
    int s;
    for (s = 0; s <= 12; ++s) RunCase(s, 6U, 2U, 4U);
    RunCase(0, 1U, 5U, 3U); /* Later call/lap uses same rule and resets slot records. */
    printf("PASS: target(%u,%u) +/- %u px: all3 objects, color/XY AND, inclusive bounds, old/zero/stale frames, tick/frame wrap, no fixed delay, slots and repeated calls\n",
           (unsigned)RAW_MATERIAL_GRAB_TARGET_X_PX,
           (unsigned)RAW_MATERIAL_GRAB_TARGET_Y_PX,
           (unsigned)RAW_MATERIAL_GRAB_TOLERANCE_PX);
    return 0;
}
