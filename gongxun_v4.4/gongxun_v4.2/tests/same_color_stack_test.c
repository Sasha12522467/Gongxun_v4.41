/* Host mocks only. Runner compiles actual production functions, not copies.
 * No motors/UART hardware is used. Do not add this file to Keil.
 */
uint8_t A1, B1, C1, A2, B2, C2, A3, B3, C3, A4, B4, C4;
volatile uint8_t mat_1, mat_2, mat_3;
volatile uint16_t YuanData[2];
volatile uint32_t maixcam_frame_count, maixcam_last_update_ms;
int min = 1, max = 11, x_cha, y_cha;
typedef struct { int unused; } UART_HandleTypeDef;
UART_HandleTypeDef huart1;

static bool query_enabled;
static unsigned now_ms, camera_sends, first_calls, second_calls, processing_calls;
static unsigned servo_calls, stop_calls, move_calls, qr_calls;
static uint8_t sent_mode, placed_first[3], placed_second[3], processed_second[3];
/* 0=live target frames, 1=no frames, 2=wrong coordinates, 3=one frame only. */
static unsigned vision_behavior;

uint8_t ColorToCameraMode(uint8_t color);
void Maixcam_setmode(uint8_t mode);
VisualPosition_Result_t VisualPosition_Run(uint16_t x, uint16_t y, float yaw,
                                           uint8_t tolerance, uint32_t timeout);
static uint8_t MainTask_RunStageAction(uint8_t stage);

uint32_t HAL_GetTick(void) { return now_ms; }
void HAL_Delay(uint32_t ms) { now_ms += ms; }
int osDelay(uint32_t ms)
{
    now_ms += ms;
    if (vision_behavior == 1U ||
        (vision_behavior == 3U && maixcam_frame_count != 0U)) return 0;
    YuanData[0] = sent_mode == 2U ? 155U : 78U;
    YuanData[1] = sent_mode == 2U ? 138U : 93U;
    if (vision_behavior == 2U) YuanData[0] += 20U;
    maixcam_last_update_ms = now_ms;
    maixcam_frame_count++;
    return 0;
}
void UART_SendbyteOne(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && size == 3U);
    assert(data[0] == 0xFFU && data[2] == 0xFEU);
    assert(data[1] >= 1U && data[1] <= 8U);
    sent_mode = data[1];
    camera_sends++;
}
bool ChassisRuntime_IsEncoderQueryEnabled(void) { return query_enabled; }
void ChassisRuntime_SetEncoderQueryEnabled(bool enabled) { query_enabled = enabled; }
void Emm_V5_Stop_Now(uint8_t address, bool sync)
{
    assert(address == 0U && !sync);
    stop_calls++;
}
void motor_Omnidirectional_set(int x, int y, float yaw, uint16_t acceleration)
{
    assert(!query_enabled && acceleration == 10U && isfinite(yaw));
    assert(x >= -11 && x <= 11 && y >= -11 && y <= 11);
    move_calls++;
}
void ser_set(int angle, int duration)
{
    assert((angle == -70 || angle == -145) && duration == 1000);
    servo_calls++;
}
void Storage_First(uint8_t a, uint8_t b, uint8_t c)
{
    assert(!query_enabled && g_storage_stack.position_result == VISUAL_POSITION_OK);
    placed_first[0] = a; placed_first[1] = b; placed_first[2] = c;
    first_calls++;
}
void Storage_Second(uint8_t a, uint8_t b, uint8_t c)
{
    assert(!query_enabled && g_storage_stack.first_layer_done);
    assert(g_storage_stack.position_result == VISUAL_POSITION_OK);
    placed_second[0] = a; placed_second[1] = b; placed_second[2] = c;
    second_calls++;
}
void QRcode_print_1(void) { qr_calls++; }
void RawMaterial_First(uint8_t a, uint8_t b, uint8_t c) { (void)a; (void)b; (void)c; }
void RawMaterial_Second(uint8_t a, uint8_t b, uint8_t c) { (void)a; (void)b; (void)c; }
void Processing_First(uint8_t a, uint8_t b, uint8_t c) { (void)a; (void)b; (void)c; }
void Processing_Second(uint8_t a, uint8_t b, uint8_t c)
{
    processed_second[0] = a; processed_second[1] = b; processed_second[2] = c;
    processing_calls++;
}

static void ResetHardware(void)
{
    query_enabled = true;
    now_ms = 1000U;
    camera_sends = first_calls = second_calls = processing_calls = 0U;
    servo_calls = stop_calls = move_calls = qr_calls = 0U;
    vision_behavior = 0U;
    sent_mode = 0U;
    maixcam_frame_count = maixcam_last_update_ms = 0U;
    memset(placed_first, 0, sizeof(placed_first));
    memset(placed_second, 0, sizeof(placed_second));
    memset(&g_storage_stack, 0, sizeof(g_storage_stack));
    /* Prove stacking does not depend on these slot globals. */
    mat_1 = mat_2 = mat_3 = 0U;
}

static void TestAllMappings(void)
{
    static const uint8_t permutations[6][3] = {
        {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
    };
    static const uint8_t expected_mode[7] = {0,3,7,5,4,6,8};
    uint8_t a, b, c, i, r, p;
    unsigned checked = 0U;
    ResetHardware();
    for (a = 1U; a <= 6U; ++a) for (b = 1U; b <= 6U; ++b)
    for (c = 1U; c <= 6U; ++c) {
        uint8_t first[3] = {a,b,c};
        if (a == b || a == c || b == c) continue;
        for (r = 0U; r < 6U; ++r) for (p = 0U; p < 6U; ++p) {
            uint8_t rings[3], second[3];
            for (i = 0U; i < 3U; ++i) {
                rings[i] = permutations[r][i] + 1U;
                second[i] = first[permutations[p][i]];
            }
            assert(StorageStack_Configure(first, rings, second));
            assert(g_storage_stack.configured && !g_storage_stack.first_layer_done);
            assert(!g_storage_stack.second_layer_done && g_storage_stack.ring_color[0] == 0U);
            for (i = 0U; i < 3U; ++i) {
                assert(g_storage_stack.ring_color[rings[i]] == first[i]);
                assert(g_storage_stack.target_ring[i] == rings[permutations[p][i]]);
            }
            assert(g_storage_stack.camera_mode == expected_mode[g_storage_stack.ring_color[2]]);
            checked++;
        }
    }
    assert(checked == 4320U && camera_sends == 0U);
    printf("PASS: %u scan/ring/permutation combinations, ring2 color/mode, same-color targets\n", checked);
}

static void TestInvalidScan(void)
{
    uint8_t colors[3] = {6,4,2}, rings[3] = {3,1,2}, second[3] = {2,6,4};
    uint8_t invalid[3];
    unsigned i;
    ResetHardware();
    assert(!StorageStack_Configure(NULL, rings, second));
    assert(!StorageStack_Configure(colors, NULL, second));
    assert(!StorageStack_Configure(colors, rings, NULL));
    for (i = 0U; i < 3U; ++i) {
        memcpy(invalid, colors, 3U); invalid[i] = 0U;
        assert(!StorageStack_Configure(invalid, rings, second));
        invalid[i] = 7U;
        assert(!StorageStack_Configure(invalid, rings, second));
        memcpy(invalid, rings, 3U); invalid[i] = 4U;
        assert(!StorageStack_Configure(colors, invalid, second));
        memcpy(invalid, second, 3U); invalid[i] = 0U;
        assert(!StorageStack_Configure(colors, rings, invalid));
    }
    invalid[0] = 1U; invalid[1] = 1U; invalid[2] = 2U;
    assert(!StorageStack_Configure(invalid, rings, second));
    assert(!StorageStack_Configure(colors, invalid, second));
    assert(!StorageStack_Configure(colors, rings, invalid));
    invalid[0] = 1U; invalid[1] = 6U; invalid[2] = 4U;
    assert(!StorageStack_Configure(colors, rings, invalid));
    assert(g_storage_stack.fault == STORAGE_STACK_FAULT_BAD_SCAN);
    assert(!StorageStack_RunFirst() && !StorageStack_RunSecond());
    assert(camera_sends == 0U && first_calls == 0U && second_calls == 0U);
    assert(StorageStack_Configure(colors, rings, second));
    assert(!StorageStack_RunSecond());
    assert(g_storage_stack.fault == STORAGE_STACK_FAULT_NO_FIRST_LAYER && camera_sends == 0U);
    puts("PASS: invalid/duplicate/different-set scan, no-first-layer and fault guards");
}

static void LoadExample(void)
{
    ResetHardware();
    A1=6; B1=4; C1=2; A2=3; B2=1; C2=2;
    A3=2; B3=6; C3=4; A4=1; B4=2; C4=3;
    assert(MainTask_RunStageAction(0U) == 1U);
    assert(qr_calls == 1U && g_storage_stack.camera_mode == 7U);
    assert(s_stage_action_order[6][0] == 2U && s_stage_action_order[6][1] == 3U &&
           s_stage_action_order[6][2] == 1U);
}

static void TestMainIntegration(void)
{
    unsigned saved_sends, saved_first, saved_second;
    LoadExample();
    /* Later slot/QR changes cannot overwrite the configured first-layer snapshot. */
    mat_1=1U; mat_2=3U; mat_3=5U;
    A1=1U; B1=3U; C1=5U;
    assert(MainTask_RunStageAction(3U) == 1U);
    assert(first_calls == 1U && sent_mode == 2U && g_storage_stack.first_layer_done);
    assert(placed_first[0] == 3U && placed_first[1] == 1U && placed_first[2] == 2U);
    assert(MainTask_RunStageAction(5U) == 1U);
    assert(processing_calls == 1U && processed_second[0] == 1U &&
           processed_second[1] == 2U && processed_second[2] == 3U);
    assert(MainTask_RunStageAction(6U) == 1U);
    assert(second_calls == 1U && sent_mode == 7U && g_storage_stack.second_layer_done);
    assert(placed_second[0] == 2U && placed_second[1] == 3U && placed_second[2] == 1U);
    assert(g_storage_stack.position_result == VISUAL_POSITION_OK && query_enabled);
    saved_sends=camera_sends; saved_first=first_calls; saved_second=second_calls;
    assert(MainTask_RunStageAction(3U) == 1U && MainTask_RunStageAction(6U) == 1U);
    assert(camera_sends == saved_sends && first_calls == saved_first && second_calls == saved_second);
    puts("PASS: actual Main QR/stage3/stage5/stage6 integration, group4 processing unchanged, scan snapshots, no duplicate placement");
}

static void TestPositionFailures(void)
{
    unsigned behavior;
    for (behavior = 1U; behavior <= 3U; ++behavior) {
        unsigned saved_sends, saved_servos;
        LoadExample();
        vision_behavior = behavior;
        assert(MainTask_RunStageAction(3U) == 0U);
        assert(!g_storage_stack.first_layer_done && first_calls == 0U && second_calls == 0U);
        assert(g_storage_stack.fault == STORAGE_STACK_FAULT_FIRST_POSITION);
        assert(g_storage_stack.position_result == VISUAL_POSITION_TIMEOUT && query_enabled && stop_calls > 0U);
        saved_sends=camera_sends; saved_servos=servo_calls;
        assert(MainTask_RunStageAction(3U) == 0U);
        assert(camera_sends == saved_sends && servo_calls == saved_servos);

        LoadExample();
        assert(MainTask_RunStageAction(3U) == 1U);
        vision_behavior = behavior;
        maixcam_frame_count = maixcam_last_update_ms = 0U;
        assert(MainTask_RunStageAction(6U) == 0U);
        assert(g_storage_stack.first_layer_done && !g_storage_stack.second_layer_done && second_calls == 0U);
        assert(g_storage_stack.fault == STORAGE_STACK_FAULT_SECOND_POSITION && query_enabled);
        saved_sends=camera_sends; saved_servos=servo_calls;
        assert(MainTask_RunStageAction(6U) == 0U);
        assert(camera_sends == saved_sends && servo_calls == saved_servos);
        if (behavior == 2U) assert(move_calls > 0U);
    }
    assert(VisualPosition_Run(78U,93U,0.0f,0U,8000U) == VISUAL_POSITION_BAD_PARAMETER);
    assert(VisualPosition_Run(78U,93U,NAN,1U,8000U) == VISUAL_POSITION_BAD_PARAMETER);
    puts("PASS: actual visual controller no-frame/wrong-position/single-frame timeout, stop/restore, no blind stacking or repeated retry");
}

static void TestModesAndQueryRestore(void)
{
    uint8_t color, mode;
    static const uint8_t modes[7] = {0,3,7,5,4,6,8};
    for (color = 1U; color <= 6U; ++color) {
        uint8_t first[3], rings[3] = {1,2,3}, second[3];
        first[1] = color;
        first[0] = color == 1U ? 2U : 1U;
        first[2] = 1U;
        while (first[2] == first[0] || first[2] == first[1]) first[2]++;
        second[0]=first[2]; second[1]=first[0]; second[2]=first[1];
        ResetHardware();
        query_enabled = false; /* Existing caller-disabled state must be preserved. */
        assert(StorageStack_Configure(first, rings, second));
        assert(StorageStack_RunFirst() && StorageStack_RunSecond());
        assert(sent_mode == modes[color] && !query_enabled);
        assert(second_calls == 1U && placed_second[0] == 3U &&
               placed_second[1] == 1U && placed_second[2] == 2U);
    }
    ResetHardware();
    for (mode = 1U; mode <= 8U; ++mode) Maixcam_setmode(mode);
    assert(camera_sends == 8U && sent_mode == 8U);
    Maixcam_setmode(0U); Maixcam_setmode(9U);
    assert(camera_sends == 8U);
    assert(ColorToCameraMode(0U) == 0U && ColorToCameraMode(7U) == 0U);
    puts("PASS: all six ring2 colors/modes, actual mode1..8 packets exactly3 bytes, caller query state preserved");
}

int main(void)
{
    TestAllMappings();
    TestInvalidScan();
    TestMainIntegration();
    TestPositionFailures();
    TestModesAndQueryRestore();
    puts("PASS: same-color stacking host regression");
    return 0;
}
