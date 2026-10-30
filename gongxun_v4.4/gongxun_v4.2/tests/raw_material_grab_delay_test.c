/* HISTORICAL time-based-policy test, retained for reference only; not compiled.
 * Current runner delegates to raw_material_grab_window_test.c.
 * Host-only mocks; former entry was run_raw_material_grab_delay_test.ps1.
 * The runner appends the ACTUAL GrabMaterialsInOrder function from action.c.
 * No UART, motors or other hardware is accessed. Do not add to Keil.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

uint8_t WuLiaoData;
volatile uint8_t mat_1, mat_2, mat_3;

static uint8_t expected[3];
static unsigned grabs, polls[3], match_delays[3];
static unsigned now_ms, start_ms[3], finish_ms[3];
static int scenario;

void GrabMaterialsInOrder(uint8_t a, uint8_t b, uint8_t c);

int osDelay(uint32_t duration_ms)
{
    assert(grabs < 3U);
    now_ms += duration_ms;
    assert(now_ms < 30000U); /* Unexpected retries must not hang the host test. */
    if (duration_ms == 5U) {
        polls[grabs]++;
        WuLiaoData = (scenario == 1 && grabs == 0U && polls[0] == 1U) ?
                    0U : expected[grabs];
    } else {
        assert(duration_ms == RAW_MATERIAL_NEXT_GRAB_DELAY_MS);
        assert(grabs == 1U || grabs == 2U); /* Never delay the first object. */
        match_delays[grabs]++;
        if (((scenario == 2 && grabs == 1U) ||
             (scenario == 3 && grabs == 2U)) && match_delays[grabs] == 1U) {
            /* Simulate an interrupt changing the label during the delay. */
            WuLiaoData = scenario == 3 ? expected[0] : 0U;
        }
    }
    return 0;
}

void grab_middle(int slot)
{
    assert(grabs < 3U && slot == (int)grabs + 1);
    assert(WuLiaoData == expected[grabs]); /* Wrong label must never be grabbed. */
    assert((grabs == 0U && mat_1 == 0U && mat_2 == 0U && mat_3 == 0U) ||
           (grabs == 1U && mat_1 == expected[0] && mat_2 == 0U && mat_3 == 0U) ||
           (grabs == 2U && mat_1 == expected[0] && mat_2 == expected[1] && mat_3 == 0U));
    if (grabs > 0U) {
        assert(match_delays[grabs] >= 1U);
        assert(now_ms - finish_ms[grabs - 1U] >= RAW_MATERIAL_NEXT_GRAB_DELAY_MS);
    }
    start_ms[grabs] = now_ms;
    now_ms += 1000U; /* Mock duration of the existing mechanical operation. */
    finish_ms[grabs] = now_ms;
    grabs++;
    WuLiaoData = grabs < 3U ? expected[grabs] : 0U;
}

static void RunCase(int test_scenario, uint8_t a, uint8_t b, uint8_t c)
{
    unsigned i;
    scenario = test_scenario;
    expected[0] = a; expected[1] = b; expected[2] = c;
    grabs = now_ms = 0U;
    for (i = 0U; i < 3U; ++i) {
        polls[i] = match_delays[i] = start_ms[i] = finish_ms[i] = 0U;
    }
    /* Nonzero stale slot records must be reset for each new lap/call. */
    mat_1 = mat_2 = mat_3 = 6U;
    WuLiaoData = test_scenario == 1 ? 0U : a;
    GrabMaterialsInOrder(a, b, c);
    assert(grabs == 3U && mat_1 == a && mat_2 == b && mat_3 == c);
    assert(match_delays[0] == 0U);
    assert(start_ms[0] == (test_scenario == 1 ? 10U : 0U));
    assert(polls[0] == (test_scenario == 1 ? 2U : 0U));
    for (i = 1U; i < 3U; ++i) {
        int lost = (test_scenario == 2 && i == 1U) ||
                   (test_scenario == 3 && i == 2U);
        assert(match_delays[i] == (lost ? 2U : 1U));
        assert(polls[i] == (lost ? 1U : 0U));
        assert(start_ms[i] - finish_ms[i - 1U] ==
               RAW_MATERIAL_NEXT_GRAB_DELAY_MS * (lost ? 2U : 1U) + (lost ? 5U : 0U));
    }
}

int main(void)
{
    RunCase(0, 1U, 4U, 6U); /* Next target already visible after each operation. */
    RunCase(1, 6U, 2U, 4U); /* First target absent: cooperative polling, no 400ms. */
    RunCase(2, 5U, 3U, 1U); /* Second target disappears during the first delay. */
    RunCase(3, 2U, 6U, 5U); /* Third target changes to another color during delay. */
    RunCase(0, 6U, 2U, 4U); /* A later call/lap must also start without extra delay. */
    printf("PASS: %u ms next-object delay, first immediate, RTOS polling, label recheck/retry, slots/order, repeat call\n",
           (unsigned)RAW_MATERIAL_NEXT_GRAB_DELAY_MS);
    return 0;
}
