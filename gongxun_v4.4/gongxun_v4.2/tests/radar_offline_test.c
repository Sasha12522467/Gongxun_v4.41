/* Host test: radar startup mocked, real A*; never sends hardware commands. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "radar_startup.h"

static radar_client_callbacks_t callbacks;
static unsigned init_calls, zone_calls, poll_calls, scan_calls;
static bool ready, busy, block_all;
static uint32_t tick;
static uint8_t expected_zone, selected_radar_zone;

uint32_t HAL_GetTick(void) { return tick; }
void RadarClient_Init(const radar_client_callbacks_t *cb, uint8_t node_id)
{
    radar_client_state_t state = {0};
    assert(node_id == 1U);
    callbacks = *cb;
    init_calls++;
    ready = true;
    busy = false;
    state.connected = true;
    state.ready = true;
    callbacks.state(&state);
}
radar_client_request_result_t RadarClient_RequestSelectZone(uint8_t zone)
{
    assert(zone == expected_zone);
    assert(zone == path_plan_get_start_zone());
    selected_radar_zone = zone;
    zone_calls++;
    return RADAR_CLIENT_REQUEST_ACCEPTED;
}
bool RadarClient_IsReady(void) { return ready; }
bool RadarClient_IsBusy(void) { return busy; }
radar_client_request_result_t RadarClient_RequestGridScan(void)
{
    assert(selected_radar_zone == expected_zone);
    scan_calls++;
    busy = true;
    return RADAR_CLIENT_REQUEST_ACCEPTED;
}
void RadarClient_Poll(void)
{
    unsigned i;
    poll_calls++;
    if (busy) {
        radar_client_grid_result_t grid = {0};
        grid.scan_sequence = 7U;
        if (block_all) {
            grid.count = 25U;
            for (i = 0; i < 25U; ++i) grid.grid_ids[i] = (uint8_t)i;
        }
        callbacks.grid(&grid);
        busy = false;
    }
}

static void CheckOffline(uint8_t zone)
{
    path_plan_result_t expected;
    unsigned before_init = init_calls, before_zone = zone_calls;
    unsigned before_poll = poll_calls, before_scan = scan_calls;
    unsigned i;
    assert(path_plan_get_start_zone() == zone);
    assert(RadarStartup_PlanWithoutRadar());
    assert(init_calls == before_init && zone_calls == before_zone);
    assert(poll_calls == before_poll && scan_calls == before_scan);
    assert(RadarStartup_IsFinished() && RadarStartup_IsSuccessful());
    assert(g_radar_startup.zone == zone);
    assert(g_radar_startup.start_ms == tick);
    assert(g_radar_startup.node_id == 0 && g_radar_startup.timeout_ms == 0);
    assert(g_radar_startup.fault == RADAR_STARTUP_FAULT_NONE);
    assert(g_radar_startup.grid_count == 0 && g_radar_startup.grid_received == 0);
    assert(g_radar_startup.scan_requested == 0 && !g_radar_startup.radar_state.connected);
    assert(g_radar_startup.scan_sequence == 0 && g_radar_startup.path_all_valid == 1);
    assert(!g_radar_startup.radar_state.ready && !g_radar_startup.radar_state.busy);
    assert(g_radar_startup.radar_state.selected_zone == 0);
    for (i = 0; i < RADAR_CLIENT_MAX_GRID_IDS; ++i) assert(g_radar_startup.grid_ids[i] == 0);
    path_plan_run(NULL, 0, &expected);
    assert(memcmp(RadarStartup_GetPath(), &expected, sizeof(expected)) == 0);
    assert(RadarStartup_GetPath()->start_zone == zone);
    for (i = 0; i < PP_MAX_STAGES; ++i) assert(RadarStartup_GetPath()->ok[i]);
    RadarStartup_Process();
    assert(poll_calls == before_poll); /* Done offline mode cannot accidentally poll UART. */
}

int main(void)
{
    uint8_t zone;
    for (zone = PP_START_ZONE_1; zone <= PP_START_ZONE_2; ++zone) {
        path_plan_result_t expected;
        unsigned i;
        expected_zone = zone;
        selected_radar_zone = 0;
        init_calls = zone_calls = poll_calls = scan_calls = 0;
        ready = busy = block_all = false;
        tick += 100U;
        assert(path_plan_select_start_zone(zone));
        memset(&g_radar_startup, 0xA5, sizeof(g_radar_startup));
        CheckOffline(zone);
        assert(init_calls == 0 && zone_calls == 0 && poll_calls == 0 && scan_calls == 0);

        /* Re-enabled radar receives the selected zone, and uses the same zone's A*. */
        assert(RadarStartup_Start(1U, zone, 20000U));
        RadarStartup_Process();
        RadarStartup_Process();
        assert(init_calls == 1 && zone_calls == 1 && scan_calls == 1 && poll_calls == 2);
        assert(selected_radar_zone == zone && g_radar_startup.zone == zone);
        assert(RadarStartup_IsSuccessful() && g_radar_startup.grid_received == 1);
        assert(g_radar_startup.scan_requested == 1 && g_radar_startup.scan_sequence == 7);
        assert(g_radar_startup.grid_count == 0 && g_radar_startup.path_all_valid == 1);
        path_plan_run(NULL, 0, &expected);
        assert(memcmp(RadarStartup_GetPath(), &expected, sizeof(expected)) == 0);
        assert(expected.start_zone == zone);
        for (i = 0; i < PP_MAX_STAGES; ++i) assert(expected.ok[i]);

        /* Both zones must refuse a scan that blocks all12 original roads. */
        block_all = true;
        assert(RadarStartup_Start(1U, zone, 20000U));
        RadarStartup_Process();
        RadarStartup_Process();
        assert(init_calls == 2 && zone_calls == 2 && scan_calls == 2 && poll_calls == 4);
        assert(RadarStartup_IsFinished() && !RadarStartup_IsSuccessful());
        assert(g_radar_startup.fault == RADAR_STARTUP_FAULT_NO_PATH);
        assert(g_radar_startup.zone == zone && g_radar_startup.path.start_zone == zone);
        assert(g_radar_startup.grid_count == 25 && g_radar_startup.path_all_valid == 0);
        assert(RadarStartup_GetPath() == NULL);
        for (i = 0; i < PP_MAX_STAGES; ++i) assert(!g_radar_startup.path.ok[i]);
        tick += 100U;
        CheckOffline(zone); /* Clear real stale scan data and failure, with zero extra I/O. */
        assert(init_calls == 2 && zone_calls == 2 && scan_calls == 2 && poll_calls == 4);
    }
    assert(path_plan_select_start_zone(PP_START_ZONE_1));
    CheckOffline(PP_START_ZONE_1); /* Switching back must not retain zone2's path. */
    puts("PASS: both zones offline/online, correct radar selection and8-stage plans,");
    puts("      zero offline I/O, stale data/fault cleared, both no-path guards, switch back to zone1");
    return 0;
}
