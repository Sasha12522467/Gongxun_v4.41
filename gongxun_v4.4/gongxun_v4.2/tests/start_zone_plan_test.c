/* Host-only API regression; does not initialize UART or drive any motor.
 * gcc -std=c99 -Wall -Wextra -Werror -Ibsp/inc tests/start_zone_plan_test.c
 *     bsp/scr/path_plan.c -lm -o start_zone_plan_test
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "path_plan.h"

static const int node_xy[9][2] = {
    {150, 150}, {275, 1200}, {275, 2060}, {1200, 2000},
    {1200, 300}, {1200, 1200}, {2050, 1200}, {2060, 320}, {2060, 2060}
};
static const int edge_nodes[12][2] = {
    {0, 1}, {1, 2}, {0, 4}, {1, 5}, {2, 3}, {4, 5},
    {5, 3}, {4, 7}, {5, 6}, {3, 8}, {7, 6}, {6, 8}
};
static const int stage_nodes[PP_MAX_STAGES][2] = {
    {0, 1}, {1, 4}, {4, 3}, {3, 6}, {6, 4}, {4, 3}, {3, 6}, {6, 0}
};

static void CheckGraphUnchanged(void)
{
    int i;
    for (i = 0; i < 12; ++i) {
        float sx, sy, tx, ty;
        int a = edge_nodes[i][0], b = edge_nodes[i][1];
        assert(path_plan_edge_points(i + 1, &sx, &sy, &tx, &ty));
        assert(sx == node_xy[a][0] && sy == node_xy[a][1]);
        assert(tx == node_xy[b][0] && ty == node_xy[b][1]);
        assert(path_plan_edge_points(-(i + 1), &sx, &sy, &tx, &ty));
        assert(sx == node_xy[b][0] && sy == node_xy[b][1]);
        assert(tx == node_xy[a][0] && ty == node_xy[a][1]);
    }
}

static void CheckGridMapping(void)
{
    static const int expected[25][2] = {
        {0, 0}, {2, 0}, {2, 0}, {1, 0}, {0, 0},
        {3, 0}, {0, 0}, {4, 0}, {0, 0}, {5, 0},
        {0, 0}, {7, 0}, {0, 0}, {6, 0}, {0, 0},
        {8, 0}, {0, 0}, {9, 0}, {0, 0}, {10, 0},
        {12, 10}, {12, 0}, {0, 0}, {11, 0}, {11, 8}
    };
    int grid;
    for (grid = 0; grid < 25; ++grid) {
        int out[2] = {0, 0};
        int count = grid_to_edges(grid, out);
        int expected_count = expected[grid][1] ? 2 : (expected[grid][0] ? 1 : 0);
        assert(count == expected_count);
        if (count > 0) assert(out[0] == expected[grid][0]);
        if (count > 1) assert(out[1] == expected[grid][1]);
    }
    {
        int out[2];
        assert(grid_to_edges(-1, out) == 0 && grid_to_edges(25, out) == 0);
    }
}

static void CheckContinuity(const path_plan_result_t *plan, const int *grids, int count)
{
    unsigned blocked = 0U;
    int i, stage;
    int route_node = plan->start_zone == PP_START_ZONE_1 ? 0 : 2;
    for (i = 0; i < count; ++i) {
        int out[2], j, n = grid_to_edges(grids[i], out);
        for (j = 0; j < n; ++j) blocked |= 1U << (out[j] - 1);
    }
    for (stage = 0; stage < PP_MAX_STAGES; ++stage) {
        int node = stage == 0 ? route_node : stage_nodes[stage][0];
        int target = stage == PP_MAX_STAGES - 1 ? route_node : stage_nodes[stage][1];
        if (!plan->ok[stage]) {
            assert(plan->length[stage] == 0);
            continue;
        }
        for (i = 0; i < plan->length[stage]; ++i) {
            int signed_edge = plan->edge_path[stage][i];
            int edge = abs(signed_edge) - 1;
            int from, to;
            assert(edge >= 0 && edge < 12);
            assert(!(blocked & (1U << edge)));
            from = edge_nodes[edge][signed_edge > 0 ? 0 : 1];
            to = edge_nodes[edge][signed_edge > 0 ? 1 : 0];
            assert(from == node);
            node = to;
        }
        assert(node == target);
    }
}

static void CheckZone1Golden(const path_plan_result_t *plan)
{
    /* Baseline captured before adding zone2; zone1 routes/statistics stay unchanged. */
    static const int lengths[8] = {1, 2, 2, 2, 2, 2, 2, 3};
    static const int paths[8][3] = {
        {1, 0, 0}, {4, -6, 0}, {6, 7, 0}, {-7, 9, 0},
        {-9, -6, 0}, {6, 7, 0}, {-7, 9, 0}, {-9, -4, -1}
    };
    static const int distances[8] = {1225, 1825, 1700, 1650, 1750, 1700, 1650, 2950};
    static const int turns[8] = {0, 3, 2, 3, 3, 2, 3, 2};
    int stage, i;
    assert(plan->start_zone == PP_START_ZONE_1);
    for (stage = 0; stage < PP_MAX_STAGES; ++stage) {
        assert(plan->ok[stage] && plan->length[stage] == lengths[stage]);
        assert(plan->distance_mm[stage] == distances[stage]);
        assert(plan->turn_count[stage] == turns[stage]);
        for (i = 0; i < lengths[stage]; ++i) assert(plan->edge_path[stage][i] == paths[stage][i]);
    }
}

static void CheckProfileGeometry(const PathPlan_StartProfile_t *profile)
{
    float half_x = PP_ROBOT_SIZE_X_MM * 0.5f;
    float half_y = PP_ROBOT_SIZE_Y_MM * 0.5f;
    float radius = hypotf(half_x, half_y);
    float uncertainty = 20.0f; /* Current transit position tolerance. */
    float yellow_corner_y = profile->zone == PP_START_ZONE_1 ? 550.0f : 1850.0f;
    unsigned sample;
    assert(profile->exit_x_mm - uncertainty > radius);
    assert(profile->exit_y_mm - uncertainty > radius);
    assert(PP_FIELD_SIZE_MM - profile->exit_x_mm - uncertainty > radius);
    assert(PP_FIELD_SIZE_MM - profile->exit_y_mm - uncertainty > radius);
    assert(hypotf(550 - profile->exit_x_mm, yellow_corner_y - profile->exit_y_mm) >
           radius + uncertainty);
    for (sample = 0; sample <= 100; ++sample) {
        float t = sample * 0.01f;
        float x = profile->initial_x_mm + (profile->exit_x_mm - profile->initial_x_mm) * t;
        float y = profile->initial_y_mm + (profile->exit_y_mm - profile->initial_y_mm) * t;
        /* Initial heading stays exactly0 during omnidirectional departure. */
        assert(x - half_x >= 0 && x + half_x < 550);
        assert(y - half_y >= 0 && y + half_y <= PP_FIELD_SIZE_MM);
        if (profile->final_parking_required) {
            x = node_xy[profile->route_node][0] +
                (profile->park_x_mm - node_xy[profile->route_node][0]) * t;
            y = node_xy[profile->route_node][1] +
                (profile->park_y_mm - node_xy[profile->route_node][1]) * t;
            assert(x - half_x >= 0 && x + half_x < 550);
            assert(y - half_y > 1850 && y + half_y <= PP_FIELD_SIZE_MM);
        }
    }
    if (profile->zone == PP_START_ZONE_2) {
        /* Explicitly record the user's exact center: nominal bottom margin is ZERO.
         * This test does not prove real parking safe under slip/heading/position error. */
        assert(profile->park_y_mm + half_y == PP_FIELD_SIZE_MM);
    }
}

int main(void)
{
    path_plan_result_t baseline, plan, restored;
    const PathPlan_StartProfile_t *profile;
    const int obstacle_sets[][4] = {{3, -1, -1, -1}, {1, -1, -1, -1},
                                  {5, 9, -1, -1}, {7, 13, 21, -1},
                                  {3, 5, 1, 9}};
    unsigned zone;
    int set, i;
    assert(path_plan_get_start_zone() == PP_START_ZONE_1);
    path_plan_run(NULL, 0, &baseline);
    CheckZone1Golden(&baseline);

    for (zone = PP_START_ZONE_1; zone <= PP_START_ZONE_2; ++zone) {
        float docking_yaw;
        assert(path_plan_select_start_zone((uint8_t)zone));
        assert(path_plan_get_start_zone() == zone);
        profile = path_plan_get_start_profile();
        assert(profile->zone == zone && profile->initial_yaw_deg == 0);
        assert(profile->exit_yaw_deg == 0 && profile->park_yaw_deg == 0);
        assert(profile->initial_x_mm == 150 && profile->park_x_mm == 150);
        assert(profile->initial_y_mm == (zone == PP_START_ZONE_1 ? 150 : 2250));
        assert(profile->park_y_mm == profile->initial_y_mm);
        assert(profile->exit_x_mm == 300 && profile->exit_y_mm == (zone == PP_START_ZONE_1 ? 350 : 2050));
        assert(profile->route_node == (zone == PP_START_ZONE_1 ? 0 : 2));
        assert(profile->final_parking_required == (zone == PP_START_ZONE_2));
        CheckProfileGeometry(profile);
        assert(path_plan_stage_docking_yaw(7, &docking_yaw) == (zone == PP_START_ZONE_2));
        if (zone == PP_START_ZONE_2) assert(docking_yaw == 0);
        assert(!path_plan_select_start_zone(0) && !path_plan_select_start_zone(3));
        assert(!path_plan_select_start_zone(255));
        assert(path_plan_get_start_zone() == zone && path_plan_get_start_profile() == profile);

        path_plan_run(NULL, 0, &plan);
        assert(plan.start_zone == zone);
        for (i = 0; i < PP_MAX_STAGES; ++i) assert(plan.ok[i]);
        CheckContinuity(&plan, NULL, 0);
        CheckGraphUnchanged();
        CheckGridMapping();
        if (zone == PP_START_ZONE_1) CheckZone1Golden(&plan);
        for (set = 0; set < (int)(sizeof(obstacle_sets) / sizeof(obstacle_sets[0])); ++set) {
            int count = 0;
            while (count < 4 && obstacle_sets[set][count] >= 0) ++count;
            path_plan_run(obstacle_sets[set], count, &plan);
            CheckContinuity(&plan, obstacle_sets[set], count);
            CheckGraphUnchanged();
            CheckGridMapping();
        }
    }
    assert(path_plan_select_start_zone(PP_START_ZONE_1));
    path_plan_run(NULL, 0, &restored);
    assert(memcmp(&baseline, &restored, sizeof(baseline)) == 0);
    puts("PASS: both start/return zones, valid profiles, invalid selection unchanged,");
    puts("      8-stage continuity, original graph/grid map, zone1 golden and byte-identical restore");
    puts("      nominal departure/parking sweeps and safe-turn clearance; zone2 center has ZERO bottom margin");
    return 0;
}
