/* Host-only exhaustive oracle: enumerate simple paths for every obstacle mask.
 * gcc -std=c99 -Wall -Wextra -Werror -Ibsp/inc tests/path_turn_cost_test.c -lm
 */
#include <assert.h>
#include <stdio.h>
#include "../bsp/scr/path_plan.c"

static float best;
static int shortest;
static int goal;
static int dock;
static float final_yaw;
static int exit_prefix;

static float OracleYaw(int from, int to)
{
    return atan2f((float)(nodes[from].x - nodes[to].x),
                  (float)(nodes[to].y - nodes[from].y)) * 180.0f /
                  3.14159265358979323846f;
}
static float OracleAngle(float a, float b)
{
    float angle = fabsf(remainderf(b - a, 360.0f));
    return angle > PP_TURN_IGNORE_DEG ? angle : 0.0f;
}
static float OracleTurnCost(float a, float b)
{
    float angle = OracleAngle(a, b);
    return angle == 0 ? 0 : PP_TURN_START_COST_MM + angle * PP_TURN_90_COST_MM / 90;
}
static void Enumerate(int node, unsigned visited, float yaw, float cost, int distance)
{
    int i;
    if (node == goal) {
        if (dock) cost += OracleTurnCost(yaw, final_yaw);
        if (cost < best) best = cost;
        if (distance < shortest) shortest = distance;
        return;
    }
    for (i = 0; i < num_edges; i++) {
        int next;
        float next_yaw;
        if (edges[i].is_blocked) continue;
        if (edges[i].from_node == node) next = edges[i].to_node;
        else if (edges[i].to_node == node) next = edges[i].from_node;
        else continue;
        if (visited & (1U << next)) continue;
        next_yaw = OracleYaw(node, next);
        Enumerate(next, visited | (1U << next), next_yaw,
                  cost + edges[i].cost + OracleTurnCost(yaw, next_yaw),
                  distance + edges[i].cost);
    }
}
static void CheckPlan(const path_plan_result_t *plan, int stage)
{
    int node = stages[stage][0];
    float yaw = path_plan_get_start_profile()->exit_yaw_deg;
    float cost = (float)exit_prefix, degrees = 0;
    int distance = exit_prefix, turns = 0, i;
    if (stage > 0) assert(path_plan_stage_docking_yaw(stage - 1, &yaw));
    assert(plan->length[stage] >= 0 && plan->length[stage] <= PP_MAX_PATH_LEN);
    for (i = 0; i <= plan->length[stage]; i++) {
        float angle, next_yaw;
        if (i == plan->length[stage]) {
            assert(node == goal);
            if (!dock) break;
            next_yaw = final_yaw;
        } else {
            int signed_edge = plan->edge_path[stage][i];
            int index = abs(signed_edge) - 1;
            int from, to;
            assert(index >= 0 && index < num_edges && !edges[index].is_blocked);
            from = signed_edge > 0 ? edges[index].from_node : edges[index].to_node;
            to = signed_edge > 0 ? edges[index].to_node : edges[index].from_node;
            assert(from == node);
            node = to;
            next_yaw = OracleYaw(from, to);
            distance += edges[index].cost;
            cost += edges[index].cost;
        }
        angle = OracleAngle(yaw, next_yaw);
        if (angle > 0) turns++;
        degrees += angle;
        cost += OracleTurnCost(yaw, next_yaw);
        yaw = next_yaw;
    }
    assert(fabsf(cost - best) < .02f);
    assert(fabsf(plan->total_cost[stage] - cost) < .02f);
    assert(plan->distance_mm[stage] == distance);
    assert(plan->turn_count[stage] == turns);
    assert(fabsf(plan->turn_degrees[stage] - degrees) < .01f);
}

int main(void)
{
    const int one_grid_per_edge[12] = {3, 1, 5, 7, 9, 13, 11, 15, 17, 19, 23, 21};
    path_plan_result_t plan;
    int grid[12], longer_choices = 0;
    unsigned mask, zone;
    for (zone = PP_START_ZONE_1; zone <= PP_START_ZONE_2; ++zone) {
        const PathPlan_StartProfile_t *profile;
        assert(path_plan_select_start_zone((uint8_t)zone));
        profile = path_plan_get_start_profile();
        for (mask = 0; mask < (1U << 12); mask++) {
            int n = 0, i, stage;
            int canonical_x, canonical_y;
            for (i = 0; i < 12; i++) if (mask & (1U << i)) grid[n++] = one_grid_per_edge[i];
            path_plan_run(grid, n, &plan);
            assert(plan.start_zone == zone);
            canonical_x = nodes[profile->route_node].x;
            canonical_y = nodes[profile->route_node].y;
            for (stage = 0; stage < PP_MAX_STAGES; stage++) {
                float start_yaw = profile->exit_yaw_deg;
                /* Independent oracle: only the selected start node becomes the safe
                 * exit during stage0. Return uses the restored canonical graph. */
                nodes[profile->route_node].x = stage == 0 ? (int)profile->exit_x_mm : canonical_x;
                nodes[profile->route_node].y = stage == 0 ? (int)profile->exit_y_mm : canonical_y;
                for (i = 0; i < num_edges; ++i) {
                    int a = edges[i].from_node, b = edges[i].to_node;
                    edges[i].cost = abs(nodes[a].x - nodes[b].x) + abs(nodes[a].y - nodes[b].y);
                }
                exit_prefix = stage == 0 ?
                    (int)(fabsf(profile->exit_x_mm - profile->initial_x_mm) +
                          fabsf(profile->exit_y_mm - profile->initial_y_mm)) : 0;
                if (stage == PP_MAX_STAGES - 1 && profile->final_parking_required) {
                    exit_prefix = (int)(fabsf(profile->park_x_mm - canonical_x) +
                                        fabsf(profile->park_y_mm - canonical_y));
                }
                goal = stages[stage][1];
                dock = path_plan_stage_docking_yaw(stage, &final_yaw);
                if (stage > 0) assert(path_plan_stage_docking_yaw(stage - 1, &start_yaw));
                best = FLT_MAX;
                shortest = 0x7fffffff;
                Enumerate(stages[stage][0], 1U << stages[stage][0], start_yaw,
                          (float)exit_prefix, exit_prefix);
                assert(plan.ok[stage] == (best < FLT_MAX));
                if (plan.ok[stage]) {
                    CheckPlan(&plan, stage);
                    if (plan.distance_mm[stage] > shortest) longer_choices++;
                } else {
                    assert(plan.length[stage] == 0 && plan.turn_count[stage] == 0);
                }
            }
        }
    }
    assert(longer_choices > 0); /* Preference can trade some distance for turns. */
    assert(path_plan_select_start_zone(PP_START_ZONE_1));
    path_plan_run(NULL, 0, &plan);
    puts("No obstacles (stage: edges, mm, turns, degrees):");
    for (mask = 0; mask < PP_MAX_STAGES; mask++) {
        int i;
        printf("%u:", mask);
        for (i = 0; i < plan.length[mask]; i++) printf(" %d", plan.edge_path[mask][i]);
        printf("; %dmm, %d turns, %.1fdeg\n", plan.distance_mm[mask],
               plan.turn_count[mask], plan.turn_degrees[mask]);
    }
    printf("PASS: 2 zones x 4096 obstacle masks x 8 stages, weighted optimum, blocked edges, stats; %d longer choices\n",
           longer_choices);
    return 0;
}
