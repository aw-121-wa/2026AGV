#include "survey.h"

/* 3x3 交叉点，编号 n -> (axis[n % 3], axis[n / 3])；8 条外圈边 + 4 条辐条。 */
#define NODES      9U
#define BASE_EDGES 12U
#define MAX_EDGES  (2U * BASE_EDGES)

static const uint8_t base_edges[BASE_EDGES][2] = {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},
                                                  {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}};
static const int32_t axis[3] = {355, 1205, 2055};

/* 多重图：基础边 + 为消除奇度点重复走的边，edge_base 记录对应的基础边号。 */
static uint8_t edge_base[MAX_EDGES], edge_count;
static uint8_t route[MAX_EDGES + 1U], leg_edge[MAX_EDGES];
static unsigned length;

static MapPoint Point(unsigned n)
{
    return (MapPoint){axis[n % 3U], axis[n / 3U]};
}

static unsigned Other(unsigned base, unsigned n)
{
    return base_edges[base][0] == n ? base_edges[base][1] : base_edges[base][0];
}

/* BFS 求 from -> to 的最短边序列，追加为重复边；返回边数。 */
static unsigned AddShortestPath(unsigned from, unsigned to, uint8_t commit)
{
    uint8_t via[NODES], seen[NODES] = {0}, queue[NODES];
    unsigned head = 0, tail = 0;
    queue[tail++] = (uint8_t)from;
    seen[from] = 1U;
    while (head < tail && !seen[to])
    {
        unsigned n = queue[head++];
        for (unsigned e = 0; e < BASE_EDGES; ++e)
        {
            if (base_edges[e][0] != n && base_edges[e][1] != n)
                continue;
            unsigned m = Other(e, n);
            if (seen[m])
                continue;
            seen[m] = 1U;
            via[m] = (uint8_t)e;
            queue[tail++] = (uint8_t)m;
        }
    }
    unsigned count = 0;
    for (unsigned n = to; n != from; n = Other(via[n], n), ++count)
        if (commit && edge_count < MAX_EDGES)
            edge_base[edge_count++] = via[n];
    return count;
}

/* 奇度点最小权完美匹配（点数很少，直接递归枚举）。 */
static unsigned BestPairing(uint8_t *odd, unsigned count, uint8_t *best, unsigned cost_limit)
{
    if (count == 0U)
        return 0U;
    unsigned best_cost = cost_limit;
    uint8_t rest[NODES], rest_best[NODES];
    for (unsigned j = 1; j < count; ++j)
    {
        unsigned cost = AddShortestPath(odd[0], odd[j], 0U);
        if (cost >= best_cost)
            continue;
        unsigned k = 0;
        for (unsigned i = 1; i < count; ++i)
            if (i != j)
                rest[k++] = odd[i];
        cost += BestPairing(rest, k, rest_best, best_cost - cost);
        if (cost < best_cost)
        {
            best_cost = cost;
            best[0] = odd[0];
            best[1] = odd[j];
            for (unsigned i = 0; i < k; ++i)
                best[2 + i] = rest_best[i];
        }
    }
    return best_cost;
}

/* 中国邮路：给奇度点补最短重复边后求欧拉回路（Hierholzer），
   每条通道至少走一次且总里程最短（3x3 网格 16 段，原来来回各走一遍是 24 段）。 */
void Survey_Init(unsigned zone)
{
    uint8_t degree[NODES] = {0}, odd[NODES], pairs[NODES];
    unsigned odd_count = 0;
    edge_count = 0;
    for (unsigned e = 0; e < BASE_EDGES; ++e)
    {
        edge_base[edge_count++] = (uint8_t)e;
        ++degree[base_edges[e][0]];
        ++degree[base_edges[e][1]];
    }
    for (unsigned n = 0; n < NODES; ++n)
        if (degree[n] & 1U)
            odd[odd_count++] = (uint8_t)n;
    (void)BestPairing(odd, odd_count, pairs, ~0U);
    for (unsigned i = 0; i + 1U < odd_count; i += 2U)
        (void)AddShortestPath(pairs[i], pairs[i + 1U], 1U);

    uint8_t used[MAX_EDGES] = {0}, stack_node[MAX_EDGES + 1U], stack_edge[MAX_EDGES + 1U];
    uint8_t out_node[MAX_EDGES + 1U], out_edge[MAX_EDGES + 1U];
    unsigned top = 0, out = 0;
    stack_node[top] = (uint8_t)Survey_StartNode(zone);
    stack_edge[top++] = 0U;
    while (top)
    {
        unsigned n = stack_node[top - 1U], e = 0;
        while (e < edge_count &&
               (used[e] || (base_edges[edge_base[e]][0] != n && base_edges[edge_base[e]][1] != n)))
            ++e;
        if (e < edge_count)
        {
            used[e] = 1U;
            stack_node[top] = (uint8_t)Other(edge_base[e], n);
            stack_edge[top++] = edge_base[e];
        }
        else
        {
            --top;
            out_node[out] = (uint8_t)n;
            out_edge[out++] = stack_edge[top];
        }
    }
    /* 出栈序列是倒序回路，out_edge[k] 连接 out_node[k] 与 out_node[k + 1]。 */
    length = out;
    for (unsigned i = 0; i < out; ++i)
        route[i] = out_node[out - 1U - i];
    for (unsigned i = 0; i + 1U < out; ++i)
        leg_edge[i] = out_edge[out - 2U - i];
}

unsigned Survey_StartNode(unsigned zone)
{
    return zone == 1U ? 8U : 2U;
}

unsigned Survey_Count(void)
{
    return length ? length - 1U : 0U;
}

uint8_t Survey_Leg(unsigned i, MapPoint *a, MapPoint *b, unsigned *edge)
{
    if (i >= Survey_Count() || !a || !b || !edge)
        return 0;
    *a = Point(route[i]);
    *b = Point(route[i + 1U]);
    *edge = leg_edge[i];
    return 1;
}

MapPoint Survey_Gateway(unsigned zone)
{
    return Point(Survey_StartNode(zone));
}
