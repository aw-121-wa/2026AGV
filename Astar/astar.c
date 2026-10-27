#include "astar.h"
#include "agv_config.h"
#define UNSEEN UINT16_MAX
#define CLOSED (UINT16_MAX - 1U)
_Static_assert(MAP_CELLS < CLOSED, "Grid too large for 16-bit node ids");
static uint16_t cost[MAP_CELLS] AGV_RAM;
static uint16_t heap[MAP_CELLS] AGV_RAM;
static uint16_t position[MAP_CELLS] AGV_RAM;
static uint8_t parent[MAP_CELLS] AGV_RAM;
static uint16_t count, start_id, goal_id, path_len, trace_id;
static uint16_t min_r, min_c, max_r, max_c;
static unsigned clear_index;
static uint8_t phase;
static uint32_t version;
static AStarStatus status;
static const int dr[4] = {-1, 1, 0, 0}, dc[4] = {0, 0, -1, 1};

/* 曼哈顿距离启发值，用于开放列表排序。 */
static unsigned Heuristic(uint16_t n)
{
    int r = (int)(n / MAP_COLS) - (int)(goal_id / MAP_COLS);
    int c = (int)(n % MAP_COLS) - (int)(goal_id % MAP_COLS);
    return (unsigned)((r < 0 ? -r : r) + (c < 0 ? -c : c));
}

/* 比较两个节点的 f = g + h，f 相同时再比 h。 */
static uint8_t Less(uint16_t a, uint16_t b)
{
    unsigned ha = Heuristic(a), hb = Heuristic(b), fa = cost[a] + ha, fb = cost[b] + hb;
    return fa < fb || (fa == fb && ha < hb);
}

/* 交换堆中两个位置，并同步反向索引。 */
static void Swap(unsigned a, unsigned b)
{
    uint16_t t = heap[a];
    heap[a] = heap[b];
    heap[b] = t;
    position[heap[a]] = (uint16_t)a;
    position[heap[b]] = (uint16_t)b;
}

/* 新节点上浮，维持小顶堆性质。 */
static void Up(unsigned i)
{
    while (i && Less(heap[i], heap[(i - 1) / 2]))
    {
        unsigned p = (i - 1) / 2;
        Swap(i, p);
        i = p;
    }
}

/* 弹出 f 最小的节点并下沉修复堆，同时把它标记为已关闭。 */
static uint16_t Pop(void)
{
    uint16_t n = heap[0];
    --count;
    if (count)
    {
        heap[0] = heap[count];
        position[heap[0]] = 0;
        unsigned i = 0;
        for (;;)
        {
            unsigned l = 2 * i + 1, r = l + 1, b = i;
            if (l < count && Less(heap[l], heap[b]))
                b = l;
            if (r < count && Less(heap[r], heap[b]))
                b = r;
            if (b == i)
                break;
            Swap(i, b);
            i = b;
        }
    }
    position[n] = CLOSED;
    return n;
}

/* 初始化一次搜索：校验起终点与搜索窗口后进入 BUSY 状态。 */
uint8_t AStar_Begin(MapCell a, MapCell b, uint16_t r0, uint16_t c0, uint16_t r1, uint16_t c1)
{
    status = ASTAR_NO_PATH;
    path_len = 0;
    if (r0 > r1 || c0 > c1 || r1 >= MAP_ROWS || c1 >= MAP_COLS || a.row < r0 || a.row > r1 ||
        a.col < c0 || a.col > c1 || b.row < r0 || b.row > r1 || b.col < c0 || b.col > c1 ||
        !Map_CanStand(a.row, a.col) || !Map_CanStand(b.row, b.col))
        return 0;
    min_r = r0;
    min_c = c0;
    max_r = r1;
    max_c = c1;
    start_id = a.row * MAP_COLS + a.col;
    goal_id = b.row * MAP_COLS + b.col;
    count = 0;
    clear_index = 0;
    phase = 0;
    version = Map_Revision();
    status = ASTAR_BUSY;
    return 1;
}

/* 分步推进搜索：先分块清表，再按预算扩展节点，命中终点后回溯路径。 */
void AStar_Step(unsigned budget)
{
    if (status != ASTAR_BUSY)
        return;
    if (version != Map_Revision())
    {
        status = ASTAR_INVALIDATED;
        return;
    }
    /* Clearing is also bounded: no large memset in the motor-control tick. */
    if (phase == 0)
    {
        for (unsigned n = 0; n < budget * 16U && clear_index < MAP_CELLS; ++n)
            position[clear_index++] = UNSEEN;
        if (clear_index == MAP_CELLS)
        {
            cost[start_id] = 0;
            heap[0] = start_id;
            position[start_id] = 0;
            count = 1;
            phase = 1;
        }
        return;
    }
    while (budget-- && status == ASTAR_BUSY)
    {
        if (phase == 2)
        {
            if (path_len >= MAP_CELLS)
            {
                status = ASTAR_NO_PATH;
                return;
            }
            heap[path_len++] = trace_id; /* Reuse heap for reversed path. */
            if (trace_id == start_id)
            {
                status = ASTAR_FOUND;
                return;
            }
            unsigned d = parent[trace_id];
            trace_id = (uint16_t)((int)trace_id - dr[d] * MAP_COLS - dc[d]);
            continue;
        }
        if (!count)
        {
            status = ASTAR_NO_PATH;
            return;
        }
        uint16_t n = Pop();
        if (n == goal_id)
        {
            trace_id = n;
            phase = 2;
            continue;
        }
        int row = n / MAP_COLS, col = n % MAP_COLS;
        for (unsigned d = 0; d < 4; ++d)
        {
            int r = row + dr[d], c = col + dc[d];
            if (r < min_r || r > max_r || c < min_c || c > max_c)
                continue;
            uint16_t id = (uint16_t)(r * MAP_COLS + c);
            if (position[id] == CLOSED || !Map_CanStand((uint16_t)r, (uint16_t)c))
                continue;
            /* Verify continuous sweep, including circles between grid centres. */
            MapPoint a, b;
            Map_GridToWorld((uint16_t)row, (uint16_t)col, &a);
            Map_GridToWorld((uint16_t)r, (uint16_t)c, &b);
            if (!Map_CanSegment(a, b, MAP_MARGIN_MM))
                continue;
            uint16_t next = (uint16_t)(cost[n] + 1U);
            if (position[id] != UNSEEN && next >= cost[id])
                continue;
            cost[id] = next;
            parent[id] = (uint8_t)d;
            if (position[id] == UNSEEN)
            {
                position[id] = count;
                heap[count++] = id;
            }
            Up(position[id]);
        }
    }
}

/* 返回当前搜索状态。 */
AStarStatus AStar_Status(void)
{
    return status;
}

/* 返回找到的路径长度，未找到时为 0。 */
uint16_t AStar_PathLength(void)
{
    return status == ASTAR_FOUND ? path_len : 0;
}

/* 按索引取路径上的栅格，索引用起点侧为 0。 */
uint8_t AStar_PathCell(uint16_t i, MapCell *p)
{
    if (!p || status != ASTAR_FOUND || i >= path_len || version != Map_Revision())
        return 0;
    uint16_t n = heap[path_len - 1U - i];
    *p = (MapCell){n / MAP_COLS, n % MAP_COLS};
    return 1;
}
