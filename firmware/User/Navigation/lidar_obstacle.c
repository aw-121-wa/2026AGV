#include "lidar_obstacle.h"
#include "comp_config.h"
#include "user_math.h"
#include <math.h>
#include <string.h>

typedef struct
{
    float x, y;
    uint32_t last_scan;
    uint8_t hits, used, confirmed, reported;
} Track;

typedef struct
{
    float fx, fy, lx, ly; // 首点、末点
    float sx, sy;         // 坐标和
    float ox, oy;         // 雷达光心
    unsigned n;
} Cluster;

#define LIDAR_TUNE_DEFAULTS                                                                        \
    {                                                                                              \
        .lidar_x = COMP_LIDAR_X_MM, .lidar_y = COMP_LIDAR_Y_MM, .lidar_yaw = COMP_LIDAR_YAW_DEG,   \
        .lidar_min = COMP_LIDAR_MIN_MM, .lidar_max = COMP_LIDAR_MAX_MM,                            \
        .obs_gap = COMP_OBS_GAP_MM, .obs_width = COMP_OBS_MAX_WIDTH_MM,                            \
        .obs_match = COMP_OBS_MATCH_MM, .obs_confirm = (float)COMP_OBS_CONFIRM,                    \
    }

LidarObsStats lidar_obs;
LidarTuning lidar_tune = LIDAR_TUNE_DEFAULTS;
const LidarTuning lidar_tune_default = LIDAR_TUNE_DEFAULTS;
static Track tracks[COMP_OBS_TRACKS];
static Cluster cl;

void LidarObs_Init(void)
{
    memset(tracks, 0, sizeof(tracks));
    memset(&cl, 0, sizeof(cl));
    memset(&lidar_obs, 0, sizeof(lidar_obs));
}

static uint8_t FreeAt(float x, float y)
{
    uint16_t r, c;
    return Map_WorldToGrid((int32_t)x, (int32_t)y, &r, &c) && Map_IsFree(r, c);
}

/* 只有场内、远离已知结构的点才可能是新障碍。 */
static uint8_t Interesting(float x, float y)
{
    const float w = COMP_OBS_WALL_MM, s = COMP_OBS_STATIC_MM;
    if (x < w || y < w || x > MAP_WIDTH_MM - w || y > MAP_HEIGHT_MM - w)
        return 0;
    /* 3×3 采样（含对角），矩形结构的角点附近也能挡住。 */
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            if (!FreeAt(x + i * s, y + j * s))
                return 0;
    return 1;
}

static void Track_Update(float x, float y)
{
    Track *best = 0;
    float best_d = lidar_tune.obs_match;
    for (unsigned i = 0; i < COMP_OBS_TRACKS; ++i)
    {
        if (!tracks[i].used)
            continue;
        float d = hypotf(tracks[i].x - x, tracks[i].y - y);
        if (d < best_d)
        {
            best_d = d;
            best = &tracks[i];
        }
    }
    if (!best)
    {
        /* 新目标：占空槽；没有空槽就顶掉最久没见的未确认目标。 */
        for (unsigned i = 0; i < COMP_OBS_TRACKS; ++i)
        {
            Track *t = &tracks[i];
            if (!t->used)
            {
                best = t;
                break;
            }
            if (!t->confirmed && (!best || t->last_scan < best->last_scan))
                best = t;
        }
        if (!best)
            return;
        *best = (Track){x, y, lidar_obs.scans, 1, 1, 0, 0};
        return;
    }
    if (best->confirmed)
        return;
    /* 滑动平均，最多按 8 次平均，保证后面的测量还能修正位置。 */
    float k = 1.0f / (float)(best->hits < 8U ? best->hits + 1U : 8U);
    best->x += (x - best->x) * k;
    best->y += (y - best->y) * k;
    if (best->last_scan != lidar_obs.scans) // 同一圈被拆成两簇只算一次
    {
        ++best->hits;
        best->last_scan = lidar_obs.scans;
    }
    if (best->hits >= lidar_tune.obs_confirm)
    {
        best->confirmed = 1;
        ++lidar_obs.confirmed;
        if (!Map_AddObstacle((int32_t)lroundf(best->x), (int32_t)lroundf(best->y),
                             COMP_OBS_MAP_RADIUS))
            ++lidar_obs.map_full;
    }
}

static void Cluster_Close(void)
{
    if (cl.n >= 2U)
    {
        ++lidar_obs.clusters;
        float width = hypotf(cl.lx - cl.fx, cl.ly - cl.fy);
        if (width > lidar_tune.obs_width)
            ++lidar_obs.rejected_width;
        else
        {
            /* 雷达只看到朝向自己的半圈，弧上点的重心离圆心约 2r/π，沿视线向外补回。 */
            float cx = cl.sx / cl.n, cy = cl.sy / cl.n;
            float dx = cx - cl.ox, dy = cy - cl.oy, d = hypotf(dx, dy);
            if (d > 1.0f)
            {
                float push = COMP_OBS_RADIUS_MM * (2.0f / MATH_PI);
                Track_Update(cx + dx / d * push, cy + dy / d * push);
            }
        }
    }
    cl.n = 0;
}

static void EndScan(void)
{
    ++lidar_obs.scans;
    for (unsigned i = 0; i < COMP_OBS_TRACKS; ++i)
        if (tracks[i].used && !tracks[i].confirmed &&
            lidar_obs.scans - tracks[i].last_scan > COMP_OBS_FORGET)
            tracks[i].used = 0;
}

void LidarObs_Feed(const RPLidarSample *s, const LidarPose *pose)
{
    ++lidar_obs.points;
    if (s->start)
    {
        Cluster_Close();
        EndScan();
    }
    if (s->dist_mm < lidar_tune.lidar_min || s->dist_mm > lidar_tune.lidar_max)
    {
        Cluster_Close();
        return;
    }
    /* 雷达系（x 前，y 左）：雷达角度顺时针为正，取负变成逆时针。 */
    float a = (-s->angle_deg + lidar_tune.lidar_yaw) * MATH_DEG_TO_RAD;
    float bx = lidar_tune.lidar_x + s->dist_mm * cosf(a);
    float by = lidar_tune.lidar_y + s->dist_mm * sinf(a);
    float h = pose->heading_deg * MATH_DEG_TO_RAD, ch = cosf(h), sh = sinf(h);
    float x = pose->x + ch * bx - sh * by, y = pose->y + sh * bx + ch * by;
    if (!Interesting(x, y))
    {
        Cluster_Close();
        return;
    }
    ++lidar_obs.used;
    if (cl.n && hypotf(x - cl.lx, y - cl.ly) > lidar_tune.obs_gap)
        Cluster_Close();
    if (!cl.n)
    {
        cl.fx = x;
        cl.fy = y;
        cl.sx = cl.sy = 0.0f;
        cl.ox = pose->x + ch * lidar_tune.lidar_x - sh * lidar_tune.lidar_y;
        cl.oy = pose->y + sh * lidar_tune.lidar_x + ch * lidar_tune.lidar_y;
    }
    cl.lx = x;
    cl.ly = y;
    cl.sx += x;
    cl.sy += y;
    ++cl.n;
}

unsigned LidarObs_List(MapPoint *out, unsigned max)
{
    unsigned n = 0;
    for (unsigned i = 0; i < COMP_OBS_TRACKS && n < max; ++i)
        if (tracks[i].used && tracks[i].confirmed)
            out[n++] = (MapPoint){(int32_t)lroundf(tracks[i].x), (int32_t)lroundf(tracks[i].y)};
    return n;
}

uint8_t LidarObs_PopNew(MapPoint *p)
{
    for (unsigned i = 0; i < COMP_OBS_TRACKS; ++i)
        if (tracks[i].used && tracks[i].confirmed && !tracks[i].reported)
        {
            tracks[i].reported = 1;
            *p = (MapPoint){(int32_t)lroundf(tracks[i].x), (int32_t)lroundf(tracks[i].y)};
            return 1;
        }
    return 0;
}
