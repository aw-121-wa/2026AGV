#ifndef LIDAR_OBSTACLE_H
#define LIDAR_OBSTACLE_H

#include "map.h"
#include "rplidar.h"

/* 雷达障碍物识别（比赛模式）。开局在几个位置各停下来扫一次，之后不再处理雷达数据：
   每个雷达点按位姿换算到场地坐标 → 去掉围挡和转盘上的点 → 沿扫描顺序流式聚类 → 簇外推出圆心 →
   跨圈关联并平均。每次扫完调用 LidarObs_Finish，看到的圈数够多的目标写入栅格地图，写入后位置
   不再变，和地图保持一致；后面的位置新看到的再追加。 */

typedef struct
{
    float x, y;        // 车体中心在场地坐标，mm
    float heading_deg; // 车头相对场地 +X 的逆时针角
} LidarPose;

typedef struct
{
    uint32_t points, used, clusters, rejected_width, scans, map_full;
    uint8_t confirmed;
} LidarObsStats;

extern LidarObsStats lidar_obs;

/* 运行时可调的参数，字段名就是串口控制台里的参数名（console.c），默认值见 comp_config.h。 */
typedef struct
{
    float lidar_x, lidar_y, lidar_yaw, lidar_min, lidar_max;
    float obs_gap, obs_width, obs_match, obs_confirm;
} LidarTuning;

extern LidarTuning lidar_tune;
extern const LidarTuning lidar_tune_default;

void LidarObs_Init(void);

/* 喂一个雷达采样；pose 为采样时刻的车体位姿。 */
void LidarObs_Feed(const RPLidarSample *s, const LidarPose *pose);

/* 一次扫描结束：把看到足够圈数的目标写入地图（地图版本号随之递增），返回累计的障碍物个数。 */
unsigned LidarObs_Finish(void);

/* 读出 LidarObs_Finish 写入地图的障碍物圆心（场地坐标）；返回个数。 */
unsigned LidarObs_List(MapPoint *out, unsigned max);

/* 还有没取走的新障碍物吗？有则返回 1 并把其中一个写入 *p，每个只给一次。 */
uint8_t LidarObs_PopNew(MapPoint *p);

#endif
