#ifndef LIDAR_OBSTACLE_H
#define LIDAR_OBSTACLE_H

#include "map.h"
#include "rplidar.h"

/* 雷达障碍物识别（比赛模式）：
   每个雷达点按当前位姿换算到场地坐标 → 去掉围挡、已知静态结构和已登记障碍附近的点 →
   沿扫描顺序流式聚类 → 弦长约 Ø50 的簇外推出圆心 → 跨圈关联，连续命中 COMP_OBS_CONFIRM
   圈才确认，确认后 Map_AddObstacle 写入栅格地图（地图版本号随之递增，旧路径自动作废）。 */

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

/* 读出已确认障碍物的圆心（场地坐标）；返回个数。 */
unsigned LidarObs_List(MapPoint *out, unsigned max);

/* 自上次调用以来是否有新确认的障碍物；有则返回 1 并把其中一个写入 *p。 */
uint8_t LidarObs_PopNew(MapPoint *p);

#endif
