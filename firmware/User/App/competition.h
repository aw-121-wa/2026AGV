#ifndef COMPETITION_H
#define COMPETITION_H

#include "main.h"
#include "agv_config.h"
#include "lidar_obstacle.h"
#include "task_code.h"
#include "vision_link.h"
#include <stdint.h>

/* 比赛模式（AGV_MODE_COMPETITION）运行框架：
   - USART1 驱动 RPLIDAR C1，实时识别黑色障碍物并写入栅格地图；
   - USART2 与视觉上位机通信：下发位姿 / 视觉模式 / 障碍物，接收物料和圆环偏差；
   - USART3 接二维码扫码模块读任务码，相机是备用通道（TASK 帧），两路共用一套解析校验；
   - 维护场地坐标位姿（与跑图模式相同约定：上电时车头朝场地 +X，原点为出发区中心）。
   只在比赛模式下由 main.c / freertos.c 调用；另外两个模式完全不涉及。
   后续比赛流程（取料 / 放料状态机）直接读 comp、vision、地图，并用 Comp_SetVisionMode 切换视觉任务。 */

typedef enum
{
    COMP_LIDAR_BOOT = 0, // 上电等待
    COMP_LIDAR_STOPPING, // 已发 STOP，等雷达停流
    COMP_LIDAR_START,    // 已发 SCAN，等应答
    COMP_LIDAR_RUN,      // 正在出点
} CompLidarState;

typedef struct
{
    uint8_t anchored;        // 位姿基准已建立（底盘 READY 后）
    float x, y, heading_deg; // 车体中心场地坐标 mm，航向 deg（相对场地 +X，逆时针正）
    CompLidarState lidar_state;
    uint32_t lidar_restarts;
    float lidar_hz;          // 实测扫描频率
    uint8_t obstacles;       // 已确认并写入地图的障碍物数
    uint8_t vision_online;
    VisionMode vision_mode;
    uint8_t vision_color;
    uint8_t task_valid;     // task 里是读到过的合法任务码
    TaskCode task;          // 最新的合法任务码；读到新的合法码就更新
    uint32_t task_rejected; // 读到但不合法的次数
} CompStatus;

extern CompStatus comp;

/* 调度器启动前调用：建图、初始化三路串口。 */
HAL_StatusTypeDef Comp_Init(unsigned zone);

/* 底盘任务每周期（5 ms）调用一次。 */
void Comp_Tick(void);

/* 切换视觉任务，立即下发给上位机。color 为 VisionColor，0 = 任意。 */
void Comp_SetVisionMode(VisionMode mode, uint8_t color);

/* 台架调试入口（串口控制台 /task /obs /pose），都只在底盘任务里调用。 */
/* 把文本当成扫到的任务码处理；text 以 NUL 结尾。 */
void Comp_InjectTask(const char *text, unsigned len);
/* 清掉已识别的障碍物并重建地图，要几毫秒，只在停车时调用。 */
void Comp_ClearObstacles(void);
/* 把当前位置和朝向设为给定的场地位姿，之后按里程计累加。底盘没 READY 时返回 HAL_BUSY。 */
HAL_StatusTypeDef Comp_SetPose(float x_mm, float y_mm, float heading_deg);

#endif
