#ifndef COMPETITION_H
#define COMPETITION_H

#include "main.h"
#include "agv_config.h"
#include "lidar_obstacle.h"
#include "task_code.h"
#include "vision_link.h"
#include <stdint.h>

/* 比赛模式（AGV_MODE_COMPETITION）运行框架：
   - 开局流程（Comp_Start）：启停区扫描 → 去二维码板前，再扫一次并读任务码 → 去场地中央，再扫一次。
     每段路都从刚扫描过的位置走一条直线：只要看到的障碍物不挡这条线，被挡住看不到的也不可能在
     这条线上（它和扫描点的连线落在车身扫过的矩形里，挡住它的东西也就在矩形里，会被看到）。
     路被挡住要拐弯时，在拐点停下多扫一次。之后的取料、放料还没有，等机械臂；
   - USART1 驱动 RPLIDAR C1：只在停车扫描时读数据，开动时接收中断关着，不占 CPU。
     全部扫完输出结果，关掉雷达，之后不再处理雷达数据；
   - USART3 接二维码扫码模块读任务码，相机是备用通道（TASK 帧），两路共用一套解析校验。
     第一个合法的码输出到控制台后就定下来，扫码口关掉，不再读码；
   - USART2 与视觉上位机通信：下发位姿 / 视觉模式 / 障碍物，接收物料和圆环偏差；
   - 维护场地坐标位姿（与跑图模式相同约定：上电时车头朝场地 +X，原点为出发区中心）；
   - /route 用当前地图规划路线，给调试台显示，也可以照着走。
   只在比赛模式下由 main.c / freertos.c 调用；另外两个模式完全不涉及。 */

typedef enum
{
    COMP_LIDAR_BOOT = 0, // 上电等待
    COMP_LIDAR_STOPPING, // 已发 STOP，等雷达停流
    COMP_LIDAR_START,    // 已发 SCAN，等应答
    COMP_LIDAR_RUN,      // 正在读点
    COMP_LIDAR_PAUSED,   // 雷达在转，但接收关着，不占 CPU；要扫描时立刻能接着读
    COMP_LIDAR_DONE,     // 全部扫完了，雷达已关
    COMP_LIDAR_FAILED,   // 超时没扫完，已放弃，雷达已关
} CompLidarState;

typedef enum
{
    COMP_RUN_IDLE = 0, // 等启动
    COMP_RUN_SCAN,     // 停着扫描
    COMP_RUN_PLAN,     // 规划去下一个停车点的路线
    COMP_RUN_MOVE,     // 走路线的第一段直线
    COMP_RUN_WAIT,     // 在二维码板前等任务码
    COMP_RUN_DONE,     // 开局完成
    COMP_RUN_STOPPED,  // 被 S 停下
    COMP_RUN_FAILED,   // 去不了二维码板、读不到任务码，或底盘故障
} CompRunState;

typedef enum
{
    COMP_ROUTE_NONE = 0,
    COMP_ROUTE_PLANNING,  // A* 分拍搜索中
    COMP_ROUTE_READY,     // 规划好了，等 /route go
    COMP_ROUTE_FOLLOWING, // 正在走
    COMP_ROUTE_DONE,      // 走到了
    COMP_ROUTE_STOPPED,   // 没走到就停了：S、底盘故障、偏线或路被挡
    COMP_ROUTE_FAILED,    // 规划失败或开不走
} CompRouteState;

typedef struct
{
    uint8_t anchored;        // 位姿基准已建立（底盘 READY 后）
    float x, y, heading_deg; // 车体中心场地坐标 mm，航向 deg（相对场地 +X，逆时针正）
    CompLidarState lidar_state;
    uint32_t lidar_restarts;
    float lidar_hz;          // 实测扫描频率
    uint8_t obstacles;       // 扫描结果：写入地图的障碍物数
    uint8_t vision_online;
    VisionMode vision_mode;
    uint8_t vision_color;
    uint8_t task_valid;     // task 里是这一局的任务码，已经定下来了
    TaskCode task;
    uint32_t task_rejected; // 读到但不合法的次数
    CompRouteState route_state;
    CompRunState run_state;
} CompStatus;

extern CompStatus comp;

/* 调度器启动前调用：建图、初始化三路串口。 */
HAL_StatusTypeDef Comp_Init(unsigned zone);

/* 底盘任务每周期（5 ms）调用一次。 */
void Comp_Tick(void);

/* 切换视觉任务，立即下发给上位机。color 为 VisionColor，0 = 任意。 */
void Comp_SetVisionMode(VisionMode mode, uint8_t color);

/* 开局流程。现在由 /start 触发，以后接启动按钮（规则要求一键启动）。
   底盘没 READY、位姿没定时返回 HAL_BUSY。 */
HAL_StatusTypeDef Comp_Start(void);
/* 流程在跑吗。在跑时不接受任何改变车和参数的命令：规则禁止运行中遥控，只有 S 能停。 */
uint8_t Comp_Running(void);
/* S：中止流程和正在进行的扫描，车由调用方停。 */
void Comp_Stop(void);
/* 再打印一次流程状态。 */
void Comp_RunReport(void);

/* 台架调试入口（串口控制台 /task /scan /pose /route），都只在底盘任务里调用。 */
/* 把文本当成扫到的任务码处理，合法的码会替换已有的；text 以 NUL 结尾。 */
void Comp_InjectTask(const char *text, unsigned len);
/* 再打印一次当前任务码（COMP task ... / COMP task none）。 */
void Comp_TaskReport(void);
/* 忘掉任务码，重新打开扫码口。 */
void Comp_ClearTask(void);
/* 清掉障碍物，在车现在的位置扫一次，扫完就关雷达。重建地图要十几毫秒，只在停车时调用。
   位姿没定时返回 HAL_BUSY。 */
HAL_StatusTypeDef Comp_Rescan(void);
/* 把当前位置和朝向设为给定的场地位姿，之后按里程计累加。底盘没 READY 时返回 HAL_BUSY。 */
HAL_StatusTypeDef Comp_SetPose(float x_mm, float y_mm, float heading_deg);
/* 从当前位姿规划到 (x, y)，A* 分拍搜索，结果用 ROUTE 行报告。底盘没 READY、位姿没定时返回 HAL_BUSY。
   只在停车时调用。 */
HAL_StatusTypeDef Comp_RoutePlan(int32_t x_mm, int32_t y_mm);
/* 照规划好的路线走，速度用 cmd_speed。没有规划好的路线时返回 HAL_ERROR，底盘没 READY 时返回
   HAL_BUSY，其余结果用 ROUTE 行报告。只在停车时调用。 */
HAL_StatusTypeDef Comp_RouteGo(void);
/* 再打印一次路线状态和路线点。 */
void Comp_RouteReport(void);

#endif
