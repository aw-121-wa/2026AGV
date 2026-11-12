#ifndef TUNE_LOG_H
#define TUNE_LOG_H

#include "main.h"
#include <stdint.h>

/* 调参日志，给调试台的自动调参用（tools/pid_tune.js）。
   - 底盘每做一个动作（路径、原地转、定距），每拍（5 ms）记一个样本，动作结束后再记 0.4 s，
     看车停没停稳。动作间隔不到 0.4 s 就接着记成一段。最多记 10 s，再长的只留前 10 s。
   - /log 发最近一段；/log on 以后每段记完自动发。发送缓冲过半就等，一拍最多排 4 行：
     不挤掉状态行和命令回执，也不占控制节拍。发的时候又开始动，这段作废（LOG abort）。
   - 格式：
       LOG begin n=<样本数> mode=<path|turn|move> end=<done|stop|fault> trunc=<0|1> imu=<0|1>
       LOG param <名字>=<值> ...（三行，动作开始时的航向环参数，名字和 /set 的一样；
                                 max_wheel_rpm 是编译进去的轮速上限 CHASSIS_MAX_WHEEL_RPM）
       L <序号> <模式> <平移> <目标航向> <航向> <航向环输出> <斜坡后下发> <vx> <vy>
       LOG end n=<样本数>
     模式是 chassis_mode 的值（0 停、1 定距、2 原地转、3 路径）；平移 = 1 时目标车速不为零，
     航向环没有到位死区；航向单位 0.001°，角速度 0.01 deg/s；vx/vy 是斜坡后下发的车体速度，mm/s。
     “斜坡后下发”是航向环自己的输出，不含平移前馈。前馈用参数行里的值和这一行的 vx、vy 算：
     (heading_ff_x·vx + heading_ff_y·vy) / 1000。 */

#define TUNE_LOG_CAPACITY 2000U // 10 s
#define TUNE_LOG_TAIL     80U   // 动作结束后再记 0.4 s

/* 底盘任务每拍在 Chassis_Update 之后调用一次：记样本、发日志。 */
void TuneLog_Tick(void);
/* 每段记完自动发。 */
void TuneLog_SetAuto(uint8_t on);
/* 发最近一段：HAL_OK 开始发；还在记返回 HAL_BUSY，记完就发；一段都没有返回 HAL_ERROR。 */
HAL_StatusTypeDef TuneLog_Send(void);

#endif
