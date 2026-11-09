#ifndef ARM_APP_H
#define ARM_APP_H

#include "main.h"
#include "agv_config.h"

/* 机械臂模式（AGV_MODE_ARM）：底盘一直停着，所有运动命令都不收；机械臂用调试台标定点位、
   跟随指定颜色的物料。视觉上位机接 USART2（协议同比赛模式，见 vision_link.h），一直让它找物料：
   跟随时只找那个颜色，不跟时找全部，调试台上画出来。
   串口输出（调试台按这些格式解析）：
     周期状态行后面两行，/arm 也打：
       ARM <WAIT|READY|FREE|FAULT> q=<q1>,<q2> xy=<x>,<y> goal=<q1>,<q2> move=<0|1> follow=<跟随>
           [ | waiting: <原因> | fault: <原因> (send S)]
       VIS <ON|OFF> n=<个数>[ <颜色>:<x>,<y>]...       物料按现在的姿态换算到车体系，mm
     有变化时：ARM state ...、ARM done q=.. xy=..（一次运动走完）、ARM follow <跟随>、
             ARM vision ONLINE|OFFLINE
   <跟随> = off，或 <颜色>:<looking|tracking|far>（没看到 / 跟着 / 够不着，停在最近的地方）。
   颜色名：red yellow blue green black lightblue（编号 1..6，同任务码）。 */

HAL_StatusTypeDef ArmApp_Init(void);
// 底盘任务每拍调用，在 Chassis_CycleEnd 之前。
void ArmApp_Tick(void);
void ArmApp_PrintStatus(void);
const char *ArmApp_ColorName(uint8_t color);
// 颜色名或编号 1..6，不认识返回 0。
uint8_t ArmApp_ParseColor(const char *text);

#endif
