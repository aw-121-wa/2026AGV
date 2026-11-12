#ifndef ARM_CALIB_H
#define ARM_CALIB_H

/* 机械臂的开机默认值：调试台“机械臂”页的“导出 arm_calib.h”生成整个文件，覆盖这里再编译，
   不接调试台（比赛）时用的就是这些。现场先用 /set arm_* 和 /arm 命令调，调好了再导出；手改也行。
   并联 SCARA（平面五连杆）：J1、J2 两台电机各带一根大臂，两根小臂在末端（夹爪）铰在一起，中间的
   关节都是被动的。关节角 q1、q2 = 两根大臂相对车头的角度，俯视逆时针为正，度。
   车体系：x 朝车头、y 朝左，原点是底盘中心，mm。两台电机都在车身中线上，连线和车头方向平行，
   这里 J1 在后、J2 在前。尺寸：电机间距 200 是实物的，大臂 140 是设计俯视图上标的，小臂 235 是按
   图的比例量的，图的右边当车头；图上那个姿态（两根大臂都垂直于中线朝车左）是 q = (90, 90)，末端
   在中线右边 73 mm。小臂实物量准了改。
   初始位置就是图上那个姿态 q = (90, 90)：上电前把臂摆成这样（收在出发前的 300 mm 见方里）。
   驱动器的位置读数从上电那一刻的 0 开始算，所以零点默认 -90：读数 0 就是大臂朝车左 90°，上电不用
   再标；只复位单片机、驱动器没断电，读数还在，角度也还对。电机装反了（FLIP = 1）零点是 +90，
   调试台“设为零点”会按方向算好。 */

#define ARM_J1_X_MM     -100.0f // J1 电机轴在车体系的位置
#define ARM_J1_Y_MM     0.0f
#define ARM_J2_X_MM     100.0f // J2 电机轴
#define ARM_J2_Y_MM     0.0f
#define ARM_A1_MM       140.0f // J1 的大臂：电机轴到肘
#define ARM_B1_MM       235.0f // J1 那边的小臂：肘到末端
#define ARM_A2_MM       140.0f
#define ARM_B2_MM       235.0f
#define ARM_MODE        1 // 上电时末端在哪一侧：从 J1 的肘看向 J2 的肘，末端在左边是 0、右边是 1
#define ARM_MARGIN_DEG  10.0f // 离死点（两根小臂成一条线）至少这么多度
#define ARM_ZERO1_DEG   -90.0f // 关节角为 0 时电机的读数（/arm zero 标定）；-90 = 上电时臂在初始位置
#define ARM_ZERO2_DEG   -90.0f
#define ARM_FLIP1       0 // 1 = 电机正转时大臂顺时针转
#define ARM_FLIP2       0
#define ARM_Q1_MIN_DEG  -170.0f // 关节限位
#define ARM_Q1_MAX_DEG  170.0f
#define ARM_Q2_MIN_DEG  -170.0f
#define ARM_Q2_MAX_DEG  170.0f
#define ARM_SPEED_DEG_S 180.0f // 关节最大角速度
#define ARM_ACC_DEG_S2  900.0f // 关节角加速度
#define ARM_SMOOTH_MS   40.0f // 下发的位置按最近这么多毫秒平均：起步、停下更柔，0 = 不平滑
#define ARM_SWING_MM_S  400.0f // 过死点时末端的速度：慢了会停在死点上
#define ARM_SWING_RUN_MM 100.0f // 过死点：从末端离两个肘的连线这么远助跑，过去以后也停在这么远
#define ARM_CAM         1    // 相机：0 = 固定在车上，1 = 装在 J1 那根小臂上，2 = 装在 J2 那根小臂上
#define ARM_CAM_ROT_DEG 0.0f // 相机正向：装在小臂上时那根小臂朝哪画面上方是车头（README 13.4）
#define ARM_CAM_X_MM    0.0f // 相机固定在车上时，画面 TARGET_PX 那一点在车体系的位置
#define ARM_CAM_Y_MM    -200.0f
#define ARM_CAM_LAG_MS  60.0f // 拍照到单片机收到物料帧的延迟

/* 命名点位（关节角）：ARM_POINT(名字, q1, q2, 侧)，一行一个、行尾加反斜杠；名字用小写字母、数字、
   下划线，字母开头，最长 11 个字。侧 = 记这个点时末端在两个肘连线的哪一侧（同 ARM_MODE），去另一侧
   的点固件会先过死点。 */
#define ARM_POINT_TABLE

#endif
