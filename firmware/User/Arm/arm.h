#ifndef ARM_H
#define ARM_H

#include "arm_calib.h"
#include "vision_link.h"
#include "zdt_x42s.h"

/* 并联 SCARA（平面五连杆）：J1、J2 两台 ZDT X42S 闭环步进各带一根大臂（法兰轴直连，电机转多少大臂
   就转多少），两根小臂在末端铰在一起，中间关节都是被动的。两台电机和底盘在同一条 CAN 上（ZDT_Init
   由 Chassis_Init 做）。关节角、车体系、尺寸的定义见 arm_calib.h。
   单片机自己按限速和加速度规划，两台电机按距离比例同时出发同时到（关节空间走直线），每 10 ms
   给两台各发一次绝对位置（直通限速位置模式），位置、状态各 20 ms 读一次。五连杆不是每组关节角都
   合得拢；两根小臂成一条线的地方是死点（那里两台电机管不住末端，会互相顶，或者机构翻到另一种
   装配方式）：目标要合得拢，走过去的一路上离死点都要有 arm_margin。大小臂拉直或折叠只是够得着
   的边，关节直着转过去没事，两个工作区之间就这么摆过去。
   电机读数先按零点折到 ±180°，所以每个关节的行程必须在零点两边各 180° 以内（限位参数保证）。 */

#define ARM_ADDR_J1             5U // J1 电机地址，按实车接线修改
#define ARM_ADDR_J2             6U // J2
#define ARM_POINTS              16U
#define ARM_NAME_MAX            11U
#define ARM_FEEDBACK_TIMEOUT_MS 300U  // 这么久没收到位置判掉线
#define ARM_RECOVER_MS          3000U // S 清故障最多试这么久
#define ARM_FOLLOW_LOST_MS      300U  // 这么久没看到要跟的物料算丢了

typedef enum
{
    ARM_WAIT = 0, // 等两台电机使能、读到位置
    ARM_READY,    // 锁住，接受运动命令
    ARM_FREE,     // 松开了电机，可以用手掰（示教）
    ARM_FAULT,    // 堵转、掉线或掉使能，已经停了；S 清
} arm_state;

/* 运动命令的结果。 */
typedef enum
{
    ARM_OK = 0,
    ARM_BUSY,  // 没锁住，或者刚急停、刚锁住还在对齐
    ARM_LIMIT, // 关节角超限位
    ARM_REACH, // 够不着：这个位置没有合得拢、离死点够远的姿态（或者现在的姿态就合不拢：尺寸不对）
    ARM_PATH,  // 走过去要经过死点附近，换个中间点绕过去
} arm_result;

/* 等待和故障原因（arm.detail），i = 0 是 J1。 */
#define ARM_WHY_FEEDBACK(i) (0x01U << (i)) // 没有位置反馈或超时
#define ARM_WHY_ENABLE(i)   (0x04U << (i)) // 锁住时没使能
#define ARM_WHY_STALL(i)    (0x10U << (i)) // 堵转或堵转保护

/* 运行时可调的参数，console.c 里的名字是 arm_ 加字段名；开机默认值在 arm_calib.h。
   零点和方向改了以后规划从当前位置重新开始，臂不会跳；尺寸、相机下一次换算就用新值。 */
typedef struct
{
    float j1_x, j1_y, j2_x, j2_y;
    float a1, b1, a2, b2;
    float mode, margin;
    float zero1, zero2, flip1, flip2;
    float q1_min, q1_max, q2_min, q2_max;
    float speed, acc;
    float cam, cam_rot, cam_x, cam_y, cam_lag;
} arm_tuning;

#define ARM_TUNE_DEFAULTS                                                                          \
    {                                                                                              \
        .j1_x = ARM_J1_X_MM,                                                                       \
        .j1_y = ARM_J1_Y_MM,                                                                       \
        .j2_x = ARM_J2_X_MM,                                                                       \
        .j2_y = ARM_J2_Y_MM,                                                                       \
        .a1 = ARM_A1_MM,                                                                           \
        .b1 = ARM_B1_MM,                                                                           \
        .a2 = ARM_A2_MM,                                                                           \
        .b2 = ARM_B2_MM,                                                                           \
        .mode = ARM_MODE,                                                                          \
        .margin = ARM_MARGIN_DEG,                                                                  \
        .zero1 = ARM_ZERO1_DEG,                                                                    \
        .zero2 = ARM_ZERO2_DEG,                                                                    \
        .flip1 = ARM_FLIP1,                                                                        \
        .flip2 = ARM_FLIP2,                                                                        \
        .q1_min = ARM_Q1_MIN_DEG,                                                                  \
        .q1_max = ARM_Q1_MAX_DEG,                                                                  \
        .q2_min = ARM_Q2_MIN_DEG,                                                                  \
        .q2_max = ARM_Q2_MAX_DEG,                                                                  \
        .speed = ARM_SPEED_DEG_S,                                                                  \
        .acc = ARM_ACC_DEG_S2,                                                                     \
        .cam = ARM_CAM,                                                                            \
        .cam_rot = ARM_CAM_ROT_DEG,                                                                \
        .cam_x = ARM_CAM_X_MM,                                                                     \
        .cam_y = ARM_CAM_Y_MM,                                                                     \
        .cam_lag = ARM_CAM_LAG_MS,                                                                 \
    }

extern arm_tuning arm_tune;
extern const arm_tuning arm_tune_default;

typedef struct
{
    char name[ARM_NAME_MAX + 1U];
    float q[2];
} arm_point;

typedef struct
{
    uint8_t state;            // arm_state
    uint8_t detail;           // 等待/故障原因，ARM_WHY_*
    uint8_t moving;           // 规划还没走完
    float q[2];               // 实测关节角，°
    float sp[2];              // 规划到的关节角（正在下发的位置），°
    float goal[2];            // 目标关节角，°
    float x, y;               // 末端实测位置，车体系 mm；按现在的尺寸合不拢时是 NAN
    uint32_t moves;           // 走完的运动次数（跟随不算），遥测据此打印 DONE
    uint8_t follow;           // 正在跟的物料颜色（VisionColor），0 = 不跟
    uint8_t seen;             // 跟随时最近 ARM_FOLLOW_LOST_MS 内看到过
    uint8_t reach;            // 跟随时物料够得着（够不着就停在朝它走能走到的最远处）
    float follow_x, follow_y; // 最近一次看到的物料，车体系 mm
} arm_status;

extern arm_status arm;

// 在 Chassis_Init 之后调用一次：注册两台电机，载入 arm_calib.h 里的点位。
HAL_StatusTypeDef Arm_Init(void);
// 底盘任务每拍调用：在 Chassis_Update（解析 CAN）之后、Chassis_CycleEnd（发 CAN）之前。
void Arm_Tick(void);

// 关节空间走到 (q1, q2)：两台电机同时到，一路上离死点够远才接受。
arm_result Arm_MoveJoints(float q1, float q2);
// 末端走到车体系 (x, y)：逆解挑离现在最近、离死点够远的那组关节角。
arm_result Arm_MoveTo(float x, float y);
// 急停（S）：两台电机立即停，规划对齐到停下的位置，跟随取消；故障或等待时开始清故障。
void Arm_Stop(void);
// 1 = 松开电机（用手掰来示教），0 = 锁住；故障时 BUSY。
HAL_StatusTypeDef Arm_SetFree(uint8_t free);
// 要把现在的姿态算成关节角 (q1, q2)，零点该是多少（写进 arm_zero1/2 就生效，臂不动）。
// 没有反馈或正在动时 BUSY，角度超出 ±180° ERROR。
HAL_StatusTypeDef Arm_ZeroFor(float q1, float q2, float zero[2]);
// 跟随某个颜色的物料（VisionColor 1..6），0 = 停止跟随。开始跟随要 READY。
HAL_StatusTypeDef Arm_Follow(uint8_t color);
// 视觉上位机每发来一帧物料就调用一次；frame_ms 是收到的时刻。
void Arm_SeeMaterials(const VisionMaterial *materials, unsigned count, uint32_t frame_ms);

// 正解：末端在车体系的位置；这组关节角合不拢返回 0。
uint8_t Arm_Forward(float q1, float q2, float *x, float *y);
// 两个肘的位置：elbow[0..1] = J1 的肘，elbow[2..3] = J2 的肘。
void Arm_Elbows(float q1, float q2, float elbow[4]);
// 离死点还有多少度（两根小臂的夹角离 0°/180° 多远）；超限位或合不拢返回 -1。
float Arm_Clearance(float q1, float q2);
// 逆解：每根大臂两支解，四种组合里挑合得拢到 (x, y)、离死点至少 arm_margin、离 near
// 最近的；没有返回 0。
uint8_t Arm_Inverse(float x, float y, const float near[2], float q[2]);
// 关节角在限位内吗。
uint8_t Arm_InLimits(float q1, float q2);
// 相机装在小臂上时，那根小臂现在朝哪（/arm camdir 用）；相机固定在车上或者合不拢返回 0。
uint8_t Arm_CameraDirection(float *deg);
// 按现在的姿态把相机看到的偏差（物料帧的 fwd/left）换算成车体系位置，给遥测显示用；合不拢返回 0。
uint8_t Arm_CameraToBody(float fwd, float left, float *x, float *y);

// 点位表：名字 1..11 个小写字母、数字或下划线，字母开头。存的是关节角，要在限位内。
// SetPoint：新建或覆盖；名字或角度不对 ERROR，表满 BUSY。
HAL_StatusTypeDef Arm_SetPoint(const char *name, float q1, float q2);
HAL_StatusTypeDef Arm_DeletePoint(const char *name); // 没有这个点 ERROR
const arm_point *Arm_FindPoint(const char *name);
unsigned Arm_PointCount(void);
const arm_point *Arm_PointAt(unsigned index);
uint8_t Arm_ValidName(const char *name);

#endif
