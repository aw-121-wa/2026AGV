#ifndef TASK_CODE_H
#define TASK_CODE_H

#include <stdint.h>

/* 比赛任务码：四组三位数，以 + 连接，例如 156+123+516+231。
 *   第 1 组  第一批三个物料的颜色，按搬运顺序
 *   第 2 组  第一批在粗加工区和暂存区放的环号
 *   第 3 组  第二批三个物料的颜色，按搬运顺序
 *   第 4 组  第二批在粗加工区放的环号
 * 第二批在暂存区码垛到第一批同色物料上，所以暂存区环号不在码里，解析时推出来。
 * 颜色数字就是规则的编号，与 vision_link.h 的 VisionColor 相同。
 * 纯软件模块，不依赖 HAL，扫码模块和相机通道共用。 */

#define TASK_CODE_TEXT_LEN 15U // "156+123+516+231"
#define TASK_LINE_MAX      63U

typedef enum
{
    TASK_CODE_OK = 0,
    TASK_CODE_ERR_FORMAT, // 找不到完整的 ddd+ddd+ddd+ddd
    TASK_CODE_ERR_COLOR,  // 颜色不是 1~6，或同一批有重复
    TASK_CODE_ERR_RING,   // 环号不是 1、2、3 各一次
    TASK_CODE_ERR_BATCH,  // 第二批颜色与第一批不同，暂存区无法码垛
} TaskCodeResult;

/* 下标：[批次 0/1][搬运顺序 0/1/2]。 */
typedef struct
{
    char text[TASK_CODE_TEXT_LEN + 1U]; // 只含任务码本身，用于显示
    uint8_t color[2][3];
    uint8_t rough_ring[2][3];
    uint8_t storage_ring[2][3];
} TaskCode;

/* 在 text 的前 len 个字节里找任务码并校验，前后可以带模块加的前后缀。
   只有返回 TASK_CODE_OK 时 *out 才有效。 */
TaskCodeResult TaskCode_Parse(const char *text, unsigned len, TaskCode *out);

/* 把扫码模块的字节流切成行。text 始终以 NUL 结尾，不可打印的字节存成 '?'。 */
typedef struct
{
    char text[TASK_LINE_MAX + 1U];
    uint8_t len;
    uint32_t last_ms;
} TaskLine;

/* 喂一个字节。返回 1 表示 text 里是完整的一行（遇到 CR/LF，或缓冲写满），
   调用方处理后把 len 清零。空行不返回 1。 */
uint8_t TaskLine_Feed(TaskLine *line, uint8_t byte, uint32_t now_ms);

/* 没有行尾符的模块靠空闲分行：缓冲非空且 idle_ms 内没有新字节时返回 1，处理方式同上。 */
uint8_t TaskLine_Idle(const TaskLine *line, uint32_t now_ms, uint32_t idle_ms);

#endif
