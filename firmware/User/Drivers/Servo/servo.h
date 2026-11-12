#ifndef SERVO_H
#define SERVO_H

/* 一路舵机：TIM3_CH1 从 PA6 出 50 Hz 脉宽信号，比赛模式用来伸缩雷达。脉宽和角度的对应见
   comp_config.h 的 COMP_SERVO_*。舵机没有位置反馈，转到位要多久由调用方自己等。 */
void Servo_Init(void);

/* 转到 deg（0 ~ COMP_SERVO_RANGE_DEG，超出的截到两头），下一个 20 ms 周期起生效。 */
void Servo_SetDeg(float deg);

#endif
