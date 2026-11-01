#ifndef USER_MATH_H
#define USER_MATH_H

#include <math.h>

#define MATH_PI         3.14159265f
#define MATH_DEG_TO_RAD 0.01745329252f
#define MATH_RAD_TO_DEG 57.2957795f

/* 角度归一化到 [-180, 180]；恰好 ±180 时保留符号，便于指定转向方向。 */
static inline float Math_WrapDeg(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
        angle -= 360.0f;
    if (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static inline float Math_Clamp(float x, float limit)
{
    if (x > limit)
        return limit;
    if (x < -limit)
        return -limit;
    return x;
}

#endif
