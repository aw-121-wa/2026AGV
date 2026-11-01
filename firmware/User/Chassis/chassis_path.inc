/* 连续路径控制与底盘共享反馈和限幅状态，仅由 chassis.c 引入。 */
#define PATH_INTERNAL_CAPACITY (CHASSIS_PATH_CAPACITY * 10U)
#define REFERENCE_CAPACITY     (CHASSIS_PATH_CAPACITY + 1U)
static chassis_waypoint s_path[PATH_INTERNAL_CAPACITY];
static float s_path_limit[PATH_INTERNAL_CAPACITY];
static unsigned s_path_count, s_path_index;
static chassis_waypoint s_reference[REFERENCE_CAPACITY];
static unsigned s_reference_count, s_reference_cursor;
static uint8_t s_reference_truncated;
static chassis_waypoint s_path_start;
static uint8_t s_path_stop_each;
static uint32_t s_path_started, s_path_segment_started;
static float s_path_last_x, s_path_last_y;
chassis_path_stats chassis_path_result;

/* 校验并追加有限航点，容量不足时保持原路径。 */
HAL_StatusTypeDef Chassis_AppendPath(const chassis_waypoint *points, unsigned count)
{
    if (!points || !count || count > CHASSIS_PATH_CAPACITY ||
        count * 10U > PATH_INTERNAL_CAPACITY - (s_path_count - s_path_index))
        return HAL_ERROR;
    for (unsigned i = 0; i < count; ++i)
        if (!isfinite(points[i].x) || !isfinite(points[i].y) || !isfinite(points[i].heading))
            return HAL_ERROR;
    /* 参考折线只服务偏线统计：容量满时先丢已经走过的航点，累计最大偏线已经记在结果里。
       指令模式可以一直追加，不会因为整条路径累计航点数超过容量而永久报错。 */
    if (s_reference_count + count > REFERENCE_CAPACITY)
    {
        unsigned drop = s_reference_count + count - REFERENCE_CAPACITY;
        if (drop > s_reference_cursor)
            s_reference_truncated = 1U; /* 当前段被挤掉，偏线改量当前弦 */
        if (drop > s_reference_count)
            drop = s_reference_count; /* 防御：正常不会发生 */
        memmove(s_reference, s_reference + drop, (s_reference_count - drop) * sizeof(*s_reference));
        s_reference_count -= drop;
        s_reference_cursor = s_reference_cursor > drop ? s_reference_cursor - drop : 0U;
    }
    memcpy(s_reference + s_reference_count, points, count * sizeof(*points));
    s_reference_count += count;
    if (s_path_index)
    {
        memmove(s_path, s_path + s_path_index, (s_path_count - s_path_index) * sizeof(*s_path));
        memmove(s_path_limit, s_path_limit + s_path_index,
                (s_path_count - s_path_index) * sizeof(*s_path_limit));
        s_path_count -= s_path_index;
        s_path_index = 0;
    }
    for (unsigned i = 0; i < count; ++i)
    {
        chassis_waypoint a =
            i ? points[i - 1] : (s_path_count ? s_path[s_path_count - 1] : s_path_start);
        chassis_waypoint b = points[i];
        float ux = b.x - a.x, uy = b.y - a.y, ul = hypotf(ux, uy);
        if (!s_path_stop_each && i + 1 < count && ul > 0.001f)
        {
            chassis_waypoint c = points[i + 1];
            float vx = c.x - b.x, vy = c.y - b.y, vl = hypotf(vx, vy);
            if (vl > 0.001f)
            {
                ux /= ul;
                uy /= ul;
                vx /= vl;
                vy /= vl;
                float dot = ux * vx + uy * vy;
                if (dot > 0.99999f && fabsf(Math_WrapDeg(b.heading - a.heading)) < 0.001f &&
                    fabsf(Math_WrapDeg(c.heading - b.heading)) < 0.001f)
                    continue;
                if (dot > -0.99f && dot < 0.99999f)
                {
                    /* 二次贝塞尔控制多边形在原拐角两侧各裁不超过 2 倍容差；
                       曲线到两条原线段的最小距离不超过裁切距离的一半。 */
                    float trim = fminf(2 * CHASSIS_PATH_DEVIATION_MM, fminf(ul, vl) * 0.25f);
                    float sh = a.heading + Math_WrapDeg(b.heading - a.heading) * (1 - trim / ul);
                    float eh = b.heading + Math_WrapDeg(c.heading - b.heading) * (trim / vl);
                    float radius = trim * powf(fmaxf(0.001f, (1 + dot) * 0.5f), 1.5f) /
                                   sqrtf(fmaxf(0.001f, (1 - dot) * 0.5f));
                    float cap =
                        fminf(s_move_speed_limit, sqrtf(CHASSIS_ACC_LINEAR * 0.25f * radius));
                    for (unsigned j = 0; j <= 8; ++j)
                    {
                        float t = j / 8.0f, r = 1 - t;
                        s_path[s_path_count] =
                            (chassis_waypoint){b.x - r * r * ux * trim + t * t * vx * trim,
                                               b.y - r * r * uy * trim + t * t * vy * trim,
                                               sh + Math_WrapDeg(eh - sh) * t};
                        s_path_limit[s_path_count++] = cap;
                    }
                    continue;
                }
            }
        }
        s_path[s_path_count] = b;
        s_path_limit[s_path_count++] = s_move_speed_limit;
    }
    return HAL_OK;
}

/* 从当前位置启动一条路径，后续追加不会重置速度。 */
HAL_StatusTypeDef Chassis_FollowPath(const chassis_waypoint *points, unsigned count,
                                     uint8_t stop_each)
{
    if (chassis.state != CHASSIS_STATE_READY || chassis.mode != CHASSIS_MODE_VEL ||
        !Chassis_FeedbackReady())
        return HAL_BUSY;
    s_path_count = 0;
    s_path_index = 0;
    s_path_stop_each = stop_each;
    s_path_start = (chassis_waypoint){chassis.pos_x, chassis.pos_y, chassis.yaw};
    s_reference[0] = s_path_start;
    s_reference_count = 1;
    s_reference_cursor = 0U;
    s_reference_truncated = 0U;
    if (Chassis_AppendPath(points, count) != HAL_OK)
        return HAL_ERROR;
    s_path_index = 0;
    s_path_stop_each = stop_each;
    s_path_start = (chassis_waypoint){chassis.pos_x, chassis.pos_y, chassis.yaw};
    s_path_last_x = chassis.pos_x;
    s_path_last_y = chassis.pos_y;
    s_path_started = s_path_segment_started = HAL_GetTick();
    memset(&chassis_path_result, 0, sizeof(chassis_path_result));
    /* 航向动态指标跟随本次路径重新统计，$H 在 PATH 模式下才有意义。 */
    chassis.heading_peak_dynamic = 0.0f;
    chassis.heading_over_limit = 0U;
    s_heading_integral = 0;
    chassis.mode = CHASSIS_MODE_PATH;
    return HAL_OK;
}

/* 以剩余距离限制刹车速度，并在航点处保持连续速度与航向。 */
static void Chassis_PathControl(float dt)
{
    /* 预测只用于控制，编码器里程计保持原始累计，避免新反馈到来时重复积分。 */
    float prediction[4] = {0};
    uint32_t now = HAL_GetTick();
    for (unsigned i = 0; i < 4; ++i)
    {
        float age = fminf((float)(now - s_odom_sample_ms[i]), 40.0f) * 0.001f;
        prediction[i] = Chassis_WheelRpm(i) * (MATH_PI * CHASSIS_WHEEL_DIA_MM) /
                        (60 * CHASSIS_GEAR_RATIO) * age;
    }
    float bx = (prediction[0] + prediction[1] + prediction[2] + prediction[3]) * 0.25f;
    float by = (-prediction[0] + prediction[1] + prediction[2] - prediction[3]) * 0.25f;
    float yaw_rad = chassis.yaw * MATH_DEG_TO_RAD;
    float px = chassis.pos_x + cosf(yaw_rad) * bx - sinf(yaw_rad) * by;
    float py = chassis.pos_y + sinf(yaw_rad) * bx + cosf(yaw_rad) * by;
    /* 偏线只量当前所在的一段参考折线：从上次匹配处附近向后扫，
       走过的旧段（跑图时重复经过的边）不参与取最小，避免指标偏乐观。 */
    float cross = INFINITY;
    unsigned from = s_reference_cursor >= 4U ? s_reference_cursor - 4U : 0U;
    for (unsigned i = from + 1U; i < s_reference_count; ++i)
    {
        float ax = s_reference[i - 1].x, ay = s_reference[i - 1].y;
        float ux = s_reference[i].x - ax, uy = s_reference[i].y - ay;
        float l2 = ux * ux + uy * uy;
        float t = l2 > 0.001f ? ((chassis.pos_x - ax) * ux + (chassis.pos_y - ay) * uy) / l2 : 0;
        t = fmaxf(0, fminf(1, t));
        float d = hypotf(chassis.pos_x - ax - t * ux, chassis.pos_y - ay - t * uy);
        if (d < cross)
        {
            cross = d;
            s_reference_cursor = i;
        }
    }
    chassis_path_result.distance_mm +=
        hypotf(chassis.pos_x - s_path_last_x, chassis.pos_y - s_path_last_y);
    s_path_last_x = chassis.pos_x;
    s_path_last_y = chassis.pos_y;
    chassis_path_result.elapsed_ms = HAL_GetTick() - s_path_started;
    chassis_waypoint goal;
    float dx = 0, dy = 0, length = 0, ex = 0, ey = 0, distance = 0, exit_speed = 0;
    uint8_t transit = 0;
    for (;;)
    {
        goal = s_path[s_path_index];
        dx = goal.x - s_path_start.x;
        dy = goal.y - s_path_start.y;
        length = hypotf(dx, dy);
        ex = goal.x - px;
        ey = goal.y - py;
        distance = hypotf(ex, ey);
        float progress =
            length > 0.001f
                ? ((px - s_path_start.x) * dx + (py - s_path_start.y) * dy) / (length * length)
                : 1;
        float projected = progress;
        progress = fmaxf(0, fminf(1, progress));
        chassis.heading_target =
            s_path_start.heading + Math_WrapDeg(goal.heading - s_path_start.heading) * progress;
        chassis.heading_error = Math_WrapDeg(chassis.heading_target - chassis.yaw);
        chassis_path_result.max_heading_deg =
            fmaxf(chassis_path_result.max_heading_deg, fabsf(chassis.heading_error));
        exit_speed = 0;
        if (!s_path_stop_each && s_path_index + 1 < s_path_count && length > 0.001f)
        {
            float nx = s_path[s_path_index + 1].x - goal.x,
                  ny = s_path[s_path_index + 1].y - goal.y;
            float nl = hypotf(nx, ny);
            float dot = nl > 0.001f ? (dx * nx + dy * ny) / (length * nl) : -1;
            if (dot > 0.999f)
                exit_speed = s_move_speed_limit;
            else if (dot > -0.999f)
            {
                float c = sqrtf(fmaxf(0, (1 + dot) * 0.5f));
                exit_speed =
                    fminf(s_move_speed_limit, sqrtf(CHASSIS_ACC_LINEAR * CHASSIS_PATH_DEVIATION_MM *
                                                    c / fmaxf(0.001f, 1 - c)));
            }
        }
        exit_speed = fminf(exit_speed, s_path_limit[s_path_index]);
        transit = exit_speed > 0;
        if (!(transit && (distance <= 0.15f || (projected >= 1 && distance < 1.0f))))
            break;
        /* 零长度或极短航点会连续命中推进条件：用循环推进，不能递归。 */
        s_path_start = goal;
        ++s_path_index;
        s_path_segment_started = HAL_GetTick();
    }
    /* 参考折线被截断时退回当前段弦长：仍能反映偏线，代价是含一点贝塞尔自身的偏移。 */
    if (!isfinite(cross) || s_reference_truncated)
    {
        float ll = hypotf(goal.x - s_path_start.x, goal.y - s_path_start.y);
        if (ll > 0.001f)
        {
            float t = ((chassis.pos_x - s_path_start.x) * (goal.x - s_path_start.x) +
                       (chassis.pos_y - s_path_start.y) * (goal.y - s_path_start.y)) /
                      (ll * ll);
            t = fmaxf(0, fminf(1, t));
            cross = hypotf(chassis.pos_x - s_path_start.x - t * (goal.x - s_path_start.x),
                           chassis.pos_y - s_path_start.y - t * (goal.y - s_path_start.y));
        }
        else
            cross = hypotf(goal.x - chassis.pos_x, goal.y - chassis.pos_y);
    }
    chassis_path_result.max_cross_mm = fmaxf(chassis_path_result.max_cross_mm, cross);
    float available = distance;
    float speed = fminf(s_move_speed_limit,
                        sqrtf(exit_speed * exit_speed + 2 * CHASSIS_ACC_LINEAR * distance));
    for (unsigned i = s_path_index; i < s_path_count; ++i)
    {
        if (i > s_path_index)
            available += hypotf(s_path[i].x - s_path[i - 1].x, s_path[i].y - s_path[i - 1].y);
        float limit = i + 1 == s_path_count ? 0 : s_path_limit[i];
        speed = fminf(speed, sqrtf(limit * limit + 2 * CHASSIS_ACC_LINEAR * available));
    }
    speed = fminf(speed, s_path_limit[s_path_index]);
    if (exit_speed == 0 && distance < 20)
        speed = fminf(speed, 8 * distance);
    float rad = chassis.yaw * MATH_DEG_TO_RAD;
    uint8_t translating = (transit || distance > CHASSIS_DISTANCE_TOL_MM) && distance > 0.001f;
    chassis.target.vx = translating ? speed * (cosf(rad) * ex + sinf(rad) * ey) / distance : 0;
    chassis.target.vy = translating ? speed * (-sinf(rad) * ex + cosf(rad) * ey) / distance : 0;
    chassis.target.wz = Chassis_HeadingControl(dt, distance <= CHASSIS_DISTANCE_TOL_MM);
    if (translating)
    {
        float error = fabsf(chassis.heading_error);
        if (error > chassis.heading_peak_dynamic)
            chassis.heading_peak_dynamic = error;
        if (error > CHASSIS_DYNAMIC_TOL_DEG)
            ++chassis.heading_over_limit;
    }
    if (HAL_GetTick() - s_path_segment_started > 60000U)
    {
        Chassis_Fault(CHASSIS_FAULT_PATH_TIMEOUT, 0U);
        return;
    }
    if (!transit && distance <= CHASSIS_DISTANCE_TOL_MM &&
        (hypotf(chassis.current.vx, chassis.current.vy) < 2 &&
         fabsf(chassis.heading_error) <= CHASSIS_HEADING_TOL_DEG))
    {
        s_path_start = goal;
        s_path_segment_started = HAL_GetTick();
        if (++s_path_index >= s_path_count)
        {
            Chassis_Stop();
            chassis_path_result.completed = 1;
            s_heading_hold = 1;
        }
    }
}
