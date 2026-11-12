#include <assert.h>
#include <stdio.h>
#include "map.h"
int main(void)
{
    Map_Init();
    assert(Map_IsFree(35, 35)); /* outer ring missing in teaching map */
    assert(Map_IsFree(120, 120));
    assert(!Map_IsFree(75, 75));
    assert(!Map_IsFree(120, 5)); /* storage */
    assert(!Map_IsFree(5, 120)); /* processing */
    assert(Map_IsFree(225, 225));
    assert(Map_IsFree(15, 225));
    uint16_t r = 77, c = 88;
    assert(!Map_WorldToGrid(-1, 10, &r, &c) && r == 77 && c == 88);
    assert(!Map_WorldToGrid(2400, 10, &r, &c));
    assert(!Map_WorldToGrid(0, 0, 0, &c));
    assert(Map_WorldToGrid(125, 237, &r, &c) && r == 23 && c == 12);
    assert(Map_CanPoseHeading(2250, 2250, 0, 0));
    assert(Map_CanPoseHeading(2265, 2250, 0, 0)); /* 270/2 = 135 */
    assert(!Map_CanPoseHeading(2266, 2250, 0, 0));
    assert(Map_CanPoseHeading(2250, 2250, 0, 10));
    assert(!Map_CanPoseHeading(2256, 2250, 0, 10));
    assert(Map_CanSegment((MapPoint){2250, 2250}, (MapPoint){2250, 2055}, 0));
    assert(Map_CanSegment((MapPoint){2250, 2055}, (MapPoint){2055, 2055}, 0));
    assert(Map_CanSegment((MapPoint){2250, 150}, (MapPoint){2250, 355}, 0));
    assert(!Map_CanSegment((MapPoint){355, 355}, (MapPoint){2055, 2055}, 0));
    assert(Map_AddObstacle(1200, 1200, 25));
    assert(!Map_CanPoseHeading(1200, 1200, 0, 0));
    assert(!Map_CanSegment((MapPoint){355, 1205}, (MapPoint){2055, 1205}, 10));
    Map_Init();
    /* 270 mm body fits a small angle, but not 10 degrees in the corner. */
    assert(Map_CanPoseHeading(2250, 2250, 0.2f, 10));
    assert(!Map_CanPoseHeading(2250, 2250, 10.0f, 0));
    assert(Map_CanPoseHeading(1205, 1205, 1.0f, 10));
    assert(Map_CanPoseHeading(410, 755, 1.0f, 0));
    assert(!Map_CanPoseHeading(410, 755, 1.0f, 10));

    /* 附加圆（伸出去的雷达）：车身放得下、圆出了场地或压到障碍物/禁区也不能站；加上才改版本，去掉不改。 */
    assert(Map_WorldToGrid(2255, 1205, &r, &c) && Map_CanStand(r, c));
    uint32_t revision = Map_Revision();
    Map_SetAttachment(155.0f, 0.0f, 38.0f); /* 圆心 x = 2410，出了场地 */
    assert(Map_Revision() == revision + 1U && !Map_CanStand(r, c));
    Map_SetAttachment(155.0f, 0.0f, 38.0f);
    assert(Map_Revision() == revision + 1U);
    Map_SetAttachment(-155.0f, -155.0f, 38.0f); /* (2100, 1050)：车道里 */
    assert(Map_CanStand(r, c));
    assert(Map_AddObstacle(2100, 990, 25)); /* 离圆心 60 < 38 + 25 */
    assert(!Map_CanStand(r, c));
    Map_Init();
    assert(Map_WorldToGrid(1205, 1205, &r, &c) && Map_CanStand(r, c));
    Map_SetAttachment(-155.0f, -155.0f, 38.0f); /* (1050, 1050)：离左下的岛角 71 mm */
    assert(Map_CanStand(r, c));
    assert(Map_WorldToGrid(905, 1205, &r, &c) && Map_CanStand(r, c)); /* 圆心离岛 50 mm */
    assert(Map_WorldToGrid(905, 1185, &r, &c) && !Map_CanStand(r, c)); /* 30 mm：压到岛，车身没压 */
    revision = Map_Revision();
    Map_SetAttachment(0.0f, 0.0f, 0.0f);
    assert(Map_Revision() == revision && Map_CanStand(r, c));
    Map_SetAttachment(-155.0f, -155.0f, 38.0f);
    Map_Init();
    assert(Map_CanStand(r, c));
    puts("map geometry OK");
}
