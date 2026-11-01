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
    assert(Map_CanPose(2250, 2250, 0));
    assert(Map_CanPose(2265, 2250, 0)); /* 270/2 = 135 */
    assert(!Map_CanPose(2266, 2250, 0));
    assert(Map_CanPose(2250, 2250, 10));
    assert(!Map_CanPose(2256, 2250, 10));
    assert(Map_CanSegment((MapPoint){2250, 2250}, (MapPoint){2250, 2055}, 0));
    assert(Map_CanSegment((MapPoint){2250, 2055}, (MapPoint){2055, 2055}, 0));
    assert(Map_CanSegment((MapPoint){2250, 150}, (MapPoint){2250, 355}, 0));
    assert(!Map_CanSegment((MapPoint){355, 355}, (MapPoint){2055, 2055}, 0));
    assert(Map_AddObstacle(1200, 1200, 25));
    assert(!Map_CanPose(1200, 1200, 0));
    assert(!Map_CanSegment((MapPoint){355, 1205}, (MapPoint){2055, 1205}, 10));
    Map_Init();
    /* 270 mm body fits a small angle, but not 10 degrees in the corner. */
    assert(Map_CanPoseHeading(2250, 2250, 0.2f, 10));
    assert(!Map_CanPoseHeading(2250, 2250, 10.0f, 0));
    assert(Map_CanPoseHeading(1205, 1205, 1.0f, 10));
    assert(Map_CanPoseHeading(410, 755, 1.0f, 0));
    assert(!Map_CanPoseHeading(410, 755, 1.0f, 10));
    puts("map geometry OK");
}
