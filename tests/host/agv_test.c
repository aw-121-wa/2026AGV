#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "chassis.h"
#include "agv_run.h"
#include "agv_config.h"
#include "map.h"
chassis_t chassis;
static unsigned now,moves;
static float gx,gy;
static unsigned physical_edges,test_zone,freeze;
uint32_t HAL_GetTick(void) { return now; }
void Chassis_Stop(void) { chassis.mode=CHASSIS_MODE_VEL; }
void Chassis_SetMoveSpeedLimit(float x) { (void)x; }
HAL_StatusTypeDef Chassis_MoveDistanceHold(float x,float y,float heading) {
 (void)heading;
 if(chassis.state!=CHASSIS_STATE_READY) return HAL_BUSY;
 float a=chassis.yaw*0.01745329252f;
 gx=chassis.pos_x+cosf(a)*x-sinf(a)*y; gy=chassis.pos_y+sinf(a)*x+cosf(a)*y;
 /* Independent trace coverage: a segment must cross each channel midpoint. */
 float tx=2250+cosf(a)*gx+sinf(a)*gy;
 float ty=(test_zone==1?2250:150)-sinf(a)*gx+cosf(a)*gy;
 const float axes[3]={355,1205,2055};
 for(unsigned row=0;row<3;++row) for(unsigned col=0;col<2;++col) {
  float mid=(axes[col]+axes[col+1])*0.5f;
  if(fabsf(agv.y-axes[row])<150 && fabsf(ty-axes[row])<150 &&
     fminf(agv.x,tx)<mid && fmaxf(agv.x,tx)>mid) physical_edges|=1U<<(row*2+col);
 }
 for(unsigned col=0;col<3;++col) for(unsigned row=0;row<2;++row) {
  float mid=(axes[row]+axes[row+1])*0.5f;
  if(fabsf(agv.x-axes[col])<150 && fabsf(tx-axes[col])<150 &&
     fminf(agv.y,ty)<mid && fmaxf(agv.y,ty)>mid) physical_edges|=1U<<(6+col*2+row);
 }
 chassis.mode=CHASSIS_MODE_MOVE; ++moves; return HAL_OK;
}
static void tick(void) {
 now+=5;
 if(!freeze && chassis.mode==CHASSIS_MODE_MOVE) { chassis.pos_x=gx; chassis.pos_y=gy; chassis.mode=CHASSIS_MODE_VEL; }
 Agv_Tick();
}
int main(void) { setvbuf(stdout, NULL, _IONBF, 0);
 for(unsigned zone=1;zone<=2;++zone) {
  test_zone=zone; physical_edges=0;
  chassis=(chassis_t){0}; chassis.state=CHASSIS_STATE_READY; chassis.yaw=37;
  Agv_Init(AGV_MODE_SURVEY,zone);
  for(unsigned n=0;n<100000 && agv.state!=AGV_DONE && agv.state!=AGV_FAULT;++n) tick();
  if(agv.state!=AGV_DONE) printf("state=%d error=%d leg=%u x=%f y=%f\n",agv.state,agv.error,agv.leg,agv.x,agv.y);
  assert(agv.state==AGV_DONE && agv.covered==0xfff);
  assert(physical_edges==0xfff);
  assert(fabsf(agv.x-2250)<5 && fabsf(agv.y-(zone==1?2250:150))<5);
 }
 Agv_Init(AGV_MODE_SURVEY,1); Agv_Stop(); unsigned before=moves;
 for(unsigned n=0;n<1000;++n) tick();
 assert(agv.state==AGV_STOPPED && moves==before);
 Agv_Init(AGV_MODE_COMPETITION,1);
 for(unsigned n=0;n<1000;++n) tick();
 assert(moves==before);
 Agv_Init(AGV_MODE_SURVEY,1); assert(Map_AddObstacle(2250,2250,25));
 for(unsigned n=0;n<1000 && agv.state!=AGV_FAULT;++n) tick();
 assert(agv.state==AGV_FAULT && moves==before);
 /* Readiness never arrives: no autonomous motion. */
 chassis.state=CHASSIS_STATE_IDLE; Agv_Init(AGV_MODE_SURVEY,1);
 for(unsigned n=0;n<4000;++n) tick();
 assert(agv.state==AGV_FAULT && agv.error==AGV_ERR_READY && moves==before);
 /* Chassis fault must not be interpreted as a completed movement. */
 chassis.state=CHASSIS_STATE_READY; Agv_Init(AGV_MODE_SURVEY,1);
 for(unsigned n=0;n<1000 && agv.state==AGV_WAIT;++n) tick();
 chassis.state=CHASSIS_STATE_FAULT; tick();
 assert(agv.state==AGV_FAULT && agv.error==AGV_ERR_CHASSIS && agv.covered==0);
 /* A blocked channel must fail, not shortcut through another channel. */
 chassis=(chassis_t){0}; chassis.state=CHASSIS_STATE_READY;
 Agv_Init(AGV_MODE_SURVEY,1); assert(Map_AddObstacle(1605,2055,200));
 for(unsigned n=0;n<20000 && agv.state!=AGV_FAULT;++n) tick();
 assert(agv.state==AGV_FAULT && agv.covered!=0xfff);
 /* Updating obstacle map invalidates an in-flight route. */
 chassis=(chassis_t){0}; chassis.state=CHASSIS_STATE_READY;
 Agv_Init(AGV_MODE_SURVEY,1);
 for(unsigned n=0;n<1000 && agv.state==AGV_WAIT;++n) tick();
 assert(Map_AddObstacle(1200,1200,25)); tick();
 assert(agv.state==AGV_FAULT && agv.error==AGV_ERR_MAP);
 /* Heading fault needs a sustained excursion; a transient spike is tolerated. */
 chassis=(chassis_t){0}; chassis.state=CHASSIS_STATE_READY;
 Agv_Init(AGV_MODE_SURVEY,1);
 for(unsigned n=0;n<1000 && agv.state==AGV_WAIT;++n) tick();
 freeze=1;
 chassis.yaw=0.5f;tick();assert(agv.state!=AGV_FAULT);
 chassis.yaw=2.9f;for(unsigned n=0;n<60;++n) tick();assert(agv.state!=AGV_FAULT);
 chassis.yaw=3.1f;for(unsigned n=0;n<30;++n) tick();assert(agv.state!=AGV_FAULT);
 chassis.yaw=0;tick();
 chassis.yaw=3.1f;for(unsigned n=0;n<50 && agv.state!=AGV_FAULT;++n) tick();
 assert(agv.state==AGV_FAULT && agv.error==AGV_ERR_POSE);
 freeze=0;
 /* During ordinary planning, physical clearance alone is insufficient. */
 chassis=(chassis_t){0}; chassis.state=CHASSIS_STATE_READY;
 Agv_Init(AGV_MODE_SURVEY,1);
 for(unsigned n=0;n<2000 && agv.state!=AGV_DRIVE;++n) tick();
 assert(agv.state==AGV_DRIVE); /* clear channel: straight-line fast path, no A* */
 chassis.pos_x=-260; chassis.pos_y=-260; /* field (1990,1990), body clears by 5 mm */
 tick();
 assert(agv.state==AGV_FAULT && agv.error==AGV_ERR_POSE);
 chassis=(chassis_t){0}; chassis.state=CHASSIS_STATE_READY;
 Agv_Init(AGV_MODE_SURVEY,1); freeze=1;
 for(unsigned n=0;n<8000 && agv.state!=AGV_FAULT;++n) tick();
 assert(agv.state==AGV_FAULT && agv.error==AGV_ERR_TIMEOUT);
 puts("runner both docks, cover/return, stop latch, placeholder, blocked dock OK");
}

