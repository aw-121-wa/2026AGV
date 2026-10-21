#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "hwt101.h"
#include "zdt_x42s.h"
host_dwt host_clock;
host_debug host_core;
uint32_t SystemCoreClock = 480000000U;
static uint32_t ms;
imu_data imu;
static zdt_motor motors[4];
static int16_t output[4];
static int fail_send;
static uint32_t event_type = HAL_UART_RXEVENT_IDLE;
uint32_t HAL_GetTick(void) { return ms; }
HAL_StatusTypeDef ZDT_Init(FDCAN_HandleTypeDef *p) { (void)p; return HAL_OK; }
zdt_motor *ZDT_AddMotor(uint8_t a) { motors[a-1].address=a; return &motors[a-1]; }
HAL_StatusTypeDef ZDT_SetSpeed(zdt_motor *m,int16_t rpm,uint8_t acc) {
    (void)acc;
    if (fail_send) return HAL_ERROR;
    output[m->address-1] = rpm; return HAL_OK;
}
HAL_StatusTypeDef ZDT_Stop(zdt_motor *m) {
    if (fail_send) return HAL_ERROR;
    output[m->address-1]=0; return HAL_OK;
}
HAL_StatusTypeDef ZDT_EnableMotor(zdt_motor *m,uint8_t en) { m->enabled=en; return HAL_OK; }
HAL_StatusTypeDef ZDT_RequestFeedback(zdt_motor *m) { (void)m; return HAL_OK; }
void ZDT_ProcessData(void) {}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n) {
    (void)u;(void)b;(void)n;return HAL_OK;
}
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u) { (void)u;return event_type; }
#include "../../APP/Chassis_motor/chassis.c"
#include "../../APP/Chassis_motor/chassis_cmd.c"
void HWT101_RxEvent(UART_HandleTypeDef *u, uint16_t n) { (void)u; (void)n; }
void HWT101_RxError(UART_HandleTypeDef *u) { (void)u; }
#include "../../APP/uart_callbacks.c"

static void feedback(void) {
    imu.valid=1;imu.updated_ms=ms;
    for (int i=0;i<4;i++) {
        motors[i].enabled=1;motors[i].status_valid=1;motors[i].speed_valid=1;
        motors[i].position_seq++;
        motors[i].position_ms=motors[i].status_ms=motors[i].speed_ms=ms;
    }
}
static void tick(int fresh) {
    ms+=5;host_clock.CYCCNT+=2400000;
    if (fresh) feedback();
    Chassis_Update();
}
static void plant(void) {
    /* Ideal wheels, actual output RPM; yaw follows inverse kinematics. */
    float v[4];
    for(int i=0;i<4;i++) {
        motors[i].speed_rpm=output[i];
        motors[i].position_deg+=output[i]*6.0f*0.005f;
        v[i]=output[i]*3.14159265f*CHASSIS_WHEEL_DIA_MM/60.0f;
    }
    imu.yaw=Chassis_Wrap(imu.yaw+(-v[0]+v[1]-v[2]+v[3])/
        (4*CHASSIS_K_MM)*57.2957795f*0.005f);
    tick(1);
}
static void complete(void) {
    for (int i=0;i<5900 && chassis.mode!=CHASSIS_MODE_VEL;i++) plant();
    assert(chassis.state==CHASSIS_STATE_READY);
    assert(chassis.mode==CHASSIS_MODE_VEL);
}
static void sampled_move(void) {
    float position[4];
    for(int i=0;i<4;i++) position[i]=motors[i].position_deg;
    float x=chassis.pos_x;
    assert(Chassis_MoveDistance(500,0)==HAL_OK);
    int steps=0;
    while(chassis.mode!=CHASSIS_MODE_VEL && steps++<5900) {
        float v[4];
        for(int i=0;i<4;i++) {
            position[i]+=output[i]*6.0f*0.005f;
            v[i]=output[i]*3.14159265f*CHASSIS_WHEEL_DIA_MM/60.0f;
        }
        imu.yaw=Chassis_Wrap(imu.yaw+(-v[0]+v[1]-v[2]+v[3])/
            (4*CHASSIS_K_MM)*57.2957795f*0.005f);
        ms+=5;host_clock.CYCCNT+=2400000;imu.updated_ms=ms;
        if(steps%4==0) { // One wheel each 20 ms, full sweep 80 ms.
            int i=(steps/4)%4;
            motors[i].position_deg=position[i];motors[i].speed_rpm=output[i];
            motors[i].position_seq++;
            motors[i].position_ms=motors[i].speed_ms=motors[i].status_ms=ms;
        }
        Chassis_Update();
    }
    assert(chassis.mode==CHASSIS_MODE_VEL && chassis.state==CHASSIS_STATE_READY);
    assert(fabsf(chassis.pos_x-x-500)<4);
}
static void command(UART_HandleTypeDef *u,const char *text) {
    size_t n=strlen(text);assert(n<=CMD_BUF_SIZE);
    memcpy(rx_buf,text,n);HAL_UARTEx_RxEventCallback(u,(uint16_t)n);ChassisCmd_Poll();
}
int main(void) {
    FDCAN_HandleTypeDef can={0};UART_HandleTypeDef uart={0};
    assert(Chassis_Init(&can)==HAL_OK);
    assert(Chassis_TurnTo(90)==HAL_BUSY);
    tick(1);assert(chassis.state==CHASSIS_STATE_READY);
    assert(ChassisCmd_Init(&uart)==HAL_OK);
    imu.yaw=179;tick(1);
    assert(Chassis_TurnTo(-179)==HAL_OK);tick(1);
    assert(chassis.heading_error>0 && chassis.heading_error<3);
    assert(chassis.target.wz>0);Chassis_Stop();
    imu.yaw=0;tick(1);
    assert(Chassis_TurnBy(180)==HAL_OK);tick(1);
    assert(chassis.target.wz>0);Chassis_Stop();
    assert(Chassis_TurnBy(-180)==HAL_OK);tick(1);
    assert(chassis.target.wz<0);Chassis_Stop();
    imu.yaw=90;tick(1);
    assert(Chassis_TurnBy(180)==HAL_OK);tick(1);
    assert(chassis.target.wz>0);Chassis_Stop();
    imu.yaw=0;tick(1);
    float x=chassis.pos_x,y=chassis.pos_y;
    command(&uart,"F1000\n");assert(chassis_cmd_status==HAL_OK);
    command(&uart,"T90\n");assert(chassis_cmd_status==HAL_BUSY);
    for(int i=0;i<100;i++) plant();
    imu.yaw+=8; // external heading disturbance while translating
    complete();
    assert(fabsf(chassis.pos_x-x-1000)<4);
    assert(fabsf(chassis.pos_y-y)<4);
    assert(fabsf(chassis.yaw)<1.1f);
    command(&uart,"A90\n");complete();assert(fabsf(chassis.yaw-90)<1.1f);
    x=chassis.pos_x;y=chassis.pos_y;
    command(&uart,"F200\n");complete();
    assert(fabsf(chassis.pos_x-x)<6);assert(fabsf(chassis.pos_y-y-200)<4);
    command(&uart,"T-90\n");complete();assert(fabsf(chassis.yaw)<2.1f);
    command(&uart,"A0\n");assert(chassis_cmd_status==HAL_OK);complete();
    command(&uart,"F1.5\n");assert(chassis_cmd_status==HAL_ERROR);
    command(&uart,"T999999999999999999\n");assert(chassis_cmd_status==HAL_ERROR);
    assert(Chassis_MoveDistance(NAN,0)==HAL_ERROR);
    command(&uart,"F100\n");tick(1);
    for(int i=0;i<102;i++) tick(0);
    assert(chassis.state==CHASSIS_STATE_FAULT);
    tick(1);assert(chassis.state==CHASSIS_STATE_FAULT);
    command(&uart,"S\n");assert(chassis.state==CHASSIS_STATE_READY);
    assert(output[0]==0 && output[1]==0 && output[2]==0 && output[3]==0);
    Chassis_SetVelocity(100,0,0);tick(1);fail_send=1;Chassis_Stop();tick(1);
    assert(s_force_apply);fail_send=0;tick(1);assert(output[0]==0);
    assert(Chassis_MoveDistance(100,0)==HAL_OK);
    for(int i=0;i<6100;i++) tick(1); // fresh sensors but stalled motion
    assert(chassis.state==CHASSIS_STATE_FAULT);
    command(&uart,"S\n");
    imu.yaw=0;tick(1);sampled_move();
    for(int i=0;i<300;i++) Push('X');
    ChassisCmd_Poll();assert(chassis_cmd_status==HAL_ERROR);
    assert(chassis.mode==CHASSIS_MODE_VEL);
    event_type=0;command(&uart,"T-"); // Full-buffer callback does not terminate command.
    event_type=HAL_UART_RXEVENT_IDLE;command(&uart,"90\n");
    assert(chassis_cmd_status==HAL_OK);command(&uart,"S\n");
    assert(Chassis_TurnBy(90)==HAL_OK);
    uint32_t speed_time=motors[0].speed_ms;
    for(int i=0;i<102;i++) {
        ms+=5;host_clock.CYCCNT+=2400000;feedback();
        motors[0].speed_ms=speed_time;Chassis_Update();
    }
    assert(chassis.state==CHASSIS_STATE_FAULT);tick(1);command(&uart,"S\n");
    Chassis_MoveRequest(100,0);tick(1);assert(chassis.mode==CHASSIS_MODE_MOVE);
    command(&uart,"S\n");
    uint32_t before=Chassis_Micros();
    host_clock.CYCCNT+=480U;assert(Chassis_Micros()-before==1U);
    host_clock.CYCCNT=0xffffff00U;(void)Chassis_Micros();before=Chassis_Micros();
    host_clock.CYCCNT+=480U;assert(Chassis_Micros()-before==1U);
    puts("PASS: distance, heading disturbance, world coordinates, turns, parsing, feedback loss, timeout, stop retry, DWT wrap");
    return 0;
}
