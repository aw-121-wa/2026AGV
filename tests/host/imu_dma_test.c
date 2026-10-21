#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "hwt101.h"
#include "chassis_cmd.h"

static uint32_t tick_ms;
static int starts, fail_start, command_events;
static uint8_t *active_buffer;
static uint16_t active_size, producer;
static UART_HandleTypeDef *active_uart;
static MPU_Region_InitTypeDef configured_region;
uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_MPU_Disable(void) {}
void HAL_MPU_Enable(uint32_t mode) { assert(mode == MPU_PRIVILEGED_DEFAULT); }
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *region) { configured_region = *region; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) {
    uart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *buf, uint16_t n) {
    ++starts;
    if (fail_start) return HAL_ERROR;
    active_uart=uart;active_buffer=buf;active_size=n;producer=0;
    uart->hdmarx->remaining=n;uart->RxState=1;
    return HAL_OK;
}
void ChassisCmd_RxEvent(UART_HandleTypeDef *uart,uint16_t size) {
    (void)size;
    if (uart != active_uart) ++command_events;
}
#include "../../APP/HWT101/hwt101.c"
#include "../../APP/uart_callbacks.c"

static void bytes(const uint8_t *data, unsigned n, int idle, int process) {
    for(unsigned i=0;i<n;i++) {
        active_buffer[producer++]=data[i];
        if(producer==active_size) producer=0;
        active_uart->hdmarx->remaining=active_size-producer;
        if(producer==0 || producer==active_size/2) {
            HAL_UARTEx_RxEventCallback(active_uart,producer ? producer : active_size);
            if(process) HWT101_ProcessData(active_uart,&imu);
        }
    }
    if(idle) HAL_UARTEx_RxEventCallback(active_uart,producer);
    if(process) HWT101_ProcessData(active_uart,&imu);
}
static void yaw_frame(uint8_t *f, int16_t yaw) {
    memset(f,0,11);f[0]=0x55;f[1]=0x53;
    f[6]=(uint16_t)yaw & 255;f[7]=(uint16_t)yaw >> 8;
    for(int i=0;i<10;i++) f[10]+=f[i];
}
int main(void) {
    DMA_HandleTypeDef dma={.Init={.Mode=DMA_CIRCULAR}};
    UART_HandleTypeDef uart={.hdmarx=&dma}, other={0};
    assert(HWT101_Init(&other)==HAL_ERROR);
    assert(HWT101_Init(&uart)==HAL_OK);
    assert(HWT101_Init(&uart)==HAL_BUSY);
    assert(configured_region.Size==MPU_REGION_SIZE_256B);
    assert(configured_region.IsCacheable==MPU_ACCESS_NOT_CACHEABLE);
    assert(((uintptr_t)active_buffer%256)==0);
    uint8_t f[11];yaw_frame(f,16384);tick_ms=10;
    bytes(f,5,1,1);assert(!imu.valid);
    bytes(f+5,6,1,1);assert(imu.valid && fabsf(imu.yaw-90)<0.001f);
    assert(imu.updated_ms==10);
    uint16_t head_before=rx_head;
    HAL_UARTEx_RxEventCallback(&uart,128); // Delayed notification, NDTR unchanged.
    assert(rx_head==head_before);
    HAL_UARTEx_RxEventCallback(&other,3);assert(command_events==1 && rx_head==head_before);
    uint8_t padding[244]={0};bytes(padding,sizeof padding,1,1); // Producer at 255.
    yaw_frame(f,-8192);tick_ms=20;bytes(f,11,1,1);
    assert(fabsf(imu.yaw+45)<0.001f); // Frame crosses circular buffer end.
    for(int i=0;i<100;i++) { tick_ms++;bytes(f,11,0,1); }
    HAL_UARTEx_RxEventCallback(&uart,producer);HWT101_ProcessData(&uart,&imu);
    assert(imu.valid && imu.updated_ms==tick_ms && starts==1);
    uint32_t previous=imu.updated_ms;f[10]^=1;tick_ms++;
    bytes(f,11,1,1);assert(imu.updated_ms==previous);
    yaw_frame(f,4096);bytes(f,11,1,1);assert(fabsf(imu.yaw-22.5f)<0.001f);
    tick_ms=200;bytes(f,11,1,0);tick_ms=900;HWT101_ProcessData(&uart,&imu);
    assert(imu.updated_ms==200); // A delayed task cannot make old data fresh.
    tick_ms=300;bytes(f,11,1,0);
    tick_ms=900;uint8_t noise=0;bytes(&noise,1,1,0);
    HWT101_ProcessData(&uart,&imu);assert(imu.updated_ms==300);
    for(int i=0;i<60;i++) bytes(f,11,1,0);
    assert(rx_gap);HWT101_ProcessData(&uart,&imu);assert(!imu.valid && !rx_gap);
    bytes(f,11,1,1);assert(imu.valid);
    HAL_UART_ErrorCallback(&other);assert(!restart_rx);
    HAL_UART_ErrorCallback(&uart);assert(restart_rx && !imu.valid);
    fail_start=1;HWT101_ProcessData(&uart,&imu);assert(restart_rx && !imu.valid);
    fail_start=0;HWT101_ProcessData(&uart,&imu);assert(!restart_rx && !imu.valid);
    bytes(f,11,1,1);assert(imu.valid && fabsf(imu.yaw-22.5f)<0.001f);
    assert(starts==3);
    puts("PASS: DMA split/wrap, HT/TC continuous stream, duplicate events, callback routing, checksum, overflow, error restart, arrival timestamp");
    return 0;
}
