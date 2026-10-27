"""Check production X-firmware speed packing against manual byte fixtures."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
source = (root / 'APP/ZDT_X42S/zdt_x42s.c').read_text(encoding='utf-8')
start = source.index('HAL_StatusTypeDef ZDT_SetSpeed(')
end = source.index('\n/*', start)
test = r'''
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "zdt_x42s.h"
#define ZDT_CODE_SPEED 0xF6U
#define ZDT_SYNC_NOW 0U
static uint8_t captured[16], size;
static HAL_StatusTypeDef ZDT_SendCommand(uint8_t a,uint8_t c,const uint8_t *v,uint8_t n) {
    if(a!=1 || c!=0xF6) exit(2);
    memcpy(captured,v,n);size=n;return HAL_OK;
}
''' + source[start:end] + r'''
int main(void) {
    zdt_motor m={.address=1};
    const uint8_t reverse[]={1,0,10,0x3A,0x98,0}; /* -1500 RPM, 10 RPM/s */
    if(ZDT_SetSpeed(&m,-1500,10)!=HAL_OK || size!=6 || memcmp(captured,reverse,6)) return 1;
    const uint8_t stopped[]={0,0,0,0,0,0};
    if(ZDT_SetSpeed(&m,0,0)!=HAL_OK || size!=6 || memcmp(captured,stopped,6)) return 1;
    const uint8_t limited[]={0,0,0,0x75,0x30,0}; /* clamp to 3000.0 RPM */
    if(ZDT_SetSpeed(&m,6000,0)!=HAL_OK || size!=6 || memcmp(captured,limited,6)) return 1;
    return 0;
}
'''
out = root/'build/zdt_protocol_test.c'
out.write_text(test, encoding='utf-8')
exe = root/'build/zdt_protocol_test.exe'
subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itests/host',
                '-IAPP/ZDT_X42S',str(out),'-o',str(exe)],cwd=root,check=True)
subprocess.run([str(exe)],check=True,timeout=5)
print('PASS: X speed format, sign, RPM scale, zero and limit')
