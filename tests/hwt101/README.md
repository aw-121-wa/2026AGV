# HWT101 主机测试

在工程根目录用 GCC 执行（不需要连接开发板）：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/hwt101 -IAPP/HWT101 tests/hwt101/test_hwt101.c APP/HWT101/hwt101.c -o build/test_hwt101.exe
./build/test_hwt101.exe
```

使用最小 UART 替身向真实驱动注入字节，验证分段接收、正负角度、环形回绕、噪声与坏帧恢复、校验失败、非角度帧、空指针及缓冲区满后的恢复。
不验证硬件接线、串口中断时序或 HAL 错误恢复。
