#ifndef CONSOLE_H
#define CONSOLE_H

/* 串口控制台（USART6）：chassis_cmd.c 把以 '/' 开头的一行（不含 '/' 和行尾）交给这里。
   line 会被就地切分。只在底盘任务里调用。命令说明见 README 4.4 节，界面在 tools/agv_console.html，
   它按文本格式解析回复，改输出格式要同时改界面。 */
void Console_Line(char *line);

#endif
