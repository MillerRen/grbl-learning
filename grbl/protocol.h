/*
  protocol.h - 控制Grbl执行协议和过程
  Grbl 的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef protocol_h
#define protocol_h

// 要执行的串口输入流的行缓冲区大小。
// 注意：除了极端情况，这不是问题，但是行缓冲区可能太小G代码块可能被截断。
// 官方标准最多支持256个字符。在未来版本，可能会增加它，如果我们知道这里需要耗费多少内存。
// 者我们重写G代码编辑器不再使用这个缓冲器。
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif


// 启动Grbl主循环。它处理所有的即将从串口端口到来的字符，并在他们结束时执行他们。
// 它还负责完成初始化过程。
void protocol_main_loop();

// 在主程序中的一系列停止点位检查和执行实时命令
void protocol_execute_realtime();
// 执行实时系统
void protocol_exec_rt_system();

// 执行自动循环功能，如果开启了的话
void protocol_auto_cycle_start();

// 阻塞直到所有缓冲的步数被执行完
void protocol_buffer_synchronize();

#endif
