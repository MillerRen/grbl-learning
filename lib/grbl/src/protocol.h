/*
  protocol.h - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef protocol_h
#define protocol_h

// 从串口输入的用来执行的流的行缓冲区大小
// 注意：行缓冲区可能太小了，g代码块可能会被截取，一般没问题，除非极端情况下。
// 官方g代码标准支持到256个字符。在未来的版本中，这将会被增加，当我们知道我们有多少额外空间可以投入时，
// 或者重写g代码解析器不需要这个缓冲区
// Line buffer size from the serial input stream to be executed.
// NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
// and g-code blocks can get truncated. Officially, the g-code standards support up to 256
// characters. In future versions, this will be increased, when we know how much extra
// memory space we can invest into here or we re-write the g-code parser not to have this
// buffer.
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif

// 开始Grbl主循环。这将会处理从串口即将到来的字符串，在他们传输完成时执行他们。
// 这也负责完成初始化过程。
// Starts Grbl main loop. It handles all incoming characters from the serial port and executes
// them as they complete. It is also responsible for finishing the initialization procedures.
void protocol_main_loop();

// 在主程序中一系列停止点检查和执行一个实时命令
// Checks and executes a realtime command at various stop points in main program
void protocol_execute_realtime();
void protocol_exec_rt_system();

// 执行自动循环特性，如果启用了该特性。
// Executes the auto cycle feature, if enabled.
void protocol_auto_cycle_start();

// 阻塞直到所有被缓冲的脉冲步数被执行了
// Block until all buffered steps are executed
void protocol_buffer_synchronize();

#endif
