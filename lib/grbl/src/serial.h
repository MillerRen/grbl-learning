/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
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

#ifndef serial_h
#define serial_h

// 定义收发缓冲区大小
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define TX_BUFFER_SIZE 112
  #else
    #define TX_BUFFER_SIZE 104
  #endif
#endif

// 定义数据为空标记
#define SERIAL_NO_DATA 0xff

// 串口初始化
void serial_init();
// 写入一个字节到串口TX缓冲区。被主程序调用
// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data);
// 从串口读缓冲区里获取第一个字节。被主程序调用
// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read();
// 重置和清空读缓冲区的数据。用于紧急停止和重置
// Reset and empty data in read buffer. Used by e-stop and reset.
void serial_reset_read_buffer();
// 返回串口RX缓冲区中可用数据的字节数
// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available();
// 返回串口RX缓冲区已用数据的字节数
// 注意：已弃用。没在使用了除非经典的状态报告在config.h被使能
// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count();
// 返回串口TX缓冲区已用的字节数
// 注意：已弃用，除非用于调试且TX没有性能问题
// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count();

#endif
