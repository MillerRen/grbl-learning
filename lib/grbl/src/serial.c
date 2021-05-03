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

#include "grbl.h"

// 初始化环形队列，注意：rear所指的单元始终为空，如果用尾指针=头指针来判断的话，是不知道是空还是满的，因为空和满尾指针都等于头指针
#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_RING_BUFFER];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;

// 返回串口RX缓冲区中可用数据的字节数
// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  // 拷贝一份，防止多个调用被修改
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}

// 返回串口RX缓冲区已用数据的字节数
// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
  // 拷贝一份，防止多个调用被修改
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}

// 返回串口TX缓冲区已用的字节数
// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  // 拷贝一份，防止多个调用被修改
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
  // Set baud rate 设置波特率
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200 高比特率加倍
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  // 使能rx，tx，和接收一个字节完成时的中断
  // enable rx, tx, and interrupt on complete reception of a byte
  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

  // 默认8位无奇偶校验1个停止位
  // defaults to 8-bit, no parity, 1 stop bit
}

// 写入一个字节到串口TX缓冲区。被主程序调用
// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
  // Calculate next head 计算下一个头指针，如果到达最大值就返回，如此循环
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }
  
  // 如果队列满了，就一直等待直到有空间为止
  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // 待完成：重构st_prep_buffer在这里被调用在长期打印时
    // 检查系统如果是重置状态则返回函数，防止死循环，因为那个状态下不再消费数据
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // 储存数据并且向前移动头指针
  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // 使能数据寄存器为空的中断以保证tx流运行
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0);
}

// 数据寄存器为空中断处理
// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
  // 临时的serial_tx_buffer_tail 为了优化不稳定的值
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
  
  // 从环形队列发送一个字节到串口
  // Send a byte from the buffer
  UDR0 = serial_tx_buffer[tail];

  // 更新尾指针位置
  // Update tail position
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // 关闭串口数据寄存器为空的中断用来阻止发送流如果发送环形队列为空
  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}

// 从串口读缓冲区里获取第一个字节。被主程序调用
// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  // 临时的serial_rx_buffer_tail 为了优化不稳定的值
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  // 如果读环形队列为空，发送数据结束标记
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    // 否则发送环形队列第一个字节，并更新尾指针位置
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data;
  }
}

// 串口接收中断处理
ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint8_t next_head;
  // 直接从串口流取出实时命令字符。这些字符不会被传递给主缓冲区，但是会设置系统状态标志位给实时执行
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    // 几个特殊控制命令字符
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      // 实时命令都是扩展字符
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // 丢掉没有识别的扩展字符，不会传递它给串口缓冲区
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        // 剩下的ascii码都存入缓冲区交给主程序处理
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }
        // 写入字符到缓冲区直到它满了为止
        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}

// 重置串口读缓冲区,非常简单让尾指针指向头指针
void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}
