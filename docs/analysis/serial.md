# 串口

  grbl使用串口从上位机接收信息，并通过串口反馈给上位机，它使用各自的 `环形队列` 作为缓冲器，用以匹配不同系统的处理能力。

## 环形队列

环形队列是在实际编程极为有用的数据结构,它是一个首尾相连的FIFO的数据结构，采用数组的线性空间,数据组织简单。能很快知道队列是否满为空。能以很快速度的来存取数据。

1. 缓冲：使用队列可以缓冲数据，提升收发数据的性能。

2. 高效：相比直线队列，空间利用率高。

3. 多任务：配合中断，串口和主循环可以在互不干扰的情况下独立工作。

grbl中的环形队列使用数组实现，使用两个指针标记队头队尾（不过grbl这里是反的），通过保持一个粗存单元为空策略判断队列满和空。

- 队列满：tail+1==head
- 队列空：head==tail

下面以大小为8字节的队列进行说明：

1. 初始状态：

 | head+tail |   |   |   |   |   |   |   |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 |   |   |   |  |   |   |   |   |  

此时head==tail，队列为空。

2. 插入一个元素：
判断tail+1!=head,数据插入队尾tail的位置, tail++。

 | head | tail  |   |   |   |   |   |   |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 | *  |   |   |  |   |   |   |   |

3. 再插入一个元素：
判断tail+1!=head,数据插入队尾tail的位置, tail++。

 | head |   | tail |   |   |   |   |   |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 | *  | * |   |  |   |   |   |   |

4. 取出一个元素：
判断head!=tail队列不为空,取出head位置的值, head++。

 |  | head | tail |   |   |   |   |   |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 |    | * |   |  |   |   |   |   |

5. 继续插入元素：
判断tail+1!=head,数据插入队尾tail的位置。

 |  | head  |   |   |   |   |   | tail |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 |    | * | * | * | * | * | * |   |

6. 继续插入元素（达到数组最大值）：
判断tail+1!=head,数据插入队尾tail的位置，这时tail+1超过了数组索引值size，要返回来设置tail=0,这样就形成了环形。

 | tail | head  |   |   |   |   |   |   |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 |    | * | * | * | * | * | * | * |

7. 取出元素：
判断tail!=head,head++。

 | tail |   |   |   |   |   |   | head |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 |    |   |   |   |   |   |   | * |

8. 继续取出元素（达到数组最大值）：
判断tail!=head,head+1这时head超过了数组索引值size，要返回来设置head=0,这样就形成了环形。

 | tail+head |   |   |   |   |   |   |   |
 | --- | --- | --- | --- | --- | --- | --- | --- |
 |   |   |  |   |   |   |   |   |

## 源代码

串口的实现是在`serial.c`模块中完成的，它基于 **环形队列** 用 **中断** 方式分别实现了串口接收缓冲器和串口发送缓冲器，接收中断时把数据放入串口接收缓冲区，发送为空中断时，从串口发送缓冲区取一个字节发送到串口。

## 串口初始化

串口(USART)是通用异步串行接收发送标准化接口，因为是异步的，所以要定义好波特率、停止位、校验位等参数，两个设备波特率一致才能正常通信。

与串口相关的寄存器：

- UCSRnA and UCSRnB and UCSRnC: USART Control and Status Register n A 串口控制与状态寄存器

- UDRn: USART I/O Data Register n 串口IO数据寄存器

- UBRRnL and UBRRnH – USART Baud Rate Registers 串口波特率寄存器

grbl现在默认的波特率是115200，现在电脑和单片机的性能已经够好，没必要再使用9600了，并且这个速度对已经足够用了。根据官方给出的波特率计算公式计算得出`ubrrn = fosc/(8*baud)-1。16000000/(8*115200)-1 = 16.36111111111111`取整后得到值为16(查表也能得到)，由于计算出来的值有可能大于8位所以需要两个8位寄存器接收波特率值,高波特率需要开启倍频。默认的8位无校验位1停止位就可以了，没必要重新配置。最后使能串口接收和串口发送中断。

``` c
// 串口初始化
void serial_init()
{
  // 设置波特率
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // 关闭波特率倍增器。 - 只在Uno xxx上需要。
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // 波特率高的波特率倍增器开启，即115200
  #endif
  // 波特率是比较大的数字，需要两个8位寄存器存放
  UBRR0H = UBRR0_value >> 8; // 高8位右移到低8位，放入高8位寄存器，右移不会改变源数值
  UBRR0L = UBRR0_value; // 第八位直接放入低8位寄存器

  // 启用接收，发送和接收完成一个字节的中断
  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

  // 默认协议是8位，无奇偶校验，1个停止位
}
```

## 接收缓冲器

首先定义了一个`RX_BUFFER_SIZE`大小(128字节)的环形队列，并使用了队头和队尾两个指针记录队列状态：

``` c
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#define RX_RING_BUFFER (RX_BUFFER_SIZE+1) // 定义接收缓冲区环形队列长度
uint8_t serial_rx_buffer[RX_RING_BUFFER]; // 定义串口接收环形队列
uint8_t serial_rx_buffer_head = 0; // 定义串口接收环形队列头指针
volatile uint8_t serial_rx_buffer_tail = 0; // 定义串口接收环形队列尾指针
```

一个读取缓冲器的接口：这个接口在主循环中调用，它从串口接收一个字节就更新一下队尾的指针，因为使用的是数组，指针到达数组尾部要返回数组头部形成环形，如果队列是空的`serial_rx_buffer_head == tail`，就返回结束符号`0xff`。

``` c
// 获取串口接收缓冲区的第一个字节。被主程序调用。
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // 临时变量暂存 serial_rx_buffer_tail (优化volatile)
  if (serial_rx_buffer_head == tail) { // 如果接收环形队列为空，则设置结束符号
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail]; // 从接受环形队列取一个字节

    tail++; // 更新尾指针
    if (tail == RX_RING_BUFFER) { tail = 0; } // 环形
    serial_rx_buffer_tail = tail;

    return data;
  }
}
```

串口接收数据是在串口接收的中断中处理的,一旦串口中接收到了一个字节数据，就会触发中断，从串口数据寄存器中取出数据后，会做简单区分，这里有三种类型的数据：

1. 实时命令，不会放入缓冲区
2. 实时覆盖命令，能实时调整部分参数，也不会放入串口接收缓冲器
3. 正常的G代码和系统命令，会放入串口接收缓冲器。

``` c
// 串口数据接收中断处理
ISR(SERIAL_RX)
{
  uint8_t data = UDR0; // 从串口数据寄存器取出数据
  uint8_t next_head; // 初始化下一个头指针

  // 直接从串行流中选取实时命令字符。这些字符不被传递到主缓冲区，但是它们设置了实时执行的系统状态标志位。
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // 调用运动控制重置程序
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // 设置为 true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // 设置为 true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // 设置为 true
    default :
      if (data > 0x7F) { // 实时控制都是扩展的ASCII字符
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // 设置为 true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // 阻止所有其他状态，调用运动取消。
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          // 以下为实时覆盖命令
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
        // 除了上面已知的实时命令，其他的ASCII扩展字符都被丢掉
      } else { // 其他的字符被认为都是G代码，会被写入到主缓冲区
        next_head = serial_rx_buffer_head + 1; // 更新临时头指针
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // 写入到接收缓冲区，直到它满了为止。
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}
```
