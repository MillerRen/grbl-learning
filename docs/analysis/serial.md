# 串口

  grbl使用串口从上位机接收信息，并通过串口反馈给上位机。共分为以下几部分：1.串口的配置，如波特率、停止位、校验位等；2.使用`环形队列` 作为缓冲器，用以匹配上位机和单片机的速度差异，分为输入缓冲器和输出缓冲器； 3.串口数据的简单分配，即分为实时响应和普通响应（放入队列）。

## 串口配置 

1. ### 串口初始化

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
**代码解析：** 

**UBRR0（串口波特率寄存器）：** 这是一个16位的寄存器，需要两次分别传入一个高字节`UBRR0H`和低字节`UBRR0L`,根据公式`UBBR0=(F_CPU/(4*BAUD_RATE)-1)/2`,F_CPU设置为16000000（跟硬件一致），BAUD_RATE配置为115200，得出结果为16.36111111111111取整后得到16，跟手册Table19-12中的波特率为115200时的值16一致（注1）。
**UCSR0A（串口控制及状态寄存器A）：** 在高波特率时（>57600）使能的波特率倍增器`U2X0`以减少误差，但是为了保证在较老的`Arduino`设备上禁用波特率倍增器用以兼容它们的bootloader。

**UCSR0B（串口控制及状态寄存器B）：** 使能串口接受`RXEN0`和串口发送功能`TXEN0`, 并使能串口接受完成中断`RXCIE0`，串口发送中断只在需要时开启。

**UCSR0C（串口控制及状态寄存器C）：** `UMSEL0`(串口模式选择位)，默认为00即异步串口， `UPMSEL0`(串口校验模式选择位)，默认为00即默认无奇偶校验。`USBS0`(串口停止位模式选择位)默认为0即1停止位。`UCSZ0`(串口字符长度寄存器)，默认为011即8位字符。这些都是默认值，不需要手动再配置。

2. ### 串口中断处理：

**串口接收中断：**

``` c
// 串口数据接收中断处理
ISR(SERIAL_RX)
{
  uint8_t data = UDR0; // 从串口数据寄存器取出数据
  uint8_t next_head; // 初始化下一个头指针

  // 直接从串行流中选取实时命令字符。这些字符不被传递到主缓冲区，但是它们设置了实时执行的系统状态标志位。
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // 调用运动控制重置程序
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // 状态报告
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // 循环开始
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // 进给保持
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

**代码解析：** 

AVR并没有实现中断功能，中断实现是由编译器`gcc-avr`完成的，具体用法是在中断函数前用`__attribute__((interrupt))`修饰，`ISR(SERIAL_RX)`是一个宏定义，它在`interrupt.h`中定义,功能是根据传入的中断向量号生成中断函数定义和函数声明。

开启了中断并设置了中断函数，一旦串口中接收到了一个字节数据，就会触发中断，从`UDR0`串口数据寄存器中取出数据后，会做简单区分，这里有三种类型的数据：
1. 实时命令，不会放入串口接收队列。
2. 实时覆盖命令，能实时调整部分参数，也不会放入串口接收队列。
3. 正常的G代码和系统命令，会放入串口接收队列。

**串口发送中断：** 

```c
// 数据寄存器为空的中断处理
ISR(SERIAL_UDRE)
{
  // 由于环形队列尾指针中断和主程序都会使用，有可能导致数据读取时，指针已经发生了变化，
  // 存在不稳定性，所以要用临时变量暂存，增加读取时的稳定性。
  uint8_t tail = serial_tx_buffer_tail; // 临时变量暂存 serial_tx_buffer_tail (为volatile优化)

  // 从缓冲区发送一个字节到串口
  UDR0 = serial_tx_buffer[tail];

  // 更新尾指针位置，如果已经到达顶端，返回初始位置，形成环形
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // 如果环形队列为空，关闭串口数据寄存器为空的中断，阻止继续发送串口流
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}
```

**代码解析：** 

`ISR(SERIAL_UDRE)`也是一个宏定义，展开后是根据串口数据为空的中断号定义的中断处理函数，发送中断的开启是在有数据需要返回给上位机时（如`serial_write`函数被调用时）使能`UDRIE0`（串口数据寄存器为空中断使能位）实现的的，`UCSR0B |=  (1 << UDRIE0);`，开启中断后如果数据发送寄存器为空会立即触发中断。随后把发送队列的数据放到`UDR0`（串口数据寄存器）发送给上位机。当串口发送队列没有数据时需要禁用串口发送中断，以防止误触发中断。需要注意的是串口接收和发送都是用的`UDR0`寄存器，这样不会导致接收发送冲突吗？不会，`UDR0`的接收和发送只是共享了寄存器地址，读和写是分离在不同的硬件上实现的。

## 环形队列

环形队列是在实际编程极为有用的数据结构,它是一个首尾相连的FIFO的数据结构，采用数组的线性空间,数据组织简单。能很快知道队列是否满为空。能以很快速度的来存取数据。

1. 缓冲：使用队列可以缓冲数据，提升收发数据的性能。

2. 高效：相比直线队列，空间利用率高。

3. 多任务：配合中断，串口和主循环可以在互不干扰的情况下独立工作。

grbl中的环形队列使用数组实现，使用两个指针标记队头队尾（不过grbl这里是反的），通过保持一个数据单元为空策略判断队列满和空。我制作了一个[演示程序](/demos/ringbuffer.html)，想不明白的可以实操试一试更容易理解。

### 串口接收环形队列

``` c
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#define RX_RING_BUFFER (RX_BUFFER_SIZE+1) // 定义接收缓冲区环形队列长度
uint8_t serial_rx_buffer[RX_RING_BUFFER]; // 定义串口接收环形队列
uint8_t serial_rx_buffer_head = 0; // 定义串口接收环形队列头指针
volatile uint8_t serial_rx_buffer_tail = 0; // 定义串口接收环形队列尾指针
```

定义了一个`RX_BUFFER_SIZE`大小(128字节)的串口接收环形队列`serial_rx_buffer`，并使用了队头`serial_rx_buffer_head`和队尾`serial_rx_buffer_tail`两个指针记录队列状态。

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
`serial_read`一个读取串口环形队列的接口：这个接口在主循环中调用，它从串口接收一个字节就更新一下队尾的指针，因为使用的是数组，指针到达数组尾部要返回数组头部形成环形，如果队列是空的`serial_rx_buffer_head == tail`，就返回结束符号`0xff`。

### 串口发送环形队列

```c
#define TX_BUFFER_SIZE 104 // 定义串口发送缓冲区大小
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1) // 定义发送缓冲区队列长度

int8_t serial_tx_buffer[TX_RING_BUFFER]; // 定义串口发送环形队列
uint8_t serial_tx_buffer_head = 0; // 定义串口发送环形队列头指针
volatile uint8_t serial_tx_buffer_tail = 0; // 定义串口发送环形队列尾指针
```

定义了一个大小为`TX_BUFFER_SIZE`(104)的串口发送环形队列`serial_tx_buffer`，并使用了队头`serial_tx_buffer_head`和队尾`serial_tx_buffer_tail`两个指针记录队列状态。

```c
// 写入一个字节到串口发送缓冲区。被主程序调用。
void serial_write(uint8_t data) {
  // 计算下一个头指针，如果已经到达最大值，移到开始，形成环形
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // 等待，直到缓冲区有空间
  while (next_head == serial_tx_buffer_tail) {
    // 代办：重构st_prep_tx_buffer()调用，在长打印期间在这里执行。
    if (sys_rt_exec_state & EXEC_RESET) { return; } // 只检查终止防止死循环。
  }

  // 储存数据并向前移动头指针
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // 开启数据寄存器为空的中断，确保串口发送流运行。
  // 只要环形队列有空间，就可以持续不断地从串口接收数据。
  UCSR0B |=  (1 << UDRIE0);
}
```
`serial_write`一个串口写入接口，这个接口主要被反馈报告程序调用，报告程序把字符串格式化之后传入这个接口，随后把传进来的数据放入发送队列，如果队列满了就一直等着，直到队列数据被串口取出留出空间，最后开启串口数据寄存器为空的中断，开启串口发送处理中断，由`ISR(SERIAL_UDRE)`将队列中的数据发送给上位机。
