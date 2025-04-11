# 协议-状态报告

`grbl`报告程序`report.c`的主要作用是返回格式化的消息给上位机，包含状态信息、警报信息、回馈信息及配置信息。它并没有使用c语言标准库`stdio.h`的`printf`函数，而是自己实现了一套格式化打印程序`print.c`，目的是为了减少程序体积。最后调用底层串口接口`serial_write`实现输出。

## 自定义打印
grbl中打印主要是字符串、按2进制，10进制打印`uint8_t` `uint16_t` `uint32_t` `float`等类型，通过几个函数实现了这些数据类型的语义化输出，这些实现都比较简单很容易读懂源码。

**打印字符串：** 移动指针，逐个打印字符。

```c
// print.c
// 打印字符串
void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}
```
**打印uint8_t：**
这里使用`'0'+n`的方式把数字变成了可读的字符数字，如果有高位就把它除以响应的位数，得到单字符（0-9）后再输出。

```c
//打印基数为10的uint8变量。
void print_uint8_base10(uint8_t n)
{
  uint8_t digit_a = 0;
  uint8_t digit_b = 0;
  if (n >= 100) { // 100-255
    digit_a = '0' + n % 10;
    n /= 10;
  }
  if (n >= 10) { // 10-99
    digit_b = '0' + n % 10;
    n /= 10;
  }
  serial_write('0' + n);
  if (digit_b) { serial_write(digit_b); }
  if (digit_a) { serial_write(digit_a); }
}
```

**打印uint32_t：** 打印`uint32_t`类型的数据跟`uint8_t`类型的数据差不多，但是考虑到`uint32_t`的10进制数有10位数字（4294967296）每个都判断一次会有大量重复代码，因此定义了一个数组保存数位上的字符，循环执行除以10和取余操作。
```c
// print.c
// 以10进制显示uint32_t类型的数据
void print_uint32_base10(uint32_t n)
{
  if (n == 0) {
    serial_write('0');
    return;
  }

  unsigned char buf[10];
  uint8_t i = 0;

  while (n > 0) {
    buf[i++] = n % 10;
    n /= 10;
  }

  for (; i > 0; i--)
    serial_write('0' + buf[i-1]);
}
```

**打印整数：** 打印整数的帮助函数，主要包含了负整数的判断。
```c
// 打印整数
void printInteger(long n)
{
  if (n < 0) {
    serial_write('-');
    print_uint32_base10(-n);
  } else {
    print_uint32_base10(n);
  }
}
```

**打印浮点数：** 打印浮点数跟整数也类似，只是小数部分是乘以10，然后在对应位置输出小数点`.`。
```c
//通过立即转换为长整数，将浮点转换为字符串，长整数包含的数字比浮点多。
//由计数器跟踪的小数位数可由用户设置。然后将整数有效地转换为字符串。
//注意：AVR“%”和“/”整数操作非常有效。位移加速技术实际上只是稍微慢一点。我是通过艰苦的努力才发现这一点的。
void printFloat(float n, uint8_t decimal_places)
{
  if (n < 0) {
    serial_write('-');
    n = -n;
  }

  uint8_t decimals = decimal_places;
  while (decimals >= 2) { //快速将预期为E0的值转换为E-4。
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // 添加舍入因子。 确保整个值的进位。

  //向后生成数字并以字符串形式存储。
  unsigned char buf[13];
  uint8_t i = 0;
  uint32_t a = (long)n;
  while(a > 0) {
    buf[i++] = (a % 10) + '0'; //获取数字
    a /= 10;
  }
  while (i < decimal_places) {
     buf[i++] = '0'; //将零填入小数点（n<1）
  }
  if (i == decimal_places) { //如果需要，填写前导零。
    buf[i++] = '0';
  }

  //打印生成的字符串。
  for (; i > 0; i--) {
    if (i == decimal_places) { serial_write('.'); } //在正确的位置插入小数点。
    serial_write(buf[i-1]);
  }
}
```

**打印静态字符串：** 比较有意思的是它为了节约内存资源，把常量字符串放到了flash中，通过引用flash中字符串头指针一个一个地打印，而不是把整个字符串放到内存中再打印，这是一个不错的技巧,不过缺点是没有内存快并且没有办法修改。我们看下源码：
```c
// pgmspace.h
# define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];}))
```
上面代码中`PSTR(s)`的作用是声明字符串`s`为静态常量，存储位置为`PROGMEM`，并返回字符串的首地址指针。

```c
// print.c
//打印存储在PGM内存中的字符串
void printPgmString(const char *s)
{
  char c;
  while ((c = pgm_read_byte_near(s++)))
    serial_write(c);
}
```
打印储存在flash中的字符串。

```c
// pgmspace.h
#define __LPM_enhanced__(addr)  \
(__extension__({                \
    uint16_t __addr16 = (uint16_t)(addr); \
    uint8_t __result;           \
    __asm__ __volatile__        \
    (                           \
        "lpm %0, Z" "\n\t"      \
        : "=r" (__result)       \
        : "z" (__addr16)        \
    );                          \
    __result;                   \
}))
#define __LPM(addr)         __LPM_enhanced__(addr)
#define pgm_read_byte_near(address_short) __LPM((uint16_t)(address_short))
```
上面代码最终展开为通过汇编指令`lpm`获得flash中的对应静态常量字符串指针处的字符，循环打印字符串。


## 状态报告

**报告状态消息** 
```c
// report.c
// 报告状态消息
void report_status_message(uint8_t status_code)
{
  switch(status_code) {
    case STATUS_OK: // STATUS_OK
      printPgmString(PSTR("ok\r\n")); break;
    default:
      printPgmString(PSTR("error:"));
      print_uint8_base10(status_code);
      report_util_line_feed();
  }
}
```
上面代码用来拼接状态消息，如果状态正常就返回`ok\r\n`,异常则返回错误及错误代码如`error:ERROR_CODE\r\n`。这是处理流式接口和人工反馈的主要确认协议响应。对于输入的每一行数据，此方法都会对成功的命令或命令进行“ok”响应，“error:”用于指示线路发生的某个错误事件或在运行期间发生的某个关键系统错误，活动错误事件可以源于g代码解析器、设置模块或来自异步严重错误，例如触发的硬限制。接口应始终监视这些响应。

**报告警报信息** 
```c
// report.c
// 打印警报信息。
void report_alarm_message(uint8_t alarm_code)
{
  printPgmString(PSTR("ALARM:"));
  print_uint8_base10(alarm_code);
  report_util_line_feed();
  delay_ms(500); //强制延迟以确保消息清空串行写入队列。
}
```
上面代码用来拼接警报消息。

**报告反馈消息** 
```c
// report.c
// 报告反馈消息
void report_feedback_message(uint8_t message_code)
{
  printPgmString(PSTR("[MSG:"));
  switch(message_code) {
    case MESSAGE_CRITICAL_EVENT:
      printPgmString(PSTR("Reset to continue")); break;
    case MESSAGE_ALARM_LOCK:
      printPgmString(PSTR("'$H'|'$X' to unlock")); break;
    case MESSAGE_ALARM_UNLOCK:
      printPgmString(PSTR("Caution: Unlocked")); break;
    case MESSAGE_ENABLED:
      printPgmString(PSTR("Enabled")); break;
    case MESSAGE_DISABLED:
      printPgmString(PSTR("Disabled")); break;
    case MESSAGE_SAFETY_DOOR_AJAR:
      printPgmString(PSTR("Check Door")); break;
    case MESSAGE_CHECK_LIMITS:
      printPgmString(PSTR("Check Limits")); break;
    case MESSAGE_PROGRAM_END:
      printPgmString(PSTR("Pgm End")); break;
    case MESSAGE_RESTORE_DEFAULTS:
      printPgmString(PSTR("Restoring defaults")); break;
    case MESSAGE_SPINDLE_RESTORE:
      printPgmString(PSTR("Restoring spindle")); break;
    case MESSAGE_SLEEP_MODE:
      printPgmString(PSTR("Sleeping")); break;
  }
  report_util_feedback_line_feed();
}
```
上面代码用来拼接反馈信息。这是一种集中的方法，用于为不属于状态/报警消息协议的内容提供额外的用户反馈。这些消息包括设置警告、开关切换以及如何退出警报。注意：对于接口，消息总是放在括号内。

**报告实时信息** 
```c
// report.c
// 打印实时数据。
void report_realtime_status()
{
  uint8_t idx;
  int32_t current_position[N_AXIS]; //复制系统位置变量的当前状态
  memcpy(current_position,sys_position,sizeof(sys_position));
  float print_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position,current_position);

  //报告当前机器状态和子状态
  serial_write('<');
  switch (sys.state) {
    case STATE_IDLE: printPgmString(PSTR("Idle")); break;
    case STATE_CYCLE: printPgmString(PSTR("Run")); break;
    case STATE_HOLD:
      if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
        printPgmString(PSTR("Hold:"));
        if (sys.suspend & SUSPEND_HOLD_COMPLETE) { serial_write('0'); } //准备好恢复
        else { serial_write('1'); } // 激活保持
        break;
      } //在点动取消期间继续打印点动状态。
    case STATE_JOG: printPgmString(PSTR("Jog")); break;
    case STATE_HOMING: printPgmString(PSTR("Home")); break;
    case STATE_ALARM: printPgmString(PSTR("Alarm")); break;
    case STATE_CHECK_MODE: printPgmString(PSTR("Check")); break;
    case STATE_SAFETY_DOOR:
      printPgmString(PSTR("Door:"));
      if (sys.suspend & SUSPEND_INITIATE_RESTORE) {
        serial_write('3'); //恢复
      } else {
        if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
          if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) {
            serial_write('1'); // 门打开
          } else {
            serial_write('0');
          } //门关上，准备恢复
        } else {
          serial_write('2'); //收回
        }
      }
      break;
    case STATE_SLEEP: printPgmString(PSTR("Sleep")); break;
  }

  float wco[N_AXIS];
  if (bit_isfalse(settings.status_report_mask,BITFLAG_RT_STATUS_POSITION_TYPE) ||
      (sys.report_wco_counter == 0) ) {
    for (idx=0; idx< N_AXIS; idx++) {
      //将工作坐标偏移和刀具长度偏移应用到当前位置。
      wco[idx] = gc_state.coord_system[idx]+gc_state.coord_offset[idx];
      if (idx == TOOL_LENGTH_OFFSET_AXIS) { wco[idx] += gc_state.tool_length_offset; }
      if (bit_isfalse(settings.status_report_mask,BITFLAG_RT_STATUS_POSITION_TYPE)) {
        print_position[idx] -= wco[idx];
      }
    }
  }

  //报告机器位置
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_POSITION_TYPE)) {
    printPgmString(PSTR("|MPos:"));
  } else {
    printPgmString(PSTR("|WPos:"));
  }
  report_util_axis_values(print_position);

  //返回计划器和串行读取缓冲区状态。
  #ifdef REPORT_FIELD_BUFFER_STATE
    if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_BUFFER_STATE)) {
      printPgmString(PSTR("|Bf:"));
      print_uint8_base10(plan_get_block_buffer_available());
      serial_write(',');
      print_uint8_base10(serial_get_rx_buffer_available());
    }
  #endif

  #ifdef USE_LINE_NUMBERS
    #ifdef REPORT_FIELD_LINE_NUMBERS
      //报告当前行号
      plan_block_t * cur_block = plan_get_current_block();
      if (cur_block != NULL) {
        uint32_t ln = cur_block->line_number;
        if (ln > 0) {
          printPgmString(PSTR("|Ln:"));
          printInteger(ln);
        }
      }
    #endif
  #endif

  //报告实时进给速度
  #ifdef REPORT_FIELD_CURRENT_FEED_SPEED
    #ifdef VARIABLE_SPINDLE
      printPgmString(PSTR("|FS:"));
      printFloat_RateValue(st_get_realtime_rate());
      serial_write(',');
      printFloat(sys.spindle_speed,N_DECIMAL_RPMVALUE);
    #else
      printPgmString(PSTR("|F:"));
      printFloat_RateValue(st_get_realtime_rate());
    #endif      
  #endif

  #ifdef REPORT_FIELD_PIN_STATE
    uint8_t lim_pin_state = limits_get_state();
    uint8_t ctrl_pin_state = system_control_get_state();
    uint8_t prb_pin_state = probe_get_state();
    if (lim_pin_state | ctrl_pin_state | prb_pin_state) {
      printPgmString(PSTR("|Pn:"));
      if (prb_pin_state) { serial_write('P'); }
      if (lim_pin_state) {
        #ifdef ENABLE_DUAL_AXIS
          #if (DUAL_AXIS_SELECT == X_AXIS)
            if (bit_istrue(lim_pin_state,(bit(X_AXIS)|bit(N_AXIS)))) { serial_write('X'); }
            if (bit_istrue(lim_pin_state,bit(Y_AXIS))) { serial_write('Y'); }
          #endif
          #if (DUAL_AXIS_SELECT == Y_AXIS)
            if (bit_istrue(lim_pin_state,bit(X_AXIS))) { serial_write('X'); }
            if (bit_istrue(lim_pin_state,(bit(Y_AXIS)|bit(N_AXIS)))) { serial_write('Y'); }
          #endif
          if (bit_istrue(lim_pin_state,bit(Z_AXIS))) { serial_write('Z'); }
        #else
          if (bit_istrue(lim_pin_state,bit(X_AXIS))) { serial_write('X'); }
          if (bit_istrue(lim_pin_state,bit(Y_AXIS))) { serial_write('Y'); }
          if (bit_istrue(lim_pin_state,bit(Z_AXIS))) { serial_write('Z'); }
        #endif
      }
      if (ctrl_pin_state) {
        #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
          if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_SAFETY_DOOR)) { serial_write('D'); }
        #endif
        if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_RESET)) { serial_write('R'); }
        if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_FEED_HOLD)) { serial_write('H'); }
        if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_CYCLE_START)) { serial_write('S'); }
      }
    }
  #endif

  #ifdef REPORT_FIELD_WORK_COORD_OFFSET
    if (sys.report_wco_counter > 0) { sys.report_wco_counter--; }
    else {
      if (sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)) {
        sys.report_wco_counter = (REPORT_WCO_REFRESH_BUSY_COUNT-1); // Reset counter for slow refresh
      } else { sys.report_wco_counter = (REPORT_WCO_REFRESH_IDLE_COUNT-1); }
      if (sys.report_ovr_counter == 0) { sys.report_ovr_counter = 1; } // Set override on next report.
      printPgmString(PSTR("|WCO:"));
      report_util_axis_values(wco);
    }
  #endif

  #ifdef REPORT_FIELD_OVERRIDES
    if (sys.report_ovr_counter > 0) { sys.report_ovr_counter--; }
    else {
      if (sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)) {
        sys.report_ovr_counter = (REPORT_OVR_REFRESH_BUSY_COUNT-1); // Reset counter for slow refresh
      } else { sys.report_ovr_counter = (REPORT_OVR_REFRESH_IDLE_COUNT-1); }
      printPgmString(PSTR("|Ov:"));
      print_uint8_base10(sys.f_override);
      serial_write(',');
      print_uint8_base10(sys.r_override);
      serial_write(',');
      print_uint8_base10(sys.spindle_speed_ovr);

      uint8_t sp_state = spindle_get_state();
      uint8_t cl_state = coolant_get_state();
      if (sp_state || cl_state) {
        printPgmString(PSTR("|A:"));
        if (sp_state) { // != SPINDLE_STATE_DISABLE
          #ifdef VARIABLE_SPINDLE 
            #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
              serial_write('S'); // CW
            #else
              if (sp_state == SPINDLE_STATE_CW) { serial_write('S'); } // CW
              else { serial_write('C'); } // CCW
            #endif
          #else
            if (sp_state & SPINDLE_STATE_CW) { serial_write('S'); } // CW
            else { serial_write('C'); } // CCW
          #endif
        }
        if (cl_state & COOLANT_STATE_FLOOD) { serial_write('F'); }
        #ifdef ENABLE_M7
          if (cl_state & COOLANT_STATE_MIST) { serial_write('M'); }
        #endif
      }  
    }
  #endif

  serial_write('>');
  report_util_line_feed();
}
```
此函数获取步进子程序的实时快照和数控机床的实际位置。用户可以根据自己的具体需要更改以下功能，但所需的实时数据报告必须尽可能短。这是必需的，因为它将计算开销降至最低，并允许grbl保持平稳运行，特别是在具有快速、短线段和高频报告（5-20Hz）的g代码程序期间。这个函数在`protocol.c`中调用，当上位机发送`?`给`grbl`时，就会返回实时信息给上位机。

**报告构建时信息** 
```c
//报告构建信息
void report_build_info(char *line)
{
  printPgmString(PSTR("[VER:" GRBL_VERSION "." GRBL_VERSION_BUILD ":"));
  printString(line);
  report_util_feedback_line_feed();
  printPgmString(PSTR("[OPT:")); //生成编译时生成选项列表
  #ifdef VARIABLE_SPINDLE
    serial_write('V');
  #endif
  #ifdef USE_LINE_NUMBERS
    serial_write('N');
  #endif
  #ifdef ENABLE_M7
    serial_write('M');
  #endif
  #ifdef COREXY
    serial_write('C');
  #endif
  #ifdef PARKING_ENABLE
    serial_write('P');
  #endif
  #ifdef HOMING_FORCE_SET_ORIGIN
    serial_write('Z');
  #endif
  #ifdef HOMING_SINGLE_AXIS_COMMANDS
    serial_write('H');
  #endif
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES
    serial_write('T');
  #endif
  #ifdef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
    serial_write('A');
  #endif
  #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
    serial_write('D');
  #endif
  #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
    serial_write('0');
  #endif
  #ifdef ENABLE_SOFTWARE_DEBOUNCE
    serial_write('S');
  #endif
  #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    serial_write('R');
  #endif
  #ifndef HOMING_INIT_LOCK
    serial_write('L');
  #endif
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    serial_write('+');
  #endif  
  #ifndef ENABLE_RESTORE_EEPROM_WIPE_ALL //注意：禁用时显示。
    serial_write('*');
  #endif
  #ifndef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS //注意：禁用时显示。
    serial_write('$');
  #endif
  #ifndef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS //注意：禁用时显示。
    serial_write('#');
  #endif
  #ifndef ENABLE_BUILD_INFO_WRITE_COMMAND //注意：禁用时显示。
    serial_write('I');
  #endif
  #ifndef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE //注意：禁用时显示。
    serial_write('E');
  #endif
  #ifndef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE //注意：禁用时显示。
    serial_write('W');
  #endif
  #ifdef ENABLE_DUAL_AXIS
    serial_write('2');
  #endif
  //注意：编译后的值，如覆盖增量/最大/最小值，可能会在以后的某个时间添加。
  serial_write(',');
  print_uint8_base10(BLOCK_BUFFER_SIZE-1);
  serial_write(',');
  print_uint8_base10(RX_BUFFER_SIZE);

  report_util_feedback_line_feed();
}
```
此函数报告构建时信息，当上位机发送`$I`给`grbl`时，返回构建时功能开关，用以查询机器支持的功能，如变速主轴、CoreXY结构等。

**报告启动脚本**
```c
//打印指定的启动行
void report_startup_line(uint8_t n, char *line)
{
  printPgmString(PSTR("$N"));
  print_uint8_base10(n);
  serial_write('=');
  printString(line);
  report_util_line_feed();
}

void report_execute_startup_message(char *line, uint8_t status_code)
{
  serial_write('>');
  printString(line);
  serial_write(':');
  report_status_message(status_code);
}
```
上面两个函数用于报告启动脚本和启动脚本执行结果。上位机可以发送`$N`查询启动脚本`$N=xxx`设置启动脚本。

**报告欢迎消息**
```c
//欢迎辞
void report_init_message()
{
  printPgmString(PSTR("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n"));
}

//Grbl帮助消息
void report_grbl_help() {
  printPgmString(PSTR("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H ~ ! ? ctrl-x]\r\n"));    
}
```
`report_init_message`会在grbl重置后返回给上位机，报告grbl的版本信息，并告诉用户可以发送`$`命令查询帮助。上位机发送`$`命令给grbl后，`report_grbl_help`会返回帮助信息告诉用户还有哪些命令可以使用。

**报告全局设置**
```c
//Grbl全局设置打印输出。
//注：此处的编号方案必须与存储在settings.c中相关
void report_grbl_settings() {
  //打印Grbl设置。
  report_util_uint8_setting(0,settings.pulse_microseconds);
  report_util_uint8_setting(1,settings.stepper_idle_lock_time);
  report_util_uint8_setting(2,settings.step_invert_mask);
  report_util_uint8_setting(3,settings.dir_invert_mask);
  report_util_uint8_setting(4,bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
  report_util_uint8_setting(5,bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS));
  report_util_uint8_setting(6,bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN));
  report_util_uint8_setting(10,settings.status_report_mask);
  report_util_float_setting(11,settings.junction_deviation,N_DECIMAL_SETTINGVALUE);
  report_util_float_setting(12,settings.arc_tolerance,N_DECIMAL_SETTINGVALUE);
  report_util_uint8_setting(13,bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
  report_util_uint8_setting(20,bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE));
  report_util_uint8_setting(21,bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
  report_util_uint8_setting(22,bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
  report_util_uint8_setting(23,settings.homing_dir_mask);
  report_util_float_setting(24,settings.homing_feed_rate,N_DECIMAL_SETTINGVALUE);
  report_util_float_setting(25,settings.homing_seek_rate,N_DECIMAL_SETTINGVALUE);
  report_util_uint8_setting(26,settings.homing_debounce_delay);
  report_util_float_setting(27,settings.homing_pulloff,N_DECIMAL_SETTINGVALUE);
  report_util_float_setting(30,settings.rpm_max,N_DECIMAL_RPMVALUE);
  report_util_float_setting(31,settings.rpm_min,N_DECIMAL_RPMVALUE);
  #ifdef VARIABLE_SPINDLE
    report_util_uint8_setting(32,bit_istrue(settings.flags,BITFLAG_LASER_MODE));
  #else
    report_util_uint8_setting(32,0);
  #endif
  //打印轴设置
  uint8_t idx, set_idx;
  uint8_t val = AXIS_SETTINGS_START_VAL;
  for (set_idx=0; set_idx<AXIS_N_SETTINGS; set_idx++) {
    for (idx=0; idx<N_AXIS; idx++) {
      switch (set_idx) {
        case 0: report_util_float_setting(val+idx,settings.steps_per_mm[idx],N_DECIMAL_SETTINGVALUE); break;
        case 1: report_util_float_setting(val+idx,settings.max_rate[idx],N_DECIMAL_SETTINGVALUE); break;
        case 2: report_util_float_setting(val+idx,settings.acceleration[idx]/(60*60),N_DECIMAL_SETTINGVALUE); break;
        case 3: report_util_float_setting(val+idx,-settings.max_travel[idx],N_DECIMAL_SETTINGVALUE); break;
      }
    }
    val += AXIS_SETTINGS_INCREMENT;
  }
}
```
此函数报告全局设置信息，上位机可以通过发送类似`$1`命令查询设置，`$1=xxx`命令修改设置，`$$`查询所有全局设置。注意这里的顺序必须跟`settings.c`中的顺序一致，否则上位机显示会错误。

**报告对刀设置**
```c
//打印当前探针参数。
void report_probe_parameters()
{
  //根据机器位置进行报告。
  printPgmString(PSTR("[PRB:"));
  float print_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position,sys_probe_position);
  report_util_axis_values(print_position);
  serial_write(':');
  print_uint8_base10(sys.probe_succeeded);
  report_util_feedback_line_feed();
}
```
在执行探测命令时，这些参数将在探测成功或探测失败时使用G38.3 无错误命令（如果支持）进行更新。这些值将一直保留，直到Grbl电源重启，从而重新归零。