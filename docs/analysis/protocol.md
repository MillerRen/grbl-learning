# 协议-主循环 

主循环`protocol_main_loop()`是在`protocol.c`中定义的一个接口，在`main.c`中被循环调用。主要的功能就是从 **GRBL串口接收环形队列** 读取字符串，经过处理后放入 **G代码行缓冲区**，然后交给**G代码解析器**，并且根据各个子模块状态机做出响应。

``` c
// protocol.c
// Define line flags. Includes comment type tracking and line overflow detection.
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)

#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif
static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.


/*
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop()
{
  // Perform some machine checks to make sure everything is good to go.
  #ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
      if (limits_get_state()) {
        sys.state = STATE_ALARM; // Ensure alarm state is active.
        report_feedback_message(MESSAGE_CHECK_LIMITS);
      }
    }
  #endif
  // Check for and report alarm state after a reset, error, or an initial power up.
  // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
  // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // Ensure alarm state is set.
  } else {
    // Check if the safety door is open.
    sys.state = STATE_IDLE;
    if (system_check_safety_door_ajar()) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // Enter safety door mode. Should return as IDLE state.
    }
    // All systems go!
    system_execute_startup(line); // Execute startup script.
  }

  // ---------------------------------------------------------------------------------
  // Primary loop! Upon a system abort, this exits back to main() to reset the system.
  // This is also where Grbl idles while waiting for something to do.
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached

        protocol_execute_realtime(); // Runtime command check point.
        if (sys.abort) { return; } // Bail to calling function upon system abort

        line[char_counter] = 0; // Set string termination character.
        #ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // Direct and execute one line of formatted input, and report status of execution.
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // Report line overflow error.
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // Empty or comment line. For syncing purposes.
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl '$' system command
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // Everything else is gcode. Block if in alarm or jog mode.
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // Parse and execute g-code block.
          report_status_message(gc_execute_line(line));
        }

        // Reset tracking data for next line.
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // Throw away all (except EOL) comment characters and overflow characters.
          if (c == ')') {
            // End of '()' comment. Resume line allowed.
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // Throw away whitepace and control characters
          } else if (c == '/') {
            // Block delete NOT SUPPORTED. Ignore character.
            // NOTE: If supported, would simply need to check the system if block delete is enabled.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
            // In the future, we could simply remove the items within the comments, but retain the
            // comment control characters, so that the g-code parser can error-check it.
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // TODO: Install '%' feature
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow and set flag.
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
  }

  return; /* Never reached */
}
```

## 安全检查

``` c
// 执行一些机器检查以确保一切正常。
  #ifdef CHECK_LIMITS_AT_INIT // 检查是否开启了初始化系统时检查限位开关，默认是开启的。
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) { // 检查是否设置了硬件限位功能
      if (limits_get_state()) { // 检查限位开关状态
        sys.state = STATE_ALARM; // 如果限位开关被触发，确保警报状态被激活。
        report_feedback_message(MESSAGE_CHECK_LIMITS); // 报告限位开关被触发的反馈消息
      }
    }
  #endif

  // 在重置、错误、或上电初始化之后检查并报告警报状态。
  // 注意： 睡眠模式禁用步进驱动器并且位置不能保持。步进驱动器的EN引脚释放，不能保持力矩。
  // 将睡眠状态重新初始化为警报模式确保让用户归位或让用户知道。
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) { // 如果系统处于STATE_ALARM或STATE_SLEEP状态
    report_feedback_message(MESSAGE_ALARM_LOCK); // 报告警报锁定消息
    sys.state = STATE_ALARM; // 确保警报状态被设置
  } else {
    // 检查安全门是否打开
    sys.state = STATE_IDLE;
    if (system_check_safety_door_ajar()) { // 检查安全门
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR); // 设置执行安全门标志位
      protocol_execute_realtime(); // 进入安全门模式。应该返回IDLE状态。
    }
    // 所有系统正常！
    system_execute_startup(line); // 执行启动脚本
  }
```
先执行限位开关、安全门等检查，执行初始化脚本（储存在EEPROM中的G代码），如果检查不通过会进入警报状态，这个状态会被运行时检查到从而阻止下一步动作，直到警报解除。

## 处理串口字符串

``` c
// ---------------------------------------------------------------------------------
  // 主循环! 系统终止后这回退出返回到main()去重置系统。
  // 这也是Grbl空闲等待其他事情的地方。
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0; // 初始化行标志位
  uint8_t char_counter = 0; // 初始化字符计数
  uint8_t c; // 声明放字符的变量
  for (;;) { // 无限循环

    // 处理一行从串口缓冲区到来的数据，如果数据可用的话。通过溢出空格和注释执行一个初始的过滤，并且大写所有字母。
    while((c = serial_read()) != SERIAL_NO_DATA) { // 从串口读取一个字节，直到遇到结束符
      if ((c == '\n') || (c == '\r')) { // 到达一行

        protocol_execute_realtime(); // 运行时命令检查点
        if (sys.abort) { return; } // 系统终止后退出函数

        line[char_counter] = 0; // 设置字符串结束符号
        #ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line); // 报告接收了多少行
        #endif

        // 直接执行格式化的输入行并报告执行状态
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // 报告行溢出错误
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // 空行或注释行。用于同步。
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl 系统命令 '$'
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // 其他的都是G代码。如果处于警报或点动模式就阻塞。
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // 解析并执行G代码块。
          report_status_message(gc_execute_line(line));
        }

        // 为下一行重置跟踪数据变量
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // 丢弃所有（除了行结束符）注释字符和溢出字符。
          if (c == ')') {
            // 匹配括号'()'包裹的注释。继续允许行数据。
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          // 丢弃空白字符和控制字符
          if (c <= ' ') {
            // 丢弃空白字符
          } else if (c == '/') {
            // 块删除不支持，忽略字符。
            // 注意：如果支持了，应该需要简单地检查系统如果启用了块删除的话。
          } else if (c == '(') {
            // 开启注释标志位并忽略所有字符，直到碰到')'或EOL
            // 注意：这没有完全遵守NIST定义，但是现在来说足够了。未来我们可能简单地移除注释项，但保留注释控制字符，
            // 因此G代码解析器可以进行错误检查。
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // 注意：';' 注释结束符是 LinuxCNC定义的，不是NIST定义的。
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // 代办：安装 '%' 功能
          // } else if (c == '%') {
            // 程序开始-结束百分比符号还不支持。
            // 注意：这可能被安装用来告诉Grbl一个程序相对输入运行的时间
            // 位置，间隔，系统自动循环开始继续执行所有事情直到下一个标记。
            // 这将会帮助修复带有明确功能的恢复问题，清空规划缓冲区用来准时执行任务
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // 检查行缓冲区溢出并设置标志位。
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // 字母改成大写
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }    
    
  }
```
一直从串口接收环形队列读取字符`serial_read()`，直到遇到串口数据结束符`SERIAL_NO_DATA`(0xff)也就是串口空了为止，然后把读取到的G代码行经过去掉注释和空格、 字母转换为大写后放入G代码行缓冲区`line[LINE_BUFFER_SIZE]`，以换行符`\n`或`\r`结尾，如果开头是以`$`开头的系统命令，则传递给命令处理器入口`system_execute_line()`，如果是G代码则传递给G代码解释器入口`gc_execute_line()`。行缓冲区大小`LINE_BUFFER_SIZE`设置为80个字符，超出这个字符数量会报行缓冲区溢出`STATUS_OVERFLOW`错误，标准要求是255个字符，但这里足够用了，如果你需要缓冲更多字符，可以修改`LINE_BUFFER_SIZE`为更大值。  

## 开始自动周期

``` c
// 如果在串口读缓冲区没有字符需要处理或执行，这会通知g代码流已填充到规划器缓冲区或已完成。
// 不管哪种情况，如果开启了自动循环，就会开始自动循环，队列就会移动。
protocol_auto_cycle_start();
```
G代码经过一系列处理后会进行一系列的信号加工，最后生成待执行的队列，默认设置了自动开启循环，会依次执行队列中的数据，并更新队列，这部分我们等到后面信号处理的时候在说，这也是一个初次看GRBL源码容易被忽略的地方导致分析断层。

## 终止信号

``` c
if (sys.abort) { return; } // 如果系统终止，返回到main()程序循环去重置系统。
```
如果碰到sys.abort信号就会退出主循环到main的循环中去重置系统（GRBL的状态机）。

## 运行时检查

``` c
protocol_execute_realtime();  // 运行时命令检查点。
```

由于GRBL没有实时操作系统，为了保证系统能够及时响应一些危险事件，在每一个可能会阻塞或长时间运行的地方，都安插了运行时命令检查点，这部分内容比较多我们单独章节分析。
