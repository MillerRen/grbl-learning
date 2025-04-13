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

#include "grbl.h"

// 定义行标志位。包括注释类型跟踪和行溢出检查。
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)


static char line[LINE_BUFFER_SIZE]; // 被执行的行。以零为结束符。

static void protocol_exec_rt_suspend();


/*
  GRBL 主循环:
*/
void protocol_main_loop()
{
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
    report_feedback_message(MESSAGE_ALARM_LOCK); // 报告警报锁定小小
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


    // 如果在串口读缓冲区没有字符需要处理或执行，这会通知g代码流已填充到规划器缓冲区或已完成。
    // 不管哪种情况，如果开启了自动循环，就会开始自动循环，队列就会移动。
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // 运行时命令检查点。
    if (sys.abort) { return; } // 如果系统终止，返回到main()程序循环去重置系统。
  }

  return; /* 永远不会到达 */
}


// 阻塞直到所有缓冲的步数被执行或处于循环状态。同步期间，进给保持时可能会发生。并且等待循环结束。
void protocol_buffer_synchronize()
{
  // 如果系统进入队列，确保循环继续如果自动循环标志位提供了。
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // 检查并执行运行时命令。
    if (sys.abort) { return; } // 检查系统是否终止
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


// 如果有一个运动主备好执行并且主程序没有激活解析命令，触发开始自动循环。
// 注意：这个函数仅被主循环，缓冲区同步和mc_line调用和执行，当其中一个条件存在时。
// 没有代码块发送（列入流完成了，单个命令）.
// 一个命令需要等待在缓冲区的运动去执行缓冲区同步，
// 或者规划器缓冲区满了准备就绪。
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // 检查缓冲区是否有块
    system_set_exec_state_flag(EXEC_CYCLE_START); // 如果有就执行他们
  }
}


// 这个函数是给Grbl的实时命令执行系统的一个通用接口。它被主程序中一系列的检查点调用，主要是有可能有一个while
// 循环等待一个缓冲区清空空间或一个上一次检查点执行时间超过一秒。这是一个异步执行实时命令(又叫多任务)的方式。
// 这是一种使用grbl的g代码解析和规划功能异步执行实时命令（又称多任务）的方法。
// 此函数还用作中断的接口，用于设置系统实时标志，其中只有主程序处理这些标志，无需定义计算成本更高的易失性变量。
// 这还提供了一种受控的方式来执行某些任务，而不必拥有同一任务的两个或多个实例，
// 规划器在进给保持或覆盖参数时重新计算缓冲区。
// 注意：sys_rt_exec_state变量标志被任意进程设置，步进或串口中断，引出引脚，限位开关，或主程序。
void protocol_execute_realtime()
{
  protocol_exec_rt_system(); // 执行运行时命令
  if (sys.suspend) {  // 如果需要暂停
    protocol_exec_rt_suspend(); // 执行暂停
  }
}


// 执行运行时命令，如果需要的话。这个函数主要操作是作为Grbl的状态机和控制一系列实时功能。
// 注意：不要修改这里除非你明确直到自己在做什么！
// 注意：不要修改这里除非你明确直到自己在做什么！
// 注意：不要修改这里除非你明确直到自己在做什么！
void protocol_exec_rt_system()
{
  uint8_t rt_exec; // 临时变量防止多次调用不稳定
  rt_exec = sys_rt_exec_alarm; // 复制易失性变量 sys_rt_exec_alarm.
  if (rt_exec) { // 只在任意标志位被设置时才进入。
    // 系统警报。一切都因为出了严重问题而停止了。
    // 向用户报告错误的来源。如果严重，Grbl关闭进入无限循环，直到系统复位/中止。
    sys.state = STATE_ALARM; // 设置系统警报状态
    report_alarm_message(rt_exec);
    // 在遇到危险事件标记后关闭系统，当前只有硬件限位和软件限位会这么做。
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT); // 报告危险事件
      system_clear_exec_state_flag(EXEC_RESET); // 禁用一切重置操作，保留现场。
      do {
        // 阻止一切操作，除了重置和状态报告，直到用户发出重置或电源重启。硬件限位通常发生在无人看管或不注意的情况下。
        // 让用户和GUI有时间在重置之前执行所需的操作，如终止传入流。
        // 软件限位也是如此。虽然位置没有丢失，但如果有机会执行，继续处理流可能会导致严重崩溃而损坏机器。
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // 清除警报
  }

  rt_exec = sys_rt_exec_state; // 复制易失性的 sys_rt_exec_state.
  if (rt_exec) {

    // 如果检测到重置，执行系统终止
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // 只把这个设置成 true.
      return; // 啥也不做，退出。
    }

    // 执行并通过串口打印状态
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status();
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    
    // 注意：一旦启动了保持，系统立即进入挂起状态阻止主程序进程直到重置或恢复。这确保保持完整性和安全性。
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

      // 为保持方法允许的状态进行状态检查
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
      
        // 如果处于 CYCLE 或 JOG 状态，立即启动运动保持。
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) { // 如果已经保持中，就阻塞。
            st_update_plan_block_parameters(); // 通知步进模块重新计算保持减速。
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; // 激活标志位启动挂起状态。
            if (sys.state == STATE_JOG) { // 点动在任何保持事件中取消，睡眠模式除外。
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; } 
            }
          }
        }
        // 如果处于 IDLE 状态，Grbl没有运动。简单地通知挂起状态和保持已经完成。
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        
        // 执行并标记运动取消，减速并返回空闲。主要用于探测周期时挂起并取消没进行完的运动。
        if (rt_exec & EXEC_MOTION_CANCEL) {
          // 运动取消仅在循环期间发生，但可提前启动保持和安全门以保持循环。
          // 运动取消仅对单个规划器块运动有效，而点动取消将处理和清除多个规划器块运动。
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; } // NOTE: State is STATE_CYCLE.
        }

        // 如果需要，执行减速进给保持。然后，挂起系统。
        if (rt_exec & EXEC_FEED_HOLD) {
          // 阻止安全门、点动和睡眠状态更改为保持状态。
          if (!(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }

        // 使用进给保持执行安全门停止，并禁用主轴/冷却液。
        // 注意：安全门不同于进给保持，它停止所有状态，禁用电力设备（主轴/冷却液），并阻止恢复，直到开关重新接合。
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR);
          // 如果点动，则阻塞安全门方法，直到点动取消完成。只标记它发生了。
          if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
            // 检查在恢复停靠运动期间安全门是否重新打开了。如果已经收回、停放或处于睡眠状态，则忽略。
            if (sys.state == STATE_SAFETY_DOOR) {
              if (sys.suspend & SUSPEND_INITIATE_RESTORE) { // 激活恢复
                #ifdef PARKING_ENABLE
                  // 设置保持并重置相应的控制标志以重新启动停车顺序。
                  if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    st_update_plan_block_parameters(); // 通知步进模块重新计算保持减速。
                    sys.step_control = (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION);
                    sys.suspend &= ~(SUSPEND_HOLD_COMPLETE);
                  } // else NO_MOTION is active.
                #endif
                // 清除暂停标志位
                sys.suspend &= ~(SUSPEND_RETRACT_COMPLETE | SUSPEND_INITIATE_RESTORE | SUSPEND_RESTORE_COMPLETE);
                sys.suspend |= SUSPEND_RESTART_RETRACT;
              }
            }
            if (sys.state != STATE_SLEEP) { sys.state = STATE_SAFETY_DOOR; }
          }
          // 注意:这个标志在门关闭时不会改变，不像sys.state。确保在门开关关闭并返回等待状态时执行停车动作。
          sys.suspend |= SUSPEND_SAFETY_DOOR_AJAR;
        }
        
      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      // 清除执行标志位
      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
    }

    // 通过启动步进中断来开始执行队列中的块来执行一个自动循环。
    if (rt_exec & EXEC_CYCLE_START) {
      // 阻塞如果保持命令同时调用：进给保持、运动取消和安全门。
      // 确保自动循环启动不会在没有明确用户输入的情况下恢复保持。
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
        //当停靠运动已缩回且门已关闭时，恢复门状态。
        if ((sys.state == STATE_SAFETY_DOOR) && !(sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)) {
          if (sys.suspend & SUSPEND_RESTORE_COMPLETE) {
            sys.state = STATE_IDLE; //设置为IDLE可立即恢复循环。
          } else if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
            // 如果被安全门禁用，则标记为重新激活带电部件并恢复原始位置。
            // 注意：要恢复安全门，开关必须关闭，如保持状态所示，并且缩回执行完成，这意味着初始进给保持未激活。
            // 要恢复正常操作，必须通过以下标志启动恢复过程。 
            // 一旦完成，它将调用CYCLE_START自动恢复并退出暂停。
            sys.suspend |= SUSPEND_INITIATE_RESTORE;
          }
        }
        // 循环只在 IDLE 或 保持完成并准备恢复时启动
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
            sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE; //设置为在挂起例程中恢复，然后循环启动。
          } else {
            //仅当规划器缓冲区中存在排队运动且运动未取消时，才开始循环。
            sys.step_control = STEP_CONTROL_NORMAL_OP; //将步进控制恢复到正常操作
            if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
              sys.suspend = SUSPEND_DISABLE; //中断挂起状态。
              sys.state = STATE_CYCLE;
              st_prep_buffer(); //在开始循环之前初始化步进段缓冲区。
              st_wake_up();
            } else { //否则，什么也不做。设置并恢复空闲状态。
              sys.suspend = SUSPEND_DISABLE; // 中断挂起状态。
              sys.state = STATE_IDLE;
            }
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
      // 在进给保持恢复时重新初始化循环计划和步进系统。由主程序中的实时命令执行调用，确保规划器安全地重新规划。
      // 注：Bresenham算法变量仍然通过规划器和步进循环循环重新初始化进行维护。步进器路径应该继续，就像什么都没有发生一样。
      // 注：EXEC_CYCLE_STOP 在一个循环或进给保持完成时被步进子系统设置。
      if ((sys.state & (STATE_HOLD|STATE_SAFETY_DOOR|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
        //保持完整。设置为指示准备恢复。保持在保持或门状态，直到用户已发出恢复命令或重置。
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control,(STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
        // 运动完成。包括 CYCLE/JOG/HOMING 状态和点动取消/运动取消/软件限位事件。
        // 注：运动和点动取消都会在保持完成后立即返回空闲。
        if (sys.suspend & SUSPEND_JOG_CANCEL) {   //对于点动取消，刷新缓冲区和同步位置。
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) { //仅在点动期间安全门打开时发生。
          sys.suspend &= ~(SUSPEND_JOG_CANCEL);
          sys.suspend |= SUSPEND_HOLD_COMPLETE;
          sys.state = STATE_SAFETY_DOOR;
        } else {
          sys.suspend = SUSPEND_DISABLE;
          sys.state = STATE_IDLE;
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP);
    }
  }

  // 执行覆盖。
  rt_exec = sys_rt_exec_motion_override; //复制易失性系统执行运动覆盖
  if (rt_exec) {
    system_clear_exec_motion_overrides(); //清除所有运动覆盖标志。

    uint8_t new_f_override =  sys.f_override;
    if (rt_exec & EXEC_FEED_OVR_RESET) { new_f_override = DEFAULT_FEED_OVERRIDE; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) { new_f_override += FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) { new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) { new_f_override += FEED_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) { new_f_override -= FEED_OVERRIDE_FINE_INCREMENT; }
    new_f_override = min(new_f_override,MAX_FEED_RATE_OVERRIDE);
    new_f_override = max(new_f_override,MIN_FEED_RATE_OVERRIDE);

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; //设置为立即报告更改
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

  rt_exec = sys_rt_exec_accessory_override;
  if (rt_exec) {
    system_clear_exec_accessory_overrides(); //清除所有附件覆盖标志。

    //注意：与运动覆盖不同，主轴覆盖不需要规划器重新初始化。
    uint8_t last_s_override =  sys.spindle_speed_ovr;
    if (rt_exec & EXEC_SPINDLE_OVR_RESET) { last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_PLUS) { last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_PLUS) { last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT; }
    last_s_override = min(last_s_override,MAX_SPINDLE_SPEED_OVERRIDE);
    last_s_override = max(last_s_override,MIN_SPINDLE_SPEED_OVERRIDE);

    if (last_s_override != sys.spindle_speed_ovr) {
      sys.spindle_speed_ovr = last_s_override;
      // 注意：保持状态期间的主轴速度覆盖由暂停功能处理。
      if (sys.state == STATE_IDLE) { spindle_set_state(gc_state.modal.spindle, gc_state.spindle_speed); }
			else { bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); }
      sys.report_ovr_counter = 0; //设置为立即报告更改
    }

    if (rt_exec & EXEC_SPINDLE_OVR_STOP) {
      // 主轴停止覆盖只在 HOLD 状态下允许
      // 注意：报告计数器在 spindle_set_state() 中当主轴停止执行后被设置。
      if (sys.state == STATE_HOLD) {
        if (!(sys.spindle_stop_ovr)) { sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE; }
        else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) { sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE; }
      }
    }

    
    // 注意：由于冷却液状态改变时都会执行规划器同步，因此当前运行状态可以通过检查解析器状态确定。
    // 注意：冷却液覆盖只能在 IDLE、CYCLE、HOLD 和 JOG 状态下操作。否则就忽略。
    if (rt_exec & (EXEC_COOLANT_FLOOD_OVR_TOGGLE | EXEC_COOLANT_MIST_OVR_TOGGLE)) {
      if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_JOG))) {
        uint8_t coolant_state = gc_state.modal.coolant;
        #ifdef ENABLE_M7
          if (rt_exec & EXEC_COOLANT_MIST_OVR_TOGGLE) {
            if (coolant_state & COOLANT_MIST_ENABLE) { bit_false(coolant_state,COOLANT_MIST_ENABLE); }
            else { coolant_state |= COOLANT_MIST_ENABLE; }
          }
          if (rt_exec & EXEC_COOLANT_FLOOD_OVR_TOGGLE) {
            if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state,COOLANT_FLOOD_ENABLE); }
            else { coolant_state |= COOLANT_FLOOD_ENABLE; }
          }
        #else
          if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state,COOLANT_FLOOD_ENABLE); }
          else { coolant_state |= COOLANT_FLOOD_ENABLE; }
        #endif
        coolant_set_state(coolant_state); //报告计数器设置为“冷却液设置”状态（）。
        gc_state.modal.coolant = coolant_state;
      }
    }
  }

  #ifdef DEBUG
    if (sys_rt_exec_debug) {
      report_realtime_debug();
      sys_rt_exec_debug = 0;
    }
  #endif

  //重新加载步进段缓冲区
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG)) {
    st_prep_buffer();
  }

}


//处理Grbl系统暂停程序，如进给保持、安全门和停车运动。
//系统将进入该循环，为挂起任务创建局部变量，然后返回到调用挂起的任何函数，以便Grbl恢复正常操作。
//此函数的编写方式旨在促进自定义停车动作。只需将此用作样板
static void protocol_exec_rt_suspend()
{
  #ifdef PARKING_ENABLE
    //声明并初始化局部变量
    float restore_target[N_AXIS];
    float parking_target[N_AXIS];
    float retract_waypoint = PARKING_PULLOUT_INCREMENT;
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data,0,sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
    #ifdef USE_LINE_NUMBERS
      pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
    #endif
  #endif

  plan_block_t *block = plan_get_current_block();
  uint8_t restore_condition;
  #ifdef VARIABLE_SPINDLE
    float restore_spindle_speed;
    if (block == NULL) {
      restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant);
      restore_spindle_speed = gc_state.spindle_speed;
    } else {
      restore_condition = (block->condition & PL_COND_SPINDLE_MASK) | coolant_get_state();
      restore_spindle_speed = block->spindle_speed;
    }
    #ifdef DISABLE_LASER_DURING_HOLD
      if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) { 
        system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP);
      }
    #endif
  #else
    if (block == NULL) { restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant); }
    else { restore_condition = (block->condition & PL_COND_SPINDLE_MASK) | coolant_get_state(); }
  #endif

  while (sys.suspend) {

    if (sys.abort) { return; }

    //阻塞，直到初始保持完成且机器停止运动。
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

      //停靠管理器。处理断电/再通电、开关状态检查和停车动作安全门和睡眠状态。
      if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {
      
        //处理回缩运动和断电。
        if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {

          //确保在安全门例行程序开始时禁用任何先前的主轴停止覆盖。
          sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;

          #ifndef PARKING_ENABLE

            spindle_set_state(SPINDLE_DISABLE,0.0); //断电
            coolant_set_state(COOLANT_DISABLE);     //断电

          #else
					
            //获取当前位置并存储恢复位置和主轴收回航路点。
            system_convert_array_steps_to_mpos(parking_target,sys_position);
            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              memcpy(restore_target,parking_target,sizeof(parking_target));
              retract_waypoint += restore_target[PARKING_AXIS];
              retract_waypoint = min(retract_waypoint,PARKING_TARGET);
            }

            //执行缓慢的拉出停靠回缩动作。停靠需要启用归位，则
            //当前位置未超过停靠目标位置，且激光模式已禁用。
            //注意：在断电和缩回完成之前，状态将保持在DOOR（车门）状态。
            #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
            if ((bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) &&
                            (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                            bit_isfalse(settings.flags,BITFLAG_LASER_MODE) &&
                            (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
            #else
            if ((bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) &&
                            (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                            bit_isfalse(settings.flags,BITFLAG_LASER_MODE)) {
            #endif
              //按拉出距离缩回主轴。确保回缩运动远离
              //工件和航路点运动不超过停车目标位置。
              if (parking_target[PARKING_AXIS] < retract_waypoint) {
                parking_target[PARKING_AXIS] = retract_waypoint;
                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); //保留附属状态
                pl_data->spindle_speed = restore_spindle_speed;
                mc_parking_motion(parking_target, pl_data);
              }

              //注意：收回和中止恢复动作后，清除附件状态。
              pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
              pl_data->spindle_speed = 0.0;
              spindle_set_state(SPINDLE_DISABLE,0.0); //断电
              coolant_set_state(COOLANT_DISABLE); //断电

              //执行快速停靠回缩运动至停靠目标位置。
              if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                parking_target[PARKING_AXIS] = PARKING_TARGET;
                pl_data->feed_rate = PARKING_RATE;
                mc_parking_motion(parking_target, pl_data);
              }

            } else {

              //停靠运动未设置。只需禁用主轴和冷却液。
              //注：激光模式不会启动驻靠动作，以确保激光立即停止。
              spindle_set_state(SPINDLE_DISABLE,0.0); //断电
              coolant_set_state(COOLANT_DISABLE);     //断电

            }

          #endif

          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;

        } else {

          
          if (sys.state == STATE_SLEEP) {
            report_feedback_message(MESSAGE_SLEEP_MODE);
            //主轴和冷却液应该已经停止，但要再次确保停止。
            spindle_set_state(SPINDLE_DISABLE,0.0); //断电
            coolant_set_state(COOLANT_DISABLE); //断电
            st_go_idle(); // Disable steppers
            while (!(sys.abort)) { protocol_exec_rt_system(); } //在重置之前不要执行任何操作。
            return; //接收到中止。返回以重新初始化。
          }    
          
          //允许从停车/安全门恢复。主动检查安全门是否已关闭并准备恢复。
          if (sys.state == STATE_SAFETY_DOOR) {
            if (!(system_check_safety_door_ajar())) {
              sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR); //重置车门微开标志，表示准备恢复。
            }
          }

          //处理停车恢复和安全门恢复。
          if (sys.suspend & SUSPEND_INITIATE_RESTORE) {

            #ifdef PARKING_ENABLE
              //执行快速恢复运动至拉出位置。停车需要启用归位。
              //注意：在断电和缩回完成之前，状态将保持在DOOR（门）状态。
              #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
              if (((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                   (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
              #else
              if ((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
              #endif
                //检查以确保运动不会在拉出位置内移动。
                if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                  parking_target[PARKING_AXIS] = retract_waypoint;
                  pl_data->feed_rate = PARKING_RATE;
                  mc_parking_motion(parking_target, pl_data);
                }
              }
            #endif

            //延迟任务：重新启动主轴和冷却液，延迟通电，然后恢复循环。
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              //如果在之前的恢复操作中安全门重新打开，则将其阻塞。
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
                  //在激光模式下，忽略主轴旋转延迟。设置为在循环开始时打开激光器。
                  bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                } else {
                  spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
                  delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DELAY_MODE_SYS_SUSPEND);
                }
              }
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) {
              //如果在之前的恢复操作中安全门重新打开，则将其阻塞。
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                //注：激光模式将允许此延迟。排气系统通常由该引脚控制。
                coolant_set_state((restore_condition & (PL_COND_FLAG_COOLANT_FLOOD | PL_COND_FLAG_COOLANT_MIST)));
                delay_sec(SAFETY_DOOR_COOLANT_DELAY, DELAY_MODE_SYS_SUSPEND);
              }
            }

            #ifdef PARKING_ENABLE
              // 从回拉位置执行缓冲击运动到恢复位置
              #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
              if (((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                   (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
              #else
              if ((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
              #endif
                //如果在之前的恢复操作中安全门重新打开，则将其阻塞。
                if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                  //无论收回停车动作是否为有效/安全动作恢复停靠运动在逻辑上应该有效，
                  // 可以通过返回原始位置通过有效的机器空间或完全不移动。
                  pl_data->feed_rate = PARKING_PULLOUT_RATE;
									pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); //恢复附件状态
									pl_data->spindle_speed = restore_spindle_speed;
                  mc_parking_motion(restore_target, pl_data);
                }
              }
            #endif

            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              sys.suspend |= SUSPEND_RESTORE_COMPLETE;
              system_set_exec_state_flag(EXEC_CYCLE_START); //设置为恢复程序。
            }
          }

        }


      } else {

        // 进给保持管理器。控制“主轴停止覆盖”状态。
        // 注：在暂停例行程序开始时，通过状态检查确保保持完整性。
        if (sys.spindle_stop_ovr) {
          //控制主轴停止的开始
          if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_INITIATE) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              spindle_set_state(SPINDLE_DISABLE,0.0); //断电
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_ENABLED; //如果断电，将停止覆盖状态设置为启用。
            } else {
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; //清除停止覆盖状态
            }
          //处理主轴状态的恢复
          } else if (sys.spindle_stop_ovr & (SPINDLE_STOP_OVR_RESTORE | SPINDLE_STOP_OVR_RESTORE_CYCLE)) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              report_feedback_message(MESSAGE_SPINDLE_RESTORE);
              if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
                //在激光模式下，忽略主轴旋转延迟。设置为在循环开始时打开激光器。
                bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
              } else {
                spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
              }
            }
            if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_RESTORE_CYCLE) {
              system_set_exec_state_flag(EXEC_CYCLE_START);  //设置为恢复程序。
            }
            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; //清除停止覆盖状态
          }
        } else {
          //在保持期间处理主轴状态。注意：主轴速度覆盖可能在保持状态期间改变。
          //注：在步进发电机中恢复时，步进控制更新主轴PWM自动复位。
          
          if (bit_istrue(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
            spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
            bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
          }
        }

      }
    }

    protocol_exec_rt_system();

  }
}
