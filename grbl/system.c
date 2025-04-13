/*
  system.c - 处理系统级命令和实时进程
  Grbl 的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


void system_init()
{
  CONTROL_DDR &= ~(CONTROL_MASK); //配置为输入引脚
  #ifdef DISABLE_CONTROL_PIN_PULL_UP
    CONTROL_PORT &= ~(CONTROL_MASK); //正常低电压运行。需要外部下拉。
  #else
    CONTROL_PORT |= CONTROL_MASK;   //启用内部上拉电阻器。正常高位运行。
  #endif
  CONTROL_PCMSK |= CONTROL_MASK;  //启用管脚更改中断的特定管脚
  PCICR |= (1 << CONTROL_INT);   //启用引脚更改中断
}


//将控制管脚状态作为uint8位字段返回。每个位表示输入引脚状态，其中触发为1，未触发为0。
// 应用反转掩码。字段组织由头文件中的控件索引定义。
uint8_t system_control_get_state()
{
  uint8_t control_state = 0;
  uint8_t pin = (CONTROL_PIN & CONTROL_MASK) ^ CONTROL_MASK;
  #ifdef INVERT_CONTROL_PIN_MASK
    pin ^= INVERT_CONTROL_PIN_MASK;
  #endif
  if (pin) {
    #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
      if (bit_istrue(pin,(1<<CONTROL_SAFETY_DOOR_BIT))) { control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR; }
    #else
      if (bit_istrue(pin,(1<<CONTROL_FEED_HOLD_BIT))) { control_state |= CONTROL_PIN_INDEX_FEED_HOLD; }
    #endif
    if (bit_istrue(pin,(1<<CONTROL_RESET_BIT))) { control_state |= CONTROL_PIN_INDEX_RESET; }
    if (bit_istrue(pin,(1<<CONTROL_CYCLE_START_BIT))) { control_state |= CONTROL_PIN_INDEX_CYCLE_START; }
  }
  return(control_state);
}


//引脚输出命令的引脚更改中断，即循环启动、进给保持和复位。
// 仅设置实时执行命令变量，以便主程序在就绪时执行这些命令。
// 这与直接从传入串行数据流中拾取的基于字符的实时命令完全相同。
ISR(CONTROL_INT_vect)
{
  uint8_t pin = system_control_get_state();
  if (pin) {
    if (bit_istrue(pin,CONTROL_PIN_INDEX_RESET)) {
      mc_reset();
    }
    if (bit_istrue(pin,CONTROL_PIN_INDEX_CYCLE_START)) {
      bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
    }
    #ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
      if (bit_istrue(pin,CONTROL_PIN_INDEX_FEED_HOLD)) {
        bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
    #else
      if (bit_istrue(pin,CONTROL_PIN_INDEX_SAFETY_DOOR)) {
        bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
    #endif
    }
  }
}


//如果安全门未关（T）或关闭（F），则根据引脚脚状态返回。
uint8_t system_check_safety_door_ajar()
{
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
  #else
    return(false); //输入引脚未启用，因此只需返回它已关闭。
  #endif
}


//执行用户启动脚本（如果已存储）。
void system_execute_startup(char *line)
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++) {
    if (!(settings_read_startup_line(n, line))) {
      line[0] = 0;
      report_execute_startup_message(line,STATUS_SETTING_READ_FAIL);
    } else {
      if (line[0] != 0) {
        uint8_t status_code = gc_execute_line(line);
        report_execute_startup_message(line,status_code);
      }
    }
  }
}


//指示并执行来自protocol_进程的一行格式化输入。
//虽然主要是传入的流式g代码块，但它也执行Grbl内部命令，如设置、启动归位循环和切换开关状态。
//这与实时命令模块的不同之处在于，Grbl在一个周期内准备好执行下一行的时间很容易受到影响，因此对于块删除之类的开关，开关只影响随后处理的行，而不一定在一个周期内实时处理，因为缓冲区中已经存储了运动。
//然而，这种“滞后”不应该是一个问题，因为这些命令通常不会在一个周期中使用。
uint8_t system_execute_line(char *line)
{
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; //辅助变量
  float parameter, value;
  switch( line[char_counter] ) {
    case 0 : report_grbl_help(); break; // 如果是空行，输出帮助信息
    case 'J' : //点动
      //仅当处于空闲或点动状态时执行。
      if (sys.state != STATE_IDLE && sys.state != STATE_JOG) { return(STATUS_IDLE_ERROR); }
      if(line[2] != '=') { return(STATUS_INVALID_STATEMENT); }
      return(gc_execute_line(line)); //注意：$J=在g代码解析器中被忽略，并用于检测点动运动。
      break;
    case '$': case 'G': case 'C': case 'X': // 这些开头的命令必须有后面的字符才有意义
      if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[1] ) {
        case '$' : //打印Grbl设置 对应'$$'命令
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // Block during cycle. Takes too long to print.
          else { report_grbl_settings(); }
          break;
        case 'G' : //打印gcode解析器状态
          //代办:将其移动到实时命令，以便GUI在挂起状态下请求此数据。
          report_gcode_modes();
          break;
        case 'C' : 
          // 设置检查g代码模式[空闲/检查]在关闭时执行复位。
          // 检查g代码模式应仅在Grbl空闲且准备就绪时工作，无论报警锁如何。
          // 这主要是为了保持事情的简单性和一致性。
          if ( sys.state == STATE_CHECK_MODE ) {
            mc_reset();
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if (sys.state) { return(STATUS_IDLE_ERROR); } // 需要非警报模式
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break;
        case 'X' : // $X 命令解除警报
          if (sys.state == STATE_ALARM) {
            // 如果安全门打开就阻止这个动作
            if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); }
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            //不要运行启动脚本。防止启动时存储的移动导致事故。
          } // 其他情况没影响
          break;
      }
      break;
    default :
      //阻止任何要求状态为空闲/报警的系统命令。（即EEPROM、复位）
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[1] ) {
        case '#' : //打印Grbl NGC参数
          if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;
        case 'H' : // $H 命令执行归位循环 [IDLE/ALARM]
          if (bit_isfalse(settings.flags,BITFLAG_HOMING_ENABLE)) {return(STATUS_SETTING_DISABLED); }
          if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); } // 如果安全门打开，就不进行
          sys.state = STATE_HOMING; //设置系统状态变量
          if (line[2] == 0) {
            mc_homing_cycle(HOMING_CYCLE_ALL);
          #ifdef HOMING_SINGLE_AXIS_COMMANDS
            } else if (line[3] == 0) {
              switch (line[2]) {
                case 'X': mc_homing_cycle(HOMING_CYCLE_X); break;
                case 'Y': mc_homing_cycle(HOMING_CYCLE_Y); break;
                case 'Z': mc_homing_cycle(HOMING_CYCLE_Z); break;
                default: return(STATUS_INVALID_STATEMENT);
              }
          #endif
          } else { return(STATUS_INVALID_STATEMENT); }
          if (!sys.abort) {  //成功归位后执行启动脚本。
            sys.state = STATE_IDLE; //完成后设置为空闲。
            st_go_idle(); //返回前，将步进电机设置为设置空闲状态。
            if (line[2] == 0) { system_execute_startup(line); }
          }
          break;
        case 'S' : // 使Grbl进入睡眠模式 [IDLE/ALARM]
          if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0)) { return(STATUS_INVALID_STATEMENT); }
          system_set_exec_state_flag(EXEC_SLEEP); //设置为立即执行睡眠模式
          break;
        case 'I' : // 打印或储存构建信息 [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) {
            settings_read_build_info(line);
            report_build_info(line);
          #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
            } else { // 储存启动脚本 [IDLE/ALARM]
              if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
              helper_var = char_counter; //将helper变量设置为用户信息行开头的计数器。
              do {
                line[char_counter-helper_var] = line[char_counter];
              } while (line[char_counter++] != 0);
              settings_store_build_info(line);
          #endif
          }
          break;
        case 'R' : // 恢复默认值 [IDLE/ALARM]
          if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0)) { return(STATUS_INVALID_STATEMENT); }
          switch (line[5]) {
            #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
              case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
              case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
              case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
            #endif
            default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); //强制重置以确保正确初始化设置。
          break;
        case 'N' : // 启动脚本 [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { // 打印启动行
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // 储存启动行，只在 [IDLE] 时进行防止警报期间运动
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } //仅在空闲时存储。
            helper_var = true;  // 设置 helper_var 标记储存方法
            // 不断开，继续到default:读取剩下的命令字符。
          }
        default :  // 储存设置方法 [IDLE/ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { //储存启动行
            //通过移动所有字符，准备将gcode块发送到gcode解析器
            helper_var = char_counter; //将helper变量设置为gcode块开始的计数器
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            //执行gcode块以确保块有效。
            helper_var = gc_execute_line(line); //将helper_var设置为返回的状态代码。
            if (helper_var) { return(helper_var); }
            else {
              helper_var = trunc(parameter); //将helper_var设置为参数的int值
              settings_store_startup_line(helper_var,line);
            }
          } else { //存储全局设置。
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }
  }
  return(STATUS_OK); //如果“$”命令到达此处，则一切正常。
}



void system_flag_wco_change()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
  sys.report_wco_counter = 0;
}


//返回轴“idx”的机器位置。必须发送一个“步骤”数组。
//注意：如果电机步数和机器位置不在同一坐标系中，此函数将用作计算变换的中心位置。
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
  #ifdef COREXY
    if (idx==X_AXIS) {
      pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx];
    } else if (idx==Y_AXIS) {
      pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx];
    } else {
      pos = steps[idx]/settings.steps_per_mm[idx];
    }
  #else
    pos = steps[idx]/settings.steps_per_mm[idx];
  #endif
  return(pos);
}


void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}


//仅限CoreXY计算。基于CoreXY电机步数返回x轴或y轴“步数”。
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] + steps[B_MOTOR])/2 );
  }
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] - steps[B_MOTOR])/2 );
  }
#endif


//检查并报告目标阵列是否超过机器行程限制。
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef HOMING_FORCE_SET_ORIGIN
      //启用归位强制设置原点时，软限位检查需要考虑方向性。
      //注意：最大行程存储为负数
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { return(true); }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
      }
    #else
      // NOTE: max_travel is stored as negative
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
    #endif
  }
  return(false);
}


//用于设置和清除Grbl实时执行标志的特殊处理程序。
void system_set_exec_state_flag(uint8_t mask) {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_state |= (mask);
  SREG = sreg;
}

void system_clear_exec_state_flag(uint8_t mask) {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_state &= ~(mask);
  SREG = sreg;
}

void system_set_exec_alarm(uint8_t code) {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_alarm = code;
  SREG = sreg;
}

void system_clear_exec_alarm() {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_alarm = 0;
  SREG = sreg;
}

void system_set_exec_motion_override_flag(uint8_t mask) {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_motion_override |= (mask);
  SREG = sreg;
}

void system_set_exec_accessory_override_flag(uint8_t mask) {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_accessory_override |= (mask);
  SREG = sreg;
}

void system_clear_exec_motion_overrides() {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_motion_override = 0;
  SREG = sreg;
}

void system_clear_exec_accessory_overrides() {
  uint8_t sreg = SREG;
  cli();
  sys_rt_exec_accessory_override = 0;
  SREG = sreg;
}
