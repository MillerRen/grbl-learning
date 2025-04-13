/*
  report.c - 报告和消息传递方法
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

/*
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a
  different style feedback is desired (i.e. JSON), then a user can change these following
  methods to accomodate their needs.
*/

#include "grbl.h"


//内部报告实用程序可减少flash，将重复性任务转化为功能。
void report_util_setting_prefix(uint8_t n) { serial_write('$'); print_uint8_base10(n); serial_write('='); }
static void report_util_line_feed() { printPgmString(PSTR("\r\n")); }
static void report_util_feedback_line_feed() { serial_write(']'); report_util_line_feed(); }
static void report_util_gcode_modes_G() { printPgmString(PSTR(" G")); }
static void report_util_gcode_modes_M() { printPgmString(PSTR(" M")); }
// static void report_util_comment_line_feed() { serial_write(')'); report_util_line_feed(); }
static void report_util_axis_values(float *axis_value) {
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    printFloat_CoordValue(axis_value[idx]);
    if (idx < (N_AXIS-1)) { serial_write(','); }
  }
}

/*
static void report_util_setting_string(uint8_t n) {
  serial_write(' ');
  serial_write('(');
  switch(n) {
    case 0: printPgmString(PSTR("stp pulse")); break;
    case 1: printPgmString(PSTR("idl delay")); break; 
    case 2: printPgmString(PSTR("stp inv")); break;
    case 3: printPgmString(PSTR("dir inv")); break;
    case 4: printPgmString(PSTR("stp en inv")); break;
    case 5: printPgmString(PSTR("lim inv")); break;
    case 6: printPgmString(PSTR("prb inv")); break;
    case 10: printPgmString(PSTR("rpt")); break;
    case 11: printPgmString(PSTR("jnc dev")); break;
    case 12: printPgmString(PSTR("arc tol")); break;
    case 13: printPgmString(PSTR("rpt inch")); break;
    case 20: printPgmString(PSTR("sft lim")); break;
    case 21: printPgmString(PSTR("hrd lim")); break;
    case 22: printPgmString(PSTR("hm cyc")); break;
    case 23: printPgmString(PSTR("hm dir inv")); break;
    case 24: printPgmString(PSTR("hm feed")); break;
    case 25: printPgmString(PSTR("hm seek")); break;
    case 26: printPgmString(PSTR("hm delay")); break;
    case 27: printPgmString(PSTR("hm pulloff")); break;
    case 30: printPgmString(PSTR("rpm max")); break;
    case 31: printPgmString(PSTR("rpm min")); break;
    case 32: printPgmString(PSTR("laser")); break;
    default:
      n -= AXIS_SETTINGS_START_VAL;
      uint8_t idx = 0;
      while (n >= AXIS_SETTINGS_INCREMENT) {
        n -= AXIS_SETTINGS_INCREMENT;
        idx++;
      }
      serial_write(n+'x');
      switch (idx) {
        case 0: printPgmString(PSTR(":stp/mm")); break;
        case 1: printPgmString(PSTR(":mm/min")); break;
        case 2: printPgmString(PSTR(":mm/s^2")); break;
        case 3: printPgmString(PSTR(":mm max")); break;
      }
      break;
  }
  report_util_comment_line_feed();
}
*/

static void report_util_uint8_setting(uint8_t n, int val) { 
  report_util_setting_prefix(n); 
  print_uint8_base10(val); 
  report_util_line_feed(); // report_util_setting_string(n); 
}
static void report_util_float_setting(uint8_t n, float val, uint8_t n_decimal) { 
  report_util_setting_prefix(n); 
  printFloat(val,n_decimal);
  report_util_line_feed(); // report_util_setting_string(n);
}


//处理流式接口和人工反馈的主要确认协议响应。
//对于每一条输入线，此方法都会对成功的命令或命令进行“ok”响应
//“error:”用于指示线路发生的某个错误事件或在运行期间发生的某个关键系统错误
//活动错误事件可以源于g代码解析器、设置模块或异步
//来自严重错误，例如触发的硬限制。接口应始终监视这些
//响应。
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

//打印报警信息。
void report_alarm_message(uint8_t alarm_code)
{
  printPgmString(PSTR("ALARM:"));
  print_uint8_base10(alarm_code);
  report_util_line_feed();
  delay_ms(500); //强制延迟以确保消息清除串行写入缓冲区。
}

//打印反馈信息。这是一种集中的方法，用于为不属于状态/报警消息协议的内容提供额外的用户反馈。
//这些消息包括设置警告、开关切换以及如何退出警报。
//注意：对于接口，消息总是放在括号内。如果安装了静默模式，则消息编号代码小于零。
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


//欢迎辞
void report_init_message()
{
  printPgmString(PSTR("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n"));
}

//Grbl帮助消息
void report_grbl_help() {
  printPgmString(PSTR("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H ~ ! ? ctrl-x]\r\n"));    
}


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


//打印当前探针参数。在执行探测命令时，这些参数将在探测成功或探测失败时使用G38.3 无错误命令（如果支持）进行更新。
//这些值将一直保留，直到Grbl通电，从而重新归零。
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


//打印Grbl NGC参数（坐标偏移、探测）
void report_ngc_parameters()
{
  float coord_data[N_AXIS];
  uint8_t coord_select;
  for (coord_select = 0; coord_select <= SETTING_INDEX_NCOORD; coord_select++) {
    if (!(settings_read_coord_data(coord_select,coord_data))) {
      report_status_message(STATUS_SETTING_READ_FAIL);
      return;
    }
    printPgmString(PSTR("[G"));
    switch (coord_select) {
      case 6: printPgmString(PSTR("28")); break;
      case 7: printPgmString(PSTR("30")); break;
      default: print_uint8_base10(coord_select+54); break; // G54-G59
    }
    serial_write(':');
    report_util_axis_values(coord_data);
    report_util_feedback_line_feed();
  }
  printPgmString(PSTR("[G92:")); //打印G92，G92.1在内存中不是持久的
  report_util_axis_values(gc_state.coord_offset);
  report_util_feedback_line_feed();
  printPgmString(PSTR("[TLO:")); //打印刀具长度偏移值
  printFloat_CoordValue(gc_state.tool_length_offset);
  report_util_feedback_line_feed();
  report_probe_parameters(); //打印探针参数。在内存中不持久。
}


//打印当前gcode解析器模式状态
void report_gcode_modes()
{
  printPgmString(PSTR("[GC:G"));
  if (gc_state.modal.motion >= MOTION_MODE_PROBE_TOWARD) {
    printPgmString(PSTR("38."));
    print_uint8_base10(gc_state.modal.motion - (MOTION_MODE_PROBE_TOWARD-2));
  } else {
    print_uint8_base10(gc_state.modal.motion);
  }

  report_util_gcode_modes_G();
  print_uint8_base10(gc_state.modal.coord_select+54);

  report_util_gcode_modes_G();
  print_uint8_base10(gc_state.modal.plane_select+17);

  report_util_gcode_modes_G();
  print_uint8_base10(21-gc_state.modal.units);

  report_util_gcode_modes_G();
  print_uint8_base10(gc_state.modal.distance+90);

  report_util_gcode_modes_G();
  print_uint8_base10(94-gc_state.modal.feed_rate);

  if (gc_state.modal.program_flow) {
    report_util_gcode_modes_M();
    switch (gc_state.modal.program_flow) {
      case PROGRAM_FLOW_PAUSED : serial_write('0'); break;
      // case PROGRAM_FLOW_OPTIONAL_STOP : serial_write('1'); break; // M1 is ignored and not supported.
      case PROGRAM_FLOW_COMPLETED_M2 : 
      case PROGRAM_FLOW_COMPLETED_M30 : 
        print_uint8_base10(gc_state.modal.program_flow);
        break;
    }
  }

  report_util_gcode_modes_M();
  switch (gc_state.modal.spindle) {
    case SPINDLE_ENABLE_CW : serial_write('3'); break;
    case SPINDLE_ENABLE_CCW : serial_write('4'); break;
    case SPINDLE_DISABLE : serial_write('5'); break;
  }

  #ifdef ENABLE_M7
    if (gc_state.modal.coolant) { // Note: Multiple coolant states may be active at the same time.
      if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_MIST) { report_util_gcode_modes_M(); serial_write('7'); }
      if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_FLOOD) { report_util_gcode_modes_M(); serial_write('8'); }
    } else { report_util_gcode_modes_M(); serial_write('9'); }
  #else
    report_util_gcode_modes_M();
    if (gc_state.modal.coolant) { serial_write('8'); }
    else { serial_write('9'); }
  #endif

  #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    if (sys.override_ctrl == OVERRIDE_PARKING_MOTION) { 
      report_util_gcode_modes_M();
      print_uint8_base10(56);
    }
  #endif
  
  printPgmString(PSTR(" T"));
  print_uint8_base10(gc_state.tool);

  printPgmString(PSTR(" F"));
  printFloat_RateValue(gc_state.feed_rate);

  #ifdef VARIABLE_SPINDLE
    printPgmString(PSTR(" S"));
    printFloat(gc_state.spindle_speed,N_DECIMAL_RPMVALUE);
  #endif

  report_util_feedback_line_feed();
}

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

//打印构建信息
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


//打印Grbl从用户处收到的字符串行，该字符串行已预解析，并已发送到protocol_execute_line例程中，由Grbl执行。
void report_echo_line_received(char *line)
{
  printPgmString(PSTR("[echo: ")); printString(line);
  report_util_feedback_line_feed();
}


 //打印实时数据。此函数获取步进子程序的实时快照和数控机床的实际位置。
 //用户可以根据自己的具体需要更改以下功能，但所需的实时数据报告必须尽可能短。
 //这是必需的，因为它将计算开销降至最低，并允许grbl保持平稳运行，特别是在具有快速、短线段和高频报告（5-20Hz）的g代码程序期间。
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


#ifdef DEBUG
  void report_realtime_debug()
  {

  }
#endif
