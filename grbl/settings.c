/*
  settings.c - eeprom configuration handling
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

settings_t settings;

const __flash settings_t defaults = {\
    .pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS,
    .stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME,
    .step_invert_mask = DEFAULT_STEPPING_INVERT_MASK,
    .dir_invert_mask = DEFAULT_DIRECTION_INVERT_MASK,
    .status_report_mask = DEFAULT_STATUS_REPORT_MASK,
    .junction_deviation = DEFAULT_JUNCTION_DEVIATION,
    .arc_tolerance = DEFAULT_ARC_TOLERANCE,
    .rpm_max = DEFAULT_SPINDLE_RPM_MAX,
    .rpm_min = DEFAULT_SPINDLE_RPM_MIN,
    .homing_dir_mask = DEFAULT_HOMING_DIR_MASK,
    .homing_feed_rate = DEFAULT_HOMING_FEED_RATE,
    .homing_seek_rate = DEFAULT_HOMING_SEEK_RATE,
    .homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY,
    .homing_pulloff = DEFAULT_HOMING_PULLOFF,
    .flags = (DEFAULT_REPORT_INCHES << BIT_REPORT_INCHES) | \
             (DEFAULT_LASER_MODE << BIT_LASER_MODE) | \
             (DEFAULT_INVERT_ST_ENABLE << BIT_INVERT_ST_ENABLE) | \
             (DEFAULT_HARD_LIMIT_ENABLE << BIT_HARD_LIMIT_ENABLE) | \
             (DEFAULT_HOMING_ENABLE << BIT_HOMING_ENABLE) | \
             (DEFAULT_SOFT_LIMIT_ENABLE << BIT_SOFT_LIMIT_ENABLE) | \
             (DEFAULT_INVERT_LIMIT_PINS << BIT_INVERT_LIMIT_PINS) | \
             (DEFAULT_INVERT_PROBE_PIN << BIT_INVERT_PROBE_PIN),
    .steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM,
    .steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM,
    .steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM,
    .max_rate[X_AXIS] = DEFAULT_X_MAX_RATE,
    .max_rate[Y_AXIS] = DEFAULT_Y_MAX_RATE,
    .max_rate[Z_AXIS] = DEFAULT_Z_MAX_RATE,
    .acceleration[X_AXIS] = DEFAULT_X_ACCELERATION,
    .acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION,
    .acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION,
    .max_travel[X_AXIS] = (-DEFAULT_X_MAX_TRAVEL),
    .max_travel[Y_AXIS] = (-DEFAULT_Y_MAX_TRAVEL),
    .max_travel[Z_AXIS] = (-DEFAULT_Z_MAX_TRAVEL)};


//将启动行存储到EEPROM中的方法
void settings_store_startup_line(uint8_t n, char *line)
{
  #ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize(); //启动行可能包含运动并正在执行。
  #endif
  uint32_t addr = n*(LINE_BUFFER_SIZE+1)+EEPROM_ADDR_STARTUP_BLOCK;
  memcpy_to_eeprom_with_checksum(addr,(char*)line, LINE_BUFFER_SIZE);
}


//将构建信息存储到EEPROM中的方法
//注意：此函数只能在空闲状态下调用。
void settings_store_build_info(char *line)
{
  //生成信息只能在状态为空闲时存储。
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_BUILD_INFO,(char*)line, LINE_BUFFER_SIZE);
}


//将坐标数据参数存储到EEPROM中的方法
void settings_write_coord_data(uint8_t coord_select, float *coord_data)
{
  #ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize();
  #endif
  uint32_t addr = coord_select*(sizeof(float)*N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  memcpy_to_eeprom_with_checksum(addr,(char*)coord_data, sizeof(float)*N_AXIS);
}


//将Grbl全局设置结构和版本号存储到EEPROM中的方法
//注意：此函数只能在空闲状态下调用。
void write_global_settings()
{
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char*)&settings, sizeof(settings_t));
}


//方法将EEPROM保存的Grbl全局设置恢复为默认值。
void settings_restore(uint8_t restore_flag) {
  if (restore_flag & SETTINGS_RESTORE_DEFAULTS) {    
    settings = defaults;
    write_global_settings();
  }

  if (restore_flag & SETTINGS_RESTORE_PARAMETERS) {
    uint8_t idx;
    float coord_data[N_AXIS];
    memset(&coord_data, 0, sizeof(coord_data));
    for (idx=0; idx <= SETTING_INDEX_NCOORD; idx++) { settings_write_coord_data(idx, coord_data); }
  }

  if (restore_flag & SETTINGS_RESTORE_STARTUP_LINES) {
    #if N_STARTUP_LINE > 0
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK, 0);
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK+1, 0); // Checksum
    #endif
    #if N_STARTUP_LINE > 1
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK+(LINE_BUFFER_SIZE+1), 0);
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK+(LINE_BUFFER_SIZE+2), 0); //校验和
    #endif
  }

  if (restore_flag & SETTINGS_RESTORE_BUILD_INFO) {
    eeprom_put_char(EEPROM_ADDR_BUILD_INFO , 0);
    eeprom_put_char(EEPROM_ADDR_BUILD_INFO+1 , 0); //校验和
  }
}


//从EEPROM读取启动行。更新指向的线字符串数据。
uint8_t settings_read_startup_line(uint8_t n, char *line)
{
  uint32_t addr = n*(LINE_BUFFER_SIZE+1)+EEPROM_ADDR_STARTUP_BLOCK;
  if (!(memcpy_from_eeprom_with_checksum((char*)line, addr, LINE_BUFFER_SIZE))) {
    //使用默认值重置行
    line[0] = 0; //空行
    settings_store_startup_line(n, line);
    return(false);
  }
  return(true);
}


//从EEPROM读取启动行。更新指向的线字符串数据。
uint8_t settings_read_build_info(char *line)
{
  if (!(memcpy_from_eeprom_with_checksum((char*)line, EEPROM_ADDR_BUILD_INFO, LINE_BUFFER_SIZE))) {
    //使用默认值重置行
    line[0] = 0; //空行
    settings_store_build_info(line);
    return(false);
  }
  return(true);
}


//从EEPROM读取选定的坐标数据。更新点坐标数据值。
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data)
{
  uint32_t addr = coord_select*(sizeof(float)*N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  if (!(memcpy_from_eeprom_with_checksum((char*)coord_data, addr, sizeof(float)*N_AXIS))) {
    //使用默认零向量重置
    clear_vector_float(coord_data);
    settings_write_coord_data(coord_select,coord_data);
    return(false);
  }
  return(true);
}


//从EEPROM读取Grbl全局设置结构。
uint8_t read_global_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);
  if (version == SETTINGS_VERSION) {
    //读取设置记录并检查校验和
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t)))) {
      return(false);
    }
  } else {
    return(false);
  }
  return(true);
}


//从命令行设置设置的助手方法
uint8_t settings_store_global_setting(uint8_t parameter, float value) {
  if (value < 0.0) { return(STATUS_NEGATIVE_VALUE); }
  if (parameter >= AXIS_SETTINGS_START_VAL) {
    //存储轴配置。由轴设置设置的轴编号顺序定义。
//注意：确保设置索引与打印的 report.c设置相对应。
    parameter -= AXIS_SETTINGS_START_VAL;
    uint8_t set_idx = 0;
    while (set_idx < AXIS_N_SETTINGS) {
      if (parameter < N_AXIS) {
        //校验轴设置。
        switch (set_idx) {
          case 0:
            #ifdef MAX_STEP_RATE_HZ
              if (value*settings.max_rate[parameter] > (MAX_STEP_RATE_HZ*60.0)) { return(STATUS_MAX_STEP_RATE_EXCEEDED); }
            #endif
            settings.steps_per_mm[parameter] = value;
            break;
          case 1:
            #ifdef MAX_STEP_RATE_HZ
              if (value*settings.steps_per_mm[parameter] > (MAX_STEP_RATE_HZ*60.0)) {  return(STATUS_MAX_STEP_RATE_EXCEEDED); }
            #endif
            settings.max_rate[parameter] = value;
            break;
          case 2: settings.acceleration[parameter] = value*60*60; break; //转换为mm/min^2以供grbl内部使用。
          case 3: settings.max_travel[parameter] = -value; break;  //作为负值储存，供grbl内部使用。
        }
        break; //配置设置后退出while循环，继续EEPROM写入调用。
      } else {
        set_idx++;
        //如果轴索引大于N_AXIS或设置索引大于轴设置数，则会出现错误。
        if ((parameter < AXIS_SETTINGS_INCREMENT) || (set_idx == AXIS_N_SETTINGS)) { return(STATUS_INVALID_STATEMENT); }
        parameter -= AXIS_SETTINGS_INCREMENT;
      }
    }
  } else {
    //存储非轴Grbl设置
    uint8_t int_value = trunc(value);
    switch(parameter) {
      case 0:
        if (int_value < 3) { return(STATUS_SETTING_STEP_PULSE_MIN); }
        settings.pulse_microseconds = int_value; break;
      case 1: settings.stepper_idle_lock_time = int_value; break;
      case 2:
        settings.step_invert_mask = int_value;
        st_generate_step_dir_invert_masks(); //重新生成步进和方向端口反转掩码。
        break;
      case 3:
        settings.dir_invert_mask = int_value;
        st_generate_step_dir_invert_masks(); //重新生成步进和方向端口反转掩码。
        break;
      case 4: //重置以确保更改。立即重新初始化可能会导致问题。
        if (int_value) { settings.flags |= BITFLAG_INVERT_ST_ENABLE; }
        else { settings.flags &= ~BITFLAG_INVERT_ST_ENABLE; }
        break;
      case 5: //重置以确保更改。立即重新初始化可能会导致问题。
        if (int_value) { settings.flags |= BITFLAG_INVERT_LIMIT_PINS; }
        else { settings.flags &= ~BITFLAG_INVERT_LIMIT_PINS; }
        break;
      case 6: //重置以确保更改。立即重新初始化可能会导致问题。
        if (int_value) { settings.flags |= BITFLAG_INVERT_PROBE_PIN; }
        else { settings.flags &= ~BITFLAG_INVERT_PROBE_PIN; }
        probe_configure_invert_mask(false);
        break;
      case 10: settings.status_report_mask = int_value; break;
      case 11: settings.junction_deviation = value; break;
      case 12: settings.arc_tolerance = value; break;
      case 13:
        if (int_value) { settings.flags |= BITFLAG_REPORT_INCHES; }
        else { settings.flags &= ~BITFLAG_REPORT_INCHES; }
        system_flag_wco_change(); //确保立即更新WCO。
        break;
      case 20:
        if (int_value) {
          if (bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE)) { return(STATUS_SOFT_LIMIT_ERROR); }
          settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE;
        } else { settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE; }
        break;
      case 21:
        if (int_value) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
        else { settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE; }
        limits_init(); //重新初始化以立即更改。注意：很好，但以后可能会有问题。
        break;
      case 22:
        if (int_value) { settings.flags |= BITFLAG_HOMING_ENABLE; }
        else {
          settings.flags &= ~BITFLAG_HOMING_ENABLE;
          settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE; //强制禁用软限位。
        }
        break;
      case 23: settings.homing_dir_mask = int_value; break;
      case 24: settings.homing_feed_rate = value; break;
      case 25: settings.homing_seek_rate = value; break;
      case 26: settings.homing_debounce_delay = int_value; break;
      case 27: settings.homing_pulloff = value; break;
      case 30: settings.rpm_max = value; spindle_init(); break; //重新初始化主轴转速校准
      case 31: settings.rpm_min = value; spindle_init(); break; //重新初始化主轴转速校准
      case 32:
        #ifdef VARIABLE_SPINDLE
          if (int_value) { settings.flags |= BITFLAG_LASER_MODE; }
          else { settings.flags &= ~BITFLAG_LASER_MODE; }
        #else
          return(STATUS_SETTING_DISABLED_LASER);
        #endif
        break;
      default:
        return(STATUS_INVALID_STATEMENT);
    }
  }
  write_global_settings();
  return(STATUS_OK);
}


//初始化配置子系统
void settings_init() {
  if(!read_global_settings()) {
    report_status_message(STATUS_SETTING_READ_FAIL);
    settings_restore(SETTINGS_RESTORE_ALL); //强制恢复所有EEPROM数据。
    report_grbl_settings();
  }
}


//根据Grbl内部轴索引返回步进引脚掩码。
uint8_t get_step_pin_mask(uint8_t axis_idx)
{
  if ( axis_idx == X_AXIS ) { return((1<<X_STEP_BIT)); }
  if ( axis_idx == Y_AXIS ) { return((1<<Y_STEP_BIT)); }
  return((1<<Z_STEP_BIT));
}


//根据Grbl内部轴索引返回方向引脚掩码。
uint8_t get_direction_pin_mask(uint8_t axis_idx)
{
  if ( axis_idx == X_AXIS ) { return((1<<X_DIRECTION_BIT)); }
  if ( axis_idx == Y_AXIS ) { return((1<<Y_DIRECTION_BIT)); }
  return((1<<Z_DIRECTION_BIT));
}


//根据Grbl内部轴索引返回限制引脚掩码。
uint8_t get_limit_pin_mask(uint8_t axis_idx)
{
  if ( axis_idx == X_AXIS ) { return((1<<X_LIMIT_BIT)); }
  if ( axis_idx == Y_AXIS ) { return((1<<Y_LIMIT_BIT)); }
  return((1<<Z_LIMIT_BIT));
}
