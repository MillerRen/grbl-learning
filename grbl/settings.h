/*
  settings.h - eeprom配置处理
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef settings_h
#define settings_h

#include "grbl.h"


//EEPROM数据的版本。固件升级时，将用于从较旧版本的Grbl迁移现有数据。始终存储在eeprom的字节0中
#define SETTINGS_VERSION 10  //注意：移动到下一版本时，请检查settings_reset（）。

//为settings.flag中的布尔设置定义位标志掩码
#define BIT_REPORT_INCHES      0
#define BIT_LASER_MODE         1
#define BIT_INVERT_ST_ENABLE   2
#define BIT_HARD_LIMIT_ENABLE  3
#define BIT_HOMING_ENABLE      4
#define BIT_SOFT_LIMIT_ENABLE  5
#define BIT_INVERT_LIMIT_PINS  6
#define BIT_INVERT_PROBE_PIN   7

#define BITFLAG_REPORT_INCHES      bit(BIT_REPORT_INCHES)
#define BITFLAG_LASER_MODE         bit(BIT_LASER_MODE)
#define BITFLAG_INVERT_ST_ENABLE   bit(BIT_INVERT_ST_ENABLE)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(BIT_HARD_LIMIT_ENABLE)
#define BITFLAG_HOMING_ENABLE      bit(BIT_HOMING_ENABLE)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(BIT_SOFT_LIMIT_ENABLE)
#define BITFLAG_INVERT_LIMIT_PINS  bit(BIT_INVERT_LIMIT_PINS)
#define BITFLAG_INVERT_PROBE_PIN   bit(BIT_INVERT_PROBE_PIN)

//在settings.status_report_mask中定义状态报告布尔启用位标志
#define BITFLAG_RT_STATUS_POSITION_TYPE     bit(0)
#define BITFLAG_RT_STATUS_BUFFER_STATE      bit(1)

//定义设置还原位标志。
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)
#ifndef SETTINGS_RESTORE_ALL
  #define SETTINGS_RESTORE_ALL 0xFF //所有位标志
#endif

//为Grbl设置和参数定义EEPROM存储器地址位置值
//注：Atmega328p具有1KB EEPROM。上半部分保留用于参数和启动脚本。下半部分包含全球设置和未来发展空间。
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_STARTUP_BLOCK  768U
#define EEPROM_ADDR_BUILD_INFO     942U

//定义坐标参数的EEPROM地址索引
#define N_COORDINATE_SYSTEM 6  //支持的工作坐标系数量（从索引1开始）
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 //存储的系统总数（从索引0开始）
//注：工作坐标指数为（0=G54，1=G55，…，6=G59）
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // 归位位置1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // 归位位置2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

//定义Grbl轴设置编号方案。 始于START_VAL, 每次递增, 直到 N_SETTINGS.
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_START_VAL  100 //注意：保留轴设置的设置值>=100。最多255个。
#define AXIS_SETTINGS_INCREMENT  10  //必须大于轴设置的数量

//全局持久设置（从字节EEPROM_ADDR_GLOBAL开始存储）
typedef struct {
  //轴设置
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];

  //剩余Grbl设置
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time; //如果最大值为255，则步进器不禁用。
  uint8_t status_report_mask; //用于指示所需报告数据的掩码。
  float junction_deviation;
  float arc_tolerance;

  float rpm_max;
  float rpm_min;

  uint8_t flags;  //包含默认的布尔设置

  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
} settings_t;
extern settings_t settings;

//初始化配置子系统（从EEPROM加载设置）
void settings_init();

//用于清除和恢复EEPROM默认值的辅助功能
void settings_restore(uint8_t restore_flag);

//从命令行设置新设置的助手方法
uint8_t settings_store_global_setting(uint8_t parameter, float value);

//将协议行变量作为启动行存储在EEPROM中
void settings_store_startup_line(uint8_t n, char *line);

//将EEPROM启动行读取到协议行变量
uint8_t settings_read_startup_line(uint8_t n, char *line);

//存储生成信息用户定义的字符串
void settings_store_build_info(char *line);

//读取生成信息用户定义的字符串
uint8_t settings_read_build_info(char *line);

//将所选坐标数据写入EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

//从EEPROM读取选定的坐标数据
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

//根据Grbl的内部轴编号返回步进管脚掩码
uint8_t get_step_pin_mask(uint8_t i);

//根据Grbl的内部轴编号返回方向接点掩码
uint8_t get_direction_pin_mask(uint8_t i);

//根据Grbl的内部轴编号返回限制引脚掩码
uint8_t get_limit_pin_mask(uint8_t i);


#endif
