/*
  spindle_control.h - 主轴控制模块
  Grbl 的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef spindle_control_h
#define spindle_control_h

#define SPINDLE_NO_SYNC false
#define SPINDLE_FORCE_SYNC true

#define SPINDLE_STATE_DISABLE  0//必须是零。
#define SPINDLE_STATE_CW       bit(0)
#define SPINDLE_STATE_CCW      bit(1)


//初始化主轴销和硬件PWM（如果启用）。
void spindle_init();

//返回当前主轴输出状态。覆盖可能会改变它的编程状态。
uint8_t spindle_get_state();

//设置主轴状态时由g代码解析器调用，需要缓冲区同步。
//如果启用，则立即通过PWM设置主轴运行状态以及方向和主轴转速。
//在恢复过程中同步和驻车运动/主轴停止覆盖后由spindle_sync（）调用。
#ifdef VARIABLE_SPINDLE

//设置主轴状态时由g代码解析器调用，需要缓冲区同步。
  void spindle_sync(uint8_t state, float rpm);

//使用方向、启用和主轴PWM设置主轴运行状态。
  void spindle_set_state(uint8_t state, float rpm); 
  
//为步进电机ISR快速设置主轴PWM。也称为主轴设置状态（）。
//注：328p PWM寄存器为8位。
  void spindle_set_speed(uint8_t pwm_value);
  
//计算给定RPM的328p特定PWM寄存器值，以便快速更新。
  uint8_t spindle_compute_pwm_value(float rpm);
  
#else
  
//设置主轴状态时由g代码解析器调用，需要缓冲区同步。
  #define spindle_sync(state, rpm) _spindle_sync(state)
  void _spindle_sync(uint8_t state);

//使用方向和启用设置主轴运行状态。
  #define spindle_set_state(state, rpm) _spindle_set_state(state)
  void _spindle_set_state(uint8_t state);

#endif

//停止和启动主轴例行程序。由所有主轴例程和步进ISR调用。
void spindle_stop();


#endif
