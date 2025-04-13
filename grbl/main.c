/*
  main.c - 支持rs274/ngc (g-code)的嵌入式CNC控制器
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


// 声明系统全局变量结构体
system_t sys;
int32_t sys_position[N_AXIS];      // 用步数表示的实时机器（如归位）位置向量。
int32_t sys_probe_position[N_AXIS]; // 探针在机器坐标的最后位置和步数。
volatile uint8_t sys_probe_state;   // 对刀状态值。用于在步进中断中配合对刀周期。
volatile uint8_t sys_rt_exec_state;   // 用于状态管理的全局实时执行器比特位标志位。 请参阅执行比特位掩码。
volatile uint8_t sys_rt_exec_alarm;   // 用于设置一系列警告的全局实时执行器比特位标志位变量。
volatile uint8_t sys_rt_exec_motion_override; // 用于基于运动的覆盖的全局执行器位标志位变量。
volatile uint8_t sys_rt_exec_accessory_override; // 用于主轴/冷却覆盖的全局实时执行器比特位标志位变量。
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif


int main(void)
{
  // 开机后初始化系统
  serial_init();   // 设置串口波特率和中断。
  settings_init(); // 从EEPROM加载Grbl设置。
  stepper_init();  // 配置步进电机引脚和中断定时器。
  system_init();   // 配置引出引脚和引脚电平改变中断。

  memset(sys_position,0,sizeof(sys_position)); // 清空机器位置。
  sei(); // 开启总中断。

  // 初始化系统状态
  #ifdef FORCE_INITIALIZATION_ALARM
    // 在系统上电或硬件重置时强制Grbl进入ALARM状态。
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif
  // 通过设置Grbl的警报状态，如果归位被启用，则上电后检查系统警报以强制进入归位周期。
  // 警报锁定所有G代码命令，包括启动脚本，但是允许设置和内部命令。
  // 只允许归位周期'$H'命令或解除警报'$X'命令。
  // 注意：启动脚本将会在归位周期成功完成后执行，但是不会在解除警报后执行。
  // 阻止运动启动块从崩溃中进入不可控的事情。那是非常糟糕的。
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Grbl 在上电或系统终止后初始化循环。稍后，所有过程将会返回到这个循环进行干净地重新初始化。
  for(;;) {

    // 重置系统变量。
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // 清空系统结构体变量。
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // 设置进给覆盖 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // 设置速度覆盖为 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // 设置主轴覆盖为 100%
		memset(sys_probe_position,0,sizeof(sys_probe_position)); // 清空探针位置。
    sys_probe_state = 0; // 初始化探针状态
    sys_rt_exec_state = 0; // 初始化实时执行状态
    sys_rt_exec_alarm = 0; // 初始化实时警报状态
    sys_rt_exec_motion_override = 0; // 初始化实时执行运动覆盖
    sys_rt_exec_accessory_override = 0; // 初始化实时主轴或冷却覆盖

    // 重置Grbl主系统。
    serial_reset_read_buffer(); // 清空串口读缓冲区
    gc_init(); // 设置G代码解析器到默认状态。
    spindle_init(); // 初始化主轴子系统
    coolant_init(); // 初始化冷却子系统
    limits_init(); // 初始化限位子系统
    probe_init(); // 初始化对刀子系统
    plan_reset(); // 清空块缓冲区和规划器变量。
    st_reset(); // 清空步进子系统变量。

    // 同步清空了的G代码和规划器位置到当前系统位置。
    plan_sync_position();
    gc_sync_position();

    // 打印欢迎信息。通知在上电或复位时发生了初始化。
    report_init_message();

    // 开启Grbl主循环。处理程序输入并执行他们。
    protocol_main_loop();

  }
  return 0;   /* 根本不会到达这里 */
}
