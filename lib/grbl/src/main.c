/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

// 声明系统全局变量结构体
// Declare system global variable structure
system_t sys;
// 用步数表示的实时机器位置向量例如归位
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
// 最近的刀位用机器坐标和步数表示
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
// 对刀状态值。用于配合对刀周期和步进电机中断。
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
// 全局实时执行者位标记变量用于状态管理。见EXEC比特掩码。
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
// 全局实时执行者位标记用于设置一系列警报。
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
// 全局实时执行者位标记变量用于运动的重写
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
// 全局实施执行者位标记用于主轴/冷却的重写
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif


int main(void)
{
  // 上电后初始化系统
  // Initialize system upon power-up.
  // 设置串口波特率和中断
  serial_init();   // Setup serial baud rate and interrupts
  // 从EEPROM中加载Grbl设置
  settings_init(); // Load Grbl settings from EEPROM
  // 配置步进电机引脚和中断定时器
  stepper_init();  // Configure stepper pins and interrupt timers
  // 配置引出引脚和引脚电平变化中断
  system_init();   // Configure pinout pins and pin-change interrupt

  // 清空机器位置
  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.
  // 开启总中断
  sei(); // Enable interrupts

  // 初始化系统状态
  // Initialize system state.
  #ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif
  
  // 上电检测设置系统警报，如果归位被使能，强制归位循环。警报锁定所有G代码命令，包括启动脚本
  // 但是允许访问设置和内部命令。只有通过'$H'命令归位或通过'$X'命令移除警报锁定才会让警报消失。
  // 注意：启动脚本将会在归位后运行，但是在解除警报时不会。阻止不受控制的运动启动块产生损坏。
  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // 在上电或终止后，Grbl初始化循环。然后，所有进程返回到这个循环干净的重新初始化。
  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // 重置系统变量
    // Reset system variables.
    uint8_t prior_state = sys.state;
    // 清空系统结构体变量。
    memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
    sys.state = prior_state;
    // 设置默认重写
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
    // 清空对刀位置
		memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
    // 清空执行状态
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // 重置Grbl主要的系统
    // Reset Grbl primary systems.
    // 清空串口读缓冲区
    serial_reset_read_buffer(); // Clear serial read buffer
    // 设置g代码解析器到默认状态
    gc_init(); // Set g-code parser to default state
    // 初始化主轴
    spindle_init();
    // 初始化冷却系统
    coolant_init();
    // 初始化限位开关
    limits_init();
    // 初始化对刀
    probe_init();
    // 清空块缓冲区和规划器变量
    plan_reset(); // Clear block buffer and planner variables
    // 清空步进电机子系统变量
    st_reset(); // Clear stepper subsystem variables.

    // 同步g代码和规划器位置信息给当前系统位置
    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();
    // 打印欢迎信息。通知初始化完成。
    // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
    report_init_message();

    // 开始Grbl主循环。处理程序输入并执行。
    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

  }
  return 0;   /* Never reached */
}
