# 入口 `main.c`
Grbl的主入口是grbl文件夹里面的`main.c`,入口函数是`main()`，而不是Arduino的 `setup()`和`loop()`，其实功能上差不多，就是先执行初始化，再加一个无限循环。我们先看下`main.c`的源代码和注释。
``` c
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


// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif


int main(void)
{
  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt

  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.
  sei(); // Enable interrupts

  // Initialize system state.
  #ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif
  
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

  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // Reset system variables.
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
		memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
    report_init_message();

    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

  }
  return 0;   /* Never reached */
}
```

注释的开头说明grbl是一个支持NIST（国家标准及技术局） rs274/ngc规范（[见附件](/docs/RS274NGC_3.pdf)）的嵌入式控制器，它在GNU GPL许可下发布。   
- grbl是模块化的，所有模块通过`#include "grbl.h"`文件头引入.   
然后初始化了一些全局状态，通过状态机维护这些状态:   
``` c
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
```

- 在main函数中对各个模块进行初始化，主要包括定义引脚，设置中断等,然后开启总中断：
``` c
// 开机后初始化系统
  serial_init();   // 设置串口波特率和中断。
  settings_init(); // 从EEPROM加载Grbl设置。
  stepper_init();  // 配置步进电机引脚和中断定时器。
  system_init();   // 配置引出引脚和引脚电平改变中断。

  memset(sys_position,0,sizeof(sys_position)); // 清空机器位置。
  sei(); // 开启总中断。
```

- 如果设置了进行限位开关的检查，就进入限位警报状态，机器启动时就得进行`$H`归位，或者通过`$X`命令解除警报:
``` c
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
```

- 然后进入循环，进行干净的初始化：
``` c
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
```

- 然后进入grbl的主循环`protocol_main_loop()`,开始协议解析和后台任务处理。
