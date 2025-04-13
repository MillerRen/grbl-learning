/*
  system.h - 系统级别命令和实时处理的头文件
  Grbl 的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef system_h
#define system_h

#include "grbl.h"

//定义系统执行器位图。由实时协议内部用作实时命令标志，
//通知主程序异步执行指定的实时命令。
//注意：系统执行器使用无符号8位易失性变量（8标志限制）默认值
//标志始终为false，因此realtime协议只需检查非零值即可
//知道何时有实时命令要执行。
#define EXEC_STATUS_REPORT  bit(0) // bitmask 00000001
#define EXEC_CYCLE_START    bit(1) // bitmask 00000010
#define EXEC_CYCLE_STOP     bit(2) // bitmask 00000100
#define EXEC_FEED_HOLD      bit(3) // bitmask 00001000
#define EXEC_RESET          bit(4) // bitmask 00010000
#define EXEC_SAFETY_DOOR    bit(5) // bitmask 00100000
#define EXEC_MOTION_CANCEL  bit(6) // bitmask 01000000
#define EXEC_SLEEP          bit(7) // bitmask 10000000

//报警执行器代码。有效值（1-255）。零是保留的。
#define EXEC_ALARM_HARD_LIMIT                 1
#define EXEC_ALARM_SOFT_LIMIT                 2
#define EXEC_ALARM_ABORT_CYCLE                3
#define EXEC_ALARM_PROBE_FAIL_INITIAL         4
#define EXEC_ALARM_PROBE_FAIL_CONTACT         5
#define EXEC_ALARM_HOMING_FAIL_RESET          6
#define EXEC_ALARM_HOMING_FAIL_DOOR           7
#define EXEC_ALARM_HOMING_FAIL_PULLOFF        8
#define EXEC_ALARM_HOMING_FAIL_APPROACH       9
#define EXEC_ALARM_HOMING_FAIL_DUAL_APPROACH  10

//覆盖位图。用于控制进给、快速、主轴和冷却液覆盖的实时位标志。
//主轴/冷却液和进给/急流分为两个控制标志变量。
#define EXEC_FEED_OVR_RESET         bit(0)
#define EXEC_FEED_OVR_COARSE_PLUS   bit(1)
#define EXEC_FEED_OVR_COARSE_MINUS  bit(2)
#define EXEC_FEED_OVR_FINE_PLUS     bit(3)
#define EXEC_FEED_OVR_FINE_MINUS    bit(4)
#define EXEC_RAPID_OVR_RESET        bit(5)
#define EXEC_RAPID_OVR_MEDIUM       bit(6)
#define EXEC_RAPID_OVR_LOW          bit(7)
// #define EXEC_RAPID_OVR_EXTRA_LOW   bit(*) // *NOT SUPPORTED*

#define EXEC_SPINDLE_OVR_RESET         bit(0)
#define EXEC_SPINDLE_OVR_COARSE_PLUS   bit(1)
#define EXEC_SPINDLE_OVR_COARSE_MINUS  bit(2)
#define EXEC_SPINDLE_OVR_FINE_PLUS     bit(3)
#define EXEC_SPINDLE_OVR_FINE_MINUS    bit(4)
#define EXEC_SPINDLE_OVR_STOP          bit(5)
#define EXEC_COOLANT_FLOOD_OVR_TOGGLE  bit(6)
#define EXEC_COOLANT_MIST_OVR_TOGGLE   bit(7)

//定义系统状态位图。state变量主要跟踪Grbl的各个功能，以便在不重叠的情况下管理每个功能。它还用作关键事件的消息传递标志。
#define STATE_IDLE          0//必须是零。没有标志。
#define STATE_ALARM         bit(0)//处于报警状态。锁定所有g代码进程。允许设置访问。
#define STATE_CHECK_MODE    bit(1)//G代码检查模式。仅锁定“规划器”和“运动”。
#define STATE_HOMING        bit(2)//执行归位循环
#define STATE_CYCLE         bit(3)//正在运行循环或正在执行运动。
#define STATE_HOLD          bit(4)//激活进给保持
#define STATE_JOG           bit(5)//点动模式。
#define STATE_SAFETY_DOOR   bit(6)//安全门半开着。进给保持并断开系统电源。
#define STATE_SLEEP         bit(7)//睡眠状态。

//定义系统挂起标志。以各种方式用于管理挂起状态和过程。
#define SUSPEND_DISABLE           0//必须是零。
#define SUSPEND_HOLD_COMPLETE     bit(0)//表示初始进给保持完成。
#define SUSPEND_RESTART_RETRACT   bit(1)//指示从恢复停靠运动中收回的标志。
#define SUSPEND_RETRACT_COMPLETE  bit(2)//（仅限安全门）表示缩回和断电完成。
#define SUSPEND_INITIATE_RESTORE  bit(3)//（仅限安全门）从循环开始启动恢复程序的标志。
#define SUSPEND_RESTORE_COMPLETE  bit(4)//（仅限安全门）表示准备恢复正常操作。
#define SUSPEND_SAFETY_DOOR_AJAR  bit(5)//跟踪安全门状态以恢复。
#define SUSPEND_MOTION_CANCEL     bit(6)//指示已取消的恢复动作。当前由探测例程使用。
#define SUSPEND_JOG_CANCEL        bit(7)//指示进程中的点动取消，并在完成时重置缓冲区。

//定义步进段生成器状态标志。
#define STEP_CONTROL_NORMAL_OP            0//必须是零。
#define STEP_CONTROL_END_MOTION           bit(0)
#define STEP_CONTROL_EXECUTE_HOLD         bit(1)
#define STEP_CONTROL_EXECUTE_SYS_MOTION   bit(2)
#define STEP_CONTROL_UPDATE_SPINDLE_PWM   bit(3)

//定义Grbl内部使用的控制引脚索引。管脚映射可能会更改，但这些值不会更改。
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
  #define N_CONTROL_PIN 4
  #define CONTROL_PIN_INDEX_SAFETY_DOOR   bit(0)
  #define CONTROL_PIN_INDEX_RESET         bit(1)
  #define CONTROL_PIN_INDEX_FEED_HOLD     bit(2)
  #define CONTROL_PIN_INDEX_CYCLE_START   bit(3)
#else
  #define N_CONTROL_PIN 3
  #define CONTROL_PIN_INDEX_RESET         bit(0)
  #define CONTROL_PIN_INDEX_FEED_HOLD     bit(1)
  #define CONTROL_PIN_INDEX_CYCLE_START   bit(2)
#endif

//定义主轴停止覆盖控制状态。
#define SPINDLE_STOP_OVR_DISABLED       0//必须是零。
#define SPINDLE_STOP_OVR_ENABLED        bit(0)
#define SPINDLE_STOP_OVR_INITIATE       bit(1)
#define SPINDLE_STOP_OVR_RESTORE        bit(2)
#define SPINDLE_STOP_OVR_RESTORE_CYCLE  bit(3)


//定义全局系统变量
typedef struct {
  uint8_t state;//跟踪Grbl的当前系统状态。
  uint8_t abort;//系统中止标志。强制退出回到主循环进行复位。
  uint8_t suspend;//系统挂起标志位变量，用于管理保持、取消和安全门。
  uint8_t soft_limit;//跟踪状态机的软限位错误。（布尔值）
  uint8_t step_control;//根据系统状态控制步进段生成器。
  uint8_t probe_succeeded;//跟踪上一个探测周期是否成功。
  uint8_t homing_axis_lock;//当限位装置接合时锁定轴。在步进ISR中用作轴运动掩码。
  #ifdef ENABLE_DUAL_AXIS
    uint8_t homing_axis_lock_dual;
  #endif
  uint8_t f_override;//进给速度覆盖值（以百分比为单位）
  uint8_t r_override;//急流覆盖值（以百分比表示）
  uint8_t spindle_speed_ovr;//主轴速度值（百分比）
  uint8_t spindle_stop_ovr;//跟踪主轴停止覆盖状态
  uint8_t report_ovr_counter;//跟踪何时向状态报告添加覆盖数据。
  uint8_t report_wco_counter;//跟踪何时将工作坐标偏移数据添加到状态报告。
  #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    uint8_t override_ctrl;     // Tracks override control states.
  #endif
  #ifdef VARIABLE_SPINDLE
    float spindle_speed;
  #endif
} system_t;
extern system_t sys;

//注：如果出现问题，这些位置变量可能需要声明为易失性。
extern int32_t sys_position[N_AXIS]; // 实时机器（比如原点）矢量位置，以步为单位。
extern int32_t sys_probe_position[N_AXIS];//机器坐标和步骤中的最后一个探针位置。

extern volatile uint8_t sys_probe_state;//探测状态值。用于与步进式ISR协调探测周期。
extern volatile uint8_t sys_rt_exec_state;//用于状态管理的全局实时执行器位标志变量。请参阅EXEC位掩码。
extern volatile uint8_t sys_rt_exec_alarm;//全局实时执行器bitflag变量，用于设置各种报警。
extern volatile uint8_t sys_rt_exec_motion_override;//基于运动的覆盖的全局实时执行器位标志变量。
extern volatile uint8_t sys_rt_exec_accessory_override;//主轴/冷却液覆盖的全局实时执行器位标志变量。

#ifdef DEBUG
  #define EXEC_DEBUG_REPORT  bit(0)
  extern volatile uint8_t sys_rt_exec_debug;
#endif

//初始化串行协议
void system_init();

//返回控制引脚状态的位字段，按控制引脚索引组织。（1=已触发，0=未触发）。
uint8_t system_control_get_state();

//根据引脚状态返回安全门是否打开或关闭。
uint8_t system_check_safety_door_ajar();

//执行内部系统命令，该命令定义为以“$”开头的字符串
uint8_t system_execute_line(char *line);

//初始化时执行EEPROM中存储的启动脚本行
void system_execute_startup(char *line);


void system_flag_wco_change();

//返回轴“idx”的机器位置。必须发送一个“步”数组。
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx);

//根据发送的“步”数组更新机器“位置”数组。
void system_convert_array_steps_to_mpos(float *position, int32_t *steps);

//仅限CoreXY计算。基于CoreXY电机步数返回x轴或y轴“步数”。
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps);
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps);
#endif

//检查并报告目标阵列是否超过机器行程限制。
uint8_t system_check_travel_limits(float *target);

//用于设置和清除Grbl实时执行标志的特殊处理程序。
void system_set_exec_state_flag(uint8_t mask);
void system_clear_exec_state_flag(uint8_t mask);
void system_set_exec_alarm(uint8_t code);
void system_clear_exec_alarm();
void system_set_exec_motion_override_flag(uint8_t mask);
void system_set_exec_accessory_override_flag(uint8_t mask);
void system_clear_exec_motion_overrides();
void system_clear_exec_accessory_overrides();


#endif
