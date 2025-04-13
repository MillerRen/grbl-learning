/*
  stepper.h - 步进电机驱动器：使用步进电机执行planner.c的运动计划
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef stepper_h
#define stepper_h

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

//初始化并设置步进电机子系统
void stepper_init();

//启用步进器，但除非由运动控制或实时命令调用，否则循环不会启动。
void st_wake_up();

//立即禁用步进器
void st_go_idle();

// 生成步进和方向端口反转掩码。
void st_generate_step_dir_invert_masks();

//重置步进机子系统变量
void st_reset();

//更改步进段缓冲区的运行状态以执行特殊停车动作。
void st_parking_setup_buffer();

//停车运动后，将步进段缓冲区恢复到正常运行状态。
void st_parking_restore_buffer();

//重新加载步进段缓冲区。由实时执行系统连续调用。
void st_prep_buffer();

//当执行块由新计划更新时，由planner_recalculate（）调用。
void st_update_plan_block_parameters();

//如果在配置中启用了实时速率报告，则由实时状态报告调用。H
float st_get_realtime_rate();

#endif
