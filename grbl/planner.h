/*
  planner.c - 缓冲移动命令并管理加速度剖面计划
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef planner_h
#define planner_h


//在任何给定时间可以在计划中出现的线性运动的数量
#ifndef BLOCK_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define BLOCK_BUFFER_SIZE 15
  #else
    #define BLOCK_BUFFER_SIZE 16
  #endif
#endif

//从计划器返回状态消息。
#define PLAN_OK true
#define PLAN_EMPTY_BLOCK false

//定义计划器数据条件标志。用于表示块的运行条件。
#define PL_COND_FLAG_RAPID_MOTION      bit(0)
#define PL_COND_FLAG_SYSTEM_MOTION     bit(1)//单个动作。绕过计划者状态。用于归位/停靠。
#define PL_COND_FLAG_NO_FEED_OVERRIDE  bit(2)//运动不支持进给覆盖。
#define PL_COND_FLAG_INVERSE_TIME      bit(3)//设置时，将进给速度值解释为反时间。
#define PL_COND_FLAG_SPINDLE_CW        bit(4)
#define PL_COND_FLAG_SPINDLE_CCW       bit(5)
#define PL_COND_FLAG_COOLANT_FLOOD     bit(6)
#define PL_COND_FLAG_COOLANT_MIST      bit(7)
#define PL_COND_MOTION_MASK    (PL_COND_FLAG_RAPID_MOTION|PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE)
#define PL_COND_SPINDLE_MASK   (PL_COND_FLAG_SPINDLE_CW|PL_COND_FLAG_SPINDLE_CCW)
#define PL_COND_ACCESSORY_MASK (PL_COND_FLAG_SPINDLE_CW|PL_COND_FLAG_SPINDLE_CCW|PL_COND_FLAG_COOLANT_FLOOD|PL_COND_FLAG_COOLANT_MIST)


//此结构存储g代码块运动的线性运动，其临界“标称”值如源g代码中所规定。
typedef struct {
//bresenham算法用于跟踪直线的字段
//注：步进算法用于正确执行块。不要改变这些值。
  uint32_t steps[N_AXIS];//沿每个轴的步数
  uint32_t step_event_count;//完成此块所需的最大步长轴计数和步长数。
  uint8_t direction_bits;//为该块设置的方向位（参考config.h中的*\u direction\u位）

//根据状态和覆盖阻止条件数据以确保正确执行。
  uint8_t condition;//定义块运行条件的块位标志变量。从pl_line_data复制。
  #ifdef USE_LINE_NUMBERS
    int32_t line_number;//用于实时报告的块行号。从pl_line_data复制。
  #endif

//运动规划器用于管理加速度的字段。
//为了重新规划，在执行特殊运动情况期间，步进器模块可能会更新其中一些值。
  float entry_speed_sqr;//节点的当前计划进入速度（mm/min）^2
  float max_entry_speed_sqr;//基于最小节点限制和相邻标称速度的最大允许进入速度（mm/min）^2
  float acceleration;//轴限制调整线加速度（mm/min^2）。不会改变。
  float millimeters;//要执行此块的剩余距离（mm）。 注意：在执行过程中，此值可能会被步进算法更改。

//发生更改时，规划器使用的存储速率限制数据。
  float max_junction_speed_sqr;//基于方向矢量的节点入口速度限制（mm/min）^2
  float rapid_rate;//该块方向的轴限制调整最大速率（mm/min）
  float programmed_rate;//此块的编程速率（mm/min）。

  #ifdef VARIABLE_SPINDLE
//由主轴覆盖和恢复方法使用的存储主轴速度数据。
    float spindle_speed;//块主轴转速。从pl_line_data复制。
  #endif
} plan_block_t;


//规划器数据原型。将新动作传递给规划器时必须使用。
typedef struct {
  float feed_rate;//直线运动所需的进给速度。如果快速运动，则忽略该值。
  float spindle_speed;//通过直线运动所需的主轴速度。
  uint8_t condition;//指示计划器条件的Bitflag变量。见上文定义。
  #ifdef USE_LINE_NUMBERS
    int32_t line_number;//执行时要报告的所需行号。
  #endif
} plan_line_data_t;


//初始化并重置运动计划子系统
void plan_reset();//全部重置
void plan_reset_buffer();//仅重置缓冲区。

//向缓冲区添加新的线性移动。

//target[N_AXIS]是有符号的绝对目标位置，单位为毫米。
//进给速率指定运动的速度。如果进给速度倒置，则进给速度被视为“频率”，并将在1/进给速度分钟内完成操作。
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data);

//当不再需要当前块时调用。丢弃该块并使内存可用于新块。
void plan_discard_current_block();

//获取特殊系统运动情况的规划器块。（停车/归位）
plan_block_t *plan_get_system_motion_block();

//获取当前块。如果缓冲区为空，则返回NULL
plan_block_t *plan_get_current_block();

//由分步段缓冲区定期调用。主要由计划者内部使用。
uint8_t plan_next_block_index(uint8_t block_index);

//在计算执行块速度剖面时，由步进段缓冲区调用。
float plan_get_exec_block_exit_speed_sqr();

//在规划器计算期间由主程序调用，在初始化期间由步进段缓冲区调用。
float plan_compute_profile_nominal_speed(plan_block_t *block);

//根据基于运动的覆盖更改重新计算缓冲运动剖面参数。
void plan_update_velocity_profile_parameters();

//重置规划器位置向量（以步为单位）
void plan_sync_position();

//使用部分完成的块重新初始化计划
void plan_cycle_reinitialize();

//返回规划器缓冲区中的可用块数。
uint8_t plan_get_block_buffer_available();

//返回planner缓冲区中的活动块数。
//注意：已弃用。除非在配置中(config.h)启用了经典状态报告，否则不使用
uint8_t plan_get_block_buffer_count();

//返回块环形缓冲区的状态。如果缓冲区已满，则为True。
uint8_t plan_check_full_buffer();

void plan_get_planner_mpos(float *target);


#endif
