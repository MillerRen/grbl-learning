/*
  stepper.c - 步进电机驱动器：使用步进电机执行planner.c的运动计划
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


//一些有用的常数。
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // 分钟/段
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2
#define RAMP_DECEL_OVERRIDE 3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_PARKING bit(2)
#define PREP_FLAG_DECEL_OVERRIDE bit(3)

//定义自适应多轴步进平滑（AMASS）级别和截止频率。
//最高电平频率槽开始于0Hz，结束于其截止频率。
//下一个低电平频率单元从下一个高截止频率开始，依此类推。
//必须仔细考虑每个级别的截止频率对步进ISR的过度驱动程度、16位计时器的精度和CPU开销。
//0级（无AMASS，正常运行）频率槽从1级截止频率开始，并以CPU允许的最快速度（在有限测试中超过30kHz）。
//注：AMASS截止频率乘以ISR超速驱动系数不得超过最大步进频率。
//注意：当前设置将ISR超速驱动至不超过16kHz，以平衡CPU开销和计时器精度。除非您知道自己在做什么，否则不要更改这些设置。
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
	#define MAX_AMASS_LEVEL 3
	//AMASS_LEVEL0：正常运行。没有积累。没有上限截止频率。从1级截止频率开始。
	#define AMASS_LEVEL1 (F_CPU/8000) //Over-drives ISR（x2）。定义为F_CPU/（截止频率，单位为Hz）
	#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
	#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)

  #if MAX_AMASS_LEVEL <= 0
    error "AMASS must have 1 or more levels to operate correctly."
  #endif
#endif


//将分段的规划器块Bresenham算法执行数据存储在分段缓冲区中。
//通常，该缓冲区部分处于使用状态，但在最坏情况下，它永远不会超过可访问的步进缓冲区段数（段缓冲区大小-1）。
//注意：此数据是从预先准备好的计划程序块复制而来的，这样，当计划程序块被段缓冲区完全使用和完成时，就可以丢弃这些计划程序块。
//此外，AMASS会更改这些数据以供自己使用。
typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint8_t direction_bits;
  #ifdef ENABLE_DUAL_AXIS
    uint8_t direction_bits_dual;
  #endif
  #ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; //跟踪需要恒定激光功率/速率的运动
  #endif
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

//主步进段环形缓冲区。
//包含用于执行步进算法的小而短的线段，这些线段从planner缓冲区中的第一个块开始递增“签出”。
//一旦“签出”，规划器就不能修改段缓冲区中的步，剩余的规划器块步仍然可以修改。
typedef struct {
  uint16_t n_step;           //要为此段执行的步事件数
  uint16_t cycles_per_tick;  //ISR周期移动的步距，也称为步率。
  uint8_t  st_block_index;   //步进块数据索引。使用此信息执行此段。
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t AMASS_level;    //指示ISR执行此段的AMASS级别
  #else
    uint8_t prescaler;      //如果没有AMASS，需要一个预分频器来调整缓慢的定时。
  #endif
  #ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

//步进ISR数据结构。包含主步进电机ISR的运行数据。
typedef struct {
  //由bresenham直线算法使用
  uint32_t counter_x,        //bresenham线跟踪器的计数器变量
           counter_y,
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  //存储out_bits输出以完成步进脉冲延迟
  #endif

  uint8_t execute_step;     //为每个中断标记步执行。
  uint8_t step_pulse_time;  // 在步进上升后步进脉冲重置时间。
  uint8_t step_outbits;         //要输出的下一个步进位
  uint8_t dir_outbits;
  #ifdef ENABLE_DUAL_AXIS
    uint8_t step_outbits_dual;
    uint8_t dir_outbits_dual;
  #endif
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       //直线段运动中的剩余步数
  uint8_t exec_block_index; //跟踪当前st_block索引。更改表示新块。
  st_block_t *exec_block;   //指向正在执行的段的块数据的指针
  segment_t *exec_segment;  //指向正在执行的段的指针
} stepper_t;
static stepper_t st;

//步进段环形缓冲区索引
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// 步进和方向端口反转掩码
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;
#ifdef ENABLE_DUAL_AXIS
  static uint8_t step_port_invert_mask_dual;
  static uint8_t dir_port_invert_mask_dual;
#endif

// 用于避免“步进驱动程序中断”的ISR嵌套。但这应该永远不会发生。
static volatile uint8_t busy;

//从规划器缓冲区准备的步进段的指针。只能由主程序访问。
//指针可能是计划段或计划块，位于执行的内容之前。
static plan_block_t *pl_block;     //指向正在准备的计划程序块的指针
static st_block_t *st_prep_block;  //指向正在准备的步进程序块数据的指针

//准备段数据结构。
//包含根据当前执行的规划器块计算新线段所需的所有信息。
typedef struct {
  uint8_t st_block_index;  //正在准备的步进机公用数据块的索引
  uint8_t recalculate_flag;

  float dt_remainder;
  float steps_remaining;
  float step_per_mm;
  float req_mm_increment;

  #ifdef PARKING_ENABLE
    uint8_t last_st_block_index;
    float last_steps_remaining;
    float last_step_per_mm;
    float last_dt_remainder;
  #endif

  uint8_t ramp_type;      //当前段斜坡状态
  float mm_complete;      //距离当前计划块末端的速度剖面末端（毫米）。
                          //注意：转换时，此值必须与步数（无小数）一致。
  float current_speed;    //段缓冲区末端的当前速度（毫米/分钟）
  float maximum_speed;    //执行块的最大速度。不总是标称速度。（毫米/分钟）
  float exit_speed;       //执行块退出速度（毫米/分钟）
  float accelerate_until; //从块端测量的加速度斜坡端（毫米）
  float decelerate_after; //减速坡道起点从挡块末端测量（毫米）

  #ifdef VARIABLE_SPINDLE
    float inv_rate;    //PWM激光模式用于加速分段计算。
    uint8_t current_spindle_pwm; 
  #endif
} st_prep_t;
static st_prep_t prep;


/*    块体速度剖面定义
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  时间 ----->      示例：块2入口速度为最大接合速度

  规划器块缓冲区的规划假设为恒定加速度速度剖面，并在块连接处连续连接，如上所示。
  但是，规划器仅主动计算最优速度计划的区块进入速度，而不计算区块内部速度剖面。
  这些速度剖面是通过步进算法临时计算的，仅包括7种可能的剖面类型：仅巡航、巡航减速、加速巡航、仅加速、仅减速、全梯形和三角形（无巡航）。
 
                                        maximum_speed (< nominal_speed) ->  +
                    +--------+ <- maximum_speed (= nominal_speed)          /|\
                   /          \                                           / | \
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                     current_speed -> +----+--+
                   time -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)

  步进段缓冲区计算执行块速度剖面，并跟踪步进算法的关键参数，以准确跟踪剖面。
  上图显示并定义了这些关键参数。
*/


//步进状态初始化。仅当st.Cycle_start标志启用时，循环才应开始。
//启动初始化和限位会调用此函数，但不应启动循环。
void st_wake_up()
{
  //启用步进驱动程序。
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
  else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }

  //初始化步进器输出位，以确保第一次ISR调用不步进。
  st.step_outbits = step_port_invert_mask;

  //从设置中初始化步进脉冲定时。此处用于确保重新写入后进行更新。
  #ifdef STEP_PULSE_DELAY
    //设置方向引脚设置后的总步进脉冲时间。从示波器进行特别计算。
    st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
    //设置方向引脚写入和步进命令之间的延迟。
    OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
  #else // Normal operation
    // 设置步进脉冲时间。 从示波器进行特别计算。 使用二的补？。
    st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
  #endif

  //启用步进驱动程序中断
  TIMSK1 |= (1<<OCIE1A);
}


//步进停机
void st_go_idle()
{
  //禁用步进驱动程序中断。如果激活，允许步进器端口重置中断完成。
  TIMSK1 &= ~(1<<OCIE1A); //禁用定时器1中断
  TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); //将时钟重置为无预刻度。
  busy = false;

  //根据设置和环境，设置步进驱动程序空闲状态、禁用或启用。
  bool pin_state = false; //保持启用状态。
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
    //强制步进电机保持锁定轴一段规定的时间，以确保轴完全停止，并且在最后一次移动结束时不会从残余惯性力中漂移。
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // 覆盖， 禁用步进器。
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } //应用引脚反转。
  if (pin_state) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
  else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }
}


/* “步进驱动程序中断”-此定时器中断是Grbl的主要工作。
   Grbl采用久负盛名的Bresenham直线算法来管理和精确同步多轴移动。
   与流行的DDA算法不同，Bresenham算法不受数值舍入误差的影响，只需要快速整数计数器，这意味着计算开销较低，并最大限度地提高了Arduino的性能。
   然而，Bresenham算法的缺点是，对于某些多轴运动，非主导轴可能会受到不平滑的步进脉冲序列或混叠的影响，这可能导致奇怪的可听噪声或抖动。
   这一点特别明显，或者在低步进频率（0-5kHz）下可能会导致运动问题，但在较高频率下通常不是物理问题，尽管可以听到。
     为了提高Bresenham多轴性能，Grbl使用了我们称之为自适应多轴步长平滑（AMASS）算法，该算法实现了顾名思义的功能。
   在较低的步进频率下，AMASS人为地提高了Bresenham分辨率，而不影响算法固有的精确性。
   AMASS根据要执行的步进频率自动调整其分辨率级别，这意味着对于更低的步进频率，步进平滑级别也会增加。
   从算法上讲，对于每个AMASS级别，通过对Bresenham步计数进行简单的位移位来实现AMASS。

   例如，对于1级步长平滑，我们对Bresenham步长事件计数进行位移位，有效地将其乘以2，而轴步长计数保持不变，然后将步进机ISR频率加倍。
   实际上，我们允许非主导的Bresenham轴在中间ISR记号中步进，而主导轴每两个ISR记号步进一次，而不是传统意义上的每一个ISR记号步进一次。
   在AMASS级别2，我们只需再次进行位移位，这样非主导Bresenham轴就可以在四个ISR信号中的任何一个信号中进行步进，主导轴每四个ISR信号步进一次，步进机ISR频率增加四倍。等等.
   这实际上消除了Bresenham算法的多轴混叠问题，并且不会显著改变Grbl的性能，但事实上，在所有配置中更有效地利用未使用的CPU周期。 
    AMASS通过要求始终执行完整的Bresenham步骤来保持Bresenham算法的准确性，而不管AMASS的级别如何。
   这意味着对于AMASS 2级，必须完成所有四个中间步骤，以便始终保留基线Bresenham（0级）计数。 
   类似地，AMASS级别3意味着必须执行所有八个中间步骤。
   尽管AMASS级别实际上是任意的，基线Bresenham计数可以与任何整数值相乘，但二次幂的乘法仅用于通过位移位整数操作减轻CPU开销。  
   这个中断设计得简单而愚蠢。
     在确定加速度时，所有的计算性重载都在别处进行。
   该中断从步进段缓冲区弹出预先计算的段（定义为n步数上的恒定速度），然后通过Bresenham算法适当地脉冲步进器引脚来执行这些段。
   该ISR由步进器端口重置中断支持，该中断用于在每次脉冲后重置步进器端口。
   bresenham 线跟踪算法通过这两个中断同时控制所有步进器输出。
   注：该中断必须尽可能有效，并在下一个ISR周期前完成，Grbl的周期必须小于33.3usec（@30kHz ISR速率）。
   示波器在ISR中测量的时间通常为5usec，最大为25usec，远低于要求。
   注：本ISR要求每个段至少执行一个步骤。
*/
// TODO:以某种方式替换ISR中int32位置计数器的直接更新。 
//可能使用较小的int8变量，并仅在段完成时更新位置计数器。
//由于探测和寻的周期需要真正的实时位置，这可能会变得复杂。
ISR(TIMER1_COMPA_vect)
{
  if (busy) { return; } //忙标志用于避免重新进入该中断

  //在我们步进步进器之前，将方向引脚设置几纳秒
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);
  #ifdef ENABLE_DUAL_AXIS
    DIRECTION_PORT_DUAL = (DIRECTION_PORT_DUAL & ~DIRECTION_MASK_DUAL) | (st.dir_outbits_dual & DIRECTION_MASK_DUAL);
  #endif

  // Then pulse the stepping pins
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; //存储_位以防止覆盖。
    #ifdef ENABLE_DUAL_AXIS
      st.step_bits_dual = (STEP_PORT_DUAL & ~STEP_MASK_DUAL) | st.step_outbits_dual;
    #endif
  #else  // Normal operation
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
    #ifdef ENABLE_DUAL_AXIS
      STEP_PORT_DUAL = (STEP_PORT_DUAL & ~STEP_MASK_DUAL) | st.step_outbits_dual;
    #endif
  #endif

  //启用步进脉冲重置定时器，以便步进器端口重置中断能够在准确settings.pulse.microseconds微秒后重置信号，与主定时器1预分频器无关。
  TCNT0 = st.step_pulse_time; //重新加载计时器0计数器
  TCCR0B = (1<<CS01); //开始计时0。全速，1/8预分频器

  busy = true;
  sei(); //重新启用中断，以允许步进器端口重置中断按时触发。
//注意：此ISR中的剩余代码将在返回主程序之前完成。

  //如果没有步进段，尝试从步进缓冲区弹出一个
  if (st.exec_segment == NULL) {
    //缓冲区里有东西吗？如果是，加载并初始化下一步段。
    if (segment_buffer_head != segment_buffer_tail) {
      //初始化新步骤段并加载要执行的步骤数
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        //禁用AMASS时，为步频率较低（<250Hz）的分段设置定时器预分频器。
        TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
      #endif

      //初始化每个步骤的步骤段计时，并加载要执行的步骤数。
      OCR1A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step; //注意：缓慢移动时，有时可能为零。
//如果新段启动了新的规划器块，则初始化步进器变量和计数器。
//注意：当段数据索引更改时，这表示一个新的规划器块。
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        //初始化Bresenham测线和距离计数器
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;
      #ifdef ENABLE_DUAL_AXIS
        st.dir_outbits_dual = st.exec_block->direction_bits_dual ^ dir_port_invert_mask_dual;
      #endif

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        //启用AMASS后，根据AMASS级别调整Bresenham轴增量计数器。
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->AMASS_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->AMASS_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->AMASS_level;
      #endif

      #ifdef VARIABLE_SPINDLE
        //在第一步之前，在加载段时设置实时主轴输出。
        spindle_set_speed(st.exec_segment->spindle_pwm);
      #endif

    } else {
      //段缓冲区为空。关闭。
      st_go_idle();
      #ifdef VARIABLE_SPINDLE
        //完成速率控制运动后，确保pwm设置正确。
        if (st.exec_block->is_pwm_rate_adjusted) { spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
      #endif
      system_set_exec_state_flag(EXEC_CYCLE_STOP); //为循环结束标记主程序
      return; //除了退出别无选择。
    }
  }


  //检查探测状态。
  if (sys_probe_state == PROBE_ACTIVE) { probe_state_monitor(); }

  //重置步输出位。
  st.step_outbits = 0;
  #ifdef ENABLE_DUAL_AXIS
    st.step_outbits_dual = 0;
  #endif

  //用Bresenham线算法执行步进位移剖面
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    #if defined(ENABLE_DUAL_AXIS) && (DUAL_AXIS_SELECT == X_AXIS)
      st.step_outbits_dual = (1<<DUAL_STEP_BIT);
    #endif
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys_position[X_AXIS]--; }
    else { sys_position[X_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    #if defined(ENABLE_DUAL_AXIS) && (DUAL_AXIS_SELECT == Y_AXIS)
      st.step_outbits_dual = (1<<DUAL_STEP_BIT);
    #endif
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys_position[Y_AXIS]--; }
    else { sys_position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys_position[Z_AXIS]--; }
    else { sys_position[Z_AXIS]++; }
  }

  //在归位循环期间，锁定并防止所需轴移动。
  if (sys.state == STATE_HOMING) { 
    st.step_outbits &= sys.homing_axis_lock;
    #ifdef ENABLE_DUAL_AXIS
      st.step_outbits_dual &= sys.homing_axis_lock_dual;
    #endif
  }

  st.step_count--; //递减步事件计数
  if (st.step_count == 0) {
    //这一部分已经完成。放弃当前段并推进段索引。
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  st.step_outbits ^= step_port_invert_mask;  //应用步进端口反转掩码
  #ifdef ENABLE_DUAL_AXIS
    st.step_outbits_dual ^= step_port_invert_mask_dual;
  #endif
  busy = false;
}


/*步进器端口复位中断：Timer0 OVF中断处理步进脉冲的下降沿。
如果计时器1在完成移动后被禁用，则应始终在下一个计时器1 COMPA中断之前触发，并独立完成。
注：串行和步进中断之间的中断冲突可能会导致延迟几微秒，如果它们在彼此之前执行。
这没什么大不了的，但如果Grbl中添加了另一个高频异步中断，可能会导致高步进率问题。
*/
//当ISR_TIMER1_COMPAREA设置电机端口位以执行一个步骤时，该中断由ISR_TIMER1_COMPAREA启用。
//此ISR在短时间（settings.pulse_microseconds）完成一步循环后重置电机端口。
ISR(TIMER0_OVF_vect)
{
  //重置步进引脚（保留方向引脚）
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK);
  #ifdef ENABLE_DUAL_AXIS
    STEP_PORT_DUAL = (STEP_PORT_DUAL & ~STEP_MASK_DUAL) | (step_port_invert_mask_dual & STEP_MASK_DUAL);
  #endif
  TCCR0B = 0; //禁用定时器0以防止在不需要时重新进入此中断。
}
#ifdef STEP_PULSE_DELAY
  //此中断仅在启用步进脉冲延迟时使用。这里，在经过阶跃脉冲延迟时间段之后启动阶跃脉冲。ISR TIMER2_OVF中断将在适当pulse_microseconds后触发，如在正常操作中。
//方向、步进脉冲和步进完成事件之间的新计时在st_wake_up（）例程中设置。
  ISR(TIMER0_COMPA_vect)
  {
    STEP_PORT = st.step_bits; //开始步进脉冲。
    #ifdef ENABLE_DUAL_AXIS
      STEP_PORT_DUAL = st.step_bits_dual;
    #endif
  }
#endif


//生成步进中断驱动程序中使用的步进和方向端口反转掩码。
void st_generate_step_dir_invert_masks()
{
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= get_step_pin_mask(idx); }
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= get_direction_pin_mask(idx); }
  }
  #ifdef ENABLE_DUAL_AXIS
    step_port_invert_mask_dual = 0;
    dir_port_invert_mask_dual = 0;
    //注：双轴反转使用N_AXIS位设置步进和方向反转引脚。
    if (bit_istrue(settings.step_invert_mask,bit(N_AXIS))) { step_port_invert_mask_dual = (1<<DUAL_STEP_BIT); }
    if (bit_istrue(settings.dir_invert_mask,bit(N_AXIS))) { dir_port_invert_mask_dual = (1<<DUAL_DIRECTION_BIT); }
  #endif
}


//重置并清除步进机子系统变量
void st_reset()
{
  //初始化步进驱动程序空闲状态。
  st_go_idle();

  //初始化步进算法变量。
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  //段缓冲区使用的规划器块指针
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask; //将方向位初始化为默认值。

  //初始化步进和方向端口引脚。
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask;
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;
  
  #ifdef ENABLE_DUAL_AXIS
    st.dir_outbits_dual = dir_port_invert_mask_dual;
    STEP_PORT_DUAL = (STEP_PORT_DUAL & ~STEP_MASK_DUAL) | step_port_invert_mask_dual;
    DIRECTION_PORT_DUAL = (DIRECTION_PORT_DUAL & ~DIRECTION_MASK_DUAL) | dir_port_invert_mask_dual;
  #endif
}


//初始化并启动步进电机子系统
void stepper_init()
{
  //配置步进和方向接口引脚
  STEP_DDR |= STEP_MASK;
  STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
  DIRECTION_DDR |= DIRECTION_MASK;
  
  #ifdef ENABLE_DUAL_AXIS
    STEP_DDR_DUAL |= STEP_MASK_DUAL;
    DIRECTION_DDR_DUAL |= DIRECTION_MASK_DUAL;
  #endif

  //配置定时器1：步进驱动程序中断
  TCCR1B &= ~(1<<WGM13); // waveform generation = 0100 = CTC
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0)); // 断开OC1输出
  // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // 在st_go_idle（）中设置。
  // TIMSK1 &= ~(1<<OCIE1A);  // 在st_go_idle（）中设置。

  //配置计时器0：步进器端口重置中断
  TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); //断开OC0输出和OVF中断。
  TCCR0A = 0; //正常运行
  TCCR0B = 0; //在需要时禁用计时器0
  TIMSK0 |= (1<<TOIE0); //启用定时器0溢出中断
  #ifdef STEP_PULSE_DELAY
    TIMSK0 |= (1<<OCIE0A); //启用定时器0比较匹配中断
  #endif
}


//当执行块由新计划更新时，由planner_recalculate（）调用。
void st_update_plan_block_parameters()
{
  if (pl_block != NULL) { //如果在新块的开头，则忽略。
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; //更新进入速度。
    pl_block = NULL; //标记st_prep_segment（）以加载和检查活动速度剖面。
  }
}


//递增步段缓冲块数据环缓冲区。
static uint8_t st_next_block_index(uint8_t block_index)
{
  block_index++;
  if ( block_index == (SEGMENT_BUFFER_SIZE-1) ) { return(0); }
  return(block_index);
}


#ifdef PARKING_ENABLE
  //更改步进段缓冲区的运行状态以执行特殊停车动作。
  void st_parking_setup_buffer()
  {
    //如有必要，存储部分完成的块的步骤执行数据。
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      prep.last_st_block_index = prep.st_block_index;
      prep.last_steps_remaining = prep.steps_remaining;
      prep.last_dt_remainder = prep.dt_remainder;
      prep.last_step_per_mm = prep.step_per_mm;
    }
    //设置标志以执行停车运动
    prep.recalculate_flag |= PREP_FLAG_PARKING;
    prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
    pl_block = NULL; //始终重置停车运动以重新加载新块。
  }


  //停车运动后，将步进段缓冲区恢复到正常运行状态。
  void st_parking_restore_buffer()
  {
    //如有必要，恢复步骤执行数据和部分完成块的标志。
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      st_prep_block = &st_block_buffer[prep.last_st_block_index];
      prep.st_block_index = prep.last_st_block_index;
      prep.steps_remaining = prep.last_steps_remaining;
      prep.dt_remainder = prep.last_dt_remainder;
      prep.step_per_mm = prep.last_step_per_mm;
      prep.recalculate_flag = (PREP_FLAG_HOLD_PARTIAL_BLOCK | PREP_FLAG_RECALCULATE);
      prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm; //重新计算此值。
    } else {
      prep.recalculate_flag = false;
    }
    pl_block = NULL; //设置为重新加载下一个块。
  }
#endif


/*准备步进段缓冲区。从主程序连续调用。

分段缓冲区是步进算法执行步骤与规划器生成的速度剖面之间的中间缓冲界面。
步进算法仅在段缓冲区内执行步骤，当步骤从planner缓冲区的第一个块“签出”时，由主程序填充。
这使步骤执行和计划优化过程原子化，并相互保护。
从规划器缓冲区“签出”的步数和段缓冲区中的段数的大小和计算应确保主程序中的任何操作所花费的时间都不会超过重新填充之前步进算法清空的时间。
目前，段缓冲区保守地保持大约40-50毫秒的步长。
注：计算单位为步、毫米和分钟。
*/
void st_prep_buffer()
{
  //当处于挂起状态且没有要执行的挂起运动时，阻止步骤准备缓冲区。
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }

  while (segment_buffer_tail != segment_next_head) { //检查是否需要填充缓冲区。

    //确定是否需要加载新的规划器块或是否需要重新计算该块。
    if (pl_block == NULL) {

      //排队块的查询计划器
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) { pl_block = plan_get_system_motion_block(); }
      else { pl_block = plan_get_current_block(); }
      if (pl_block == NULL) { return; } //没有规划块。退出

      //检查是否只需要重新计算速度剖面或加载新块。
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        #ifdef PARKING_ENABLE
          if (prep.recalculate_flag & PREP_FLAG_PARKING) { prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE); }
          else { prep.recalculate_flag = false; }
        #else
          prep.recalculate_flag = false;
        #endif

      } else {

        //加载块的Bresenham步进数据。
        prep.st_block_index = st_next_block_index(prep.st_block_index);

        //准备并复制新规划器块中的Bresenham算法段数据，以便当段缓冲区完成规划器块时，当段缓冲区完成预处理的块时，该数据可能会被丢弃，但步进程序ISR仍在执行该数据。
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        #ifdef ENABLE_DUAL_AXIS
          #if (DUAL_AXIS_SELECT == X_AXIS)
            if (st_prep_block->direction_bits & (1<<X_DIRECTION_BIT)) { 
          #elif (DUAL_AXIS_SELECT == Y_AXIS)
            if (st_prep_block->direction_bits & (1<<Y_DIRECTION_BIT)) { 
          #endif
            st_prep_block->direction_bits_dual = (1<<DUAL_DIRECTION_BIT); 
          }  else { st_prep_block->direction_bits_dual = 0; }
        #endif
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = (pl_block->steps[idx] << 1); }
          st_prep_block->step_event_count = (pl_block->step_event_count << 1);
        #else
          //启用AMASS后，只需将所有Bresenham数据乘以最大AMASS级别，这样我们就不会在算法中的任何地方除原始数据之外。
//如果对原始数据进行分割，我们可能会丢失整数舍入的一步。
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL; }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif

        //初始化用于生成段的段缓冲区数据。
        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0; //重置新的段块

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          //新块加载中间保持。覆盖规划器块进入速度以强制减速。
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrt(pl_block->entry_speed_sqr);
        }
        
        #ifdef VARIABLE_SPINDLE
          //设置激光模式变量。PWM速率调整运动始终会在主轴关闭的情况下完成运动。
          st_prep_block->is_pwm_rate_adjusted = false;
          if (settings.flags & BITFLAG_LASER_MODE) {
            if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW) { 
              //预先计算逆编程速率，以加快每个步长段的PWM更新。
              prep.inv_rate = 1.0/pl_block->programmed_rate;
              st_prep_block->is_pwm_rate_adjusted = true; 
            }
          }
        #endif
      }

			/*---------------------------------------------------------------------------------
        根据新规划器块的入口和出口速度计算其速度剖面，或者如果规划器已更新部分完成的规划器块，则重新计算其剖面。对于指令强制减速，例如从进料保持，超越规划器速度并减速至目标出口速度。
      */
			prep.mm_complete = 0.0; //默认速度剖面在距块端0.0mm处完成。
			float inv_2_accel = 0.5/pl_block->acceleration;
			if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { //[强制减速至零速度]
//计算正在进行的进给保持的速度剖面参数。如果将此减速外形强制为零，则减速外形将覆盖块。
				prep.ramp_type = RAMP_DECEL;
				//计算相对于块末端的减速距离。
				float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
				if (decel_dist < 0.0) {
					//减速通过整个规划块。进给保持结束不在此块中。
					prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
				} else {
					prep.mm_complete = decel_dist; //进给保持结束。
					prep.exit_speed = 0.0;
				}
			} else { //[正常操作]
//计算或重新计算准备好的计划块的速度剖面参数。
				prep.ramp_type = RAMP_ACCEL; //初始化为加速斜坡。
				prep.accelerate_until = pl_block->millimeters;

				float exit_speed_sqr;
				float nominal_speed;
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          prep.exit_speed = exit_speed_sqr = 0.0; //在系统运动结束时强制停止。
        } else {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrt(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
				float nominal_speed_sqr = nominal_speed*nominal_speed;
				float intersect_distance =
								0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr) { //仅在覆盖减少期间发生。
          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0) { //只有减速。
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            //计算覆盖块退出速度，因为它与规划器退出速度不匹配。
            prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; //将下一个块加载为减速覆盖的标志。

            //TODO：仅在减速时确定参数的正确处理。
//这可能会很棘手，因为进入速度将是当前速度，如进给保持。
//此外，还可以研究这种接近零速度的处理问题。

          } else {
            //减速至巡航或巡航减速类型。保证与更新的计划相交。
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr); // Should always be >= 0.0 due to planner reinit.
            prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
				} else if (intersect_distance > 0.0) {
					if (intersect_distance < pl_block->millimeters) { //梯形或三角形类型
//注：对于加速巡航和仅巡航类型，以下计算将为0.0。
						prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
						if (prep.decelerate_after < intersect_distance) { //梯形型
							prep.maximum_speed = nominal_speed;
							if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
								//巡航减速或仅巡航类型。
								prep.ramp_type = RAMP_CRUISE;
							} else {
								//全梯形或加速巡航类型
								prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
							}
						} else { //三角形类型
							prep.accelerate_until = intersect_distance;
							prep.decelerate_after = intersect_distance;
							prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
						}
					} else { //仅减速型
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
					}
				} else { //仅加速度型
					prep.accelerate_until = 0.0;
					// prep.decelerate_after = 0.0;
					prep.maximum_speed = prep.exit_speed;
				}
			}
      
      #ifdef VARIABLE_SPINDLE
        bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); //每当更新块时强制更新。
      #endif
    }
    
    //初始化新段
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    //将新线段设置为指向当前线段数据块。
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        通过确定路段时间DT_SEGMENT上行驶的总距离，计算新路段的平均速度。
      以下代码首先尝试基于当前斜坡条件创建完整路段。
      如果在斜坡状态更改时终止时段时间不完整，代码将继续在斜坡状态中循环，以填充剩余的段执行时间。
      但是，如果一个不完整的段在速度剖面的末尾终止，则该段被视为已完成，尽管其截断执行时间小于DT_SEGMENT。  
      始终假定速度剖面在斜坡序列中前进：
      加速斜坡、巡航状态和减速斜坡。
      每个斜坡的行驶距离可能从零到块的长度。
      速度曲线可以在计划块（典型）的末端结束，也可以在强制减速的末端（例如从进给保持）的中间块结束。 */
    float dt_max = DT_SEGMENT; //最大分段时间
    float dt = 0.0; //初始化段时间
    float time_var = dt_max; //时间工作者变量
    float mm_var; //毫米距离工作变量
    float speed_var; //速度工作者变量
    float mm_remaining = pl_block->millimeters; //新线段到块末端的距离。
    float minimum_mm = mm_remaining-prep.req_mm_increment; //保证至少一步。
    if (minimum_mm < 0.0) { minimum_mm = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
          if (prep.current_speed-prep.maximum_speed <= speed_var) {
            //巡航或巡航减速类型仅适用于减速覆盖。
            mm_remaining = prep.accelerate_until;
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else { //中间斜坡减速覆盖。
            mm_remaining -= time_var*(prep.current_speed - 0.5*speed_var);
            prep.current_speed -= speed_var;
          }
          break;
        case RAMP_ACCEL:
          //注意：加速斜坡仅在第一个do while循环期间计算。
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { //加速坡道的终点。 加速巡航、加速减速斜坡结点或块终点。
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { //只有加速。
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE:
          //注：mm_var用于保留最后剩余的mm_remaining，用于不完整的分段time_ var计算。
//注意：如果最大速度*时间变量值过低，舍入可能会导致mm_var不变。要防止出现这种情况，只需在规划器中强制执行最低速度阈值。
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { //巡航结束。巡航减速结点或结束块。
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { //仅限巡航。
            mm_remaining = mm_var;
          }
          break;
        default: //斜坡减速情况：注：mm_var用作misc辅助变量，以防止接近零速度时出现错误。
          speed_var = pl_block->acceleration*time_var; //用作增量速度（毫米/分钟）
          if (prep.current_speed > speed_var) { //检查是否处于或低于零速。计算段末端到块末端的距离。
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); //（毫米）
            if (mm_var > prep.mm_complete) { //典型案例。在减速坡道上。
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; //段完成。退出开关案例语句。继续边做边循环。
            }
          }
          //否则，在挡块末端或强制减速末端。
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed;
      }
      dt += time_var; //将计算的爬坡时间添加到总分段时间。
      if (dt < dt_max) { time_var = dt_max - dt; } // **未完成** 在斜坡结点。
      else {
        if (mm_remaining > minimum_mm) { //检查零步距的非常慢的段。
//增加分段时间，以确保分段中至少有一个步骤。覆盖并循环计算距离，直到完成最小距离或最小距离。
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; //**完成**退出循环。段执行时间已达到最大值。
        }
      }
    } while (mm_remaining > prep.mm_complete); //**完成**退出循环。剖面完成。

    #ifdef VARIABLE_SPINDLE
      /*-----------------------------------------------------------------------------------
      计算步进段的主轴转速PWM输出
      */
      
      if (st_prep_block->is_pwm_rate_adjusted || (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
        if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)) {
          float rpm = pl_block->spindle_speed;
          //注：进给和快速超越与PWM值无关，不会改变激光功率/速率。
          if (st_prep_block->is_pwm_rate_adjusted) { rpm *= (prep.current_speed * prep.inv_rate); }
          //如果当前速度为零，则可能需要rpm_min*（100/MAX_SPINDLE_SPEED_OVERRIDE），但这仅在运动过程中是瞬时的。可能根本不用关心。
          prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
        } else { 
          sys.spindle_speed = 0.0;
          prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
        }
        bit_false(sys.step_control,STEP_CONTROL_UPDATE_SPINDLE_PWM);
      }
      prep_segment->spindle_pwm = prep.current_spindle_pwm; //重新加载段PWM值

    #endif
    
    /* -----------------------------------------------------------------------------------
       计算分段步进速率、要执行的步进，并应用必要的速率修正。
       注意：步数是通过块中剩余毫米距离的直接标量转换来计算的，而不是递增地计算每个段执行的步数。 
       这有助于消除浮点舍入问题的几个补充。
       但是，由于浮点数只有7.2个有效数字，具有极高步计数的长移动可能会超过浮点数的精度，从而导致步丢失。 
       幸运的是，在Grbl支持的CNC机床中，这种情况极不可能也不现实（即以200步/毫米的速度超过10米的轴行程）。
    */
    float step_dist_remaining = prep.step_per_mm*mm_remaining; //将mm_remaining转换为步
    float n_steps_remaining = ceil(step_dist_remaining); // 当前剩余步数向上取整
    float last_n_steps_remaining = ceil(prep.steps_remaining); // 最后剩余步数向上取整
    prep_segment->n_step = last_n_steps_remaining-n_steps_remaining; //计算要执行的步骤数。

    //如果我们处于进给保持的末尾，并且没有步要执行，就退出。
    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        //减速到零速度不到一步，但已经非常接近。
//AMASS需要执行完整的步骤。所以只需退出。
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; //未生成段，但仍保留当前步骤数据。
      }
    }

    //计算分段步长。
    //由于步进是整数，而移动的毫米距离不是整数，因此每个段的末端可以有一个不执行的大小不同的部分步进，
    //因为由于AMASS算法，步进ISR需要整个步进。
    //为了补偿，我们跟踪执行前一段的部分步长的时间，并简单地将其与部分步长距离一起应用于当前段，这样它就可以精确地调整整个段速率，以保持步长输出的精确性。
    //这些速率调整通常非常小，不会对性能产生不利影响，但可确保Grbl输出规划师计算的准确加速度和速度剖面。
    dt += prep.dt_remainder; //应用上一段部分步骤执行时间
    float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); //计算调整步进速率逆

    //计算预处理段每一步的CPU周期。
    uint32_t cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); //（周期/步）

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      //计算步长定时和多轴平滑级别。
      //注意：AMASS在每个级别都会使计时器超速，因此只需要一个预分频器。
      if (cycles < AMASS_LEVEL1) { prep_segment->AMASS_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->AMASS_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->AMASS_level = 2; }
        else { prep_segment->AMASS_level = 3; }
        cycles >>= prep_segment->AMASS_level;
        prep_segment->n_step <<= prep_segment->AMASS_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } //只需设置尽可能低的速度。
    #else
      //计算正常步长生成的步长定时和定时器预分频。
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // prescaler: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; //预分频器：8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else {
        prep_segment->prescaler = 3; //预分频器：64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { //只需设置尽可能低的速度。（大约4步/秒。）
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    //段完成！增加段缓冲区索引，以便步进ISR可以立即执行它。
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    //更新相应的规划器和部门数据。
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;

    //检查退出条件并标记以加载下一个规划器块。
    if (mm_remaining == prep.mm_complete) {
      //计划块结束或强制终止。没有更多的距离要执行。
      if (mm_remaining > 0.0) { // 在强制终止结束时。
        // 重置恢复准备参数，然后退出。
        //允许步进式ISR完成段队列，其中实时协议将在从ISR接收循环停止标志时设置新状态。在此之前，准备段将被阻止。
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // 退出!
      } else { // 计划块的末尾
        // 规划器块已完成。所有步都设置为在段缓冲区中执行。
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; //设置指针以指示检查并加载下一个规划器块。
        plan_discard_current_block();
      }
    }

  }
}


//由实时状态报告调用，以获取当前正在执行的速度。
//但是，该值并不完全是当前速度，而是在段缓冲区中最后一步段中计算的速度。
//它将始终落后于段块数（-1）除以每秒的加速度（以秒为单位）。
float st_get_realtime_rate()
{
  if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)){
    return prep.current_speed;
  }
  return 0.0f;
}
