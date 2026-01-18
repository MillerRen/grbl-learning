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

#include "grbl.h"


static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  //用于运动指令的环形缓冲区
static uint8_t block_buffer_tail;     //要立即处理的块的索引
static uint8_t block_buffer_head;     //要推送的下一个块的索引
static uint8_t next_buffer_head;      //下一个缓冲头的索引
static uint8_t block_buffer_planned;  //优化后的规划块的索引

//定义规划器变量
typedef struct {
  int32_t position[N_AXIS];          //刀具的规划器位置（绝对步长）。
//对于需要多直线运动的运动，即圆弧、封闭圆和齿隙补偿，与g代码位置分开。
  float previous_unit_vec[N_AXIS];   //上一条路径线段的单位向量
  float previous_nominal_speed;  //前一路径线段的标称速度
} planner_t;
static planner_t pl;


//返回环形缓冲区中下一个块的索引。也称为步进段缓冲。
uint8_t plan_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


//返回环形缓冲区中上一个块的索引
static uint8_t plan_prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


/*                            规划器速度定义
                                     +--------+   <- current->nominal_speed
                                    /          \
         current->entry_speed ->   +            \
                                   |             + <- next->entry_speed (aka exit speed)
                                   +-------------+
                                       时间 -->

  根据以下基本准则重新计算运动计划：

    1. 按相反顺序依次检查每个可行块，并计算接合速度（即当前->入口速度），以便：
      a. 没有结点速度超过预先计算的最大结点速度限制或相邻块的标称速度。
      b. 块进入速度不能超过后顾计算下一个块的退出速度（next->entry_speed）且在块移动距离上具有最大允许减速度。
      c. 最后一个（或最新添加的）块是从完全停止（退出速度为零）开始规划的。
    2. 按时间顺序（向前）检查每个块，如果需要，斜向下连接速度值
      a. 出口速度超过根据其入口速度前瞻计算得出速度，且在滑车移动距离上具有最大允许加速度。
  当这些阶段完成时，规划器将在所有规划器区块内最大化速度剖面，其中每个区块以其最大允许加速度极限运行。
  换句话说，对于规划器中的所有区块，该计划都是最优的，不可能进一步提高速度。
  如果一个新的块被添加到缓冲区，则根据新的最优计划的所述准则重新计算该计划。

  为了提高这些指南的计算效率，创建了一组规划器块指针，用于在正常运行时，当规划器指南在逻辑上无法对计划进行任何进一步更改或改进时，指示停止计算点，并将新块流式传输并添加到规划器缓冲区。
  例如，如果规划器中的一个子集连续块已规划，并等于最大连接速度（或第一个规划器块），则添加到规划器缓冲区的新块不会改变其中的速度剖面。
  所以我们不再需要计算它们。
  或者，如果来自规划器中第一个块（或最佳停止计算点）的一组连续块都在加速，则它们都是最优的，并且不能通过添加到规划器缓冲区的新块进行更改，因为这只会进一步将计划速度增加到按时间顺序排列的块，直到达到最大连接速度。
  但是，如果计划的运行条件因不常使用的进给保持或进给速度覆盖而发生变化，停止计算指针将重置，并按照一般指南中的规定重新计算整个计划。
  
  规划器缓冲区索引映射：
  - block_buffer_tail: 指向计划器缓冲区的开头。第一个被执行或正在执行的。
  - block_buffer_head: 指向缓冲区中最后一个块之后的缓冲区块。 用于指示缓冲区是满的还是空的。
      如标准环形缓冲区所述，此块始终为空。
  - next_buffer_head: 指向缓冲头块后的下一个规划器缓冲块。 当等于缓冲区尾部时，表示缓冲区已满。
  - block_buffer_planned: 指向正常流操作条件下最后一个优化规划块之后的第一个缓冲块。
    用于规划优化，方法是避免重新计算不会随添加新块而更改的规划器缓冲区部分，如上所述。
    此外，此块永远不能小于块缓冲区尾端，并且在一个周期中，当计划丢弃当前块（）例程遇到此要求时，将始终向前推并保持此要求。
  注意：由于规划器仅对规划器缓冲区中的内容进行计算，因此一些具有大量短线段的运动（如G2/3圆弧或复杂曲线）可能移动缓慢。这是因为在整个缓冲区中的总运动距离不足以按照上述准则先加速到标称速度，再在缓冲区末端减速到完全停止。
  如果这种情况发生并成为麻烦，有几种简单的解决方案：
  (1) 最大化机器加速度。
    规划器将会在相同的组合距离内计算更高的速度剖面。
  (2) 最大化线
  每个块的运动距离达到所需公差。 规划器使用的组合距离越大，它可以走得越快。 
  (3) 最大化规划器缓冲区大小。 这也将增加规划器计算的总距离。
  它还增加了计划者计算最优计划所需的计算次数，因此请仔细选择。
  Arduino 328p内存已经达到最大值，但未来的ARM版本应该有足够的内存和速度，以支持多达100个或更多的前瞻块。
*/
static void planner_recalculate()
{
  //将块索引初始化为规划器缓冲区中的最后一个块。
  uint8_t block_index = plan_prev_block_index(block_buffer_head);

  // 退出。 只有一个可规划的块，无法执行任何操作。
  if (block_index == block_buffer_planned) { return; }

  //反向通过：粗略地最大化所有可能的减速曲线，从缓冲区中的最后一个块开始反向规划。
  //当达到最后一个最优计划或尾部指针时停止计划。
  // 注：向前传递将在稍后完善和纠正反向传递，以创建最佳计划。
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  //计算缓冲区中最后一个块的最大进入速度，其中退出速度始终为零。
  current->entry_speed_sqr = min( current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);

  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned) { //缓冲区中只有两个可规划块。反向传递完成。
//检查第一块是否为尾部。如果是，通知步进器更新其当前参数。
    if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  } else { //三个或三个以上可规划区块
    while (block_index != block_buffer_planned) {
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      //检查下一个区块是否为尾部区块（=计划区块）。如果是，更新当前步进器参数。
      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }

      //从当前块的退出速度计算最大进入速度减速。
      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        } else {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }

  //向前传递：向前规划从计划指针开始的加速曲线。
//还扫描最佳计划断点，并适当更新计划指针。
  next = &block_buffer[block_buffer_planned]; //从缓冲区计划指针开始
  block_index = plan_next_block_index(block_buffer_planned);
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];

    //在向前传球中检测到的任何加速度都会自动向前移动最佳计划指针，因为在此之前的一切都是最佳的。
    //换句话说，没有什么可以通过逻辑改善从缓冲区尾部到计划指针的计划。
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
      // 如果为true，则当前块为完全加速，我们可以向前移动计划指针。
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // 总是 <= max_entry_speed_sqr. 向后传递设置此选项。
        block_buffer_planned = block_index; //设置最佳计划指针。
      }
    }

    //以最大进入速度设置的任何块也会在缓冲区中创建一个最佳计划。
//当计划被缓冲区的开始和一个最大进入速度或两个最大进入速度包围时，在逻辑上不能进一步改进中间的每个块。因此，我们不再需要重新计算它们。
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index( block_index );
  }
}


void plan_reset()
{
  memset(&pl, 0, sizeof(planner_t)); //清除计划器结构
  plan_reset_buffer();
}


void plan_reset_buffer()
{
  block_buffer_tail = 0;
  block_buffer_head = 0; // Empty = tail
  next_buffer_head = 1; // plan_next_block_index(block_buffer_head)
  block_buffer_planned = 0; // = block_buffer_tail;
}


void plan_discard_current_block()
{
  if (block_buffer_head != block_buffer_tail) { //丢弃非空缓冲区。
    uint8_t block_index = plan_next_block_index( block_buffer_tail );
    // 推进 block_buffer_planned 指针, 如果遇到.
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index;
  }
}


//返回系统运动使用的计划器缓冲块的地址。由段生成器调用。
plan_block_t *plan_get_system_motion_block()
{
  return(&block_buffer[block_buffer_head]);
}


//返回第一个计划程序块的地址（如果可用）。由各种主程序函数调用。
plan_block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); } //缓冲区空
  return(&block_buffer[block_buffer_tail]);
}


float plan_get_exec_block_exit_speed_sqr()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return( 0.0 ); }
  return( block_buffer[block_index].entry_speed_sqr );
}


//返回块环缓冲区的可用性状态。如果已满，则为True。
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { return(true); }
  return(false);
}


//根据运行条件和覆盖值计算并返回块标称速度。
//注意：所有系统运动命令（如归位/停车）均不受覆盖的影响。
float plan_compute_profile_nominal_speed(plan_block_t *block)
{
  float nominal_speed = block->programmed_rate;
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { nominal_speed *= (0.01*sys.r_override); }
  else {
    if (!(block->condition & PL_COND_FLAG_NO_FEED_OVERRIDE)) { nominal_speed *= (0.01*sys.f_override); }
    if (nominal_speed > block->rapid_rate) { nominal_speed = block->rapid_rate; }
  }
  if (nominal_speed > MINIMUM_FEED_RATE) { return(nominal_speed); }
  return(MINIMUM_FEED_RATE);
}


//根据结点以前和当前的最小标称速度以及最大结点速度，计算并更新块的最大进入速度（sqr）。
static void plan_compute_profile_parameters(plan_block_t *block, float nominal_speed, float prev_nominal_speed)
{
  //根据结点速度和相邻标称速度的最小值计算交叉口最大入口。
  if (nominal_speed > prev_nominal_speed) { block->max_entry_speed_sqr = prev_nominal_speed*prev_nominal_speed; }
  else { block->max_entry_speed_sqr = nominal_speed*nominal_speed; }
  if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) { block->max_entry_speed_sqr = block->max_junction_speed_sqr; }
}


//根据基于运动的覆盖更改重新计算缓冲运动剖面参数。
void plan_update_velocity_profile_parameters()
{
  uint8_t block_index = block_buffer_tail;
  plan_block_t *block;
  float nominal_speed;
  float prev_nominal_speed = SOME_LARGE_VALUE; //为第一块标称速度计算设置一个比较高的值。
  while (block_index != block_buffer_head) {
    block = &block_buffer[block_index];
    nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
    prev_nominal_speed = nominal_speed;
    block_index = plan_next_block_index(block_index);
  }
  pl.previous_nominal_speed = prev_nominal_speed; //更新下一个输入块的上一个标称速度。
}


/* 向缓冲区添加新的线性移动。
  target[N_AXIS]是有符号的绝对目标位置，单位为毫米。
   进给速率指定运动的速度。
   如果进给速度倒置，则进给速度被视为“频率”，并将在1/进给速度分钟内完成操作。
   传递给规划器的所有位置数据必须是机器位置，以使规划器独立于任何坐标系更改和偏移，这些更改和偏移由g代码解析器处理。
   注意：假设缓冲区可用。
   缓冲区检查由运动控制在更高级别上处理。
   换句话说，缓冲头永远不等于缓冲尾。  
   此外，进给速度输入值有三种使用方式：如果反向进给速度为假，则作为正常进给速度；如果反向进给速度为真，则作为反向时间；如果进给速度值为负（且反向进给速度始终为假），则作为寻找/急流速度。
   系统运动条件告知计划员在始终未使用的块缓冲头中计划运动。
   它避免更改规划器状态并保留缓冲区，以确保后续gcode运动仍能正确规划，而步进器模块仅指向块缓冲头以执行特殊的系统运动。
   */
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data)
{
  //准备并初始化新块。复制块执行的相关pl_data。
  plan_block_t *block = &block_buffer[block_buffer_head];
  memset(block,0,sizeof(plan_block_t)); //将所有块值归零。
  block->condition = pl_data->condition;
  #ifdef VARIABLE_SPINDLE
    block->spindle_speed = pl_data->spindle_speed;
  #endif
  #ifdef USE_LINE_NUMBERS
    block->line_number = pl_data->line_number;
  #endif

  //计算并存储初始移动距离数据。
  int32_t target_steps[N_AXIS], position_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;

  //根据计划的运动类型复制位置数据。
  if (block->condition & PL_COND_FLAG_SYSTEM_MOTION) { 
    #ifdef COREXY
      position_steps[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
      position_steps[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
      position_steps[Z_AXIS] = sys_position[Z_AXIS];
    #else
      memcpy(position_steps, sys_position, sizeof(sys_position)); 
    #endif
  } else { memcpy(position_steps, pl.position, sizeof(pl.position)); }

  #ifdef COREXY
    target_steps[A_MOTOR] = lround(target[A_MOTOR]*settings.steps_per_mm[A_MOTOR]);
    target_steps[B_MOTOR] = lround(target[B_MOTOR]*settings.steps_per_mm[B_MOTOR]);
    block->steps[A_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) + (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
    block->steps[B_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) - (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
  #endif

  for (idx=0; idx<N_AXIS; idx++) {
    //以绝对步长计算目标位置、每个轴的步长数，并确定最大步长事件。
//此外，计算单个轴的距离以进行移动和准备单位向量计算。
//注意：根据转换的步长值计算真实距离。
    #ifdef COREXY
      if ( !(idx == A_MOTOR) && !(idx == B_MOTOR) ) {
        target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
        block->steps[idx] = labs(target_steps[idx]-position_steps[idx]);
      }
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      if (idx == A_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] + target_steps[Y_AXIS]-position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else if (idx == B_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] - target_steps[Y_AXIS]+position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else {
        delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
      }
    #else
      target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
      block->steps[idx] = labs(target_steps[idx]-position_steps[idx]);
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
	  #endif
    unit_vec[idx] = delta_mm; //存储单位矢量分子

    //设置方向位。位启用始终意味着方向为负。
    if (delta_mm < 0.0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
  }

  //如果这是一个长度为零的区块，则退出。极不可能发生。
  if (block->step_event_count == 0) { return(PLAN_EMPTY_BLOCK); }

  //计算直线移动的单位矢量和按比例缩小的块最大进给速率和加速度，以确保在直线方向上不超过单个轴的最大值。
//注：此计算假设所有轴都是正交的（笛卡尔坐标），如果它们也是正交/独立的，则与ABC轴一起工作。对单位向量的绝对值进行运算。
  block->millimeters = convert_delta_vector_to_unit_vector(unit_vec);
  block->acceleration = limit_value_by_axis_maximum(settings.acceleration, unit_vec);
  block->rapid_rate = limit_value_by_axis_maximum(settings.max_rate, unit_vec);

  //存储编程速率。
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { block->programmed_rate = block->rapid_rate; }
  else { 
    block->programmed_rate = pl_data->feed_rate;
    if (block->condition & PL_COND_FLAG_INVERSE_TIME) { block->programmed_rate *= block->millimeters; }
  }

  //TODO：从静止开始时，需要检查此处理零结速度的方法。
  if ((block_buffer_head == block_buffer_tail) || (block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {

    //将块进入速度初始化为零。
    //假设它将从静止的开始。
    //计划员将在稍后更正此问题。
    //如果系统运动，则系统运动块始终假定从静止开始，并在完全停止时结束。
    block->entry_speed_sqr = 0.0;
    block->max_junction_speed_sqr = 0.0; //从静止开始。强制从零速度开始。

  } else {
    //通过向心加速度近似计算结点处的最大允许进入速度。
//让一个圆与以前的路径线段和当前路径线段相切，其中连接点偏差定义为从连接点到圆最近边的距离，与圆心共线。
    //连接两条路径的圆段表示向心加速度路径。
    //基于圆半径的最大加速度求解最大速度，该半径由结点偏差间接定义。
    //在先前的Grbl版本中，这也可被视为路径宽度或最大加加速度。
    // 这种方法实际上并不偏离路径，而是作为一种稳健的方法来计算转弯速度，因为它同时考虑了连接角和连接速度的非线性。
    // 注：如果连接偏差值是有限的，Grbl以精确路径模式（G61）执行运动。
    //如果交叉点偏差值为零，Grbl将以精确停止模式（G61.1）的方式执行运动。
    //将来，如果需要连续模式（G64），这里的数学是完全相同的。 
    //机器不会一直移动到连接点，而是沿着此处定义的圆弧。
    //Arduino没有CPU周期来执行连续模式路径，但基于ARM的微控制器肯定有。
    //
    // 注：最大连接速度是一个固定值，因为机器加速度限制在运行期间不能动态改变，线路也不能移动几何体。
    //当进给速度超控改变块的标称速度时，必须将其保存在内存中，这可能会改变所有块的整体最大进入速度条件。
    float junction_unit_vec[N_AXIS];
    float junction_cos_theta = 0.0;
    for (idx=0; idx<N_AXIS; idx++) {
      junction_cos_theta -= pl.previous_unit_vec[idx]*unit_vec[idx];
      junction_unit_vec[idx] = unit_vec[idx]-pl.previous_unit_vec[idx];
    }

    //注：通过cos（θ）的三角半角恒等式计算，无需任何昂贵的三角，sin（）或acos（）。
    if (junction_cos_theta > 0.999999) {
      //对于0度急转弯，只需设置为最小转弯速度。
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      if (junction_cos_theta < -0.999999) {
        //结点是一条直线或180度。结点速度是无限的。
        block->max_junction_speed_sqr = SOME_LARGE_VALUE;
      } else {
        convert_delta_vector_to_unit_vector(junction_unit_vec);
        float junction_acceleration = limit_value_by_axis_maximum(settings.acceleration, junction_unit_vec);
        float sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta)); //三角半角恒等式。总是正的。
        block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                       (junction_acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2) );
      }
    }
  }

  //阻止系统运动更新此数据，以确保正确计算下一个g代码运动。
  if (!(block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {
    float nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);
    pl.previous_nominal_speed = nominal_speed;
    
    //更新以前的路径单位向量和规划器位置。
    memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
    memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

    //新的街区已经准备好了。更新缓冲头和下一个缓冲头索引。
    block_buffer_head = next_buffer_head;
    next_buffer_head = plan_next_block_index(block_buffer_head);

    //最后，使用新块重新计算平面。
    planner_recalculate();
  }
  return(PLAN_OK);
}


//重置规划器位置向量。由系统中止/初始化例程调用。
void plan_sync_position()
{
  //TODO：对于与机器位置不在同一坐标系中的电机配置，需要更新此功能以适应差异。
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
      if (idx==X_AXIS) {
        pl.position[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
      } else if (idx==Y_AXIS) {
        pl.position[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
      } else {
        pl.position[idx] = sys_position[idx];
      }
    #else
      pl.position[idx] = sys_position[idx];
    #endif
  }
}


//返回规划器缓冲区中的可用块数。
uint8_t plan_get_block_buffer_available()
{
  if (block_buffer_head >= block_buffer_tail) { return((BLOCK_BUFFER_SIZE-1)-(block_buffer_head-block_buffer_tail)); }
  return((block_buffer_tail-block_buffer_head-1));
}


//返回规划器缓冲区中的活动块数。
//注意：已弃用。除非在配置中启用了经典状态报告，否则不使用
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head));
}


//使用部分完成的块重新初始化缓冲区计划，假定存在于缓冲区尾部。
//在步进电机完全停止进给保持并停止循环后调用。
void plan_cycle_reinitialize()
{
  // 从一个完成的停止点重新计划。 重置计划器输入速度和缓冲区计划指针。
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();
}
