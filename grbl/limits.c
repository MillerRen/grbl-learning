/*
  limits.c - 与限位开关和执行复位循环有关的代码
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


//归位轴搜索距离乘数。由该值乘以循环行程计算得出。
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 //必须大于1，以确保限位开关接合。
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 //必须大于1，以确保限位开关已清除。
#endif

#ifdef ENABLE_DUAL_AXIS
  //双轴异步限制触发器检查的标志。
  #define DUAL_AXIS_CHECK_DISABLE     0  //必须为零
  #define DUAL_AXIS_CHECK_ENABLE      bit(0)
  #define DUAL_AXIS_CHECK_TRIGGER_1   bit(1)
  #define DUAL_AXIS_CHECK_TRIGGER_2   bit(2)
#endif

void limits_init()
{
  LIMIT_DDR &= ~(LIMIT_MASK); //设置为输入引脚

  #ifdef DISABLE_LIMIT_PIN_PULL_UP
    LIMIT_PORT &= ~(LIMIT_MASK); //正常低电压运行。需要外部下拉。
  #else
    LIMIT_PORT |= (LIMIT_MASK);  //启用内部上拉电阻器。正常高位运行。
  #endif

  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    LIMIT_PCMSK |= LIMIT_MASK; //启用管脚更改中断的特定管脚
    PCICR |= (1 << LIMIT_INT); //启用引脚更改中断
  } else {
    limits_disable();
  }

  #ifdef ENABLE_SOFTWARE_DEBOUNCE
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDP0); //将超时设置为约32毫秒。
  #endif
}


//禁用硬限位。
void limits_disable()
{
  LIMIT_PCMSK &= ~LIMIT_MASK;  //禁用管脚更改中断的特定管脚
  PCICR &= ~(1 << LIMIT_INT);  //禁用引脚更改中断
}


//以逐位uint8变量的形式返回限位状态。每个位表示轴限位，其中触发为1，未触发为0。
//应用反转掩码。轴由其在位位置的编号定义，即Z_AXIS为（1<<2）或位2，Y_AXIS为（1<<1）或位1。
uint8_t limits_get_state()
{
  uint8_t limit_state = 0;
  uint8_t pin = (LIMIT_PIN & LIMIT_MASK);
  #ifdef INVERT_LIMIT_PIN_MASK
    pin ^= INVERT_LIMIT_PIN_MASK;
  #endif
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= LIMIT_MASK; }
  if (pin) {
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
      if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
    }
    #ifdef ENABLE_DUAL_AXIS
      if (pin & (1<<DUAL_LIMIT_BIT)) { limit_state |= (1 << N_AXIS); }
    #endif
  }
  return(limit_state);
}


//这是限制引脚更改中断，用于处理硬限位功能。
//限位开关抖动会导致很多问题，比如错误读数和多次中断调用。
//如果开关被触发，则表明发生了不好的情况，不管限位开关是否处于断开状态，都应如此处理。

//由于Arduino微控制器在检测管脚变化时不保留任何状态信息，因此不可能可靠地判断管脚抖动的状态。
//如果我们轮询ISR中的管脚，如果开关抖动，您可能会错过正确的读数。
// 注意：不要将急停装置连接到限位引脚上，因为该中断在复位循环期间被禁用，并且不会正确响应。
//根据用户要求或需要，可能会有一个特殊的急停引脚，但通常建议直接将急停开关连接到Arduino复位引脚，因为这是最正确的方法。
#ifndef ENABLE_SOFTWARE_DEBOUNCE
  ISR(LIMIT_INT_vect) //默认值：限制引脚更改中断处理。
  {
    //如果已经处于报警状态或正在执行报警，则忽略限位开关。
    //当处于报警状态时，Grbl应已重置或将强制重置，因此规划器和串行缓冲区中的任何等待运动都将被清除，新发送的块将被锁定，直到重新定位循环或终止锁定命令。
    //允许用户禁用硬限位设置，如果重置后不断触发其限位并移动其轴。
    if (sys.state != STATE_ALARM) {
      if (!(sys_rt_exec_alarm)) {
        #ifdef HARD_LIMIT_FORCE_STATE_CHECK
          // Check limit pin state.
          if (limits_get_state()) {
            mc_reset(); //启动系统终止。
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); //指示硬限位危险事件
          }
        #else
          mc_reset(); //启动系统终止。
          system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); //指示硬限位危险事件
        #endif
      }
    }
  }
#else //可选：软件去盎司限制引脚例行程序。
//在限制引脚改变时，启用看门狗定时器以创建短延迟。
  ISR(LIMIT_INT_vect) { if (!(WDTCSR & (1<<WDIE))) { WDTCSR |= (1<<WDIE); } }
  ISR(WDT_vect) //看门狗定时器
  {
    WDTCSR &= ~(1<<WDIE); //禁用看门狗定时器。
    if (sys.state != STATE_ALARM) {  //如果已处于报警状态，则忽略。
      if (!(sys_rt_exec_alarm)) {
        //检查限位引脚状态。
        if (limits_get_state()) {
          mc_reset(); //启动系统终止。
          system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // 指示硬限位危险事件
        }
      }  
    }
  }
#endif

//返回指定的循环轴，设置机器位置，并在完成后执行回拉运动。
//归位是一种特殊的运动情况，涉及快速不受控停止，以定位限位开关的触发点。
//快速停止由系统级轴锁定掩码处理，该掩码防止步进算法执行步进脉冲。
// 归位运动通常会绕过正常操作中执行运动的过程。
// 注意：只有abort 实时命令才能中断此过程。
// TODO:将特定限位引脚的调用移动到通用函数以实现可移植性。
void limits_go_home(uint8_t cycle_mask)
{
  if (sys.abort) { return; } //如果已发出系统重置，则阻塞。

  //初始化归位运动的平面数据结构。主轴和冷却液已禁用。
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data,0,sizeof(plan_line_data_t));
  pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;
  #endif

  //初始化用于归位计算的变量。
  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
  uint8_t step_pin[N_AXIS];
  #ifdef ENABLE_DUAL_AXIS
    uint8_t step_pin_dual;
    uint8_t dual_axis_async_check;
    int32_t dual_trigger_position;
    #if (DUAL_AXIS_SELECT == X_AXIS)
      float fail_distance = (-DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT/100.0)*settings.max_travel[Y_AXIS];
    #else
      float fail_distance = (-DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT/100.0)*settings.max_travel[X_AXIS];
    #endif
    fail_distance = min(fail_distance, DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX);
    fail_distance = max(fail_distance, DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN);
    int32_t dual_fail_distance = trunc(fail_distance*settings.steps_per_mm[DUAL_AXIS_SELECT]);
    // int32_t dual_fail_distance = trunc((DUAL_AXIS_HOMING_TRIGGER_FAIL_DISTANCE)*settings.steps_per_mm[DUAL_AXIS_SELECT]);
  #endif
  float target[N_AXIS];
  float max_travel = 0.0;
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    // Initialize step pin masks
    step_pin[idx] = get_step_pin_mask(idx);
    #ifdef COREXY
      if ((idx==A_MOTOR)||(idx==B_MOTOR)) { step_pin[idx] = (get_step_pin_mask(X_AXIS)|get_step_pin_mask(Y_AXIS)); }
    #endif

    if (bit_istrue(cycle_mask,bit(idx))) {
      //根据最大行程设置设置目标。确保寻的开关与搜索标量接合。
      //注意：settings.max_travel[]存储为负值。
      max_travel = max(max_travel,(-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);
    }
  }
  #ifdef ENABLE_DUAL_AXIS
    step_pin_dual = (1<<DUAL_STEP_BIT);
  #endif

  //将搜索模式设置为接近搜索速率，以快速接合指定的循环掩码限位开关。
  bool approach = true;
  float homing_rate = settings.homing_seek_rate;

  uint8_t limit_state, axislock, n_active_axis;
  do {

    system_convert_array_steps_to_mpos(target,sys_position);

    //初始化并声明归位例程所需的变量。
    axislock = 0;
    #ifdef ENABLE_DUAL_AXIS
      sys.homing_axis_lock_dual = 0;
      dual_trigger_position = 0;
      dual_axis_async_check = DUAL_AXIS_CHECK_DISABLE;
    #endif
    n_active_axis = 0;
    for (idx=0; idx<N_AXIS; idx++) {
      //设置活动轴的目标位置，并设置归位率的计算。
      if (bit_istrue(cycle_mask,bit(idx))) {
        n_active_axis++;
        #ifdef COREXY
          if (idx == X_AXIS) {
            int32_t axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
            sys_position[A_MOTOR] = axis_position;
            sys_position[B_MOTOR] = -axis_position;
          } else if (idx == Y_AXIS) {
            int32_t axis_position = system_convert_corexy_to_x_axis_steps(sys_position);
            sys_position[A_MOTOR] = sys_position[B_MOTOR] = axis_position;
          } else {
            sys_position[Z_AXIS] = 0;
          }
        #else
          sys_position[idx] = 0;
        #endif
        //根据周期掩码和寻的周期进近状态设置目标方向。
        //注意：这种编译比任何其他尝试过的实现都要小。
        if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
          if (approach) { target[idx] = -max_travel; }
          else { target[idx] = max_travel; }
        } else {
          if (approach) { target[idx] = max_travel; }
          else { target[idx] = -max_travel; }
        }
        //将轴锁定应用于此循环中激活的步进端口引脚。
        axislock |= step_pin[idx];
        #ifdef ENABLE_DUAL_AXIS
          if (idx == DUAL_AXIS_SELECT) { sys.homing_axis_lock_dual = step_pin_dual; }
        #endif
      }

    }
    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] 调整以使各个轴都以归位速率移动。
    sys.homing_axis_lock = axislock;

    //执行归位循环。规划器缓冲区应为空，以启动归位循环。
    pl_data->feed_rate = homing_rate; //设置当前归位速率。
    plan_buffer_line(target, pl_data); // 传递给 mc_line(). 直接规划归位运动。

    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION; //设置为执行归位运动并清除现有标志。
    st_prep_buffer(); //准备并填充新计划区块的段缓冲区。
    st_wake_up(); // 启动运动
    do {
      if (approach) {
        //检查限位状态。当循环轴发生变化时锁定它们。
        limit_state = limits_get_state();
        for (idx=0; idx<N_AXIS; idx++) {
          if (axislock & step_pin[idx]) {
            if (limit_state & (1 << idx)) {
              #ifdef COREXY
                if (idx==Z_AXIS) { axislock &= ~(step_pin[Z_AXIS]); }
                else { axislock &= ~(step_pin[A_MOTOR]|step_pin[B_MOTOR]); }
              #else
                axislock &= ~(step_pin[idx]);
                #ifdef ENABLE_DUAL_AXIS
                  if (idx == DUAL_AXIS_SELECT) { dual_axis_async_check |= DUAL_AXIS_CHECK_TRIGGER_1; }
                #endif
              #endif
            }
          }
        }
        sys.homing_axis_lock = axislock;
        #ifdef ENABLE_DUAL_AXIS
          if (sys.homing_axis_lock_dual) { //注：仅在归位双轴时为真。
            if (limit_state & (1 << N_AXIS)) { 
              sys.homing_axis_lock_dual = 0;
              dual_axis_async_check |= DUAL_AXIS_CHECK_TRIGGER_2;
            }
          }
          
          //当第一个双轴限位触发时，记录位置并开始检查距离，直到其他限位触发。失败后退出。
          if (dual_axis_async_check) {
            if (dual_axis_async_check & DUAL_AXIS_CHECK_ENABLE) {
              if (( dual_axis_async_check &  (DUAL_AXIS_CHECK_TRIGGER_1 | DUAL_AXIS_CHECK_TRIGGER_2)) == (DUAL_AXIS_CHECK_TRIGGER_1 | DUAL_AXIS_CHECK_TRIGGER_2)) {
                dual_axis_async_check = DUAL_AXIS_CHECK_DISABLE;
              } else {
                if (abs(dual_trigger_position - sys_position[DUAL_AXIS_SELECT]) > dual_fail_distance) {
                  system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DUAL_APPROACH);
                  mc_reset();
                  protocol_execute_realtime();
                  return;
                }
              }
            } else {
              dual_axis_async_check |= DUAL_AXIS_CHECK_ENABLE;
              dual_trigger_position = sys_position[DUAL_AXIS_SELECT];
            }
          }
        #endif
      }

      st_prep_buffer(); //检查并准备段缓冲区。注：所需时间不应超过200 US。

      //退出例程：没有时间在此循环中运行protocol_execute_realtime（）。
      if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
        uint8_t rt_exec = sys_rt_exec_state;
        //归位故障条件：在循环期间发出重置。
        if (rt_exec & EXEC_RESET) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
        //归位故障条件：安全门打开。
        if (rt_exec & EXEC_SAFETY_DOOR) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR); }
        //归位故障条件：回拉动作后限位开关仍处于接合状态
        if (!approach && (limits_get_state() & cycle_mask)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_PULLOFF); }
        //归位故障条件：进近过程中未找到限位开关。
        if (approach && (rt_exec & EXEC_CYCLE_STOP)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_APPROACH); }
        if (sys_rt_exec_alarm) {
          mc_reset(); //如果电机正在运行，则停止电机。
          protocol_execute_realtime();
          return;
        } else {
          // 完成回拉动作。 禁止CYCLE_STOP执行。
          system_clear_exec_state_flag(EXEC_CYCLE_STOP);
          break;
        }
      }

    #ifdef ENABLE_DUAL_AXIS
      } while ((STEP_MASK & axislock) || (sys.homing_axis_lock_dual));
    #else
      } while (STEP_MASK & axislock);
    #endif

    st_reset(); //立即强制杀死步进器并重置步进段缓冲区。
    delay_ms(settings.homing_debounce_delay); // 延迟，以便运动衰减。

    //为定位循环反转方向和重置归位速率。
    approach = !approach;

    //第一个周期后，寻的进入定位阶段。缩短搜索回拉距离。
    if (approach) {
      max_travel = settings.homing_pulloff*HOMING_AXIS_LOCATE_SCALAR;
      homing_rate = settings.homing_feed_rate;
    } else {
      max_travel = settings.homing_pulloff;
      homing_rate = settings.homing_seek_rate;
    }

  } while (n_cycle-- > 0);

  // 活动的归位周期轴现在应该已经归位并且机器限位已经定位。
  // 默认情况下，Grbl定义机器位置都是负值，就像大部分CNC所作的那样。
  // 由于限位开关可以位于轴的任意一侧，请检查并适当地设置轴机零位。  
  // 同时，在已归位的轴限开关上进行拉拔操作。  
  // 这提供了一些初始的关闭开关的间隙，也应该有助于防止它们在启用硬限制或多个轴共享一个限制引脚时错误触发。  
  int32_t set_axis_position;
  // 为限位开关设置机器位置。不更新没有设置归位的轴。
  for (idx=0; idx<N_AXIS; idx++) {
    //注意：settings.max_travel[]存储为负值。
    if (cycle_mask & bit(idx)) {
      #ifdef HOMING_FORCE_SET_ORIGIN
        set_axis_position = 0;
      #else
        if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) {
          set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff)*settings.steps_per_mm[idx]);
        } else {
          set_axis_position = lround(-settings.homing_pulloff*settings.steps_per_mm[idx]);
        }
      #endif

      #ifdef COREXY
        if (idx==X_AXIS) {
          int32_t off_axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
          sys_position[A_MOTOR] = set_axis_position + off_axis_position;
          sys_position[B_MOTOR] = set_axis_position - off_axis_position;
        } else if (idx==Y_AXIS) {
          int32_t off_axis_position = system_convert_corexy_to_x_axis_steps(sys_position);
          sys_position[A_MOTOR] = off_axis_position + set_axis_position;
          sys_position[B_MOTOR] = off_axis_position - set_axis_position;
        } else {
          sys_position[idx] = set_axis_position;
        }
      #else
        sys_position[idx] = set_axis_position;
      #endif

    }
  }
  sys.step_control = STEP_CONTROL_NORMAL_OP; //将步进控制返回到正常操作。
}


// 执行一个软限位检查，只在mc_line()中调用。假如机器已经归位,工作区位置都是负的(右手定则)，系统处于正常操作状态。
// 注意：用于手动控制时，将行程限制在软限位的体积内。
void limits_soft_check(float *target)
{
  if (system_check_travel_limits(target)) {
    sys.soft_limit = true;
    //如果循环激活，则强制进给保持。
    //所有缓冲块都保证位于工作区体积内，因此只需到达受控停止位置，就不会丢失位置。完成后，进入报警模式。
    if (sys.state == STATE_CYCLE) {
      system_set_exec_state_flag(EXEC_FEED_HOLD);
      do {
        protocol_execute_realtime();
        if (sys.abort) { return; }
      } while ( sys.state != STATE_IDLE );
    }
    mc_reset(); //发布系统重置，确保主轴和冷却液关闭。
    system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT); //指示软限位危险事件
    protocol_execute_realtime(); //执行以进入危险事件循环和系统中止
    return;
  }
}
