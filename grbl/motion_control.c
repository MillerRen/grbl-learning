/*
  motion_control.c - 用于发出运动命令的高级接口
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


//在绝对毫米坐标系下执行线性运动。
//除非反向进给速度为真，否则进给速度以毫米/秒为单位。
//那么进给率意味着运动应在（1分钟）/进给率时间内完成。
//注：这是grbl 规划器的主要出口。
//所有直线运动（包括圆弧线段）在传递给规划器之前必须通过此例程。
//mc_line和plan_buffer_line的分离主要是为了将非规划器类型的功能从规划器中分离出来，并使齿隙补偿或封闭圆集成简单直接。
void mc_line(float *target, plan_line_data_t *pl_data)
{
  //如果启用，请检查是否存在软限位冲突。在这里，所有从Grbl中的任何地方拾取的直线运动都到达这里。
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
    // 注意: 阻止点动状态。 点动是一种特殊情况，软限位是独立处理的。
    if (sys.state != STATE_JOG) { limits_soft_check(target); }
  }

  //如果处于“检查gcode”模式，请阻止规划器运动。软限位仍然有效。
  if (sys.state == STATE_CHECK_MODE) { return; }

  //注意：齿隙补偿可安装在此处。
  //它需要方向信息来跟踪何时在预期直线运动之前插入齿隙直线运动，并需要自己的计划检查缓冲区已满（）和检查系统中止循环。
  //此外，对于位置报告，还需要跟踪反冲步骤，这需要保持在系统级别。可能还有其他一些事情需要跟踪。
  //然而，我们认为齿隙补偿不应由Grbl本身处理，因为有无数种方法可以实现它，并且对于不同的CNC机器可能有效或无效。
  //这将更好地由接口作为后处理器任务处理，其中原始g代码被翻译并插入最适合机器的齿隙运动。
  //注：可能作为一个中间地带，需要发送的只是一个标志或特殊命令，向Grbl指示什么是齿隙补偿运动，以便Grbl执行移动，但不更新机器位置值。
  //由于g代码解析器和规划器使用的位置值与系统机器位置是分开的，因此这是可行的。
  //如果缓冲区已满：好！这意味着我们远远领先于机器。
  //保持此循环，直到缓冲区中有空间为止。
  do {
    protocol_execute_realtime(); //检查是否有任何运行时命令
    if (sys.abort) { return; } //退出，如果系统中止。
    if ( plan_check_full_buffer() ) { protocol_auto_cycle_start(); } //缓冲区满时自动循环开始。
    else { break; }
  } while (1);

  //计划并将运动排入规划器缓冲区
  if (plan_buffer_line(target, pl_data) == PLAN_EMPTY_BLOCK) {
    if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
      //如果传递了一致的位置，则正确设置主轴状态。仅在M3激光模式下强制进行缓冲区同步。
      if (pl_data->condition & PL_COND_FLAG_SPINDLE_CW) {
        spindle_sync(PL_COND_FLAG_SPINDLE_CW, pl_data->spindle_speed);
      }
    }
  }
}


//以偏移模式格式执行圆弧。
//位置==当前xyz，目标==目标xyz，偏移==与当前xyz的偏移，轴X定义刀具空间中的圆平面，轴线性是螺旋移动的方向，半径==圆半径，是顺时针布尔值。
//用于矢量变换方向。
// 通过生成大量微小的直线段来近似圆弧。
//在“设置”中配置每个线段的弦公差。圆弧公差，定义为当端点均位于圆上时，从线段到圆的最大法向距离。
void mc_arc(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius,
  uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc)
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  //从中心到当前位置的半径向量
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;

  //从圆心到位置和目标之间的逆时针角度。只需要一次atan2（）触发器计算。
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (is_clockwise_arc) { // 纠正artan2输出方向
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel += 2*M_PI; }
  }

  //注：段端点位于弧上，这可能导致弧直径小于（2x）设置。弧长公差。
//对于99%的用户来说，这很好。
//如果需要不同的圆弧段拟合，即最小二乘法，圆弧中点，只需更改mm_per_arc_segment计算。
//对于Grbl的预期用途，在最严格的情况下，该值不应超过2000。
  uint16_t segments = floor(fabs(0.5*angular_travel*radius)/
                          sqrt(settings.arc_tolerance*(2*radius - settings.arc_tolerance)) );

  if (segments) {
    //乘以反向进给率，以补偿该运动由多个离散段近似实际的。
    //对于所有段的总和，反向进给率应纠正。
    if (pl_data->condition & PL_COND_FLAG_INVERSE_TIME) { 
      pl_data->feed_rate *= segments; 
      bit_false(pl_data->condition,PL_COND_FLAG_INVERSE_TIME); //在弧段上的进给绝对模式下的力。
    }
    
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear])/segments;

    /*通过变换矩阵进行矢量旋转：r为原始矢量，r_T为旋转矢量，phi为旋转角度。
       Jens Geisler的解决方案方法。
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       对于圆弧生成，圆心是旋转轴，半径向量是从圆心到初始位置定义的。
       每个线段由连续的矢量旋转形成。在极少数情况下，单个精度值的累积误差可能大于刀具精度。
       因此，实现了精确的弧径校正。
       这种方法避免了太多非常昂贵的trig操作[sin（），cos（），tan（）]的问题，每个操作可能需要100-200 微秒来计算。

      小角度近似可用于进一步减少计算开销。
       三阶近似（二阶sin（）误差太大）适用于大多数（如果不是的话）CNC应用程序。
       请注意，当theta_per_segment大于~0.25 rad（14 deg）时，该近似值将开始累积数值漂移误差，并且连续使用该近似值而不进行几十次校正。
       这种情况极不可能发生，因为线段长度和theta_per_segment是由圆弧公差设置自动生成和缩放的。
       只有非常大的圆弧公差设置（对于CNC应用来说是不现实的）才会导致该数值漂移误差。
       但是，最好将xxxxx从~4的低位设置为~20左右的高位，以避免触发操作，同时保持电生成的准确性。
       此近似值还允许mc_arc立即将线段插入规划器，而无需计算cos（）或sin（）的初始开销。
       当电弧需要应用校正时，规划器应该已经赶上由初始mc_arc开销引起的滞后。
       当存在连续的圆弧运动时，这一点很重要。
    */
    // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
    float cos_T = 2.0 - theta_per_segment*theta_per_segment;
    float sin_T = theta_per_segment*0.16666667*(cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;

    for (i = 1; i<segments; i++) { //增量（段-1）。

      if (count < N_ARC_CORRECTION) {
        //应用向量旋转矩阵约~40 微秒
        r_axisi = r_axis0*sin_T + r_axis1*cos_T;
        r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {
        //对半径向量进行圆弧校正。仅每N_ARC_CORRECTION增量计算一次。
        
        //约~375 微秒，通过应用初始半径向量（=-offset）的变换矩阵来计算精确位置。
        cos_Ti = cos(i*theta_per_segment);
        sin_Ti = sin(i*theta_per_segment);
        r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
        r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
        count = 0;
      }

      //更新arc_target位置
      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;

      mc_line(position, pl_data);

      //在系统中止时，保持中间循环。运行时命令检查已由mc_line执行。
      if (sys.abort) { return; }
    }
  }
  //确保最后一段到达目标位置。
  mc_line(target, pl_data);
}


//以秒为单位执行驻留。
void mc_dwell(float seconds)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize();
  delay_sec(seconds, DELAY_MODE_DWELL);
}


//执行归位循环以定位和设置机器零位。只有“$H”执行此命令。
//注意：缓冲器中不应有任何运动，Grbl在执行回零循环前必须处于空闲状态。
//这可以防止重新引导后出现错误的缓冲计划。
void mc_homing_cycle(uint8_t cycle_mask)
{
  // 如果已启用硬限制，则检查并中止归位循环。
  //有助于防止将行程两端的限位线连接到一个限位引脚的机器出现问题。
  // TODO: 将特定限位引脚作为limit.c函数调用
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES
    if (limits_get_state()) {
      mc_reset(); //发布系统重置，确保主轴和冷却液关闭。
      system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
      return;
    }
  #endif

  limits_disable(); //禁用循环时间内的硬限位引脚更改寄存器

  //-------------------------------------------------------------------------------------
//执行归位程序。注：特殊运动案例。只有系统重置工作。
  
  #ifdef HOMING_SINGLE_AXIS_COMMANDS
    if (cycle_mask) { limits_go_home(cycle_mask); } //根据掩码执行归位循环。
    else
  #endif
  {
    //搜索以更快的寻的速度接合所有轴限位开关。
    limits_go_home(HOMING_CYCLE_0);  //归位周期0
    #ifdef HOMING_CYCLE_1
      limits_go_home(HOMING_CYCLE_1);  //归位周期1
    #endif
    #ifdef HOMING_CYCLE_2
      limits_go_home(HOMING_CYCLE_2);  //归位周期2
    #endif
  }

  protocol_execute_realtime(); //检查是否复位并设置系统中止。
  if (sys.abort) { return; } // 没有完成。 由 mc_alarm设置报警状态。

  //归航周期完成！为正常运行设置系统。
//-------------------------------------------------------------------------------------

  //将gcode解析器和计划器位置同步到归位位置。
  gc_sync_position();
  plan_sync_position();

  //如果硬限制功能启用，则在复位循环后重新启用硬限制引脚更改寄存器。
  limits_init();
}


//执行刀具长度探测循环。需要探针开关。
//注：探头故障时，程序将停止并进入报警状态。
uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags)
{
  //TODO:需要更新此循环，使其服从非自动循环启动。
  if (sys.state == STATE_CHECK_MODE) { return(GC_PROBE_CHECK_MODE); }

  //在开始探测循环之前，完成所有排队命令并清空planner缓冲区。
  protocol_buffer_synchronize();
  if (sys.abort) { return(GC_PROBE_ABORT); } //如果已发出系统重置，则返回。

  //初始化探测控制变量
  uint8_t is_probe_away = bit_istrue(parser_flags,GC_PARSER_PROBE_IS_AWAY);
  uint8_t is_no_error = bit_istrue(parser_flags,GC_PARSER_PROBE_IS_NO_ERROR);
  sys.probe_succeeded = false; //在开始循环之前重新初始化探测历史记录。
  probe_configure_invert_mask(is_probe_away);

  //同步后，检查探针是否已触发。如果是，停止并发出警报。
//注意：此探测初始化错误适用于所有探测周期。
  if ( probe_get_state() ) { //检查探针引脚状态。
    system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_INITIAL);
    protocol_execute_realtime();
    probe_configure_invert_mask(false); //返回前重新初始化反转掩码。
    return(GC_PROBE_FAIL_INIT); // 啥也不做，退出
  }

  //设置和队列探测动作。自动循环不应启动循环。
  mc_line(target, pl_data);

  //激活步进器模块中的探测状态监视器。
  sys_probe_state = PROBE_ACTIVE;

  //执行探测循环。在此处等待，直到触发探针或运动完成。
  system_set_exec_state_flag(EXEC_CYCLE_START);
  do {
    protocol_execute_realtime();
    if (sys.abort) { return(GC_PROBE_ABORT); } //检查系统是否中止
  } while (sys.state != STATE_IDLE);

  //探测周期完成！

  //如果探测失败并且启用了“带错误循环”，则设置状态变量和错误输出。
  if (sys_probe_state == PROBE_ACTIVE) {
    if (is_no_error) { memcpy(sys_probe_position, sys_position, sizeof(sys_position)); }
    else { system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_CONTACT); }
  } else {
    sys.probe_succeeded = true; //向系统指示探测循环已成功完成。
  }
  sys_probe_state = PROBE_OFF; //确保探测器状态监视器已禁用。
  probe_configure_invert_mask(false); //重新初始化反转掩码。
  protocol_execute_realtime();   //检查并执行运行时命令

  //重置步进器和规划器缓冲区，以移除剩余的探头运动。
  st_reset(); //重置步骤段缓冲区。
  plan_reset(); //重置计划器缓冲区。清零规划器位置。确保清除探测运动。
  plan_sync_position(); //将计划器位置同步到当前机器位置。

  #ifdef MESSAGE_PROBE_COORDINATES
    //全部完成！将探针位置作为消息输出。
    report_probe_parameters();
  #endif

  if (sys.probe_succeeded) { return(GC_PROBE_FOUND); } //成功的探测周期。
  else { return(GC_PROBE_FAIL_END); } //未能在行程内触发探针。有无错误。
}


//计划并执行单个停车特殊运动案例。独立于主计划缓冲区。
//注意：使用始终空闲的规划器环形缓冲区头存储运动参数以供执行。
#ifdef PARKING_ENABLE
  void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data)
  {
    if (sys.abort) { return; } //在中止期间阻塞。

    uint8_t plan_status = plan_buffer_line(parking_target, pl_data);

    if (plan_status) {
      bit_true(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
      bit_false(sys.step_control, STEP_CONTROL_END_MOTION); //如果进刀保持激活，允许执行停车运动。
      st_parking_setup_buffer(); //特殊停车运动箱的设置步骤段缓冲区
      st_prep_buffer();
      st_wake_up();
      do {
        protocol_exec_rt_system();
        if (sys.abort) { return; }
      } while (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION);
      st_parking_restore_buffer(); //将步骤段缓冲区恢复到正常运行状态。
    } else {
      bit_false(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
      protocol_exec_rt_system();
    }

  }
#endif


#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
  void mc_override_ctrl_update(uint8_t override_state)
  {
    //在更改覆盖控制状态之前，请完成所有排队的命令
    protocol_buffer_synchronize();
    if (sys.abort) { return; }
    sys.override_ctrl = override_state;
  }
#endif


//该方法通过设置实时重置命令并终止系统中的任何活动进程来准备系统重置。
//这还检查Grbl处于运动状态时是否发出系统重置。
//如果是这样，则停止步进器并将系统警报设置为标记位置丢失，因为存在突然的不受控制的减速。
//通过实时中止命令和硬限位在中断级别调用。所以，保持在最小化。
void mc_reset()
{
  //只有此功能才能设置系统复位。有助于防止多次终止呼叫。
  if (bit_isfalse(sys_rt_exec_state, EXEC_RESET)) {
    system_set_exec_state_flag(EXEC_RESET);

    //杀死主轴和冷却液。
    spindle_stop();
    coolant_stop();

    //仅当步进电机处于任何运动状态，即循环、主动保持或归位时，才杀死步进器。
//注意：如果通过“步进空闲延迟”设置使步进器保持启用状态，这也会通过完全避免go_idle调用使步进器保持启用状态，
//除非运动状态冲突，否则所有其他动作都将关闭。
    if ((sys.state & (STATE_CYCLE | STATE_HOMING | STATE_JOG)) ||
    		(sys.step_control & (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION))) {
      if (sys.state == STATE_HOMING) { 
        if (!sys_rt_exec_alarm) {system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
      } else { system_set_exec_alarm(EXEC_ALARM_ABORT_CYCLE); }
      st_go_idle(); // 强制关闭步进电机，因为位置可能已经丢失.
    }
  }
}
