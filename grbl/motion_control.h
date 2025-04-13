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

#ifndef motion_control_h
#define motion_control_h


//系统运动命令的行号必须为零。
#define HOMING_CYCLE_LINE_NUMBER 0
#define PARKING_MOTION_LINE_NUMBER 0

#define HOMING_CYCLE_ALL  0//必须是零。
#define HOMING_CYCLE_X    bit(X_AXIS)
#define HOMING_CYCLE_Y    bit(Y_AXIS)
#define HOMING_CYCLE_Z    bit(Z_AXIS)


//在绝对毫米坐标系下执行线性运动。
//除非反向进给速度为真，否则进给速度以毫米/秒为单位。
//那么进给率意味着运动应在（1分钟）/进给率时间内完成。
void mc_line(float *target, plan_line_data_t *pl_data);

//以偏移模式格式执行圆弧。
//位置==当前xyz，目标==目标xyz，偏移==与当前xyz的偏移，轴XXX定义刀具空间中的圆平面，轴线性是螺旋移动的方向，半径==圆半径，是顺时针圆弧布尔值。
//用于矢量变换方向。
void mc_arc(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius,
  uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc);

//停留特定的秒数
void mc_dwell(float seconds);

//执行归位循环以定位机器零位。需要限位开关。
void mc_homing_cycle(uint8_t cycle_mask);

//执行刀具长度探测循环。需要探针开关。
uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags);

//处理更新覆盖控制状态。
void mc_override_ctrl_update(uint8_t override_state);

//计划并执行单个停车特殊运动案例。独立于主计划缓冲区。
void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data);

//执行系统重置。如果处于运动状态，则停止所有运动并设置系统警报。
void mc_reset();

#endif
