/*
  limits.h - 与限位开关和执行复位循环有关的代码
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef limits_h
#define limits_h


//初始化限位模块
void limits_init();

//禁用硬限位。
void limits_disable();

//以逐位uint8变量的形式返回限位状态。
uint8_t limits_get_state();

//根据输入设置执行一部分归位循环。
void limits_go_home(uint8_t cycle_mask);

//检查是否存在违反软限位的情况
void limits_soft_check(float *target);

#endif
