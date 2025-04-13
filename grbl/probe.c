/*
  probe.c - 与探测方法有关的代码
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


//根据用户设置和探测循环模式，反转探针引脚状态。
uint8_t probe_invert_mask;


//探针引脚初始化例行程序。
void probe_init()
{
  PROBE_DDR &= ~(PROBE_MASK); //配置为输入引脚
  #ifdef DISABLE_PROBE_PIN_PULL_UP
    PROBE_PORT &= ~(PROBE_MASK); //正常低电压运行。需要外部下拉。
  #else
    PROBE_PORT |= PROBE_MASK;    //启用内部上拉电阻器。正常高位运行。
  #endif
  probe_configure_invert_mask(false); //初始化反转掩码。
}

//由probe_init（）和mc_probe（）例程调用。
//根据正常高/正常低操作的设置和朝向工件/远离工件的探测循环模式，设置探针引脚反转掩码，以适当设置引脚逻辑。
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; //初始化为零。
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}


//返回探针引脚状态。触发=真。由gcode解析器和探测状态监视器调用。
uint8_t probe_get_state() { return((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }


//监测探针引脚状态，并在检测到时记录系统位置。由步进ISR按ISR周期调用。
void probe_state_monitor()
{
  if (probe_get_state()) {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}
