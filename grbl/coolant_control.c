/*
  coolant_control.c - 冷却剂控制方法
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


void coolant_init()
{
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT); //配置为输出引脚
  #ifdef ENABLE_M7
    COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
  #endif
  coolant_stop();
}


//返回当前冷却液输出状态。覆盖可能会改变其编程状态。
uint8_t coolant_get_state()
{
  uint8_t cl_state = COOLANT_STATE_DISABLE;
  #ifdef INVERT_COOLANT_FLOOD_PIN
    if (bit_isfalse(COOLANT_FLOOD_PORT,(1 << COOLANT_FLOOD_BIT))) {
  #else
    if (bit_istrue(COOLANT_FLOOD_PORT,(1 << COOLANT_FLOOD_BIT))) {
  #endif
    cl_state |= COOLANT_STATE_FLOOD;
  }
  #ifdef ENABLE_M7
    #ifdef INVERT_COOLANT_MIST_PIN
      if (bit_isfalse(COOLANT_MIST_PORT,(1 << COOLANT_MIST_BIT))) {
    #else
      if (bit_istrue(COOLANT_MIST_PORT,(1 << COOLANT_MIST_BIT))) {
    #endif
      cl_state |= COOLANT_STATE_MIST;
    }
  #endif
  return(cl_state);
}


//由coolant_init（）、coolant_set_state（）和mc_reset（）直接调用，它们可以处于中断级别。
//未设置报告标志，但仅由不需要它的例程调用。
void coolant_stop()
{
  #ifdef INVERT_COOLANT_FLOOD_PIN
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
  #else
    COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  #endif
  #ifdef ENABLE_M7
    #ifdef INVERT_COOLANT_MIST_PIN
      COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
    #else
      COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
    #endif
  #endif
}


//仅主程序。立即设置溢流冷却液运行状态，并在启用时雾化冷却液。还设置一个标志以报告冷却液状态的更新。
//由coolant toggle override、parking restore、parking retract、sleep mode、g-code parser program end和g-code parser coolant_sync（）调用。
void coolant_set_state(uint8_t mode)
{
  if (sys.abort) { return; } //在中止期间阻塞。
  
	if (mode & COOLANT_FLOOD_ENABLE) {
		#ifdef INVERT_COOLANT_FLOOD_PIN
			COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
		#else
			COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
		#endif
	} else {
	  #ifdef INVERT_COOLANT_FLOOD_PIN
			COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
		#else
			COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
		#endif
	}
  
	#ifdef ENABLE_M7
		if (mode & COOLANT_MIST_ENABLE) {
			#ifdef INVERT_COOLANT_MIST_PIN
				COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
			#else
				COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
			#endif
		} else {
			#ifdef INVERT_COOLANT_MIST_PIN
				COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
			#else
				COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
			#endif
		}
	#endif
	
  sys.report_ovr_counter = 0; //设置为立即报告更改
}


//用于设置冷却液状态的G代码解析器入口点。如果中止或检查模式处于活动状态，则强制规划器缓冲区同步并停止。
void coolant_sync(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); //确保冷却液在程序中指定时开启。
  coolant_set_state(mode);
}
