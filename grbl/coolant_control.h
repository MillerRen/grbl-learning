/*
  coolant_control.h - spindle control methods
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef coolant_control_h
#define coolant_control_h

#define COOLANT_NO_SYNC     false
#define COOLANT_FORCE_SYNC  true

#define COOLANT_STATE_DISABLE   0//必须为零
#define COOLANT_STATE_FLOOD     PL_COND_FLAG_COOLANT_FLOOD
#define COOLANT_STATE_MIST      PL_COND_FLAG_COOLANT_MIST


//初始化冷却液控制引脚。
void coolant_init();

//返回当前冷却液输出状态。覆盖可能会改变其编程状态。
uint8_t coolant_get_state();

//立即禁用冷却液引脚。
void coolant_stop();

//根据指定的状态设置冷却液引脚。
void coolant_set_state(uint8_t mode);

//用于设置冷却液状态的G代码解析器入口点。检查并执行附加条件。
void coolant_sync(uint8_t mode);

#endif
