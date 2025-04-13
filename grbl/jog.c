/*
  jog.h - Jogging methods
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

//设置从g代码解析器接收的有效j点动运动，检查软限制，并执行点动。
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block)
{
  //初始化点动运动的规划器数据结构。注意：在点动期间，允许主轴和冷却液在覆盖时完整功能。
  pl_data->feed_rate = gc_block->values.f;
  pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_block->values.n;
  #endif

  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
    if (system_check_travel_limits(gc_block->values.xyz)) { return(STATUS_TRAVEL_EXCEEDED); }
  }

  //校验点动命令。计划、设置状态和执行。
  mc_line(gc_block->values.xyz,pl_data);
  if (sys.state == STATE_IDLE) {
    if (plan_get_current_block() != NULL) { //检查是否有要执行的块。
      sys.state = STATE_JOG;
      st_prep_buffer();
      st_wake_up();  //注：手动启动。不需要状态机。
    }
  }

  return(STATUS_OK);
}
