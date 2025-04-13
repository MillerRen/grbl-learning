/*
  jog.h - 点动模块
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef jog_h
#define jog_h

#include "gcode.h"

//系统运动线数目必须为零。
#define JOG_LINE_NUMBER 0

//设置从g代码解析器接收的有效j点动运动，检查软限制，并执行点动。
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block);

#endif
