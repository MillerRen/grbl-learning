/*
  gcode.h - rs274/ngc 解释器.
  Grbl 的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef gcode_h
#define gcode_h


 // 定义模态组内部编号，用于检查多个命令冲突并跟踪
 // 在块中调用的命令类型。模态组是一组g代码命令，它们是
 // 相互排斥，或不能存在于同一行上，因为它们各自切换状态或执行
 // 独特的运动。这些定义见NIST RS274-NGC v3 g代码标准，可在线获取，
 // 并且与制造商（Haas、Fanuc、Mazak等）提供的其他g代码解释器类似/相同。
 // 注意：模态组定义值必须是连续的，并且从零开始。
#define MODAL_GROUP_G0 0 // [G4、G10、G28、G28.1、G30、G30.1、G53、G92、G92.1]非模态
#define MODAL_GROUP_G1 1 // [G0，G1，G2，G3，G38,2，G38,3，G38,4，G38,5，G80]运动
#define MODAL_GROUP_G2 2 // [G17、G18、G19]平面选择
#define MODAL_GROUP_G3 3 // [G90，G91]距离模式
#define MODAL_GROUP_G4 4 // [G91.1]电弧IJK距离模式
#define MODAL_GROUP_G5 5 // [G93，G94]进给速度模式
#define MODAL_GROUP_G6 6 // [G20，G21]单位
#define MODAL_GROUP_G7 7 // [G40]刀具半径补偿模式。不支持G41/42。
#define MODAL_GROUP_G8 8 // [G43.1，G49]刀具长度偏移
#define MODAL_GROUP_G12 9 // [G54、G55、G56、G57、G58、G59]坐标系选择
#define MODAL_GROUP_G13 10 // [G61]控制模式

#define MODAL_GROUP_M4 11 // [M0，M1，M2，M30]停止
#define MODAL_GROUP_M7 12 // [M3、M4、M5]主轴调整
#define MODAL_GROUP_M8 13 // [M7，M8，M9]冷却液控制
#define MODAL_GROUP_M9 14 // [M56]覆盖控制

 // 定义执行类型内模态组（运动、停止、非模态）的命令动作。
 // 解析器在内部使用，以了解要执行的命令。
 // 注意：一些宏值被指定为特定值，以使g代码状态报告和解析编译更小。由于328p上的闪存完全耗尽，因此有必要。
 // 虽然不理想，但只要小心使用状态为“不改变”的值，并检查两个报告即可。c和gcode。如果您需要更改它们，请查看它们是如何使用的。

 // 模态组G0：非模态动作
#define NON_MODAL_NO_ACTION 0 // （默认值：必须为零）
#define NON_MODAL_DWELL 4 // G4（不要改变值）
#define NON_MODAL_SET_COORDINATE_DATA 10 // G10（不要改变值）
#define NON_MODAL_GO_HOME_0 28 // G28（不要改变值）
#define NON_MODAL_SET_HOME_0 38 // G28.1（不要改变值）
#define NON_MODAL_GO_HOME_1 30 // G30（不要改变值）
#define NON_MODAL_SET_HOME_1 40 // G30.1（不要改变值）
#define NON_MODAL_ABSOLUTE_OVERRIDE 53 // G53（不要改变值）
#define NON_MODAL_SET_COORDINATE_OFFSET 92 // G92（不要改变值）
#define NON_MODAL_RESET_COORDINATE_OFFSET 102 // G92。1（不要改变值）

 // 模态组G1：运动模态
#define MOTION_MODE_SEEK 0 // G0（默认值：必须为零）
#define MOTION_MODE_LINEAR 1 // G1（不改变值）
#define MOTION_MODE_CW_ARC 2 // G2（不改变值）
#define MOTION_MODE_CCW_ARC 3 // G3（不要改变值）
#define MOTION_MODE_PROBE_TOWARD 140 // G38.2（不要改变值）
#define MOTION_MODE_PROBE_TOWARD_NO_ERROR 141 // G38。3（不要改变值）
#define MOTION_MODE_PROBE_AWAY 142 // G38.4（不要改变值）
#define MOTION_MODE_PROBE_AWAY_NO_ERROR 143 // G38。5（不要改变值）
#define MOTION_MODE_NONE 80 // G80（不要改变值）

 // 模态组G2：平面选择
#define PLANE_SELECT_XY 0 // G17（默认值：必须为零）
#define PLANE_SELECT_ZX 1 // G18（不要改变值）
#define PLANE_SELECT_YZ 2 // G19（不要改变值）

 // 模式组G3：距离模式
#define DISTANCE_MODE_ABSOLUTE 0 // G90（默认值：必须为零）
#define DISTANCE_MODE_INCREMENTAL 1 // G91（不要改变值）

 // 模式组G4：Arc IJK距离模式
#define DISTANCE_ARC_MODE_INCREMENTAL 0 // G91.1（默认值：必须为零）

 // 模态组M4：程序流
#define PROGRAM_FLOW_RUNNING 0 // （默认值：必须为零）
#define PROGRAM_FLOW_PAUSED 3  //  M0
#define PROGRAM_FLOW_OPTIONAL_STOP 1 // M1注意：不支持，但有效且被忽略。
#define PROGRAM_FLOW_COMPLETED_M2  2 // M2（不改变值）
#define PROGRAM_FLOW_COMPLETED_M30 30 // M30（不改变值）

 // 模式组G5：进给速度模式
#define FEED_RATE_MODE_UNITS_PER_MIN  0 // G94（默认值：必须为零）
#define FEED_RATE_MODE_INVERSE_TIME   1 // G93（不要改变值）

 // 模态组G6：单元模态
#define UNITS_MODE_MM 0 // G21（默认值：必须为零）
#define UNITS_MODE_INCHES 1 // G20（不改变值）

 // 模式组G7：刀具半径补偿模式
#define CUTTER_COMP_DISABLE 0 // G40（默认值：必须为零）

 // 模式组G13：控制模式
#define CONTROL_MODE_EXACT_PATH 0 // G61（默认值：必须为零）

 // 模态组M7：主轴控制
#define SPINDLE_DISABLE 0 // M5（默认值：必须为零）
#define SPINDLE_ENABLE_CW   PL_COND_FLAG_SPINDLE_CW // M3（注：使用计划器条件位标志）
#define SPINDLE_ENABLE_CCW  PL_COND_FLAG_SPINDLE_CCW // M4（注意：使用计划器条件位标志）

 // 模式组M8：冷却液控制
#define COOLANT_DISABLE 0 // M9（默认值：必须为零）
#define COOLANT_FLOOD_ENABLE  PL_COND_FLAG_COOLANT_FLOOD // M8（注意：使用计划器条件位标志）
#define COOLANT_MIST_ENABLE   PL_COND_FLAG_COOLANT_MIST // M7（注：使用计划器条件位标志）

 // 模式组G8：刀具长度偏移
#define TOOL_LENGTH_OFFSET_CANCEL 0 // G49（默认值：必须为零）
#define TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC 1  //  G43.1

 // 模态组M9：超越控制
#ifdef DEACTIVATE_PARKING_UPON_INIT
  #define OVERRIDE_DISABLED  0 // （默认值：必须为零）
  #define OVERRIDE_PARKING_MOTION 1  //  M56
#else
  #define OVERRIDE_PARKING_MOTION 0 // M56（默认值：必须为零）
  #define OVERRIDE_DISABLED  1 // 禁止停车。
#endif

 // 模态组G12：活动工作坐标系
 // 不适用：存储要更改为的坐标系值（54-59）。

 // 定义参数词映射。
#define WORD_F  0
#define WORD_I  1
#define WORD_J  2
#define WORD_K  3
#define WORD_L  4
#define WORD_N  5
#define WORD_P  6
#define WORD_R  7
#define WORD_S  8
#define WORD_T  9
#define WORD_X  10
#define WORD_Y  11
#define WORD_Z  12

 // 定义g代码解析器位置更新标志
#define GC_UPDATE_POS_TARGET   0 // 必须为零
#define GC_UPDATE_POS_SYSTEM   1
#define GC_UPDATE_POS_NONE     2

 // 定义探头循环退出状态并分配适当的位置更新。
#define GC_PROBE_FOUND      GC_UPDATE_POS_SYSTEM
#define GC_PROBE_ABORT      GC_UPDATE_POS_NONE
#define GC_PROBE_FAIL_INIT  GC_UPDATE_POS_NONE
#define GC_PROBE_FAIL_END   GC_UPDATE_POS_TARGET
#ifdef SET_CHECK_MODE_PROBE_TO_START
  #define GC_PROBE_CHECK_MODE   GC_UPDATE_POS_NONE  
#else
  #define GC_PROBE_CHECK_MODE   GC_UPDATE_POS_TARGET
#endif

 // 定义用于处理特殊情况的gcode解析器标志。
#define GC_PARSER_NONE                  0 // 必须是零。
#define GC_PARSER_JOG_MOTION            bit(0)
#define GC_PARSER_CHECK_MANTISSA        bit(1)
#define GC_PARSER_ARC_IS_CLOCKWISE      bit(2)
#define GC_PARSER_PROBE_IS_AWAY         bit(3)
#define GC_PARSER_PROBE_IS_NO_ERROR     bit(4)
#define GC_PARSER_LASER_FORCE_SYNC      bit(5)
#define GC_PARSER_LASER_DISABLE         bit(6)
#define GC_PARSER_LASER_ISMOTION        bit(7)


 // 注意：当此结构为零时，上面定义了系统的默认设置。
typedef struct {
  uint8_t motion;           //  {G0,G1,G2,G3,G38.2,G80}
  uint8_t feed_rate;        //  {G93,G94}
  uint8_t units;            //  {G20,G21}
  uint8_t distance;         //  {G90,G91}
   //  uint8_t distance_arc;  //  {G91.1} NOTE: Don't track. Only default supported.
  uint8_t plane_select;     //  {G17,G18,G19}
   //  uint8_t cutter_comp;   //  {G40} NOTE: Don't track. Only default supported.
  uint8_t tool_length;      //  {G43.1,G49}
  uint8_t coord_select;     //  {G54,G55,G56,G57,G58,G59}
   //  uint8_t control;       //  {G61} NOTE: Don't track. Only default supported.
  uint8_t program_flow;     //  {M0,M1,M2,M30}
  uint8_t coolant;          //  {M7,M8,M9}
  uint8_t spindle;          //  {M3,M4,M5}
  uint8_t override;         //  {M56}
} gc_modal_t;

typedef struct {
  float f;          //  进给
  float ijk[3]; // I、J、K轴圆弧偏移
  uint8_t l;        //  G10 or canned cycles parameters
  int32_t n; // 行号
  float p;          //  G10 or 暂停参数
   //  float q;       //  G82 peck drilling
  float r; // 弧半径
  float s; // 主轴转速
  uint8_t t; // 刀具选择
  float xyz[3]; // X，Y，Z平移轴
} gc_values_t;


typedef struct {
  gc_modal_t modal;

  float spindle_speed; // 转速
  float feed_rate; // 毫米/分钟
  uint8_t tool; // 跟踪刀具编号。没有用。
  int32_t line_number; // 最后发送的行号

  float position[N_AXIS]; // 解释器认为工具在代码中的这一点上

  float coord_system[N_AXIS]; // 当前工作坐标系（G54+）。存储机器绝对位置的偏移量，单位为mm。调用时从EEPROM加载。
  float coord_offset[N_AXIS]; // 保留相对于机器零点的G92坐标偏移（工作坐标），单位为mm。非持久性。重置和引导时清除。
  float tool_length_offset; // 启用时跟踪刀具长度偏移值。
} parser_state_t;
extern parser_state_t gc_state;


typedef struct {
  uint8_t non_modal_command;
  gc_modal_t modal;
  gc_values_t values;
} parser_block_t;


 // 初始化解析器
void gc_init();

 // 执行一个rs275/ngc/g代码块
uint8_t gc_execute_line(char *line);

 // 设置g代码解析器的位置。以步为单位。
void gc_sync_position();

#endif
