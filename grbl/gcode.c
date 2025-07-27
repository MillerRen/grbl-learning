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

#include "grbl.h"

// 注：g代码标准将最大行号定义为99999。
// 这似乎是一个任意值，某些GUI可能需要更多。
// 因此，在将浮点（7.2位精度）转换为整数时，我们根据最大安全值增加了它。
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // 受最大无符号8位值限制

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *未定义但必需

// 声明gc外部结构
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return (status);

void gc_init()
{
  memset(&gc_state, 0, sizeof(parser_state_t));

  // 加载默认G54坐标系。
  if (!(settings_read_coord_data(gc_state.modal.coord_select, gc_state.coord_system)))
  {
    report_status_message(STATUS_SETTING_READ_FAIL);
  }
}

// 以毫米为单位设置g代码解析器的位置。用步表示。由系统中止和硬限位回拉例程调用。
void gc_sync_position()
{
  system_convert_array_steps_to_mpos(gc_state.position, sys_position);
}

// 执行一行以0结尾的G代码。
// 假定该行仅包含大写字符和有符号浮点值（无空格）。
// 注释和块删除字符已被删除。
// 在该函数中，所有单位和位置分别以（毫米，毫米/分钟）和绝对机器坐标转换并导出为grbl的内部函数。
uint8_t gc_execute_line(char *line)
{
  /*-------------------------------------------------------------------------------------
步骤1：
初始化解析器块结构并复制当前g代码状态模式。
解析器将更新这些模式和命令，因为块行是解析器，并且仅用于和错误检查成功后执行。
解析器块结构还包含块值结构、字跟踪变量和新块的非模态命令跟踪器。
此结构包含执行块所需的所有信息。*/

  memset(&gc_block, 0, sizeof(parser_block_t));                 // 初始化解析器块结构。
  memcpy(&gc_block.modal, &gc_state.modal, sizeof(gc_modal_t)); // 复制当前模式

  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; // 跟踪G10 P坐标选择以执行

  // 初始化轴索引兼容操作的位标志跟踪变量。
  uint8_t axis_words = 0; // XYZ跟踪
  uint8_t ijk_words = 0;  // IJK跟踪

  // 初始化命令和值字以及解析器标志变量。
  uint16_t command_words = 0; // 跟踪G和M命令字。也用于模式组冲突。
  uint16_t value_words = 0;   // 跟踪值词
  uint8_t gc_parser_flags = GC_PARSER_NONE;

  // 确定该线是点动运动还是正常g代码块。
  if (line[0] == '$')
  { // 注意：`$J=`在传递到此函数时已被解析。
    // 设置G1和G94强制模式以确保准确的错误检查。
    gc_parser_flags |= GC_PARSER_JOG_MOTION;
    gc_block.modal.motion = MOTION_MODE_LINEAR;
    gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
#ifdef USE_LINE_NUMBERS
    gc_block.values.n = JOG_LINE_NUMBER; // 初始化点动期间报告的默认行号。
#endif
  }

  /*-------------------------------------------------------------------------------------
步骤2：导入块行中的所有g代码字。g代码字是一个字母后跟
一个数字，可以是“G”/“M”命令，也可以设置/分配命令值。而且
对任何重复的命令字模式组冲突执行初始错误检查
单词，以及为值单词F、N、P、T和S设置的负值。*/

  uint8_t word_bit; // 用于分配跟踪变量的位值
  uint8_t char_counter;
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;
  if (gc_parser_flags & GC_PARSER_JOG_MOTION)
  {
    char_counter = 3;
  } // 在“$J”之后开始解析=`
  else
  {
    char_counter = 0;
  }

  while (line[char_counter] != 0)
  { // 循环，直到行中不再有g代码字。

    // 导入下一个g代码字，应为字母后跟值。否则，就会出错。
    letter = line[char_counter];
    if ((letter < 'A') || (letter > 'Z'))
    {
      FAIL(STATUS_EXPECTED_COMMAND_LETTER);
    } // [期望词字母]
    char_counter++;
    if (!read_float(line, &char_counter, &value))
    {
      FAIL(STATUS_BAD_NUMBER_FORMAT);
    } // [期望词值]

    // 将值转换为较小的uint8有效位和尾数值以解析此单词。
    // 注意：尾数乘以100以捕获非整数命令值。
    // 当用于命令时，这比NIST gcode x10的要求更精确，但对于要求整数在0.0001范围内的值字，这还不够精确。
    // 这应该是一个足够好的妥协，可以捕获大多数非整数误差。
    // 为了使其兼容，我们只需要将尾数更改为int16，但这会增加编译的闪存空间。
    // 也许以后再更新。
    int_value = trunc(value);
    mantissa = round(100 * (value - int_value)); // 计算Gxx.x命令的尾数。

    // 注意：舍入必须用于捕捉小的浮点误差。

    // 检查g代码字是否受支持，是否因模式组冲突而出现错误，是否在g代码块中重复出现。
    // 如果可以，则更新命令或记录其值。
    switch (letter)
    {

      /*“G”和“M”命令字：解析命令并检查模态组冲突。
        注：模式组号在NIST RS274-NGC v3第20页表4中定义*/

    case 'G':
      // 确定“G”命令及其模态组
      switch (int_value)
      {
      case 10:
      case 28:
      case 30:
      case 92:
        // 检查是否在同一块上调用G10/28/30/92和G0/1/2/3/38。
        //  *G43.1也是一个轴命令，但没有以这种方式明确定义。
        if (mantissa == 0)
        { // 忽略 G28.1, G30.1, and G92.1
          if (axis_command)
          {
            FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT);
          } //[轴字/命令冲突]
          axis_command = AXIS_COMMAND_NON_MODAL;
        }
        // 不中断。继续下一行。
      case 4:
      case 53:
        word_bit = MODAL_GROUP_G0;
        gc_block.non_modal_command = int_value;
        if ((int_value == 28) || (int_value == 30) || (int_value == 92))
        {
          if (!((mantissa == 0) || (mantissa == 10)))
          {
            FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
          }
          gc_block.non_modal_command += mantissa;
          mantissa = 0; // 设置为零表示有效的非整数G命令。
        }
        break;
      case 0:
      case 1:
      case 2:
      case 3:
      case 38:
        // 检查是否在同一块上使用G10/28/30/92调用G0/1/2/3/38。
        //  *G43。1也是一个轴命令，但没有以这种方式明确定义。
        if (axis_command)
        {
          FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT);
        } //[轴字/命令冲突]
        axis_command = AXIS_COMMAND_MOTION_MODE;
        // 不中断。继续下一行。
      case 80:
        word_bit = MODAL_GROUP_G1;
        gc_block.modal.motion = int_value;
        if (int_value == 38)
        {
          if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50)))
          {
            FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); //[不支持的G38.x命令] G38.2 G38.3 G38.4 G38.5
          }
          gc_block.modal.motion += (mantissa / 10) + 100;
          mantissa = 0; // 设置为零表示有效的非整数G命令。
        }
        break;
      case 17:
      case 18:
      case 19:
        word_bit = MODAL_GROUP_G2;
        gc_block.modal.plane_select = int_value - 17;
        break;
      case 90:
      case 91:
        if (mantissa == 0)
        {
          word_bit = MODAL_GROUP_G3;
          gc_block.modal.distance = int_value - 90;
        }
        else
        {
          word_bit = MODAL_GROUP_G4;
          if ((mantissa != 10) || (int_value == 90))
          {
            FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
          }             // [G90.1 不支持]
          mantissa = 0; // 设置为零表示有效的非整数G命令。
          // 否则，默认为arc IJK增量模式。G91.1什么也不做。
        }
        break;
      case 93:
      case 94:
        word_bit = MODAL_GROUP_G5;
        gc_block.modal.feed_rate = 94 - int_value;
        break;
      case 20:
      case 21:
        word_bit = MODAL_GROUP_G6;
        gc_block.modal.units = 21 - int_value;
        break;
      case 40:
        word_bit = MODAL_GROUP_G7;
        // 注意：不需要，因为刀具半径补偿始终处于禁用状态。
        // 仅在此处支持经常出现在g代码程序标题中的G40命令，以设置默认值。
        //  gc_block.
        //  modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
        break;
      case 43:
      case 49:
        word_bit = MODAL_GROUP_G8;
        // 注：NIST g代码标准含糊地规定，当刀具长度偏移改变时，不能更新任何轴运动或坐标偏移。
        // 意思是G43，G43.1和G49都是显式轴命令，无论它们是否需要轴字。
        if (axis_command)
        {
          FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT);
        } //[轴字/命令冲突]}
        axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
        if (int_value == 49)
        { // G49
          gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL;
        }
        else if (mantissa == 10)
        { // G43.1
          gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
        }
        else
        {
          FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        }             //[不支持的G43.x命令]
        mantissa = 0; // 设置为零表示有效的非整数G命令。
        break;
      case 54:
      case 55:
      case 56:
      case 57:
      case 58:
      case 59:
        // 注意：不支持G59.x。（但它们的int_值分别为60、61和62。）
        word_bit = MODAL_GROUP_G12;
        gc_block.modal.coord_select = int_value - 54; // 切换到数组索引。
        break;
      case 61:
        word_bit = MODAL_GROUP_G13;
        if (mantissa != 0)
        {
          FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        } // [G61.1 不支持]
        // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
        break;
      default:
        FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); //[不支持的G命令]
      }
      if (mantissa > 0)
      {
        FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER);
      } //[不支持或无效的Gxx.x命令]

      // 检查当前块中每个模式组是否有多个命令冲突
      // 注意：如果命令有效，则始终指定变量“word_bit”。
      if (bit_istrue(command_words, bit(word_bit)))
      {
        FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION);
      }
      command_words |= bit(word_bit);
      break;

    case 'M':

      // 确定“M”命令及其模式组
      if (mantissa > 0)
      {
        FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER);
      } //[没有Mxx.x命令]
      switch (int_value)
      {
      case 0:
      case 1:
      case 2:
      case 30:
        word_bit = MODAL_GROUP_M4;
        switch (int_value)
        {
        case 0:
          gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED;
          break; // 程序暂停
        case 1:
          break; // 不支持可选停止。忽视
        default:
          gc_block.modal.program_flow = int_value; // 程序结束和重置
        }
        break;
      case 3:
      case 4:
      case 5:
        word_bit = MODAL_GROUP_M7;
        switch (int_value)
        {
        case 3:
          gc_block.modal.spindle = SPINDLE_ENABLE_CW;
          break;
        case 4:
          gc_block.modal.spindle = SPINDLE_ENABLE_CCW;
          break;
        case 5:
          gc_block.modal.spindle = SPINDLE_DISABLE;
          break;
        }
        break;
#ifdef ENABLE_M7
      case 7:
      case 8:
      case 9:
#else
      case 8:
      case 9:
#endif
        word_bit = MODAL_GROUP_M8;
        switch (int_value)
        {
#ifdef ENABLE_M7
        case 7:
          gc_block.modal.coolant |= COOLANT_MIST_ENABLE;
          break;
#endif
        case 8:
          gc_block.modal.coolant |= COOLANT_FLOOD_ENABLE;
          break;
        case 9:
          gc_block.modal.coolant = COOLANT_DISABLE;
          break; // M9同时禁用M7和M8。
        }
        break;
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
      case 56:
        word_bit = MODAL_GROUP_M9;
        gc_block.modal.override = OVERRIDE_PARKING_MOTION;
        break;
#endif
      default:
        FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); //[不支持的M命令]
      }

      // 检查当前块中每个模式组是否有多个命令冲突
      // 注意：如果命令有效，则始终指定变量“word_bit”。
      if (bit_istrue(command_words, bit(word_bit)))
      {
        FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION);
      }
      command_words |= bit(word_bit);
      break;

    // 注意：所有剩余字母都指定值。
    default:

      /*非命令字：此初始解析阶段仅检查剩余合法g代码字的重复并存储其值。
      由于某些单词（I、J、K、L、P、R）具有多个含义和/或取决于发出的命令，因此稍后将执行错误检查。*/
      switch (letter)
      {
      // case 'A': // Not supported
      // case 'B': // Not supported
      // case 'C': // Not supported
      // case 'D': // Not supported
      case 'F':
        word_bit = WORD_F;
        gc_block.values.f = value;
        break;
      // case 'H': // Not supported
      case 'I':
        word_bit = WORD_I;
        gc_block.values.ijk[X_AXIS] = value;
        ijk_words |= (1 << X_AXIS);
        break;
      case 'J':
        word_bit = WORD_J;
        gc_block.values.ijk[Y_AXIS] = value;
        ijk_words |= (1 << Y_AXIS);
        break;
      case 'K':
        word_bit = WORD_K;
        gc_block.values.ijk[Z_AXIS] = value;
        ijk_words |= (1 << Z_AXIS);
        break;
      case 'L':
        word_bit = WORD_L;
        gc_block.values.l = int_value;
        break;
      case 'N':
        word_bit = WORD_N;
        gc_block.values.n = trunc(value);
        break;
      case 'P':
        word_bit = WORD_P;
        gc_block.values.p = value;
        break;
      // 注意：对于某些命令，P值必须是整数，但不支持这些命令。

      // case 'Q': // Not supported
      case 'R':
        word_bit = WORD_R;
        gc_block.values.r = value;
        break;
      case 'S':
        word_bit = WORD_S;
        gc_block.values.s = value;
        break;
      case 'T':
        word_bit = WORD_T;
        if (value > MAX_TOOL_NUMBER)
        {
          FAIL(STATUS_GCODE_MAX_VALUE_EXCEEDED);
        }
        gc_block.values.t = int_value;
        break;
      case 'X':
        word_bit = WORD_X;
        gc_block.values.xyz[X_AXIS] = value;
        axis_words |= (1 << X_AXIS);
        break;
      case 'Y':
        word_bit = WORD_Y;
        gc_block.values.xyz[Y_AXIS] = value;
        axis_words |= (1 << Y_AXIS);
        break;
      case 'Z':
        word_bit = WORD_Z;
        gc_block.values.xyz[Z_AXIS] = value;
        axis_words |= (1 << Z_AXIS);
        break;
      default:
        FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
      }

      // 注意：如果非命令字母有效，则始终指定变量“word_bit”。
      if (bit_istrue(value_words, bit(word_bit)))
      {
        FAIL(STATUS_GCODE_WORD_REPEATED);
      } //[字重复]

      // 检查字F、N、P、T和S的无效负值。
      // 注意：这里进行负值检查只是为了提高代码效率。
      if (bit(word_bit) & (bit(WORD_F) | bit(WORD_N) | bit(WORD_P) | bit(WORD_T) | bit(WORD_S)))
      {
        if (value < 0.0)
        {
          FAIL(STATUS_NEGATIVE_VALUE);
        } //[字值不能为负]
      }
      value_words |= bit(word_bit); // 指示指定参数的标志。
    }
  }
  // 解析完成！

  /* -------------------------------------------------------------------------------------
    步骤3：错误检查此块中传递的所有命令和值。这一步确保了所有命令执行有效，并尽可能遵循NIST标准。
     如果发现错误，将转储此块中的所有命令和值，并且不会更新激活的系统g代码模式。
     如果模块正常，则激活的系统g代码模式将被激活根据此块的命令进行更新，并发出执行信号。

     此外，我们还必须根据被解析对象设置的模式对传递的所有值进行预转换块 。
     有许多错误检查需要目标信息，只有在我们结合错误检查转换这些值时才能准确计算这些信息。
     这将下一执行步骤降级为仅更新系统g代码模式并按顺序执行编程动作。
     执行步骤不需要任何转换计算，只需要执行所需的最少检查。
  */

  /* 注意：此时，g代码块已被解析，可以释放块行。
     注意：在将来的某个时候，也可以中断步骤2，以允许在每个单词的基础上对块进行分段解析，而不是对整个块进行分段解析。
     这可以消除为整个块维护大字符串变量的需要，并释放一些内存。
     为此，只需保留步骤1中的所有数据，例如新的块数据结构、模式组和值bitflag跟踪变量以及轴数组索引兼容变量。
     该数据包含接收结束字符时错误检查新g代码块所需的所有信息。 然而，这将打破Grbl目前的工作方式，并需要一些重构以使其兼容。
  */

  //[0.非特定/常见错误检查和其他设置]：

  // 确定隐式轴命令条件。已传递轴单词，但没有明确的轴命令已发送。如果是，请将轴命令设置为当前运动模式。
  if (axis_words)
  {
    if (!axis_command)
    {
      axis_command = AXIS_COMMAND_MOTION_MODE;
    } // Assign implicit motion-mode
  }

  // 检查有效的行号N值。
  if (bit_istrue(value_words, bit(WORD_N)))
  {
    // 行号值不能小于零（完成）或大于最大行号。
    if (gc_block.values.n > MAX_LINE_NUMBER)
    {
      FAIL(STATUS_GCODE_INVALID_LINE_NUMBER);
    } //[超过最大行号]
  }
  // bit_false(value_words,bit(WORD_N)); // NOTE: Single-meaning value word. Set at end of error-checking.

  // 在错误检查结束时跟踪未使用的字。
  // 注意：在错误检查结束时，单个意义值的单词会一次全部删除，因为它们总是在出现时使用。
  // 这样做是为了节省几个字节的闪存。
  // 为清楚起见，在使用时可以删除单个意义值词。此外，轴字的处理方式也相同。
  // 如果存在显式/隐式轴命令，则始终使用XYZ字，并在错误检查结束时将其删除。
  //[1.注释]：不支持MSG。由协议执行的注释处理。
  //[2.设置进给速度模式]：G93 F字缺失，G1、G2/3激活，隐式或显式。
  // 从G93切换到G94后，未定义进给速度。
  // 注意：对于点动，忽略先前的进给速度模式。强制执行G94并检查所需的F字。
  if (gc_parser_flags & GC_PARSER_JOG_MOTION)
  {
    if (bit_isfalse(value_words, bit(WORD_F)))
    {
      FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE);
    }
    if (gc_block.modal.units == UNITS_MODE_INCHES)
    {
      gc_block.values.f *= MM_PER_INCH;
    }
  }
  else
  {
    if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME)
    { // = G93

      // 注：G38也可以反时限运行，但未定义为错误。此处添加了缺少的F字检查。
      if (axis_command == AXIS_COMMAND_MOTION_MODE)
      {
        if ((gc_block.modal.motion != MOTION_MODE_NONE) && (gc_block.modal.motion != MOTION_MODE_SEEK))
        {
          if (bit_isfalse(value_words, bit(WORD_F)))
          {
            FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE);
          } //[F单词缺失]
        }
      }
      // 注意：从G94切换到G93后，检查是否要传递F字似乎是多余的。
      // 如果进给速率值在每个反向时间块后总是重置为零且未定义，我们将完成完全相同的事情，因为使用此值的命令已经执行未定义的检查。
      // 这也将允许在该开关之后执行其他命令，而不会出现不必要的错误。
      // 他的代码与上面的进给速度模式和下面设置的进给速度错误检查相结合。
      //[3.设定进给速度]：F为负（完成）
      //-在反时限模式下：始终隐式地将块完成前后的进给速率值归零。
      // 注意：如果处于G93模式或从G94切换到G93模式，只需将F值保持为初始化零或在块中传递F字值即可。
      // 如果没有F字通过需要进给速率的运动命令传递，这将在运动模式错误检查中出错。
      // 然而，如果没有F字在没有需要进给速度的运动命令的情况下被传递，我们只需继续，状态进给速度值就会被更新为零，并且保持未定义状态。
    }
    else
    { // = G94

      //-在单位/毫米模式下：如果F字已通过，请确保值为毫米/分钟，否则按最后状态值。
      if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN)
      { // Last state is also G94
        if (bit_istrue(value_words, bit(WORD_F)))
        {
          if (gc_block.modal.units == UNITS_MODE_INCHES)
          {
            gc_block.values.f *= MM_PER_INCH;
          }
        }
        else
        {
          gc_block.values.f = gc_state.feed_rate; // 推送最后状态进给速率
        }
      } // 否则，从G93切换到G94，所以不要推最后状态进给速率。其未定义或传递的F字值。
    }
  }
  // bit_false(value_words,bit(WORD_F)); // 注：单义值词。在错误检查结束时设置。

  //[4.设置主轴转速]：S为负（完成）
  if (bit_isfalse(value_words, bit(WORD_S)))
  {
    gc_block.values.s = gc_state.spindle_speed;
  }
  // bit_false(value_words,bit(WORD_S)); // 注：单义值词。在错误检查结束时设置。

  //[5.选择工具]：不支持。只跟踪价值。T为负（完成）不是整数。大于最大刀具值。
  // bit_false(value_words,bit(WORD_T)); // 注：单义值词。在错误检查结束时设置。

  //[6.更换工具]：不适用
  //[7.主轴控制]：不适用
  //[8.冷却液控制]：不适用

//[9.覆盖控制]：不受支持，仅Grbl停靠运动覆盖控制除外。
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
  if (bit_istrue(command_words, bit(MODAL_GROUP_M9)))
  { // 已在解析器中设置为已启用。
    if (bit_istrue(value_words, bit(WORD_P)))
    {
      if (gc_block.values.p == 0.0)
      {
        gc_block.modal.override = OVERRIDE_DISABLED;
      }
      bit_false(value_words, bit(WORD_P));
    }
  }
#endif

  //[10.暂停]：缺少P值。P为负（完成）注：见下文。
  if (gc_block.non_modal_command == NON_MODAL_DWELL)
  {
    if (bit_isfalse(value_words, bit(WORD_P)))
    {
      FAIL(STATUS_GCODE_VALUE_WORD_MISSING);
    } //[P单词缺失]
    bit_false(value_words, bit(WORD_P));
  }

  //[11.设置活动平面]：不适用
  switch (gc_block.modal.plane_select)
  {
  case PLANE_SELECT_XY:
    axis_0 = X_AXIS;
    axis_1 = Y_AXIS;
    axis_linear = Z_AXIS;
    break;
  case PLANE_SELECT_ZX:
    axis_0 = Z_AXIS;
    axis_1 = X_AXIS;
    axis_linear = Y_AXIS;
    break;
  default: // case PLANE_SELECT_YZ:
    axis_0 = Y_AXIS;
    axis_1 = Z_AXIS;
    axis_linear = X_AXIS;
  }

  //[12.设定长度单位]：不适用
  // 将XYZ坐标值预转换为毫米（如果适用）。
  uint8_t idx;
  if (gc_block.modal.units == UNITS_MODE_INCHES)
  {
    for (idx = 0; idx < N_AXIS; idx++)
    { // 轴索引是一致的，因此可以使用循环。
      if (bit_istrue(axis_words, bit(idx)))
      {
        gc_block.values.xyz[idx] *= MM_PER_INCH;
      }
    }
  }

  //[13.刀具半径补偿]：不支持G41/42。错误，如果在G53激活时启用。
  //[G40错误]：G2/3电弧在G40之后编程。禁用后的线性移动小于刀具直径。
  // 注意：由于刀具半径补偿从未启用，这些G40错误不适用。Grbl支持G40
  // 仅为避免G40与g代码程序头一起发送时出错，以设置默认模式。

  //[14.刀具长度补偿]：不支持G43，但支持G43.1和G49支持。
  //[G43.1错误]：同一行中的运动命令。
  // 注：尽管没有明确说明，G43.1应仅应用于一个有效的配置的轴（在config.h中）。
  // 如果配置的轴不存在或存在任何其他轴单词。
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET)
  { // Indicates called in block.
    if (gc_block.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC)
    {
      if (axis_words ^ (1 << TOOL_LENGTH_OFFSET_AXIS))
      {
        FAIL(STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR);
      }
    }
  }

  //[15.坐标系选择]：*不适用。错误，如果刀具半径补偿激活。

  // TODO：循环激活时，EEPROM读取坐标数据可能需要缓冲区同步。
  // 读取操作会暂时暂停处理器，并可能导致罕见的崩溃。
  // 对于具有足够内存的处理器上的未来版本，所有坐标数据应存储在内存中，并仅在没有激活循环时写入EEPROM。
  float block_coord_system[N_AXIS];
  memcpy(block_coord_system, gc_state.coord_system, sizeof(gc_state.coord_system));
  if (bit_istrue(command_words, bit(MODAL_GROUP_G12)))
  { // 检查是否在块中调用
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM)
    {
      FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS);
    } //[大于N sys]
    if (gc_state.modal.coord_select != gc_block.modal.coord_select)
    {
      if (!(settings_read_coord_data(gc_block.modal.coord_select, block_coord_system)))
      {
        FAIL(STATUS_SETTING_READ_FAIL);
      }
    }
  }

  //[16.设置路径控制模式]：不适用。仅G61。G61.1和G64不受支持。
  //[17.设置距离模式]：不适用。仅G91.1.G90.1不受支持。
  //[18.设置缩回模式]：不支持。

  //[19.剩余非模态动作]：选中“转到预定义位置”、“设置G10”或“设置轴偏移”。
  // 注意：我们需要使用（G10/G28/G30/G92）来分离轴字的非模态命令，如下所示
  // 所有命令都以不同的方式处理轴字。G10作为绝对偏移或计算当前位置作为
  // 轴值G92类似于G10 L20，G28/30作为观察的中间目标位置
  // 所有当前坐标系和G92偏移。
  switch (gc_block.non_modal_command)
  {
  case NON_MODAL_SET_COORDINATE_DATA:
    //[G10错误]：缺少L且不是2或20。P字缺失。（完成负P值。）
    //[G10 L2错误]：不支持R字。
    // P值不为0到NCORDSYS（最大值为9）。缺少Axis单词。
    //[G10 L20错误]：P对于NCORDSYS必须为0（最大值为9）。缺少Axis单词。
    if (!axis_words)
    {
      FAIL(STATUS_GCODE_NO_AXIS_WORDS)
    }; // [No axis words]
    if (bit_isfalse(value_words, ((1 << WORD_P) | (1 << WORD_L))))
    {
      FAIL(STATUS_GCODE_VALUE_WORD_MISSING);
    }                                        //[P/L缺字]
    coord_select = trunc(gc_block.values.p); // 将p值转换为int。
    if (coord_select > N_COORDINATE_SYSTEM)
    {
      FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS);
    } //[大于N sys]
    if (gc_block.values.l != 20)
    {
      if (gc_block.values.l == 2)
      {
        if (bit_istrue(value_words, bit(WORD_R)))
        {
          FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        } //[G10 L2 R不受支持]
      }
      else
      {
        FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
      } //[无支持的L]
    }
    bit_false(value_words, (bit(WORD_L) | bit(WORD_P)));

    // 确定要更改的坐标系并尝试从EEPROM加载。
    if (coord_select > 0)
    {
      coord_select--;
    } // 将P1-P6索引调整为EEPROM坐标数据索引。
    else
    {
      coord_select = gc_block.modal.coord_select;
    } // 指数P0作为活动坐标系

    // 注意：将参数数据存储在IJK值中。根据规则，它们不与此命令一起使用。
    if (!settings_read_coord_data(coord_select, gc_block.values.ijk))
    {
      FAIL(STATUS_SETTING_READ_FAIL);
    } //[EEPROM读取失败]

    // 预先计算坐标数据更改。
    for (idx = 0; idx < N_AXIS; idx++)
    { // 轴索引是一致的，因此可以使用循环。
      // 更新仅在块中定义的轴。始终在机器坐标系中。可以更改非活动系统。
      if (bit_istrue(axis_words, bit(idx)))
      {
        if (gc_block.values.l == 20)
        {
          // L20：使用编程值更新当前位置的坐标系轴（带修改器）

          // WPos = MPos - WCS - G92 - TLO  ->  WCS = MPos - G92 - TLO - WPos
          gc_block.values.ijk[idx] = gc_state.position[idx] - gc_state.coord_offset[idx] - gc_block.values.xyz[idx];
          if (idx == TOOL_LENGTH_OFFSET_AXIS)
          {
            gc_block.values.ijk[idx] -= gc_state.tool_length_offset;
          }
        }
        else
        {
          // L2：将坐标系轴更新为编程值。
          gc_block.values.ijk[idx] = gc_block.values.xyz[idx];
        }
      } // 否则，保留当前存储的值。
    }
    break;
  case NON_MODAL_SET_COORDINATE_OFFSET:
    //[G92错误]：无轴字。
    if (!axis_words)
    {
      FAIL(STATUS_GCODE_NO_AXIS_WORDS);
    } //[无轴字]

    // 更新仅在块中定义的轴。将当前系统偏移到定义的值。
    // 选择活动坐标系时不更新，但仍处于活动状态，除非G92.1将其禁用。
    for (idx = 0; idx < N_AXIS; idx++)
    { // 轴索引是一致的，因此可以使用循环。
      if (bit_istrue(axis_words, bit(idx)))
      {
        // WPos = MPos - WCS - G92 - TLO  ->  G92 = MPos - WCS - TLO - WPos
        gc_block.values.xyz[idx] = gc_state.position[idx] - block_coord_system[idx] - gc_block.values.xyz[idx];
        if (idx == TOOL_LENGTH_OFFSET_AXIS)
        {
          gc_block.values.xyz[idx] -= gc_state.tool_length_offset;
        }
      }
      else
      {
        gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
      }
    }
    break;

  default:

    // 此时，其余的显式轴命令将轴值视为传统命令
    // 具有坐标系偏移、G92偏移、绝对替代和距离的目标位置
    // 应用的模式。这包括运动模式命令。我们现在可以预先计算目标位置。
    // 注：添加此功能时/如果添加此功能，可将刀具偏移附加到这些转换中。
    if (axis_command != AXIS_COMMAND_TOOL_LENGTH_OFFSET)
    { // TLO阻止任何轴命令。
      if (axis_words)
      {
        for (idx = 0; idx < N_AXIS; idx++)
        { // 轴索引是一致的，所以可以使用循环来节省闪存空间。
          if (bit_isfalse(axis_words, bit(idx)))
          {
            gc_block.values.xyz[idx] = gc_state.position[idx]; // 块中没有轴字。保持相同的轴位置。
          }
          else
          {
            // 根据距离模式更新指定值，如果绝对覆盖处于活动状态，则忽略。
            // 注：G28/30中的G53从未激活，因为它们位于同一模态组中。
            if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE)
            {
              // 基于距离模式应用坐标偏移。
              if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE)
              {
                gc_block.values.xyz[idx] += block_coord_system[idx] + gc_state.coord_offset[idx];
                if (idx == TOOL_LENGTH_OFFSET_AXIS)
                {
                  gc_block.values.xyz[idx] += gc_state.tool_length_offset;
                }
              }
              else
              { // 增量模式
                gc_block.values.xyz[idx] += gc_state.position[idx];
              }
            }
          }
        }
      }
    }

    // 检查其余非模态命令是否存在错误。
    switch (gc_block.non_modal_command)
    {
    case NON_MODAL_GO_HOME_0: // G28
    case NON_MODAL_GO_HOME_1: // G30
      //[G28/30错误]：刀具补偿已启用。
      // 从EEPROM中检索G28/30原点位置数据（机器坐标）
      // 注意：将参数数据存储在IJK值中。根据规则，它们不与此命令一起使用。
      if (gc_block.non_modal_command == NON_MODAL_GO_HOME_0)
      {
        if (!settings_read_coord_data(SETTING_INDEX_G28, gc_block.values.ijk))
        {
          FAIL(STATUS_SETTING_READ_FAIL);
        }
      }
      else
      { // == NON_MODAL_GO_HOME_1
        if (!settings_read_coord_data(SETTING_INDEX_G30, gc_block.values.ijk))
        {
          FAIL(STATUS_SETTING_READ_FAIL);
        }
      }
      if (axis_words)
      {
        // 仅移动辅助移动中指定的轴。
        for (idx = 0; idx < N_AXIS; idx++)
        {
          if (!(axis_words & (1 << idx)))
          {
            gc_block.values.ijk[idx] = gc_state.position[idx];
          }
        }
      }
      else
      {
        axis_command = AXIS_COMMAND_NONE; // 如果没有中间运动，则设置为“无”。
      }
      break;
    case NON_MODAL_SET_HOME_0: // G28.1
    case NON_MODAL_SET_HOME_1: // G30。1.
      //[G28.1/30.1错误]：刀具补偿已启用。
      // 注意：如果在此处传递轴单词，它们将被解释为隐式运动模式。
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET:
      // 注意：如果在此处传递轴单词，它们将被解释为隐式运动模式。
      break;
    case NON_MODAL_ABSOLUTE_OVERRIDE:
      //[G53错误]：G0和G1未激活。刀具补偿已启用。
      // 注意：所有显式轴字命令都在此模式组中。因此不需要隐式检查。
      if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR))
      {
        FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); //[G53 G0/1未激活]
      }
      break;
    }
  }

  //[20.运动模式]：
  if (gc_block.modal.motion == MOTION_MODE_NONE)
  {
    //[G80错误]：在G80激活时对轴字进行编程。
    // 注意：即使是使用轴字的非模态命令或TLO也会抛出此严格错误。
    if (axis_words)
    {
      FAIL(STATUS_GCODE_AXIS_WORDS_EXIST);
    } //[不允许使用轴字]

    // 如果轴字是隐式的（存在且G10/28/30/92未使用），则检查其余的运动模式，或
    // 在g代码块中被明确命令。
  }
  else if (axis_command == AXIS_COMMAND_MOTION_MODE)
  {

    if (gc_block.modal.motion == MOTION_MODE_SEEK)
    {
      //[G0错误]：未配置轴字母或没有实际值（完成）
      // 轴字是可选的。如果缺少，请将轴命令标志设置为忽略执行。
      if (!axis_words)
      {
        axis_command = AXIS_COMMAND_NONE;
      }

      // 所有其余运动模式（除G0和G80外）都需要有效的进给速率值。在单位/毫米模式下，
      // 该值必须为正。在反时限模式下，每个块必须传递一个正值。
    }
    else
    {
      // 检查是否为需要进给速度的运动模式定义了进给速度。
      if (gc_block.values.f == 0.0)
      {
        FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE);
      } //[进给速度未定义]

      switch (gc_block.modal.motion)
      {
      case MOTION_MODE_LINEAR:
        //[G1错误]：未定义进给速度。未配置轴字母或没有实际值。
        // 轴字是可选的。如果缺少，请将axis命令标志设置为忽略执行。
        if (!axis_words)
        {
          axis_command = AXIS_COMMAND_NONE;
        }
        break;
      case MOTION_MODE_CW_ARC:
        gc_parser_flags |= GC_PARSER_ARC_IS_CLOCKWISE; // 按意思继续下一个
      case MOTION_MODE_CCW_ARC:
        //[G2/3错误所有模式]：进给速度未定义。

        //[G2/3半径模式错误]：所选平面中没有轴字。目标点与当前点相同。

        //[G2/3偏移模式错误]：所选平面中没有轴字和/或偏移。到当前点的半径和到目标点的半径相差超过0.002mm（EMC def.0.5mm或0.005mm和0.1%半径）。
        //[G2/3整圈模式错误]：不支持。轴心词存在。未编程任何偏移。P必须是整数。
        // 注：圆弧跟踪需要半径和偏移，并通过错误检查预先计算。

        if (!axis_words)
        {
          FAIL(STATUS_GCODE_NO_AXIS_WORDS);
        } //[无轴字]
        if (!(axis_words & (bit(axis_0) | bit(axis_1))))
        {
          FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE);
        } //[平面中无轴字]

        // 计算沿每个选定轴的位置变化
        float x, y;
        x = gc_block.values.xyz[axis_0] - gc_state.position[axis_0]; // 当前位置和目标之间的增量x
        y = gc_block.values.xyz[axis_1] - gc_state.position[axis_1]; // 当前位置和目标之间的增量y

        if (value_words & bit(WORD_R))
        { // 弧半径模式
          bit_false(value_words, bit(WORD_R));
          if (isequal_position_vector(gc_state.position, gc_block.values.xyz))
          {
            FAIL(STATUS_GCODE_INVALID_TARGET);
          } //[无效目标]

          // 将半径值转换为适当的单位。
          if (gc_block.modal.units == UNITS_MODE_INCHES)
          {
            gc_block.values.r *= MM_PER_INCH;
          }
          /*  我们需要计算具有指定半径并通过当前位置和目标位置的圆心。
              该方法计算以下方程组，其中[x，y]是从当前位置到目标位置的矢量，d==该矢量的大小，h==由圆半径形成的三角形的斜边，即到移动矢量中心的距离。
              垂直于移动向量[-y，x]的向量被缩放为h[-y/d*h，x/d*h]的长度，并添加到移动向量[x/2，y/2]的中心，以形成[x/2-y/d*h，y/2+x/d*h]处的新点[i，j]，该点将是弧的中心。
              d^2 == x^2 + y^2
              h^2 == r^2 - (d/2)^2
              i == x/2 - y/d*h
              j == y/2 + x/d*h

                                                                   O <- [i,j]
                                                                -  |
                                                      r      -     |
                                                          -        |
                                                       -           | h
                                                    -              |
                                      [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                | <------ d/2 ---->|

              C - 当前位置
              T - 目标位置
              O - 通过C和T的圆心
              d - 从C到T的距离
              r - 指定半径
              h - CT中心到O的距离

              展开方程式：

              d -> sqrt(x^2 + y^2)
              h -> sqrt(4 * r^2 - x^2 - y^2)/2
              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

              可以这样写：

              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

              由于尺寸和速度的原因，我们对其进行了优化：

              h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
              i = (x - (y * h_x2_div_d))/2
              j = (y + (x * h_x2_div_d))/2
          */

          // 首先，使用h_x2_div_d来计算4*h^2，以检查它是负值还是r更小
          // 如果是这样的话，负数的sqrt是复杂的并且是错误的。
          float h_x2_div_d = 4.0 * gc_block.values.r * gc_block.values.r - x * x - y * y;

          if (h_x2_div_d < 0)
          {
            FAIL(STATUS_GCODE_ARC_RADIUS_ERROR);
          } // [Arc radius error]

          // 完成h_x2_div_d的计算。
          h_x2_div_d = -sqrt(h_x2_div_d) / hypot_f(x, y); //==-（高*2/d）
                                                          // 如果圆圈为逆时针方向，则反转h_x2_div_d的符号（见下图）
          if (gc_block.modal.motion == MOTION_MODE_CCW_ARC)
          {
            h_x2_div_d = -h_x2_div_d;
          }

          /* 逆时针圆圈位于目标方向的左侧。当偏移为正时，将生成左手圆-当偏移为负时，将生成右手圆。
                                                                 T  <-- 目标位置

                                                                 ^
                        以该中心顺时针旋转的圆的角行程大于180度    |          以该中心顺时针旋转的圆的角行程小于180度，这是一件好事！
                                                                 |
                                                       \         |          /
                            h_x2_div_d为正时的弧中心 ->  x <----- | -----> x <- h_x2_div_d为负时的弧中心
                                                                 |
                                                                 |

                                                                 C  <-- 当前位置
          */
          // 负R表示“我想要一个行程超过180度的圆”（见图！），即使建议不要在一行g代码中生成这样的圆圈。
          // 通过反转h_x2_div_d的符号，圆的中心位于行程线的另一侧，因此我们得到了规定的不合适的长弧。
          if (gc_block.values.r < 0)
          {
            h_x2_div_d = -h_x2_div_d;
            gc_block.values.r = -gc_block.values.r; // 完成后，r设置为正，用于mc_arc
          }
          // 通过计算圆弧的实际中心完成操作
          gc_block.values.ijk[axis_0] = 0.5 * (x - (y * h_x2_div_d));
          gc_block.values.ijk[axis_1] = 0.5 * (y + (x * h_x2_div_d));
        }
        else
        { // 弧心格式偏移模式
          if (!(ijk_words & (bit(axis_0) | bit(axis_1))))
          {
            FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE);
          } //[平面内无偏移]
          bit_false(value_words, (bit(WORD_I) | bit(WORD_J) | bit(WORD_K)));

          // 将IJK值转换为适当的单位。
          if (gc_block.modal.units == UNITS_MODE_INCHES)
          {
            for (idx = 0; idx < N_AXIS; idx++)
            { // 轴索引是一致的，所以可以使用循环来节省闪存空间。
              if (ijk_words & bit(idx))
              {
                gc_block.values.ijk[idx] *= MM_PER_INCH;
              }
            }
          }

          // 从中心到目标的弧半径
          x -= gc_block.values.ijk[axis_0]; // 圆心和目标之间的增量x
          y -= gc_block.values.ijk[axis_1]; // 圆心和目标之间的增量y
          float target_r = hypot_f(x, y);

          // 计算mc_arc的圆弧半径。从当前位置到中心定义。
          gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]);

          // 计算当前位置和目标半径之间的差值，以进行最终错误检查。
          float delta_r = fabs(target_r - gc_block.values.r);
          if (delta_r > 0.005)
          {
            if (delta_r > 0.5)
            {
              FAIL(STATUS_GCODE_INVALID_TARGET);
            } //[电弧定义误差]>0.5mm
            if (delta_r > (0.001 * gc_block.values.r))
            {
              FAIL(STATUS_GCODE_INVALID_TARGET);
            } //[电弧定义误差]>0.005mm和0.1%半径
          }
        }
        break;
      case MOTION_MODE_PROBE_TOWARD_NO_ERROR:
      case MOTION_MODE_PROBE_AWAY_NO_ERROR:
        gc_parser_flags |= GC_PARSER_PROBE_IS_NO_ERROR; // 继续
      case MOTION_MODE_PROBE_TOWARD:
      case MOTION_MODE_PROBE_AWAY:
        if ((gc_block.modal.motion == MOTION_MODE_PROBE_AWAY) ||
            (gc_block.modal.motion == MOTION_MODE_PROBE_AWAY_NO_ERROR))
        {
          gc_parser_flags |= GC_PARSER_PROBE_IS_AWAY;
        }
        //[G38错误]：目标与当前相同。没有轴字。刀具补偿已启用。
        // 进给速度未定义。探针被触发。
        // 注：探头检查已移至探头循环。
        // 它不会返回错误，而是发出警报以防止探针进一步移动。
        // 它还允许规划器缓冲区清空并在另一个探测周期之前移出探测触发器。
        if (!axis_words)
        {
          FAIL(STATUS_GCODE_NO_AXIS_WORDS);
        } // [No axis words]
        if (isequal_position_vector(gc_state.position, gc_block.values.xyz))
        {
          FAIL(STATUS_GCODE_INVALID_TARGET);
        } // [Invalid target]
        break;
      }
    }
  }

  //[21.程序流程]：无需进行错误检查。

  //[0.非特定错误检查]：完成未使用的值字检查，即在圆弧半径模式下使用的IJK，或块中未使用的轴字。
  if (gc_parser_flags & GC_PARSER_JOG_MOTION)
  {
    // 点动仅使用F进给率和XYZ值字。N有效，但S和T无效。
    bit_false(value_words, (bit(WORD_N) | bit(WORD_F)));
  }
  else
  {
    bit_false(value_words, (bit(WORD_N) | bit(WORD_F) | bit(WORD_S) | bit(WORD_T))); // 删除单义词。
  }
  if (axis_command)
  {
    bit_false(value_words, (bit(WORD_X) | bit(WORD_Y) | bit(WORD_Z)));
  } // 删除轴字。
  if (value_words)
  {
    FAIL(STATUS_GCODE_UNUSED_WORDS);
  } //[未使用的单词]

  /*-------------------------------------------------------------------------------------
第四步：执行！！
假设已完成所有错误检查，且不存在任何故障模式。我们只是需要更新状态并根据执行顺序执行块。
*/

  // 初始化运动块的规划器数据结构。
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data, 0, sizeof(plan_line_data_t)); // 零pl_data结构

  // 截取jog命令并完成有效jog命令的错误检查，然后执行。

  // 注：G代码解析器状态未更新，但确保正确计算顺序jog目标的位置除外。
  // 当点动完成或取消时，在protocol_execute_realtime（）中更新点动后的最终解析器位置。
  if (gc_parser_flags & GC_PARSER_JOG_MOTION)
  {
    // 仅允许使用距离和单位模式命令以及G53绝对覆盖命令。
    // 注意：在步骤3中已执行进给率字和轴字检查。
    if (command_words & ~(bit(MODAL_GROUP_G3) | bit(MODAL_GROUP_G6) | bit(MODAL_GROUP_G0)))
    {
      FAIL(STATUS_INVALID_JOG_COMMAND)
    };
    if (!(gc_block.non_modal_command == NON_MODAL_ABSOLUTE_OVERRIDE || gc_block.non_modal_command == NON_MODAL_NO_ACTION))
    {
      FAIL(STATUS_INVALID_JOG_COMMAND);
    }

    // 将规划器数据初始化为当前主轴和冷却液模式状态。
    pl_data->spindle_speed = gc_state.spindle_speed;
    plan_data.condition = (gc_state.modal.spindle | gc_state.modal.coolant);

    uint8_t status = jog_execute(&plan_data, &gc_block);
    if (status == STATUS_OK)
    {
      memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz));
    }
    return (status);
  }

  // 如果处于激光模式，则根据当前和过去的条件设置激光功率。
  if (bit_istrue(settings.flags, BITFLAG_LASER_MODE))
  {
    if (!((gc_block.modal.motion == MOTION_MODE_LINEAR) || (gc_block.modal.motion == MOTION_MODE_CW_ARC) || (gc_block.modal.motion == MOTION_MODE_CCW_ARC)))
    {
      gc_parser_flags |= GC_PARSER_LASER_DISABLE;
    }

    // 允许从主轴速度更新传递任何带有轴字的运动模式。
    // 注：不带轴字的G1和G0将轴_命令设置为无。故意省略G28/30。
    // TODO:检查未进入规划器的启用M3的运动的同步条件。（零长度）。
    if (axis_words && (axis_command == AXIS_COMMAND_MOTION_MODE))
    {
      gc_parser_flags |= GC_PARSER_LASER_ISMOTION;
    }
    else
    {
      // M3恒功率激光器需要规划器同步在G1/2/3运动模式状态之间切换时更新激光器，反之亦然，当线路中没有运动时。
      if (gc_state.modal.spindle == SPINDLE_ENABLE_CW)
      {
        if ((gc_state.modal.motion == MOTION_MODE_LINEAR) || (gc_state.modal.motion == MOTION_MODE_CW_ARC) || (gc_state.modal.motion == MOTION_MODE_CCW_ARC))
        {
          if (bit_istrue(gc_parser_flags, GC_PARSER_LASER_DISABLE))
          {
            gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC; // 从G1/2/3运动模式更改。
          }
        }
        else
        {
          // 从非G1/2/3运动模式更改为不带轴字的G1运动模式时。
          if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE))
          {
            gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC;
          }
        }
      }
    }
  }

  //[0.非特定/常见错误检查和其他设置]：
  // 注意：如果没有行号，则该值为零。
  gc_state.line_number = gc_block.values.n;
#ifdef USE_LINE_NUMBERS
  pl_data->line_number = gc_state.line_number; // 记录数据供规划器使用。
#endif

  //[1.注释反馈]：不支持

  //[2.设置进给速度模式]：
  gc_state.modal.feed_rate = gc_block.modal.feed_rate;
  if (gc_state.modal.feed_rate)
  {
    pl_data->condition |= PL_COND_FLAG_INVERSE_TIME;
  } // 设置用于规划器使用的条件标志。

  //[3.设置进给速度]：
  gc_state.feed_rate = gc_block.values.f;  // 始终复制此值。请参阅进给率错误检查。
  pl_data->feed_rate = gc_state.feed_rate; // 记录数据供规划器使用。

  //[4.设置主轴转速]：
  if ((gc_state.spindle_speed != gc_block.values.s) || bit_istrue(gc_parser_flags, GC_PARSER_LASER_FORCE_SYNC))
  {
    if (gc_state.modal.spindle != SPINDLE_DISABLE)
    {
#ifdef VARIABLE_SPINDLE
      if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_ISMOTION))
      {
        if (bit_istrue(gc_parser_flags, GC_PARSER_LASER_DISABLE))
        {
          spindle_sync(gc_state.modal.spindle, 0.0);
        }
        else
        {
          spindle_sync(gc_state.modal.spindle, gc_block.values.s);
        }
      }
#else
      spindle_sync(gc_state.modal.spindle, 0.0);
#endif
    }
    gc_state.spindle_speed = gc_block.values.s; // 更新主轴速度状态。
  }
  // 注：所有受限激光运动均通过零主轴转速。
  if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE))
  {
    pl_data->spindle_speed = gc_state.spindle_speed; // 记录数据供计划器使用。
  } // else { pl_data->spindle_speed = 0.0; } // 已初始化为零。

  //[5.选择工具]：不支持。仅跟踪工具值。
  gc_state.tool = gc_block.values.t;

  //[6.更改工具]：不支持

  //[7.主轴控制]：
  if (gc_state.modal.spindle != gc_block.modal.spindle)
  {
    // 在此块中启用时，更新主轴控制并应用主轴速度。
    // 注意：即使在激光模式下，所有主轴状态变化也会同步。此外，pl_data（而非gc_state）用于管理非激光运动的激光状态。
    spindle_sync(gc_block.modal.spindle, pl_data->spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;
  }
  pl_data->condition |= gc_state.modal.spindle; // 设置用于计划器使用的条件标志。

  //[8.冷却液控制]：
  if (gc_state.modal.coolant != gc_block.modal.coolant)
  {
    // 注意：冷却液M代码是模态代码。每行仅允许一条命令。
    // 但是，多个状态可以同时存在，而“冷却液禁用”将清除所有状态。
    coolant_sync(gc_block.modal.coolant);
    gc_state.modal.coolant = gc_block.modal.coolant;
  }
  pl_data->condition |= gc_state.modal.coolant; // 设置用于计划器使用的条件标志。

//[9.覆盖控制]：不支持。始终启用。除了仅限Grbl的停靠控制。
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
  if (gc_state.modal.override != gc_block.modal.override)
  {
    gc_state.modal.override = gc_block.modal.override;
    mc_override_ctrl_update(gc_state.modal.override);
  }
#endif

  // [10. Dwell ]:
  if (gc_block.non_modal_command == NON_MODAL_DWELL)
  {
    mc_dwell(gc_block.values.p);
  }

  //[11.设置活动平面]：
  gc_state.modal.plane_select = gc_block.modal.plane_select;

  //[12.设置长度单位]：
  gc_state.modal.units = gc_block.modal.units;

  //[13.刀具半径补偿]：不支持G41/42
  // 乔治亚州。情态动词切刀组件=gc块。情态动词切割机组件；//注意：由于始终禁用，因此不需要。

  //[14.刀具长度补偿]：G43。1和G49支持。不支持G43。
  // 注意：如果支持G43，其操作与G43没有任何不同。1在执行方面。
  // 错误检查步骤只是将偏移值加载到块XYZ值数组的正确轴中。
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET)
  { // 表示更改。
    gc_state.modal.tool_length = gc_block.modal.tool_length;
    if (gc_state.modal.tool_length == TOOL_LENGTH_OFFSET_CANCEL)
    { // G49
      gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS] = 0.0;
    } // else G43.1
    if (gc_state.tool_length_offset != gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS])
    {
      gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
      system_flag_wco_change();
    }
  }

  // [15. 坐标系选择 ]:
  if (gc_state.modal.coord_select != gc_block.modal.coord_select)
  {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system, block_coord_system, N_AXIS * sizeof(float));
    system_flag_wco_change();
  }

  // [16. 设置路径控制模式 ]: G61.1/G64 不支持

  // gc_state.modal.control = gc_block.modal.control; // NOTE: Always default.

  //[17.设置距离模式]：
  gc_state.modal.distance = gc_block.modal.distance;

  //[18.设置缩回模式]：不支持

  //[19.转到预定义位置，设置G10或设置轴偏移]：
  switch (gc_block.non_modal_command)
  {
  case NON_MODAL_SET_COORDINATE_DATA:
    settings_write_coord_data(coord_select, gc_block.values.ijk);
    // 如果当前处于活动状态，则更新系统坐标系。
    if (gc_state.modal.coord_select == coord_select)
    {
      memcpy(gc_state.coord_system, gc_block.values.ijk, N_AXIS * sizeof(float));
      system_flag_wco_change();
    }
    break;
  case NON_MODAL_GO_HOME_0:
  case NON_MODAL_GO_HOME_1:
    // 在归位前移动到中间位置。遵循当前坐标系和偏移以及绝对和增量模式。
    pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // 设置快速运动条件标志。
    if (axis_command)
    {
      mc_line(gc_block.values.xyz, pl_data);
    }
    mc_line(gc_block.values.ijk, pl_data);
    memcpy(gc_state.position, gc_block.values.ijk, N_AXIS * sizeof(float));
    break;
  case NON_MODAL_SET_HOME_0:
    settings_write_coord_data(SETTING_INDEX_G28, gc_state.position);
    break;
  case NON_MODAL_SET_HOME_1:
    settings_write_coord_data(SETTING_INDEX_G30, gc_state.position);
    break;
  case NON_MODAL_SET_COORDINATE_OFFSET:
    memcpy(gc_state.coord_offset, gc_block.values.xyz, sizeof(gc_block.values.xyz));
    system_flag_wco_change();
    break;
  case NON_MODAL_RESET_COORDINATE_OFFSET:
    clear_vector(gc_state.coord_offset); // 通过将偏移向量归零来禁用G92偏移。
    system_flag_wco_change();
    break;
  }

  //[20.运动模式]：
  // 注意：命令G10、G28、G30、G92锁定并防止轴字在运动模式中使用。
  // 仅当块中存在轴字或运动模式命令字时，才输入运动模式。
  gc_state.modal.motion = gc_block.modal.motion;
  if (gc_state.modal.motion != MOTION_MODE_NONE)
  {
    if (axis_command == AXIS_COMMAND_MOTION_MODE)
    {
      uint8_t gc_update_pos = GC_UPDATE_POS_TARGET;
      if (gc_state.modal.motion == MOTION_MODE_LINEAR)
      {
        mc_line(gc_block.values.xyz, pl_data);
      }
      else if (gc_state.modal.motion == MOTION_MODE_SEEK)
      {
        pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // 设置快速运动条件标志。
        mc_line(gc_block.values.xyz, pl_data);
      }
      else if ((gc_state.modal.motion == MOTION_MODE_CW_ARC) || (gc_state.modal.motion == MOTION_MODE_CCW_ARC))
      {
        mc_arc(gc_block.values.xyz, pl_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
               axis_0, axis_1, axis_linear, bit_istrue(gc_parser_flags, GC_PARSER_ARC_IS_CLOCKWISE));
      }
      else
      {
// 注：gc_块。价值观xyz从mc_probe_循环返回，并带有更新的位置值。
// 所以成功探测循环后，机器位置和返回值应相同。
#ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
        pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
#endif
        gc_update_pos = mc_probe_cycle(gc_block.values.xyz, pl_data, gc_parser_flags);
      }

      // 就解析器而言，位置现在是==target。实际上
      // 运动控制系统可能仍在处理动作和实际刀具位置
      // 在任何中间位置。
      if (gc_update_pos == GC_UPDATE_POS_TARGET)
      {
        memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
      }
      else if (gc_update_pos == GC_UPDATE_POS_SYSTEM)
      {
        gc_sync_position(); // gc_state.position[] = sys_position
      } // == GC_UPDATE_POS_NONE
    }
  }

  //[21.程序流程]：
  // M0、M1、M2、M30：执行非运行程序流动作。
  // 在程序暂停期间，缓冲区可能会重新填充，并且只能通过循环开始运行时命令恢复。
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow)
  {
    protocol_buffer_synchronize(); // 继续之前，同步并完成所有剩余的缓冲运动。
    if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED)
    {
      if (sys.state != STATE_CHECK_MODE)
      {
        system_set_exec_state_flag(EXEC_FEED_HOLD); // 使用进给保持暂停程序。
        protocol_execute_realtime();                // 执行挂起。
      }
    }
    else
    { //==程序流已完成

      // 根据LinuxCNC的程序结束描述和测试，程序完成后，只有一部分g代码重置为某些默认值。
      // 仅模态组[G代码1,2,3,5,7,12]和[M代码7,8,9]重置为[G1、G17、G90、G94、G40、G54、M5、M9、M48]。
      // 其余的模态组[G-代码4,6,8,10,13,14,15]和[M-代码4,5,6]以及模态词[F，S，T，H]不会重置。
      gc_state.modal.motion = MOTION_MODE_LINEAR;
      gc_state.modal.plane_select = PLANE_SELECT_XY;
      gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
      gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
      // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // Not supported.
      gc_state.modal.coord_select = 0; // G54
      gc_state.modal.spindle = SPINDLE_DISABLE;
      gc_state.modal.coolant = COOLANT_DISABLE;
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
#ifdef DEACTIVATE_PARKING_UPON_INIT
      gc_state.modal.override = OVERRIDE_DISABLED;
#else
      gc_state.modal.override = OVERRIDE_PARKING_MOTION;
#endif
#endif

#ifdef RESTORE_OVERRIDES_AFTER_PROGRAM_END
      sys.f_override = DEFAULT_FEED_OVERRIDE;
      sys.r_override = DEFAULT_RAPID_OVERRIDE;
      sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE;
#endif

      // 执行坐标变换和主轴/冷却液停止。
      if (sys.state != STATE_CHECK_MODE)
      {
        if (!(settings_read_coord_data(gc_state.modal.coord_select, gc_state.coord_system)))
        {
          FAIL(STATUS_SETTING_READ_FAIL);
        }
        system_flag_wco_change(); // 设置为立即刷新，以防发生更改。
        spindle_set_state(SPINDLE_DISABLE, 0.0);
        coolant_set_state(COOLANT_DISABLE);
      }
      report_feedback_message(MESSAGE_PROGRAM_END);
    }
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // 重置程序流。
  }

  // TODO:%s表示程序开始。

  return (STATUS_OK);
}

/*
  不支持：

  - 封闭圆
  - 刀具半径补偿
  - A,B,C-轴
  - 表达式的求值
  - 变量
  - 覆盖控制 (TBD)
  - 换刀
  - 开关

   （*）表示可选参数，通过配置启用。重新编译
组0={G92.2，G92.3}（非模态：取消并重新启用G92偏移）
第1组={G81-G89}（运动模式：罐装循环）
组4={M1}（可选停止，忽略）
第6组={M6}（换刀）
组7={G41，G42}刀具半径补偿（支持G40）
组8={G43}刀具长度偏移（支持G43.1/G49）
组8={M7*}启用喷雾冷却液（*编译选项）
组9={M48、M49、M56*}启用/禁用覆盖开关（*编译选项）
组10={G98，G99}返回模式屏蔽循环
组13={G61.1，G64}路径控制模式（支持G61）
*/
