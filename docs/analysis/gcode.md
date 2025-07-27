# G代码解析器

## G代码

  G代码（G-code）是一种用于控制数控(CNC) 机器的标准编程语言，特别是在计算机辅助制造中。它用于指导机器执行各种操作，如移动、钻孔、切割等。它有多个版本，grbl参考[RS274/NGC第3版](/docs/RS274NGC_3.pdf)实现了一个G代码解释器。

## G代码的模态

G-code 模态（Modal）是指G-code 命令的“状态”或“模式”。在G-code 中，某些命令会保持激活状态，直到被其他命令显式地更改或取消，这些保持激活状态的命令就被称为模态命令。简单来说，模态命令就像是设置了一个开关，一旦打开，除非你关闭它，否则它会一直保持这个状态，影响后续的G-code 执行。

### 模态命令：

激活后会一直保持状态，直到被其他模态命令覆盖或取消。例如，G01 (直线插补) 激活后，除非使用G00 (快速移动) 或其他移动指令来取消，否则后续的移动指令都会被认为是直线插补。

### 非模态命令：

每次执行都会生效，且只对紧随其后的运动有效。例如，F (进给率) 它只影响下一个移动命令，而不是所有后续移动。

### 模态组：

G-code 命令被分成了几个模态组，每个组内的命令是互斥的，即同一时刻只能有一个组内的命令处于激活状态。常见的模态组包括：  

- 组0={G92.2，G92.3}（非模态：取消并重新启用G92偏移）
- 第1组={G81-G89}（运动模式：罐装循环）
- 组4={M1}（可选停止，忽略）
- 第6组={M6}（换刀）
- 组7={G41，G42}刀具半径补偿（支持G40）
- 组8={G43}刀具长度偏移（支持G43.1/G49）
- 组8={M7*}启用喷雾冷却液（*编译选项）
- 组9={M48、M49、M56*}启用/禁用覆盖开关（*编译选项）
- 组10={G98，G99}返回模式屏蔽循环
- 组13={G61.1，G64}路径控制模式（支持G61）

grbl不支持的：  

- 封闭圆
- 刀具半径补偿
- A,B,C-轴
- 表达式的求值
- 变量
- 覆盖控制 (TBD)
- 换刀
- 开关

这些模态组都在grbl的`gcode.h`中定义，代码如下：  

```c
// gcode.h
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
```

## 解释器（interpretor）

解析器这部分都代码比较长，也比较复杂，初看很难理解，但我们可以把它当作一门脚本语言（实际上就是）来理解，就清楚多了，因为脚本语言有标准化的解释方法及步骤。
实现一个解析器的主要步骤包括：**词法分析、语法分析和语义分析**。词法分析将源代码分解成词法单元（tokens），语法分析验证这些词法单元是否符合语法规则，语义分析则检查代码的含义和正确性。

具体步骤如下：

1. **词法分析(Lexical Analysis/Scanning):**  

导入块行中的所有g代码字。g代码字是一个字母后跟一个数字，可以是“G”/“M”命令，也可以设置/分配命令值。
而且对任何重复的命令字模式组冲突执行初始错误检查，代码简化为：

```c
// gcode.c

// 执行一行以0结尾的G代码。
// 假定该行仅包含大写字符和有符号浮点值（无空格）。
// 注释和块删除字符已被删除。
// 在该函数中，所有单位和位置分别以（毫米，毫米/分钟）和绝对机器坐标转换并导出为grbl的内部函数。
uint8_t gc_execute_line(char *line)
{
    uint8_t char_counter;
    char letter;
    float value;
    uint8_t int_value = 0;
    uint8_t mantissa = 0;
    while(line[char_counter] !=0) {
        letter = line[char_counter];
        char_counter++;
        read_float(line, &char_counter, &value);
        int_value = trunc(value);
        mantissa = round(100*(value - int_value));

        switch(letter) {
            case "G":
                break;
            case "M":
                break;
            default:
                // 非命令字，检查剩余其他合法的G代码字（F,I,J,K,L,N,P,R,S,T,X,Y,Z），并为其赋值。
        }
    }
}
```

通过一个while循环，不断地从字符串中分离出词（word），例如`M3`、`G1` 、`X11.111` 等。然后把它们分为`G`命令、`M`命令和其他合法G代码。此时，g代码块已被解析，可以释放行缓冲区。

2. **语法分析(Syntax Analysis/Parsing):**

grbl不支持变量和表达式求值，因此这部分并不复杂。
根据G代码的定义，将词法单元序列构建成对应的语法结构`gc_block`，如果不符合语法规则则报告语法错误。如`M`命令后没有小数部分。  

- `gc_block.non_modal_command`非模态命令。  
- `gc_block.modal`模态命令的一些属性运动模式、进给率、单位选择、距离模式、平面选择、坐标系选择主轴、冷却、覆盖。  
- `gc_block.values`非命令的字参数值：进给、圆弧偏移、暂停、弧半径、主轴、转速、xyz轴值。

3. **语义分析(Semantic Analysis):**  

错误检查此块中传递的所有命令和值。这一步确保了所有命令执行有效，并尽可能遵循NIST标准。
如果发现错误，将转储此块中的所有命令和值，并且不会更新激活的系统g代码模式。
如果模块正常，则激活的系统g代码模式将被激活根据此块的命令进行更新，并发出执行信号。

此外，我们还必须根据被解析对象设置的模式对传递的所有值进行预转换块 。
有许多错误检查需要目标信息，只有在我们结合错误检查转换这些值时才能准确计算这些信息。  

0. 非特定/常见错误检查和其他设置
1. 注释，不支持MSG。由协议执行的注释处理。
2. 设置进给速度模式，G1、G2/3激活，隐式或显式
3. 设定进给速度
4. 设置主轴转速
5. 选择工具，不支持
6. 更换工具，不适用
7. 主轴控制，不适用
8. 冷却液控制：不适用
9. 覆盖控制：不受支持，仅Grbl停靠运动覆盖控制除外。
10. 暂停：缺少P值。P为负
11. 设置活动平面：不适用
13. 刀具半径补偿：不支持G41/42。如果在G53激活时启用则报错。
14. 刀具长度补偿：不支持G43，但支持G43.1和G49支持。
15. 坐标系选择：不适用。如果刀具半径补偿激活则报错。
16. 设置路径控制模式：不适用。仅G61。G61.1和G64不受支持。
17. 设置距离模式：不适用。仅G91.1.G90.1不受支持。
18. 设置缩回模式：不支持。
19. 剩余非模态动作：选中“转到预定义位置”、“设置G10”或“设置轴偏移”。
20. 运动模式
21. 程序流程：无需进行错误检查。

经过上述步骤检查和预转换，已经排除了错误，运动所需信息已准备好，下一执行步骤降级为仅更新系统g代码模式并按顺序执行编程动作。

4. **代码执行(Code Execute) (可选):**

如果需要将源代码编译成其他语言或机器代码，则需要进行代码生成。grbl是解释性语言，这一步不会生成目标代码，而是直接执行。
执行步骤不需要任何转换计算，只需要执行所需的最少检查。
假设已完成所有错误检查，且不存在任何故障模式。我们只是需要更新状态并根据执行顺序执行块。
执行主要有以下几部分：

- `jog_execute()`：执行点动动作
- `spindle_sync`： 执行更新主轴转速
- `coolant_sync`： 执行冷却控制
- `mc_dwell`：     执行暂停
- `mc_line`：      执行直线运动
- `mc_arc`：       执行圆弧运动
- `mc_probe_cycle`：执行探针运动

## 总结

grbl通过`gc_execute_line()` 从行缓冲区`line[]`取出G代码并进行解析，解析完成后调用其他模块执行对应的动作，例如主轴和轴运动（mc_xxx）。
