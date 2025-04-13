/*
  config.h - 编译时配置
  Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

//此文件包含Grbl内部系统的编译时配置。
//在大多数情况下，用户不需要直接修改它们，但它们是为了满足特定的需要，即性能调整或适应非典型机器。

//重要提示：此处的任何更改都需要重新编译源代码以更新它们。

#ifndef config_h
#define config_h
#include "grbl.h" // 兼容 Arduino IDE .


//定义CPU引脚映射和默认设置。
//注意：OEM可以避免维护/更新defaults.h和cpu_map.h文件，只使用一个配置文件，方法是将其特定的默认值和引脚映射放置在此文件底部。
//如果这样做，只需注释掉这两个定义，并参见下面的说明。
#define DEFAULTS_GENERIC
#define CPU_MAP_ATMEGA328P//阿杜伊诺一个CPU

//串行波特率
// #define BAUD_RATE 230400
#define BAUD_RATE 115200

//定义实时命令特殊字符。这些字符不会直接从grbl数据流中读取到grbl执行。选择流式g代码程序中不存在且不得存在的字符。
//如果每个用户设置都有ASCII控制字符，则可以使用ASCII控制字符。
//此外，扩展ASCII码（>127）永远不会出现在g代码程序中，可以选择用于接口程序。
//注意：如果更改，请手动更新报告中的帮助消息。C

#define CMD_RESET 0x18 // ctrl-x.
#define CMD_STATUS_REPORT '?'
#define CMD_CYCLE_START '~'
#define CMD_FEED_HOLD '!'

// 注意：所有覆盖实时命令必须在扩展ASCII字符集中，从字符值128（0x80）开始，最多255（0xFF）。
//如果将正常的实时命令集（如状态报告、馈电保持、复位和循环启动）移动到扩展设置空间，则需要修改serial.c的RX ISR以适应更改。
// #define CMD_RESET 0x80
// #define CMD_STATUS_REPORT 0x81
// #define CMD_CYCLE_START 0x82
// #define CMD_FEED_HOLD 0x83
#define CMD_SAFETY_DOOR 0x84
#define CMD_JOG_CANCEL  0x85
#define CMD_DEBUG_REPORT 0x86//仅当启用调试时，才会以“{}”大括号发送调试报告。
#define CMD_FEED_OVR_RESET 0x90//将进给覆盖值恢复为100%。
#define CMD_FEED_OVR_COARSE_PLUS 0x91
#define CMD_FEED_OVR_COARSE_MINUS 0x92
#define CMD_FEED_OVR_FINE_PLUS  0x93
#define CMD_FEED_OVR_FINE_MINUS  0x94
#define CMD_RAPID_OVR_RESET 0x95//将快速覆盖值恢复为100%。
#define CMD_RAPID_OVR_MEDIUM 0x96
#define CMD_RAPID_OVR_LOW 0x97
// #define CMD_RAPID_OVR_EXTRA_LOW 0x98 // *NOT SUPPORTED*
#define CMD_SPINDLE_OVR_RESET 0x99//将主轴覆盖值恢复为100%。
#define CMD_SPINDLE_OVR_COARSE_PLUS 0x9A
#define CMD_SPINDLE_OVR_COARSE_MINUS 0x9B
#define CMD_SPINDLE_OVR_FINE_PLUS 0x9C
#define CMD_SPINDLE_OVR_FINE_MINUS 0x9D
#define CMD_SPINDLE_OVR_STOP 0x9E
#define CMD_COOLANT_FLOOD_OVR_TOGGLE 0xA0
#define CMD_COOLANT_MIST_OVR_TOGGLE 0xA1

//如果启用了归零，则归零初始锁定会在通电时将Grbl设置为报警状态。
//这将强制用户在执行任何其他操作之前执行重新定位循环（或覆盖锁）。
//这主要是提醒用户归位的安全功能，因为Grbl不知道位置。
#define HOMING_INIT_LOCK // 注释后禁用

// 使用位掩码定义归位循环模式。 
//归位循环首先执行搜索模式以快速接合限位开关，然后执行较慢的定位模式，最后通过短距离回拉动作来断开限位开关。
//以下归零循环定义以后缀0开始的顺序执行，并仅完成指定轴的归零例程。
//如果定义中省略了轴，则该轴将不在原点，系统也不会更新其位置。
//这意味着，这允许使用非标准笛卡尔机器的用户，例如车床（x然后z，没有y）根据自己的需要配置回零循环行为。
//注意：如果轴不在同一周期内，则回零周期设计为允许共享限位引脚，但这需要在cpu_map.h中更改一些引脚设置。
//例如，默认归零循环可以与X或Y限位引脚共享Z限限位引脚，因为它们处于不同的周期。
//这个引脚为了其他目的腾出了一个宝贵的IO。
//理论上，如果所有轴都以单独的周期进行定位，则所有轴限位销可减少为一个引脚，
//反之亦然，所有三个轴均位于单独的引脚上，但在一个周期内进行定位。 
//此外，还应注意，硬限位的功能不会受到管脚共享的影响。
// 注：默认值是为传统的三轴数控机床设置的。先清除Z轴，然后清除X和Y轴。
#define HOMING_CYCLE_0 (1<<Z_AXIS)//必需：首先移动Z以清除工作区。
#define HOMING_CYCLE_1 ((1<<X_AXIS)|(1<<Y_AXIS))  // 可选：然后同时移动X、Y。
// #define HOMING_CYCLE_2                         // OPTIONAL: Uncomment and add axes mask to enable

// 注：以下是为2轴机器设置回零的两个示例。
// #define HOMING_CYCLE_0 ((1<<X_AXIS)|(1<<Y_AXIS))  // NOT COMPATIBLE WITH COREXY: Homes both X-Y in one cycle. 

// #define HOMING_CYCLE_0 (1<<X_AXIS)  // COREXY COMPATIBLE: First home X
// #define HOMING_CYCLE_1 (1<<Y_AXIS)  // COREXY COMPATIBLE: Then home Y

//机器初始点动至限位开关后执行的回位循环次数。
// 这有助于防止过冲，并应提高重复性。此值应为1或1更大。
#define N_HOMING_LOCATE_CYCLE 1 // 整数 (1-128)

//启用单轴原点命令$用于X、Y和Z轴原点的HX、$HY和$HZ。$H命令仍会调用整个归位循环。这在默认情况下是禁用的。
//这里只针对需要在两轴和三轴机器之间切换的用户。这实际上是非常罕见的。
//如果你有一个双轴机器，不要使用这个。相反，只需改变两个轴的归零周期即可。

// #define HOMING_SINGLE_AXIS_COMMANDS // 默认禁用。取消注释以启用.

//归零后，Grbl将默认将整个机器空间设置为负空间，这是专业CNC机器的典型情况，无论限位开关位于何处。
//取消对该定义的注释，以强制Grbl始终将机器原点设置在原点位置，而不管开关方向如何。

// #define HOMING_FORCE_SET_ORIGIN // Uncomment to enable.

//启动时执行的Grbl块数。这些块存储在EEPROM中，其大小和地址在settings.h中定义
//使用当前设置，最多可存储2个启动块，并按顺序执行。这些启动块通常用于根据用户偏好设置g代码解析器状态。
#define N_STARTUP_LINE 2 // Integer (1-2)

//Grbl为某些值类型打印的浮点小数点的数目。这些设置由数控机床中的实际值和常见观察值确定。
//例如，位置值不能小于0.001mm或0.0001in，因为机器在物理上不能比这更精确。
//所以，可能没有必要更改这些，但是如果需要，可以在这里更改。
//注意：必须是0到~4之间的整数值。超过4个可能会出现舍入误差。
#define N_DECIMAL_COORDVALUE_INCH 4//坐标或位置值（英寸）
#define N_DECIMAL_COORDVALUE_MM   3//坐标或位置值（单位：mm）
#define N_DECIMAL_RATEVALUE_INCH  1//速率或速度值（单位：in/min）
#define N_DECIMAL_RATEVALUE_MM    0//速率或速度值，单位为mm/min
#define N_DECIMAL_SETTINGVALUE    3//浮点设置值的小数
#define N_DECIMAL_RPMVALUE        0//每分钟转数的RPM值。

//如果您的机器有两个平行于一个轴的限位开关，则需要启用此功能。
//由于两个开关共用一个管脚，Grbl无法判断哪一个已启用。 
//此选项仅影响回零，如果启用限位，Grbl将发出警报并强制用户手动断开限位开关。
//否则，如果每个轴都有一个限位开关，则不要启用此选项。 
//通过使其处于禁用状态，您可以在限位开关上执行复位循环，而不必将机器移出限位开关。
// #define LIMITS_TWO_SWITCHES_ON_AXES

//允许GRBL跟踪和报告gcode行号。启用这意味着计划缓冲区从16变为15，以便为计划块结构中的额外行号数据腾出空间
// #define USE_LINE_NUMBERS // Disabled by default. Uncomment to enable.

//探测循环成功后，此选项通过自动生成的消息立即提供探测坐标的反馈。
//如果禁用，用户仍可以通过Grbl“$#”打印参数访问最后一个探测器坐标。
#define MESSAGE_PROBE_COORDINATES // Enabled by default. Comment to disable.

//通过Arduino Uno模拟针脚4上的雾化冷却液g代码命令M7启用第二个冷却液控制针脚。
//仅当需要第二个冷却液控制引脚时才使用此选项。
//注意：不管怎样，模拟针脚3上的M8溢流冷却液控制针脚仍将正常工作。
// #define ENABLE_M7 // Disabled by default. Uncomment to enable.

// 此选项使进给保持输入充当安全门开关。
// 安全门一旦触发，将立即强制进给保持，然后安全断电。 
//在安全门重新接合之前，将阻止恢复。
//此时，Grbl将使机器重新通电，然后恢复上一条刀具路径，就像什么也没发生一样。
// #define ENABLE_SAFETY_DOOR_INPUT_PIN // 默认禁用。取消注释以启用。

//切换并恢复安全门开关后，此设置设置恢复主轴和冷却液与恢复循环之间的通电延迟。
#define SAFETY_DOOR_SPINDLE_DELAY 4.0 // Float (seconds)
#define SAFETY_DOOR_COOLANT_DELAY 1.0 // Float (seconds)

// 启用CoreXY运动学。仅与CoreXY机器一起使用。
// 重要提示：如果启用了归位，则必须重新配置归位循环。
// #defines 上面的 #define HOMING_CYCLE_0 (1<<X_AXIS) 和 #define HOMING_CYCLE_1 (1<<Y_AXIS)
// 注意：此配置选项将X轴和Y轴的运动更改为在(http://corexy.com/theory.html)中定义的工作原理 . 
//假设电机的位置和接线与所述完全一致，否则，运动可能会向奇怪的方向移动。
//Grbl要求CoreXY A和B电机内部每毫米的步数相同。
// #define COREXY // 默认禁用。取消注释以启用。

//基于掩码反转控制命令引脚的引脚逻辑。这基本上意味着您可以在指定的管脚上使用常闭开关，而不是默认的常开开关。
//注：顶部选项将掩码和反转所有控制引脚。底部选项是仅反转两个控制引脚（安全门和复位）的示例。参见cpu_map.h其他位定义。
// #define INVERT_CONTROL_PIN_MASK CONTROL_MASK // 默认禁用。取消注释以禁用。
// #define INVERT_CONTROL_PIN_MASK ((1<<CONTROL_SAFETY_DOOR_BIT)|(1<<CONTROL_RESET_BIT)) // 默认禁用。

//基于以下掩码反转选择限位引脚状态。这会影响所有限位引脚功能，例如硬限位和回零。但是，这与整体反转限位设置不同。
//此构建选项将仅反转此处定义的限制管脚，然后反转限制设置将应用于所有管脚。
//当用户的机器上安装有一组混合的限位引脚，且带有常开（NO）和常闭（NC）开关时，此功能非常有用。
//注意：请不要使用这个，除非你有需要它的情况。
// #define INVERT_LIMIT_PIN_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)) //默认禁用。取消注释以启用。


//将主轴启用引脚从低禁用/高启用反转为低启用/高禁用。适用于某些预制电子板。
//注意：如果启用可变_主轴（默认），此选项不起作用，因为PWM输出和主轴启用共用一个引脚。
//如果您同时需要此选项和主轴速度PWM，请取消注释下面的配置选项USE_spindle_DIR_AS_ENABLE_PIN。
// #define INVERT_SPINDLE_ENABLE_PIN //默认禁用。取消注释以启用。

//将选定的冷却液针脚从低禁用/高启用反转为低启用/高禁用。适用于某些预制电子板。
// #define INVERT_COOLANT_FLOOD_PIN // 默认禁用。取消注释以启用。
// #define INVERT_COOLANT_MIST_PIN // 默认禁用。注意：在config.h中启用M7喷雾冷却液

//当Grbl通电循环或使用Arduino重置按钮硬重置时，默认情况下，Grbl在无报警的情况下启动。
//这是为了使新用户开始使用Grbl尽可能简单。启用归位且用户已安装限位开关时，Grbl将在报警状态下启动，以指示
//Grbl不知道它的位置，因此无法在继续之前强制用户归位。这一选择迫使无论是否归位，Grbl始终初始化为报警状态。
//此选项更适合OEM和LinuxCNC用户希望这种功率循环行为。
// #define FORCE_INITIALIZATION_ALARM // 默认禁用。取消注释以启用.

//通电或复位时，Grbl将检查限位开关状态，以确保它们在初始化前未处于激活状态。
//如果检测到问题并且启用了硬限制设置，Grbl将简单地通知用户检查限制并进入报警状态，而不是空闲状态。Grbl不会抛出警报消息。
#define CHECK_LIMITS_AT_INIT

//---------------------------------------------------------------------------------------
//高级配置选项：

//启用用于调试目的的代码。不适用于一般用途。
// #define DEBUG // 取消注释以启用。默认禁用。

//配置快速、进给和主轴覆盖设置。这些值定义了允许的最大和最小覆盖值以及每个接收命令的粗略增量和精细增量。
//请注意以下各定义说明中的允许值。
#define DEFAULT_FEED_OVERRIDE           100// 100%. 不要更改此值。
#define MAX_FEED_RATE_OVERRIDE          200//编程进给速度的百分比（100-255）。通常为120%或200%
#define MIN_FEED_RATE_OVERRIDE           10//编程进给速度的百分比（1-100）。通常为50%或1%
#define FEED_OVERRIDE_COARSE_INCREMENT   10// (1-99). 通常是10%。
#define FEED_OVERRIDE_FINE_INCREMENT      1// (1-99). 通常为1%。

#define DEFAULT_RAPID_OVERRIDE  100// 100%. 不要更改此值。
#define RAPID_OVERRIDE_MEDIUM    50//快速的百分比（1-99）。通常是50%。
#define RAPID_OVERRIDE_LOW       25 //快速的百分比（1-99）。通常是25%。
// #define RAPID_OVERRIDE_EXTRA_LOW 5 // *不支持*rapid的百分比（1-99）。通常是5%。

#define DEFAULT_SPINDLE_SPEED_OVERRIDE    100// 100%. 不要更改此值。
#define MAX_SPINDLE_SPEED_OVERRIDE        200//编程主轴转速的百分比（100-255）。通常是200%。
#define MIN_SPINDLE_SPEED_OVERRIDE         10//编程主轴转速的百分比（1-100）。通常是10%。
#define SPINDLE_OVERRIDE_COARSE_INCREMENT  10// (1-99). 通常是10%。
#define SPINDLE_OVERRIDE_FINE_INCREMENT     1// (1-99). 通常为1%。

//当执行M2或M30程序结束命令时，大多数g代码状态将恢复为默认状态。
//此编译时选项包括在程序结束时将进给、快速和主轴速度覆盖值恢复为其默认值。
#define RESTORE_OVERRIDES_AFTER_PROGRAM_END//默认启用。注释后禁用

//Grbl v1.1及其后的状态报告更改还删除了从报告中禁用/启用大多数数据字段的功能。
//这给GUI开发人员带来了问题，他们不得不管理多个场景和配置。新报告样式效率的提高允许发送所有数据字段，而不会出现潜在的性能问题。
//注意：下面的选项仅在特殊情况需要时提供禁用某些数据字段的方法，但请注意GUI可能依赖于此数据。如果禁用，则可能不兼容。
#define REPORT_FIELD_BUFFER_STATE //默认启用。注释后禁用
#define REPORT_FIELD_PIN_STATE //默认启用。注释后禁用
#define REPORT_FIELD_CURRENT_FEED_SPEED //默认启用。注释后禁用
#define REPORT_FIELD_WORK_COORD_OFFSET //默认启用。注释后禁用
#define REPORT_FIELD_OVERRIDES //默认启用。注释后禁用
#define REPORT_FIELD_LINE_NUMBERS //默认启用。注释后禁用

//某些状态报告数据不是实时所必需的，只是间歇性的，因为这些值不会经常更改。
//以下宏配置在刷新关联数据并将其包含在状态报告中之前需要调用状态报告的次数。
//但是，如果其中一个值发生变化，Grbl将自动在下一个状态报告中包含该数据，而不管当时的计数是多少。
//这有助于减少高频报告和强流所涉及的通信开销。
//还有一个忙刷新计数和一个空闲刷新计数，它设置Grbl在不做任何重要事情时更频繁地发送刷新。
//一个好的GUI，这些数据不需要经常刷新，只需几秒钟。
// 注意：WCO刷新必须为2或更大。OVR刷新必须为1或更大。
#define REPORT_OVR_REFRESH_BUSY_COUNT 20  // (1-255)
#define REPORT_OVR_REFRESH_IDLE_COUNT 10//（1-255）必须小于或等于忙计数
#define REPORT_WCO_REFRESH_BUSY_COUNT 30  // (2-255)
#define REPORT_WCO_REFRESH_IDLE_COUNT 10  // (2-255) 必须小于或等于忙计数

//加速度管理子系统的时间分辨率。数值越大，加速越平稳，在进给速度非常高的机器上尤其明显，但可能会对性能产生负面影响。
//此参数的正确值取决于机器，因此建议仅将其设置为所需的最高值。成功的近似值范围很广，从50到200或更多。
//注意：更改此值也会更改步骤段缓冲区中段的执行时间。
//增加此值时，段缓冲区中存储的总时间会减少，反之亦然。确保增加/减少步长段缓冲区，以应对这些变化。
#define ACCELERATION_TICKS_PER_SECOND 100

//自适应多轴步进平滑（AMASS）是一种高级功能，它实现了其名称所暗示的多轴运动的步进平滑。此功能可平滑运动，尤其是在10kHz以下的低阶跃频率下，多轴运动轴之间的混叠可能会导致可听噪音并震动机器。在更低的阶跃频率下，AMASS可以适应并提供更好的阶跃平滑。见步进电机。c获取有关AMASS系统工作的更多详细信息。
#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  //默认启用。注释后禁用

//设置允许写入Grbl设置的最大步进速率。此选项启用设置模块中的错误检查，以防止设置值超过此限制。最大步进速率严格受CPU速度的限制，如果使用除16MHz运行的AVR以外的其他设备，则会发生变化。
//注意：现在禁用，如果闪存空间允许，将启用。
// #define MAX_STEP_RATE_HZ 30000 // Hz

// 默认情况下，Grbl将所有输入引脚设置为正常高电平操作，并启用其内部上拉电阻器。
//这就简化了用户的布线，只需要一个接地开关，尽管建议用户在低通滤波器中进行额外的布线，以减少引脚检测到的电气噪声。
//如果用户在Grbl设置中反转引脚，则只会翻转指示激活信号的高或低读数。
//在正常操作中，这意味着用户需要连接一个正常打开的开关，但如果反转，这意味着用户应该连接一个正常关闭的开关。
//以下选项禁用内部上拉电阻器，将引脚设置为正常低操作，开关现在必须连接到Vcc而不是接地。
//这也颠覆了反向引脚Grbl设置的含义，反向设置现在意味着用户应连接正常打开的开关，反之亦然。
//注：与该特征相关的所有引脚均被禁用，即XYZ限位引脚，而不是单个轴。
//警告：当上拉被禁用时，这需要使用下拉电阻器进行额外接线！
//#define DISABLE_LIMIT_PIN_PULL_UP
//#define DISABLE_PROBE_PIN_PULL_UP
//#define DISABLE_CONTROL_PIN_PULL_UP

//设置应用刀具长度偏移的轴。假设主轴始终与选定轴平行，刀具朝向负方向。换句话说，从当前位置减去正的刀具长度偏移值。
#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS//默认z轴。有效值为X_轴、Y_轴或Z_轴。

//启用不同转速值的可变主轴输出电压。在Arduino Uno上，主轴启用引脚将在256个中间电平的最大转速下输出5V，禁用时输出0V。
//注意：对于Arduino Unos来说很重要！启用时，Z限制引脚D11和主轴启用引脚D12开关！
//可变主轴输出电压需要引脚D11上的硬件PWM输出。
#define VARIABLE_SPINDLE //默认启用。注释后禁用

// 仅用于可变主轴输出。这将强制PWM输出在启用时达到最小占空比。
// 当主轴停用时，PWM引脚仍将读取0V。 大多数用户不需要此选项，但在某些情况下它可能很有用。
//该最小PWM设置与主轴rpm最小设置一致，如rpm max 到 max PWM。
//如果您需要在0V禁用和最小PWM设置的电压之间有更大的电压差以达到最小转速，这将非常方便。
//该差值为每PWM值0.02V。
//因此，当最小PWM为1时，只有0.02伏单独启用和禁用。
//在PWM 5时，这将为0.1V。请记住，随着最小PWM值的增加，您将开始失去PWM分辨率，因为您在总共255个PWM电平上发出不同主轴转速信号的范围越来越小。
//注：通过以下等式计算最小脉宽调制下的占空比：（%占空比）=（SPINDLE_PWM_MIN_VALUE/255）*100
// #define SPINDLE_PWM_MIN_VALUE 5 // 默认禁用。取消注释以启用。必须大于零。整数（1-255）。

//默认情况下，在328p（Uno）上，Grbl将可变主轴PWM和启用共用为一个引脚，以帮助保留I/O引脚。
//对于某些设置，这些可能需要单独的引脚。此配置选项将主轴方向引脚（D13）与引脚D11上的主轴速度PWM一起用作单独的主轴启用引脚。
// 注意：此配置选项仅适用于启用可变_主轴和328p处理器（Uno）。
// 注：如果没有方向引脚，M4将没有指示与M3差异的引脚输出。
// 注意：小心！Arduino引导加载程序在通电时切换D13引脚。 
//如果您使用编程器烧录Grbl（您可以使用备用Arduino作为“Arduino作为ISP”）。
//在网站上搜索如何连接。），此D13 LED切换应消失。
//我们还没有测试过这个。请报告进展情况！
// #define USE_SPINDLE_DIR_AS_ENABLE_PIN // 默认禁用。取消注释以启用。

//使用“USE_spindle_DIR_AS_enable_pin”选项更改主轴启用引脚的行为。默认情况下，
//如果主轴转速为零且M3/4激活，Grbl不会禁用启用引脚，但仍将PWM输出设置为零。
//这允许用户知道主轴是否处于活动状态，并将其用作附加控制输入。
//但是，在某些使用情况下，用户可能希望启用引脚在主轴转速为零时禁用，并在主轴转速大于零时重新启用。这个选项可以做到这一点。
//注意：需要USE_SPINDLE_DIR_AS_ENABLE_PIN启用。
// #define SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED // 默认禁用。取消注释以启用。

//启用此选项后，Grbl会发回其接收到的行的回显，该行已被预解析（空格已删除、大写字母、无注释），Grbl将立即执行。
//线路缓冲区溢出时不会发送回显，但应针对发送到Grbl的所有正常线路发送回波。
//例如，如果用户发送行“g1 x1.032 y2.45（测试注释）”，Grbl将以“[echo:G1X1.032Y2.45]”的形式回传。
//注意：仅用于调试目的！！回显时，这会占用宝贵的资源并影响性能。
//如果正常操作绝对需要串行写入缓冲区，则应大大增加串行写入缓冲区，以帮助最小化串行写入协议中的传输等待。
// #define REPORT_ECHO_LINE_RECEIVED // 默认禁用。取消注释以启用。

// 最小规划器连接速度。 设置规划器计划在每个缓冲区块连接处设置的默认最小连接速度，但从缓冲区的剩余部分开始和结束部分（始终为零）除外。
//该值控制机器通过交叉点的速度，而不考虑加速度限制或相邻块线移动方向之间的角度。//This is useful for machines that can't tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. 
//如果使用，该值不应远大于零或机器工作所需的最小值。
#define MINIMUM_JUNCTION_SPEED 0.0 // (mm/min)

//设置规划器允许的最小进给速率。低于该值的任何值都将设置为该最小值。
//这还可以确保计划的运动始终完成，并考虑任何浮点舍入错误。
//虽然不建议使用低于1.0 mm/min的值，但在较小的机器上可能适用，可能为0.1 mm/min，但您的成功率可能因多种因素而异。
#define MINIMUM_FEED_RATE 1.0 // (mm/min)

//在使用昂贵的sin（）和cos（）计算进行精确圆弧轨迹校正之前，通过小角度近似生成圆弧的迭代次数。
//若弧生成的准确性存在问题，则该参数可能会减小，若弧执行因太多trig计算而陷入困境，则该参数可能会增大。
#define N_ARC_CORRECTION 12 // Integer (1-255)

//根据定义，arc G2/3 g代码标准存在问题。当圆弧位于半圆（pi）或全圆（2*pi）时，基于半径的圆弧具有可怕的数值误差。
//基于偏移的圆弧更精确，但当圆弧为整圆（2*pi）时仍然存在问题。当
//基于偏移的圆弧被命令为整圆时，该定义解释了浮点问题，但由于数值舍入和精度问题，被解释为机器ε（1.2e-7rad）附近的极小圆弧。
//此定义值设置机器ε截止，以确定圆弧是否为整圆。
//注意：调整此值时要非常小心。它应该始终大于1.2e-7，但不能太大。默认设置应捕获大多数（如果不是全部）全圆弧错误情况。
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)

//暂停期间执行的时间延迟增量。默认值设置为50ms，这提供了大约55分钟的最大时间延迟，对于大多数应用程序来说已经足够了。
//增加此延迟将线性增加最大驻留时间，但也会降低运行时命令执行（如状态报告）的响应性，因为这些执行是在每个驻留时间步长之间执行的。
//另外，请记住，Arduino延迟计时器对于长延迟不是很准确。
#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

//通过创建另一个中断（Timer2比较）来管理方向引脚设置和相应阶跃脉冲之间的延迟。
//主Grbl中断（定时器1比较）设置方向引脚，而不会像在正常操作中那样立即设置步进器引脚。
//Timer2 比较器在步进脉冲延迟时间后触发，设置步进器引脚，Timer2 溢出将完成步进脉冲，但现在被步进脉冲时间加上步进脉冲延迟延迟。
//（感谢朗瓦特的创意！）
//注意：取消注释以启用。建议的延迟必须大于3us，并且当添加用户提供的步进脉冲时间时，总时间不得超过127us。
//报告的某些设置的成功值范围为5到20us。

// #define STEP_PULSE_DELAY 10 // 以微秒为单位的步进脉冲延迟。默认禁用。

//在任何给定时间，计划器缓冲区中要计划的线性运动数。Grbl使用的绝大多数RAM都基于此缓冲区大小。
//只有在有额外可用RAM的情况下，如为Mega2560重新编译时，才增加内存。
//或者，如果Arduino由于缺少可用RAM而开始崩溃，或者如果CPU在执行新的传入动作时无法跟上规划，则减少。

// #define BLOCK_BUFFER_SIZE 16 // 取消注释以覆盖planner.h中的默认值

//控制步执行算法和规划器块之间中间步进段缓冲区的大小。
//每一段都是在一个固定时间内以恒定速度执行的一组步进，该时间由每秒的加速度确定。
//计算它们时，可精确追踪规划器块速度剖面。
//此缓冲区的大小控制其他Grbl进程在返回并重新填充此缓冲区之前必须计算和执行的步进执行前置时间，当前为50毫秒的步进移动。
// #define SEGMENT_BUFFER_SIZE 6 // 取消注释以覆盖stepper.h中的默认值.

//要执行的串行输入流的行缓冲区大小。此外，还控制每个启动块的大小，因为它们都存储为该大小的字符串。
//确保在settings.h中的定义内存地址处说明可用的EEPROM以及所需启动块的数量。

//注意：除了极端情况外，80个字符不是问题，但行缓冲区大小可能太小，g代码块可能会被截断。
//官方规定，g代码标准最多支持256个字符。在未来的版本中，当我们知道可以重新投入多少额外的内存空间时，这个默认值将增加。
// #define LINE_BUFFER_SIZE 80  // 取消注释以覆盖protocol.h中的默认值

//串行发送和接收缓冲区大小。接收缓冲区通常用作另一个流式缓冲区，用于存储Grbl准备就绪时要处理的传入块。
//大多数流式接口将对发送到每个块响应的每个块进行字符计数和跟踪。

//因此，如果流和可用内存允许需要更深的接收缓冲区，请增加接收缓冲区。
//发送缓冲区主要处理Grbl中的消息。仅当发送大消息且Grbl开始暂停，等待发送其余消息时，才增加。

//注：Grbl大约在0.5毫秒内生成一个平均状态报告，但115200波特的串行TX流传输一个典型的55字符报告需要5毫秒。
//最坏情况报告约为90-100个字符。
//只要串行TX缓冲区没有持续最大化，Grbl将继续有效运行。根据最坏情况报告的大小调整TX缓冲区的大小。
// #define RX_BUFFER_SIZE 128 // (1-254) 取消注释以覆盖serial.h中的默认值
// #define TX_BUFFER_SIZE 100 // (1-254)

//硬限位开关的简单软件去抖动功能。启用时，监控硬限位开关引脚的中断将使Arduino的看门狗定时器在大约32毫秒的延迟后重新检查限位引脚状态。
//这有助于解决数控机床硬限位开关错误触发的问题，但无法解决外部电源信号电缆的电气干扰问题。
//建议首先使用屏蔽连接到地面的屏蔽信号电缆（旧的USB/计算机电缆工作良好，价格便宜），并在低通电路中连接到每个限位引脚。
// #define ENABLE_SOFTWARE_DEBOUNCE // 默认禁用。取消注释以启用。

//在Grbl的检查模式中配置探测循环后的位置。禁用将位置设置为探针目标，启用时将位置设置为开始位置。
// #define SET_CHECK_MODE_PROBE_TO_START // 默认禁用。取消注释以启用。

//当处理器检测到硬限位ISR例程内的引脚变化时，强制Grbl检查硬限位开关的状态。
//默认情况下，Grbl将在任何管脚更改时触发硬限制报警，因为反弹开关可能导致这样的状态检查误读管脚。
//触发硬限制时，它们应100%可靠，这就是默认情况下禁用此选项的原因。
//只有当您的系统/电子设备能够保证开关不会抖动时，我们建议启用此选项。
//这将有助于防止机器脱离开关时触发硬限位。
// 注意：如果启用SOFTWARE_DEBOUNCE，此选项无效。
// #define HARD_LIMIT_FORCE_STATE_CHECK // 默认禁用。取消注释以启用。

//调整归位循环搜索和定位标量。这些是Grbl的归位循环使用的乘数，以确保限位开关在循环的每个阶段都接合和清除。
//搜索阶段使用轴最大行程设置乘以搜索标量来确定查找限位开关的距离。
//一旦找到，定位阶段开始，并使用归位回拉距离设置乘以定位标量拔出并重新接合限位开关。

//注：这两个值必须大于1.0，以确保功能正常。
// #define HOMING_AXIS_SEARCH_SCALAR  1.5 // 取消注释以覆盖limits.c中的默认值。
// #define HOMING_AXIS_LOCATE_SCALAR  10.0 // 取消注释以覆盖limits.c中的默认值。

//启用“$RST=*”、“$RST=$”和“$RST=#”eeprom恢复命令。在某些情况下，这些命令可能是不需要的。只需注释所需的宏即可将其禁用。
//注意：有关自定义“$RST=*`命令的信息，请参见SETTINGS_RESTORE_ALL宏。
#define ENABLE_RESTORE_EEPROM_WIPE_ALL         // '$RST=*' 默认启用。注释后禁用。
#define ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // '$RST=$' 默认启用。注释后禁用。
#define ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // '$RST=#' 默认启用。注释后禁用。

//定义设置版本更改和“$RST=*`命令后恢复的EEPROM数据。
//当Grbl版本之间的设置或其他EEPROM数据结构发生变化时，Grbl将自动擦除并恢复EEPROM。
//此宏控制擦除和恢复哪些数据。这对于需要保留某些数据的原始设备制造商尤其有用。
//例如，构建信息字符串可以通过单独的.INO草图写入Arduino EEPROM，以包含产品数据。
//更改此宏以不恢复构建信息EEPROM将确保固件升级后保留此数据。
//注意：取消注释以覆盖settings.h中的默认值
// #define SETTINGS_RESTORE_ALL (SETTINGS_RESTORE_DEFAULTS | SETTINGS_RESTORE_PARAMETERS | SETTINGS_RESTORE_STARTUP_LINES | SETTINGS_RESTORE_BUILD_INFO)

//启用“$I=（字符串）”生成信息写入命令。如果禁用，任何现有的构建信息数据必须通过具有有效校验和值的外部方式放入EEPROM。
//当用于存储OEM产品数据时，此宏选项有助于防止用户过度写入此数据。
//注意：如果禁用，并且为了确保Grbl永远不会更改构建信息行，您还需要启用上面的SETTINGS_RESTORE_BUILD_INFO宏并从掩码中删除SETTINGS_RESTORE_BUILD_INFO。
//注：请参阅随附的grblWrite_BuildInfo。在示例文件中单独写入此字符串。
#define ENABLE_BUILD_INFO_WRITE_COMMAND // '$I=' 默认启用。注释后禁用。

//AVR处理器要求在EEPROM写入期间禁用所有中断。这包括步进式ISR和串行通信ISR。
//在长时间EEPROM写入的情况下，此ISR暂停可能会导致主动步进丢失位置和串行接收数据丢失。
//每当写入EEPROM时，此配置选项将强制规划器缓冲区完全清空，以防止丢失任何步骤。
//然而，这并不能防止EEPROM写入期间串行RX数据丢失的问题，特别是当GUI同时预先填充串行RX缓冲区时。这是高度建议的
//GUI标记这些GCODE（G10、G28.1、G30.1），以便在发送更多数据以消除此问题之前，始终在包含这些命令之一的块之后等待“ok”。
//注：大多数EEPROM写入命令在作业期间被隐式阻止（所有“$”命令）。
//但是，坐标集g代码命令（G10、G28/30.1）不是，因为它们是活动流作业的一部分。此时，此选项仅强制规划器缓冲区与这些g-code命令同步。
#define FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE //默认启用。注释后禁用

//在GRBLV0。9和之前的版本中, 有一个老的未解决的bug，`WPos:`工作位置报告可能与正在执行的内容不相关，因为`WPos:`基于g-code解析器状态，而可能会有几个动作落后。
//每当有命令更改工作坐标偏移“G10,G43.1,G92,G54-59”时，此选项将强制规划器缓冲区清空、同步和停止运动。
//这是确保“WPos:”始终正确的最简单方法。幸运的是，使用这些命令中的任何一个都需要通过它们进行连续运动，这是非常罕见的。
#define FORCE_BUFFER_SYNC_DURING_WCO_CHANGE //默认启用。注释后禁用

//默认情况下，Grbl禁用所有G38xx探头循环命令的进给速率覆盖。虽然这可能不同于一些专业级的机器控制，但有争议的是，它应该是这种方式。
//大多数探针传感器产生不同程度的误差，这取决于速度。通过将探测周期保持在编程进给速率，探测传感器应具有更高的可重复性。
//如果需要，可以通过取消注释下面的定义来禁用此行为。
// #define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES // 默认禁用。取消注释以启用。

//启用和配置安全门状态下的停车运动方法。
//主要针对希望其集成机器具有此功能的原始设备制造商。
//目前，Grbl假设停车运动只涉及一个轴，尽管停车实现通过修改停车源代码可以轻松地针对不同轴上的任意数量的运动进行重构。
//此时，Grbl仅支持一个轴（通常为Z轴）的停车，该轴在缩回时正向移动，在恢复位置时反向移动。
//该动作以缓慢的拉出收回动作、断电和快速停车执行。
//恢复到恢复位置遵循以下相反的设置动作：快速恢复到拉回位置，超时通电，以较慢的拉回速率跳回原始位置。
//注意：仍在进行中的工作。机器坐标必须在所有负空间中，并且在启用原点时不工作。停车运动也仅向正方向移动。
// #define PARKING_ENABLE  // 默认禁用。取消注释以启用

//配置停车运动的选项（如果启用）。
#define PARKING_AXIS Z_AXIS//定义执行停车运动的轴
#define PARKING_TARGET -5.0//停车轴目标。以毫米为单位，作为机器坐标[-最大行程，0]。
#define PARKING_RATE 500.0//拔出后的快速停车率（mm/min）。
#define PARKING_PULLOUT_RATE 100.0//拉出/插入慢速进给速率，单位为mm/min。
#define PARKING_PULLOUT_INCREMENT 5.0//心轴拉出和插入距离（mm）。增量距离。必须为正值或等于零。

//启用启用和禁用停车运动的一组特殊M代码命令。 
//它们由'M56'、'M56 P1'或'M56 Px'控制以启用，由'M56 P0'控制以禁用。 
//该命令为模式命令，将在计划器同步后设置。
//因为它是g代码，所以它与g代码命令同步执行。它不是一个实时命令。
//注意：需要启用驻车功能。默认情况下，M56在初始化时处于活动状态。
//使用DEACTIVATE_PARKING_UPON_INIT将M56 P0设置为通电默认值。
// #define ENABLE_PARKING_OVERRIDE_CONTROL   // 默认禁用。取消注释以启用
// #define DEACTIVATE_PARKING_UPON_INIT // 默认禁用。取消注释以启用.

//该选项将通过在停止后立即调用主轴停止覆盖，在进给保持期间自动禁用激光器。
//但是，这也意味着，如果需要，可通过禁用主轴停止超控来重新启用激光器。
//这纯粹是一种安全功能，以确保激光器在停止时不会无意中保持通电状态并引发火灾。
#define DISABLE_LASER_DURING_HOLD //默认启用。注释后禁用

//此功能通过简单的分段线性曲线将主轴PWM/速度改变为非线性输出。
//适用于Grbl标准主轴PWM线性模型不能产生正确转速的主轴。
//需要通过仓库中的/doc/script文件夹中的“fit_nonlinear_spindle.py”脚本提供解决方案。
//请参阅有关如何收集主轴数据和运行脚本以生成解决方案的文件注释。
// #define ENABLE_PIECEWISE_LINEAR_SPINDLE  // 默认禁用。取消注释以启用.

//N_PIECES、RPM_MAX、RPM_MIN、RPM_POINTXX和RPM_LINEXX常数均由“fit_nonlinear_spindle.py’脚本解决方案。
//仅当启用“分段线性”主轴时使用。确保常量值与脚本解决方案完全相同。
//注：当N_件小于4时，不需要且省略未使用的RPM_线和RPM_点定义。
#define N_PIECES 4//整数（1-4）。脚本解决方案中使用的分段行数。
#define RPM_MAX  11686.4//模型的最大转速$30>最大转速将限制为最大转速。
#define RPM_MIN  202.5    // Min RPM of model. $31 < RPM_MIN will be limited to RPM_MIN.
#define RPM_POINT12  6145.4  // Used N_PIECES >=2. Junction point between lines 1 and 2.
#define RPM_POINT23  9627.8  // Used N_PIECES >=3. Junction point between lines 2 and 3.
#define RPM_POINT34  10813.9 // Used N_PIECES = 4. Junction point between lines 3 and 4.
#define RPM_LINE_A1  3.197101e-03  // Used N_PIECES >=1. A and B constants of line 1.
#define RPM_LINE_B1  -3.526076e-1
#define RPM_LINE_A2  1.722950e-2   // Used N_PIECES >=2. A and B constants of line 2.
#define RPM_LINE_B2  8.588176e+01
#define RPM_LINE_A3  5.901518e-02  // Used N_PIECES >=3. A and B constants of line 3.
#define RPM_LINE_B3  4.881851e+02
#define RPM_LINE_A4  1.203413e-01  // Used N_PIECES = 4. A and B constants of line 4.
#define RPM_LINE_B4  1.151360e+03

/* --------------------------------------------------------------------------------------- 
  该可选双轴功能主要用于归位循环，以独立定位双电机机架的两侧，即自成方形。
  这需要为克隆电机配备一个额外的限位开关。 
  要自成方形，克隆轴上的两个限位开关必须物理定位，以便在机架为方形时触发。
  强烈建议始终启用电机，以确保机架与$1=255设置保持垂直。
  对于Arduino Uno上的Grbl，由于缺少可用引脚，克隆的轴限位开关必须与z轴限位引脚共享并与之连接。
  归位循环必须在不同循环中归位z轴和克隆轴，这已经是默认配置。

  双轴功能通过将轴步输出克隆到另一对步来工作和方向引脚。可设置克隆电机的步进脉冲和方向独立于主轴电机。
  然而，为了节省宝贵的闪存和内存，这双轴功能必须与其他功能共享相同的设置（步长/毫米、最大速度、加速度）
  母马达。这不是独立第四轴的特征。只是克隆一个电机。

  警告：确保测试双轴电机的方向！必须对其进行设置在运行第一个归位循环或任何长运动之前，向同一方向移动！
相反方向移动的电机可能会对机器造成严重损坏！用这个双轴功能，风险自负。
*/
//注意：此功能需要大约400字节的闪存。某些配置可能会耗尽闪存，无法安装在Arduino 328p/Uno上。
//仅支持X轴和Y轴。支持可变主轴/激光模式，但仅适用于一个配置选项。CoreXY、主轴方向销和M7喷雾冷却液已禁用/不受支持。
// #define ENABLE_DUAL_AXIS	// 默认禁用。取消注释以启用.

//选择一个轴以镜像另一个电机。此时仅支持X轴和Y轴。
#define DUAL_AXIS_SELECT  X_AXIS//必须是X_AXIS或Y_AXIS

//为防止回零循环使双轴发生倾斜，当一个限位开关因开关故障或噪音而先触发另一个限位开关时，如果第二台电机的限位开关未在以下三个距离参数内触发，回零循环将自动中止。
//轴长度百分比将自动计算故障距离，作为其他非双轴最大行程的百分比，即，如果双轴选择X_AXIS为5.0%，则故障距离将计算为y轴最大行程的5.0%。失效距离最大值和最小值是有效失效距离的极限。
#define DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT  5.0  // Float (percent)
#define DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX  25.0  // Float (mm)
#define DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN  2.5  // Float (mm)

//双轴引脚配置目前支持两个扩展板。
//取消注释所需的扩展板，并注释掉其他扩展板。
// 注：Protoneer CNC Shield v3。51的A.STP和A.DIR分别连接到引脚A4和A3。
// 可变主轴（即激光模式）构建选项工作，可以启用或禁用。
// 冷却液引脚A3移动到D13，更换主轴方向。
#define DUAL_AXIS_CONFIG_PROTONEER_V3_51    // 取消注释以选择。注释其他配置。

//注：Arduino CNC 扩展板克隆（最初为Protoneer v3.0）的A.STP和A.DIR分别连接到D12和D13。
//由于限位销和步进器启用销位于同一端口上，必须移动主轴启用销并删除主轴方向销。
//主轴启用销现在位于A3上，取代冷却液启用。冷却液启用到针脚A4。
//主轴启用使用得更多，这种引脚设置有助于方便用户集成此功能，而无需进行太多工作。
//可变主轴（例如激光模式）不能与配置的扩展一起工作。
//虽然从技术上讲，可变主轴可以与此扩展板一起工作，但它需要太多的更改，大多数用户设置无法适应。
//最好通过共享插脚D9/D10上的所有限位开关（如[X1，Z]/[X2，Y]或[X，Y2]/[Y1，Z]）、独立设置每个轴的原点以及更新大量代码来实现，以确保一切正常运行。
// #define DUAL_AXIS_CONFIG_CNC_SHIELD_CLONE  // 取消注释以选择。注释其他配置。


/* ---------------------------------------------------------------------------------------
   OEM单文件配置选项

   说明： 在下面粘贴cpu_映射和默认设置定义不带#ifdef。
   注释掉这个文件顶部的CPU_MAP_xxx 和 DEFAULT_xxx定义，然后编译器会忽略defaults.h和cpu_map.h，并使用下面定义的。
*/

// 将CPU映射定义粘贴到此处。

// 在此处粘贴默认设置定义。


#endif
