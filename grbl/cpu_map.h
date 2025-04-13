/*
  cpu_map.h - CPU和引脚映射配置文件
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

/*cpu_map.h文件用作不同处理器类型或备选管脚布局的中心管脚映射选择文件。此版本的Grbl官方仅支持Arduino Mega328p。*/


#ifndef cpu_map_h
#define cpu_map_h


#ifdef CPU_MAP_ATMEGA328P//（Arduino Uno）由Grbl正式支持。

//定义串行端口引脚和中断向量。
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

//定义步进脉冲输出引脚。注意：所有步进位引脚必须位于同一端口上。
  #define STEP_DDR        DDRD
  #define STEP_PORT       PORTD
  #define X_STEP_BIT      2//Uno数字引脚2
  #define Y_STEP_BIT      3//Uno数字管脚3
  #define Z_STEP_BIT      4//Uno数字引脚4
  #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT))//所有步进位

//定义步进方向输出引脚。注意：所有方向引脚必须位于同一端口上。
  #define DIRECTION_DDR     DDRD
  #define DIRECTION_PORT    PORTD
  #define X_DIRECTION_BIT   5//Uno数字管脚5
  #define Y_DIRECTION_BIT   6//Uno数字管脚6
  #define Z_DIRECTION_BIT   7//Uno数字引脚7
  #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // 所有方向位

//定义步进驱动器启用/禁用输出引脚。
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0 // 一个数字管脚8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

//定义归位/硬限位开关输入引脚和限位中断向量。
//注意：所有限制位引脚必须位于同一端口上，但不能位于具有其他输入引脚（控制）的端口上。
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno数字管脚9
  #define Y_LIMIT_BIT      2//Uno数字管脚10
  #ifdef VARIABLE_SPINDLE//Z限制引脚和主轴启用已交换，以访问引脚11上的硬件PWM。
    #define Z_LIMIT_BIT	   4 // Uno数字管脚12
  #else
    #define Z_LIMIT_BIT    3//Uno数字管脚11
  #endif
  #if !defined(ENABLE_DUAL_AXIS)
    #define LIMIT_MASK     ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT))//所有限制位
  #endif
  #define LIMIT_INT        PCIE0//引脚更改中断启用引脚
  #define LIMIT_INT_vect   PCINT0_vect
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

//定义用户控制（循环启动、复位、进给保持）输入引脚。
//注意：所有控制管脚必须位于同一端口上，而不是位于具有其他输入管脚的端口上（限位）。
  #define CONTROL_DDR       DDRC
  #define CONTROL_PIN       PINC
  #define CONTROL_PORT      PORTC
  #define CONTROL_RESET_BIT         0//Uno模拟引脚0
  #define CONTROL_FEED_HOLD_BIT     1//Uno模拟引脚1
  #define CONTROL_CYCLE_START_BIT   2//Uno模拟引脚2
  #define CONTROL_SAFETY_DOOR_BIT   1//Uno模拟引脚1注：安全门与馈电保持共用。由配置定义启用。
  #define CONTROL_INT       PCIE1//引脚更改中断启用引脚
  #define CONTROL_INT_vect  PCINT1_vect
  #define CONTROL_PCMSK     PCMSK1//引脚改变中断寄存器
  #define CONTROL_MASK      ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))
  #define CONTROL_INVERT_MASK   CONTROL_MASK//可重新定义为仅反转某些控制引脚。

  //定义探针开关输入引脚。
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       5//Uno模拟引脚5
  #define PROBE_MASK      (1<<PROBE_BIT)

  #if !defined(ENABLE_DUAL_AXIS)

    //定义也冷和雾化冷却液启用输出引脚。
    #define COOLANT_FLOOD_DDR   DDRC
    #define COOLANT_FLOOD_PORT  PORTC
    #define COOLANT_FLOOD_BIT   3//Uno模拟引脚3
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   4//Uno模拟引脚4

//定义主轴启用和主轴方向输出引脚。
    #define SPINDLE_ENABLE_DDR    DDRB
    #define SPINDLE_ENABLE_PORT   PORTB
//Z限制引脚和主轴PWM/启用引脚交换以访问引脚11上的硬件PWM。
    #ifdef VARIABLE_SPINDLE
      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
//如果启用，主轴方向引脚现在用作主轴启用，而PWM保持在D11上。
        #define SPINDLE_ENABLE_BIT    5//Uno数字引脚13（注：由于LED，D13不能上拉输入。）
      #else
        #define SPINDLE_ENABLE_BIT    3//Uno数字管脚11
      #endif
    #else
      #define SPINDLE_ENABLE_BIT    4  // Uno数字管脚12
    #endif
    #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
      #define SPINDLE_DIRECTION_DDR   DDRB
      #define SPINDLE_DIRECTION_PORT  PORTB
      #define SPINDLE_DIRECTION_BIT   5//Uno数字引脚13（注：由于LED，D13不能上拉输入。）
    #endif

//可变主轴配置如下。除非你知道自己在做什么，否则不要改变。
//注：仅在启用可变主轴时使用。
    #define SPINDLE_PWM_MAX_VALUE     255//不要改变。328p快速PWM模式将最大值固定为255。
    #ifndef SPINDLE_PWM_MIN_VALUE
      #define SPINDLE_PWM_MIN_VALUE   1//必须大于零。
    #endif
    #define SPINDLE_PWM_OFF_VALUE     0
    #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
    #define SPINDLE_TCCRA_REGISTER    TCCR2A
    #define SPINDLE_TCCRB_REGISTER    TCCR2B
    #define SPINDLE_OCR_REGISTER      OCR2A
    #define SPINDLE_COMB_BIT          COM2A1

//预分频，8位快速PWM模式。
    #define SPINDLE_TCCRA_INIT_MASK   ((1<<WGM20) | (1<<WGM21))  // 配置快速PWM模式。
    // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS20)               // Disable prescaler -> 62.5kHz
    // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS21)               // 1/8 prescaler -> 7.8kHz (Used in v0.9)
    // #define SPINDLE_TCCRB_INIT_MASK   ((1<<CS21) | (1<<CS20)) // 1/32 prescaler -> 1.96kHz
    #define SPINDLE_TCCRB_INIT_MASK      (1<<CS22)               // 1/64 prescaler -> 0.98kHz (J-tech laser)

//注意：在328p上，这些设置必须与主轴启用设置相同。
    #define SPINDLE_PWM_DDR   DDRB
    #define SPINDLE_PWM_PORT  PORTB
    #define SPINDLE_PWM_BIT   3//Uno数字管脚11
  
  #else

    // 双轴功能需要一个独立的步进脉冲引脚才能工作。
    //独立方向引脚不是绝对必要的，但有助于通过Grbl$$设置轻松反转方向。
    //这些引脚取代主轴方向引脚和可选冷却液雾引脚。

    #ifdef DUAL_AXIS_CONFIG_PROTONEER_V3_51
//注：步进脉冲和方向引脚可能位于任何端口和输出引脚上。
      #define STEP_DDR_DUAL       DDRC
      #define STEP_PORT_DUAL      PORTC
      #define DUAL_STEP_BIT       4//Uno模拟引脚4
      #define STEP_MASK_DUAL      ((1<<DUAL_STEP_BIT))
      #define DIRECTION_DDR_DUAL  DDRC
      #define DIRECTION_PORT_DUAL PORTC
      #define DUAL_DIRECTION_BIT  3//Uno模拟引脚3
      #define DIRECTION_MASK_DUAL ((1<<DUAL_DIRECTION_BIT))

//注意：默认情况下，双轴限制与z轴限制引脚共享。使用的引脚必须位于同一端口上与其他限位引脚一样。
      #define DUAL_LIMIT_BIT    Z_LIMIT_BIT
      #define LIMIT_MASK        ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<DUAL_LIMIT_BIT))

//定义冷却液启用输出引脚。
//注：冷却液从A3移动到A4。Arduino Uno上的双轴功能不支持冷却液雾。
      #define COOLANT_FLOOD_DDR   DDRB
      #define COOLANT_FLOOD_PORT  PORTB
      #define COOLANT_FLOOD_BIT   5一个数字管脚13

//定义主轴启用输出引脚。
//注意：主轴启用从D12移动到A3（旧的冷却液启用引脚）。主轴方向引脚已移除。
      #define SPINDLE_ENABLE_DDR    DDRB
      #define SPINDLE_ENABLE_PORT   PORTB
      #ifdef VARIABLE_SPINDLE
//注：双轴功能不支持USE_SPINDLE_DIR_AS_ENABLE_PIN。
        #define SPINDLE_ENABLE_BIT    3//Uno数字管脚11
      #else
        #define SPINDLE_ENABLE_BIT    4一个数字引脚12
      #endif

//可变主轴配置如下。除非你知道自己在做什么，否则不要改变。
//注：仅在启用可变主轴时使用。
      #define SPINDLE_PWM_MAX_VALUE     255//不要改变。328p快速PWM模式将最大值固定为255。
      #ifndef SPINDLE_PWM_MIN_VALUE
        #define SPINDLE_PWM_MIN_VALUE   1//必须大于零。
      #endif
      #define SPINDLE_PWM_OFF_VALUE     0
      #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
      #define SPINDLE_TCCRA_REGISTER    TCCR2A
      #define SPINDLE_TCCRB_REGISTER    TCCR2B
      #define SPINDLE_OCR_REGISTER      OCR2A
      #define SPINDLE_COMB_BIT          COM2A1

//预分频，8位快速PWM模式。
      #define SPINDLE_TCCRA_INIT_MASK   ((1<<WGM20) | (1<<WGM21))  //配置快速PWM模式。
      // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS20)               // Disable prescaler -> 62.5kHz
      // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS21)               // 1/8 prescaler -> 7.8kHz (Used in v0.9)
      // #define SPINDLE_TCCRB_INIT_MASK   ((1<<CS21) | (1<<CS20)) // 1/32 prescaler -> 1.96kHz
      #define SPINDLE_TCCRB_INIT_MASK      (1<<CS22)               // 1/64 prescaler -> 0.98kHz (J-tech laser)

//注意：在328p上，这些设置必须与主轴启用设置相同。
      #define SPINDLE_PWM_DDR   DDRB
      #define SPINDLE_PWM_PORT  PORTB
      #define SPINDLE_PWM_BIT   3//Uno数字管脚11
    #endif

//注：此字段不支持可变主轴。
    #ifdef DUAL_AXIS_CONFIG_CNC_SHIELD_CLONE
//注：步进脉冲和方向引脚可能位于任何端口和输出引脚上。
      #define STEP_DDR_DUAL       DDRB
      #define STEP_PORT_DUAL      PORTB
      #define DUAL_STEP_BIT       4  // Uno 数字引脚 12
      #define STEP_MASK_DUAL      ((1<<DUAL_STEP_BIT))
      #define DIRECTION_DDR_DUAL  DDRB
      #define DIRECTION_PORT_DUAL PORTB
      #define DUAL_DIRECTION_BIT  5  // Uno 数字引脚 13
      #define DIRECTION_MASK_DUAL ((1<<DUAL_DIRECTION_BIT))

//注意：默认情况下，双轴限制与z轴限制引脚共享。
      #define DUAL_LIMIT_BIT    Z_LIMIT_BIT
      #define LIMIT_MASK        ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<DUAL_LIMIT_BIT))

//定义冷却液启用输出引脚。
//注：冷却液从A3移动到A4。Arduino Uno上的双轴功能不支持冷却液雾。
      #define COOLANT_FLOOD_DDR   DDRC
      #define COOLANT_FLOOD_PORT  PORTC
      #define COOLANT_FLOOD_BIT   4//Uno模拟引脚4

//定义主轴启用输出引脚。
//注意：主轴启用从D12移动到A3（旧的冷却液启用引脚）。主轴方向引脚已移除。
      #define SPINDLE_ENABLE_DDR    DDRC
      #define SPINDLE_ENABLE_PORT   PORTC
      #define SPINDLE_ENABLE_BIT    3//Uno模拟引脚3
    #endif

  #endif

#endif

/*
#ifdef CPU_MAP_CUSTOM_PROC
  //对于自定义pin映射或不同的处理器，请复制并编辑一个可用的cpu
//映射文件并根据需要进行修改。确保在中也更改了定义的名称
//config.h文件。
#endif
*/

#endif
