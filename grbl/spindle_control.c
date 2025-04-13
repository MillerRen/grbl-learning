/*
  spindle_control.c - 主轴控制模块
  Grbl 的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


#ifdef VARIABLE_SPINDLE
  static float pwm_gradient; //用于加速rpm到PWM转换的预计算值。
#endif


void spindle_init()
{
  #ifdef VARIABLE_SPINDLE
    //如果需要，配置可变主轴PWM和启用引脚。
    //在Uno上，PWM和enable是共用的，除非另有配置。
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); //配置为PWM输出引脚。
    SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; //配置PWM输出比较定时器
    SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); //配置为输出引脚。
    #else
      #ifndef ENABLE_DUAL_AXIS
        SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); //配置为输出引脚。
      #endif
    #endif
    pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); //配置为输出引脚。
    #ifndef ENABLE_DUAL_AXIS
      SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); //配置为输出引脚。
    #endif
  #endif

  spindle_stop();
}


uint8_t spindle_get_state()
{
  #ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      //没有主轴方向输出引脚。
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #else
        if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #endif
    #else
      if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { //检查PWM是否启用。
        #ifdef ENABLE_DUAL_AXIS
          return(SPINDLE_STATE_CW);
        #else
          if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
          else { return(SPINDLE_STATE_CW); }
        #endif
      }
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { 
    #else
      if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
    #endif
      #ifdef ENABLE_DUAL_AXIS    
        return(SPINDLE_STATE_CW);
      #else
        if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      #endif
    }
  #endif
  return(SPINDLE_STATE_DISABLE);
}


//启用PWM可变主轴转速时，禁用主轴并将PWM输出设置为零。
//由各种主程序和ISR例程调用。保持小规模、快速、高效的运行。
//由主轴_init（）、主轴_set_speed（）、主轴_set_state（）和mc_reset（）调用。
void spindle_stop()
{
  #ifdef VARIABLE_SPINDLE
    SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); //禁用PWM。输出电压为零。
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  //将引脚设置为高
      #else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); //将引脚设置为低
      #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  //将引脚设置为高
    #else
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
    #endif
  #endif
}


#ifdef VARIABLE_SPINDLE
  //设置主轴速度PWM输出和启用引脚（如果配置）。由spindle_set_state（）和步进ISR调用。保持小规模和高效率的运行。
  void spindle_set_speed(uint8_t pwm_value)
  {
    SPINDLE_OCR_REGISTER = pwm_value; //设置PWM输出电平。
    #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        spindle_stop();
      } else {
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); //确保PWM输出已启用。
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        #else
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      }
    #else
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); //禁用PWM。输出电压为零。
      } else {
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); //确保PWM输出已启用。
      }
    #endif
  }


  #ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE
  
    // 由spindle_set_state和步进段生成器调用。 保持小规模和高效率的运行。
    uint8_t spindle_compute_pwm_value(float rpm) //328p PWM寄存器为8位。
    {
      uint8_t pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); //按主轴速度覆盖值缩放。根据rpm最大/最小设置和编程rpm计算PWM寄存器值。
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
        rpm = RPM_MAX;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= RPM_MIN) {
        if (rpm == 0.0) { //S0禁用主轴
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else {
          rpm = RPM_MIN;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else {
        //通过分段线性拟合模型，利用线性主轴转速模型计算中间PWM值。
        #if (N_PIECES > 3)
          if (rpm > RPM_POINT34) {
            pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4);
          } else 
        #endif
        #if (N_PIECES > 2)
          if (rpm > RPM_POINT23) {
            pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3);
          } else 
        #endif
        #if (N_PIECES > 1)
          if (rpm > RPM_POINT12) {
            pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2);
          } else 
        #endif
        {
          pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1);
        }
      }
      sys.spindle_speed = rpm;
      return(pwm_value);
    }
    
  #else 
  
    //由spindle_set_state和步进段生成器调用。保持小规模和高效率的运行。
    uint8_t spindle_compute_pwm_value(float rpm) //328p PWM寄存器为8位。
    {
      uint8_t pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); //按主轴速度覆盖值缩放。根据rpm最大/最小设置和编程rpm计算PWM寄存器值。
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        //不可能有PWM范围。设置简单的开/关主轴控制引脚状态。
        sys.spindle_speed = settings.rpm_max;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= settings.rpm_min) {
        if (rpm == 0.0) { //S0禁用主轴
          sys.spindle_speed = 0.0;
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else { //设置最小PWM输出
          sys.spindle_speed = settings.rpm_min;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else { 
        //使用线性主轴转速模型计算中间PWM值。
        //注：如果需要，可以在此处安装非线性模型，但要保持非常轻量。
        sys.spindle_speed = rpm;
        pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
      }
      return(pwm_value);
    }
    
  #endif
#endif


//如果启用，则立即通过PWM设置主轴运行状态以及方向和主轴转速。
//由g-code解析器spindle_sync（）调用、驻车收回和恢复、g-code程序结束、睡眠和主轴停止覆盖。
#ifdef VARIABLE_SPINDLE
  void spindle_set_state(uint8_t state, float rpm)
#else
  void _spindle_set_state(uint8_t state)
#endif
{
  if (sys.abort) { return; } //在中止期间阻塞。

  if (state == SPINDLE_DISABLE) { //停止或设置主轴方向和转速。
  
    #ifdef VARIABLE_SPINDLE
      sys.spindle_speed = 0.0;
    #endif
    spindle_stop();
  
  } else {
    
    #if !defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(ENABLE_DUAL_AXIS)
      if (state == SPINDLE_ENABLE_CW) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }
    #endif
  
    #ifdef VARIABLE_SPINDLE
      //注意：假设对该函数的所有调用都是在Grbl未移动或必须保持关闭的情况下进行的。
      if (settings.flags & BITFLAG_LASER_MODE) { 
        if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } //TODO:可能需要为rpm_min*（100/MAX_SPINDLE_SPEED_OVERRIDE）；
      }
      spindle_set_speed(spindle_compute_pwm_value(rpm));
    #endif
    #if (defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && \
        !defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)) || !defined(VARIABLE_SPINDLE)
      //注意：在没有可变主轴的情况下，启用位应仅打开或关闭，无论主轴速度值是否为零，因为其无论如何都会被忽略。
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif    
    #endif
  
  }
  
  sys.report_ovr_counter = 0; //设置为立即报告更改
}


//用于设置主轴状态的G代码解析器入口点。如果中止或检查模式处于活动状态，则强制规划器缓冲区同步并停止。
#ifdef VARIABLE_SPINDLE
  void spindle_sync(uint8_t state, float rpm)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); //清空规划器缓冲区，以确保编程时设置主轴。
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(uint8_t state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); //清空规划器缓冲区，以确保编程时设置主轴。
    _spindle_set_state(state);
  }
#endif
