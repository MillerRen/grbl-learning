/*
  nuts_bolts.c - Shared functions
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


#define MAX_INT_DIGITS 8 //int32（和浮点）中的最大位数


//从字符串中提取浮点值。下面的代码松散地基于Michael Stumpf和Dmitry Xmelkov的avr libc strod（）函数以及许多免费提供的转换方法示例，但已经针对Grbl进行了高度优化。众所周知
//在CNC应用中，典型的十进制值预计在E0到E-4之间。
//g代码不支持科学符号，在某些CNC系统上，“E”字符可能是g代码。因此，“E”符号不会被识别。
//注意：感谢Radu Eosif Mihailescu发现使用strod（）的问题。
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)
{
  char *ptr = line + *char_counter;
  unsigned char c;

  //抓取第一个字符和递增指针。直线上不设空格。
  c = *ptr++;

  //捕获初始正/负字符
  bool isnegative = false;
  if (c == '-') {
    isnegative = true;
    c = *ptr++;
  } else if (c == '+') {
    c = *ptr++;
  }

  //将数字提取为快速整数。按指数值跟踪小数点。
  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = false;
  while(1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (ndigit <= MAX_INT_DIGITS) {
        if (isdecimal) { exp--; }
        intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
      } else {
        if (!(isdecimal)) { exp++; }  //丢弃溢出数字
      }
    } else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
      isdecimal = true;
    } else {
      break;
    }
    c = *ptr++;
  }

  //如果未读取任何数字，则返回。
  if (!ndigit) { return(false); };

  //将整数转换为浮点。
  float fval;
  fval = (float)intval;

  //应用十进制。对于预期的E0到E-4范围，应执行不超过两次浮点乘法。
  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01;
      exp += 2;
    }
    if (exp < 0) {
      fval *= 0.1;
    } else if (exp > 0) {
      do {
        fval *= 10.0;
      } while (--exp > 0);
    }
  }

  //用正确的符号指定浮点值。
  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval;
  }

  *char_counter = ptr - line - 1; //将char_counter设置为下一个语句

  return(true);
}


//非阻塞延迟功能用于一般操作和暂停功能。
void delay_sec(float seconds, uint8_t mode)
{
 	uint16_t i = ceil(1000/DWELL_TIME_STEP*seconds);
	while (i-- > 0) {
		if (sys.abort) { return; }
		if (mode == DELAY_MODE_DWELL) {
			protocol_execute_realtime();
		} else { // DELAY_MODE_SYS_SUSPEND
		  // 仅执行rt_system（）以避免嵌套挂起循环。
		  protocol_exec_rt_system();
		  if (sys.suspend & SUSPEND_RESTART_RETRACT) { return; } //如果安全门重新打开，则退出。
		}
		_delay_ms(DWELL_TIME_STEP); //延迟DWELL_TIME_STEP增量
	}
}


//延迟变量定义的毫秒。_delay_ms（）的编译器兼容性修复程序，它只接受未来编译器版本中的常量。
void delay_ms(uint16_t ms)
{
  while ( ms-- ) { _delay_ms(1); }
}


//延迟变量定义的微秒。_delay_us（）的编译器兼容性修复程序，它只接受未来编译器版本中的常量。
//由于计数器在每次迭代中都会增加寄生时间，因此编写的代码可以更有效地执行更大的延迟。
void delay_us(uint32_t us)
{
  while (us) {
    if (us < 10) {
      _delay_us(1);
      us--;
    } else if (us < 100) {
      _delay_us(10);
      us -= 10;
    } else if (us < 1000) {
      _delay_us(100);
      us -= 100;
    } else {
      _delay_ms(1);
      us -= 1000;
    }
  }
}


//简单的斜边计算函数。
float hypot_f(float x, float y) { return(sqrt(x*x + y*y)); }


float convert_delta_vector_to_unit_vector(float *vector)
{
  uint8_t idx;
  float magnitude = 0.0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (vector[idx] != 0.0) {
      magnitude += vector[idx]*vector[idx];
    }
  }
  magnitude = sqrt(magnitude);
  float inv_magnitude = 1.0/magnitude;
  for (idx=0; idx<N_AXIS; idx++) { vector[idx] *= inv_magnitude; }
  return(magnitude);
}


float limit_value_by_axis_maximum(float *max_value, float *unit_vec)
{
  uint8_t idx;
  float limit_value = SOME_LARGE_VALUE;
  for (idx=0; idx<N_AXIS; idx++) {
    if (unit_vec[idx] != 0) {  //避免被零除。
      limit_value = min(limit_value,fabs(max_value[idx]/unit_vec[idx]));
    }
  }
  return(limit_value);
}
