/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef nuts_bolts_h
#define nuts_bolts_h

#define false 0
#define true 1

#define SOME_LARGE_VALUE 1.0E+38

//轴数组索引值。必须以0开头并连续。
#define N_AXIS 3//轴数
#define X_AXIS 0//轴索引值。
#define Y_AXIS 1
#define Z_AXIS 2
// #define A_AXIS 3

//CoreXY电机赋值。不要改变。
//注意：如果A和B电机轴绑定发生更改，则会影响Corerxy方程。
#ifdef COREXY
 #define A_MOTOR X_AXIS // 必须是 X_AXIS
 #define B_MOTOR Y_AXIS // 必须是 Y_AXIS
#endif

//转换
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)
#define TICKS_PER_MICROSECOND (F_CPU/1000000)

#define DELAY_MODE_DWELL       0
#define DELAY_MODE_SYS_SUSPEND 1

//有用的宏
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define isequal_position_vector(a,b) !(memcmp(a, b, sizeof(float)*N_AXIS))

// 位字段和掩码宏定义
#define bit(n) (1 << n)
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

//从字符串中读取浮点值。行指向输入缓冲区，char_counter是指向行的当前字符的索引器，而float_ptr是指向结果变量的指针。成功时返回true
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

//非阻塞延迟功能用于一般操作和暂停功能。
void delay_sec(float seconds, uint8_t mode);

//延迟变量定义的毫秒。_delay_ms（）的编译器兼容性修复程序。
void delay_ms(uint16_t ms);

//延迟变量定义的微秒。_delay_us（）的编译器兼容性修复程序。
void delay_us(uint32_t us);

//计算斜边，避免avr gcc的膨胀版本和额外的错误检查。
float hypot_f(float x, float y);

float convert_delta_vector_to_unit_vector(float *vector);
float limit_value_by_axis_maximum(float *max_value, float *unit_vec);

#endif
