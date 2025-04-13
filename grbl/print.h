/*
  print.h - 用于格式化输出字符串的函数
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#ifndef print_h
#define print_h


void printString(const char *s);

void printPgmString(const char *s);

void printInteger(long n);

void print_uint32_base10(uint32_t n);

//打印基数为10的uint8变量。
void print_uint8_base10(uint8_t n);

//以所需位数打印基数2中的uint8变量。
void print_uint8_base2_ndigit(uint8_t n, uint8_t digits);

void printFloat(float n, uint8_t decimal_places);

//Grbl中使用的特殊变量类型的浮点值打印处理程序。
//-坐标值：以英寸或毫米为单位处理所有位置或坐标值。
//-RateValue：以英寸或毫米为单位处理进给速度和当前速度报告。
void printFloat_CoordValue(float n);
void printFloat_RateValue(float n);

//调试工具，用于在调用点以字节为单位打印可用内存。不用于其他用途。
void printFreeMemory();

#endif
