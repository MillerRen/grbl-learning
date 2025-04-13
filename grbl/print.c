/*
  print.c - 用于格式化输出字符串的函数
   Grbl的一部分

  版权所有 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  版权所有 2009-2011 Simen Svale Skogsrud
  
  Grbl 是自由软件：你可以在自由软件基金会的GNU 普通公共许可(GPL v3+)条款下发行，或修改它。
  Grbl的发布是希望它能有用，但没有任何保证;甚至没有隐含的保证适销性或适合某一特定目的。
  更多详细信息，请参阅GNU通用公共许可证。

  您应该已经收到GNU通用公共许可证的副本和Grbl一起。如果没有，请参阅<http://www.gnu.org/licenses/>。
*/

#include "grbl.h"


void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}


//打印存储在PGM内存中的字符串
void printPgmString(const char *s)
{
  char c;
  while ((c = pgm_read_byte_near(s++)))
    serial_write(c);
}


// void printIntegerInBase(unsigned long n, unsigned long base)
// {
// 	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
// 	unsigned long i = 0;
//
// 	if (n == 0) {
// 		serial_write('0');
// 		return;
// 	}
//
// 	while (n > 0) {
// 		buf[i++] = n % base;
// 		n /= base;
// 	}
//
// 	for (; i > 0; i--)
// 		serial_write(buf[i - 1] < 10 ?
// 			'0' + buf[i - 1] :
// 			'A' + buf[i - 1] - 10);
// }


//打印基数为10的uint8变量。
void print_uint8_base10(uint8_t n)
{
  uint8_t digit_a = 0;
  uint8_t digit_b = 0;
  if (n >= 100) { // 100-255
    digit_a = '0' + n % 10;
    n /= 10;
  }
  if (n >= 10) { // 10-99
    digit_b = '0' + n % 10;
    n /= 10;
  }
  serial_write('0' + n);
  if (digit_b) { serial_write(digit_b); }
  if (digit_a) { serial_write(digit_a); }
}


//以所需位数打印基数2中的uint8变量。
void print_uint8_base2_ndigit(uint8_t n, uint8_t digits) {
  unsigned char buf[digits];
  uint8_t i = 0;

  for (; i < digits; i++) {
      buf[i] = n % 2 ;
      n /= 2;
  }

  for (; i > 0; i--)
      serial_write('0' + buf[i - 1]);
}


void print_uint32_base10(uint32_t n)
{
  if (n == 0) {
    serial_write('0');
    return;
  }

  unsigned char buf[10];
  uint8_t i = 0;

  while (n > 0) {
    buf[i++] = n % 10;
    n /= 10;
  }

  for (; i > 0; i--)
    serial_write('0' + buf[i-1]);
}


void printInteger(long n)
{
  if (n < 0) {
    serial_write('-');
    print_uint32_base10(-n);
  } else {
    print_uint32_base10(n);
  }
}


//通过立即转换为长整数，将浮点转换为字符串，长整数包含的数字比浮点多。
//由计数器跟踪的小数位数可由用户设置。然后将整数有效地转换为字符串。
//注意：AVR“%”和“/”整数操作非常有效。位移加速技术实际上只是稍微慢一点。我是通过艰苦的努力才发现这一点的。
void printFloat(float n, uint8_t decimal_places)
{
  if (n < 0) {
    serial_write('-');
    n = -n;
  }

  uint8_t decimals = decimal_places;
  while (decimals >= 2) { //快速将预期为E0的值转换为E-4。
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // 添加舍入因子。 确保整个值的进位。

  //向后生成数字并以字符串形式存储。
  unsigned char buf[13];
  uint8_t i = 0;
  uint32_t a = (long)n;
  while(a > 0) {
    buf[i++] = (a % 10) + '0'; //获取数字
    a /= 10;
  }
  while (i < decimal_places) {
     buf[i++] = '0'; //将零填入小数点（n<1）
  }
  if (i == decimal_places) { //如果需要，填写前导零。
    buf[i++] = '0';
  }

  //打印生成的字符串。
  for (; i > 0; i--) {
    if (i == decimal_places) { serial_write('.'); } //在正确的位置插入小数点。
    serial_write(buf[i-1]);
  }
}


//Grbl中使用的特殊变量类型的浮点值打印处理程序，在配置中定义。H
//-坐标值：以英寸或毫米为单位处理所有位置或坐标值。
//-RateValue：以英寸或毫米为单位处理进给速度和当前速度报告。
void printFloat_CoordValue(float n) {
  if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
    printFloat(n*INCH_PER_MM,N_DECIMAL_COORDVALUE_INCH);
  } else {
    printFloat(n,N_DECIMAL_COORDVALUE_MM);
  }
}

void printFloat_RateValue(float n) {
  if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
    printFloat(n*INCH_PER_MM,N_DECIMAL_RATEVALUE_INCH);
  } else {
    printFloat(n,N_DECIMAL_RATEVALUE_MM);
  }
}

// Debug tool to print free memory in bytes at the called point.
// NOTE: Keep commented unless using. Part of this function always gets compiled in.
// void printFreeMemory()
// {
//   extern int __heap_start, *__brkval;
//   uint16_t free;  // Up to 64k values.
//   free = (int) &free - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
//   printInteger((int32_t)free);
//   printString(" ");
// }
