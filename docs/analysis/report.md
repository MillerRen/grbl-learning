# 协议-报告

`grbl`报告程序`report.c`的主要作用是返回格式化的消息给上位机，包含状态信息、警报信息、回馈信息及配置信息。它并没有使用c语言标准库`stdio.h`的`printf`函数，而是自己实现了一套格式化打印程序`print.c`，目的是为了减少程序体积。最后调用底层串口接口`serial_write`实现输出。

## 自定义打印
grbl中打印主要是字符串、按2进制，10进制打印`uint8_t` `uint16_t` `uint32_t` `float`等类型，通过几个函数实现了这些数据类型的语义化输出，这些实现都比较简单很容易读懂源码。

**打印字符串：** 移动指针，逐个打印字符。

```c
// print.c
// 打印字符串
void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}
```
**打印uint8_t：**
这里使用`'0'+n`的方式把数字变成了可读的字符数字，如果有高位就把它除以响应的位数，得到单字符（0-9）后再输出。

```c
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
```

**打印uint32_t：** 打印`uint32_t`类型的数据跟`uint8_t`类型的数据差不多，但是考虑到`uint32_t`的10进制数有10位数字（4294967296）每个都判断一次会有大量重复代码，因此定义了一个数组保存数位上的字符，循环执行除以10和取余操作。
```c
// print.c
// 以10进制显示uint32_t类型的数据
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
```

**打印整数：** 打印整数的帮助函数，主要包含了负整数的判断。
```c
// 打印整数
void printInteger(long n)
{
  if (n < 0) {
    serial_write('-');
    print_uint32_base10(-n);
  } else {
    print_uint32_base10(n);
  }
}
```

**打印浮点数：** 打印浮点数跟整数也类似，只是小数部分是乘以10，然后在对应位置输出小数点`.`。
```c
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
```

**打印静态字符串：** 比较有意思的是它为了节约内存资源，把常量字符串放到了flash中，通过引用flash中字符串头指针一个一个地打印，而不是把整个字符串放到内存中再打印，这是一个不错的技巧,不过缺点是没有内存快并且没有办法修改。我们看下源码：
```c
// pgmspace.h
# define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];}))
```
上面代码中`PSTR(s)`的作用是声明字符串`s`为静态常量，存储位置为`PROGMEM`，并返回字符串的首地址指针。

打印储存在flash中的字符串：
```c
// print.c
//打印存储在PGM内存中的字符串
void printPgmString(const char *s)
{
  char c;
  while ((c = pgm_read_byte_near(s++)))
    serial_write(c);
}
```
```c
// pgmspace.h
#define __LPM_enhanced__(addr)  \
(__extension__({                \
    uint16_t __addr16 = (uint16_t)(addr); \
    uint8_t __result;           \
    __asm__ __volatile__        \
    (                           \
        "lpm %0, Z" "\n\t"      \
        : "=r" (__result)       \
        : "z" (__addr16)        \
    );                          \
    __result;                   \
}))
#define __LPM(addr)         __LPM_enhanced__(addr)
#define pgm_read_byte_near(address_short) __LPM((uint16_t)(address_short))
```
上面代码最终展开为通过汇编指令`lpm`获得flash中的对应静态常量字符串指针处的字符，循环打印字符串。
