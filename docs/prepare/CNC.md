# 什么是CNC？
CNC是（Computer Numberical Control）数控机床的简称，是一种装有程序控制系统的自动化机床。它能够处理符号指令规定的程序（如G代码，后面会介绍），从而使机床动作并加工零件。   

# 什么是G代码？
G代码（G-code），是最为广泛使用的数控编程语言，有多个版本，主要在计算机辅助制造中用于控制自动机床。G代码有时候也称为G编程语言。使用G代码可以实现快速定位、逆圆插补、顺圆插补、中间点圆弧插补、半径编程、跳转加工。

# CNC的结构组成
一台完整的数控机床的典型组成部分应该包括上位机下位机两部分。
上位机指的是可以直接发送操作指令的计算机或者单片机，一般提供用户操作交互界面并向用户展示反馈数据。典型设备：电脑、平板、手机、面板、触摸屏。
下位机：指的是与机器相连接的计算机或者单片机，一般用于接收和反馈上位机的指令，并根据指令控制机器执行动作以及从机器传感器读取数据。