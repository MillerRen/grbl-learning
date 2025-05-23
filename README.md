# grbl-learning

grbl源码精读与解析，代码行级中文注释，包含详细例程

## 为什么写这个？

1. grbl在广大CNC爱好者中被广泛应用，它也提供了编译好的固件，刷了就能用。但是真正能够理解并能够移植的人并不多,因为grbl中涉及大量专业知识机械制造，电子电路，编译原理等，需要跨学科知识体系。
1. grbl直接操作底层寄存器实现相应的功能，并没有使用arduino框架，并且大量使用二进制位操作，代码晦涩，新手难以理解。
1. 市面上关于grbl源码和移植的内容寥寥无几，大部分都是零散的内容，不够系统化。
1. 新的MCU层出不穷，爱好者们希望能将grbl移植到不同的平台上，但是原版的grbl对avr atmega328p这款芯片做了高度优化，代码抽象程度不够，移植起来比较复杂。
基于以上几点，我希望能在理解了原版grbl源码的基础上，进行注释和实验案例，给需要移植grbl的小伙伴们一些帮助。

## 目录

1. [前言](./docs/preface.md)
1. [准备工作](./docs/prepare.md)
    1. [CNC基础](./docs/prepare.md)
    1. [Grbl简介](./docs/prepare.md)
    1. [软件硬件准备](./docs/prepare.md)

1. [grbl源码解析](./docs/analysis/)
    1. [开始](./docs/analysis/)
    1. [架构](./docs/analysis/architecture.md)
    1. [入口](./docs/analysis/main.md)
    1. [串口](./docs/analysis/serial.md)
    1. [协议-主循环](./docs/analysis/protocol.md)
    1. [协议-状态报告](./docs/analysis/report.md)
    1. [协议-gcode解析](./docs/analysis/gcode.md)
    1. 运动规划
    1. 运动算法
    1. 电机动作
    1. 主轴动作
    1. 冷却动作
    1. 限位
    1. 归位
    1. 对刀
    1. 参数设置

1. [Grbl移植](./docs/porting/)
    1. 硬件软件分层
    1. 硬件对应

1. [Grbl扩展](./docs/extensions/)
    1. gcode扩展
    1. 功能扩展


## 内容许可

本仓库所有内容可以自由分享、使用，不得用于各种形式售卖再发行和欺诈。
