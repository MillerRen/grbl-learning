module.exports = {
  title: "GRBL 源码解析与移植",
  description: "GRBL源码精度与解析，代码行级中文注释并提供丰富详尽的案例展示。",
  head: [["link", { rel: "icon", href: "/logo.png" }]],
  themeConfig: {
    logo: "/logo.png",
    nav: [
      // { text: '首页', link: '/' },
      { text: "打赏", link: "/donate/" },
      // { text: 'GRBL中文注解', link: 'https://github.com/MillerRen/grbl' },
      { text: "Github", link: "https://github.com/MillerRen/grbl-learning" },
    ],
    sidebar: [
      ["/", "首页"],
      ["/preface", "前言"],
      ["/prepare", "准备工作"],
      {
        title: "GRBL源码解析",
        path: "/analysis/",
        children: [
          {
            title: "开始",
            path: "/analysis/",
          },
          {
            title: "架构",
            path: "/analysis/architecture.md",
          },
          {
            title: "入口",
            path: "/analysis/main.md",
          },
          {
            title: "协议-串口",
            path: "/analysis/serial.md",
          },
          {
            title: "协议-主循环",
            path: "/analysis/protocol.md",
          },
          
          {
            title: "协议-G代码解析",
            path: "/analysis/gcode.md",
          },
          {
            title: "运动-控制",
            path: "/analysis/motion.md",
          },
          {
            title: "运动-规划",
            path: "/analysis/planner.md",
          },
          {
            title: "运动-加减速",
            path: "/analysis/acceleration.md",
          },
          {
            title: "运动-差值",
            path: "/analysis/bresenham.md",
          },
          {
            title: "运动-执行",
            path: "/analysis/stepper.md",
          },
          {
            title: "附件-主轴",
            path: "/analysis/spindle.md",
          },
          {
            title: "附件-冷却",
            path: "/analysis/coolant.md",
          },
          {
            title: "运动-限制",
            path: "/analysis/endstop.md",
          },
          {
            title: "运动-安全",
            path: "/analysis/emergency.md",
          },
          {
            title: "运动-归位",
            path: "/analysis/homing.md",
          },
          {
            title: "运动-对刀",
            path: "/analysis/probe.md",
          },
          {
            title: "接口-报告",
            path: "/analysis/report.md",
          },
          {
            title: "接口-上位机",
            path: "/analysis/interface.md",
          },
          {
            title: "参数-设置",
            path: "/analysis/settings.md",
          },
          {
            title: "参数-保存",
            path: "/analysis/eeprom.md",
          },
          {
            title: "系统-运行时",
            path: "/analysis/runtime.md",
          },
          {
            title: "系统-状态机",
            path: "/analysis/state.md",
          },
        ],
      },
      {
        title: "GRBL移植",
        path: "/porting/",
        children: [
          {
            title: "MCU选型",
            path: "/porting/lectotype.md",
          },
          {
            title: "GPIO移植",
            path: "/porting/GPIO.md",
          },
          {
            title: "串口移植",
            path: "/porting/serial.md",
          },
          {
            title: "定时器移植",
            path: "/porting/timer.md",
          },
          {
            title: "EEPROM移植",
            path: "/porting/EEPROM.md",
          },
          {
            title: "原子化操作",
            path: "/porting/ATOMIC.md",
          },
        ],
      },
      {
        title: "GRBL扩展",
        path: "/extensions/",
        children: [
          {
            title: "支持舵机",
            path: "/extensions/servo.md",
          },
          {
            title: "支持换刀",
            path: "/extensions/toolchange.md",
          },
        ],
      },
      ["/appendix/", "附录"],
    ],
  },
};
