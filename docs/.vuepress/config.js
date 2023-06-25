module.exports = {
    title: 'Grbl 源码解析与移植',
    description: 'Grbl源码精度与解析，代码行级中文注释并提供丰富详尽的案例展示。',
    head: [
        [
            'link', { rel: 'icon', href: '/logo.png' }
        ]
    ],
    themeConfig: {
        nav: [
            { text: '首页', link: '/' },
            // { text: '指南', link: '/guide/' },
            { text: 'Github', link: 'https://github.com/MillerRen/grbl-learning' },
        ],
        sidebar: [
            ['/', '首页'],
            ['/preface', '前言'],
            ['/prepare', '准备工作'],
            // ['/analysis/', 'Grbl源码解析'],
            {
                title: 'Grbl源码解析',   
                path: '/analysis/',   
                children: [
                  {
                    title: '开始',
                    path: '/analysis/'
                  },
                  {
                    title: '架构',
                    path: '/analysis/architecture.md'
                  },
                  {
                    title: '主入口',
                    path: '/analysis/main.md'
                  },
                  {
                    title: '串口',
                    path: '/analysis/serial.md'
                  },
                  {
                    title: '协议',
                    path: '/analysis/protocol.md'
                  },
                  {
                    title: '状态机',
                    path: '/analysis/system.md'
                  },
                  {
                    title: 'G代码解析',
                    path: '/analysis/gcode.md'
                  },
                  {
                    title: '运动控制',
                    path: '/analysis/motion.md'
                  },
                  {
                    title: '运动规划',
                    path: '/analysis/planner.md'
                  },
                  {
                    title: '加速度算法',
                    path: '/analysis/acceleration.md'
                  },
                  {
                    title: 'bresenham插值算法',
                    path: '/analysis/bresenham.md'
                  },
                  {
                    title: '运动执行-步进电机',
                    path: '/analysis/stepper.md'
                  },
                  {
                    title: '主轴',
                    path: '/analysis/spindle.md'
                  },
                  {
                    title: '冷却',
                    path: '/analysis/coolant.md'
                  },
                  {
                    title: '限位开关',
                    path: '/analysis/endstop.md'
                  },
                  {
                    title: '归位',
                    path: '/analysis/homing.md'
                  },
                  {
                    title: '对刀',
                    path: '/analysis/probe.md'
                  },
                  {
                    title: '安全-紧急事件',
                    path: '/analysis/emergency.md'
                  },
                  {
                    title: '参数设置',
                    path: '/analysis/settings.md'
                  },
                  {
                    title: '上位机接口',
                    path: '/analysis/interface.md'
                  }
                ]
              },
            ['/porting/', 'Grbl移植'],
            ['/extensions/', 'Grbl扩展'],
        ]
    }
}