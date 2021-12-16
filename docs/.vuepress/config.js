module.exports = {
    title: 'Grbl 源码解析与移植',
    description: 'Grbl源码精度与解析，代码行级中文注释并提供丰富详尽的案例展示。',
    themeConfig: {
        nav: [
            { text: '首页', link: '/' },
            { text: '指南', link: '/guide/' },
            { text: 'Github', link: 'https://github.com/MillerRen/grbl-learning' },
        ],
        sidebar: [
            ['/', '首页'],
            ['/preface', '前言'],
            ['/CNC/', 'CNC基础'],
            ['/analysis/', 'Grbl源码解析'],
            ['/porting/', 'Grbl移植'],
            ['/extensions/', 'Grbl扩展'],
        ]
    }
}