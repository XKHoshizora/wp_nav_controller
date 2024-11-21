```
wp_nav_controller/               # 主功能包目录
├── CMakeLists.txt              # 主功能包的CMake文件
├── package.xml                 # 主功能包的包描述文件
├── scripts/                    # 可执行脚本目录
├── src/                        # 源代码目录
├── srv/                        # 服务定义目录
│   ├── NavigationCommand.srv
│   └── SequenceCommand.srv
└── wp_nav_gui/                 # GUI子包目录
    ├── CMakeLists.txt         # GUI包的CMake文件
    ├── package.xml            # GUI包的包描述文件
    ├── README.md              # GUI使用说明
    ├── setup.py              # Python包配置文件
    └── wp_nav_gui/           # Python模块目录
        ├── __init__.py
        ├── navigation_ui.py   # 主UI代码文件
        └── scripts/          # GUI可执行脚本目录
            └── navigation_ui_node.py  # 启动脚本
```

这个UI提供了以下功能：

导航控制标签页：

支持三种导航目标类型的输入（航点名称、索引、位姿坐标）
导航控制按钮（导航至目标、暂停、继续、停止、清除代价地图）
导航状态显示


序列控制标签页：

航点序列的创建和管理（添加、删除、清空）
序列导航模式设置（单程、循环、往返）
序列导航控制（启动、暂停、继续、停止）
序列状态显示


其他功能：

状态栏显示操作结果
定时更新状态显示
错误提示对话框
ROS服务调用错误处理