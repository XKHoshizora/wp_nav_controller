# Waypoint Navigation Controller (wp_nav_controller)

## 目录

- [概述](#概述)
- [功能特性](#功能特性)
- [功能包架构](#功能包架构)
- [依赖说明](#依赖说明)
- [安装说明](#安装说明)
- [使用指南](#使用指南)
  - [基本使用](#基本使用)
  - [命令详解](#命令详解)
  - [配置说明](#配置说明)
- [接口文档](#接口文档)
- [扩展开发](#扩展开发)
- [故障排除](#故障排除)
- [常见问题](#常见问题)
- [贡献指南](#贡献指南)

## 概述

wp_nav_controller 是一个用于机器人航点导航的 ROS 功能包，提供了灵活的航点导航控制功能，支持单点导航、序列导航、暂停/恢复等操作。该包可与 amr_map_tools 无缝集成，实现完整的机器人导航解决方案。

### 主要特性

- 支持单点导航和序列导航
- 支持导航任务的暂停/恢复
- 提供实时状态反馈
- 支持导航可视化
- 可扩展的命令接口
- 完整的错误处理机制

## 功能包架构

### 整体架构

```plaintext
+------------------------+
|    外部接口层          |
|  (ROS Topics/Services) |
+------------------------+
           ↕
+------------------------+
|    节点控制层          |
| (WaypointNavigatorNode)|
+------------------------+
           ↕
+------------------------+
|    核心导航层          |
|  (WaypointNavigator)   |
+------------------------+
           ↕
+------------------------+
|    move_base接口层     |
|   (Action Client)      |
+------------------------+
```

### 目录结构

```
wp_nav_controller/
├── msg/                        # 消息定义
│   └── NavigationCommand.msg   # 导航命令消息定义
├── include/wp_nav_controller/  # 头文件
│   ├── waypoint_navigator.h    # 导航器核心类定义
│   └── navigation_status.h     # 状态管理定义
├── src/                        # 源代码实现
│   ├── waypoint_navigator.cpp  # 导航器核心实现
│   ├── navigation_status.cpp   # 状态管理实现
│   └── wp_nav_controller_node.cpp  # ROS节点实现
├── launch/                     # 启动文件
│   └── wp_nav_controller.launch
├── rviz/                      # 可视化配置
│   └── wp_nav.rviz
└── scripts/                   # 测试和工具脚本
    └── nav_test.py
```

### 核心组件说明

#### 1. 导航命令处理 (NavigationCommand)

- **功能**: 定义统一的导航命令接口
- **位置**: `msg/NavigationCommand.msg`
- **职责**:
  - 命令类型定义
  - 参数传递
  - 扩展性预留

#### 2. 导航器核心 (WaypointNavigator)

- **功能**: 实现核心导航逻辑
- **位置**: `include/wp_nav_controller/waypoint_navigator.h`
- **主要职责**:
  - 导航控制
  - 状态管理
  - 路径规划
  - 错误处理

#### 3. 节点控制器 (WaypointNavigatorNode)

- **功能**: 提供 ROS 接口和控制逻辑
- **位置**: `src/wp_nav_controller_node.cpp`
- **主要职责**:
  - 话题订阅
  - 命令处理
  - 状态发布
  - 参数管理

### 数据流

```plaintext
外部命令 → NavigationCommand 话题
    ↓
WaypointNavigatorNode
    ↓
命令解析和验证
    ↓
WaypointNavigator处理
    ↓
move_base动作调用
    ↓
状态反馈和可视化
```

### 接口设计

#### 1. 外部接口

```plaintext
发布话题:
- /wp_nav/status
- /wp_nav/current_waypoint
- /waypoint_markers

订阅话题:
- /wp_nav/command

服务调用:
- /waypoint/get_num_waypoint
- /waypoint/get_waypoint_index
- /waypoint/get_waypoint_name
```

#### 2. 内部接口

- WaypointNavigator 类接口
- NavigationStatus 接口
- move_base Action 接口

### 扩展机制

#### 1. 命令扩展

- 通过 NavigationCommand 消息扩展
- 预留参数字段
- 自定义命令类型

#### 2. 行为扩展

- 继承 WaypointNavigator 类
- 重写导航行为
- 添加自定义功能

#### 3. 状态扩展

- 自定义状态消息
- 扩展状态发布
- 添加新的监控指标

### 配置系统

#### 1. 启动配置

- launch 文件参数配置
- RViz 可视化配置
- 节点参数配置

#### 2. 运行时配置

- 动态参数调整
- 服务前缀设置
- 导航参数配置

### 安全机制

#### 1. 错误处理

- 命令验证
- 超时处理
- 异常恢复

#### 2. 状态监控

- 导航状态检查
- 位置监控
- 错误报告

## 依赖说明

### 必要依赖

本功能包依赖于 amr_map_tools 包提供的航点管理功能，主要使用以下服务：

- /waypoint/get_num_waypoint：获取航点总数
- /waypoint/get_waypoint_index：根据索引获取航点
- /waypoint/get_waypoint_name：根据名称获取航点

### 启动顺序

launch 文件已经包含了必要的节点启动顺序：

1. 首先启动 amr_map_tools 的航点管理节点(wp_waypoint_manager)
2. 等待 2 秒确保服务可用
3. 启动导航控制器节点(wp_nav_controller)

### 手动启动

如果需要手动启动各个节点，请按以下顺序：

```bash
# 1. 先启动航点管理器
rosrun amr_map_tools wp_waypoint_manager

# 2. 等待几秒后启动导航控制器
rosrun wp_nav_controller wp_nav_controller_node
```

### 验证依赖服务

可以通过以下命令验证必要的服务是否可用：

```bash
# 检查服务是否存在
rosservice list | grep waypoint

# 测试服务调用
rosservice call /waypoint/get_num_waypoint
```

## 安装说明

### 前置依赖

```bash
# ROS 基础包
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-tf
sudo apt-get install ros-noetic-rviz

# 安装 amr_map_tools（必需）
cd ~/catkin_ws/src
git clone https://github.com/XKHoshizora/amr_map_tools.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 安装步骤

```bash
# 1. 克隆仓库
cd ~/catkin_ws/src
git clone https://github.com/XKHoshizora/wp_nav_controller.git

# 2. 安装依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译
catkin_make

# 4. 配置环境
source ~/catkin_ws/devel/setup.bash
```

### 验证安装

```bash
# 检查包是否正确安装
rospack find wp_nav_controller

# 检查消息类型
rosmsg show wp_nav_controller/NavigationCommand

# 检查依赖服务是否可用
rosservice call /waypoint/get_num_waypoint
```

## 使用指南

### 基本使用

1. 启动导航控制器：

```bash
# 使用默认配置启动
roslaunch wp_nav_controller wp_nav_controller.launch

# 自定义配置启动
roslaunch wp_nav_controller wp_nav_controller.launch \
    use_rviz:=true \
    update_rate:=10.0 \
    goal_tolerance:=0.5 \
    timeout:=300.0
```

2. 通过命令行发送导航命令：

```bash
# 导航到指定航点
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 3
waypoint_index: 2"

# 开始序列导航
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: false"

# 暂停导航
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 1"

# 恢复导航
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 2"
```

3. 使用 Python API：

```python
#!/usr/bin/env python
import rospy
from wp_nav_controller.msg import NavigationCommand
from geometry_msgs.msg import PoseStamped

class NavigationController:
    def __init__(self):
        rospy.init_node('nav_controller')
        self.cmd_pub = rospy.Publisher('/wp_nav/command', NavigationCommand, queue_size=1)

    def goto_waypoint(self, index):
        cmd = NavigationCommand()
        cmd.command = NavigationCommand.GOTO_WAYPOINT
        cmd.waypoint_index = index
        self.cmd_pub.publish(cmd)

    def start_sequence(self, reverse=False):
        cmd = NavigationCommand()
        cmd.command = NavigationCommand.START_SEQUENCE
        cmd.reverse = reverse
        self.cmd_pub.publish(cmd)

    def pause_navigation(self):
        cmd = NavigationCommand()
        cmd.command = NavigationCommand.PAUSE
        self.cmd_pub.publish(cmd)

    def resume_navigation(self):
        cmd = NavigationCommand()
        cmd.command = NavigationCommand.RESUME
        self.cmd_pub.publish(cmd)

# 使用示例
if __name__ == '__main__':
    controller = NavigationController()
    controller.goto_waypoint(1)
    rospy.sleep(2)
    controller.pause_navigation()
```

### 命令详解

#### NavigationCommand 消息结构

```plaintext
uint8 command        # 命令类型
int32 waypoint_index # 目标航点索引
geometry_msgs/PoseStamped target_pose  # 目标位置
bool reverse         # 序列执行方向
string params        # 扩展参数
```

#### 命令类型说明

| 命令           | 值  | 说明           | 所需参数       |
| -------------- | --- | -------------- | -------------- |
| STOP           | 0   | 停止当前导航   | 无             |
| PAUSE          | 1   | 暂停导航任务   | 无             |
| RESUME         | 2   | 恢复导航任务   | 无             |
| GOTO_WAYPOINT  | 3   | 导航到指定航点 | waypoint_index |
| GOTO_POSE      | 4   | 导航到指定位置 | target_pose    |
| START_SEQUENCE | 5   | 开始序列导航   | reverse        |
| STOP_SEQUENCE  | 6   | 停止序列导航   | 无             |

### 配置说明

#### Launch 文件参数

```xml
<arg name="use_rviz" default="true"/>           <!-- 是否启动RViz -->
<arg name="update_rate" default="10.0"/>        <!-- 状态更新频率 -->
<arg name="goal_tolerance" default="0.5"/>      <!-- 目标达到容差 -->
<arg name="timeout" default="300.0"/>          <!-- 导航超时时间 -->
<arg name="service_prefix" default="waypoint"/> <!-- 服务前缀 -->
```

#### RViz 配置

可以通过修改 `rviz/wp_nav.rviz` 文件自定义显示选项：

- 添加/移除显示项
- 修改显示样式
- 配置坐标系
- 自定义可视化效果

## 扩展开发

### 1. 添加新的导航命令

1. 在 NavigationCommand.msg 中添加新命令：

```plaintext
uint8 NEW_COMMAND = 7  # 新命令类型
```

2. 在 WaypointNavigator 类中实现处理：

```cpp
class WaypointNavigator {
public:
    void handleNewCommand() {
        // 实现新命令逻辑
    }
private:
    // 添加必要的成员变量和辅助函数
};
```

3. 在节点中添加处理：

```cpp
void commandCallback(const NavigationCommand::ConstPtr& msg) {
    switch(msg->command) {
        case NavigationCommand::NEW_COMMAND:
            navigator_.handleNewCommand();
            break;
    }
}
```

### 2. 自定义导航行为

1. 创建派生类：

```cpp
class CustomNavigator : public WaypointNavigator {
public:
    CustomNavigator(ros::NodeHandle& nh) : WaypointNavigator(nh) {
        // 自定义初始化
    }

    // 重写基类方法
    void navigateToWaypoint(int wpIndex) override {
        // 自定义导航逻辑
    }

    // 添加新方法
    void customBehavior() {
        // 实现自定义行为
    }
};
```

### 3. 添加新的状态信息

1. 创建自定义状态消息：

```cpp
void publishCustomStatus() {
    std_msgs::String status;
    std::stringstream ss;
    ss << "Custom Status: " << customData;
    status.data = ss.str();
    customStatusPub_.publish(status);
}
```

## 故障排除

### 常见问题解决

1. 导航失败

- 检查 `move_base` 是否正常运行
- 确认目标点是否在地图允许范围内
- 检查导航参数配置
- 查看 CPU 和内存使用情况

2. 服务连接失败

```bash
# 检查服务是否存在
rosservice list | grep waypoint

# 检查服务类型
rosservice info /waypoint/get_waypoint_index

# 测试服务调用
rosservice call /waypoint/get_num_waypoint
```

3. 位置转换错误

```bash
# 检查TF树
rosrun tf view_frames
evince frames.pdf

# 查看具体转换
rosrun tf tf_echo map base_link
```

### 日志说明

重要的日志消息及含义：

```plaintext
[INFO] "Navigation to waypoint[X] is now active"
- 开始导航到航点X

[WARN] "Failed to get robot pose"
- TF转换失败，检查坐标系配置

[ERROR] "Failed to get waypoint at index X"
- 无法获取航点信息，检查amr_map_tools服务
```

## 贡献指南

### 开发流程

1. Fork 项目仓库
2. 创建功能分支
3. 提交更改
4. 创建 Pull Request

### 代码规范

1. C++代码风格

- 遵循 ROS C++ 编码规范
- 使用 clang-format 格式化代码
- 添加适当的注释
- 处理所有异常情况

2. Python 代码风格

- 遵循 PEP 8 规范
- 使用 pylint 检查代码质量
- 添加文档字符串

### 提交准则

1. 提交消息格式

```
<type>(<scope>): <subject>

<body>

<footer>
```

2. 类型说明

- feat: 新功能
- fix: 错误修复
- docs: 文档更新
- style: 代码风格修改
- refactor: 代码重构
- test: 测试相关
- chore: 构建过程或辅助工具的变动

### 问题报告

创建 Issue 时请包含：

- 问题描述
- 复现步骤
- 期望行为
- 实际行为
- 环境信息
- 相关日志
