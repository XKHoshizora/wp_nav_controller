# Waypoint Navigation Controller

## 目录

- [概述](#概述)
- [功能特性](#功能特性)
- [功能包架构](#功能包架构)
- [依赖说明](#依赖说明)
- [安装说明](#安装说明)
- [使用指南](#使用指南)
  - [基本使用](#基本使用)
  - [循环导航模式](#循环导航模式)
  - [命令详解](#命令详解)
  - [配置说明](#配置说明)
- [接口文档](#接口文档)
- [扩展开发](#扩展开发)
- [故障排除](#故障排除)
- [常见问题](#常见问题)
- [贡献指南](#贡献指南)

## 概述

wp_nav_controller 是一个用于机器人航点导航的 ROS 功能包，提供了灵活的航点导航控制功能，支持单点导航、序列导航、循环导航等多种导航模式。该包可与 amr_map_tools 无缝集成，实现完整的机器人导航解决方案。

### 主要特性

- 支持单点导航和序列导航
- 支持多种循环导航模式（单程、原路返回、重新开始）
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
├── CMakeLists.txt
├── package.xml
├── msg/                        # 消息定义
│   └── NavigationCommand.msg   # 导航命令消息定义
├── include/wp_nav_controller/  # 头文件
│   ├── waypoint_navigator.h    # 导航器核心类定义
├── src/                        # 源代码实现
│   ├── waypoint_navigator.cpp  # 导航器核心实现
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

### 循环导航模式

本功能包支持三种循环导航模式，可以通过 `loop_mode` 参数进行设置：

#### 1. 单程模式 (LOOP_NONE = 0)

- 完成单次导航序列后停止
- 适用于单次任务执行

```bash
# 正向单程导航
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: false
loop_mode: 0"

# 反向单程导航
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: true
loop_mode: 0"
```

#### 2. 重启模式 (LOOP_RESTART = 1)

- 完成导航序列后返回起点重新开始
- 适用于重复性任务

```bash
# 正向循环导航（到终点后回到起点重新开始）
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: false
loop_mode: 1"

# 反向循环导航（到起点后回到终点重新开始）
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: true
loop_mode: 1"
```

#### 3. 逆序模式 (LOOP_REVERSE = 2)

- 完成当前序列后按相反顺序返回
- 适用于往返任务

```bash
# 完成正向导航后按逆序返回
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: false
loop_mode: 2"

# 完成反向导航后按正序返回
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: true
loop_mode: 2"
```

#### Python API 使用循环导航

```python
#!/usr/bin/env python
import rospy
from wp_nav_controller.msg import NavigationCommand

class NavigationController:
    def __init__(self):
        rospy.init_node('nav_controller')
        self.cmd_pub = rospy.Publisher('/wp_nav/command', NavigationCommand, queue_size=1)

    def goto_waypoint(self, index):
        cmd = NavigationCommand()
        cmd.command = NavigationCommand.GOTO_WAYPOINT
        cmd.waypoint_index = index
        self.cmd_pub.publish(cmd)

    def start_sequence_with_loop(self, reverse=False, loop_mode=0):
        cmd = NavigationCommand()
        cmd.command = NavigationCommand.START_SEQUENCE
        cmd.reverse = reverse
        cmd.loop_mode = loop_mode
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
    # 启动逆序循环导航
    controller.start_sequence_with_loop(reverse=False, loop_mode=2)
    # 启动正向重复导航
    controller.start_sequence_with_loop(reverse=False, loop_mode=1)
```

#### Python 测试脚本使用示例

```bash
# 启动导航控制器
roslaunch wp_nav_controller wp_nav_controller.launch

# 查看使用说明
rosrun wp_nav_controller nav_test.py --help

# 开始循环导航（正向，逆序模式）
rosrun wp_nav_controller nav_test.py start_sequence false 2

# 开始循环导航（反向，重启模式）
rosrun wp_nav_controller nav_test.py start_sequence true 1

# 暂停导航
rosrun wp_nav_controller nav_test.py pause

# 恢复导航
rosrun wp_nav_controller nav_test.py resume

# 停止序列导航
rosrun wp_nav_controller nav_test.py stop_sequence
```

#### 循环导航状态监控

可以通过状态话题监控循环导航的执行情况：

```bash
# 订阅状态信息
rostopic echo /wp_nav/status

# 状态信息示例
Navigation Status:
  Current Waypoint: 2
  Navigation State: Navigating
  Sequence State: Forward      # 或 Reverse
  Loop Mode: Reverse          # No Loop/Restart/Reverse
  Current Goal: (1.23, 4.56)
```

#### 循环导航行为说明

1. **单程模式 (LOOP_NONE)**：

   - 正向：0 → 1 → 2 → 3 → 停止
   - 反向：3 → 2 → 1 → 0 → 停止

2. **重启模式 (LOOP_RESTART)**：

   - 正向：0 → 1 → 2 → 3 → 0 → 1 → 2 → 3 ...
   - 反向：3 → 2 → 1 → 0 → 3 → 2 → 1 → 0 ...

3. **逆序模式 (LOOP_REVERSE)**：
   - 开始正向：0 → 1 → 2 → 3 → 3 → 2 → 1 → 0 → 0 → 1 ...
   - 开始反向：3 → 2 → 1 → 0 → 0 → 1 → 2 → 3 → 3 → 2 ...

#### 注意事项

1. 在循环导航过程中可以随时发送暂停命令：

```bash
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 1"  # PAUSE
```

2. 循环模式切换：

- 可以通过发送新的 START_SEQUENCE 命令切换循环模式
- 切换时会重新开始导航序列

3. 安全考虑：

- 建议在首次使用新的循环模式时先在低速模式下测试
- 确保路径上没有障碍物
- 准备好紧急停止手段

### 命令详解

#### NavigationCommand 消息结构

```plaintext
# 命令类型
uint8 command              # 命令类型
int32 waypoint_index      # 目标航点索引 (用于GOTO_WAYPOINT)
geometry_msgs/PoseStamped target_pose  # 目标位置 (用于GOTO_POSE)
bool reverse              # 序列执行方向 (用于START_SEQUENCE)
uint8 loop_mode           # 循环模式 (0: 单程, 1: 重启, 2: 逆序)
string params             # 扩展参数

# 预定义的命令类型常量
uint8 STOP = 0           # 停止当前导航
uint8 PAUSE = 1         # 暂停导航
uint8 RESUME = 2        # 恢复导航
uint8 GOTO_WAYPOINT = 3 # 导航到指定航点
uint8 GOTO_POSE = 4     # 导航到指定位置
uint8 START_SEQUENCE = 5 # 开始执行航点序列
uint8 STOP_SEQUENCE = 6  # 停止序列执行

# 循环模式常量
uint8 LOOP_NONE = 0    # 不循环（单程）
uint8 LOOP_RESTART = 1 # 回到起点重新开始
uint8 LOOP_REVERSE = 2 # 逆序返回
```

#### 命令类型详解

| 命令           | 值  | 说明           | 所需参数           | 示例                                       |
| -------------- | --- | -------------- | ------------------ | ------------------------------------------ |
| STOP           | 0   | 停止当前导航   | 无                 | `command: 0`                               |
| PAUSE          | 1   | 暂停导航任务   | 无                 | `command: 1`                               |
| RESUME         | 2   | 恢复导航任务   | 无                 | `command: 2`                               |
| GOTO_WAYPOINT  | 3   | 导航到指定航点 | waypoint_index     | `command: 3, waypoint_index: 2`            |
| GOTO_POSE      | 4   | 导航到指定位置 | target_pose        | `command: 4, target_pose: {...}`           |
| START_SEQUENCE | 5   | 开始序列导航   | reverse, loop_mode | `command: 5, reverse: false, loop_mode: 2` |
| STOP_SEQUENCE  | 6   | 停止序列导航   | 无                 | `command: 6`                               |

#### 循环模式说明

| 模式         | 值  | 行为说明               | 适用场景       |
| ------------ | --- | ---------------------- | -------------- |
| LOOP_NONE    | 0   | 执行一次序列后停止     | 单次任务执行   |
| LOOP_RESTART | 1   | 完成后返回起点重新开始 | 重复性巡检任务 |
| LOOP_REVERSE | 2   | 按相反顺序返回起点     | 往返式巡检任务 |

### 配置说明

#### Launch 文件参数

```xml
<!-- wp_nav_controller.launch -->
<arg name="use_rviz" default="true"/>           <!-- 是否启动RViz -->
<arg name="update_rate" default="10.0"/>        <!-- 状态更新频率 -->
<arg name="goal_tolerance" default="0.5"/>      <!-- 目标达到容差 -->
<arg name="timeout" default="300.0"/>          <!-- 导航超时时间 -->
<arg name="service_prefix" default="waypoint"/> <!-- 服务前缀 -->

<!-- 启动导航控制器节点 -->
<node pkg="wp_nav_controller" type="wp_nav_controller_node" name="wp_nav_controller">
    <!-- 节点参数配置 -->
    <param name="update_rate" value="$(arg update_rate)"/>
    <param name="goal_tolerance" value="$(arg goal_tolerance)"/>
    <param name="timeout" value="$(arg timeout)"/>
    <param name="service_prefix" value="$(arg service_prefix)"/>
</node>
```

#### 核心参数说明

| 参数名         | 类型   | 默认值     | 说明                     |
| -------------- | ------ | ---------- | ------------------------ |
| update_rate    | double | 10.0       | 状态更新频率(Hz)         |
| goal_tolerance | double | 0.5        | 到达目标点判定距离(米)   |
| timeout        | double | 300.0      | 单个导航任务超时时间(秒) |
| service_prefix | string | "waypoint" | 服务调用前缀             |

#### 状态监控

通过订阅状态话题可获取导航器实时状态：

```bash
# 订阅状态信息
rostopic echo /wp_nav/status

# 状态信息格式
Navigation Status:
  Current Waypoint: <index>
  Navigation State: <Navigating/Paused/Idle>
  Sequence State: <Forward/Reverse>
  Loop Mode: <No Loop/Restart/Reverse>
  Current Goal: (<x>, <y>)  # 当前导航目标坐标
```

#### RViz 配置

可以通过修改 `rviz/wp_nav.rviz` 文件自定义显示选项：

- 添加/移除显示项
- 修改显示样式
- 配置坐标系
- 自定义可视化效果

#### 动态参数调整

以下参数可以在运行时动态调整：

```bash
# 修改目标达到容差
rosparam set /wp_nav_controller/goal_tolerance 0.3

# 修改状态更新频率
rosparam set /wp_nav_controller/update_rate 20.0

# 修改导航超时时间
rosparam set /wp_nav_controller/timeout 400.0
```

#### 安全配置建议

1. 导航参数设置：

   - 根据机器人实际情况设置适当的目标容差
   - 为避免死锁设置合理的超时时间
   - 考虑设置最大速度限制

2. 路径规划参数：
   - 配置适当的避障距离
   - 调整路径规划的代价值
   - 设置合理的加速度限制

## 接口文档

### 发布的话题（Publishers）

| 话题名                   | 消息类型                  | 说明           |
| ------------------------ | ------------------------- | -------------- |
| /wp_nav/status           | std_msgs/String           | 导航器状态信息 |
| /wp_nav/current_waypoint | std_msgs/Int32            | 当前航点索引   |
| /waypoint_markers        | visualization_msgs/Marker | 航点可视化标记 |

### 订阅的话题（Subscribers）

| 话题名              | 消息类型                              | 说明         |
| ------------------- | ------------------------------------- | ------------ |
| /wp_nav/command     | NavigationCommand                     | 导航命令接收 |
| /move_base/result   | move_base_msgs/MoveBaseActionResult   | 导航结果反馈 |
| /move_base/feedback | move_base_msgs/MoveBaseActionFeedback | 导航过程反馈 |

### 服务调用（Service Clients）

| 服务名                       | 服务类型           | 说明             |
| ---------------------------- | ------------------ | ---------------- |
| /waypoint/get_num_waypoint   | GetNumOfWaypoints  | 获取航点总数     |
| /waypoint/get_waypoint_index | GetWaypointByIndex | 根据索引获取航点 |
| /waypoint/get_waypoint_name  | GetWaypointByName  | 根据名称获取航点 |

### Action 客户端

| Action 名  | Action 类型                   | 说明           |
| ---------- | ----------------------------- | -------------- |
| /move_base | move_base_msgs/MoveBaseAction | 导航动作客户端 |

### 参数服务器

| 参数名           | 类型   | 说明         |
| ---------------- | ------ | ------------ |
| ~/update_rate    | double | 状态更新频率 |
| ~/goal_tolerance | double | 目标达到容差 |
| ~/timeout        | double | 导航超时时间 |
| ~/service_prefix | string | 服务名称前缀 |

## 扩展开发

### 1. 添加新的循环导航模式

1. 在 NavigationCommand.msg 中添加新的模式定义：

```plaintext
# 在消息定义中添加新的循环模式
uint8 LOOP_CUSTOM = 3  # 自定义循环模式
```

2. 修改 WaypointNavigator 类：

```cpp
class WaypointNavigator {
public:
    // 添加新的循环模式处理逻辑
    void processCustomLoop() {
        // 实现自定义循环逻辑
    }

protected:
    void processNextWaypoint() {
        switch(loopMode_) {
            case NavigationCommand::LOOP_CUSTOM:
                processCustomLoop();
                break;
            // 其他 case 保持不变...
        }
    }
};
```

3. 更新状态发布：

```cpp
void publishStatus() {
    std::stringstream ss;
    // ... 其他状态信息 ...
    ss << "Loop Mode: ";
    switch(loopMode_) {
        case NavigationCommand::LOOP_CUSTOM:
            ss << "Custom";
            break;
        // 其他 case 保持不变...
    }
}
```

### 2. 自定义导航行为

1. 创建派生类：

```cpp
class CustomNavigator : public WaypointNavigator {
public:
    CustomNavigator(ros::NodeHandle& nh) : WaypointNavigator(nh) {
        // 初始化自定义参数
    }

    // 重写导航行为
    bool navigateToWaypoint(int wpIndex) override {
        // 自定义导航实现
        return true;
    }

    // 添加新功能
    void executeCustomBehavior() {
        // 实现自定义行为
    }

protected:
    // 重写回调函数
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                     const move_base_msgs::MoveBaseResultConstPtr& result) override {
        // 自定义完成处理
    }
};
```

2. 使用自定义导航器：

```cpp
int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_nav_node");
    ros::NodeHandle nh;

    CustomNavigator navigator(nh);
    // 使用自定义功能
    navigator.executeCustomBehavior();

    ros::spin();
    return 0;
}
```

### 3. 添加新的状态监控

1. 创建自定义状态消息：

```cpp
// 在导航器类中添加新的状态发布
class WaypointNavigator {
private:
    ros::Publisher customStatusPub_;

    void publishCustomStatus() {
        std_msgs::String msg;
        std::stringstream ss;

        ss << "Custom Status:\n"
           << "  Custom Data: " << customData_ << "\n"
           << "  Extra Info: " << extraInfo_;

        msg.data = ss.str();
        customStatusPub_.publish(msg);
    }
};
```

2. 实现状态监控节点：

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class StatusMonitor:
    def __init__(self):
        rospy.init_node('nav_status_monitor')
        rospy.Subscriber('/wp_nav/status', String, self.status_callback)
        # 订阅自定义状态
        rospy.Subscriber('/wp_nav/custom_status', String, self.custom_status_callback)

    def status_callback(self, msg):
        # 处理标准状态信息
        pass

    def custom_status_callback(self, msg):
        # 处理自定义状态信息
        pass

if __name__ == '__main__':
    monitor = StatusMonitor()
    rospy.spin()
```

### 4. 扩展配置系统

1. 添加新的配置参数：

```xml
<!-- 在launch文件中添加新参数 -->
<arg name="custom_param" default="default_value"/>
<node pkg="wp_nav_controller" type="wp_nav_controller_node" name="wp_nav_controller">
    <param name="custom_param" value="$(arg custom_param)"/>
</node>
```

2. 在代码中使用新参数：

```cpp
class WaypointNavigator {
private:
    void loadParameters() {
        ros::NodeHandle private_nh("~");
        private_nh.param<string>("custom_param", customParam_, "default_value");
    }
};
```

### 5. 添加自定义命令

1. 更新命令消息：

```plaintext
# 在NavigationCommand.msg中添加
uint8 CUSTOM_COMMAND = 7
string custom_params
```

2. 实现命令处理：

```cpp
void commandCallback(const NavigationCommand::ConstPtr& msg) {
    switch(msg->command) {
        case NavigationCommand::CUSTOM_COMMAND:
            handleCustomCommand(msg->custom_params);
            break;
        // 其他case保持不变...
    }
}
```

3. 使用新命令：

```bash
# 发送自定义命令
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 7
custom_params: 'custom_action'"
```

## 故障排除

### 1. 启动问题

#### 服务连接失败

```bash
# 检查服务是否存在
rosservice list | grep waypoint

# 检查航点管理器是否运行
rosnode list | grep wp_waypoint_manager

# 测试服务调用
rosservice call /waypoint/get_num_waypoint

# 手动启动航点管理器
rosrun amr_map_tools wp_waypoint_manager
```

#### 导航器启动失败

```bash
# 检查ROS日志
roscd wp_nav_controller
cat ~/.ros/log/latest/rosout.log | grep wp_nav_controller

# 检查 move_base 是否运行
rosnode list | grep move_base

# 检查 TF 树
rosrun tf view_frames
evince frames.pdf
```

#### 参数加载问题

```bash
# 检查参数是否正确加载
rosparam list | grep wp_nav
rosparam get /wp_nav_controller/goal_tolerance
rosparam get /wp_nav_controller/timeout
```

### 2. 导航问题

#### 导航精度问题

```bash
# 检查目标达到容差设置
rosparam get /wp_nav_controller/goal_tolerance

# 检查当前位置与目标位置
rosrun tf tf_echo map base_link

# 可视化导航路径
rosrun rviz rviz -d `rospack find wp_nav_controller`/rviz/wp_nav.rviz
```

#### 导航超时

- 检查 timeout 参数设置
- 确认路径是否可达
- 检查障碍物情况
- 查看 CPU 负载

#### 路径规划失败

- 检查地图更新情况
- 确认目标点在地图范围内
- 检查 costmap 配置
- 查看传感器数据

### 3. 循环导航问题

#### 循环模式切换失败

```bash
# 检查当前循环模式
rostopic echo /wp_nav/status | grep "Loop Mode"

# 尝试停止当前序列
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 6"

# 重新开始循环导航
rostopic pub /wp_nav/command wp_nav_controller/NavigationCommand "command: 5
reverse: false
loop_mode: 2"
```

#### 逆序导航问题

- 检查航点序列完整性
- 确认航点可达性
- 验证路径规划结果

#### 循环中断

- 检查导航状态日志
- 确认电池电量
- 查看错误消息

### 4. 系统资源问题

#### 内存使用

```bash
# 检查节点内存使用
top -p `pidof wp_nav_controller_node`

# 查看系统内存
free -h
```

#### CPU 负载

```bash
# 检查CPU使用情况
htop

# 查看具体进程
ps aux | grep wp_nav
```

#### 网络延迟

```bash
# 检查ROS网络连接
rostopic hz /wp_nav/status
```

## 常见问题

### 1. 导航相关问题

Q: 如何调整导航精度？
A: 可以通过修改 goal_tolerance 参数来调整：

```bash
rosparam set /wp_nav_controller/goal_tolerance 0.3  # 更精确的导航
```

Q: 导航过程中如何处理障碍物？
A: 系统会自动避障，但可以通过以下方式优化：

- 调整 costmap 参数
- 修改路径规划参数
- 使用动态避障算法

Q: 如何处理导航卡死？
A: 可以采取以下步骤：

1. 发送暂停命令
2. 检查传感器数据
3. 重新规划路径
4. 如果问题持续，可以取消当前目标并重新发送

### 2. 循环导航问题

Q: 如何在循环导航中添加等待时间？
A: 可以通过以下方式实现：

```python
def start_sequence_with_delay(self):
    # 发送导航命令
    self.start_sequence_with_loop(loop_mode=2)
    # 在每个航点处等待
    rospy.sleep(5.0)  # 等待5秒
```

Q: 循环导航中如何跳过某些航点？
A: 可以自定义导航序列：

```python
def create_custom_sequence(self, waypoints):
    for wp in waypoints:
        # 发送选定的航点
        self.goto_waypoint(wp)
        rospy.sleep(1.0)
```

Q: 如何在不同循环模式间切换？
A: 建议按以下步骤操作：

1. 先停止当前序列
2. 等待状态更新
3. 启动新的循环模式

### 3. 配置问题

Q: 如何持久化保存配置？
A: 可以通过以下方式：

1. 修改 launch 文件中的默认参数
2. 创建自定义参数文件
3. 使用 rosparam dump 保存当前配置

Q: launch 文件中的参数无效？
A: 检查以下几点：

1. 参数名称是否正确
2. 命名空间是否正确
3. launch 文件加载顺序
4. 参数类型是否匹配

### 4. 开发相关问题

Q: 如何添加新的导航行为？
A: 参考扩展开发章节，主要步骤：

1. 继承 WaypointNavigator 类
2. 重写相关方法
3. 添加新的功能实现

Q: 如何集成自定义的状态监控？
A: 可以通过以下方式：

1. 创建新的状态发布话题
2. 实现自定义状态消息
3. 添加状态监控节点

### 5. 系统集成问题

Q: 如何与其他导航系统集成？
A: 可以通过以下方式：

1. 使用 ROS 话题进行通信
2. 实现接口适配器
3. 创建桥接节点

Q: 如何处理多机器人场景？
A: 建议采用以下方案：

1. 使用不同的命名空间
2. 实现协调管理节点
3. 添加碰撞避免机制

## 贡献指南

### 开发流程

1. Fork 项目仓库

```bash
# 1. Fork 仓库到你的 GitHub 账户

# 2. 克隆你的 fork
git clone https://github.com/YOUR_USERNAME/wp_nav_controller.git

# 3. 添加上游仓库
cd wp_nav_controller
git remote add upstream https://github.com/XKHoshizora/wp_nav_controller.git
```

2. 创建功能分支

```bash
# 更新主分支
git checkout master
git pull upstream master

# 创建新分支
git checkout -b feature/your-feature-name
```

3. 开发流程

```bash
# 定期同步上游更改
git fetch upstream
git rebase upstream/master

# 提交更改
git add .
git commit -m "feat: your commit message"
```

4. 提交 Pull Request

- 确保代码通过所有测试
- 撰写清晰的 PR 描述
- 关联相关的 Issue

### 代码规范

#### C++代码风格

1. 命名规范

```cpp
// 类名使用大驼峰
class WaypointNavigator {};

// 方法名使用小驼峰
void processNextWaypoint();

// 成员变量使用下划线后缀
int waypoint_index_;

// 常量使用全大写
const int MAX_WAYPOINTS = 100;
```

2. 格式化规则

```cpp
// 使用 clang-format 格式化代码
clang-format -i src/*.cpp include/wp_nav_controller/*.h

// 缩进使用空格
void function() {
    if (condition) {
        doSomething();
    }
}
```

3. 注释规范

```cpp
/**
 * @brief 类的简要说明
 *
 * 详细说明，包括功能、用途等
 */
class MyClass {
public:
    /**
     * @brief 方法说明
     * @param param1 参数1说明
     * @return 返回值说明
     */
    int method(int param1);
};
```

4. 错误处理

```cpp
try {
    // 操作代码
    performOperation();
} catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error: " << e.what());
    // 进行错误恢复
}
```

#### Python 代码风格

1. 遵循 PEP 8 规范

```python
# 类名使用大驼峰
class NavigationController:

    # 方法名使用下划线
    def process_navigation_command(self):
        pass

    # 私有方法前缀下划线
    def _internal_method(self):
        pass
```

2. 文档字符串

```python
class NavigationController:
    """导航控制器类

    此类用于控制机器人导航行为，包括：
    - 单点导航
    - 序列导航
    - 循环导航
    """

    def navigate_to_point(self, point):
        """导航到指定点位

        Args:
            point (Point): 目标点位坐标

        Returns:
            bool: 导航是否成功
        """
        pass
```

### 问题报告

创建 Issue 时请包含：

1. 问题描述
2. 复现步骤
3. 期望行为
4. 实际行为
5. 环境信息
   - ROS 版本
   - 操作系统版本
   - 相关依赖版本
6. 相关日志
7. 如可能，提供最小复现代码
