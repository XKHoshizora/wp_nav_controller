# Keyboard Control for Waypoint Navigation

`keyboard_control.py` 是一个基于 ROS 的键盘控制节点，允许用户通过键盘发送导航命令，方便快捷地控制机器人执行多种导航操作。

## 功能概述

通过键盘操作，您可以：

1. **控制导航状态**：
   - 暂停、恢复、停止导航。
2. **跳转到航点**：
   - 输入航点编号直接导航。
3. **动态查询航点数量**：
   - 实时显示可用航点的数量。
4. **切换导航模式**：
   - 切换为不同的导航模式（单次导航、循环导航等）。
5. **手动输入目标坐标**：
   - 输入 `(x, y)` 坐标，选择预定义方向（北、东、南、西），或者自定义 `yaw` 角。
6. **实时查看导航状态**：
   - 显示当前导航状态和目标信息。
7. **紧急停止**：
   - 立即取消所有导航任务。
8. **保存导航历史**：
   - 导出用户的导航操作记录，便于回溯或调试。

## 快速开始

### 环境需求

- ROS 运行环境。
- `wp_nav_controller` 功能包及其依赖项。
- Python 3。

### 安装与配置

1. 将 `keyboard_control.py` 放入 ROS 工作空间的 `scripts` 文件夹中，例如：

   ```
   wp_nav_controller/scripts/keyboard_control.py
   ```

2. 为脚本添加执行权限：

   ```bash
   chmod +x wp_nav_controller/scripts/keyboard_control.py
   ```

3. 确保 ROS 的 `wp_nav_controller` 包已正确编译：
   ```bash
   catkin_make
   ```

### 启动节点

运行以下命令

启动键盘控制节点：

```bash
rosrun wp_nav_controller keyboard_control.py
```

或者创建一个对应的 `launch` 文件，内容如下：

```xml
<launch>
    <node name="keyboard_control" pkg="wp_nav_controller" type="keyboard_control.py" output="screen"/>
</launch>
```

通过 `roslaunch` 启动：

```bash
roslaunch wp_nav_controller keyboard_control.launch
```

## 操作说明

启动后，终端会显示以下按键绑定和功能提示：

```
Waypoint Navigation Keyboard Control
-----------------------------------
按键操作:
  p: PAUSE       （暂停导航）
  r: RESUME      （恢复导航）
  s: STOP        （停止导航）
  n: NEXT        （跳转到下一个航点）
  t: GOTO_POSE   （手动输入目标坐标）
  l: LIST        （列出航点数量）
  m: MODE        （切换导航模式）
  e: EMERGENCY   （紧急停止）
  h: SAVE_HISTORY（保存导航历史）
  q: QUIT        （退出程序）
  1-9: 前往指定航点（无 0 号航点，支持多字符输入，如 '12' 表示航点 12）
```

### 使用示例

#### 控制导航

- **暂停导航**：按 `p` 键。
- **恢复导航**：按 `r` 键。
- **停止导航**：按 `s` 键。
- **紧急停止**：按 `e` 键。

#### 跳转到航点

- 按 `数字键` 输入目标航点编号，然后按 `Enter` 确认。例如：
  ```
  按 1 -> 按 2 -> 按 Enter  -> 导航到航点 12
  ```

#### 手动输入目标坐标

按 `t` 键后输入目标点的坐标：

```
Enter x coordinate: 2.5
Enter y coordinate: -1.5
Enter direction (n/e/s/w or yaw in radians): n
```

- **快捷方向**：

  - `n`: 北 (yaw = π/2)
  - `e`: 东 (yaw = 0)
  - `s`: 南 (yaw = -π/2)
  - `w`: 西 (yaw = π)

- **自定义方向**：直接输入 `yaw` 的数值，例如 `1.57`。

#### 列出航点数量

按 `l` 键，显示当前可用航点数量：

```
Number of available waypoints: 5
```

#### 保存导航历史

按 `h` 键将导航历史保存到 `nav_history.txt` 文件中。
