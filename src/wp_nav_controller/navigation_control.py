#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from wp_nav_controller.move_base_client import MoveBaseClient
from wp_nav_controller.waypoint_service import WaypointServiceManager


class NavigationState:
    """导航状态枚举类
    包含三个基本状态：
        - IDLE: 空闲状态，可以开始新的导航任务
        - NAVIGATING: 正在执行导航任务
        - PAUSED: 导航任务暂停，可以选择继续或停止

    导航基础状态转移图：
           ←  (停止导航/导航完成)  ←
          ↓                       ↑
        空闲  →  (开始导航)  →  正在导航    ←
          ↑                       ↓        ↑
           ←    (停止导航)    ← 暂停中 → 恢复导航

    导致状态转移的动作：
        IDLE -> NAVIGATING: 开始导航
        NAVIGATING -> PAUSED: 暂停导航
        NAVIGATING -> IDLE: 停止导航/导航完成
        PAUSED -> NAVIGATING: 恢复导航
        PAUSED -> IDLE: 停止导航
    """
    IDLE = "IDLE"               # 空闲状态（初始状态、导航完成或停止后的状态）
    NAVIGATING = "NAVIGATING"   # 正在导航（导航任务执行中）
    PAUSED = "PAUSED"           # 导航暂停（暂停导航后的状态）


class NavigationControl:
    """导航控制类

    提供机器人导航的高级控制功能，包括：
    1. 导航控制：开始导航、暂停导航、继续导航、停止导航
    2. 目标设置：支持导航到指定位姿或预定义航点
    3. 状态管理：维护导航状态机，确保状态转换的正确性
    4. 反馈机制：提供导航过程中的状态和位置反馈
    """

    def __init__(self, node_name='navigation_control', base_frame='base_link'):
        """初始化导航控制器

        Args:
            node_name (str): ROS节点名称，默认为'navigation_control'
            base_frame (str): 机器人基座坐标系名称，默认为'base_link'
        """
        # 初始化导航客户端，设置反馈回调
        self._move_base = MoveBaseClient(
            node_name=node_name,
            feedback_callback=self._navigation_feedback_callback
        )

        # 初始化航点服务管理器
        self._waypoint_manager = WaypointServiceManager(
            node_name=node_name,
            services=['info', 'navigation']
        )

        # 保存基座坐标系名称
        self._base_frame = base_frame

        # 状态相关变量
        self._state = NavigationState.IDLE    # 初始状态为空闲
        self._current_goal = None             # 当前导航目标
        self._paused_goal = None              # 暂停时保存的导航目标
        self._paused_position = None          # 暂停时的机器人位置
        self._navigation_thread = None        # 导航执行线程
        self._state_lock = threading.Lock()   # 状态访问锁

        # tf相关对象
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # 用户设置的导航反馈回调函数
        self._feedback_callback = None

        rospy.loginfo("Navigation control initialized")

    def set_feedback_callback(self, callback):
        """设置导航反馈回调函数

        Args:
            callback (callable): 回调函数，接收参数：(current_pose, navigation_state)
                   current_pose: 机器人当前位姿
               navigation_state: 当前导航状态
        """
        self._feedback_callback = callback

    def _update_state(self, new_state):
        """更新导航状态（线程安全）

        Args:
            new_state (str): 新的导航状态
        """
        with self._state_lock:
            old_state = self._state
            if new_state == old_state:
                rospy.logdebug(f"Ignoring redundant state change to {new_state}")
                return
            self._state = new_state

            # 状态转换信息
            if new_state == NavigationState.NAVIGATING and self._current_goal:
                goal_pose = self._current_goal.target_pose.pose
                rospy.loginfo(f"Navigation state changed: {old_state} -> {new_state} "
                              f"(Goal: x={goal_pose.position.x:.2f}, y={goal_pose.position.y:.2f})")
            elif new_state == NavigationState.PAUSED and self._paused_position:
                pause_pose = self._paused_position.pose
                rospy.loginfo(f"Navigation state changed: {old_state} -> {new_state} "
                              f"(Pause position: x={pause_pose.position.x:.2f}, y={pause_pose.position.y:.2f})")
            else:
                rospy.loginfo(
                    f"Navigation state changed: {old_state} -> {new_state}")

    def get_state(self):
        """获取当前导航状态（线程安全）

        Returns:
            str: 当前导航状态
        """
        with self._state_lock:
            return self._state

    def _navigation_feedback_callback(self, pose):
        """处理导航反馈信息

        Args:
            pose: 机器人当前位姿
        """
        if self._feedback_callback:
            try:
                self._feedback_callback(pose, self.get_state())
            except Exception as e:
                rospy.logerr(f"Error in navigation feedback callback: {e}")

    def _get_current_robot_pose(self):
        """获取机器人当前位置

        通过tf2获取机器人在map坐标系下的实时位置

        Returns:
            PoseStamped: 机器人当前位姿，获取失败则返回None
        """
        try:
            # 获取机器人在map坐标系下的变换
            transform = self._tf_buffer.lookup_transform(
                "map",               # 目标坐标系
                self._base_frame,    # 源坐标系
                rospy.Time(0),       # 获取最新的变换
                rospy.Duration(0.5)  # 超时时间0.5秒
            )

            # 创建一个在机器人基座坐标系下的原点位姿
            pose = PoseStamped()
            pose.header.frame_id = self._base_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.orientation.w = 1.0

            # 将位姿转换到map坐标系
            map_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

            return map_pose

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get robot pose: {e}")
            return None

    def _create_move_base_goal(self, pose):
        """创建导航目标消息

        Args:
            pose (geometry_msgs/Pose): 目标位姿

        Returns:
            MoveBaseGoal: 导航目标消息
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return goal

    def _navigation_thread_func(self):
        """导航执行线程函数

        负责:
        1. 发送导航目标
        2. 监控导航状态
        3. 处理导航结果
        4. 响应暂停/继续/停止命令
        """
        if not self._current_goal:
            rospy.logerr("No navigation goal set")
            self._update_state(NavigationState.IDLE)
            return

        try:
            # 发送导航目标
            if not self._move_base.send_goal(self._current_goal):
                rospy.logerr("Failed to send navigation goal")
                self._update_state(NavigationState.IDLE)
                return

            # 监控导航状态
            while not rospy.is_shutdown():
                status = self._move_base.check_navigation_status()

                # 检查导航是否完成
                if status['is_finished']:
                    if status['status'] == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Navigation completed successfully")
                    else:
                        rospy.logwarn(
                            f"Navigation ended with status: {status['status_text']}")
                    break

                # 检查是否暂停
                if self.get_state() == NavigationState.PAUSED:
                    rospy.loginfo("Navigation paused")
                    return  # 保持在PAUSED状态

                rospy.sleep(0.1)

        except Exception as e:
            rospy.logerr(f"Error during navigation: {e}")
        finally:
            # 如果不是暂停状态，则回到空闲状态
            if self.get_state() != NavigationState.PAUSED:
                self._current_goal = None
                self._update_state(NavigationState.IDLE)

    def navigate_to_pose(self, pose):
        """导航到指定位姿

        Args:
            pose (geometry_msgs/Pose): 目标位姿

        Returns:
            bool: 是否成功启动导航

        状态转换：IDLE -> NAVIGATING
        """
        if self.get_state() != NavigationState.IDLE:
            rospy.logwarn(
                "Cannot start navigation: robot is not in IDLE state")
            return False

        try:
            self._current_goal = self._create_move_base_goal(pose)
            self._update_state(NavigationState.NAVIGATING)

            # 启动导航线程
            self._navigation_thread = threading.Thread(
                target=self._navigation_thread_func
            )
            self._navigation_thread.start()
            return True

        except Exception as e:
            rospy.logerr(f"Failed to start navigation: {e}")
            self._update_state(NavigationState.IDLE)
            return False

    def navigate_to_waypoint_name(self, waypoint_name):
        """导航到指定航点

        Args:
            waypoint_name (str): 航点名称

        Returns:
            bool: 是否成功启动导航

        状态转换：IDLE -> NAVIGATING
        """
        # 获取航点位姿
        pose = self._waypoint_manager.get_waypoint_by_name(waypoint_name)
        if not pose:
            rospy.logerr(f"Waypoint not found: {waypoint_name}")
            return False

        return self.navigate_to_pose(pose)

    def navigate_to_waypoint_index(self, index):
        """导航到指定索引的航点

        Args:
            index (int): 航点的索引值

        Returns:
            bool: 是否成功启动导航

        状态转换：IDLE -> NAVIGATING
        """
        # 获取航点信息
        waypoint = self._waypoint_manager.get_waypoint_by_index(index)
        if not waypoint:
            rospy.logerr(f"Waypoint not found at index: {index}")
            return False

        # waypoint是一个包含名称和位姿的元组(name, pose)
        name, pose = waypoint
        rospy.loginfo(f"Navigating to waypoint {name} at index {index}")

        return self.navigate_to_pose(pose)

    def pause_navigation(self):
        """暂停当前导航任务

        暂停时会记录：
        1. 当前的导航目标
        2. 机器人的当前位置（通过tf实时获取）

        Returns:
            bool: 是否成功暂停导航

        状态转换：NAVIGATING -> PAUSED
        """
        if self.get_state() != NavigationState.NAVIGATING:
            rospy.logwarn("Cannot pause: robot is not navigating")
            return False

        try:
            # 检查导航是否处于活动状态
            status = self._move_base.check_navigation_status()
            if status['is_active']:
                # 保存当前导航目标
                self._paused_goal = self._current_goal

                # 取消当前导航目标
                self._move_base.cancel_goal()

                # 停止后，获取并保存当前位置
                current_pose = self._get_current_robot_pose()
                if current_pose:
                    self._paused_position = current_pose
                    rospy.loginfo(
                        "Saved current robot position at pause point")
                else:
                    rospy.logwarn("Could not get current robot position")
                    self._paused_position = None

                # 更新状态为 PAUSED
                self._update_state(NavigationState.PAUSED)

                rospy.loginfo("Navigation paused - Goal and position saved")
                return True
            return False

        except Exception as e:
            rospy.logerr(f"Failed to pause navigation: {e}")
            self._paused_goal = None
            self._paused_position = None
            return False

    def resume_navigation(self):
        """继续已暂停的导航任务

        使用暂停时保存的导航目标继续导航。
        如果有记录暂停位置，会在日志中记录从暂停点到目标点的距离。

        Returns:
            bool: 是否成功继续导航

        状态转换：PAUSED -> NAVIGATING
        """
        if self.get_state() != NavigationState.PAUSED:
            rospy.logwarn("Cannot resume: navigation is not paused")
            return False

        try:
            if self._paused_goal:
                # 恢复之前的导航目标
                self._current_goal = self._paused_goal

                # 如果有暂停位置记录，计算距离
                if self._paused_position:
                    try:
                        # 计算暂停点到目标点的距离
                        dx = self._paused_position.pose.position.x - \
                            self._current_goal.target_pose.pose.position.x
                        dy = self._paused_position.pose.position.y - \
                            self._current_goal.target_pose.pose.position.y
                        distance = (dx * dx + dy * dy) ** 0.5
                        rospy.loginfo(
                            f"Resuming navigation - Distance to goal: {distance:.2f} meters")
                    except Exception as e:
                        rospy.logwarn(
                            f"Could not calculate distance to goal: {e}")

                # 更新状态为 NAVIGATING
                self._update_state(NavigationState.NAVIGATING)

                # 启动新的导航线程
                self._navigation_thread = threading.Thread(
                    target=self._navigation_thread_func
                )
                self._navigation_thread.start()

                # 清除暂停状态的记录
                self._paused_goal = None
                self._paused_position = None
                return True

            rospy.logerr("No paused goal found to resume")
            return False

        except Exception as e:
            rospy.logerr(f"Failed to resume navigation: {e}")
            return False

    def stop_navigation(self):
        """停止当前导航任务

        Returns:
            bool: 是否成功停止导航

        状态转换：NAVIGATING/PAUSED -> IDLE
        """
        current_state = self.get_state()
        if current_state == NavigationState.IDLE:
            rospy.logwarn("No active navigation to stop")
            return False

        try:
            # 取消当前导航目标
            self._move_base.cancel_goal()

            # 清理所有状态
            self._current_goal = None
            self._paused_goal = None
            self._paused_position = None
            self._update_state(NavigationState.IDLE)

            return True

        except Exception as e:
            rospy.logerr(f"Failed to stop navigation: {e}")
            return False

    def clear_costmaps(self):
        """清除导航代价地图

        Returns:
            bool: 是否成功清除代价地图
        """
        return self._move_base.clear_costmaps()


# 使用示例
if __name__ == '__main__':
    try:
        # 创建导航控制器
        navigator = NavigationControl()

        # 设置导航反馈回调
        def on_navigation_feedback(pose, state):
            rospy.loginfo(f"Robot state: {state}")
            rospy.loginfo(f"Position - x: {pose.pose.position.x:.2f}, "
                          f"y: {pose.pose.position.y:.2f}")

        navigator.set_feedback_callback(on_navigation_feedback)

        # 示例1：导航到自定义位姿
        target_pose = Pose()
        target_pose.position.x = 1.0
        target_pose.position.y = 1.0
        target_pose.orientation.w = 1.0

        if navigator.navigate_to_pose(target_pose):
            rospy.sleep(5.0)  # 等待5秒

            # 暂停导航
            if navigator.pause_navigation():
                rospy.loginfo("Navigation paused")
                rospy.sleep(2.0)  # 等待2秒

                # 继续导航
                if navigator.resume_navigation():
                    rospy.loginfo("Navigation resumed")
                    rospy.sleep(5.0)  # 等待5秒

                    # 停止导航
                    if navigator.stop_navigation():
                        rospy.loginfo("Navigation stopped")

        # 示例2：导航到航点
        if navigator.navigate_to_waypoint_name("kitchen"):
            rospy.loginfo("Navigating to kitchen waypoint")

        # 示例3：通过索引导航到航点
        if navigator.navigate_to_waypoint_index(0):
            rospy.loginfo("Navigating to waypoint at index 0")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation control test interrupted")
    except Exception as e:
        rospy.logerr(f"Error in navigation control test: {e}")
