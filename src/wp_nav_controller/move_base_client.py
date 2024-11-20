#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty


class MoveBaseClient:
    """
    MoveBase客户端封装类
    提供导航相关的API接口
    """

    def __init__(self, node_name='move_base_client', action_name='move_base', feedback_callback=None, server_wait_timeout=None, anonymous=False):
        """
        初始化MoveBaseClient

        Args:
            node_name (str): ROS节点名称，默认为'move_base_client'
            action_name (str): action服务器名称，默认为'move_base'
            feedback_callback (callable, optional): 自定义的反馈处理函数，默认为None
                                                该函数接收一个参数：feedback.base_position (PoseStamped)
            server_wait_timeout (float, optional): 等待action服务器的超时时间(秒)
                                            None表示无限等待（默认值）
                                            设置具体数值则在超时后抛出异常
        """
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            # 使用用户指定的节点名称，并设置匿名标志
            # anonymous=True 会在节点名称后附加随机数，确保节点名称唯一
            rospy.init_node(node_name, anonymous)
            rospy.loginfo(f"Initialized ROS node: {rospy.get_name()}")

        # 创建action客户端
        self.client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        rospy.loginfo(f"Waiting for move_base action server: {action_name}")

        # 等待action服务器启动
        # 如果timeout是None，直接传给wait_for_server
        # 如果timeout不是None，则创建Duration对象
        if server_wait_timeout is not None:
            timeout_duration = rospy.Duration(server_wait_timeout)
            server_up = self.client.wait_for_server(timeout=timeout_duration)
        else:
            server_up = self.client.wait_for_server()

        if not server_up:
            rospy.logerr("Could not connect to move_base action server!")
            rospy.signal_shutdown("Could not connect to action server!")
        else:
            rospy.loginfo("Connected to move_base action server")

        # 初始化清除代价地图服务客户端
        self.clear_costmaps_srv = rospy.ServiceProxy(
            '/move_base/clear_costmaps', Empty)

        # 存储用户自定义的反馈回调函数
        self._custom_feedback_cb = feedback_callback

    def send_goal(self, goal):
        """
        发送MoveBaseGoal类型的导航目标

        Args:
            goal (MoveBaseGoal): 完整的导航目标对象，包含目标位姿等信息，需要外部构建

        Returns:
            bool: 发送成功返回True，否则返回False
        """
        try:
            self.client.send_goal(goal, feedback_cb=self._feedback_callback)
            rospy.loginfo("Navigation goal sent")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to send goal: {str(e)}")
            return False

    def wait_for_result(self, timeout=None):
        """
        等待导航结果

        Args:
            timeout (float, optional): 超时时间(秒)
                                     None表示无限等待（默认值）
                                     设置具体数值则在超时后返回

        Returns:
            bool: 导航成功返回True，失败或超时返回False
        """
        try:
            # 如果timeout为None，则无限等待
            # 如果timeout有具体数值，则转换为rospy.Duration
            timeout_duration = rospy.Duration(
                timeout) if timeout is not None else None

            # 等待结果
            finished_within_time = self.client.wait_for_result(
                timeout=timeout_duration)

            if not finished_within_time:
                rospy.logwarn("Navigation timed out!")
                return False

            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation succeeded!")
                return True
            else:
                rospy.logwarn(f"Navigation failed with state: {state}")
                return False

        except Exception as e:
            rospy.logerr(f"Error waiting for result: {str(e)}")
            return False

    def cancel_goal(self):
        """
        取消当前导航目标

        Returns:
            bool: 取消成功返回True
        """
        try:
            self.client.cancel_goal()
            rospy.loginfo("Navigation goal cancelled")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to cancel goal: {str(e)}")
            return False

    def clear_costmaps(self):
        """
        清除代价地图

        Returns:
            bool: 清除成功返回True
        """
        try:
            self.clear_costmaps_srv()
            rospy.loginfo("Costmaps cleared")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to clear costmaps: {str(e)}")
            return False

    def get_state(self):
        """
        获取当前导航状态

        Returns:
            int: 导航状态码，可能的值包括：
                PENDING=0    : 目标等待被执行
                ACTIVE=1     : 目标正在被执行
                PREEMPTED=2  : 目标被抢占（被新目标取代）
                SUCCEEDED=3  : 目标执行成功完成
                ABORTED=4    : 目标执行失败
                REJECTED=5   : 目标被拒绝（非法或无法执行）
                PREEMPTING=6 : 目标正在被抢占
                RECALLING=7  : 目标正在被取消
                RECALLED=8   : 目标已被成功取消
                LOST=9       : 目标丢失（服务器意外断开）
        """
        return self.client.get_state()

    def check_navigation_status(self):
        """
        检查当前导航状态

        Returns:
            dict: 包含导航状态信息的字典
                {
                    'status': int,      # GoalStatus 状态码
                    'status_text': str, # 状态描述文本
                    'is_active': bool,  # 是否正在导航
                    'is_finished': bool # 是否已结束(成功或失败)
                }
        """
        state = self.get_state()

        status_info = {
            'status': state,
            'status_text': '',
            'is_active': False,
            'is_finished': False
        }

        # PENDING = 0
        # 导航目标已被接收，正在等待执行
        # 通常在发送目标后的短暂时间内出现
        # 例如：等待机器人初始化或等待其他操作完成
        if state == GoalStatus.PENDING:
            status_info['status_text'] = 'Pending - Waiting for execution'

        # ACTIVE = 1
        # 导航正在执行中
        # 机器人正在按照规划的路径移动
        # 此时会收到持续的位置反馈
        elif state == GoalStatus.ACTIVE:
            status_info['status_text'] = 'Active - Navigation in progress'
            status_info['is_active'] = True

        # PREEMPTED = 2
        # 当前导航被取消或被新的导航目标抢占
        # 可能原因：
        # 1. 用户手动取消
        # 2. 发送了新的导航目标
        # 3. 调用了 cancel_goal()
        elif state == GoalStatus.PREEMPTED:
            status_info['status_text'] = 'Preempted - Navigation cancelled'
            status_info['is_finished'] = True

        # SUCCEEDED = 3
        # 导航成功完成
        # 机器人已到达目标位置和姿态
        # 这是一个正常的终止状态
        elif state == GoalStatus.SUCCEEDED:
            status_info['status_text'] = 'Succeeded - Navigation completed successfully'
            status_info['is_finished'] = True

        # ABORTED = 4
        # 导航执行失败
        # 可能原因：
        # 1. 目标点被障碍物堵住
        # 2. 无法规划出可行路径
        # 3. 机器人被卡住
        # 4. 局部路径规划失败
        # 这种情况通常需要重新尝试或清除代价地图
        elif state == GoalStatus.ABORTED:
            status_info['status_text'] = 'Aborted - Navigation failed'
            status_info['is_finished'] = True

        # REJECTED = 5
        # 导航目标被拒绝，无法执行
        # 可能原因：
        # 1. 目标点在地图外
        # 2. 目标点在障碍物内
        # 3. 目标姿态非法
        # 4. 参数配置错误
        elif state == GoalStatus.REJECTED:
            status_info['status_text'] = 'Rejected - Navigation goal rejected'
            status_info['is_finished'] = True

        # PREEMPTING = 6
        # 正在执行取消操作
        # 这是一个临时状态，表示正在从 ACTIVE 转换到 PREEMPTED
        # 机器人可能正在执行减速等操作
        elif state == GoalStatus.PREEMPTING:
            status_info['status_text'] = 'Preempting - Navigation being cancelled'
            status_info['is_active'] = True

        # RECALLING = 7
        # 正在召回未执行的导航目标
        # 这是一个临时状态，发生在取消 PENDING 状态的目标时
        # 不常见，因为 PENDING 状态通常持续时间很短
        elif state == GoalStatus.RECALLING:
            status_info['status_text'] = 'Recalling - Navigation being recalled'
            status_info['is_active'] = True

        # RECALLED = 8
        # 未执行的导航目标已被成功召回
        # 表示一个 PENDING 状态的目标被成功取消
        # 不常见，因为通常目标会很快进入 ACTIVE 状态
        elif state == GoalStatus.RECALLED:
            status_info['status_text'] = 'Recalled - Navigation recalled successfully'
            status_info['is_finished'] = True

        # LOST = 9
        # 与导航服务器失去连接
        # 可能原因：
        # 1. move_base 节点崩溃
        # 2. ROS 网络通信问题
        # 3. 系统资源不足
        # 这种情况需要检查 ROS 系统状态和网络连接
        elif state == GoalStatus.LOST:
            status_info['status_text'] = 'Lost - Navigation connection lost'
            status_info['is_finished'] = True

        # 未知状态
        # 正常情况下不应该出现
        # 如果出现，可能是代码版本不匹配或系统出现严重错误
        else:
            status_info['status_text'] = f'Unknown state: {state}'

        return status_info

    def get_node_info(self):
        """
        获取节点信息

        Returns:
            dict: 节点信息字典
                {
                    'node_name': str,     # 当前节点名称
                    'action_name': str,    # 使用的action服务器名称
                    'namespace': str       # 节点的命名空间
                }
        """
        return {
            'node_name': rospy.get_name(),                # 获取完整的节点名称
            'action_name': self.client.action_client.ns,  # 获取action服务器名称
            'namespace': rospy.get_namespace()            # 获取节点的命名空间
        }

    def _feedback_callback(self, feedback):
        """
        内部导航反馈回调函数

        Args:
            feedback: 导航器反馈信息
        """
        # 如果用户提供了回调函数，则调用它
        if self._custom_feedback_cb is not None:
            try:
                self._custom_feedback_cb(feedback.base_position)
            except Exception as e:
                rospy.logerr(f"Error in custom feedback callback: {str(e)}")


if __name__ == '__main__':
    try:
        # 创建 MoveBaseClient 实例
        # 定义一个简单的反馈回调函数
        def feedback_cb(pose):
            rospy.loginfo(f"Robot position - x: {pose.pose.position.x:.2f}, "
                          f"y: {pose.pose.position.y:.2f}")

        client = MoveBaseClient(feedback_callback=feedback_cb)

        # 示例 1: 发送单个导航目标
        def send_single_goal():
            rospy.loginfo("\n=== Example 1: Single Navigation Goal ===")

            # 创建导航目标
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            # 设置目标位置和方向
            goal.target_pose.pose.position.x = 1.0
            goal.target_pose.pose.position.y = 1.0
            goal.target_pose.pose.position.z = 0.0

            # 设置四元数，这里设置朝向为0度（正前方）
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0

            # 发送目标并等待结果
            if client.send_goal(goal):
                # 等待导航结果，设置60秒超时
                if client.wait_for_result(timeout=60.0):
                    rospy.loginfo("Navigation completed successfully!")
                else:
                    rospy.logwarn("Navigation timed out!")
            else:
                rospy.logerr("Failed to send goal!")

        # 示例 2: 状态监控示例
        def monitor_navigation_status():
            rospy.loginfo("\n=== Example 2: Status Monitoring ===")

            # 创建并发送一个简单的目标
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 2.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.orientation.w = 1.0

            if client.send_goal(goal):
                # 监控导航状态直到完成或超时
                rate = rospy.Rate(1)  # 1Hz 更新频率
                start_time = rospy.Time.now()
                timeout = rospy.Duration(30.0)  # 30秒超时

                while not rospy.is_shutdown():
                    status = client.check_navigation_status()
                    rospy.loginfo(
                        f"Navigation status: {status['status_text']}")

                    if status['is_finished']:
                        rospy.loginfo("Navigation finished!")
                        break

                    if (rospy.Time.now() - start_time) > timeout:
                        rospy.logwarn("Status monitoring timed out!")
                        client.cancel_goal()
                        break

                    rate.sleep()

        # 示例 3: 错误恢复示例
        def error_recovery_example():
            rospy.loginfo("\n=== Example 3: Error Recovery ===")

            # 首先清除代价地图
            client.clear_costmaps()
            rospy.sleep(1.0)  # 等待清除完成

            # 创建一个导航目标
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 3.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.orientation.w = 1.0

            max_attempts = 3
            attempt = 0

            while attempt < max_attempts and not rospy.is_shutdown():
                attempt += 1
                rospy.loginfo(f"Navigation attempt {attempt}/{max_attempts}")

                if client.send_goal(goal):
                    # 等待结果
                    if client.wait_for_result(timeout=30.0):
                        status = client.check_navigation_status()
                        if status['status'] == GoalStatus.SUCCEEDED:
                            rospy.loginfo("Navigation succeeded!")
                            break

                    # 如果失败，尝试恢复
                    rospy.logwarn("Navigation failed, attempting recovery...")
                    client.cancel_goal()
                    client.clear_costmaps()
                    rospy.sleep(2.0)  # 等待恢复
                else:
                    rospy.logerr("Failed to send goal!")
                    break

        # 示例 4: 获取节点信息
        def print_node_info():
            rospy.loginfo("\n=== Example 4: Node Information ===")
            info = client.get_node_info()
            rospy.loginfo("Node information:")
            for key, value in info.items():
                rospy.loginfo(f"  {key}: {value}")

        # 运行所有示例
        print_node_info()
        rospy.sleep(1.0)

        send_single_goal()
        rospy.sleep(2.0)

        monitor_navigation_status()
        rospy.sleep(2.0)

        error_recovery_example()

        rospy.loginfo("All examples completed!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test interrupted!")
    except Exception as e:
        rospy.logerr(f"Error during navigation test: {str(e)}")
