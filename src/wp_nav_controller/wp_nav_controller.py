#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import argparse
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from wp_nav_controller.srv import NavigationCommand, NavigationCommandResponse
from wp_nav_controller.srv import SequenceCommand, SequenceCommandResponse
from wp_nav_controller.navigation_control import NavigationControl, NavigationState
from wp_nav_controller.sequence_navigation import NavigationSequence, SequenceMode


class WPNavControllerNode:
    def __init__(self):
        rospy.init_node('wp_nav_controller', anonymous=True)

        # 创建导航控制器实例
        self.nav_control = NavigationControl(node_name='wp_nav_controller')

        # 创建序列导航实例
        self.sequence_nav = NavigationSequence(self.nav_control)

        # 设置反馈回调函数
        self.nav_control.set_feedback_callback(self._nav_feedback_callback)
        self.sequence_nav.set_feedback_callback(
            self._sequence_feedback_callback)

        # 创建服务
        self.nav_service = rospy.Service('wp_nav_controller/navigation',
                                       NavigationCommand,
                                       self.handle_navigation_command)

        self.seq_service = rospy.Service('wp_nav_controller/sequence',
                                       SequenceCommand,
                                       self.handle_sequence_command)

        rospy.loginfo("Waypoint Navigation Controller services are ready")

        # 初始化参数解析器
        self.parser = self._setup_argument_parser()

    def _setup_argument_parser(self):
        parser = argparse.ArgumentParser(
            description='Waypoint Navigation Controller')
        subparsers = parser.add_subparsers(
            dest='command', help='Available commands')

        # 单点导航命令
        nav_parser = subparsers.add_parser(
            'nav', help='Single point navigation commands')
        nav_subparsers = nav_parser.add_subparsers(dest='nav_command')

        # 导航到航点名称
        wp_parser = nav_subparsers.add_parser(
            'waypoint', help='Navigate to waypoint by name')
        wp_parser.add_argument('name', type=str, help='Waypoint name')

        # 导航到航点索引
        idx_parser = nav_subparsers.add_parser(
            'index', help='Navigate to waypoint by index')
        idx_parser.add_argument('index', type=int, help='Waypoint index')

        # 导航到指定位姿
        pose_parser = nav_subparsers.add_parser(
            'pose', help='Navigate to specific pose')
        pose_parser.add_argument('x', type=float, help='X coordinate')
        pose_parser.add_argument('y', type=float, help='Y coordinate')
        pose_parser.add_argument(
            'yaw', type=float, help='Yaw angle in radians', default=0.0, nargs='?')

        # 导航控制命令
        nav_subparsers.add_parser('pause', help='Pause current navigation')
        nav_subparsers.add_parser('resume', help='Resume paused navigation')
        nav_subparsers.add_parser('stop', help='Stop current navigation')
        nav_subparsers.add_parser('clear', help='Clear costmaps')

        # 序列导航命令
        seq_parser = subparsers.add_parser(
            'seq', help='Sequence navigation commands')
        seq_subparsers = seq_parser.add_subparsers(dest='seq_command')

        # 创建序列
        create_parser = seq_subparsers.add_parser(
            'create', help='Create navigation sequence')
        create_parser.add_argument(
            'points', nargs='+', help='List of waypoint names or indices (prefix with @ for indices)')

        # 开始序列导航
        start_parser = seq_subparsers.add_parser(
            'start', help='Start sequence navigation')
        start_parser.add_argument(
            'mode', choices=['single', 'loop', 'roundtrip'], help='Navigation mode')
        start_parser.add_argument(
            '--loop', action='store_true', help='Enable looping for roundtrip mode')
        start_parser.add_argument(
            '--reverse', action='store_true', help='Start navigation in reverse direction')
        start_parser.add_argument(
            '--start', type=str, help='Start point (name or @index)')

        # 修改序列
        insert_parser = seq_subparsers.add_parser(
            'insert', help='Insert waypoint into sequence')
        insert_parser.add_argument('index', type=int, help='Insert position')
        insert_parser.add_argument(
            'point', type=str, help='Waypoint name or @index')

        remove_parser = seq_subparsers.add_parser(
            'remove', help='Remove waypoint from sequence')
        remove_parser.add_argument('index', type=int, help='Index to remove')

        reorder_parser = seq_subparsers.add_parser(
            'reorder', help='Reorder sequence')
        reorder_parser.add_argument(
            'indices', type=int, nargs='+', help='New index order')
        reorder_parser.add_argument(
            '--reverse', action='store_true', help='Reverse the result')

        # 序列控制命令
        seq_subparsers.add_parser('pause', help='Pause sequence navigation')
        seq_subparsers.add_parser('resume', help='Resume sequence navigation')
        seq_subparsers.add_parser('stop', help='Stop sequence navigation')
        seq_subparsers.add_parser('clear', help='Clear sequence')
        seq_subparsers.add_parser('info', help='Show sequence information')
        seq_subparsers.add_parser('state', help='Show pause state')

        return parser

    def handle_navigation_command(self, req):
        """处理导航控制命令"""
        resp = NavigationCommandResponse()
        try:
            if req.command == 'nav_to':
                if req.target_type == 'waypoint':
                    success = self.nav_control.navigate_to_waypoint_name(req.params[0])
                elif req.target_type == 'index':
                    success = self.nav_control.navigate_to_waypoint_index(int(req.params[0]))
                elif req.target_type == 'pose':
                    pose = self._create_pose(float(req.params[0]),
                                          float(req.params[1]),
                                          float(req.params[2]) if len(req.params) > 2 else 0.0)
                    success = self.nav_control.navigate_to_pose(pose)
                else:
                    success = False
                    resp.message = "Invalid navigation type"
            elif req.command == 'pause':
                success = self.nav_control.pause_navigation()
            elif req.command == 'resume':
                success = self.nav_control.resume_navigation()
            elif req.command == 'stop':
                success = self.nav_control.stop_navigation()
            elif req.command == 'clear':
                success = self.nav_control.clear_costmaps()
            else:
                success = False
                resp.message = "Unknown command"

            resp.success = success
            if not resp.message:
                resp.message = "Command executed successfully" if success else "Command failed"

        except Exception as e:
            resp.success = False
            resp.message = f"Error executing command: {str(e)}"
            rospy.logerr(resp.message)

        return resp

    def handle_sequence_command(self, req):
        """处理序列导航命令"""
        resp = SequenceCommandResponse()
        try:
            if req.command == 'create':
                self.sequence_nav.clear_sequence()
                for point in req.params:
                    if point.startswith('@'):
                        self.sequence_nav.add_waypoint_by_index(int(point[1:]))
                    else:
                        self.sequence_nav.add_waypoint_by_name(point)
                resp.success = True

            elif req.command == 'start':
                mode = SequenceMode(req.mode.upper())
                resp.success = self.sequence_nav.start_sequence(
                    mode=mode,
                    is_loop=req.loop,
                    reverse=req.reverse
                )

            elif req.command == 'pause':
                resp.success = self.sequence_nav.pause_sequence()

            elif req.command == 'resume':
                resp.success = self.sequence_nav.resume_sequence()

            elif req.command == 'stop':
                resp.success = self.sequence_nav.stop_sequence()

            elif req.command == 'info':
                info = self.sequence_nav.get_sequence_info()
                resp.success = True
                resp.data_keys = list(info.keys())  # 将字典的键转换为列表
                resp.data_values = [str(v) for v in info.values()]  # 将字典的值转换为字符串列表

            elif req.command == 'state':
                state = self.sequence_nav.get_pause_state()
                resp.success = True
                if state:
                    resp.data_keys = list(state.keys())  # 将字典的键转换为列表
                    resp.data_values = [str(v) for v in state.values()]  # 将字典的值转换为字符串列表
                else:
                    resp.data = {'state': 'No pause state available'}

            else:
                resp.success = False
                resp.message = "Unknown command"

            if not resp.message:
                resp.message = "Command executed successfully" if resp.success else "Command failed"

        except Exception as e:
            resp.success = False
            resp.message = f"Error executing command: {str(e)}"
            rospy.logerr(resp.message)

        return resp

    def _nav_feedback_callback(self, pose, state):
        """导航反馈回调"""
        rospy.logdebug(f"Navigation - State: {state}, "
                      f"Position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")

    def _sequence_feedback_callback(self, current_point, sequence_info):
        """序列导航反馈回调"""
        rospy.logdebug(f"Sequence Navigation - Current: {current_point}, "
                      f"Progress: {sequence_info['current_index'] + 1}/{sequence_info['total_points']}")

    def _create_pose(self, x: float, y: float, yaw: float = 0.0) -> Pose:
        """创建位姿消息"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.orientation = Quaternion(*q)
        return pose

    def execute_command(self, args):
        try:
            if args.command == 'nav':
                self._handle_nav_command(args)
            elif args.command == 'seq':
                self._handle_seq_command(args)
            else:
                rospy.logerr("Invalid command")
                return False
            return True
        except Exception as e:
            rospy.logerr(f"Error executing command: {e}")
            return False

    def _handle_nav_command(self, args):
        if args.nav_command == 'waypoint':
            self.nav_control.navigate_to_waypoint_name(args.name)
        elif args.nav_command == 'index':
            self.nav_control.navigate_to_waypoint_index(args.index)
        elif args.nav_command == 'pose':
            pose = self._create_pose(args.x, args.y, args.yaw)
            self.nav_control.navigate_to_pose(pose)
        elif args.nav_command == 'pause':
            self.nav_control.pause_navigation()
        elif args.nav_command == 'resume':
            self.nav_control.resume_navigation()
        elif args.nav_command == 'stop':
            self.nav_control.stop_navigation()
        elif args.nav_command == 'clear':
            self.nav_control.clear_costmaps()

    def _handle_seq_command(self, args):
        if args.seq_command == 'create':
            self.sequence_nav.clear_sequence()
            for point in args.points:
                if point.startswith('@'):
                    # 索引方式
                    self.sequence_nav.add_waypoint_by_index(int(point[1:]))
                else:
                    # 名称方式
                    self.sequence_nav.add_waypoint_by_name(point)

        elif args.seq_command == 'start':
            mode = SequenceMode(args.mode.upper())
            start_point = None
            if args.start:
                if args.start.startswith('@'):
                    start_point = int(args.start[1:])
                else:
                    start_point = args.start

            self.sequence_nav.start_sequence(
                mode=mode,
                is_loop=args.loop,
                reverse=args.reverse,
                start_point=start_point
            )

        elif args.seq_command == 'insert':
            point = args.point
            if point.startswith('@'):
                wp = self.nav_control._waypoint_manager.get_waypoint_by_index(
                    int(point[1:]))
                if wp:
                    name, pose = wp
                    nav_point = NavigationPoint(
                        pose=pose, name=name, index=int(point[1:]))
                    self.sequence_nav.insert_point(args.index, nav_point)
            else:
                pose = self.nav_control._waypoint_manager.get_waypoint_by_name(
                    point)
                if pose:
                    nav_point = NavigationPoint(pose=pose, name=point)
                    self.sequence_nav.insert_point(args.index, nav_point)

        elif args.seq_command == 'remove':
            self.sequence_nav.remove_point(args.index)

        elif args.seq_command == 'reorder':
            self.sequence_nav.reorder_sequence(args.indices, args.reverse)

        elif args.seq_command == 'pause':
            self.sequence_nav.pause_sequence()

        elif args.seq_command == 'resume':
            self.sequence_nav.resume_sequence()

        elif args.seq_command == 'stop':
            self.sequence_nav.stop_sequence()

        elif args.seq_command == 'clear':
            self.sequence_nav.clear_sequence()

        elif args.seq_command == 'info':
            info = self.sequence_nav.get_sequence_info()
            rospy.loginfo("\nSequence Info:")
            rospy.loginfo(f"  Total Points: {info['total_points']}")
            rospy.loginfo(f"  Current Index: {info['current_index']}")
            rospy.loginfo(f"  Mode: {info['mode']}")
            rospy.loginfo(
                f"  Status: {'Running' if info['is_running'] else 'Stopped'}")
            rospy.loginfo(f"  Points: {', '.join(info['points'])}")

        elif args.seq_command == 'state':
            state = self.sequence_nav.get_pause_state()
            if state:
                rospy.loginfo("\nPause State:")
                rospy.loginfo(f"  Current Index: {state['current_index']}")
                rospy.loginfo(f"  Total Points: {state['total_points']}")
                rospy.loginfo(f"  Mode: {state['mode']}")
                rospy.loginfo(f"  Loop: {state['is_loop']}")
                rospy.loginfo(f"  Reverse: {state['is_reverse']}")
                rospy.loginfo(f"  Current Goal: {state['current_goal']}")
            else:
                rospy.logwarn("No pause state available")

    def run(self):
        """运行节点"""
        rospy.spin()


def main():
    try:
        controller = WPNavControllerNode()

        controller.run()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in navigation controller: {e}")


if __name__ == '__main__':
    main()
