#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from amr_waypoint_tools.srv import GetNumOfWaypoints
from amr_waypoint_tools.srv import GetWaypointByIndex
from wp_nav_controller.srv import NavigationCommand, SequenceCommand


class WaypointNavigationGo:
    def __init__(self):
        # 初始化节点
        rospy.init_node('waypoint_navigation_go', anonymous=True)

        # 等待航点服务
        rospy.wait_for_service('/waypoint/get_num_waypoint')
        self.get_num_client = rospy.ServiceProxy(
            '/waypoint/get_num_waypoint', GetNumOfWaypoints)

        rospy.wait_for_service('/waypoint/get_waypoint_index')
        self.get_info_client = rospy.ServiceProxy(
            '/waypoint/get_waypoint_index', GetWaypointByIndex)

        # 等待导航控制服务
        rospy.wait_for_service('/wp_nav_controller/navigation')
        self.nav_client = rospy.ServiceProxy(
            '/wp_nav_controller/navigation', NavigationCommand)

        rospy.wait_for_service('/wp_nav_controller/sequence')
        self.seq_client = rospy.ServiceProxy(
            '/wp_nav_controller/sequence', SequenceCommand)

    def get_waypoint_num(self):
        """获取航点总数"""
        try:
            response = self.get_num_client()
            return response.num
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_waypoint_info(self, index):
        """获取指定索引的航点信息"""
        try:
            response = self.get_info_client(index)  # 获取索引为 index 的航点
            return response.name, response.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def create_sequence(self, start_index=0):
        """创建导航序列
        :param waypoints: 航点列表
        :param start_index: 起始航点索引
        :param mode: 导航模式，可选值：'roundtrip', 'single', 'loop'
        :param loop: 是否循环
        :param reverse: 是否反向
        """
        try:
            waypoints = []
            waypoints_num = self.get_waypoint_num()

            for i in range(waypoints_num):
                waypoints.append(self.get_waypoint_info(i)[0])
            rospy.loginfo(f"waypoints_raw: {waypoints}")

            waypoints[:] = waypoints[start_index:] + waypoints[:start_index]
            rospy.loginfo(f"waypoints: {waypoints}")

            response = self.seq_client(
                command='create',
                mode='',
                loop=False,
                reverse=False,
                params=waypoints,
            )
            return response.success, response.message

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False, str(e)

    def start_sequence(self, mode='roundtrip', loop=True, reverse=False):
        """启动往返导航
        :param start_index: 起始航点索引
        :param mode: 导航模式，可选值：'roundtrip', 'single', 'loop'
        :param loop: 是否循环
        :param reverse: 是否反向
        :return: 启动成功返回True，否则返回False
        """
        try:
            response = self.seq_client(
                command='start',
                mode=mode,
                loop=loop,
                reverse=reverse,
                params=[],
            )
            return response.success, response.message

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False, str(e)


if __name__ == '__main__':
    try:
        # 创建导航对象
        navigator = WaypointNavigationGo()

        # 创建序列
        success, message = navigator.create_sequence()
        if success:
            rospy.loginfo("Sequence created successfully")

            # 启动导航
            success, message = navigator.start_sequence(
                mode='roundtrip',  # 使用往返模式
                loop=True,         # 启用循环
                reverse=False      # 正向开始
            )

            if success:
                rospy.loginfo("Navigation started successfully")
                rospy.spin()  # 保持节点运行
            else:
                rospy.logerr(f"Failed to start navigation: {message}")
        else:
            rospy.logerr(f"Failed to create sequence: {message}")

    except rospy.ROSInterruptException:
        pass
