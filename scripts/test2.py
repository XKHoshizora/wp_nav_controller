#!/usr/bin/env python3

import rospy
from wp_nav_controller.msg import WaypointSequence, NavigationStatus
from amr_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose

class WaypointNavigationTester:
    def __init__(self):
        rospy.init_node('waypoint_navigation_test')

        # 创建发布者
        self.sequence_pub = rospy.Publisher('/wp_nav_controller/waypoint_sequence',
                                          WaypointSequence, queue_size=1)
        self.single_wp_pub = rospy.Publisher('/wp_nav_controller/target_waypoint',
                                           String, queue_size=1)
        self.single_pose_pub = rospy.Publisher('/wp_nav_controller/target_pose',
                                             Pose, queue_size=1)
        self.stop_pub = rospy.Publisher('/wp_nav_controller/stop', Empty, queue_size=1)
        self.pause_pub = rospy.Publisher('/wp_nav_controller/pause', Empty, queue_size=1)
        self.resume_pub = rospy.Publisher('/wp_nav_controller/resume', Empty, queue_size=1)
        self.skip_pub = rospy.Publisher('/wp_nav_controller/skip_waypoint', Empty, queue_size=1)

        # 创建订阅者
        self.status_sub = rospy.Subscriber('/wp_nav_controller/status',
                                         NavigationStatus, self.status_callback)

        # 等待发布者和订阅者初始化
        rospy.sleep(1.0)

        # 状态变量
        self.current_status = None
        self.waypoints = []

        # 载入航点数据
        self.load_waypoints()

    def status_callback(self, msg):
        """导航状态回调函数"""
        self.current_status = msg
        self.print_navigation_status(msg)

    def print_navigation_status(self, status):
        """打印导航状态信息"""
        state_names = ["IDLE", "ACTIVE", "PAUSED", "COMPLETED"]
        mode_names = ["SINGLE_POSE", "SINGLE_WAYPOINT", "SEQUENCE_ONCE",
                     "SEQUENCE_LOOP", "SEQUENCE_BACK_FORTH"]

        rospy.loginfo(f"Navigation Status:")
        rospy.loginfo(f"  State: {state_names[status.state]}")
        rospy.loginfo(f"  Mode: {mode_names[status.mode]}")
        rospy.loginfo(f"  Current Waypoint: {status.current_waypoint}")
        rospy.loginfo(f"  Current Index: {status.current_index}")

    def load_waypoints(self):
        """加载所有可用航点"""
        try:
            # 获取航点总数
            get_num = rospy.ServiceProxy('/waypoint/get_num_waypoint', GetNumOfWaypoints)
            num_response = get_num()

            # 获取每个航点信息
            get_wp = rospy.ServiceProxy('/waypoint/get_waypoint_index', GetWaypointByIndex)
            for i in range(num_response.num):
                wp_response = get_wp(i)
                self.waypoints.append(wp_response.name)
                rospy.loginfo(f"Found waypoint {i}: {wp_response.name} at "
                           f"({wp_response.pose.position.x:.2f}, {wp_response.pose.position.y:.2f})")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

        return len(self.waypoints) > 0

    def test_single_point_navigation(self):
        """测试单点导航功能"""
        print("\nSingle Point Navigation Test")
        print("1. Navigate to waypoint by name")
        print("2. Navigate to pose")
        print("3. Back to main menu")

        choice = input("Enter your choice (1-3): ")

        if choice == '1':
            print(f"\nAvailable waypoints: {', '.join(self.waypoints)}")
            wp_name = input("Enter waypoint name: ")
            if wp_name in self.waypoints:
                msg = String()
                msg.data = wp_name
                self.single_wp_pub.publish(msg)
                rospy.loginfo(f"Starting navigation to waypoint: {wp_name}")
            else:
                rospy.logerr("Invalid waypoint name!")

        elif choice == '2':
            pose = Pose()
            print("\nEnter target pose:")
            pose.position.x = float(input("Enter X coordinate: "))
            pose.position.y = float(input("Enter Y coordinate: "))
            pose.orientation.w = 1.0  # 简单起见，只设置朝向为正前方
            self.single_pose_pub.publish(pose)
            rospy.loginfo(f"Starting navigation to pose: ({pose.position.x}, {pose.position.y})")

    def test_sequence_navigation(self):
        """测试序列导航功能"""
        print("\nSequence Navigation Test")
        print("1. SEQUENCE_ONCE (Forward)")
        print("2. SEQUENCE_ONCE (Backward)")
        print("3. SEQUENCE_LOOP (Forward)")
        print("4. SEQUENCE_LOOP (Backward)")
        print("5. SEQUENCE_BACK_FORTH (Forward)")
        print("6. SEQUENCE_BACK_FORTH (Backward)")
        print("7. Back to main menu")

        choice = input("Enter your choice (1-7): ")
        if choice == '7':
            return

        print(f"\nAvailable waypoints: {', '.join(self.waypoints)}")
        start_idx = int(input(f"Enter start index (0-{len(self.waypoints)-1}): "))

        if start_idx < 0 or start_idx >= len(self.waypoints):
            rospy.logerr("Invalid start index!")
            return

        seq_msg = WaypointSequence()
        seq_msg.waypoint_names = self.waypoints
        seq_msg.start_index = start_idx

        mode_map = {'1': (2, 0), '2': (2, 1), '3': (3, 0),
                   '4': (3, 1), '5': (4, 0), '6': (4, 1)}
        if choice in mode_map:
            seq_msg.mode, seq_msg.direction = mode_map[choice]
            self.sequence_pub.publish(seq_msg)
            rospy.loginfo(f"Started sequence navigation: mode={seq_msg.mode}, "
                       f"direction={seq_msg.direction}, start_index={start_idx}")

    def test_navigation_control(self):
        """测试导航控制功能"""
        print("\nNavigation Control Test")
        print("1. Pause navigation")
        print("2. Resume navigation")
        print("3. Stop navigation")
        print("4. Skip current waypoint")
        print("5. Back to main menu")

        choice = input("Enter your choice (1-5): ")

        if choice == '1':
            self.pause_pub.publish(Empty())
            rospy.loginfo("Navigation paused")
        elif choice == '2':
            self.resume_pub.publish(Empty())
            rospy.loginfo("Navigation resumed")
        elif choice == '3':
            self.stop_pub.publish(Empty())
            rospy.loginfo("Navigation stopped")
        elif choice == '4':
            self.skip_pub.publish(Empty())
            rospy.loginfo("Skipping current waypoint")

    def show_navigation_status(self):
        """显示当前导航状态"""
        if self.current_status:
            self.print_navigation_status(self.current_status)
        else:
            rospy.loginfo("No navigation status available")

    def run(self):
        """主测试循环"""
        if not self.waypoints:
            rospy.logerr("No waypoints available!")
            return

        rospy.loginfo(f"Found {len(self.waypoints)} waypoints")

        while not rospy.is_shutdown():
            print("\nWaypoint Navigation Test Menu")
            print("1. Test Single Point Navigation")
            print("2. Test Sequence Navigation")
            print("3. Test Navigation Control")
            print("4. Show Current Status")
            print("5. Exit")

            choice = input("Enter your choice (1-5): ")

            if choice == '1':
                self.test_single_point_navigation()
            elif choice == '2':
                self.test_sequence_navigation()
            elif choice == '3':
                self.test_navigation_control()
            elif choice == '4':
                self.show_navigation_status()
            elif choice == '5':
                break

if __name__ == '__main__':
    try:
        tester = WaypointNavigationTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass