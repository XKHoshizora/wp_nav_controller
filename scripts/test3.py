#!/usr/bin/env python3

import rospy
import threading
import time
from wp_nav_controller.msg import WaypointSequence, NavigationStatus
from amr_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose

class WaypointNavigationTester:
    def __init__(self):
        rospy.init_node('waypoint_navigation_test')

        # 状态变量
        self.current_status = None
        self.waypoints = []
        self._status_lock = threading.Lock()
        self._monitor_thread = None
        self._stop_monitor = False

        # 创建发布者
        self.sequence_pub = rospy.Publisher('/wp_nav_controller/waypoint_sequence',
                                          WaypointSequence, queue_size=1)
        self.single_wp_pub = rospy.Publisher('/wp_nav_controller/target_waypoint',
                                           String, queue_size=1)
        self.stop_pub = rospy.Publisher('/wp_nav_controller/stop',
                                      Empty, queue_size=1)
        self.pause_pub = rospy.Publisher('/wp_nav_controller/pause',
                                       Empty, queue_size=1)
        self.resume_pub = rospy.Publisher('/wp_nav_controller/resume',
                                        Empty, queue_size=1)
        self.skip_pub = rospy.Publisher('/wp_nav_controller/skip_waypoint',
                                      Empty, queue_size=1)

        # 创建订阅者
        self.status_sub = rospy.Subscriber('/wp_nav_controller/status',
                                         NavigationStatus,
                                         self.status_callback,
                                         queue_size=1)

        # 等待初始化
        rospy.sleep(1.0)

        # 载入航点数据
        self.load_waypoints()

    def status_callback(self, msg):
        """导航状态回调函数"""
        with self._status_lock:
            self.current_status = msg

    def print_navigation_status(self, status):
        """打印导航状态信息"""
        state_names = ["IDLE", "ACTIVE", "PAUSED", "COMPLETED"]
        mode_names = ["SINGLE_POSE", "SINGLE_WAYPOINT", "SEQUENCE_ONCE",
                     "SEQUENCE_LOOP", "SEQUENCE_BACK_FORTH"]

        print("\033[2J\033[H")  # 清屏并移动光标到开始位置
        print("Current Navigation Status:")
        print(f"  State: {state_names[status.state]}")
        print(f"  Mode: {mode_names[status.mode]}")
        print(f"  Current Waypoint: {status.current_waypoint}")
        print(f"  Current Index: {status.current_index}")
        print("\nPress 'p' to pause, 'r' to resume, 's' to stop, 'k' to skip")
        print("Press 'q' to return to main menu")

    def start_status_monitor(self):
        """启动状态监控线程"""
        self._stop_monitor = False
        self._monitor_thread = threading.Thread(target=self._status_monitor_loop)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def stop_status_monitor(self):
        """停止状态监控线程"""
        self._stop_monitor = True
        if self._monitor_thread:
            self._monitor_thread.join()
            self._monitor_thread = None

    def _status_monitor_loop(self):
        """状态监控循环"""
        while not self._stop_monitor and not rospy.is_shutdown():
            with self._status_lock:
                if self.current_status:
                    self.print_navigation_status(self.current_status)
            time.sleep(1.0)  # 更新频率1Hz

    def load_waypoints(self):
        """加载所有可用航点"""
        try:
            get_num = rospy.ServiceProxy('/waypoint/get_num_waypoint', GetNumOfWaypoints)
            num_response = get_num()

            get_wp = rospy.ServiceProxy('/waypoint/get_waypoint_index', GetWaypointByIndex)
            for i in range(num_response.num):
                wp_response = get_wp(i)
                self.waypoints.append(wp_response.name)
                rospy.loginfo(f"Found waypoint {i}: {wp_response.name} at "
                           f"({wp_response.pose.position.x:.2f}, {wp_response.pose.position.y:.2f})")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

        rospy.loginfo(f"Found {len(self.waypoints)} waypoints")
        return len(self.waypoints) > 0

    def monitor_navigation(self):
        """监控导航状态并处理用户输入"""
        self.start_status_monitor()

        import sys
        import tty
        import termios

        def getch():
            """获取单个字符输入"""
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        while not rospy.is_shutdown():
            c = getch()
            if c == 'p':
                self.pause_pub.publish(Empty())
                rospy.loginfo("Navigation paused")
            elif c == 'r':
                self.resume_pub.publish(Empty())
                rospy.loginfo("Navigation resumed")
            elif c == 's':
                self.stop_pub.publish(Empty())
                rospy.loginfo("Navigation stopped")
            elif c == 'k':
                self.skip_pub.publish(Empty())
                rospy.loginfo("Skipping current waypoint")
            elif c == 'q':
                break

        self.stop_status_monitor()

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

            # 开始监控导航状态
            self.monitor_navigation()

    def run(self):
        """主测试循环"""
        if not self.waypoints:
            rospy.logerr("No waypoints available!")
            return

        while not rospy.is_shutdown():
            print("\nWaypoint Navigation Test Menu")
            print("1. Test Sequence Navigation")
            print("2. Exit")

            choice = input("Enter your choice (1-2): ")

            if choice == '1':
                self.test_sequence_navigation()
            elif choice == '2':
                break

if __name__ == '__main__':
    try:
        tester = WaypointNavigationTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass