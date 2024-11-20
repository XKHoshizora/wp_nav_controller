#!/usr/bin/env python3

import rospy
from wp_nav_controller.msg import WaypointSequence
from amr_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex
from std_msgs.msg import Empty

def get_all_waypoints():
    """获取所有可用的航点"""
    waypoints = []
    try:
        # 获取航点总数
        get_num = rospy.ServiceProxy('/waypoint/get_num_waypoint', GetNumOfWaypoints)
        num_response = get_num()

        # 获取每个航点信息
        get_wp = rospy.ServiceProxy('/waypoint/get_waypoint_index', GetWaypointByIndex)
        for i in range(num_response.num):
            wp_response = get_wp(i)
            waypoints.append(wp_response.name)
            rospy.loginfo(f"Found waypoint {i}: {wp_response.name} at "
                         f"({wp_response.pose.position.x:.2f}, {wp_response.pose.position.y:.2f})")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return []

    return waypoints

def test_sequence_navigation():
    rospy.init_node('waypoint_sequence_test')

    # 创建发布者
    sequence_pub = rospy.Publisher('/wp_nav_controller/waypoint_sequence',
                                 WaypointSequence, queue_size=1)
    stop_pub = rospy.Publisher('/wp_nav_controller/stop', Empty, queue_size=1)

    # 等待发布者初始化
    rospy.sleep(1.0)

    # 获取所有航点
    waypoints = get_all_waypoints()
    if not waypoints:
        rospy.logerr("No waypoints available!")
        return

    rospy.loginfo(f"Found {len(waypoints)} waypoints")

    while not rospy.is_shutdown():
        # 显示菜单
        print("\nWaypoint Sequence Navigation Test")
        print("1. Test SEQUENCE_ONCE (Forward)")
        print("2. Test SEQUENCE_ONCE (Backward)")
        print("3. Test SEQUENCE_LOOP (Forward)")
        print("4. Test SEQUENCE_LOOP (Backward)")
        print("5. Test SEQUENCE_BACK_FORTH (Forward)")
        print("6. Test SEQUENCE_BACK_FORTH (Backward)")
        print("7. Stop Navigation")
        print("8. Exit")

        choice = input("Enter your choice (1-8): ")

        if choice == '8':
            break

        if choice == '7':
            stop_pub.publish(Empty())
            continue

        # 获取起始点
        print(f"\nAvailable waypoints: {', '.join(waypoints)}")
        start_idx = int(input(f"Enter start index (0-{len(waypoints)-1}): "))

        if start_idx < 0 or start_idx >= len(waypoints):
            rospy.logerr("Invalid start index!")
            continue

        # 创建序列导航消息
        seq_msg = WaypointSequence()
        seq_msg.waypoint_names = waypoints
        seq_msg.start_index = start_idx

        if choice == '1':
            seq_msg.mode = 2  # SEQUENCE_ONCE
            seq_msg.direction = 0  # FORWARD
        elif choice == '2':
            seq_msg.mode = 2  # SEQUENCE_ONCE
            seq_msg.direction = 1  # BACKWARD
        elif choice == '3':
            seq_msg.mode = 3  # SEQUENCE_LOOP
            seq_msg.direction = 0  # FORWARD
        elif choice == '4':
            seq_msg.mode = 3  # SEQUENCE_LOOP
            seq_msg.direction = 1  # BACKWARD
        elif choice == '5':
            seq_msg.mode = 4  # SEQUENCE_BACK_FORTH
            seq_msg.direction = 0  # FORWARD
        elif choice == '6':
            seq_msg.mode = 4  # SEQUENCE_BACK_FORTH
            seq_msg.direction = 1  # BACKWARD

        # 发送导航命令
        sequence_pub.publish(seq_msg)
        rospy.loginfo(f"Started sequence navigation: mode={seq_msg.mode}, "
                     f"direction={seq_msg.direction}, start_index={start_idx}")

if __name__ == '__main__':
    try:
        test_sequence_navigation()
    except rospy.ROSInterruptException:
        pass