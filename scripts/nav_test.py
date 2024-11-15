#!/usr/bin/env python

import rospy
from wp_nav_controller.msg import NavigationCommand
from geometry_msgs.msg import PoseStamped
import sys

def send_navigation_command():
    rospy.init_node('nav_test', anonymous=True)
    cmd_pub = rospy.Publisher('wp_nav/command', NavigationCommand, queue_size=1)
    rospy.sleep(1)  # 等待发布器初始化

    cmd = NavigationCommand()

    if len(sys.argv) < 2:
        print("Usage: nav_test.py [command] [args]")
        print("Commands:")
        print("  goto_waypoint [index]")
        print("  start_sequence [reverse:true/false]")
        print("  stop")
        print("  pause")
        print("  resume")
        return

    command = sys.argv[1]

    if command == "goto_waypoint":
        if len(sys.argv) < 3:
            print("Please specify waypoint index")
            return
        cmd.command = NavigationCommand.GOTO_WAYPOINT
        cmd.waypoint_index = int(sys.argv[2])

    elif command == "start_sequence":
        cmd.command = NavigationCommand.START_SEQUENCE
        cmd.reverse = len(sys.argv) > 2 and sys.argv[2].lower() == "true"

    elif command == "stop":
        cmd.command = NavigationCommand.STOP

    elif command == "pause":
        cmd.command = NavigationCommand.PAUSE

    elif command == "resume":
        cmd.command = NavigationCommand.RESUME

    else:
        print("Unknown command:", command)
        return

    cmd_pub.publish(cmd)
    print("Command sent:", command)

if __name__ == '__main__':
    try:
        send_navigation_command()
    except rospy.ROSInterruptException:
        pass