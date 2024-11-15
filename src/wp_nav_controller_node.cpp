#include "wp_nav_controller/waypoint_navigator.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "wp_nav_controller/NavigationCommand.h"

class WaypointNavigatorNode {
public:
    WaypointNavigatorNode(ros::NodeHandle& nh) : navigator_(nh) {
        // 订阅导航命令
        commandSub_ = nh.subscribe("wp_nav/command", 1,
            &WaypointNavigatorNode::commandCallback, this);

        // 发布状态信息
        statusPub_ = nh.advertise<std_msgs::String>("wp_nav/status", 1);
        currentWaypointPub_ = nh.advertise<std_msgs::Int32>("wp_nav/current_waypoint", 1);

        // 创建状态发布定时器
        double update_rate;
        ros::param::param<double>("~update_rate", update_rate, 10.0);
        statusTimer_ = nh.createTimer(ros::Duration(1.0/update_rate),
            &WaypointNavigatorNode::publishStatus, this);
    }

private:
    void commandCallback(const wp_nav_controller::NavigationCommand::ConstPtr& msg) {
        try {
            switch(msg->command) {
                case wp_nav_controller::NavigationCommand::STOP:
                    navigator_.stopNavigation();
                    break;

                case wp_nav_controller::NavigationCommand::PAUSE:
                    navigator_.pauseNavigation();
                    break;

                case wp_nav_controller::NavigationCommand::RESUME:
                    navigator_.resumeNavigation();
                    break;

                case wp_nav_controller::NavigationCommand::GOTO_WAYPOINT:
                    navigator_.switchToWaypoint(msg->waypoint_index);
                    break;

                case wp_nav_controller::NavigationCommand::GOTO_POSE:
                    navigator_.navigateToPose(msg->target_pose);
                    break;

                case wp_nav_controller::NavigationCommand::START_SEQUENCE:
                    navigator_.startWaypointSequence(msg->reverse);
                    break;

                case wp_nav_controller::NavigationCommand::STOP_SEQUENCE:
                    navigator_.stopWaypointSequence();
                    break;

                default:
                    ROS_WARN("Unknown navigation command: %d", msg->command);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error processing navigation command: %s", e.what());
        }
    }

    void publishStatus(const ros::TimerEvent&) {
        // 发布当前航点
        std_msgs::Int32 wpMsg;
        wpMsg.data = navigator_.getCurrentWaypointIndex();
        currentWaypointPub_.publish(wpMsg);

        // 发布导航状态
        std_msgs::String statusMsg;
        std::stringstream ss;
        ss << "Navigation Status:\n"
           << "  Current Waypoint: " << navigator_.getCurrentWaypointIndex() << "\n"
           << "  State: " << (navigator_.isNavigating() ? "Navigating" :
                             (navigator_.isNavigationPaused() ? "Paused" : "Idle")) << "\n"
           << "  Sequence: " << (navigator_.isSequenceRunning() ? "Running" : "Stopped") << "\n";

        if (navigator_.isNavigating()) {
            geometry_msgs::PoseStamped currentGoal = navigator_.getCurrentGoal();
            ss << "  Current Goal: ("
               << currentGoal.pose.position.x << ", "
               << currentGoal.pose.position.y << ")\n";
        }

        statusMsg.data = ss.str();
        statusPub_.publish(statusMsg);
    }

    wp_nav_controller::WaypointNavigator navigator_;
    ros::Subscriber commandSub_;
    ros::Publisher statusPub_;
    ros::Publisher currentWaypointPub_;
    ros::Timer statusTimer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wp_nav_controller_node");
    ros::NodeHandle nh;

    try {
        WaypointNavigatorNode node(nh);
        ROS_INFO("Waypoint navigation controller node started");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in waypoint navigator node: %s", e.what());
        return 1;
    }

    return 0;
}