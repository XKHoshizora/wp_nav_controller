#ifndef WAYPOINT_NAVIGATOR_H
#define WAYPOINT_NAVIGATOR_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <amr_map_tools/Waypoint.h>
#include <amr_map_tools/GetNumOfWaypoints.h>
#include <amr_map_tools/GetWaypointByIndex.h>
#include <amr_map_tools/GetWaypointByName.h>
#include <string>
#include <vector>

namespace wp_nav_controller {

class WaypointNavigator {
public:
    WaypointNavigator(ros::NodeHandle& nh);
    ~WaypointNavigator() = default;

    // 基本导航控制
    bool navigateToWaypoint(int wpIndex);
    bool navigateToPose(const geometry_msgs::PoseStamped& pose);
    void pauseNavigation();
    void resumeNavigation();
    bool switchToWaypoint(int newWpIndex);
    void stopNavigation();

    // 序列导航控制
    void startWaypointSequence(bool reverse = false);
    void stopWaypointSequence();

    // 状态查询接口
    bool isNavigationPaused() const { return isPaused_; }
    bool isNavigating() const { return isNavigating_; }
    int getCurrentWaypointIndex() const { return currentWaypointIndex_; }
    bool isSequenceRunning() const { return isSequenceRunning_; }
    geometry_msgs::PoseStamped getCurrentGoal() const { return currentGoal_.target_pose; }
    std::string getServicePrefix() const { return service_prefix_; }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                     const move_base_msgs::MoveBaseResultConstPtr& result);
    void activeCallback();
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    // 服务初始化
    void initializeServices();

    // 航点操作
    int getNumWaypoints();
    bool getWaypointByIndex(int index, geometry_msgs::Pose& pose, std::string& name);
    bool getWaypointByName(const std::string& wpName, geometry_msgs::Pose& pose);
    void processNextWaypoint();

    // 辅助功能
    double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
    bool isGoalReached(double tolerance = 0.5);
    void publishNavigationMarkers();

    // ROS相关
    ros::NodeHandle& nh_;
    MoveBaseClient ac_;
    ros::ServiceClient cliGetNum_;
    ros::ServiceClient cliGetWPIndex_;
    ros::ServiceClient cliGetWPName_;
    ros::Publisher markerPub_;
    tf::TransformListener tfListener_;

    // 导航状态
    int currentWaypointIndex_;
    bool isPaused_;
    bool isNavigating_;
    bool isSequenceRunning_;
    bool isReverse_;
    move_base_msgs::MoveBaseGoal currentGoal_;
    std::vector<int> waypointSequence_;
    size_t sequenceIndex_;

    // 配置参数
    std::string service_prefix_;
    double goalTolerance_;
    double navigationTimeout_;
    double updateRate_;

    // 计时器
    ros::Time lastUpdateTime_;
};

} // namespace wp_nav_controller

#endif // WAYPOINT_NAVIGATOR_H