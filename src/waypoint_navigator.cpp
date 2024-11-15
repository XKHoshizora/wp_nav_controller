#include "wp_nav_controller/waypoint_navigator.h"
#include <cmath>

namespace wp_nav_controller {

WaypointNavigator::WaypointNavigator(ros::NodeHandle& nh) :
    nh_(nh),
    ac_("move_base", true),
    currentWaypointIndex_(0),
    isPaused_(false),
    isNavigating_(false),
    isSequenceRunning_(false),
    isReverse_(false),
    sequenceIndex_(0)
{
    // 初始化参数
    ros::NodeHandle private_nh("~");
    private_nh.param<double>("goal_tolerance", goalTolerance_, 0.5);
    private_nh.param<double>("timeout", navigationTimeout_, 300.0);
    private_nh.param<double>("update_rate", updateRate_, 10.0);

    // 初始化服务和发布器
    initializeServices();
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

    // 等待move_base服务器
    ROS_INFO("Waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("Move base server connected!");
}

void WaypointNavigator::initializeServices() {
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("service_prefix", service_prefix_, "waypoint");

    std::string num_service = service_prefix_ + "/get_num_waypoint";
    std::string index_service = service_prefix_ + "/get_waypoint_index";
    std::string name_service = service_prefix_ + "/get_waypoint_name";

    cliGetNum_ = nh_.serviceClient<amr_map_tools::GetNumOfWaypoints>(num_service);
    cliGetWPIndex_ = nh_.serviceClient<amr_map_tools::GetWaypointByIndex>(index_service);
    cliGetWPName_ = nh_.serviceClient<amr_map_tools::GetWaypointByName>(name_service);

    ROS_INFO("Initialized waypoint services with prefix: %s", service_prefix_.c_str());
}

bool WaypointNavigator::navigateToWaypoint(int wpIndex) {
    geometry_msgs::Pose pose;
    std::string name;
    if (!getWaypointByIndex(wpIndex, pose, name)) {
        return false;
    }

    currentWaypointIndex_ = wpIndex;
    currentGoal_.target_pose.header.frame_id = "map";
    currentGoal_.target_pose.header.stamp = ros::Time::now();
    currentGoal_.target_pose.pose = pose;

    ROS_INFO("Navigating to waypoint[%d]: %s", wpIndex, name.c_str());
    isNavigating_ = true;
    lastUpdateTime_ = ros::Time::now();

    ac_.sendGoal(currentGoal_,
        boost::bind(&WaypointNavigator::doneCallback, this, _1, _2),
        boost::bind(&WaypointNavigator::activeCallback, this),
        boost::bind(&WaypointNavigator::feedbackCallback, this, _1));

    return true;
}

bool WaypointNavigator::navigateToPose(const geometry_msgs::PoseStamped& pose) {
    currentGoal_.target_pose = pose;
    ROS_INFO("Navigating to custom pose");
    isNavigating_ = true;
    lastUpdateTime_ = ros::Time::now();

    ac_.sendGoal(currentGoal_,
        boost::bind(&WaypointNavigator::doneCallback, this, _1, _2),
        boost::bind(&WaypointNavigator::activeCallback, this),
        boost::bind(&WaypointNavigator::feedbackCallback, this, _1));

    return true;
}

void WaypointNavigator::pauseNavigation() {
    if (isNavigating_ && !isPaused_) {
        ac_.cancelGoal();
        isPaused_ = true;
        ROS_INFO("Navigation paused at waypoint[%d]", currentWaypointIndex_);
    }
}

void WaypointNavigator::resumeNavigation() {
    if (isPaused_) {
        isPaused_ = false;
        if (isSequenceRunning_) {
            processNextWaypoint();
        } else {
            navigateToWaypoint(currentWaypointIndex_);
        }
        ROS_INFO("Resuming navigation");
    }
}

bool WaypointNavigator::switchToWaypoint(int newWpIndex) {
    stopWaypointSequence();
    if (isNavigating_) {
        ac_.cancelGoal();
    }
    return navigateToWaypoint(newWpIndex);
}

void WaypointNavigator::stopNavigation() {
    if (isNavigating_) {
        ac_.cancelGoal();
        isNavigating_ = false;
        ROS_INFO("Navigation stopped");
    }
}

void WaypointNavigator::startWaypointSequence(bool reverse) {
    isReverse_ = reverse;
    isSequenceRunning_ = true;

    int numWaypoints = getNumWaypoints();
    waypointSequence_.clear();

    if (reverse) {
        for (int i = numWaypoints - 1; i >= 0; --i) {
            waypointSequence_.push_back(i);
        }
    } else {
        for (int i = 0; i < numWaypoints; ++i) {
            waypointSequence_.push_back(i);
        }
    }

    sequenceIndex_ = 0;
    if (!waypointSequence_.empty()) {
        navigateToWaypoint(waypointSequence_[0]);
    }
}

void WaypointNavigator::stopWaypointSequence() {
    isSequenceRunning_ = false;
    waypointSequence_.clear();
    if (isNavigating_) {
        ac_.cancelGoal();
    }
}

int WaypointNavigator::getNumWaypoints() {
    amr_map_tools::GetNumOfWaypoints srv;
    if (cliGetNum_.call(srv)) {
        return srv.response.num;
    }
    return 0;
}

bool WaypointNavigator::getWaypointByIndex(int index, geometry_msgs::Pose& pose, std::string& name) {
    amr_map_tools::GetWaypointByIndex srv;
    srv.request.index = index;

    if (cliGetWPIndex_.call(srv)) {
        pose = srv.response.pose;
        name = srv.response.name;
        return true;
    }
    ROS_ERROR("Failed to get waypoint at index %d", index);
    return false;
}

bool WaypointNavigator::getWaypointByName(const std::string& wpName, geometry_msgs::Pose& pose) {
    amr_map_tools::GetWaypointByName srv;
    srv.request.name = wpName;

    if (cliGetWPName_.call(srv)) {
        pose = srv.response.pose;
        return true;
    }
    ROS_ERROR("Failed to get waypoint with name %s", wpName.c_str());
    return false;
}

void WaypointNavigator::processNextWaypoint() {
    if (!isSequenceRunning_ || isPaused_) {
        return;
    }

    sequenceIndex_++;
    if (sequenceIndex_ < waypointSequence_.size()) {
        navigateToWaypoint(waypointSequence_[sequenceIndex_]);
    } else {
        ROS_INFO("Waypoint sequence completed");
        isSequenceRunning_ = false;
    }
}

void WaypointNavigator::doneCallback(const actionlib::SimpleClientGoalState& state,
                   const move_base_msgs::MoveBaseResultConstPtr& result) {
    isNavigating_ = false;

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Arrived at waypoint[%d]!", currentWaypointIndex_);
        if (isSequenceRunning_) {
            processNextWaypoint();
        }
    } else {
        ROS_INFO("Failed to reach waypoint[%d]", currentWaypointIndex_);
    }
}

void WaypointNavigator::activeCallback() {
    ROS_INFO("Navigation to waypoint[%d] is now active", currentWaypointIndex_);
}

void WaypointNavigator::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    // 检查超时
    if ((ros::Time::now() - lastUpdateTime_).toSec() > navigationTimeout_) {
        ROS_WARN("Navigation timeout reached for waypoint[%d]", currentWaypointIndex_);
        stopNavigation();
        return;
    }

    // 检查是否到达目标点
    if (isGoalReached(goalTolerance_)) {
        ROS_INFO("Goal reached within tolerance");
        stopNavigation();
        if (isSequenceRunning_) {
            processNextWaypoint();
        }
    }
}

double WaypointNavigator::calculateDistance(const geometry_msgs::Pose& pose1,
                                          const geometry_msgs::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

bool WaypointNavigator::isGoalReached(double tolerance) {
    tf::StampedTransform transform;
    try {
        tfListener_.lookupTransform("map", "base_link", ros::Time(0), transform);

        geometry_msgs::Pose currentPose;
        currentPose.position.x = transform.getOrigin().x();
        currentPose.position.y = transform.getOrigin().y();
        currentPose.position.z = transform.getOrigin().z();

        double distance = calculateDistance(currentPose, currentGoal_.target_pose.pose);
        return distance <= tolerance;
    } catch (tf::TransformException& ex) {
        ROS_WARN("Failed to get robot pose: %s", ex.what());
        return false;
    }
}

void WaypointNavigator::publishNavigationMarkers() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = currentGoal_.target_pose.pose;
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(1.0);
    markerPub_.publish(marker);
}

} // namespace wp_nav_controller