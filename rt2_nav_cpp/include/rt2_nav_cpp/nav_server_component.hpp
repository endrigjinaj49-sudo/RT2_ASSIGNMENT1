#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rt2_nav_cpp/action/navigate_to_pose.hpp"

#include <memory>
#include <string>

namespace rt2_nav_cpp
{

class NavServerComponent : public rclcpp::Node
{
public:
  using NavigateToPose = rt2_nav_cpp::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit NavServerComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string fixed_frame_;
  std::string robot_frame_;
  bool odom_received_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool getRobotPoseFromTf(double & x, double & y, double & theta);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);
};

}  // namespace rt2_nav_cpp