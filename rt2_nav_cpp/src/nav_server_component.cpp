#include "rt2_nav_cpp/nav_server_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <thread>
#include <cmath>
#include <algorithm>

namespace rt2_nav_cpp
{

static double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

static double yawFromQuaternion(
  double x,
  double y,
  double z,
  double w)
{
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

NavServerComponent::NavServerComponent(const rclcpp::NodeOptions & options)
: Node("nav_server", options), odom_received_(false)
{
  fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "odom");
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&NavServerComponent::odomCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "navigate_to_pose",
    std::bind(&NavServerComponent::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavServerComponent::handleCancel, this, std::placeholders::_1),
    std::bind(&NavServerComponent::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "NavServerComponent ready.");
  RCLCPP_INFO(
    this->get_logger(),
    "Using tf2 transform: %s -> %s",
    fixed_frame_.c_str(),
    robot_frame_.c_str());
}

void NavServerComponent::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = msg->header.stamp;
  transform.header.frame_id = msg->header.frame_id.empty() ? fixed_frame_ : msg->header.frame_id;
  transform.child_frame_id = msg->child_frame_id.empty() ? robot_frame_ : msg->child_frame_id;

  transform.transform.translation.x = msg->pose.pose.position.x;
  transform.transform.translation.y = msg->pose.pose.position.y;
  transform.transform.translation.z = msg->pose.pose.position.z;

  transform.transform.rotation = msg->pose.pose.orientation;

  fixed_frame_ = transform.header.frame_id;
  robot_frame_ = transform.child_frame_id;

  tf_broadcaster_->sendTransform(transform);
  odom_received_ = true;
}

bool NavServerComponent::getRobotPoseFromTf(double & x, double & y, double & theta)
{
  if (!odom_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for /odom to publish TF...");
    return false;
  }

  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(
        fixed_frame_,
        robot_frame_,
        tf2::TimePointZero);

    x = transform.transform.translation.x;
    y = transform.transform.translation.y;

    const auto & q = transform.transform.rotation;
    theta = yawFromQuaternion(q.x, q.y, q.z, q.w);

    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Could not get TF transform %s -> %s: %s",
      fixed_frame_.c_str(),
      robot_frame_.c_str(),
      ex.what());
    return false;
  }
}

rclcpp_action::GoalResponse NavServerComponent::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received goal x=%.2f y=%.2f theta=%.2f",
    goal->x,
    goal->y,
    goal->theta);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavServerComponent::handleCancel(
  const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavServerComponent::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread(&NavServerComponent::execute, this, goal_handle).detach();
}

void NavServerComponent::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateToPose::Result>();
  auto feedback = std::make_shared<NavigateToPose::Feedback>();

  rclcpp::Rate rate(20.0);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);

      result->success = false;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      return;
    }

    double x;
    double y;
    double theta;

    if (!getRobotPoseFromTf(x, y, theta)) {
      rate.sleep();
      continue;
    }

    const double dx = goal->x - x;
    const double dy = goal->y - y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(target_heading - theta);
    const double final_theta_error = normalizeAngle(goal->theta - theta);

    feedback->current_x = x;
    feedback->current_y = y;
    feedback->current_theta = theta;
    feedback->distance_remaining = dist;
    feedback->heading_error = heading_error;
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::Twist cmd;

    if (dist > 0.10) {
      cmd.angular.z = std::clamp(2.0 * heading_error, -1.5, 1.5);

      if (std::abs(heading_error) < 0.35) {
        cmd.linear.x = std::clamp(0.6 * dist, 0.0, 0.5);
      } else {
        cmd.linear.x = 0.0;
      }
    } else if (std::abs(final_theta_error) > 0.05) {
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(2.0 * final_theta_error, -1.0, 1.0);
    } else {
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);

      result->success = true;
      result->message = "Goal reached";
      goal_handle->succeed(result);
      return;
    }

    cmd_pub_->publish(cmd);
    rate.sleep();
  }

  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);

  result->success = false;
  result->message = "Node stopped";
  goal_handle->abort(result);
}

}  // namespace rt2_nav_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_nav_cpp::NavServerComponent)