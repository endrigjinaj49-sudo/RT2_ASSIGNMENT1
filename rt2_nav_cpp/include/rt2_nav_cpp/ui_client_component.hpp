#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rt2_nav_cpp/action/navigate_to_pose.hpp"

#include <thread>
#include <memory>

namespace rt2_nav_cpp
{

class UiClient : public rclcpp::Node
{
public:
  using NavigateToPose = rt2_nav_cpp::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit UiClient(const rclcpp::NodeOptions & options);
  ~UiClient() override;

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandleNavigate::SharedPtr goal_handle_;
  std::thread input_thread_;
  bool running_{true};

  void inputLoop();
  void sendGoal(double x, double y, double theta);
  void cancelGoal();
};

}  // namespace rt2_nav_cpp