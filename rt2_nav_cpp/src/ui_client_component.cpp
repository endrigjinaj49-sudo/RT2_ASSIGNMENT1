#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_nav_cpp/action/navigate_to_pose.hpp"

#include <thread>
#include <sstream>
#include <string>
#include <iostream>
#include <memory>
#include <chrono>

namespace rt2_nav_cpp
{

class UiClient : public rclcpp::Node
{
public:
  using NavigateToPose = rt2_nav_cpp::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit UiClient(const rclcpp::NodeOptions & options)
  : Node("ui_client", options)
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    input_thread_ = std::thread([this]() { this->inputLoop(); });

    RCLCPP_INFO(this->get_logger(), "UiClient ready.");
    RCLCPP_INFO(this->get_logger(), "Type: x y theta");
    RCLCPP_INFO(this->get_logger(), "Or type: cancel");
  }

  ~UiClient() override
  {
    running_ = false;
    if (input_thread_.joinable()) {
      input_thread_.detach();
    }
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandleNavigate::SharedPtr goal_handle_;
  std::thread input_thread_;
  bool running_{true};

  void inputLoop()
  {
    while (rclcpp::ok() && running_) {
      std::string line;
      std::cout << "> ";
      if (!std::getline(std::cin, line)) {
        return;
      }

      if (line == "cancel") {
        cancelGoal();
        continue;
      }

      std::stringstream ss(line);
      double x, y, theta;
      if (!(ss >> x >> y >> theta)) {
        std::cout << "Use: x y theta  or  cancel" << std::endl;
        continue;
      }

      sendGoal(x, y, theta);
    }
  }

  void sendGoal(double x, double y, double theta)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.x = x;
    goal_msg.y = y;
    goal_msg.theta = theta;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
      [this](const GoalHandleNavigate::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        } else {
          goal_handle_ = handle;
          RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
      };

   options.feedback_callback =
  [this](
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback>) {
    (void)this;
  };

    options.result_callback =
      [this](const GoalHandleNavigate::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(
              this->get_logger(),
              "Goal finished: success=%s message=%s",
              result.result->success ? "true" : "false",
              result.result->message.c_str());
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Unknown result code");
            break;
        }
        goal_handle_.reset();
      };

    client_->async_send_goal(goal_msg, options);
  }

  void cancelGoal()
  {
    if (!goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
      return;
    }

    auto future = client_->async_cancel_goal(goal_handle_);
    (void)future;
    RCLCPP_INFO(this->get_logger(), "Cancel request sent");
  }
};

}  // namespace rt2_nav_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_nav_cpp::UiClient)