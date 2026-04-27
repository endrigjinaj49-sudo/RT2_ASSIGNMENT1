#include "rclcpp/rclcpp.hpp"
#include "rt2_nav_cpp/nav_server_component.hpp"
#include "rt2_nav_cpp/ui_client_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto nav_server = std::make_shared<rt2_nav_cpp::NavServerComponent>(options);
  auto ui_client = std::make_shared<rt2_nav_cpp::UiClient>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nav_server);
  executor.add_node(ui_client);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}