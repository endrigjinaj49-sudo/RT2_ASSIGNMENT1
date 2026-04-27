cmake_minimum_required(VERSION 3.8)
project(rt2_nav_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_library(nav_server_component SHARED
  src/nav_server_component.cpp
)
ament_target_dependencies(nav_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(nav_server_component
  "${cpp_typesupport_target}"
)
rclcpp_components_register_nodes(nav_server_component
  "rt2_nav_cpp::NavServerComponent"
)

add_library(ui_client_component SHARED
  src/ui_client_component.cpp
)
ament_target_dependencies(ui_client_component
  rclcpp
  rclcpp_action
  rclcpp_components
)
target_link_libraries(ui_client_component
  "${cpp_typesupport_target}"
)
rclcpp_components_register_nodes(ui_client_component
  "rt2_nav_cpp::UiClient"
)

add_executable(manual_container
  src/manual_container_main.cpp
)
ament_target_dependencies(manual_container
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(manual_container
  nav_server_component
  ui_client_component
  "${cpp_typesupport_target}"
)

install(TARGETS
  nav_server_component
  ui_client_component
  manual_container
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()