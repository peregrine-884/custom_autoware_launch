cmake_minimum_required(VERSION 3.14)
project(common_agilex_vehicle)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(agilex_joy_controller
  src/agilex_joy_controller.cpp
)
target_include_directories(agilex_joy_controller PRIVATE include)
ament_target_dependencies(agilex_joy_controller
  autoware_control_msgs
  autoware_vehicle_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS agilex_joy_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
