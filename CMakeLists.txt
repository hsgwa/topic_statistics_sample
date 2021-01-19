cmake_minimum_required(VERSION 3.5)
project(topic_statistics_sample)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(sensor_msgs REQUIRED)
find_package(statistics_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_executable(imu_talker
  src/imu_talker.cpp)
ament_target_dependencies(imu_talker
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "statistics_msgs")

add_executable(laser_talker
  src/laser_talker.cpp)
ament_target_dependencies(laser_talker
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "statistics_msgs")

add_executable(listener
  src/listener.cpp
  src/statistics_listener.cpp)
ament_target_dependencies(listener
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "statistics_msgs")

install(TARGETS imu_talker DESTINATION lib/${PROJECT_NAME})
install(TARGETS laser_talker DESTINATION lib/${PROJECT_NAME})
install(TARGETS listener DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
