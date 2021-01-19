// Copyright 2021 hasgwa. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <string>
#include <memory>

#include "topic_statistics_sample/laser_talker.hpp"

using namespace std::chrono_literals;

LaserTalker::LaserTalker(
  const std::string & topic_name,
  std::chrono::milliseconds publish_period)
: Node("laser_talker"),
  topic_name_(topic_name),
  publish_period_(publish_period)
{}

void LaserTalker::initialize()
{
  RCLCPP_INFO(get_logger(), "Talker starting up");

  publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    topic_name_,
    10 /* QoS history_depth */);
  publish_timer_ = create_wall_timer(
    publish_period_,
    [this]() -> void {
      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = this->now();

      RCLCPP_DEBUG(
        get_logger(), "Publishing header: %u",
        msg.header.stamp.nanosec);
      publisher_->publish(msg);
    });
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto imu_talker = std::make_shared<LaserTalker>("/laser", 100ms);
  imu_talker->initialize();
  executor.add_node(imu_talker);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
