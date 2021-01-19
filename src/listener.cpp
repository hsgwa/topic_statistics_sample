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

#include "topic_statistics_sample/listener.hpp"
#include "topic_statistics_sample/statistics_listener.hpp"

using namespace std::chrono_literals;

Listener::Listener(
  const std::string & imu_topic_name,
  const rclcpp::SubscriptionOptions & imu_subscription_options,
  const std::string & laser_topic_name,
  const rclcpp::SubscriptionOptions & laser_subscription_options)
: Node("listener"), imu_topic_name_(imu_topic_name),
  imu_sub_opt_(imu_subscription_options),
  laser_topic_name_(laser_topic_name),
  laser_sub_opt_(laser_subscription_options) {}

void Listener::initialize()
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  start_listening();
}

void Listener::start_listening()
{
  if (!imu_sub_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_name_, 10,   /* QoS history_depth */
      [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void {
        RCLCPP_DEBUG(
          get_logger(), "Listener heard: %u",
          msg->header.stamp.nanosec);
      },
      imu_sub_opt_);
  }

  if (!laser_sub_) {
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic_name_, 10,   /* QoS history_depth */
      [this](const typename sensor_msgs::msg::LaserScan::SharedPtr msg) -> void {
        RCLCPP_DEBUG(
          get_logger(), "Listener heard: %u",
          msg->header.stamp.nanosec);
        rclcpp::sleep_for(50ms);
      },
      laser_sub_opt_);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  imu_sub_opt.topic_stats_options.publish_topic = "/imu_topic_statistics";
  imu_sub_opt.topic_stats_options.publish_period = 5s;

  auto laser_sub_opt = rclcpp::SubscriptionOptions();
  laser_sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  laser_sub_opt.topic_stats_options.publish_topic = "/laser_topic_statistics";
  laser_sub_opt.topic_stats_options.publish_period = 5s;

  auto listener = std::make_shared<Listener>("/imu", imu_sub_opt, "/laser", laser_sub_opt);
  listener->initialize();

  auto statistics_listener =
    std::make_shared<TopicStatisticsListener>("/imu_topic_statistics");
  statistics_listener->initialize();

  executor.add_node(listener);
  executor.add_node(statistics_listener);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
