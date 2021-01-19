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

#ifndef TOPIC_STATISTICS_SAMPLE__LISTENER_HPP_
#define TOPIC_STATISTICS_SAMPLE__LISTENER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener(
    const std::string & imu_topic_name,
    const rclcpp::SubscriptionOptions & imu_subscription_options,
    const std::string & laser_topic_name,
    const rclcpp::SubscriptionOptions & laser_subscription_options);

  void initialize();

  void start_listening();

private:
  const std::string imu_topic_name_;
  rclcpp::SubscriptionOptions imu_sub_opt_;
  const std::string laser_topic_name_;
  rclcpp::SubscriptionOptions laser_sub_opt_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_ = nullptr;
};

#endif  // TOPIC_STATISTICS_SAMPLE__LISTENER_HPP_
