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

#ifndef TOPIC_STATISTICS_SAMPLE__IMU_TALKER_HPP_
#define TOPIC_STATISTICS_SAMPLE__IMU_TALKER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuTalker : public rclcpp::Node
{
public:
  ImuTalker(
    const std::string & topic_name,
    std::chrono::milliseconds publish_period);

  void initialize();

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_ = nullptr;

  const std::string topic_name_;
  std::chrono::milliseconds publish_period_;
  rclcpp::TimerBase::SharedPtr publish_timer_ = nullptr;
};


#endif  // TOPIC_STATISTICS_SAMPLE__IMU_TALKER_HPP_
