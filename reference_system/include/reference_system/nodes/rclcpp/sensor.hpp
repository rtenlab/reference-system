// Copyright 2021 Apex.AI, Inc.
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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

#include <iostream>
#include <fstream>
#define LAT_DEBUG

namespace nodes
{
namespace rclcpp_system
{

class Sensor : public rclcpp::Node
{
public:
  explicit Sensor(const SensorSettings & settings)
  : Node(settings.node_name), sequency_(0)
  {
#ifdef LAT_DEBUG
    publisher_ = this->create_publisher<latency_t>(settings.topic_name, BUFFER_SIZE);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback_latency();});
#else
    publisher_ = this->create_publisher<message_t>(settings.topic_name, 10);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});
#endif
  }

  // To access callback variable for PiCAS
  rclcpp::TimerBase::SharedPtr get_callback() {return timer_;}

private:
#ifdef LAT_DEBUG
  void timer_callback_latency()
  {
    auto message = latency_t();

    sequency_++;

    message.sequency = sequency_;
    message.start_stamp = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());
    message.cyclic_start = 0;
    message.start_node_name = this->get_name();


    publisher_->publish(std::move(message));
    
    std::ofstream latency_debug;
    std::string home_dir = std::getenv("HOME");
    latency_debug.open(home_dir + "/Documents/latency.txt", std::ios::app);
    latency_debug << this->get_name() << "," << message.start_stamp << "," << 
    message.start_node_name << "," << message.sequency << "," << message.start_stamp << "\n";
    latency_debug.close();
  }
#else
  void timer_callback()
  {
    auto message = publisher_->borrow_loaned_message();
    message.get().size = 0;

    set_sample(this->get_name(), message.get());

    publisher_->publish(std::move(message));

  }
#endif

private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t sequency_;

#ifdef LAT_DEBUG
  rclcpp::Publisher<latency_t>::SharedPtr publisher_;
#else
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
#endif

};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
