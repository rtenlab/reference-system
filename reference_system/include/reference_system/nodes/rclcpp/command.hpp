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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__COMMAND_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__COMMAND_HPP_

#include <chrono>
#include <string>

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

class Command : public rclcpp::Node
{
public:
  explicit Command(const CommandSettings & settings)
  : Node(settings.node_name)
  {
#ifdef LAT_DEBUG
    subscription_ = this->create_subscription<latency_t>(
      settings.input_topic, 1,
      [this](const latency_t::SharedPtr msg) {input_callback(msg);});
#else
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 10,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
#endif
  }

#ifdef LAT_DEBUG
  // To access callback variable for PiCAS
  rclcpp::Subscription<latency_t>::SharedPtr get_callback() {return subscription_;}
#else
  // To access callback variable for PiCAS
  rclcpp::Subscription<message_t>::SharedPtr get_callback() {return subscription_;}
#endif

private:
#ifdef LAT_DEBUG
  void input_callback(const latency_t::SharedPtr input_message) const
  {
    //print_sample_path(this->get_name(), input_message);
    uint64_t timestamp = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());

    std::ofstream latency_debug;
    std::string home_dir = std::getenv("HOME");
    latency_debug.open(home_dir + "/Documents/latency.txt", std::ios::app);
    latency_debug << this->get_name() << "," << timestamp << "," << 
    input_message->start_node_name << "," << input_message->sequency << "," << input_message->start_stamp << "\n";
    latency_debug.close();

    std::ofstream e2e_latency_debug;
    e2e_latency_debug.open(home_dir + "/Documents/e2e_latency.txt", std::ios::app);
    e2e_latency_debug << this->get_name() << "," << timestamp << "," << 
    input_message->start_node_name << "," << input_message->sequency << "," << input_message->start_stamp << "\n";
    e2e_latency_debug.close();

  }
#else
  void input_callback(const message_t::SharedPtr input_message) const
  {
    print_sample_path(this->get_name(), input_message);
  }
#endif
private:
#ifdef LAT_DEBUG
  rclcpp::Subscription<latency_t>::SharedPtr subscription_;
#else
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
#endif
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__COMMAND_HPP_
