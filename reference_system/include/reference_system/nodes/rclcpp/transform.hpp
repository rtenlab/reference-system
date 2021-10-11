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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

#include <iostream>
#include <fstream>

namespace nodes
{
namespace rclcpp_system
{

class Transform : public rclcpp::Node
{
public:
  explicit Transform(const TransformSettings & settings)
  : Node(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
  {
#ifdef LAT_DEBUG
    subscription_ = this->create_subscription<latency_t>(
      settings.input_topic, BUFFER_SIZE,
      [this](const latency_t::SharedPtr msg) {input_callback(msg);});
    publisher_ = this->create_publisher<latency_t>(settings.output_topic, BUFFER_SIZE);
#else
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 10,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
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
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    //auto output_message = publisher_->borrow_loaned_message();
    //fuse_samples(this->get_name(), output_message.get(), input_message);

    // use result so that it is not optimizied away by some clever compiler
    //output_message.get().data[0] = number_cruncher_result;
    //auto output_message = input_message;
    auto message = latency_t();
    message.sequency = input_message->sequency;
    message.start_stamp = input_message->start_stamp;
    message.cyclic_start = input_message->cyclic_start;
    message.start_node_name = input_message->start_node_name;

    publisher_->publish(std::move(message));

    uint64_t timestamp = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch())
      .count());

    std::ofstream latency_debug;
    std::string home_dir = std::getenv("HOME");
    latency_debug.open(home_dir + "/Documents/latency.txt", std::ios::app);
    latency_debug << this->get_name() << "," << timestamp << "," << 
    message.start_node_name << "," << message.sequency << "," << message.start_stamp << "\n";
    latency_debug.close();
  }
#else
  void input_callback(const message_t::SharedPtr input_message) const
  {
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    auto output_message = publisher_->borrow_loaned_message();

    fuse_samples(this->get_name(), output_message.get(), input_message);

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));
  }
#endif

private:

#ifdef LAT_DEBUG
  rclcpp::Publisher<latency_t>::SharedPtr publisher_;
  rclcpp::Subscription<latency_t>::SharedPtr subscription_;
#else
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
#endif

  uint64_t number_crunch_limit_;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
