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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__FUSION_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__FUSION_HPP_
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
#define LAT_DEBUG

namespace nodes
{
namespace rclcpp_system
{

class Fusion : public rclcpp::Node
{
public:
  explicit Fusion(const FusionSettings & settings)
  : Node(settings.node_name), miss_count_(0),
    number_crunch_limit_(settings.number_crunch_limit)
  {
#ifdef LAT_DEBUG
    subscription_[0] = this->create_subscription<latency_t>(
      settings.input_0, 1,
      [this](const latency_t::SharedPtr msg) {input_callback(0U, msg);});

    subscription_[1] = this->create_subscription<latency_t>(
      settings.input_1, 1,
      [this](const latency_t::SharedPtr msg) {input_callback(1U, msg);});
    publisher_ = this->create_publisher<latency_t>(settings.output_topic, 1);
#else
    subscription_[0] = this->create_subscription<message_t>(
      settings.input_0, 10,
      [this](const message_t::SharedPtr msg) {input_callback(0U, msg);});

    subscription_[1] = this->create_subscription<message_t>(
      settings.input_1, 10,
      [this](const message_t::SharedPtr msg) {input_callback(1U, msg);});
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
#endif
  }

#ifdef LAT_DEBUG
// To access callback variable for PiCAS
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_one() {return subscription_[0];}
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_two() {return subscription_[1];}
#else
  // To access callback variable for PiCAS
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_one() {return subscription_[0];}
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_two() {return subscription_[1];}
#endif
  
private:

#ifdef LAT_DEBUG
  void input_callback(
    const uint64_t input_number,
    const latency_t::SharedPtr input_message)
  {
    message_cache_[input_number] = input_message;

    // only process and publish when we can perform an actual fusion, this means
    // we have received a sample from each subscription
    std::string str1 = this->get_name();
    std::string str2 = "VehicleInterface";
    if (!message_cache_[0] || !message_cache_[1]) {
      if (str1.compare(str2) == 0) {
        miss_count_++;
        
        if (!message_cache_[0])
          std::cout << "[VehicleInterface] subscription topic 0 is empty." << std::endl;
        else if (!message_cache_[1])
          std::cout << "[VehicleInterface] subscription topic 1 is empty." << std::endl;
        
        std::cout << "[VehicleInterface] " << "input_topic number " << input_number << ", miss count is " << miss_count_ << std::endl;
      }
      return;
    }

    auto message = latency_t();
    
    if (!message_cache_[0]) {
      message.sequency = message_cache_[1]->sequency;
      message.start_stamp = message_cache_[1]->start_stamp;
      message.cyclic_start = message_cache_[1]->cyclic_start;
      message.start_node_name = message_cache_[1]->start_node_name;
    } else if (!message_cache_[1]) {
      message.sequency = message_cache_[0]->sequency;
      message.start_stamp = message_cache_[0]->start_stamp;
      message.cyclic_start = message_cache_[0]->cyclic_start;
      message.start_node_name = message_cache_[0]->start_node_name;
    } else {
      // For fusion node, the latency should follow the oldest message
      if (message_cache_[0]->sequency < message_cache_[1]->sequency) {
        message.sequency = message_cache_[0]->sequency;
        message.start_stamp = message_cache_[0]->start_stamp;
        message.cyclic_start = message_cache_[0]->cyclic_start;
        message.start_node_name = message_cache_[0]->start_node_name;
      } else {
        message.sequency = message_cache_[1]->sequency;
        message.start_stamp = message_cache_[1]->start_stamp;
        message.cyclic_start = message_cache_[1]->cyclic_start;
        message.start_node_name = message_cache_[1]->start_node_name;
      }
    }

    if (str1.compare(str2) == 0) {
      std::cout << "[VehicleInterface] Topic is published, sequency number is " << message.sequency << std::endl;
    }

    auto number_cruncher_result = number_cruncher(number_crunch_limit_);
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

    message_cache_[0].reset();
    message_cache_[1].reset();
  }
#else
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message)
  {
    message_cache_[input_number] = input_message;

    // only process and publish when we can perform an actual fusion, this means
    // we have received a sample from each subscription
    if (!message_cache_[0] || !message_cache_[1]) {
      return;
    }

    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    auto output_message = publisher_->borrow_loaned_message();
    fuse_samples(
      this->get_name(), output_message.get(), message_cache_[0],
      message_cache_[1]);
    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));

    message_cache_[0].reset();
    message_cache_[1].reset();
  }
#endif

private:

#ifdef LAT_DEBUG
  latency_t::SharedPtr message_cache_[2];
  rclcpp::Publisher<latency_t>::SharedPtr publisher_;
  rclcpp::Subscription<latency_t>::SharedPtr subscription_[2];
#else
  message_t::SharedPtr message_cache_[2];
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::Subscription<message_t>::SharedPtr subscription_[2];
#endif
  uint64_t miss_count_;
  uint64_t number_crunch_limit_;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__FUSION_HPP_
