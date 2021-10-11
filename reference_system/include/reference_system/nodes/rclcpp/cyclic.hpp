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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#include <chrono>
#include <string>
#include <utility>
#include <vector>

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

class Cyclic : public rclcpp::Node
{
public:
  explicit Cyclic(const CyclicSettings & settings)
  : Node(settings.node_name), sequency_(0),
    number_crunch_limit_(settings.number_crunch_limit)
  {
#ifdef LAT_DEBUG
    uint64_t input_number = 0U;
    for (const auto & input_topic : settings.inputs) {
      subscriptions_.emplace_back(
        this->create_subscription<latency_t>(
          input_topic, BUFFER_SIZE,
          [this, input_number](const latency_t::SharedPtr msg) {
            input_callback(input_number, msg);
          }));
      ++input_number;
    }
    message_cache_.resize(subscriptions_.size());
    publisher_ = this->create_publisher<latency_t>(settings.output_topic, BUFFER_SIZE);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});
#else
    uint64_t input_number = 0U;
    for (const auto & input_topic : settings.inputs) {
      subscriptions_.emplace_back(
        this->create_subscription<message_t>(
          input_topic, 10,
          [this, input_number](const message_t::SharedPtr msg) {
            input_callback(input_number, msg);
          }));
      ++input_number;
    }
    message_cache_.resize(subscriptions_.size());
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});
#endif
  }

#ifdef LAT_DEBUG
  // To access callback variable for PiCAS
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_one() {return subscriptions_[0];}
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_two() {return subscriptions_[1];}
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_three() {return subscriptions_[2];}
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_four() {return subscriptions_[3];}
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_five() {return subscriptions_[4];}
  rclcpp::Subscription<latency_t>::SharedPtr get_subcallback_six() {return subscriptions_[5];}
#else
  // To access callback variable for PiCAS
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_one() {return subscriptions_[0];}
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_two() {return subscriptions_[1];}
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_three() {return subscriptions_[2];}
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_four() {return subscriptions_[3];}
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_five() {return subscriptions_[4];}
  rclcpp::Subscription<message_t>::SharedPtr get_subcallback_six() {return subscriptions_[5];}
#endif
  rclcpp::TimerBase::SharedPtr get_callback() {return timer_;}

private:
#ifdef LAT_DEBUG
  void input_callback(
    const uint64_t input_number,
    const latency_t::SharedPtr input_message)
  {
    message_cache_[input_number] = input_message;

    uint64_t timestamp = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch())
      .count());

    std::ofstream latency_debug;
    std::string home_dir = std::getenv("HOME");
    latency_debug.open(home_dir + "/Documents/latency.txt", std::ios::app);
    latency_debug << this->get_name() << "_subscription" << "_" << input_number << "," << timestamp << "," << 
    input_message->start_node_name << "," << input_message->sequency << "," << input_message->start_stamp << "\n";
    latency_debug.close();
  }

  void timer_callback()
  {
    auto local_cache = message_cache_;
    for (auto & m : message_cache_) {
      m.reset();
    }

    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    uint64_t sent_samples = 0;
          
    for (auto & m : local_cache) {
      if (!m) {continue;}

      auto message = latency_t();
      message.sequency = m->sequency;
      message.start_stamp = m->start_stamp;
      message.cyclic_start = m->cyclic_start;
      message.start_node_name = m->start_node_name;

      publisher_->publish(std::move(message));
      m.reset();
      ++sent_samples;

      uint64_t timestamp = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch())
      .count());
      message.cyclic_start = 1;
      
      std::ofstream latency_debug;
      std::string home_dir = std::getenv("HOME");
      latency_debug.open(home_dir + "/Documents/latency.txt", std::ios::app);
      latency_debug << this->get_name() << "," << timestamp << "," << 
      message.start_node_name << "," << message.sequency << "," << message.start_stamp << "," << message.cyclic_start << "\n";
      latency_debug.close();
    }
    
    if (sent_samples == 0) {
      auto message = latency_t();

      sequency_++;

      message.sequency = sequency_;
      message.start_stamp = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch())
      .count());
      message.cyclic_start = 1;
      message.start_node_name = this->get_name();

      publisher_->publish(std::move(message));

      std::ofstream latency_debug;
      std::string home_dir = std::getenv("HOME");
      latency_debug.open(home_dir + "/Documents/latency.txt", std::ios::app);
      latency_debug << this->get_name() << "," << message.start_stamp << "," << 
      message.start_node_name << "," << message.sequency << "," << message.start_stamp << "," << message.cyclic_start << "\n";
      latency_debug.close();
    }
  }
#else
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message)
  {
    message_cache_[input_number] = input_message;

    //std::cout << "[CyclicNodeInputCallback] " << this->get_name() << std::endl;
  }

  void timer_callback()
  {
    auto local_cache = message_cache_;
    for (auto & m : message_cache_) {
      m.reset();
    }

    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    uint64_t sent_samples = 0;
   
    
    for (auto & m : local_cache) {
      if (!m) {continue;}

      auto output_message = publisher_->borrow_loaned_message();

      fuse_samples(this->get_name(), output_message.get(), m);
      output_message.get().data[0] = number_cruncher_result;

      publisher_->publish(std::move(output_message));
      m.reset();
      ++sent_samples;
    }
    
    if (sent_samples == 0) {
      auto message = publisher_->borrow_loaned_message();
      message.get().size = 0;

      set_sample(this->get_name(), message.get());
      message.get().data[0] = number_cruncher_result;

      publisher_->publish(std::move(message));
    }
  }
#endif
private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t sequency_;
  uint64_t number_crunch_limit_;

#ifdef LAT_DEBUG
  rclcpp::Publisher<latency_t>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<latency_t>::SharedPtr> subscriptions_;
  std::vector<latency_t::SharedPtr> message_cache_;
#else
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<message_t>::SharedPtr> subscriptions_;
  std::vector<message_t::SharedPtr> message_cache_;
#endif
  
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
