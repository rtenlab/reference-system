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
#ifdef AAMF
#include "reference_system/aamf_wrappers.hpp"
#endif
namespace nodes
{
  namespace rclcpp_system
  {

    class Fusion : public rclcpp::Node
    {
    public:
      explicit Fusion(const FusionSettings &settings)
          : Node(settings.node_name),
            number_crunch_limit_(settings.number_crunch_limit)
      {
        subscriptions_[0].subscription = this->create_subscription<message_t>(
            settings.input_0, 1,
            [this](const message_t::SharedPtr msg)
            { input_callback(0U, msg); });

        subscriptions_[1].subscription = this->create_subscription<message_t>(
            settings.input_1, 1,
            [this](const message_t::SharedPtr msg)
            { input_callback(1U, msg); });
        publisher_ = this->create_publisher<message_t>(settings.output_topic, 1);
#ifdef PICAS
        subscriptions_[0].subscription->callback_priority = settings.callback_priority_1;
        subscriptions_[1].subscription->callback_priority = settings.callback_priority_2;
#endif
#ifdef AAMF
        this->request_publisher_ = this->create_publisher<aamf_server_interfaces::msg::GPURequest>("request_topic", 10);
        this->reg_publisher_ = this->create_publisher<aamf_server_interfaces::msg::GPURegister>("registration_topic", 10);
        *aamf_client_1 = aamf_client_wrapper(settings.callback_priority_1, settings.callback_priority_1, request_publisher_, reg_publisher_);
        this->register_sub_ = this->create_subscription<aamf_server_interfaces::msg::GPURegister>("handshake_topic", 100,
         [this](const aamf_server_interfaces::msg::GPURegister::SharedPtr msg){ aamf_client_1->handshake_callback(msg); });
        aamf_client_1->register_subscriber(register_sub_);
        aamf_client_1->send_handshake();

        *aamf_client_2 = aamf_client_wrapper(settings.callback_priority_1, settings.callback_priority_1, request_publisher_, reg_publisher_);
        this->register_sub_2 = this->create_subscription<aamf_server_interfaces::msg::GPURegister>("handshake_topic", 100, 
        [this](const aamf_server_interfaces::msg::GPURegister::SharedPtr msg){ aamf_client_2->handshake_callback(msg); });
        aamf_client_2->register_subscriber(register_sub_2);
        aamf_client_2->send_handshake();
#endif
      }

    private:
      void input_callback(
          const uint64_t input_number,
          const message_t::SharedPtr input_message)
      {
        uint64_t timestamp = now_as_int();
        subscriptions_[input_number].cache = input_message;

        // only process and publish when we can perform an actual fusion, this means
        // we have received a sample from each subscription
        if (!subscriptions_[0].cache || !subscriptions_[1].cache)
        {
          return;
        }

        auto number_cruncher_result = number_cruncher(number_crunch_limit_);

        auto output_message = publisher_->borrow_loaned_message();

        uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
                                      subscriptions_[0].cache,
                                      subscriptions_[0].sequence_number) +
                                  get_missed_samples_and_update_seq_nr(
                                      subscriptions_[1].cache,
                                      subscriptions_[1].sequence_number);

        output_message.get().size = 0;
        merge_history_into_sample(output_message.get(), subscriptions_[0].cache);
        merge_history_into_sample(output_message.get(), subscriptions_[1].cache);
        set_sample(
            this->get_name(), sequence_number_++, missed_samples, timestamp,
            output_message.get());

        output_message.get().data[0] = number_cruncher_result;
        publisher_->publish(std::move(output_message));

        subscriptions_[0].cache.reset();
        subscriptions_[1].cache.reset();
      }

    private:
      struct subscription_t
      {
        rclcpp::Subscription<message_t>::SharedPtr subscription;
        uint32_t sequence_number = 0;
        message_t::SharedPtr cache;
      };
      rclcpp::Publisher<message_t>::SharedPtr publisher_;

      subscription_t subscriptions_[2];
#ifdef AAMF
      aamf_client_wrapper *aamf_client_1;
      aamf_client_wrapper *aamf_client_2;
      rclcpp::Publisher<aamf_server_interfaces::msg::GPURequest>::SharedPtr request_publisher_;
      rclcpp::Publisher<aamf_server_interfaces::msg::GPURegister>::SharedPtr reg_publisher_;
      rclcpp::Subscription<aamf_server_interfaces::msg::GPURegister>::SharedPtr register_sub_;
      rclcpp::Subscription<aamf_server_interfaces::msg::GPURegister>::SharedPtr register_sub_2;
#endif
      uint64_t number_crunch_limit_;
      uint32_t sequence_number_ = 0;
    };
  } // namespace rclcpp_system
} // namespace nodes
#endif // REFERENCE_SYSTEM__NODES__RCLCPP__FUSION_HPP_
