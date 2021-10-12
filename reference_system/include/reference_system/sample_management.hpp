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
#ifndef REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
#define REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <iostream>

#include "reference_system/msg_types.hpp"

bool set_benchmark_mode(const bool has_benchmark_mode, const bool set_value = true)
{
  static bool value{false};
  if (set_value) {value = has_benchmark_mode;}
  return value;
}

bool is_in_benchmark_mode()
{
  return set_benchmark_mode(false, false);
}

uint64_t now_as_int() {
  return  static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());
}

template<typename SampleTypePointer>
void set_sample(const std::string & node_name, const uint32_t sequence_number, const uint32_t dropped_samples,
    const uint64_t timestamp, SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() ) {return;}

  if (sample.size >= message_t::STATS_CAPACITY) {
    return;
  }

  uint64_t idx = sample.size;
  ++sample.size;

  memcpy(
    sample.stats[idx].node_name.data(), node_name.data(),
    std::min(
      node_name.size(),
      reference_interfaces::msg::TransmissionStats::NODE_NAME_LENGTH));

  sample.stats[idx].timestamp = timestamp;

  sample.stats[idx].sequence_number = sequence_number;
  sample.stats[idx].dropped_samples = dropped_samples;
}

template<typename SampleTypePointer>
uint64_t get_sample_timestamp(SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size == 0) {
    return 0;
  } else {
    return sample->stats[sample->size - 1].timestamp;
  }
}

template<typename SampleTypePointer, typename SourceType>
void merge_history_into_sample(SampleTypePointer & sample, const SourceType& source ) 
{
  if (is_in_benchmark_mode()) return;

  std::vector<uint64_t> entries_to_add;

  for(uint64_t i = 0; i < source->size; ++i) {
    bool entry_found = false;
    std::string source_name((const char*)source->stats[i].node_name.data());

    for(uint64_t k = 0; k < sample.size; ++k) {
      std::string sample_name((const char*)sample.stats[k].node_name.data());
      if ( source_name == sample_name ) {
        entry_found = true;
        break;
      }
    }

    if ( !entry_found ) entries_to_add.emplace_back(i);
  }

  for(auto i : entries_to_add) {
    memcpy(sample.stats.data() + sample.size, source->stats.data() + i, 
           sizeof(reference_interfaces::msg::TransmissionStats));
    ++sample.size;
  }
}

struct sample_statistic_t
{
  uint64_t number_of_received_samples = 0;
  uint64_t number_of_hot_path_samples = 0;
  uint64_t number_of_behavior_planner_samples = 0;
  uint64_t timepoint_of_first_received_sample = 0;
  uint32_t previous_behavior_planner_sequence = 0;
  uint64_t previous_behavior_planner_time_stamp = 0;

  struct statistic_value_t
  {
    uint64_t average = 0;
    uint64_t min = std::numeric_limits<uint64_t>::max();
    uint64_t max = 0;
    uint64_t current = 0;
    std::string suffix;
    double adjustment = 0.0;

    void set(const uint64_t value, const uint64_t total_number)
    {
      current = value;
      average = ((total_number - 1) * average + value) / total_number;
      min = std::min(min, value);
      max = std::max(max, value);
    }

  };


  statistic_value_t latency;
  statistic_value_t hot_path_latency;
  statistic_value_t behavior_planner_period;
};

std::ostream &operator<<(std::ostream & output, const sample_statistic_t::statistic_value_t & v) {
  if ( v.adjustment == 0.0 ) {
    output << v.current << " " << v.suffix << "  [ min = " << v.min << " " << v.suffix 
           << ", max = " << v.max << " " << v.suffix << ", average = " << v.average 
           << " " << v.suffix << " ]";
  } else {
    output << static_cast<double>(v.current)/v.adjustment << " " << v.suffix 
            << "  [ min = " << static_cast<double>(v.min)/v.adjustment << " " << v.suffix 
           << ", max = " << static_cast<double>(v.max)/v.adjustment << " " << v.suffix 
           << ", average = " << static_cast<double>(v.average)/v.adjustment << " " << v.suffix << " ]";
  }
  return output;
}

template<typename SampleTypePointer>
void print_sample_path(
  const std::string & node_name,
  const SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size <= 0) {return;}

  static std::map<std::string, sample_statistic_t> advanced_statistics;
  auto iter = advanced_statistics.find(node_name);
  if (iter == advanced_statistics.end() ) {
    advanced_statistics[node_name].timepoint_of_first_received_sample =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    advanced_statistics[node_name].latency.suffix = "ms";
    advanced_statistics[node_name].latency.adjustment = 1000000.0;
    advanced_statistics[node_name].hot_path_latency.suffix = "ms";
    advanced_statistics[node_name].hot_path_latency.adjustment = 1000000.0;
    advanced_statistics[node_name].behavior_planner_period.suffix = "ms";
    advanced_statistics[node_name].behavior_planner_period.adjustment = 1000000.0;
  }

  advanced_statistics[node_name].number_of_received_samples++;

  const uint64_t timestamp_in_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << "sample path: " << std::endl;
  std::cout << "  order timepoint           sequence nr.  node name" << std::endl;

  std::map<uint64_t, uint64_t> timestamp2Order;
  uint64_t min_time_stamp = std::numeric_limits<uint64_t>::max();
  uint64_t max_time_stamp = 0;

  for (uint64_t i = 0; i < sample->size; ++i) {
    timestamp2Order[sample->stats[i].timestamp] = 0;
    min_time_stamp = std::min(min_time_stamp, sample->stats[i].timestamp);
    max_time_stamp = std::max(max_time_stamp, sample->stats[i].timestamp);
  }
  uint64_t i = 0;
  for (auto & e : timestamp2Order) {
    e.second = i++;
  }

  for (uint64_t i = 0; i < sample->size; ++i) {
    std::cout << "  [";
    std::cout.width(2);
    std::cout << timestamp2Order[sample->stats[i].timestamp];
    std::cout << "]  " << sample->stats[i].timestamp << "  ";
    std::cout.width(10);
    std::cout << sample->stats[i].sequence_number;
    std::cout << "   " <<
      sample->stats[i].node_name.data() << std::endl;
  }

  std::cout << "  [";
  std::cout.width(2);
  std::cout << timestamp2Order.size();
  std::cout << "]  " << timestamp_in_ns << "  ";
  std::cout.width(10);
  std::cout << "endpoint";
  std::cout << "   " << node_name << std::endl;

  // hot path latency
  uint64_t hot_path_latency_in_ns = 0;
  bool does_contain_hot_path = false;
  for (uint64_t i = 0; i < sample->size; ++i) {
    uint64_t idx = sample->size - i - 1;
    std::string current_node_name(
      reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));

    if (current_node_name == "ObjectCollisionEstimator") {
      hot_path_latency_in_ns = sample->stats[idx].timestamp;
    } else if (current_node_name == "FrontLidarDriver" &&
      hot_path_latency_in_ns != 0)
    {
      hot_path_latency_in_ns = hot_path_latency_in_ns - sample->stats[idx].timestamp;
      does_contain_hot_path = true;
      break;
    }
  }

  // behavior planner cycle time
  bool does_contain_behavior_planner = false;
  for(uint64_t i = 0; i < sample->size; ++i) {
    std::string current_node_name(
      reinterpret_cast<const char *>(sample->stats[i].node_name.data()));
    if ( current_node_name == "BehaviorPlanner" ) {
      does_contain_behavior_planner = true;
      auto seq_nr = sample->stats[i].sequence_number;
      auto timestamp = sample->stats[i].timestamp;
      auto prev_seq_nr = advanced_statistics[node_name].previous_behavior_planner_sequence;
      auto prev_timestamp = advanced_statistics[node_name].previous_behavior_planner_time_stamp;
      if ( advanced_statistics[node_name].number_of_behavior_planner_samples != 0 ) {
        advanced_statistics[node_name].behavior_planner_period.set(
            static_cast<double>(timestamp - prev_timestamp)/
            static_cast<double>(seq_nr - prev_seq_nr), advanced_statistics[node_name].number_of_behavior_planner_samples);
      }
      advanced_statistics[node_name].previous_behavior_planner_sequence = seq_nr;
      advanced_statistics[node_name].previous_behavior_planner_time_stamp = timestamp;
      ++advanced_statistics[node_name].number_of_behavior_planner_samples;
    }
  }

  advanced_statistics[node_name].latency.set(max_time_stamp - min_time_stamp,
    advanced_statistics[node_name].number_of_received_samples);

  std::cout << std::endl;
  std::cout << "Statistics:" << std::endl;
  std::cout << "  latency:                  " << advanced_statistics[node_name].latency << std::endl;

  if (does_contain_hot_path) {
    advanced_statistics[node_name].number_of_hot_path_samples++;
    advanced_statistics[node_name].hot_path_latency.set(hot_path_latency_in_ns,
      advanced_statistics[node_name].number_of_hot_path_samples);
    std::cout << "  hot path:                 FrontLidarDriver -> ObjectCollisionEstimator" << std::endl;
    std::cout << "  hot path latency:         " << advanced_statistics[node_name].hot_path_latency << std::endl;
  }

  if (does_contain_behavior_planner) {
    std::cout << "  behavior planner period:  " << advanced_statistics[node_name].behavior_planner_period << std::endl;
  }

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << std::endl;
}

#endif  // REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
