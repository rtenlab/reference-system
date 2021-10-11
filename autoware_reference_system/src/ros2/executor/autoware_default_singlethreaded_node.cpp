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

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  //auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>();

  rclcpp::executors::SingleThreadedExecutor executor;
  
  // Sensor nodes
  nodes::SensorSettings sensor;
  sensor.node_name = "FrontLidarDriver";
  sensor.topic_name = "FrontLidarDriver";
  sensor.cycle_time = TimeConfig::FRONT_LIDAR_DRIVER;
  auto FrontLidarDriver = std::make_shared<nodes::rclcpp_system::Sensor>(sensor);
  
  // Transform nodes
  nodes::TransformSettings transform;
  transform.node_name = "PointsTransformerFront";
  transform.input_topic = "FrontLidarDriver";
  transform.output_topic = "PointsTransformerFront";
  transform.number_crunch_limit = TimeConfig::POINTS_TRANSFORMER_FRONT;
  auto PointsTransformerFront = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  // Add nodes to executor
  executor.add_node(FrontLidarDriver);
  executor.add_node(PointsTransformerFront);
  
  std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin, &executor);

  spinThread1.join();
  
  executor.remove_node(FrontLidarDriver);
  executor.remove_node(PointsTransformerFront);
  
  rclcpp::shutdown();

  return 0;
}

