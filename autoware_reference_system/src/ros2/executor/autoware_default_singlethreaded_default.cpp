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
  
  sensor.node_name = "RearLidarDriver";
  sensor.topic_name = "RearLidarDriver";
  sensor.cycle_time = TimeConfig::REAR_LIDAR_DRIVER;
  auto RearLidarDriver = std::make_shared<nodes::rclcpp_system::Sensor>(sensor);

  sensor.node_name = "PointCloudMap";
  sensor.topic_name = "PointCloudMap";
  sensor.cycle_time = TimeConfig::POINT_CLOUD_MAP;
  auto PointCloudMap = std::make_shared<nodes::rclcpp_system::Sensor>(sensor);

  sensor.node_name = "Visualizer";
  sensor.topic_name = "Visualizer";
  sensor.cycle_time = TimeConfig::VISUALIZER;
  auto Visualizer = std::make_shared<nodes::rclcpp_system::Sensor>(sensor);

  sensor.node_name = "Lanelet2Map";
  sensor.topic_name = "Lanelet2Map";
  sensor.cycle_time = TimeConfig::LANELET2MAP;
  auto Lanelet2Map = std::make_shared<nodes::rclcpp_system::Sensor>(sensor);

  // Transform nodes
  nodes::TransformSettings transform;
  transform.node_name = "PointsTransformerFront";
  transform.input_topic = "FrontLidarDriver";
  transform.output_topic = "PointsTransformerFront";
  transform.number_crunch_limit = TimeConfig::POINTS_TRANSFORMER_FRONT;
  auto PointsTransformerFront = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "PointsTransformerRear";
  transform.input_topic = "RearLidarDriver";
  transform.output_topic = "PointsTransformerRear";
  transform.number_crunch_limit = TimeConfig::POINTS_TRANSFORMER_REAR;
  auto PointsTransformerRear = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "VoxelGridDownsampler";
  transform.input_topic = "PointCloudFusion";
  transform.output_topic = "VoxelGridDownsampler";
  transform.number_crunch_limit = TimeConfig::VOXEL_GRID_DOWNSAMPLER;
  auto VoxelGridDownsampler = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "PointCloudMapLoader";
  transform.input_topic = "PointCloudMap";
  transform.output_topic = "PointCloudMapLoader";
  transform.number_crunch_limit = TimeConfig::POINT_CLOUD_MAP_LOADER;
  auto PointCloudMapLoader = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "RayGroundFilter";
  transform.input_topic = "PointCloudFusion";
  transform.output_topic = "RayGroundFilter";
  transform.number_crunch_limit = TimeConfig::RAY_GROUND_FILTER;
  auto RayGroundFilter = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "EuclideanClusterDetector";
  transform.input_topic = "RayGroundFilter";
  transform.output_topic = "EuclideanClusterDetector";
  transform.number_crunch_limit = TimeConfig::EUCLIDEAN_CLUSTER_DETECTOR;
  auto EuclideanClusterDetector = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "ObjectCollisionEstimator";
  transform.input_topic = "EuclideanClusterDetector";
  transform.output_topic = "ObjectCollisionEstimator";
  transform.number_crunch_limit = TimeConfig::OBJECT_COLLISION_ESTIMATOR;
  auto ObjectCollisionEstimator = std::make_shared<nodes::rclcpp_system::Transform>(transform);  

  transform.node_name = "MPCController";
  transform.input_topic = "BehaviorPlanner";
  transform.output_topic = "MPCController";
  transform.number_crunch_limit = TimeConfig::MPC_CONTROLLER;
  auto MPCController = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "ParkingPlanner";
  transform.input_topic = "Lanelet2MapLoader";
  transform.output_topic = "ParkingPlanner";
  transform.number_crunch_limit = TimeConfig::PARKING_PLANNER;
  auto ParkingPlanner = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  transform.node_name = "LanePlanner";
  transform.input_topic = "Lanelet2MapLoader";
  transform.output_topic = "LanePlanner";
  transform.number_crunch_limit = TimeConfig::LANE_PLANNER;
  auto LanePlanner = std::make_shared<nodes::rclcpp_system::Transform>(transform);

  // Fusion nodes
  nodes::FusionSettings fusion;
  fusion.node_name = "PointCloudFusion";
  fusion.input_0 = "PointsTransformerFront";
  fusion.input_1 = "PointsTransformerRear";
  fusion.output_topic = "PointCloudFusion";
  fusion.number_crunch_limit = TimeConfig::POINT_CLOUD_FUSION;
  auto PointCloudFusion = std::make_shared<nodes::rclcpp_system::Fusion>(fusion); 

  fusion.node_name = "NDTLocalizer";
  fusion.input_0 = "VoxelGridDownsampler";
  fusion.input_1 = "PointCloudMapLoader";
  fusion.output_topic = "NDTLocalizer";
  fusion.number_crunch_limit = TimeConfig::NDT_LOCALIZER;
  auto NDTLocalizer = std::make_shared<nodes::rclcpp_system::Fusion>(fusion);

  fusion.node_name = "VehicleInterface";
  fusion.input_0 = "MPCController";
  fusion.input_1 = "BehaviorPlanner";
  fusion.output_topic = "VehicleInterface";
  fusion.number_crunch_limit = TimeConfig::VEHICLE_INTERFACE;
  auto VehicleInterface = std::make_shared<nodes::rclcpp_system::Fusion>(fusion);

  fusion.node_name = "Lanelet2GlobalPlanner";
  fusion.input_0 = "Visualizer";
  fusion.input_1 = "NDTLocalizer";
  fusion.output_topic = "Lanelet2GlobalPlanner";
  fusion.number_crunch_limit = TimeConfig::LANELET_2_GLOBAL_PLANNER;
  auto Lanelet2GlobalPlanner = std::make_shared<nodes::rclcpp_system::Fusion>(fusion);

  fusion.node_name = "Lanelet2MapLoader";
  fusion.input_0 = "Lanelet2Map";
  fusion.input_1 = "Lanelet2GlobalPlanner";
  fusion.output_topic = "Lanelet2MapLoader";
  fusion.number_crunch_limit = TimeConfig::LANELET_2_MAP_LOADER;
  auto Lanelet2MapLoader = std::make_shared<nodes::rclcpp_system::Fusion>(fusion);

  // Cyclic node
  nodes::CyclicSettings cyclic;
  cyclic.node_name = "BehaviorPlanner";
  cyclic.inputs = {"ObjectCollisionEstimator", "NDTLocalizer",
      "Lanelet2GlobalPlanner", "Lanelet2MapLoader",
      "ParkingPlanner", "LanePlanner"};
  cyclic.output_topic = "BehaviorPlanner";
  cyclic.cycle_time = TimeConfig::BEHAVIOR_PLANNER_CYCLE;
  cyclic.number_crunch_limit = TimeConfig::BEHAVIOR_PLANNER;
  auto BehaviorPlanner = std::make_shared<nodes::rclcpp_system::Cyclic>(cyclic);  

  // Command node
  nodes::CommandSettings command;
  command.node_name = "VehicleDBWSystem";
  command.input_topic = "VehicleInterface";
  auto VehicleDBWSystem = std::make_shared<nodes::rclcpp_system::Command>(command);

  // Add nodes to executor
  executor.add_node(FrontLidarDriver);
  executor.add_node(RearLidarDriver);
  executor.add_node(PointCloudMap);
  executor.add_node(Visualizer);
  executor.add_node(Lanelet2Map);
  executor.add_node(PointsTransformerFront);
  executor.add_node(PointsTransformerRear);
  executor.add_node(PointCloudMapLoader);
  executor.add_node(PointCloudFusion);
  executor.add_node(VoxelGridDownsampler);
  executor.add_node(RayGroundFilter);
  executor.add_node(EuclideanClusterDetector);
  executor.add_node(LanePlanner);
  executor.add_node(ParkingPlanner);
  executor.add_node(Lanelet2MapLoader);
  executor.add_node(Lanelet2GlobalPlanner);
  executor.add_node(NDTLocalizer);
  executor.add_node(ObjectCollisionEstimator);
  executor.add_node(BehaviorPlanner);
  executor.add_node(MPCController);
  executor.add_node(VehicleInterface);
  executor.add_node(VehicleDBWSystem);

  
  std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin, &executor);

  spinThread1.join();
  
  executor.remove_node(FrontLidarDriver);
  executor.remove_node(RearLidarDriver);
  executor.remove_node(PointCloudMap);
  executor.remove_node(Visualizer);
  executor.remove_node(Lanelet2Map);
  executor.remove_node(PointsTransformerFront);
  executor.remove_node(PointsTransformerRear);
  executor.remove_node(PointCloudMapLoader);
  executor.remove_node(PointCloudFusion);
  executor.remove_node(VoxelGridDownsampler);
  executor.remove_node(RayGroundFilter);
  executor.remove_node(EuclideanClusterDetector);
  executor.remove_node(LanePlanner);
  executor.remove_node(ParkingPlanner);
  executor.remove_node(Lanelet2MapLoader);
  executor.remove_node(Lanelet2GlobalPlanner);
  executor.remove_node(NDTLocalizer);
  executor.remove_node(ObjectCollisionEstimator);
  executor.remove_node(BehaviorPlanner);
  executor.remove_node(MPCController);
  executor.remove_node(VehicleInterface);
  executor.remove_node(VehicleDBWSystem);
  
  rclcpp::shutdown();

  return 0;
}
