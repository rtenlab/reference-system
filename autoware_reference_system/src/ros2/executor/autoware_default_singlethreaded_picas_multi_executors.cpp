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

  rclcpp::executors::SingleThreadedExecutor executor1, executor2, executor3, executor4;
  
  executor1.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor1.callback_priority_enabled ? "Enabled" : "Disabled");
  executor1.set_executor_priority_cpu(90, 0);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 1's rt-priority %d and CPU %d", executor1.executor_priority, executor1.executor_cpu);

  executor2.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor2.callback_priority_enabled ? "Enabled" : "Disabled");
  executor2.set_executor_priority_cpu(89, 0);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 2's rt-priority %d and CPU %d", executor2.executor_priority, executor2.executor_cpu);

  executor3.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor3.callback_priority_enabled ? "Enabled" : "Disabled");
  executor3.set_executor_priority_cpu(88, 0);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 3's rt-priority %d and CPU %d", executor3.executor_priority, executor3.executor_cpu);

  executor4.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor4.callback_priority_enabled ? "Enabled" : "Disabled");
  executor4.set_executor_priority_cpu(87, 0);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 4's rt-priority %d and CPU %d", executor4.executor_priority, executor4.executor_cpu);

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
  executor2.add_node(FrontLidarDriver);
  executor2.add_node(RearLidarDriver);
  executor3.add_node(PointCloudMap);
  executor4.add_node(Visualizer);
  executor4.add_node(Lanelet2Map);
  executor2.add_node(PointsTransformerFront);
  executor2.add_node(PointsTransformerRear);
  executor3.add_node(PointCloudMapLoader);
  executor2.add_node(PointCloudFusion);
  executor3.add_node(VoxelGridDownsampler);
  executor2.add_node(RayGroundFilter);
  executor2.add_node(EuclideanClusterDetector);
  executor4.add_node(LanePlanner);
  executor4.add_node(ParkingPlanner);
  executor4.add_node(Lanelet2MapLoader);
  executor4.add_node(Lanelet2GlobalPlanner);
  executor3.add_node(NDTLocalizer);
  executor2.add_node(ObjectCollisionEstimator);
  executor2.add_node(BehaviorPlanner);
  executor1.add_node(MPCController);
  executor1.add_node(VehicleInterface);
  executor1.add_node(VehicleDBWSystem);

  // Assign priority
  executor2.set_callback_priority(FrontLidarDriver->get_callback(), 38);
  executor2.set_callback_priority(RearLidarDriver->get_callback(), 37);
  executor3.set_callback_priority(PointCloudMap->get_callback(), 31);
  executor4.set_callback_priority(Lanelet2Map->get_callback(), 22);
  executor4.set_callback_priority(Visualizer->get_callback(), 19);
  
  executor2.set_callback_priority(PointsTransformerRear->get_callback(), 39);
  executor2.set_callback_priority(PointsTransformerFront->get_callback(), 40);
  
  executor2.set_callback_priority(PointCloudFusion->get_subcallback_one(), 42);
  executor2.set_callback_priority(PointCloudFusion->get_subcallback_two(), 41);

  executor3.set_callback_priority(PointCloudMapLoader->get_callback(), 32);
  executor3.set_callback_priority(VoxelGridDownsampler->get_callback(), 33);
  executor2.set_callback_priority(RayGroundFilter->get_callback(), 43);

  executor3.set_callback_priority(NDTLocalizer->get_subcallback_one(), 35);
  executor3.set_callback_priority(NDTLocalizer->get_subcallback_two(), 34);

  executor2.set_callback_priority(EuclideanClusterDetector->get_callback(), 44);

  executor4.set_callback_priority(Lanelet2GlobalPlanner->get_subcallback_one(), 20);
  executor4.set_callback_priority(Lanelet2GlobalPlanner->get_subcallback_two(), 21);
  
  executor4.set_callback_priority(Lanelet2MapLoader->get_subcallback_one(), 24);
  executor4.set_callback_priority(Lanelet2MapLoader->get_subcallback_two(), 23);

  executor4.set_callback_priority(ParkingPlanner->get_callback(), 25);
  executor4.set_callback_priority(LanePlanner->get_callback(), 26);
  executor2.set_callback_priority(ObjectCollisionEstimator->get_callback(), 45);

  executor2.set_callback_priority(BehaviorPlanner->get_subcallback_one(), 46);
  executor2.set_callback_priority(BehaviorPlanner->get_subcallback_two(), 36);
  executor2.set_callback_priority(BehaviorPlanner->get_subcallback_three(), 27);
  executor2.set_callback_priority(BehaviorPlanner->get_subcallback_four(), 29);
  executor2.set_callback_priority(BehaviorPlanner->get_subcallback_five(), 28);
  executor2.set_callback_priority(BehaviorPlanner->get_subcallback_six(), 30);
  executor2.set_callback_priority(BehaviorPlanner->get_callback(), 47);

  executor1.set_callback_priority(MPCController->get_callback(), 48);
  
  executor1.set_callback_priority(VehicleInterface->get_subcallback_one(), 50);
  executor1.set_callback_priority(VehicleInterface->get_subcallback_two(), 49);

  executor1.set_callback_priority(VehicleDBWSystem->get_callback(), 51);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FrontLidarDriver->priority: %d", FrontLidarDriver->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RearLidarDriver->priority: %d", RearLidarDriver->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointCloudMap->priority: %d", PointCloudMap->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lanelet2Map->priority: %d", Lanelet2Map->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizer->priority: %d", Visualizer->get_callback()->callback_priority);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointsTransformerRear->priority: %d", PointsTransformerRear->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointsTransformerFront->priority: %d", PointsTransformerFront->get_callback()->callback_priority);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointCloudFusion_one->priority: %d", PointCloudFusion->get_subcallback_one()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointCloudFusion_two->priority: %d", PointCloudFusion->get_subcallback_two()->callback_priority);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointCloudMapLoader->priority: %d", PointCloudMapLoader->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "VoxelGridDownsampler->priority: %d", VoxelGridDownsampler->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RayGroundFilter->priority: %d", RayGroundFilter->get_callback()->callback_priority);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NDTLocalizer_one->priority: %d", NDTLocalizer->get_subcallback_one()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NDTLocalizer_two->priority: %d", NDTLocalizer->get_subcallback_two()->callback_priority);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EuclideanClusterDetector->priority: %d", EuclideanClusterDetector->get_callback()->callback_priority);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lanelet2GlobalPlanner_one->priority: %d", Lanelet2GlobalPlanner->get_subcallback_one()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lanelet2GlobalPlanner_two->priority: %d", Lanelet2GlobalPlanner->get_subcallback_two()->callback_priority);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lanelet2MapLoader_one->priority: %d", Lanelet2MapLoader->get_subcallback_one()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lanelet2MapLoader_two->priority: %d", Lanelet2MapLoader->get_subcallback_two()->callback_priority);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ParkingPlanner->priority: %d", ParkingPlanner->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LanePlanner->priority: %d", LanePlanner->get_callback()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ObjectCollisionEstimator->priority: %d", ObjectCollisionEstimator->get_callback()->callback_priority);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BehaviorPlanner_one->priority: %d", BehaviorPlanner->get_subcallback_one()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BehaviorPlanner_two->priority: %d", BehaviorPlanner->get_subcallback_two()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BehaviorPlanner_three->priority: %d", BehaviorPlanner->get_subcallback_three()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BehaviorPlanner_four->priority: %d", BehaviorPlanner->get_subcallback_four()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BehaviorPlanner_five->priority: %d", BehaviorPlanner->get_subcallback_five()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BehaviorPlanner_six->priority: %d", BehaviorPlanner->get_subcallback_six()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BehaviorPlanner->priority: %d", BehaviorPlanner->get_callback()->callback_priority);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MPCController->priority: %d", MPCController->get_callback()->callback_priority);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "VehicleInterface_one->priority: %d", VehicleInterface->get_subcallback_one()->callback_priority);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "VehicleInterface_two->priority: %d", VehicleInterface->get_subcallback_two()->callback_priority);
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "VehicleDBWSystem->priority: %d", VehicleDBWSystem->get_callback()->callback_priority);
  
  std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor1);
  std::thread spinThread2(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor2);
  std::thread spinThread3(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor3);
  std::thread spinThread4(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor4);


  spinThread1.join();
  
  executor2.remove_node(FrontLidarDriver);
  executor2.remove_node(RearLidarDriver);
  executor3.remove_node(PointCloudMap);
  executor4.remove_node(Visualizer);
  executor4.remove_node(Lanelet2Map);
  executor2.remove_node(PointsTransformerFront);
  executor2.remove_node(PointsTransformerRear);
  executor3.remove_node(PointCloudMapLoader);
  executor2.remove_node(PointCloudFusion);
  executor3.remove_node(VoxelGridDownsampler);
  executor2.remove_node(RayGroundFilter);
  executor2.remove_node(EuclideanClusterDetector);
  executor4.remove_node(LanePlanner);
  executor4.remove_node(ParkingPlanner);
  executor4.remove_node(Lanelet2MapLoader);
  executor4.remove_node(Lanelet2GlobalPlanner);
  executor3.remove_node(NDTLocalizer);
  executor2.remove_node(ObjectCollisionEstimator);
  executor2.remove_node(BehaviorPlanner);
  executor1.remove_node(MPCController);
  executor1.remove_node(VehicleInterface);
  executor1.remove_node(VehicleDBWSystem);

  
  rclcpp::shutdown();

  return 0;
}

