/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
#define CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_

#include <memory>

#include "../sensor/odometry_data.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "../mapping/global_trajectory_builder.h"
//#include "../mapping/local_trajectory_builder_options.h"
//#include "../mapping/local_trajectory_builder.h"
//#include "../mapping/sparse_pose_graph.h"

namespace cartographer_ros
{

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge
{
public:
  explicit SensorBridge( std::shared_ptr<::cartographer::mapping::GlobalTrajectoryBuilder> global_trajectory_builder_ptr)
  {
    global_trajectory_builder_ptr_ = std::move(global_trajectory_builder_ptr);
  };

  SensorBridge(const SensorBridge &) = delete;
  SensorBridge &operator=(const SensorBridge &) = delete;

  std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(const nav_msgs::Odometry::ConstPtr &msg);

  void HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr &msg);
  void HandleOdometryMessage(const nav_msgs::Odometry::ConstPtr &msg);

  std::shared_ptr<::cartographer::mapping::GlobalTrajectoryBuilder> global_trajectory_builder_ptr_;
};

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
