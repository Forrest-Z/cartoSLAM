/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_

#include "src/sensor/point_cloud.h"
#include "src/sensor/odometry_data.h"
#include "src/mapping/local_trajectory_builder.h"
#include "src/mapping/sparse_pose_graph.h"
#include "src/common/make_unique.h"

namespace cartographer {
namespace mapping {

class GlobalTrajectoryBuilder{
 public:
  GlobalTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions &local_trajectory_options,
     const proto::SparsePoseGraphOptions &sparse_pose_graphe_options):
    thread_pool_(4)
   ,local_trajectory_builder_(local_trajectory_options)
   ,sparse_pose_graph_2d_(std::move(common::make_unique<SparsePoseGraph2D>(sparse_pose_graphe_options ,&thread_pool_))) 
   {};

  ~GlobalTrajectoryBuilder(){};

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  //const mapping::PoseEstimate& pose_estimate() const {
  //  return local_trajectory_builder_.pose_estimate();
  //}

  void AddRangefinderData(const double time, const Eigen::Vector3f &origin, const sensor::PointCloud &ranges)
  {
    auto insertion_result = local_trajectory_builder_.AddRangeData( time, sensor::RangeData{origin, ranges, {}});
    if (insertion_result == nullptr)
    {
      return;
    }
    sparse_pose_graph_2d_->AddScan(
        insertion_result->constant_data, insertion_result->pose_observation,
        0, insertion_result->insertion_submaps);
  }

  void AddSensorData(const sensor::OdometryData& odometry_data){
    local_trajectory_builder_.AddOdometerData(odometry_data);
    sparse_pose_graph_2d_->AddOdometerData(0, odometry_data);
  }

  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>> GetAllSubmapData()
  {
    return sparse_pose_graph_2d_->GetAllSubmapData();
  }

private:
  common::ThreadPool thread_pool_;
  LocalTrajectoryBuilder local_trajectory_builder_;
  std::unique_ptr<SparsePoseGraph2D> sparse_pose_graph_2d_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
