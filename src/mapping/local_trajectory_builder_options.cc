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

#include "../mapping/local_trajectory_builder_options.h"

#include "../mapping/scan_matching/ceres_scan_matcher.h"
#include "../mapping/scan_matching/real_time_correlative_scan_matcher.h"
#include "../mapping/submaps.h"
#include "../sensor/voxel_filter.h"

#include "src/sensor/proto/adaptive_voxel_filter_options.pb.h"

namespace cartographer {
namespace mapping {

proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions() {
  proto::LocalTrajectoryBuilderOptions options;
  options.set_min_range(0.2);
  options.set_max_range(3.5);
  options.set_min_z(-1.);
  options.set_max_z(1.);
  options.set_missing_data_ray_length(5.);
  options.set_scans_per_accumulation(1);
  options.set_voxel_filter_size(0.025);
  options.set_use_online_correlative_scan_matching(true);

  sensor::proto::AdaptiveVoxelFilterOptions avfo;
  avfo.set_max_length(0.5);
  avfo.set_min_num_points(100);
  avfo.set_max_range(50.);
  *options.mutable_adaptive_voxel_filter_options() = avfo;

  sensor::proto::AdaptiveVoxelFilterOptions lcafo;
  avfo.set_max_length(0.9);
  avfo.set_min_num_points(100);
  avfo.set_max_range(50.);
  *options.mutable_loop_closure_adaptive_voxel_filter_options() = lcafo;
  //::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions*>(&::cartographer::mapping::scan_matching::proto::Re
  scan_matching::proto::RealTimeCorrelativeScanMatcherOptions rcso;
  rcso.set_linear_search_window(0.1);
  rcso.set_angular_search_window(0.34);
  rcso.set_translation_delta_cost_weight(0.1);
  rcso.set_rotation_delta_cost_weight(0.1);
  *options.mutable_real_time_correlative_scan_matcher_options() = rcso;

  scan_matching::proto::CeresScanMatcherOptions csmo;
  csmo.set_occupied_space_weight(1.);
  csmo.set_translation_weight(10.);
  csmo.set_rotation_weight(40.);
  common::proto::CeresSolverOptions cso;
  cso.set_use_nonmonotonic_steps(false);
  cso.set_max_num_iterations(20);
  cso.set_num_threads(1);
  *csmo.mutable_ceres_solver_options() = cso;
  *options.mutable_ceres_scan_matcher_options() = csmo;



  return options;
}

}  // namespace mapping
}  // namespace cartographer
