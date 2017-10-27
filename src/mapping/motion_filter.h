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

#ifndef CARTOGRAPHER_MAPPING_3D_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_3D_MOTION_FILTER_H_

#include <limits>

#include "src/common/lua_parameter_dictionary.h"
#include "src/mapping/proto/motion_filter_options.pb.h"
#include "src/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  explicit MotionFilter(const proto::MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(double time, const transform::Rigid3d& pose);

 private:
  const proto::MotionFilterOptions options_;
  int num_total_ = 0;
  int num_different_ = 0;
  double last_time_;
  transform::Rigid3d last_pose_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_MOTION_FILTER_H_
