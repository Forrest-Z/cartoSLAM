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

#include "src/mapping/sparse_pose_graph_base.h"

#include "src/mapping/sparse_pose_graph/constraint_builder_options.h"
#include "src/mapping/sparse_pose_graph/optimization_problem_options.h"
#include "src/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::SparsePoseGraph::Constraint::Tag ToProto(
    const SparsePoseGraph::Constraint::Tag& tag) {
  switch (tag) {
    case SparsePoseGraph::Constraint::Tag::INTRA_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTRA_SUBMAP;
    case SparsePoseGraph::Constraint::Tag::INTER_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag.";
}

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SparsePoseGraphOptions options;
  options.set_optimize_every_n_scans(
      parameter_dictionary->GetInt("optimize_every_n_scans"));
  *options.mutable_constraint_builder_options() =
      sparse_pose_graph::CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  options.set_matcher_translation_weight(
      parameter_dictionary->GetDouble("matcher_translation_weight"));
  options.set_matcher_rotation_weight(
      parameter_dictionary->GetDouble("matcher_rotation_weight"));
  *options.mutable_optimization_problem_options() =
      sparse_pose_graph::CreateOptimizationProblemOptions(
          parameter_dictionary->GetDictionary("optimization_problem").get());
  options.set_max_num_final_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
  CHECK_GT(options.max_num_final_iterations(), 0);
  options.set_global_sampling_ratio(
      parameter_dictionary->GetDouble("global_sampling_ratio"));
  options.set_log_residual_histograms(
      parameter_dictionary->GetBool("log_residual_histograms"));
  return options;
}


}  // namespace mapping
}  // namespace cartographer
