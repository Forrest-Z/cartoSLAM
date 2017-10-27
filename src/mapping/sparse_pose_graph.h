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

#ifndef CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_

#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "src/common/lua_parameter_dictionary.h"
#include "src/mapping/id.h"
//#include "src/mapping/pose_graph_trimmer.h"
//#include "src/mapping/proto/serialization.pb.h"
#include "src/mapping/proto/sparse_pose_graph.pb.h"
#include "src/mapping/proto/sparse_pose_graph_options.pb.h"
#include "src/mapping/submaps.h"
#include "src/mapping/trajectory_node.h"
#include "src/transform/rigid_transform.h"
#include "src/common/thread_pool.h"
#include "src/sensor/odometry_data.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "src/mapping/sparse_pose_graph/optimization_problem.h"

namespace cartographer {
namespace mapping {

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class SparsePoseGraph {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;
      double translation_weight;
      double rotation_weight;
    };

    SubmapId submap_id;  // 'i' in the paper.
    NodeId node_id;      // 'j' in the paper.

    // Pose of the scan 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where scan 'j' was inserted into
    // submap 'i') and inter-submap constraints (where scan 'j' was not inserted
    // into submap 'i').
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  struct SubmapData {
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };
  SparsePoseGraph(const mapping::proto::SparsePoseGraphOptions& options,
    common::ThreadPool* thread_pool);
  SparsePoseGraph() {}
  ~SparsePoseGraph() {}

  SparsePoseGraph(const SparsePoseGraph&) = delete;
  SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

  // Adds a new node with 'constant_data' and a 'pose' that will later be
  // optimized. The 'pose' was determined by scan matching against
  // 'insertion_submaps.front()' and the scan was inserted into the
  // 'insertion_submaps'. If 'insertion_submaps.front().finished()' is
  // 'true', this submap was inserted into for the last time.
  void AddScan(
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data,
    const transform::Rigid3d& pose, int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
    EXCLUDES(mutex_);

    void AddOdometerData(int trajectory_id,
      const sensor::OdometryData& odometry_data);

  // Freezes a trajectory. Poses in this trajectory will not be optimized.
  void FreezeTrajectory(int trajectory_id);

  // Adds a 'submap' from a proto with the given 'initial_pose' to the frozen
  // trajectory with 'trajectory_id'.
  //virtual void AddSubmapFromProto(int trajectory_id,
  //                                const transform::Rigid3d& initial_pose,
  //                                const proto::Submap& submap) = 0;

  // Adds a 'trimmer'. It will be used after all data added before it has been
  // included in the pose graph.
  //virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

  // Computes optimized poses.
  //void AddSubmapFromProto(int trajectory_id,
  //                        const transform::Rigid3d& initial_pose,
  //                        const mapping::proto::Submap& submap);
  //void AddTrimmer(std::unique_ptr<mapping::PoseGraphTrimmer> trimmer);
  void RunFinalOptimization();
  std::vector<std::vector<int>> GetConnectedTrajectories();
  int num_submaps(int trajectory_id) EXCLUDES(mutex_);
  mapping::SparsePoseGraph::SubmapData GetSubmapData(const mapping::SubmapId& submap_id) EXCLUDES(mutex_);
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
  GetAllSubmapData() EXCLUDES(mutex_);
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) EXCLUDES(mutex_);
  std::vector<std::vector<mapping::TrajectoryNode>> GetTrajectoryNodes() EXCLUDES(mutex_);
  std::vector<Constraint> constraints() EXCLUDES(mutex_);


  // Serializes the constraints and trajectories.
  proto::SparsePoseGraph ToProto();

private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all scans are tried to match against this submap.
  // Likewise, all new scans are matched against submaps which are finished.
  enum class SubmapState { kActive, kFinished, kTrimmed };
  struct SubmapStateData {
    std::shared_ptr<const Submap> submap;
    // IDs of the scans that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    std::set<mapping::NodeId> node_ids;
    SubmapState state = SubmapState::kActive;
  };

  void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);
  
    // Grows the optimization problem to have an entry for every element of
    // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
    std::vector<mapping::SubmapId> GrowSubmapTransformsAsNeeded(
        int trajectory_id,
        const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
        REQUIRES(mutex_);

  // Adds constraints for a scan, and starts scan matching in the background.
  void ComputeConstraintsForScan(
      int trajectory_id,
      std::vector<std::shared_ptr<const Submap>> insertion_submaps,
      bool newly_finished_submap, const transform::Rigid2d& pose)
      REQUIRES(mutex_);
  // Computes constraints for a scan and submap pair.
  void ComputeConstraint(const mapping::NodeId& node_id,
    const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  // Adds constraints for older scans whenever a new submap is finished.
  void ComputeConstraintsForOldScans(const mapping::SubmapId& submap_id)
  REQUIRES(mutex_);

  // Registers the callback to run the optimization once all constraints have
  // been computed, that will also do all work that queue up in 'work_queue_'.
  void HandleWorkQueue() REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() EXCLUDES(mutex_);
  
  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() EXCLUDES(mutex_);

  // Computes the local to global frame transform based on the given optimized
  // 'submap_transforms'.
  //transform::Rigid3d ComputeLocalToGlobalTransform(
  //    const std::vector<std::map<int, sparse_pose_graph::SubmapData>>&
  //        submap_transforms,
  //    int trajectory_id) const REQUIRES(mutex_);

  mapping::SparsePoseGraph::SubmapData GetSubmapDataUnderLock(
      const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  const mapping::proto::SparsePoseGraphOptions options_;
  common::Mutex mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  std::unique_ptr<std::deque<std::function<void()>>> work_queue_
  GUARDED_BY(mutex_);

    // How our various trajectories are related.
    //mapping::ConnectedComponents connected_components_;

  // We globally localize a fraction of the scans from each trajectory.
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
  global_localization_samplers_ GUARDED_BY(mutex_);
  std::vector<Constraint> constraints_ GUARDED_BY(mutex_);
  
    // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  mapping::NestedVectorsById<SubmapStateData, mapping::SubmapId> submap_data_
  GUARDED_BY(mutex_);

// Data that are currently being shown.
mapping::NestedVectorsById<mapping::TrajectoryNode, mapping::NodeId>
  trajectory_nodes_ GUARDED_BY(mutex_);
int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

  // Current submap transforms used for displaying data.
  std::vector<std::map<int, sparse_pose_graph::SubmapPoseData>>
  optimized_submap_transforms_ GUARDED_BY(mutex_);

// List of all trimmers to consult when optimizations finish.
//std::vector<std::unique_ptr<mapping::PoseGraphTrimmer>> trimmers_
//  GUARDED_BY(mutex_);
std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

//class TrimmingHandle : public mapping::Trimmable {
//public:
// TrimmingHandle(SparsePoseGraph* parent);
// ~TrimmingHandle() override {}
//
// int num_submaps(int trajectory_id) const override;
// void MarkSubmapAsTrimmed(const mapping::SubmapId& submap_id) override;
//
//private:
// SparsePoseGraph* const parent_;
//};

};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
