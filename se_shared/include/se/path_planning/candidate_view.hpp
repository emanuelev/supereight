/**
 * Information-theoretic exploration
 *
 * Copyright (C) 2019 Imperial College London.
 * Copyright (C) 2019 ETH Zürich.
 *
 * @file candidate_view.hpp
 * @author Anna Dai
 * @date August 22, 2019
 */

#ifndef SUPEREIGHT_CANDIDATE_VIEW_HPP
#define SUPEREIGHT_CANDIDATE_VIEW_HPP

#include <set>
#include <map>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <iterator>
#include <type_traits>
#include <cmath>
#include <glog/logging.h>
#include <Eigen/StdVector>

#include "se/geometry/octree_collision.hpp"
#include "se/continuous/volume_template.hpp"
#include "se/octree.hpp"
#include "se/node_iterator.hpp"
#include "se/constant_parameters.h"
#include "se/ray_iterator.hpp"
#include "se/utils/math_utils.h"
#include "se/config.h"
#include "se/utils/eigen_utils.h"
#include "exploration_utils.hpp"

#include "se/path_planner_ompl.hpp"

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
//typedef SE_FIELD_TYPE FieldType;
namespace se {

namespace exploration {

/**
 * Candidate View
 * all in voxel coord
 */



template<typename T>
class CandidateView {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CandidateView> SPtr;
  CandidateView(const Volume<T> &volume,
                const Planning_Configuration &planning_config,
                const std::shared_ptr<CollisionCheckerV<T> > pcc,
                const float res,
                const Configuration &config,
                const Eigen::Matrix4f &curr_pose,
                const float step,
                const float ground_height);

  void getCandidateViews(const set3i &frontier_blocks_map, const int frontier_cluster_size);

  void printFaceVoxels(const Eigen::Vector3i &voxel);

  std::pair<float, float> getRayInformationGain(const Eigen::Vector3f &direction,
                                                const float tnear,
                                                const float tfar,
                                                const Eigen::Vector3f &origin);
  float getViewInformationGain(pose3D &pose);

  void calculateCandidateViewGain();

  void calculateUtility(Candidate &candidate);
  int getBestCandidate();

  float getIGWeight_tanh(const float tanh_range,
                         const float tanh_ratio,
                         const float t,
                         const float prob_log,
                         float &t_hit,
                         bool &hit_unknown) const;

  VecPose getFinalPath(Candidate &candidate);
  VecPose getYawPath(const pose3D &start, const pose3D &goal);
  VecPose fusePath(VecPose &path_tmp, VecPose &yaw_path);
  VecPose addPathSegments(const pose3D &start, const pose3D &goal);

  int getExplorationStatus() const { return exploration_status_; }

  float getTargetIG() const { return ig_target_; }
  int getNumValidCandidates() const { return num_cands_; }

  VecCandidate candidates_;
  Candidate curr_pose_;
 private:
  pose3D pose_;
  VecVec3i cand_views_;
  Volume<T> volume_;

  float res_; // [m/vx]
  Planning_Configuration planning_config_;
  Configuration config_;
  float occ_thresh_ = 0.f;
  float ig_total_;
  float ig_target_;
  int dtheta_;
  int dphi_;
  float step_;
  float ground_height_;
  int exploration_status_;

  int num_sampling_;
  int num_cands_;

  std::shared_ptr<CollisionCheckerV<T> > pcc_ = nullptr;
};

template<typename T>
CandidateView<T>::CandidateView(const Volume<T> &volume,
                                const Planning_Configuration &planning_config,
                                const std::shared_ptr<CollisionCheckerV<T> > pcc,
                                const float res,
                                const Configuration &config,
                                const Eigen::Matrix4f &curr_pose,
                                const float step,
                                const float ground_height)
    :
    volume_(volume),
    planning_config_(planning_config),
    res_(res),
    config_(config),
    pcc_(pcc),
    dtheta_(planning_config.dtheta),
    dphi_(planning_config.dphi),
    step_(step),
    exploration_status_(0),
    num_sampling_(planning_config.num_cand_views),
    num_cands_(0),
    ground_height_(ground_height) {

  curr_pose_.pose = getCurrPose(curr_pose, res_);
  curr_pose_.path.push_back(curr_pose_.pose);
  curr_pose_.path_length = 0.f;
  pose_ = curr_pose_.pose;
  int n_col = planning_config.fov_hor * 0.75 / planning_config.dphi;
  int n_row = planning_config.fov_hor / planning_config.dtheta;
  ig_total_ = n_col * n_row * (farPlane / step) * getEntropy(0);
  ig_target_ = n_col * n_row * (farPlane / step) * getEntropy(log2(0.1 / (1.f - 0.1)));
  // std::cout << "ig total " << ig_total_ << " ig target " << ig_target_ << std::endl;
  candidates_.resize(num_sampling_);
  LOG(INFO) << "setup candidate view";
}

/**
 * calculates exploration path
 * for developing and visualization purposes also return candidate views
 * @tparam T Ofusion or SDF
 * @param shared_ptr octree_ptr : increments reference count and makes the function an owner, the
 * octree will stay alive as long as the function is sing it
 * @param frontier_map
 * @param res [m/voxel]
 * @param step interval to stride along a ray
 * @param planning_config
 * @param config supereight configuration
 * @param pose current camera pose
 * @param [out] path
 * @param [out] cand_views
 */
template<typename T>
int getExplorationPath(std::shared_ptr<Octree<T> > octree_ptr,
                       const Volume<T> &volume,
                       const set3i &frontier_map,
                       const float res,
                       const float step,
                       const Planning_Configuration &planning_config,
                       const Configuration &config,
                       const Eigen::Matrix4f &pose,
                       const Eigen::Vector3i &lower_bound,
                       const Eigen::Vector3i &upper_bound,
                       const float ground_height,
                       VecPose &path,
                       VecPose &cand_views
                       ) {

  pose3D start = getCurrPose(pose, res);
  bool valid_path = false;
  auto collision_checker_v = aligned_shared<CollisionCheckerV<T> >(octree_ptr, planning_config);
  // auto path_planner_ompl_ptr =
  // aligned_shared<PathPlannerOmpl<T> >(octree_ptr, collision_checker_v, planning_config);
  LOG(INFO) << "frontier map size " << frontier_map.size();
  // Candidate view generation
  CandidateView<T> candidate_view
      (volume, planning_config, collision_checker_v, res, config, pose, step, ground_height);
  int frontier_cluster_size = planning_config.frontier_cluster_size;
  while (candidate_view.getNumValidCandidates() < 3) {
    DLOG(INFO) << "get candidates";
    candidate_view.getCandidateViews(frontier_map, frontier_cluster_size);
    if (frontier_cluster_size >= 8) {
      frontier_cluster_size /= 2;
    } else{
      path.push_back(start);
      return 1;
    }
  }



  // if (size > 1) {
#pragma omp parallel for
  for (int i = 0; i < planning_config.num_cand_views; i++) {

    if (candidate_view.candidates_[i].pose.p != Eigen::Vector3f(0, 0, 0)) {
      auto collision_checker = aligned_shared<CollisionCheckerV<T> >(octree_ptr, planning_config);
      auto path_planner_ompl_ptr = aligned_shared<PathPlannerOmpl<T> >(octree_ptr,
                                                                       collision_checker,
                                                                       planning_config,
                                                                       ground_height);
      // LOG(INFO) << "Candidate " << i << " goal coord " << cand_views[i].p.format(InLine);
      DLOG(INFO) << "Candidate " << i << " start " << start.p.format(InLine) << " goal "
                 << candidate_view.candidates_[i].pose.p.format(InLine);

      bool setup_planner = path_planner_ompl_ptr->setupPlanner(lower_bound, upper_bound);
      DLOG(INFO) << "setup planner successful " << setup_planner;
      int path_planned = path_planner_ompl_ptr->planPath(start.p.cast<int>(),
                                                         candidate_view.candidates_[i].pose.p.template cast<
                                                             int>());
      candidate_view.candidates_[i].planning_solution_status = path_planned;
      DLOG(INFO) << "path planned " << path_planned;
      if (path_planned < 0) {
        // voxel coord
        candidate_view.candidates_[i].path_length =
            (start.p - candidate_view.candidates_[i].pose.p).squaredNorm();
        VecPose vec;
        vec.push_back(candidate_view.candidates_[i].pose);
        candidate_view.candidates_[i].path = vec;
      } else {
        // path_length[i] = path_planner_ompl_ptr->getPathLength();
        valid_path = true;
        candidate_view.candidates_[i].path_length = path_planner_ompl_ptr->getPathLength();

        VecPose vec = path_planner_ompl_ptr->getPathSegments_m(); //[voxel coord]

        // all_path[i] = vec;
        candidate_view.candidates_[i].path = vec;
        if (path_planned == 2) {
          DLOG(INFO) << "cand changed from " << candidate_view.candidates_[i].pose.p.format(InLine)
                     << " to "
                     << candidate_view.candidates_[i].path[candidate_view.candidates_[i].path.size()
                         - 1].p.format(InLine);
          candidate_view.candidates_[i].pose.p =
              candidate_view.candidates_[i].path[candidate_view.candidates_[i].path.size() - 1].p;
        }
        // DLOG(INFO) << "seg length " << all_path[i].size() << std::endl;
      }
    }
  }

  candidate_view.calculateCandidateViewGain();

  int best_cand_idx = -1;
  bool use_curr_pose = true;
  bool force_travelling = candidate_view.curr_pose_.information_gain < candidate_view.getTargetIG();
  if (valid_path) {
    best_cand_idx = candidate_view.getBestCandidate();
    LOG(INFO) << "[se/candview] best candidate is "
              << (candidate_view.candidates_[best_cand_idx].pose.p * res).format(InLine);
    // std::cout << " path length of best cand "
    //           << candidate_view.candidates_[best_cand_idx].path.size() << std::endl;
    use_curr_pose =
        candidate_view.candidates_[best_cand_idx].utility < candidate_view.curr_pose_.utility;
    LOG(INFO) << "force travelling " << force_travelling << " use curr pose " << use_curr_pose
              << " candidate utility " << candidate_view.candidates_[best_cand_idx].utility
              << " curr pose utility " << candidate_view.curr_pose_.utility;

  }
  // }
  // TODO make this nicer


  VecPose path_tmp;
  if (valid_path && (!use_curr_pose || force_travelling)) {
    // candidate_view.addPathSegments(planning_config.robot_safety_radius*2.5 ,best_cand_idx);
    // path_tmp = candidate_view.candidates_[best_cand_idx].path;
    path_tmp = candidate_view.getFinalPath(candidate_view.candidates_[best_cand_idx]);
    // best_candidate = candidate_view.candidates_[best_cand_idx];
  } else {
    path_tmp = candidate_view.getFinalPath(candidate_view.curr_pose_);
    // best_candidate = candidate_view.curr_pose_;
  }
  for (int i = 0; i <= planning_config.num_cand_views; i++) {
    if (candidate_view.candidates_[i].pose.p == Eigen::Vector3f(0, 0, 0)) {
      continue;
    }
    cand_views.push_back(candidate_view.candidates_[i].pose);
  }

  for (const auto &pose : path_tmp) {
    DLOG(INFO) << "cand view " << (pose.p * res).format(InLine) << " " << pose.q.w() << " "
               << pose.q.vec().format(InLine);
    path.push_back(pose);
  }
  if (candidate_view.getExplorationStatus() == 1) {
    return 1;
  } else {
    return -1;
  }

}

} // namespace exploration
} // namespace se

#include "candidate_view_impl.hpp"
#endif //SUPEREIGHT_CANDIDATE_VIEW_HPP
