//
// Created by anna on 27/06/19.
//

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
#include "collision_checker.hpp"
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
  typedef std::shared_ptr<CandidateView> Ptr;
  CandidateView(const Volume<T> &volume,
                const Planning_Configuration &planning_config,
                const std::shared_ptr<CollisionCheckerV<T> > pcc,
                const float res,
                const Configuration &config,
                const Eigen::Matrix4f &curr_pose,
                const float step);

  Eigen::Vector3i getOffsetCandidate(const Eigen::Vector3i &cand_v,
                                     const VecVec3i &frontier_voxels);
  void getCandidateViews(const map3i &frontier_blocks_map);

  void printFaceVoxels(const Eigen::Vector3i &voxel);

  std::pair<float, float> getInformationGain(const Eigen::Vector3f &direction,
                                             const float tnear,
                                             const float tfar,
                                             const Eigen::Vector3f &origin);

  void calculateCandidateViewGain();

  int getBestCandidate();

  float getIGWeight_tanh(const float tanh_range,
                         const float tanh_ratio,
                         const float t,
                         const float prob_log,
                         float &t_hit,
                         bool &hit_unknown) const;

  VecPose getFinalPath(const float max_yaw_rate,
                              const int idx);
  VecPose getYawPath(const pose3D& start, const pose3D &goal, const float max_yaw_rate);

  int getExplorationStatus() const { return exploration_status_; }

  void setSolutionStatus(const int solution_status, const int idx)  { candidates_[idx].planning_solution_status = solution_status;}
  int getSolutionStatus(const int idx) const {return candidates_[idx].planning_solution_status; }

  pose3D getPose(const int idx)  { return candidates_[idx].pose; }

  void setPathLength(const int idx, const float path_length)  { candidates_[idx].path_length = path_length;}
  float getPathLength(const int idx) const {return candidates_[idx].path_length ; }

  void setPath(const int idx, const VecPose & path)  { candidates_[idx].path = path;}
  VecPose getPath(const int idx ) const {return candidates_[idx].path; }

 private:
  pose3D curr_pose_;
  VecVec3i cand_views_;
  Volume<T> volume_;

  VecCandidate candidates_;

  float res_; // [m/vx]
  Planning_Configuration planning_config_;
  Configuration config_;
  float occ_thresh_ = 0.f;
  float ig_total_;
  float ig_target_;
  int dtheta_;
  int dphi_;
  float step_;
  int exploration_status_;

  int num_views_;

  std::shared_ptr<CollisionCheckerV<T> > pcc_ = nullptr;
};

template<typename T>
CandidateView<T>::CandidateView(const Volume<T> &volume,
                                const Planning_Configuration &planning_config,
                                const std::shared_ptr<CollisionCheckerV<T> > pcc,
                                const float res,
                                const Configuration &config,
                                const Eigen::Matrix4f &curr_pose,
                                const float step)
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
    num_views_(planning_config.num_cand_views) {

  curr_pose_ = getCurrPose(curr_pose, res_);
  int n_col = planning_config.fov_hor * 0.75 / planning_config.dphi;
  int n_row = planning_config.fov_hor  / planning_config.dtheta;
  ig_total_ = n_col * n_row * (farPlane / step) * getEntropy(0);
  ig_target_ = n_col * n_row * (farPlane / step) * getEntropy(log2(0.02 / (1.f - 0.02)));
  std::cout << "ig total " << ig_total_ << " ig target " << ig_target_ << std::endl;
  candidates_.resize(num_views_+1);
  candidates_[num_views_].pose = curr_pose_;
}
/**
 * helper function to print the face voxels
 * TODO sparsify
 * @param _voxel center voxel cooridnates
 */
template<typename T>
void CandidateView<T>::printFaceVoxels(const Eigen::Vector3i &_voxel) {
  VecVec3i face_neighbour_voxel(6);
  face_neighbour_voxel[0] << _voxel.x() - 1, _voxel.y(), _voxel.z();
  face_neighbour_voxel[1] << _voxel.x() + 1, _voxel.y(), _voxel.z();
  face_neighbour_voxel[2] << _voxel.x(), _voxel.y() - 1, _voxel.z();
  face_neighbour_voxel[3] << _voxel.x(), _voxel.y() + 1, _voxel.z();
  face_neighbour_voxel[4] << _voxel.x(), _voxel.y(), _voxel.z() - 1;
  face_neighbour_voxel[5] << _voxel.x(), _voxel.y(), _voxel.z() + 1;
  std::cout << "[se/cand view] face voxel states ";
  for (const auto &face_voxel : face_neighbour_voxel) {
    std::cout << volume_._map_get->get(face_voxel).st << " ";
  }
  std::cout << std::endl;

}
/**
 * offsets the candidate view by a pre-determined distance from the frontier surface and checks
 * if the sphere is collision free
 * @param cand_v [vx coord] position
 * @param frontier_voxels  vector with frontier voxels
 * @return candidate views in voxel coord global
 */
template<typename T>
Eigen::Vector3i CandidateView<T>::getOffsetCandidate(const Eigen::Vector3i &cand_v,
                                                     const VecVec3i &frontier_voxels) {
  Eigen::Vector3i offset_cand_v(0, 0, 0);
  bool is_valid = false;
  CollisionCheck<T> collision_check(volume_, planning_config_, res_);
  // curr res 24m/128 vox = 0.1875 m/vx

  int offset_v = static_cast<int>(planning_config_.robot_safety_radius / res_);
  // fit plane through frontier voxels in the block
// source https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
// https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
  const float num_points = frontier_voxels.size();
  if (num_points < 3) return Eigen::Vector3i(0, 0, 0);
  Eigen::Vector3i sum(0, 0, 0);
  for (auto p : frontier_voxels) {
    sum += p;
  }
  Eigen::Vector3f centroid = sum.cast<float>() * (1.0f / num_points);
  // calculate full 3x3 covariance matrix, excluding symmetries
  // relative around the centroid
  float xx = 0.0f, xy = 0.0f, xz = 0.0f, yy = 0.0f, yz = 0.0f, zz = 0.0f;
  for (auto p : frontier_voxels) {
    Eigen::Vector3f dist = p.cast<float>() - centroid;
    xx += dist.x() * dist.x();
    xy += dist.x() * dist.y();
    xz += dist.x() * dist.z();
    yy += dist.y() * dist.y();
    yz += dist.y() * dist.z();
    zz += dist.z() * dist.z();
  }
  xx /= num_points;
  xy /= num_points;
  xz /= num_points;
  yy /= num_points;
  yz /= num_points;
  zz /= num_points;

  float det_x = yy * zz - yz * yz;
  float det_y = xx * zz - xz * xz;
  float det_z = xx * yy - xy * xy;

  float max_det = std::max({det_x, det_y, det_z});

  if (max_det <= 0.0f) {
    return Eigen::Vector3i(0, 0, 0);
  }
  // find normal
  Eigen::Vector3f normal;
  if (max_det == det_x) {
    normal.x() = det_x;
    normal.y() = xz * yz - xy * zz;
    normal.z() = xy * yz - xz * yy;
  } else if (max_det == det_y) {

    normal.x() = xz * yz - xy * zz;
    normal.y() = det_y;
    normal.z() = xy * yz - xz * xx;
  } else if (max_det == det_z) {

    normal.x() = xz * yz - xy * yy;
    normal.y() = xy * xz - yz * xx;
    normal.z() = det_z;
  }
  normal = normal.normalized();

  // offset candidate
  offset_cand_v.x() = ceil(normal.x() * offset_v);
  offset_cand_v.y() = ceil(normal.y() * offset_v);
  offset_cand_v.z() = ceil(normal.z() * offset_v);
  offset_cand_v = cand_v + offset_cand_v.cast<int>();

//  printFaceVoxels(volume_, offset_cand_v);

  if (volume_._map_index->get(offset_cand_v).x > 0.0f) {

    offset_cand_v.x() = ceil(-normal.x() * offset_v);
    offset_cand_v.y() = ceil(-normal.y() * offset_v);
    offset_cand_v.z() = ceil(-normal.z() * offset_v);
    offset_cand_v = cand_v + offset_cand_v.cast<int>();
//    printFaceVoxels(volume_, offset_cand_v);
    if (volume_._map_index->get(offset_cand_v).x <= 0.0f) {
      is_valid = true;
    }
  } else {
    is_valid = true;
  }

  if (is_valid) {
    bool is_free = pcc_->isSphereSkeletonFree(offset_cand_v, static_cast<int>(
        planning_config_.robot_safety_radius / res_));
    if (is_free == 1) {
      return offset_cand_v;
    } else {
      return Eigen::Vector3i(0, 0, 0);
    }
  } else {
// not a valid view or sphere is not collision free
    return Eigen::Vector3i(0, 0, 0);
  }

}

/**
 *
 * generates random valid candidate views from the frontier map
 * @tparam T
 * @param frontier_blocks_map
 */
template<typename T>
void CandidateView<T>::getCandidateViews(const map3i &frontier_blocks_map) {

  mapvec3i frontier_voxels_map;
  node_iterator<T> node_it(*(volume_._map_index));

  // get all frontier voxels inside a voxel block

  for (const auto &frontier_block : frontier_blocks_map) {
    VecVec3i frontier_voxels = node_it.getFrontierVoxels(frontier_block.first);
    frontier_voxels_map[frontier_block.first] = frontier_voxels;
  }

  if (frontier_voxels_map.size() != frontier_blocks_map.size()) {
    return;
  }
  // random candidate view generator
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_int_distribution<int> distribution_block(0, frontier_blocks_map.size() - 1);

  for (int i = 0; i <= num_views_; i++) {
    auto it = frontier_voxels_map.begin();

    const int rand_num = distribution_block(generator);
    std::advance(it, rand_num);
    uint64_t rand_morton = it->first;
    if (frontier_voxels_map[rand_morton].size() < planning_config_.frontier_cluster_size) {
      continue;
    }
    std::uniform_int_distribution<int>
        distribution_voxel(0, frontier_voxels_map[rand_morton].size() - 1);
    // random frontier voxel inside the the randomly chosen voxel block
    int rand_voxel = distribution_voxel(generator);
    VecVec3i frontier_voxelblock = frontier_voxels_map[rand_morton];
    Eigen::Vector3i candidate_frontier_voxel = frontier_voxels_map[rand_morton].at(rand_voxel);
    if (boundHeight(&candidate_frontier_voxel.z(),
                    planning_config_.height_max,
                    planning_config_.height_min,
                    res_)) {

      frontier_voxelblock = frontier_voxels_map[se::keyops::encode(candidate_frontier_voxel.x(),
                                                                   candidate_frontier_voxel.y(),
                                                                   candidate_frontier_voxel.z(),
                                                                   volume_._map_index->leaf_level(),
                                                                   volume_._map_index->max_level())];

    }

    // offset the candidate frontier voxel
    Eigen::Vector3i cand_view_v = getOffsetCandidate(candidate_frontier_voxel, frontier_voxelblock);

    if (cand_view_v != Eigen::Vector3i(0, 0, 0)) {

      // cand_views_.push_back(cand_view_v);
      candidates_[i].pose.p = cand_view_v.cast<float>();
    }
  }
}

/**
 * march along the ray until max sensor depth or a surface is hit to calculate the entropy reduction
 * TODO add weight for unknown voxels
 * @param volume
 * @param origin cand view [m]
 * @param direction [m]
 * @param tnear
 * @param tfar max sensor depth
 * @param step
 * @return
 */
template<typename T>
std::pair<float, float> CandidateView<T>::getInformationGain(const Eigen::Vector3f &direction,
                                                             const float tnear,
                                                             const float tfar,
                                                             const Eigen::Vector3f &origin) {
  auto select_occupancy = [](typename Volume<T>::value_type val) { return val.x; };
  // march from camera away
  float ig_entropy = 0.0f;
  float t = tnear; // closer bound to camera
  if (tnear < tfar) {

    // occupancy prob in log2
    float prob_log = volume_.interp(origin + direction * t, select_occupancy);
    float weight = 1.0f;
    // check if current pos is free
    if (prob_log <= SURF_BOUNDARY + occ_thresh_) {
      ig_entropy = weight * getEntropy(prob_log);
      for (; t < tfar; t += step_) {
        const Eigen::Vector3f pos = origin + direction * t;
        typename Volume<T>::value_type data = volume_.get(pos);
        prob_log = volume_.interp(origin + direction * t, select_occupancy);
        ig_entropy += getEntropy(prob_log);

// next step along the ray hits a surface with a secure threshold return
        if (prob_log > SURF_BOUNDARY + occ_thresh_) {
          break;
        }
      }
    }
  } else {
    // TODO 0.4 is set fix in ray_iterator
    float num_it = (tfar - nearPlane) / step_;
    ig_entropy = num_it * 1.f;
    t = tfar;
  }

  return std::make_pair(ig_entropy, t);
}

template<typename T>
float CandidateView<T>::getIGWeight_tanh(const float tanh_range,
                                         const float tanh_ratio,
                                         const float t,
                                         const float prob_log,
                                         float &t_hit,
                                         bool &hit_unknown) const {
  float weight;
  if (prob_log == 0.f) {
    if (!hit_unknown) {
      t_hit = t;
      hit_unknown = true;
    }
    weight = tanh(tanh_range - (t - t_hit) * tanh_ratio);
  } else {
    weight = 1.f;
  }
  return weight;
}

// TODO parametrize variables

// information gain calculation
// source [1] aeplanner gainCubature
// not implemented [2]history aware aoutonomous exploration in confined environments using MAVs
// (cylindric)

template<typename T>
void CandidateView<T>::calculateCandidateViewGain() {
  // VecPairPoseFloat cand_pose_w_gain;

  // add curr pose to be evaluated
//  std::cout << "[se/candview] curr pose " << curr_pose.p.format(InLine) << " yaw "
//            << toEulerAngles(curr_pose.q).yaw * 180 / M_PI << std::endl;
//  std::cout << "[se/candview] cand view size " << cand_views_.size() << std::endl;
  // cand_views_.push_back(curr_pose_.p.cast<int>());

  // TODO parameterize the variables
  float gain = 0.0;

  const float r_max = farPlane; // equal to far plane
  // const float r_min = 0.01; // depth camera r min [m]  gazebo model
  const float fov_hor = planning_config_.fov_hor;
//  float fov_hor = static_cast<float>(planning_config_.fov_hor * 180.f / M_PI); // 2.0 = 114.59 deg
  const float fov_vert = fov_hor * 480.f / 640.f; // image size

  // temporary
  const int n_col = fov_vert / dphi_;
  const int n_row = 360 / dtheta_;

  float phi_rad, theta_rad;
  // int cand_num = 0;

  // debug matrices
  Eigen::MatrixXf gain_matrix(n_row, n_col + 2);
  Eigen::MatrixXf depth_matrix(n_row, n_col + 1);
  std::map<int, float> gain_per_yaw;
  gain_per_yaw.empty();

  // int cand_counter = 0;
// cand view in voxel coord
  for (int i = 0; i <= num_views_; i++){

    if(candidates_[i].pose.p == Eigen::Vector3f(0, 0, 0)){
      // cand_counter++;
      continue;
    }

    Eigen::Vector3f vec(0.0, 0.0, 0.0); //[m]
    Eigen::Vector3f dir(0.0, 0.0, 0.0);// [m]
    Eigen::Vector3f cand_view_m = candidates_[i].pose.p* res_;

    // sparse ray cast every x0 deg
    int row = 0;
    for (int theta = -180; theta < 180; theta += dtheta_) {
      theta_rad = static_cast<float>(M_PI * theta / 180.0); // deg to rad
      gain = 0.0;
      int col = 0;
      gain_matrix(row, col) = theta;
      depth_matrix(row, col) = theta;
      for (int phi = static_cast<int>(90 - fov_vert / 2); phi < 90 + fov_vert / 2; phi += dphi_) {
        col++;
        // calculate ray cast direction
        phi_rad = static_cast<float>(M_PI * phi / 180.0f);
        vec[0] = cand_view_m[0] + r_max * cos(theta_rad) * sin(phi_rad);
        vec[1] = cand_view_m[1] + r_max * sin(theta_rad) * sin(phi_rad);
        vec[2] = cand_view_m[2] + r_max * cos(phi_rad);
        dir = (vec - cand_view_m).normalized();
        // initialize ray
        se::ray_iterator<T> ray(*volume_._map_index, cand_view_m, dir, nearPlane, farPlane);
        ray.next();
        // lower bound dist from camera
        const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
        //get IG along ray
        std::pair<float, float> gain_tmp =
            t_min > 0.f ? getInformationGain(dir, t_min, r_max, cand_view_m) : std::make_pair(0.f,
                                                                                              0.f);
        gain_matrix(row, col) = gain_tmp.first;
        depth_matrix(row, col) = gain_tmp.second;
        gain += gain_tmp.first;
      }

      gain_matrix(row, col + 1) = gain;
      row++;
      // add gain to map
      gain_per_yaw[theta] = gain;

    }

    int best_yaw = 0;
    float best_yaw_gain = 0.0;
    // best yaw evaluation
    for (int yaw = -180; yaw < 180; yaw ++) {
      float yaw_score = 0.0;
      // gain FoV horizontal
      for (int fov = -fov_hor / 2; fov < fov_hor / 2; fov++) {
        int theta = yaw + fov;
        // wrap angle
        if (theta < -180)
          theta += 360;
        if (theta >= 180)
          theta -= 360;
        yaw_score += gain_per_yaw[theta];

      }

      if (best_yaw_gain < yaw_score) {
        best_yaw_gain = yaw_score;
        best_yaw = yaw;
      }
    }

//    std::cout << "[se/candview] gain_matrix \n" << gain_matrix.transpose() << std::endl;
//    std::cout << "[se/candview] depth_matrix \n" << depth_matrix.transpose() << std::endl;
//    std::cout << "[se/candview] for cand " << cand_view.format(InLine) << " best theta angle is "
//              << best_yaw << ", best ig is " << best_yaw_gain << std::endl;

//    saveMatrixToDepthImage(depth_matrix.block(0, 1, n_row, n_col).transpose().rowwise().reverse(),
//                           cand_num,
//                           true);
//    saveMatrixToDepthImage(gain_matrix.block(0, 1, n_row, n_col).transpose().rowwise().reverse(),
//                           cand_num,
//                           false);
    float yaw_rad = M_PI * best_yaw / 180.f;
    // pose3D best_pose;
    // best_pose.p = cand_views_[cand_num].cast<float>();
    // best_pose.q = toQuaternion(yaw_rad, 0.0, 0.0); // [rad] as input
    // cand_pose_w_gain.push_back(std::make_pair(best_pose, best_yaw_gain));
    candidates_[i].pose.q = toQuaternion(yaw_rad, 0.0, 0.0);

    // std::cout << "candidates_ " << cand_counter<<  " pos stored " << candidates_[cand_counter].pose.p.format(InLine) << " old  id "<< cand_num << " " << cand_views_[cand_num].format(InLine) << std::endl;
    // if(candidates_[cand_counter].pose.p != cand_views_[cand_num].cast<float>()){
    //   std::cout << " not equal "<< std::endl;
    // }
    candidates_[i].information_gain = best_yaw_gain;
    // cand_counter++;
    // cand_num++;
  }
  // return cand_pose_w_gain;
}

/**
 * finds the best candidate based on information gain
 * @param cand_list
 * @return
 */
template<typename T>
int CandidateView<T>::getBestCandidate() {

  float max_utility = 0.0f;
  int best_cand_idx = 0;
  int cand_counter = 0;

  // TODO parametrize
  const float max_yaw_rate = 0.85; // [rad/s] lee position controller rotors control
  const float path_discount_factor = 0.3;
  const float ig_cost = 0.2;
  float ig_sum = 0.f;

  // path cost = voxel *res[m/vox] / (v =1m/s) = time
  bool force_travelling = candidates_[num_views_].information_gain< ig_target_;
  for (int i = 0; i <= planning_config_.num_cand_views; i++){
        if(candidates_[i].pose.p == Eigen::Vector3f(0, 0, 0) || candidates_[i].planning_solution_status <0){
      continue;
    }
//    std::cout << "[bestcand] position " << (cand.first.p.cast<float>() * res_).format(InLine) <<
//    std::endl;
    ig_sum += candidates_[i].information_gain;
    if (force_travelling ) {
      std::cout << "skip last " << std::endl;
      break;

    }
    float yaw_diff = toEulerAngles(candidates_[i].pose.q).yaw - toEulerAngles(curr_pose_.q).yaw;
//    std::cout << "[bestcand] curr yaw " << toEulerAngles(curr_pose_.q).yaw << " to yaw "
//              << toEulerAngles(cand.first.q).yaw << std::endl;
    float t_path = candidates_[i].path_length * res_;
    //wrap yaw
    if (yaw_diff < M_PI)
      yaw_diff += 2 * M_PI;
    if (yaw_diff >= M_PI)
      yaw_diff -= 2 * M_PI;
    float t_yaw = fabs(yaw_diff / max_yaw_rate);

    float utility = candidates_[i].information_gain/ (t_yaw + t_path);

    std::cout << "[bestcond] cand "<< i << " ig " << candidates_[i].information_gain<< " t_yaw " << t_yaw << " t_path " << t_path
              << " utility " << utility << std::endl;
    if (max_utility < utility) {
      max_utility = utility;
      best_cand_idx = i;
    }

    cand_counter++;
  }


  if (ig_sum / cand_counter < ig_target_) {
    exploration_status_ = 1;
    std::cout << "low information gain  " << ig_sum/cand_counter << std::endl;

  }
  return best_cand_idx;
}

template<typename T>
 VecPose CandidateView<T>::getFinalPath(const float max_yaw_rate,
                              const int idx) {
VecPose path;
  VecPose yaw_path = getYawPath(curr_pose_, candidates_[idx].pose, max_yaw_rate);

  if (yaw_path.size() >= candidates_[idx].path.size()){

    for(int i = 0; i < yaw_path.size() ;i ++){
      if(i<candidates_[idx].path.size()){
      yaw_path[i].p = candidates_[idx].path[i].p;
    } else{
      yaw_path[i].p = candidates_[idx].path[candidates_[idx].path.size()-1].p;
    }
    }
    return yaw_path;
  }else{
    path = candidates_[idx].path;
    for (int i = 0; i < path.size() ; i++){
      if(i<yaw_path.size()){
      path[i].q = yaw_path[i].q;
    }else{
      path[i].p = yaw_path[yaw_path.size()-1].p;
    }
    }
  }


//  std::cout << "[interpolateYaw] path size" << path.size() << std::endl;
  return path;
}

template<typename T>
VecPose CandidateView<T>::getYawPath(const pose3D& start, const pose3D &goal, const float max_yaw_rate){
  VecPose path;
  pose3D pose_tmp;
//  path.push_back(start);
  float yaw_diff = toEulerAngles(goal.q).yaw - toEulerAngles(start.q).yaw;
  if (yaw_diff < M_PI)
    yaw_diff += 2 * M_PI;
  if (yaw_diff >= M_PI)
    yaw_diff -= 2 * M_PI;
//  std::cout << "[interpolateYaw] yaw diff " << yaw_diff << std::endl;
  // interpolate yaw
  if (yaw_diff >= 0) {
    for (float dyaw = 0.0f; dyaw < yaw_diff; dyaw += max_yaw_rate * 0.8) {
      pose_tmp = goal;
      pose_tmp.q = toQuaternion(toEulerAngles(start.q).yaw + dyaw, 0.f, 0.f);
//      std::cout << "[IY] yaw " << toEulerAngles(start.q).yaw + dyaw << std::endl;
      path.push_back(pose_tmp);
    }
  } else {

    for (float dyaw = 0.0f; dyaw > yaw_diff; dyaw -= max_yaw_rate * 0.8) {
      pose_tmp = goal;
      pose_tmp.q = toQuaternion(toEulerAngles(start.q).yaw + dyaw, 0.f, 0.f);
//      std::cout << "[IY] yaw " << toEulerAngles(start.q).yaw + dyaw << std::endl;
      path.push_back(pose_tmp);
    }
  }
  path.push_back(goal);
  return path;
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
void getExplorationPath(std::shared_ptr<Octree<T> > octree_ptr,
                        const Volume<T> &volume,
                        const map3i &free_map,
                        const map3i &frontier_map,
                        const float res,
                        const float step,
                        const Planning_Configuration &planning_config,
                        const Configuration &config,
                        const Eigen::Matrix4f &pose,
                        VecPose &path,
                        VecPose &cand_views,
                        int *exploration_done) {

  auto collision_checker_v = aligned_shared<CollisionCheckerV<T> >(octree_ptr, planning_config);
  auto path_planner_ompl_ptr =
      aligned_shared<PathPlannerOmpl < T> > (octree_ptr, collision_checker_v, planning_config);

  // Candidate view generation
  CandidateView<T>
      candidate_view(volume, planning_config, collision_checker_v, res, config, pose, step);
  candidate_view.getCandidateViews(frontier_map);

  candidate_view.calculateCandidateViewGain();
  // for (const auto &pose : pose_gain) {
  //   cand_views.push_back(pose.first);
  // }

  // const int size = cand_views.size();
  pose3D start = getCurrPose(pose, res);
  // float path_length[size];
  // VecPose all_path[size];
  // VecPose last;
  // // TODO replace push back and parallelize
  // last.push_back(cand_views.back());
  // all_path[size - 1] = last;
  // path_length[size - 1] = 0.f;
  // std::pair<pose3D, int> best_cand_pose_with_gain = std::make_pair(cand_views.back(), 0);

  // if (size > 1) {
    for (int i = 0; i <= planning_config.num_cand_views; i++) {
    if(candidate_view.getPose(i).p == Eigen::Vector3f(0, 0, 0)){
      continue;
    }
      // LOG(INFO) << "Candidate " << i << " goal coord " << cand_views[i].p.format(InLine);
      LOG(INFO) << "Candidate " << i << "start " << start.p.format(InLine) << " goal "
                << candidate_view.getPose(i).p.format(InLine);

      bool setup_planner = path_planner_ompl_ptr->setupPlanner(free_map);
      DLOG(INFO) << "setup planner successful " << setup_planner;
      int path_planned = path_planner_ompl_ptr->planPath(start.p.cast<int>(), candidate_view.getPose(i).p.template cast<int>());
      candidate_view.setSolutionStatus(i, path_planned);
      DLOG(INFO) << "path planned " << path_planned;
      if (path_planned < 0 ) {
        // path_length[i] = (start.p - cand_views[i].p).squaredNorm();
       candidate_view.setPathLength(i, (start.p - candidate_view.getPose(i).p).squaredNorm());
        VecPose vec;
        vec.push_back(candidate_view.getPose(i));
        candidate_view.setPath(i, vec);
        continue;
      }
      // path_length[i] = path_planner_ompl_ptr->getPathLength();
      candidate_view.setPathLength(i, path_planner_ompl_ptr->getPathLength());

      VecPose vec = path_planner_ompl_ptr->getPathSegments_m();
      // all_path[i] = vec;
      candidate_view.setPath(i, vec);
      // DLOG(INFO) << "seg length " << all_path[i].size() << std::endl;

    }
// only plan path if the cand view list has more than one candidate, as the only candidate is the current position
   int best_cand_idx = candidate_view.getBestCandidate();
  // }
  // TODO make this nicer
  if (candidate_view.getExplorationStatus() == 1) {
    *exploration_done = 1;
  } else {
    *exploration_done = 0;
  }
  std::cout << "[se/candview] best candidate is " << candidate_view.getPose(best_cand_idx).p.format(InLine)
            << " cand num " << best_cand_idx << std::endl;
  std::cout << " path length of best cand "<< candidate_view.getPath(best_cand_idx).size() << std::endl;
  VecPose path_tmp = candidate_view.getFinalPath(0.52, best_cand_idx);
  for (const auto &pose : path_tmp){
    std::cout << "cand view " << (pose.p*res ).format(InLine) << " " << pose.q.w() << " "
              << pose.q.vec().format(InLine) << std::endl;
    path.push_back(pose);
  }

}

} // namespace exploration
} // namespace se


#endif //SUPEREIGHT_CANDIDATE_VIEW_HPP
