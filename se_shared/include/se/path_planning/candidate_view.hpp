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

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
typedef SE_FIELD_TYPE FieldType;
namespace se {

namespace exploration {


static se::geometry::collision_status test_voxel2(const Volume<FieldType>::value_type & val) {
  if(val.st == voxel_state::kUnknown) return se::geometry::collision_status::unseen;
  if(val.st == voxel_state::kFree) return se::geometry::collision_status::empty;
  return se::geometry::collision_status::occupied;
};
/**
 * Candidate View
 */

// TODO make candviews to poses => less memory

template<typename T>
class CandidateView {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CandidateView> Ptr;
  CandidateView(const Volume<T> &volume,
                const Planning_Configuration &planning_config,
                const float res,
                const Configuration &config,
                const Eigen::Matrix4f &curr_pose);

  Eigen::Vector3i getOffsetCandidate(const Eigen::Vector3i &cand_v,
                                     const VectorVec3i &frontier_voxels);
  void getCandidateViews(const map3i &frontier_blocks_map);

  void printFaceVoxels(const Eigen::Vector3i &voxel);

  std::pair<double,float> getInformationGain(const Volume<T> &volume,
                            const Eigen::Vector3f &direction,
                            const float tnear,
                            const float tfar,
                            const float step,
                            const Eigen::Vector3f &origin);

  VectorPairPoseDouble getCandidateGain(const float step);

  std::pair<pose3D, double> getBestCandidate(const VectorPairPoseDouble &cand_list);

  pose3D getCurrPose();
  const float getIGWeight_tanh(const float tanh_range,
                               const float tanh_ratio,
                               const float t,
                               const float prob_log,
                               float &t_hit,
                               bool &hit_unknown) const;
 private:
  Eigen::Matrix4f curr_pose_;
  VectorVec3i cand_views_;
  Volume<T> volume_;
  float res_; // [m/vx]
  Planning_Configuration planning_config_;
  Configuration config_;
  float occ_thresh_ = 0.f;


};

template<typename T>
CandidateView<T>::CandidateView(const Volume<T> &volume,
                                const Planning_Configuration &planning_config,
                                const float res,
                                const Configuration &config,
                                const Eigen::Matrix4f &curr_pose)
    :
    volume_(volume),
    planning_config_(planning_config),
    res_(res),
    config_(config),
    curr_pose_(curr_pose) {

}
/**
 * helper function to print the face voxels
 * TODO sparsify
 * @param _voxel center voxel cooridnates
 */
template<typename T>
void CandidateView<T>::printFaceVoxels(const Eigen::Vector3i &_voxel) {
  VectorVec3i face_neighbour_voxel(6);
  face_neighbour_voxel[0] << _voxel.x() - 1, _voxel.y(), _voxel.z();
  face_neighbour_voxel[1] << _voxel.x() + 1, _voxel.y(), _voxel.z();
  face_neighbour_voxel[2] << _voxel.x(), _voxel.y() - 1, _voxel.z();
  face_neighbour_voxel[3] << _voxel.x(), _voxel.y() + 1, _voxel.z();
  face_neighbour_voxel[4] << _voxel.x(), _voxel.y(), _voxel.z() - 1;
  face_neighbour_voxel[5] << _voxel.x(), _voxel.y(), _voxel.z() + 1;
  std::cout << "[se/cand view] face voxel states ";
  for (const auto &face_voxel : face_neighbour_voxel) {
    std::cout << volume_._map_index->get(face_voxel).st << " ";
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
                                                     const VectorVec3i &frontier_voxels) {
  Eigen::Vector3i offset_cand_v(0, 0, 0);
  bool is_valid = false;
  CollisionCheck<T> collision_check(volume_, planning_config_, res_);
  // curr res 24m/128 vox = 0.1875 m/vx

  int offset_v = static_cast<int>(planning_config_.cand_view_safety_radius / res_);
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

  if (volume_._map_index->get(offset_cand_v).st != voxel_state::kFree) {

    offset_cand_v.x() = ceil(-normal.x() * offset_v);
    offset_cand_v.y() = ceil(-normal.y() * offset_v);
    offset_cand_v.z() = ceil(-normal.z() * offset_v);
    offset_cand_v = cand_v + offset_cand_v.cast<int>();
//    printFaceVoxels(volume_, offset_cand_v);
    if (volume_._map_index->get(offset_cand_v).st == voxel_state::kFree) {
      is_valid = true;
    }
  } else {
    is_valid = true;
  }
/*  std::cout << " [se/cand view] candidate " << cand_v.x() << " " << cand_v.y() << " " << cand_v.z()
            << " offset by " << offset_v << " results in " << offset_cand_v.x() << " "
            << offset_cand_v.y() << " " << offset_cand_v.z() << " voxel state "
            << volume_._map_index->get(offset_cand_v).st;*/

  int free_sphere = collision_check.isSphereCollisionFree(offset_cand_v);

  if (is_valid && free_sphere == 1) {
      return offset_cand_v;
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
  auto min_frontier_voxels = 20;
  // get all frontier voxels inside a voxel block
  for (const auto &frontier_block : frontier_blocks_map) {
    VectorVec3i frontier_voxels = node_it.getFrontierVoxels(frontier_block.first);
    frontier_voxels_map[frontier_block.first] = frontier_voxels;
  }
  //check if block and voxel map size are equal
  if (frontier_voxels_map.size() != frontier_blocks_map.size()) {
  }
  // random candidate view generator
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution_block(0, frontier_blocks_map.size() - 1);

  for (int i = 0; i <= planning_config_.num_cand_views; i++) {
    auto it = frontier_voxels_map.begin();
    // go to random voxel block inside the frontier blocks map
    std::advance(it, distribution_block(generator));
    uint64_t rand_morton = it->first;
    if (frontier_voxels_map[rand_morton].size() < min_frontier_voxels) {
      // happens when voxel status is updated but the morton code was not removed from map
      continue;
    }
    std::uniform_int_distribution<int>
        distribution_voxel(0, frontier_voxels_map[rand_morton].size() - 1);
    // random frontier voxel inside the the randomly chosen voxel block
    int rand_voxel = distribution_voxel(generator);
    Eigen::Vector3i candidate_frontier_voxel = frontier_voxels_map[rand_morton].at(rand_voxel);

    VectorVec3i frontier_voxelblock = frontier_voxels_map[rand_morton];
    // offset the candidate frontier voxel
    Eigen::Vector3i cand_view_v = getOffsetCandidate(candidate_frontier_voxel, frontier_voxelblock);

    if (cand_view_v != Eigen::Vector3i(0, 0, 0)) {
      cand_views_.push_back(cand_view_v);
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
std::pair<double, float> CandidateView<T>::getInformationGain(const Volume<T> &volume,
                                            const Eigen::Vector3f &direction,
                                            const float tnear,
                                            const float tfar,
                                            const float step,
                                            const Eigen::Vector3f &origin) {
  auto select_occupancy = [](typename Volume<T>::value_type val) { return val.x; };
  // march from camera away
  double ig_entropy = 0.0f;
  float tanh_range = 4.f;
  const float tanh_ratio = tanh_range / tfar ;
  float t = tnear; // closer bound to camera
  bool hit_unknown= false;
  float t_hit  = 0.f;
  if (tnear < tfar) {
    float stepsize = step;
    // occupancy prob in log2
    float prob_log = volume.interp(origin + direction * t, select_occupancy);
    float weight = 1.0f;
    // check if current pos is free
    if (prob_log <= SURF_BOUNDARY + occ_thresh_) {
      ig_entropy = weight * getEntropy(prob_log);
      for (; t < tfar; t += stepsize) {
        const Eigen::Vector3f pos = origin + direction * t;
        typename Volume<T>::value_type data = volume.get(pos);
        prob_log = volume.interp(origin + direction * t, select_occupancy);
        weight = getIGWeight_tanh(tanh_range, tanh_ratio, t, prob_log, t_hit, hit_unknown);
        ig_entropy +=  weight * getEntropy(prob_log);
/*        if (prob_log == 0.f) {
          std::cout << "[se/candview] raycast dist " << t
                    << " prob " << se::math::getProbFromLog(prob_log) << " weight " << weight
                    << " updated entropy " << ig_entropy << std::endl;
        }*/
// next step along the ray hits a surface with a secure threshold return
        if (prob_log > SURF_BOUNDARY + occ_thresh_) {
          break;
        }
      }
    }
  }
  return std::make_pair(ig_entropy,t);
}

template<typename T>
const float CandidateView<T>::getIGWeight_tanh(const float tanh_range,
                                               const float tanh_ratio,
                                               const float t,
                                               const float prob_log,
                                               float &t_hit,
                                               bool &hit_unknown) const {
  float weight;
  if (prob_log == 0.f) {
          if (!hit_unknown){
            t_hit = t;
            hit_unknown = true;
          }
          weight = tanh(tanh_range - (t-t_hit) * tanh_ratio);
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
VectorPairPoseDouble CandidateView<T>::getCandidateGain(const float step) {
  VectorPairPoseDouble cand_pose_w_gain;

  // add curr pose to be evaluated
  pose3D curr_pose = getCurrPose();
//  std::cout << "[se/candview] curr pose " << curr_pose.p.format(InLine) << " yaw "
//            << toEulerAngles(curr_pose.q).yaw * 180 / M_PI << std::endl;
//  std::cout << "[se/candview] cand view size " << cand_views_.size() << std::endl;
  cand_views_.push_back(curr_pose.p.cast<int>());
  // TODO parameterize the variables
  double gain = 0.0;

  float r_max = 4.0f; // equal to far plane
  float fov_hor = 120.f;
//  float fov_hor = static_cast<float>(planning_config_.fov_hor * 180.f / M_PI); // 2.0 = 114.59 deg
  float fov_vert = fov_hor * 480.f / 640.f; // image size
  float cyl_h = static_cast<float>(2 * r_max * sin(fov_vert * M_PI / 360.0f)); // from paper [2]
  float dr = 0.1f; //[1]

  // temporary
  int dtheta = 20;
  int dphi = 10;
  float r_min = 0.01; // depth camera r min [m]  gazebo model

  float r = 0.5; // [m] random radius
  int phi, theta;
  float phi_rad, theta_rad;
  int n_col = fov_vert / dphi;
  int n_row = 360 / dtheta;

  // debug matrices
  Eigen::MatrixXd gain_matrix(n_row, n_col + 2);
  Eigen::MatrixXd depth_matrix(n_row, n_col + 1);
  std::map<int, double> gain_per_yaw;
  gain_per_yaw.empty();

// cand view in voxel coord
  for (const auto &cand_view : cand_views_) {
    Eigen::Vector3f vec(0.0, 0.0, 0.0); //[m]
    Eigen::Vector3f dir(0.0, 0.0, 0.0);// [m]
    Eigen::Vector3f cand_view_m = cand_view.cast<float>() * res_;

    // sparse ray cast every x0 deg
    int row = 0;
    for (theta = -180; theta < 180; theta += dtheta) {
      theta_rad = static_cast<float>(M_PI * theta / 180.0); // deg to rad
      gain = 0.0;
      int col = 0;
      gain_matrix(row, col) = theta;
      depth_matrix(row, col) = theta;
      for (phi = static_cast<int>(90 - fov_vert / 2); phi < 90 + fov_vert / 2; phi += dphi) {
        col ++;
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
        std::pair<double,float> gain_tmp =
            t_min > 0.f ? getInformationGain(volume_, dir, t_min, r_max, step, cand_view_m) :
            std::make_pair(0.,0.f);
        gain_matrix(row, col) = gain_tmp.first;
        depth_matrix(row, col) = gain_tmp.second;
        gain += gain_tmp.first;
      }

      gain_matrix(row,col+1) = gain;
      row ++;
      // add gain to map
      gain_per_yaw[theta] = gain;

    }
    // TODO optimize the best yaw summation
    // FIFO
    int best_yaw = 0;
    double best_yaw_gain = 0.0;
    // best yaw evaluation
    for (int yaw = -180; yaw < 180; yaw++) {
      double yaw_score = 0.0;
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

/*    std::cout << "[se/candview] gain_matrix \n" << gain_matrix.transpose() << std::endl;
    std::cout << "[se/candview] depth_matrix \n" << depth_matrix.transpose() << std::endl;
    std::cout << "[se/candview] for cand " << cand_view.format(InLine) << " best theta angle is "
              << best_yaw << ", best ig is " << best_yaw_gain << std::endl;*/

    float yaw_rad = M_PI * best_yaw / 180.f;
    pose3D best_pose;
    best_pose.p = cand_view.cast<float>();
    best_pose.q = toQuaternion(yaw_rad, 0.0 , 0.0); // [rad] as input
    cand_pose_w_gain.push_back(std::make_pair(best_pose, best_yaw_gain));
  }
  return cand_pose_w_gain;
}

/**
 * finds the best candidate based on information gain
 * @param cand_list
 * @return
 */
template<typename T>
std::pair<pose3D,
          double> CandidateView<T>::getBestCandidate(const VectorPairPoseDouble &cand_list) {
  double max_gain = 0.0f;
  pose3D best_cand;
  for (const auto &cand : cand_list) {
    if (max_gain < cand.second) {
      best_cand = cand.first;
      max_gain = cand.second;
    }
  }

  return std::make_pair(best_cand, max_gain);
}

// transforms current pose from matrix4f to position and orientation (quaternion)
template<typename T>
pose3D CandidateView<T>::getCurrPose() {
  pose3D curr_pose;
  curr_pose.q = se::math::rot_mat_2_quat(curr_pose_.block<3, 3>(0, 0));
  curr_pose.p = curr_pose_.block<3, 1>(0, 3) / res_;
  return curr_pose;
}

/**
 * calculates exploration path
 * for developing and visualization purposes also return candidate views
 * @tparam T Ofusion or SDF
 * @param volume octree
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
void getExplorationPath(const Volume<T> &volume,
                        const map3i &frontier_map,
                        const double res,
                        const float step,
                        const Planning_Configuration &planning_config,
                        const Configuration &config,
                        const Eigen::Matrix4f &pose,
                        posevector &path,
                        posevector &cand_views) {

  CandidateView<T> candidate_view(volume, planning_config, static_cast<float>(res), config, pose);
  candidate_view.getCandidateViews(frontier_map);

  VectorPairPoseDouble pose_gain = candidate_view.getCandidateGain(step);

  std::pair<pose3D, double> best_cand_pose_with_gain = candidate_view.getBestCandidate(pose_gain);
//  std::cout << "[se/candview] best candidate is " << best_cand_pose_with_gain.first.p.format(InLine)
//            << " yaw " << toEulerAngles(best_cand_pose_with_gain.first.q).yaw * 180.f / M_PI
//            << " with gain " << best_cand_pose_with_gain.second << std::endl;
  path.push_back(best_cand_pose_with_gain.first);
  for (const auto &pose : pose_gain) {
    cand_views.push_back(pose.first);
  }

}

} // namespace exploration
} // namespace se


#endif //SUPEREIGHT_CANDIDATE_VIEW_HPP
