//
// Created by anna on 27/06/19.
//

#ifndef SUPEREIGHT_CANDIDATE_VIEW_HPP
#define SUPEREIGHT_CANDIDATE_VIEW_HPP

#include <set>
#include <map>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <iterator>
#include <type_traits>

#include <Eigen/StdVector>

#include <se/continuous/volume_template.hpp>
#include <se/octree.hpp>
#include <se/node_iterator.hpp>

#include <se/ray_iterator.hpp>
#include <se/utils/math_utils.h>
#include <se/config.h>
#include <se/utils/eigen_utils.h>
#include "collision_checker.hpp"
#include "exploration_utils.hpp"
template<typename T> using Volume = VolumeTemplate<T, se::Octree>;

namespace se {


namespace exploration {

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
                Planning_Configuration planning_config,
                float res,
                Configuration config);
  Eigen::Vector3i getOffsetCandidate(const Eigen::Vector3i &cand_v,
                                     const VectorVec3i &frontier_voxels);
  VectorVec3i getCandidateViews(const map3i &frontier_blocks_map);
  void printFaceVoxels(const Eigen::Vector3i &voxel);
  double getInformationGain(const Volume<T> &volume,
                              const Eigen::Vector3f &direction,
                              const float tnear,
                              const float tfar,
                              const float,
                              const float step,
                              const float,
                              const Eigen::Vector3f &origin);

  VectorPairPoseDouble getCandidateGain(const float step);

  std::pair<pose3D, double> getBestCandidate(const VectorPairPoseDouble& cand_list);
 private:

  VectorVec3i cand_views_;
  Volume<T> volume_;
  float res_; // [m/vx]
  Planning_Configuration planning_config_;
  Configuration config_;
};

template<typename T>
CandidateView<T>::CandidateView(const Volume<T> &volume,
                                Planning_Configuration planning_config,
                                float res,
                                Configuration config)
    :
    volume_(volume), planning_config_(planning_config), res_(res), config_(config) {

}
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
 *
 * @tparam T
 * @param volume
 * @param cand_v [vx coord]
 * @param frontier_voxels  vector with frontier voxels
 * @param res [m/voxel]
 * @param offset [m]
 * @return candidate view in voxel coord global
 */
template<typename T>
Eigen::Vector3i CandidateView<T>::getOffsetCandidate(const Eigen::Vector3i &cand_v,
                                                     const VectorVec3i &frontier_voxels) {
  Eigen::Vector3i offset_cand_v(0, 0, 0);

  CollisionCheck<T> collision_check(volume_, planning_config_, res_);
//  std::cout << "[se/cand view/getoffset] res " << res << std::endl;
  // curr res 24m/128 vox = 0.1875 m/vx
  int offset_v = static_cast<int>(planning_config_.cand_view_safety_radius / res_); // 2 vox
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

//  std::cout << "[se/cand view] max det " << max_det << std::endl;
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
//  std::cout << "[se/cand view] normal " << normal.x() << " " << normal.y() << " " << normal.z()
//            << std::endl;
//  printFaceVoxels(volume_, offset_cand_v);
  // TODO add sphere check
  if (volume_._map_index->get(offset_cand_v).st != voxel_state::kFree) {

    offset_cand_v.x() = ceil(-normal.x() * offset_v);
    offset_cand_v.y() = ceil(-normal.y() * offset_v);
    offset_cand_v.z() = ceil(-normal.z() * offset_v);
    offset_cand_v = cand_v + offset_cand_v.cast<int>();
//    printFaceVoxels(volume_, offset_cand_v);
    if (volume_._map_index->get(offset_cand_v).st == voxel_state::kFree) {
      normal = -normal;
    }
  }
//  std::cout << " [se/cand view] candidate " << cand_v.x() << " " << cand_v.y() << " " << cand_v.z()
//            << " offset by " << offset_v << " results in " << offset_cand_v.x() << " "
//            << offset_cand_v.y() << " " << offset_cand_v.z() << " voxel state "
//            << volume_._map_index->get(offset_cand_v).st << std::endl;


  if (collision_check.isSphereCollisionFree(offset_cand_v)) {
    return offset_cand_v;
  } else {
//      std::cout << "[se/cand view] no free voxel after offset. next candidate" << std::endl;
    return Eigen::Vector3i(0, 0, 0);
  }

}
// function generate candidate views
// input mapvec3i< morton code, vector with frontier voxel in the voxel block>, pointer to octree
// return a vector with possible candidates [voxel coord]



template<typename T>
VectorVec3i CandidateView<T>::getCandidateViews(const map3i &frontier_blocks_map) {

  mapvec3i frontier_voxels_map;
  node_iterator<T> node_it(*(volume_._map_index));
  std::vector<int> max_num_frontier_voxel;
  uint64_t max_morton = 0;

  // get all frontier voxels inside a voxel block
  // TODO
  for (const auto &frontier_block : frontier_blocks_map) {
    VectorVec3i frontier_voxels = node_it.getFrontierVoxels(frontier_block.first);
/*    if (frontier_voxels.size() > max_num_frontier_voxel) {
      max_num_frontier_voxel = frontier_voxels.size();
      max_morton = frontier_block.first;
    }*/
    frontier_voxels_map[frontier_block.first] = frontier_voxels;

  }
  if (frontier_voxels_map.size() != frontier_blocks_map.size()) {
//    std::cout << "[se/cand view] block and voxel map size not equal " << std::endl;
  }
//  std::cout << "[se/cand view] frontier voxels map size " << frontier_blocks_map.size()
//            << std::endl;

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution_block(0, frontier_blocks_map.size() - 1);

  // generate cand views
  for (int i = 0; i <= planning_config_.num_cand_views; i++) {
    auto it = frontier_voxels_map.begin();
    std::advance(it, distribution_block(generator));
    uint64_t rand_morton = it->first;
    if (frontier_voxels_map[rand_morton].size() < 40) {
      // happens when voxel status is updated but the morton code was not removed from map
      continue;
    }
    std::uniform_int_distribution<int>
        distribution_voxel(0, frontier_voxels_map[rand_morton].size() - 1);

//    std::cout << "[se/cand view] frontier voxel map size at " << rand_morton << " is "
//              << frontier_voxels_map[rand_morton].size() << std::endl;

    int rand_voxel = distribution_voxel(generator);
    Eigen::Vector3i candidate_frontier_voxel = frontier_voxels_map[rand_morton].at(rand_voxel);

//    std::cout << "[se/cand view] rand morton " << rand_morton << " rand voxel " << rand_voxel
//              << std::endl;
//    std::cout << "[se/cand view] candidate frontier voxel " << candidate_frontier_voxel.x() << " "
//              << candidate_frontier_voxel.y() << " " << candidate_frontier_voxel.z() << std::endl;
    VectorVec3i frontier_voxels_vec = frontier_voxels_map[rand_morton];

//  random sample N views
// from morton code with max frontier voxels, generate a cand view
    Eigen::Vector3i cand_view_v = getOffsetCandidate(candidate_frontier_voxel, frontier_voxels_vec);

//    std::cout << "[se/cand view] cand view v " << cand_view_v.format(InLine) << std::endl;

    if (cand_view_v != Eigen::Vector3i(0, 0, 0)) {
//      cand_views_.push_back(candidate_frontier_voxel);// for debug only
      cand_views_.push_back(cand_view_v);
    }
  }
//  std::cout << "[se/cand view] getCandView size " << cand_views_.size() << std::endl;
  return cand_views_;

}

/**
 *
 * @param volume
 * @param origin cand view [m]
 * @param direction [m]
 * @param tnear
 * @param tfar
* @eparam step
 * @return
 */
template<typename T>
 double CandidateView<T>::getInformationGain(const Volume<T> &volume,
                                            const Eigen::Vector3f &direction,
                                            const float tnear,
                                            const float tfar,
                                            const float,
                                            const float step,
                                            const float,
                                            const Eigen::Vector3f &origin) {
  auto select_occupancy = [](typename Volume<T>::value_type  val) { return val.x; };
  // march from camera away
  double ig_entropy = 0.0f;
  if (tnear < tfar) {
    float t = tnear; // closer bound to camera
    float stepsize = step;
    // occupancy prob in log2
    float prob_log = volume.interp(origin + direction * t, select_occupancy);
    float prob_next_log = 0;
    // check if current pos is free
    if (prob_log <= SURF_BOUNDARY) {
      ig_entropy = getEntropy(prob_log);
      for (; t < tfar; t += stepsize) {
        const Eigen::Vector3f pos = origin + direction * t;
        typename Volume<T>::value_type data = volume.get(pos);
        // check if the voxel has been updated before
//        std::cout << "[se/candview] raycast dist " << t << " at pos " << pos.format(InLine)
//        << " prob " << se::math::getProbFromLog(data.x) << std::endl;
        if (data.x > -100.f && data.y > 0.f) {
          prob_next_log = volume.interp(origin + direction * t, select_occupancy);
          ig_entropy += getEntropy(prob_next_log);
        }
        // next step along the ray hits a surface with a secure threshold
        if (prob_next_log > SURF_BOUNDARY + 0.4) {
//          std::cout << "[se/rendering] next step is occupied "
//                    << se::math::getProbFromLog(prob_next_log) << std::endl;
          break;
        }
        prob_log = prob_next_log;
      }
    }
  }
  return ig_entropy;
}



// information gain calculation
// source [1] aeplanner gainCubature
// [2]history aware aoutonomous exploration in confined environments using MAVs (cylindric)

template<typename T>
VectorPairPoseDouble CandidateView<T>::getCandidateGain(const float step) {
  VectorPairPoseDouble cand_pose_w_gain;

  // TODO parameterize the variables
  double gain = 0.0;

  float r_max = 4.0f; // equal to far plane
  float fov_hor = static_cast<float>(planning_config_.fov_hor * 180.f / M_PI); // 2.0 = 114.59 deg
  float fov_vert = fov_hor * 480.f / 640.f; // image size
  float cyl_h = static_cast<float>(2 * r_max * sin(fov_vert * M_PI / 360.0f)); // from paper [2]
  float dr = 0.1f; //[1]
  float dphi_rad = M_PI * 10.f / 180.f; // [1]
  float dtheta_rad = M_PI * 10.f / 180.f; //[1]

  // temporary
  int dtheta = 30;
  int dphi = 10;
  float r_min = 0.01; // depth camera r min [m]  gazebo model

  float r = 0.5; // [m] random radius
  int phi, theta;
  float phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

//  Eigen::Vector3d origin(state[0], state[1], state[2]);
// cand view in voxel coord
  for (const auto & cand_view : cand_views_) {
    Eigen::Vector3f vec(0.0, 0.0, 0.0); //[m]
    Eigen::Vector3f dir(0.0, 0.0, 0.0);// [m]
    Eigen::Vector3f cand_view_m = cand_view.cast<float>() * res_;
//    std::cout << "[se/candview] cand view vx " << cand_view.format(InLine) << " to m "
//              << cand_view_m.format(InLine) << std::endl;
    // sparse ray cast every 10 deg
    for (theta = -180; theta < 180; theta += dtheta) {
      theta_rad = static_cast<float>(M_PI * theta / 180.0); // deg to rad
      // up down
      for (phi = static_cast<int>(90 - fov_vert / 2); phi < 90 + fov_vert / 2; phi += dphi) {
        phi_rad = static_cast<float>(M_PI * phi / 180.0f);
//        std::cout << "[se/candview] theta " << theta << " phi " << phi << std::endl;
        vec[0] = cand_view_m[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = cand_view_m[1]  + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = cand_view_m[2] + r * cos(phi_rad);
        dir = (vec - cand_view_m).normalized();

        // raycast
        se::ray_iterator<T> ray(*volume_._map_index, cand_view_m, dir, nearPlane, farPlane);
        ray.next();
        // lower bound dist from camera
        const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
        double gain_tmp =
            t_min > 0.f ? getInformationGain(volume_,
                                             dir,
                                             t_min,
                                             ray.tmax(),
                                             0,
                                             step,
                                             0,
                                             cand_view_m) : 0.f;

        gain_per_yaw[theta] += gain_tmp;
      }
//      std::cout << "[se/candview] for theta " << theta << " information gain is "
//                << gain_per_yaw[theta] << std::endl;
    }

    int best_yaw = 0;
    double best_yaw_gain = 0.0;
    // Comment: could be more efficient ...
    // best yaw evaluation
    for (int yaw = -180; yaw < 180; yaw++) {
      double yaw_score = 0.0;
      // gain FoV horizontal
      for (int fov = -fov_hor / 2; fov < fov_hor / 2; fov++) {
        int theta = yaw + fov;
        // wrap angle
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        yaw_score += gain_per_yaw[theta];
      }

      if (best_yaw_gain < yaw_score) {
        best_yaw_gain = yaw_score;
        best_yaw = yaw;
      }
    }
    std::cout << "[se/candview] for cand " << cand_view.format(InLine) << " best yaw angle is "
              << best_yaw << ", best ig is "<< best_yaw_gain << std::endl;
    double yaw_rad = M_PI * best_yaw / 180.f;
    pose3D best_pose;
    best_pose.p = cand_view.cast<float>();
    best_pose.q = toQuaternion(yaw_rad, 0.0, 0.0);
    cand_pose_w_gain.push_back(std::make_pair(best_pose, best_yaw_gain));
  }
  return cand_pose_w_gain;
}
template<typename T>
std::pair<pose3D, double> CandidateView<T>::getBestCandidate(const VectorPairPoseDouble& cand_list){
  double max_gain =0.0f;
  pose3D best_cand;
  for(const auto& cand : cand_list){
    if(max_gain<cand.second){
      best_cand = cand.first;
      max_gain = cand.second;
    }
  }
  return std::make_pair(best_cand,max_gain);
}
// function:

// function planning called from denseSLAM pipeline
// returns vector with pose => path
// calls frunction from above
//similar structure to projective functor


template<typename T>
void getExplorationPath(const Volume<T> &volume,
                        const map3i &frontier_map,
                        const double res,
                        const float step,
                        const Planning_Configuration &planning_config,
                        const Configuration &config,
                        VectorVec3i &cand_views,
                        posevector &path) {
//  std::cout << " [se/candidate_view] getExplorationPath "  << std::endl;
  CandidateView<T> candidate_view(volume, planning_config, static_cast<float>(res), config);
  cand_views = candidate_view.getCandidateViews(frontier_map);
  VectorPairPoseDouble pose_gain = candidate_view.getCandidateGain(step);

  std::pair<pose3D, double> best_cand_pose_with_gain = candidate_view.getBestCandidate(pose_gain);
  std::cout << "[se/cand view] best candidate is " << best_cand_pose_with_gain.first.p.format
  (InLine) << " yaw " << toEulerAngles(best_cand_pose_with_gain.first.q).yaw *180.f/M_PI <<
  " with gain " << best_cand_pose_with_gain.second << std::endl;
  pose3D tmp_pose({0.f, 0.f, 0.f}, {1.f, 0.f, 0.f, 0.f});
  path.push_back(best_cand_pose_with_gain.first);

}

} // namespace exploration
} // namespace se


#endif //SUPEREIGHT_CANDIDATE_VIEW_HPP
