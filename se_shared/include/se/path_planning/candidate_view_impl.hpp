

#ifndef SUPEREIGHT_CANDIDATE_VIEW_IMPL_HPP
#define SUPEREIGHT_CANDIDATE_VIEW_IMPL_HPP

#include "candidate_view.hpp"
namespace se {

namespace exploration {

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
 *
 * generates random valid candidate views from the frontier map
 * @tparam T
 * @param frontier_blocks_map
 */
template<typename T>
void CandidateView<T>::getCandidateViews(const map3i &frontier_blocks_map, const int frontier_cluster_size) {

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

#pragma omp parallel for
  for (int i = 0; i < num_sampling_; i++) {
    auto it = frontier_voxels_map.begin();

    const int rand_num = distribution_block(generator);
    std::advance(it, rand_num);
    uint64_t rand_morton = it->first;
    if (frontier_voxels_map[rand_morton].size() < frontier_cluster_size) {
      continue;
    }
    std::uniform_int_distribution<int>
        distribution_voxel(0, frontier_voxels_map[rand_morton].size() - 1);
    // random frontier voxel inside the the randomly chosen voxel block
    int rand_voxel = distribution_voxel(generator);
    VecVec3i frontier_voxelblock = frontier_voxels_map[rand_morton];
    Eigen::Vector3i candidate_frontier_voxel = frontier_voxels_map[rand_morton].at(rand_voxel);
    if (boundHeight(&candidate_frontier_voxel.z(),
                    planning_config_.height_max + ground_height_,
                    planning_config_.height_min + ground_height_,
                    res_)) {

      frontier_voxelblock = frontier_voxels_map[se::keyops::encode(candidate_frontier_voxel.x(),
                                                                   candidate_frontier_voxel.y(),
                                                                   candidate_frontier_voxel.z(),
                                                                   volume_._map_index->leaf_level(),
                                                                   volume_._map_index->max_level())];

    }
    bool is_free = pcc_->isSphereSkeletonFreeCand(candidate_frontier_voxel, static_cast<int>(
        planning_config_.robot_safety_radius / res_));
    if (is_free == 1) {
      candidates_[i].pose.p = candidate_frontier_voxel.cast<float>();
      num_cands_++;

    } else {
      candidates_[i].pose.p = Eigen::Vector3f(0, 0, 0);
    }
    // candidates_[i].pose.p = candidate_frontier_voxel.cast<float>();
    DLOG(INFO) << "Cand voxel " << candidate_frontier_voxel.format(InLine);
  }
}

template<typename T>
float CandidateView<T>::getViewInformationGain(pose3D &pose) {
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

  Eigen::MatrixXf gain_matrix(n_row, n_col + 2);
  Eigen::MatrixXf depth_matrix(n_row, n_col + 1);
  std::map<int, float> gain_per_yaw;
  gain_per_yaw.empty();
  Eigen::Vector3f vec(0.0, 0.0, 0.0); //[m]
  Eigen::Vector3f dir(0.0, 0.0, 0.0);// [m]
  Eigen::Vector3f cand_view_m = pose.p * res_;

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
          t_min > 0.f ? getRayInformationGain(dir, t_min, r_max, cand_view_m) : std::make_pair(0.f,
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
  for (int yaw = -180; yaw < 180; yaw++) {
    float yaw_score = 0.0;
    // gain FoV horizontal
    for (int fov = -fov_hor / 2; fov < fov_hor / 2; fov++) {
      int theta = yaw + fov;
      // wrap angle
      wrapYawDeg(theta);
      yaw_score += gain_per_yaw[theta];

    }

    if (best_yaw_gain < yaw_score) {
      best_yaw_gain = yaw_score;
      best_yaw = yaw;
    }
  }

  float yaw_rad = M_PI * best_yaw / 180.f;
  pose.q = toQuaternion(yaw_rad, 0.0, 0.0);
  return best_yaw_gain;
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
std::pair<float, float> CandidateView<T>::getRayInformationGain(const Eigen::Vector3f &direction,
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
#pragma omp parallel for
  for (int i = 0; i < num_sampling_; i++) {

    if (candidates_[i].pose.p == Eigen::Vector3f(0, 0, 0)) {
      // cand_counter++;
      continue;
    }
    float information_gain= getViewInformationGain(candidates_[i].pose);
    candidates_[i].information_gain = information_gain;
  }
  float information_gain =getViewInformationGain(curr_pose_.pose);
  curr_pose_.information_gain = information_gain;

}
template<typename T>
void CandidateView<T>::calculateUtility(Candidate &candidate, const float max_yaw_rate) {

  float yaw_diff = toEulerAngles(candidate.pose.q).yaw - toEulerAngles(pose_.q).yaw;
//    std::cout << "[bestcand] curr yaw " << toEulerAngles(curr_pose_.q).yaw << " to yaw "
//              << toEulerAngles(cand.first.q).yaw << std::endl;
  float t_path = candidate.path_length * res_;
  //wrap yaw
  wrapYawRad(yaw_diff);
  float t_yaw = fabs(yaw_diff / max_yaw_rate);

  candidate.utility = candidate.information_gain / (t_yaw + t_path);

  DLOG(INFO) << "Cand coord" << candidate.pose.p.format(InLine) << "ig "
             << candidate.information_gain << " t_yaw " << t_yaw << " t_path " << t_path
             << " utility " << candidate.utility;
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
  // TODO atomic counter for this
  int cand_counter = 0;

  // TODO parametrize
  const float max_yaw_rate = 0.85; // [rad/s] lee position controller rotors control
  const float path_discount_factor = 0.3;
  const float ig_cost = 0.2;
  float ig_sum = 0.f;

  // path cost = voxel *res[m/vox] / (v =1m/s) = time

  // int views_to_evaluate = force_travelling ? planning_config_.num_cand_views : planning_config_.num_cand_views + 1;
#pragma omp parallel for
  for (int i = 0; i < num_sampling_; i++) {
    if (candidates_[i].pose.p != Eigen::Vector3f(0, 0, 0)
        && candidates_[i].planning_solution_status >= 0) {

      ig_sum += candidates_[i].information_gain;
      calculateUtility(candidates_[i], max_yaw_rate);

      cand_counter++;
    }
  }
  // highest utility in the beginning
  // TODO prune invalid ones after sorting

  std::sort(candidates_.begin(),
            candidates_.end(),
            [](const auto &a, const auto &b) { return (a.utility > b.utility); });
  // for(const auto& cand : candidates_){
  //   LOG(INFO)<< " cand path length " << cand.path_length ;
  // }
  ig_sum += curr_pose_.information_gain;
  calculateUtility(curr_pose_, max_yaw_rate);
  if (ig_sum / cand_counter < ig_target_) {
    exploration_status_ = 1;
    std::cout << "low information gain  " << ig_sum / cand_counter << std::endl;

  }
  return 0;
}

template<typename T>
VecPose CandidateView<T>::getFinalPath(const float max_yaw_rate,  Candidate &candidate ,
  const float sampling_dist) {

  VecPose path;
// first add points between paths
  if(candidate.path.size()>2){
    for (int i = 1; i < candidate.path.size(); i++) {

    LOG(INFO) << "segment start " << candidate.path[i - 1].p.format(InLine) << " end "
              << candidate.path[i].p.format(InLine);
      VecPose path_tmp = addPathSegments(sampling_dist ,candidate.path[i-1], candidate.path[i] );
      getViewInformationGain(candidate.path[i]);
      VecPose yaw_path = getYawPath(candidate.path[i-1], candidate.path[i], max_yaw_rate);
      VecPose path_fused = fusePath(path_tmp, yaw_path);
      // push back the new path
      for(const auto& pose : path_fused){
        path.push_back(pose);
      }
    }
  }else{
    VecPose yaw_path = getYawPath(pose_, candidate.pose, max_yaw_rate);
    VecPose path_tmp = candidate.path;
    path = fusePath(path_tmp, yaw_path);
  }

//  std::cout << "[interpolateYaw] path size" << path.size() << std::endl;
  return path;
}

template<typename T>
VecPose CandidateView<T>::fusePath( VecPose &path_tmp,  VecPose &yaw_path){
  VecPose path_out;
    if (yaw_path.size() >= path_tmp.size()) {

    for (int i = 0; i < yaw_path.size(); i++) {
      if (i < path_tmp.size()) {
        yaw_path[i].p = path_tmp[i].p;
      } else {
        yaw_path[i].p = path_tmp[path_tmp.size() - 1].p;
      }
    }

    path_out= yaw_path;
  } else {

    for (int i = 0; i < path_tmp.size(); i++) {
      if (i < yaw_path.size()) {
        path_tmp[i].q = yaw_path[i].q;
      } else {
        path_tmp[i].q = yaw_path[yaw_path.size() - 1].q;
      }
    }
    path_out = path_tmp;
  }
  return path_out;
}

template<typename T>
VecPose CandidateView<T>::getYawPath(const pose3D &start,
                                     const pose3D &goal,
                                     const float max_yaw_rate) {
  VecPose path;
  pose3D pose_tmp;
//  path.push_back(start);
  float yaw_diff = toEulerAngles(goal.q).yaw - toEulerAngles(start.q).yaw;
  // LOG(INFO) << "[interpolateYaw] yaw diff " << yaw_diff ;
  wrapYawRad(yaw_diff);
  // LOG(INFO) << "[interpolateYaw] yaw diff " << yaw_diff ;
  // interpolate yaw
  if (yaw_diff >= 0) {
    for (float dyaw = 0.0f; dyaw < yaw_diff; dyaw += max_yaw_rate) {
      pose_tmp = goal;
      pose_tmp.q = toQuaternion(toEulerAngles(start.q).yaw + dyaw, 0.f, 0.f);
      pose_tmp.q.normalize();
//      std::cout << "[IY] yaw " << toEulerAngles(start.q).yaw + dyaw << std::endl;
      path.push_back(pose_tmp);
    }
  } else {

    for (float dyaw = 0.0f; dyaw > yaw_diff; dyaw -= max_yaw_rate) {
      pose_tmp = goal;
      pose_tmp.q = toQuaternion(toEulerAngles(start.q).yaw + dyaw, 0.f, 0.f);
      pose_tmp.q.normalize();
//      std::cout << "[IY] yaw " << toEulerAngles(start.q).yaw + dyaw << std::endl;
      path.push_back(pose_tmp);
    }
  }
  path.push_back(goal);
  return path;
}




template<typename T>
VecPose CandidateView<T>::addPathSegments(const float sampling_dist, const pose3D &start_in,
 const pose3D &goal_in) {

  int sampling_dist_v = static_cast<int>(sampling_dist / res_);
  VecPose path_out;
  pose3D start = start_in;
  pose3D goal = goal_in;
  int start_h = start.p.z();
  int goal_h = goal.p.z();
  boundHeight(&start_h,
                    planning_config_.height_max + ground_height_,
                    planning_config_.height_min + ground_height_,
                    res_);
  boundHeight(&goal_h,
                    planning_config_.height_max + ground_height_,
                    planning_config_.height_min + ground_height_,
                    res_);

  LOG(INFO) << start.p.format(InLine) << " " << start_h;
  start.p.z() =(float)start_h ;
  goal.p.z() = (float)goal_h;
  path_out.push_back(start);
  float dist = (goal.p - start.p).norm();
  Eigen::Vector3f dir = (goal.p - start.p).normalized();
  for (float t = sampling_dist_v; t < dist; t += sampling_dist_v) {
    Eigen::Vector3f intermediate_point = start.p + dir * t;
    pose3D tmp(intermediate_point, {1.f, 0.f, 0.f, 0.f});
    LOG(INFO) << "intermediate_point " << intermediate_point.format(InLine);
    path_out.push_back(tmp);

  }
  path_out.push_back(goal);
  return path_out;

}

} // exploration
} // se
#endif