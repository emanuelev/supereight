//
// Created by anna on 30/07/19.
//

#ifndef SUPEREIGHT_COLLISION_CHECKER_VOXEL_HPP
#define SUPEREIGHT_COLLISION_CHECKER_VOXEL_HPP
#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>
#include <math.h>
#include <vector>

#include "se/utils/eigen_utils.h"
#include "se/utils/support_structs.hpp"

#include "se/continuous/volume_template.hpp"
#include "se/octree.hpp"
#include "se/planner_config.h"

namespace se {
namespace exploration {


// TODO state or log prob for collision check



template<typename FieldType>
class CollisionCheckerV {
 public:
  typedef std::shared_ptr<CollisionCheckerV<FieldType> > Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionCheckerV(const std::shared_ptr<Octree < FieldType>
  > octree_ptr,
  const Planning_Configuration &planning_config
  );

  bool isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3i &start,
                                           const Eigen::Vector3i &end,
                                           const int r_min,
                                           const int r_max) const;

  bool isVoxelFree(const Eigen::Vector3i &point_v) const;

  bool isSphereSkeletonFree(const Eigen::Vector3i &position_v, const int radius_v) const;

  // goal
//  bool checkVolume(VecVec3i volume); // checkou flight segment corridor
  bool isLineFree(const Eigen::Vector3i &start,
                  const Eigen::Vector3i &connection,
                  const int num_subpos) const; // check segment fligh corridor skeleton
  float getVoxelDim() const{return voxel_dim_;}

 private:

  std::shared_ptr<Octree < FieldType> >
  octree_ptr_ = nullptr;

   Planning_Configuration planning_params_;

  float voxel_dim_;
};

template<typename FieldType>
CollisionCheckerV<FieldType>::CollisionCheckerV(const std::shared_ptr<Octree < FieldType>
> octree_ptr,
const Planning_Configuration &planning_config)
:
octree_ptr_ (octree_ptr), planning_params_(planning_config){
  voxel_dim_ = octree_ptr->voxelDim();

  DLOG(INFO) << "Collision Checker V setup";
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSphereSkeletonFree(const Eigen::Vector3i &position_v,
                                                        const int radius_v) const {

  // LOG(INFO) << "voxel center "<< position_v.format(InLine) << " radius " << radius_v;
  if (!isVoxelFree(position_v))
    return false;

  VecVec3i shell_main_pos;
  shell_main_pos = {position_v + Eigen::Vector3i(1, 0, 0) * radius_v,
                    position_v - Eigen::Vector3i(1, 0, 0) * radius_v,
                    position_v + Eigen::Vector3i(0, 1, 0) * radius_v,
                    position_v - Eigen::Vector3i(0, 1, 0) * radius_v,
                    position_v + Eigen::Vector3i(0, 0, 1) * radius_v,
                    position_v - Eigen::Vector3i(0, 0, 1) * radius_v};
  for (VecVec3i::iterator it = shell_main_pos.begin(); it!= shell_main_pos.end(); it++) {
    if (!isVoxelFree(*it))
      return false;
  }

  Eigen::Vector3i
      vec1_u = Eigen::Vector3i(1, 0, 0) + Eigen::Vector3i(0, 1, 0) + Eigen::Vector3i(0, 0, 1);
  vec1_u.normalize();
  Eigen::Vector3i
      vec2_u = Eigen::Vector3i(-1, 0, 0) + Eigen::Vector3i(0, 1, 0) + Eigen::Vector3i(0, 0, 1);
  vec2_u.normalize();
  Eigen::Vector3i
      vec3_u = Eigen::Vector3i(1, 0, 0) + Eigen::Vector3i(0, -1, 0) + Eigen::Vector3i(0, 0, 1);
  vec3_u.normalize();
  Eigen::Vector3i
      vec4_u = Eigen::Vector3i(-1, 0, 0) + Eigen::Vector3i(0, -1, 0) + Eigen::Vector3i(0, 0, 1);
  vec4_u.normalize();

  VecVec3i shell_sub_pos;
  shell_sub_pos = {position_v + vec1_u * radius_v, position_v - vec1_u * radius_v,
                   position_v + vec2_u * radius_v, position_v - vec2_u * radius_v,
                   position_v + vec3_u * radius_v, position_v - vec3_u * radius_v,
                   position_v + vec4_u * radius_v, position_v - vec4_u * radius_v};
  for (VecVec3i::iterator it = shell_sub_pos.begin(); it != shell_sub_pos.end(); ++it) {
    if (!isVoxelFree(*it))
      return false;
  }

  return true;
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isVoxelFree(const Eigen::Vector3i &point_v) const {

  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;
  bool is_voxel_block;

  octree_ptr_->fetch_octant(point_v.x(), point_v.y(), point_v.z(), node, is_voxel_block);
  if (is_voxel_block) {
    block = static_cast<se::VoxelBlock<FieldType> *> (node);
    if (block->data(point_v).x >= 0.f && block->data(point_v).st != voxel_state::kFree) {
      // LOG(INFO) << "collision at "
      // << (point_v.cast<float>() * voxel_dim_).format(InLine) << " state "
      // << block->data(point_v).st << std::endl;
      return false;
    }
  } else {
    // TODO without up propagation, ignore unknown nodes
    if (octree_ptr_->get(se::keyops::decode(node->code_)).x > 0.f) {
      // const Eigen::Vector3i pos = se::keyops::decode(node->code_);
      // LOG(INFO) << "collision at node "
      // << (pos.cast<float>() * voxel_dim_).format(InLine) << std::endl;
      return false;
    }
  }
  return true;

}

//   !!! took code snippets from DubinsSateSpace MotionValidator
template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3i &start,
                                                                       const Eigen::Vector3i &end,
                                                                       const int r_min_v,
                                                                       const int r_max_v) const {

  if (start == end)
    return true;
  /**
   * Set up the line-of-sight connection
   */
  const float seg_prec = planning_params_.skeleton_sample_precision; //
  // Precision at which
  // to sample the connection [m/voxel]

  VecVec3i segment_flight_corridor;
  const Eigen::Vector3i corridor_axis_u = (end - start) / (end - start).norm();
  const Eigen::Vector3i start_corridor_v = start - r_max_v * corridor_axis_u;
  const Eigen::Vector3i end_corridor_v = end + r_max_v * corridor_axis_u;

  const Eigen::Vector3i vec_seg_connection_v =
      end_corridor_v - start_corridor_v; // The vector in [voxel] connecting the start and end
  // position
  const Eigen::Vector3i vec_seg_direction_u = vec_seg_connection_v
      / vec_seg_connection_v.norm(); // The vector in [voxel] connecting the start and end position
  const int num_axial_subpos =
      vec_seg_connection_v.norm() / seg_prec; // Number of sub points along the line to be checked

  if (!isSphereSkeletonFree(end, r_max_v)) {
    return false;
  }
  // TODO implement checkLine
  if (!isLineFree(start_corridor_v,
                  vec_seg_connection_v,
                  num_axial_subpos)) { // Check if the line-of-sight connection is collision free
    DLOG(INFO) << "line not free at " << (start_corridor_v).format(InLine);
    return false;
  }
  /**
   * Get cylinder extrema in horizontal and vertical direction
   */
  Eigen::Vector3i vec_z_u = Eigen::Vector3i(0, 0, 1);
  Eigen::Vector3i vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  Eigen::Vector3i vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3i(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3i(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3i(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }

  VecVec3i shell_main_pos;
  shell_main_pos =
      {start_corridor_v + vec_horizontal_u * r_max_v, start_corridor_v - vec_horizontal_u * r_max_v,
       start_corridor_v + vec_vertical_u * r_max_v, start_corridor_v - vec_vertical_u * r_max_v};
  for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); ++it) {
    // TODO
    if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
      DLOG(INFO) << "line not free at " << (*it).format(InLine);
      return false;
    }
  }

  const int num_circ_shell_subpos = r_max_v * M_PI / (2 * seg_prec);

  if (num_circ_shell_subpos > 1) {

    AlignedQueueTupleVec3i circ_shell_pos; // Not sure yet
    circ_shell_pos.push(std::make_tuple(vec_vertical_u,
                                        vec_horizontal_u,
                                        1,
                                        num_circ_shell_subpos - 1));

    while (!circ_shell_pos.empty()) {

      std::tuple<Eigen::Vector3i, Eigen::Vector3i, int, int> x = circ_shell_pos.front();
      const int mid = (std::get<2>(x) + std::get<3>(x)) / 2;
      Eigen::Vector3i vec1_u = (std::get<0>(x) + std::get<1>(x)) / 2;
      vec1_u.normalize();
      Eigen::Vector3i vec2_u = (std::get<0>(x) - std::get<1>(x)) / 2;
      vec2_u.normalize();

      VecVec3i circ_shell_sub_pos;
      circ_shell_sub_pos =
          {start_corridor_v + vec1_u * r_max_v, start_corridor_v - vec1_u * r_max_v,
           start_corridor_v + vec2_u * r_max_v, start_corridor_v - vec2_u * r_max_v};
      for (VecVec3i::iterator it = circ_shell_sub_pos.begin(); it != circ_shell_sub_pos.end();
           ++it) {
        if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
          return false;
        }
      }

      circ_shell_pos.pop();

      if (std::get<2>(x) < mid)
        circ_shell_pos.push(std::make_tuple(std::get<0>(x), vec1_u, std::get<2>(x), mid - 1));
      if (std::get<3>(x) > mid)
        circ_shell_pos.push(std::make_tuple(vec1_u, std::get<1>(x), mid + 1, std::get<3>(x)));
    }
  }

  const int num_radial_subpos = r_max_v / seg_prec;

  if (num_radial_subpos > 1) {

    std::queue<std::pair<int, int>> radial_pos; // Not sure yet
    radial_pos.push(std::make_pair(1, num_radial_subpos - 1));

    while (!radial_pos.empty()) {
      std::pair<int, int> y = radial_pos.front();
      const int mid = (y.first + y.second) / 2;
      const int r_v = mid / num_radial_subpos * r_max_v;

      VecVec3i circ_main_pos;
      circ_main_pos =
          {start_corridor_v + vec_horizontal_u * r_v, start_corridor_v - vec_horizontal_u * r_v,
           start_corridor_v + vec_vertical_u * r_v, start_corridor_v - vec_vertical_u * r_v};
      for (VecVec3i::iterator it = circ_main_pos.begin(); it != circ_main_pos.end(); ++it) {

        if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
          return false;
        }
      }

      const int num_circ_subpos = r_v * M_PI / (2 * seg_prec);

      if (num_circ_subpos > 1) {

        AlignedQueueTupleVec3i circ_sub_pos; // Not sure yet
        circ_sub_pos.push(std::make_tuple(vec_vertical_u,
                                          vec_horizontal_u,
                                          1,
                                          num_circ_subpos - 1));

        while (!circ_sub_pos.empty()) {

          std::tuple<Eigen::Vector3i, Eigen::Vector3i, int, int> x = circ_sub_pos.front();
          int mid = (std::get<2>(x) + std::get<3>(x)) / 2;
          Eigen::Vector3i vec1_u = (std::get<0>(x) + std::get<1>(x)) / 2;
          vec1_u.normalize();
          Eigen::Vector3i vec2_u = (std::get<0>(x) - std::get<1>(x)) / 2;
          vec2_u.normalize();

          VecVec3i sub_starts;
          sub_starts = {start_corridor_v + vec1_u * r_max_v, start_corridor_v - vec1_u * r_max_v,
                        start_corridor_v + vec2_u * r_max_v, start_corridor_v - vec2_u * r_max_v};
          for (VecVec3i::iterator it = sub_starts.begin(); it != sub_starts.end(); ++it) {

            if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
              DLOG(INFO) << "line not free at " << (*it).format(InLine);
              return false;
            }
          }

          circ_sub_pos.pop();

          if (std::get<2>(x) < mid)
            circ_sub_pos.push(std::make_tuple(std::get<0>(x), vec1_u, std::get<2>(x), mid - 1));
          if (std::get<3>(x) > mid)
            circ_sub_pos.push(std::make_tuple(vec1_u, std::get<1>(x), mid + 1, std::get<3>(x)));
        }
      }

      radial_pos.pop();

      if (y.first < mid)
        radial_pos.push(std::make_pair(y.first, mid - 1));
      if (y.second > mid)
        radial_pos.push(std::make_pair(mid + 1, y.second));
    }
  }

  return true;
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isLineFree(const Eigen::Vector3i &start_v,
                                              const Eigen::Vector3i &connection_v,
                                              const int num_subpos) const {

  std::queue<std::pair<int, int>> line_pos;
  line_pos.push(std::make_pair(1, num_subpos - 1));

  // Repeatedly subdivide the path segment in the middle (and check the middle)
  while (!line_pos.empty()) {
    std::pair<int, int> x = line_pos.front();

    int mid = (x.first + x.second) / 2;

    // Compute midpoint
    Eigen::Vector3i position_v = (mid / num_subpos * connection_v + start_v);

    if (!isVoxelFree(position_v))
      return false;

    line_pos.pop();

    if (x.first < mid)
      line_pos.push(std::make_pair(x.first, mid - 1));
    if (x.second > mid)
      line_pos.push(std::make_pair(mid + 1, x.second));
  }

  return true;
}

}
}
#endif //SUPEREIGHT_COLLISION_CHECKER_VOXEL_HPP
