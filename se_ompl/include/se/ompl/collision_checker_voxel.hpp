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
#include "se/occupancy_world.hpp"
#include "volume_shifter.hpp"
#include "se/planning_parameter.hpp"

#include "se/continuous/volume_template.hpp"
#include "se/octree.hpp"

namespace se {
namespace exploration {

template<typename FieldType>
class CollisionCheckerV {
 public:
  typedef std::shared_ptr<CollisionCheckerV<FieldType> > Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionCheckerV(const std::shared_ptr<Octree < FieldType>
  > octree_ptr,
  const PlanningParameter &ompl_params
  );


  bool isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3i &start,
                                          const Eigen::Vector3i &end,
                                          const int r_min,
                                          const int r_max) const;

  bool isVoxelFree(const Eigen::Vector3i &point_v) const;

  bool isSphereCollisionFree(const Eigen::Vector3i &position_v, const int radius_v) const; // start
  // end
  bool isSphereSkeletonFree(const Eigen::Vector3i &position_v, const int radius_v) const;

  // goal
//  bool checkVolume(VectorVec3i volume); // checkou flight segment corridor
  bool isLineFree(const Eigen::Vector3i &start,
                 const Eigen::Vector3i &connection,
                 const int num_subpos) const; // check segment fligh corridor skeleton
//
//  bool checkLineDistance(const Eigen::Vector3i &start,
//                         const Eigen::Vector3i &end,
//                         int radius); // motion validator
//  bool checkVoxelDistance(const Eigen::Vector3i &position, int radius); //motion validator
//
//
//  bool expandFlightCorridorDistance(Path<kDim>::Ptr path_m);
//  bool expandFlightCorridorSkeleton(Path<kDim>::Ptr path_m);
//  bool expandFlightCorridor(Path<kDim>::Ptr path_m); for evaluation
//
//  void findMinLineDistance(const Eigen::Vector3d &start,
//                           const Eigen::Vector3d &end,
//                           double &min_distance_m); // expand flight segment corridor
//  void getVoxelDistance(const Eigen::Vector3d &position_m, double &distance);
//  double getVoxelDistance(const Eigen::Vector3d &position_m); // state validty checker (NOT used)

 private:
  exploration::VolumeShifter::Ptr vs_ = nullptr;
  std::shared_ptr<Octree < FieldType> >
  octree_ptr_ = nullptr;
  // from occupancy world
  std::string id_;
  std::vector<key_t> alloc_list_;

  PlanningParameter ompl_params_;

  float voxel_dim_;
  bool treat_unknown_as_occupied_;
};

template<typename FieldType>
CollisionCheckerV<FieldType>::CollisionCheckerV(const std::shared_ptr<Octree < FieldType>
> octree_ptr,
const PlanningParameter &ompl_params
)
:
octree_ptr_ (octree_ptr), ompl_params_(ompl_params) {
  treat_unknown_as_occupied_ = ompl_params.treat_unknown_as_occupied_;
  voxel_dim_ = octree_ptr->voxelDim();
  vs_ = std::unique_ptr<VolumeShifter>(new VolumeShifter(voxel_dim_,
                                                         ompl_params.volume_precision_factor_));
  LOG(INFO) << "Collision Checker  setup";
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSphereSkeletonFree(const Eigen::Vector3i &position_v,
                                                        const int radius_v) const {

  DLOG(INFO) << "center "<< position_v.format(InLine) << " radius " << radius_v;
  if (!isVoxelFree(position_v))
    return false;

  VectorVec3i shell_main_pos;
  shell_main_pos = {position_v + Eigen::Vector3i(1, 0, 0) * radius_v,
                    position_v - Eigen::Vector3i(1, 0, 0) * radius_v,
                    position_v + Eigen::Vector3i(0, 1, 0) * radius_v,
                    position_v - Eigen::Vector3i(0, 1, 0) * radius_v,
                    position_v + Eigen::Vector3i(0, 0, 1) * radius_v,
                    position_v - Eigen::Vector3i(0, 0, 1) * radius_v};
  for (const auto &it : shell_main_pos) {
    if (!isVoxelFree(it))
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

  VectorVec3i shell_sub_pos;
  shell_sub_pos = {position_v + vec1_u * radius_v, position_v - vec1_u * radius_v,
                   position_v + vec2_u * radius_v, position_v - vec2_u * radius_v,
                   position_v + vec3_u * radius_v, position_v - vec3_u * radius_v,
                   position_v + vec4_u * radius_v, position_v - vec4_u * radius_v};
  for (VectorVec3i::iterator it = shell_sub_pos.begin(); it != shell_sub_pos.end(); ++it) {
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
    if (block->data(point_v).st != voxel_state::kFree) {
      DLOG(INFO) << "collision at "
                << (point_v.cast<float>() * voxel_dim_).format(InLine) << " state "
                << block->data(point_v).st << std::endl;
      return false;
    }
  } else {
    if (octree_ptr_->get(se::keyops::decode(node->code_)).st != voxel_state::kFree) {
      const Eigen::Vector3i pos = se::keyops::decode(node->code_);
      DLOG(INFO) << "collision at node "
                << (pos.cast<float>() * voxel_dim_).format(InLine) << std::endl;
      return false;
    }
  }
  return true;

}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSphereCollisionFree(const Eigen::Vector3i &center,
                                                         const int radius_v) const {

  DLOG(INFO) << "radius " << radius_v << " center " << center.format(InLine);
  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;
  bool is_voxel_block;
  Eigen::Vector3i prev_pos(0, 0, 0);
  for (int x = -radius_v; x <= radius_v; x++) {
    for (int y = -radius_v; y <= radius_v; y++) {
      for (int z = -radius_v; z <= radius_v; z++) {
        Eigen::Vector3i point_offset_v(x, y, z);
        //check if point is inside the sphere radius
//        std::cout << "sphere norm " << point_offset_v.norm() <<std::endl;
        if (point_offset_v.norm() <= radius_v) {
          // check if voxelblock is allocated or only node
          Eigen::Vector3i point_v = point_offset_v + center;
          // first round
          if (node == nullptr || block == nullptr) {
            octree_ptr_->fetch_octant(point_v.x(), point_v.y(), point_v.z(), node, is_voxel_block);
            prev_pos = point_v;
            if (is_voxel_block) {
              block = static_cast<se::VoxelBlock<FieldType> *> (node);
            } else {
              if (octree_ptr_->get(se::keyops::decode(node->code_)).st != voxel_state::kFree) {
                Eigen::Vector3i pos = se::keyops::decode(node->code_);
                std::cout << " [secollision] collision at node "
                          << (pos.cast<float>() * voxel_dim_).format(InLine) << std::endl;
                return false;
              }
            }
          } else {
            // if true keep old voxelblock pointer and fetch
            // else get new voxel block
            if ((point_v.x() / BLOCK_SIDE) == (prev_pos.x() / BLOCK_SIDE)
                && (point_v.y() / BLOCK_SIDE) == (prev_pos.y() / BLOCK_SIDE)
                && (point_v.z() / BLOCK_SIDE) == (prev_pos.z() / BLOCK_SIDE)) {
              if (block->data(point_v).st != voxel_state::kFree) {
                std::cout << " [secollision] collision at "
                          << (point_v.cast<float>() * voxel_dim_).format(InLine) << " state "
                          << block->data(point_v).st << std::endl;
                return false;
              }
            } else {
              octree_ptr_->fetch_octant(point_v.x(),
                                        point_v.y(),
                                        point_v.z(),
                                        node,
                                        is_voxel_block);
              if (is_voxel_block) {
                block = static_cast<se::VoxelBlock<FieldType> *> (node);
                if (block->data(point_v).st != voxel_state::kFree) {
                  std::cout << " [secollision] collision at "
                            << (point_v.cast<float>() * voxel_dim_).format(InLine) << " state "
                            << block->data(point_v).st << std::endl;
                  return false;
                }
              } else {
                block = nullptr;
                if (octree_ptr_->get(se::keyops::decode(node->code_)).st != voxel_state::kFree) {
                  Eigen::Vector3i pos = se::keyops::decode(node->code_);
                  std::cout << " [secollision] collision at node "
                            << (pos.cast<float>() * voxel_dim_).format(InLine) << std::endl;
                  return false;
                }
              }

            }
          }
          prev_pos = point_v;
        }

      }
    }
  }
//  std::cout << "[se/collision_checker] sphere radius " << planning_config_.cand_view_safety_radius
//            << " [m] =  " << radius_v << " voxels around center " << pos_v.format(InLine) << std::endl;
  return true;
}

//   !!! took code snippets from DubinsSateSpace MotionValidator
template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3i &start,
                                                                      const Eigen::Vector3i &end,
                                                                      const int r_min_v,
                                                                      const int r_max_v) const {


  if(start == end)
    return true;
  /**
   * Set up the line-of-sight connection
   */
  const double seg_prec = ompl_params_.skeleton_sample_precision_; //
  // Precision at which
  // to sample the connection [m/voxel]

  VectorVec3i segment_flight_corridor;
  const Eigen::Vector3i corridor_axis_u = (end - start) / (end - start).norm();
  const Eigen::Vector3i start_corridor_v = start - r_max_v * corridor_axis_u;
  const Eigen::Vector3i end_corridor_v = end + r_max_v * corridor_axis_u;

  const  Eigen::Vector3i vec_seg_connection_v =
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
                 num_axial_subpos)) // Check if the line-of-sight connection is collision free
    return false;

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

  VectorVec3i shell_main_pos;
  shell_main_pos =
      {start_corridor_v + vec_horizontal_u * r_max_v, start_corridor_v - vec_horizontal_u * r_max_v,
       start_corridor_v + vec_vertical_u * r_max_v, start_corridor_v - vec_vertical_u * r_max_v};
  for (VectorVec3i::iterator it = shell_main_pos.begin();
       it != shell_main_pos.end(); ++it) {
    // TODO
    if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos))
      return false;
  }

  int num_circ_shell_subpos = r_max_v * M_PI / (2 * seg_prec);

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

      VectorVec3i circ_shell_sub_pos;
      circ_shell_sub_pos =
          {start_corridor_v + vec1_u * r_max_v, start_corridor_v - vec1_u * r_max_v,
           start_corridor_v + vec2_u * r_max_v, start_corridor_v - vec2_u * r_max_v};
      for (VectorVec3i::iterator it = circ_shell_sub_pos.begin();
           it != circ_shell_sub_pos.end(); ++it) {
        if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos))
          return false;
      }

      circ_shell_pos.pop();

      if (std::get<2>(x) < mid)
        circ_shell_pos.push(std::make_tuple(std::get<0>(x), vec1_u, std::get<2>(x), mid - 1));
      if (std::get<3>(x) > mid)
        circ_shell_pos.push(std::make_tuple(vec1_u, std::get<1>(x), mid + 1, std::get<3>(x)));
    }
  }

  const  int num_radial_subpos = r_max_v / seg_prec;

  if (num_radial_subpos > 1) {

    std::queue<std::pair<int, int>> radial_pos; // Not sure yet
    radial_pos.push(std::make_pair(1, num_radial_subpos - 1));

    while (!radial_pos.empty()) {
      std::pair<int, int> y = radial_pos.front();
      const int mid = (y.first + y.second) / 2;
      const int r_v =  mid /  num_radial_subpos * r_max_v;

      VectorVec3i circ_main_pos;
      circ_main_pos =
          {start_corridor_v + vec_horizontal_u * r_v, start_corridor_v - vec_horizontal_u * r_v,
           start_corridor_v + vec_vertical_u * r_v, start_corridor_v - vec_vertical_u * r_v};
      for (VectorVec3i::iterator it = circ_main_pos.begin();
           it != circ_main_pos.end(); ++it) {

        if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos))
          return false;
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

          VectorVec3i sub_starts;
          sub_starts = {start_corridor_v + vec1_u * r_max_v, start_corridor_v - vec1_u * r_max_v,
                        start_corridor_v + vec2_u * r_max_v, start_corridor_v - vec2_u * r_max_v};
          for (VectorVec3i::iterator it = sub_starts.begin();
               it != sub_starts.end(); ++it) {

            if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos))
              DLOG(INFO) << "line not free at " << (*it).format(InLine);
              return false;
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
    Eigen::Vector3i position_v = ( mid /  num_subpos * connection_v + start_v);

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
