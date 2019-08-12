/**
 * Probabilistic Trajectory Planning, Probabilistic Collision Checker.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @file OccupancyWorld.hpp
 *
 * @author Nils Funk
 * @date July 11, 2018
 */


#ifndef EXPLORATION_CollisionCheckerM_HPP
#define EXPLORATION_CollisionCheckerM_HPP

#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>
#include <math.h>
#include <vector>

#include "se/utils/eigen_utils.h"
#include "se/utils/support_structs.hpp"


#include "se/continuous/volume_template.hpp"
#include "se/octree.hpp"
//#include "FoldedStatus.h"

//#include "state_uncertanty.hpp"
namespace se {
namespace exploration {

template<typename FieldType> using Volume = VolumeTemplate<FieldType, se::Octree>;
template<typename FieldType>
class CollisionCheckerM {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CollisionCheckerM<FieldType> > Ptr;
/**
 *
 * @param octree_ptr PCC should not be owner of octree
 * @param ompl_params
 */
  CollisionCheckerM(const std::shared_ptr<Octree<FieldType> > octree_ptr,
                       const Planning_Configuration &planning_config);


  bool isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3f &start_m,
                                          const Eigen::Vector3f &end_m,
                                          const float r_min_m,
                                          const float r_max_m) const;


  bool isVoxelFree(const Eigen::Vector3f &position_m) const ;
  bool isSphereSkeletonFree(const Eigen::Vector3f &position_m, const float radius_m) const;
  bool isLineFree(const Eigen::Vector3f &start_m,
                 const Eigen::Vector3f &connection_m,
                 const int num_subpos) const;


  // bool isSphereCollisionFree(const Eigen::Vector3f &position_m, const float radius_m) const;
  // bool checkVolume(std::vector<Eigen::Vector3f> volume) const; // check flight segment corridor

  // bool checkLineDistance(const Eigen::Vector3f &start_m,
  //                        const Eigen::Vector3f &end_m,
  //                        float radius_m) const;
  // bool checkVoxelDistance(const Eigen::Vector3f &position_m, float radius_m) const;
  // bool expandFlightCorridorDistance(Path<kDim>::Ptr path_m) const;
  // bool expandFlightCorridorSkeleton(Path<kDim>::Ptr path_m) const;
  // bool expandFlightCorridor(Path<kDim>::Ptr path_m) const;
  // void findMinLineDistance(const Eigen::Vector3f &start_m,
  //                          const Eigen::Vector3f &end_m,
  //                          double &min_distance_m) const; // expand flight corridor (not used)
  // void getVoxelDistance(const Eigen::Vector3f &position_m, double &distance) const;
  // double getVoxelDistance(const Eigen::Vector3f &position_m) const; // motion validator

 private:
  // exploration::VolumeShifter::Ptr vs_ = nullptr;
//  std::shared_ptr<OccupancyWorld> ow_ = NULL;
  std::shared_ptr<Octree<FieldType> > octree_ptr_ = nullptr;
   Planning_Configuration planning_params_;

  float voxel_dim_;
  bool treat_unknown_as_occupied_;
};

template<typename FieldType>
CollisionCheckerM<FieldType>::CollisionCheckerM(const std::shared_ptr<Octree<FieldType> > octree_ptr,
                                                      const Planning_Configuration &planning_config)
    :
    octree_ptr_(octree_ptr),
    planning_params_(planning_config)
    {

  voxel_dim_ = octree_ptr->voxelDim();

  DLOG(INFO) << "Collision Checker M setup";
}

template<typename FieldType>
bool CollisionCheckerM<FieldType>::isVoxelFree(const Eigen::Vector3f &position_m)  const {

  /**
   * Convert position from [m] to [voxel]
   */
  const Eigen::Vector3i point_v = (position_m / voxel_dim_).cast<int>();

  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;
  bool is_voxel_block;

  octree_ptr_->fetch_octant(point_v.x(), point_v.y(), point_v.z(), node, is_voxel_block);
  if (is_voxel_block) {
    block = static_cast<se::VoxelBlock<FieldType> *> (node);
    if ( block->data(point_v).st != voxel_state::kFree
      && block->data(point_v).st != voxel_state::kFrontier) {
      // LOG(INFO) << "collision at m "<< position_m.format(InLine) << " voxel "
      // << point_v.format(InLine) << " state "
      // << block->data(point_v).st << std::endl;
      return false;
    }
  } else {
    // TODO without up propagation, ignore unknown nodes
    if (octree_ptr_->get(se::keyops::decode(node->code_)).x > 0.f) {
      const Eigen::Vector3i pos = se::keyops::decode(node->code_);

      // LOG(INFO) << "collision at node "
      // << (pos.cast<float>() * voxel_dim_).format(InLine) << " prob "
      // << octree_ptr_->get(se::keyops::decode(node->code_)).x << std::endl;
      return false;
    }
  }
  return true;
}

template<typename FieldType>
bool CollisionCheckerM<FieldType>::isLineFree(const Eigen::Vector3f &start_m,
                                                const Eigen::Vector3f &connection_m,
                                                const int num_subpos) const {

  std::queue<std::pair<int, int>> line_pos;
  line_pos.push(std::make_pair(1, num_subpos - 1));

  // Repeatedly subdivide the path segment in the middle (and check the middle)
  while (!line_pos.empty()) {
    std::pair<int, int> x = line_pos.front();

    int mid = (x.first + x.second) / 2;

    // Compute midpoint
    Eigen::Vector3f position_m = ((float) mid / (float) num_subpos * connection_m + start_m);

    if (!isVoxelFree(position_m))
      return false;

    line_pos.pop();

    if (x.first < mid)
      line_pos.push(std::make_pair(x.first, mid - 1));
    if (x.second > mid)
      line_pos.push(std::make_pair(mid + 1, x.second));
  }

  return true;
}

//========================================================//
//                DENSE SAMPLING INTERFACE                //
//========================================================//

/**
 * Check for collision within a given volume
 * The robot collides if
 * - One voxel is occupied i.e. voxel_occupancy = 1;
 * - One voxel is unseen if treat_unknown_as_occupied eq. true, i.e voxel_occupancy = 0;
 * A free voxel is declared as voxel_occupancy = -1;
 */



// template<typename FieldType>
// bool CollisionCheckerM<FieldType>::checkVolume(std::vector<Eigen::Vector3f> volume) const{

//   /**
//    * Iterate through whole voxel vector
//    */
//   for (auto it_i = volume.begin(); it_i != volume.end(); ++it_i) {

//     /**
//      * Check all voxels that are within the current voxel block
//      */
//     se::VoxelBlock<FieldType>
//         *voxel_block = octree_ptr_->fetch((*it_i).x(), (*it_i).y(), (*it_i).z());

//     if (treat_unknown_as_occupied_ && voxel_block == nullptr) {
//       return false;
//     }

//     Eigen::Vector3f block_coordinates = voxel_block->coordinates();

//     for (std::vector<Eigen::Vector3f>::iterator it_j = volume.begin(); it_j != volume.end();
//          ++it_j) {
//       if (block_coordinates.x() <= (*it_j).x()
//           && (*it_j).x() < (block_coordinates.x() + se::Octree<FieldType>::blockSide)
//           && block_coordinates.y() <= (*it_j).y()
//           && (*it_j).y() < (block_coordinates.y() + se::Octree<FieldType>::blockSide)
//           && block_coordinates.z() <= (*it_j).z()
//           && (*it_j).z() < (block_coordinates.z() + se::Octree<FieldType>::blockSide)) {

//         float voxel_occupancy = voxel_block->data(*it_j).x;
//         if (voxel_occupancy > 0.5) { // Surface boundary at 0.5 occupancy
//           return false;
//         }
//         if (treat_unknown_as_occupied_) {
//           if (voxel_occupancy == 0) {
//             return false;
//           }
//         }
//       }
//     }

//     /**
//      * Erase all voxels that are within the current voxel block
//      */
//     volume.erase(std::remove_if(volume.begin(), volume.end(), [&](Eigen::Vector3f it_k) {
//       return (block_coordinates.x() <= it_k.x()
//           && it_k.x() < (block_coordinates.x() + se::Octree<FieldType>::blockSide)
//           && block_coordinates.y() <= it_k.y()
//           && it_k.y() < (block_coordinates.y() + se::Octree<FieldType>::blockSide)
//           && block_coordinates.z() <= it_k.z()
//           && it_k.z() < (block_coordinates.z() + se::Octree<FieldType>::blockSide));
//     }), volume.end());
//     --it_i;
//   }

//   return true;
// }


/*
template<typename FieldType>
bool CollisionCheckerM<FieldType>::isSphereCollisionFree(const Eigen::Vector3f &position_m,
                                                            const float radius_m) const {

  const Eigen::Vector3f center = (position_m / static_cast<double>(voxel_dim_)).cast<int>();
  int radius_v = static_cast<int>(radius_m / voxel_dim_); // m/(m/voxel)
  DLOG(INFO) << "voxel_dim " << voxel_dim_ << " center " << center.format(InLine);
  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;
  bool is_voxel_block;
  Eigen::Vector3f prev_pos(0, 0, 0);
  for (int x = -radius_v; x <= radius_v; x++) {
    for (int y = -radius_v; y <= radius_v; y++) {
      for (int z = -radius_v; z <= radius_v; z++) {
        Eigen::Vector3f point_offset_v(x, y, z);
        //check if point is inside the sphere radius
//        std::cout << "sphere norm " << point_offset_v.norm() <<std::endl;
        if (point_offset_v.norm() <= radius_v) {
          // check if voxelblock is allocated or only node
          Eigen::Vector3f point_v = point_offset_v + center;
          // first round
          if (node == nullptr || block == nullptr) {
            octree_ptr_->fetch_octant(point_v.x(), point_v.y(), point_v.z(), node, is_voxel_block);
            prev_pos = point_v;
            if (is_voxel_block) {
              block = static_cast<se::VoxelBlock<FieldType> *> (node);
            } else {
              if (octree_ptr_->get(se::keyops::decode(node->code_)).x >= 0.f) {
//                std::cout << " [secollision] collision at node "
//                          << se::keyops::decode(node->code_).format(InLine) << std::endl;
                return false;
              }
            }
          } else {
            // if true keep old voxelblock pointer and fetch
            // else get new voxel block
            if ((point_v.x() / BLOCK_SIDE) == (prev_pos.x() / BLOCK_SIDE)
                && (point_v.y() / BLOCK_SIDE) == (prev_pos.y() / BLOCK_SIDE)
                && (point_v.z() / BLOCK_SIDE) == (prev_pos.z() / BLOCK_SIDE)) {
              if (block->data(point_v).x >= 0.f) {
//                std::cout << " [secollision] collision at " << point_v.format(InLine) << " plog "
//                          << block->data(point_v).x << std::endl;
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
                if (block->data(point_v).x >= 0.f) {
//                  std::cout << " [secollision] collision at " << point_v.format(InLine) << " plog "
//                            << block->data(point_v).x << std::endl;
                  return false;
                }
              } else {
                block = nullptr;
                if (octree_ptr_->get(se::keyops::decode(node->code_)).x >= 0.f) {
//                  std::cout << " [secollision] collision at node "
//                            << se::keyops::decode(node->code_).format(InLine) << std::endl;
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

*/
/*
template<typename FieldType>
bool CollisionCheckerM<FieldType>::expandFlightCorridor(Path<kDim>::Ptr path_m) const {
  Eigen::Vector3f start_m = Eigen::Vector3f(-1, -1, -1);
  Eigen::Vector3f end_m = Eigen::Vector3f(-1, -1, -1);
  double step_size = 0.05;

  for (auto it_i = path_m->states.begin(); it_i != path_m->states.end(); ++it_i) {
    bool corridor_safe = true;
    int iteration = 0;
    start_m = end_m;
    end_m = (*it_i).segment_end;
    if (start_m != Eigen::Vector3f(-1, -1, -1)) {
      while (corridor_safe) {
        if (checkSegmentFlightCorridor(start_m,
                                       end_m,
                                       0,
                                       (*it_i).segment_radius + (iteration + 1) * step_size))
          ++iteration;
        else
          corridor_safe = false;
      }
    }
    (*it_i).segment_radius += iteration * step_size;
  }
}*/

//========================================================//
//                SKELETON SAMPLING INTERFACE             //
//========================================================//

template<typename FieldType>
bool CollisionCheckerM<FieldType>::isSphereSkeletonFree(const Eigen::Vector3f &position_m,
                                                          const float radius_m) const {

  isVoxelFree(position_m);
  // LOG(INFO) << "meter center "<< position_m.format(InLine) << " radius " << radius_m;

  VecVec3f shell_main_pos;
  shell_main_pos = {position_m + Eigen::Vector3f(1, 0, 0) * radius_m,
                    position_m - Eigen::Vector3f(1, 0, 0) * radius_m,
                    position_m + Eigen::Vector3f(0, 1, 0) * radius_m,
                    position_m - Eigen::Vector3f(0, 1, 0) * radius_m,
                    position_m + Eigen::Vector3f(0, 0, 1) * radius_m,
                    position_m - Eigen::Vector3f(0, 0, 1) * radius_m,};
  for (VecVec3f::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); ++it) {
    if (!isVoxelFree(*it))
      return false;
  }

  Eigen::Vector3f
      vec1_u = Eigen::Vector3f(1, 0, 0) + Eigen::Vector3f(0, 1, 0) + Eigen::Vector3f(0, 0, 1);
  vec1_u.normalize();
  Eigen::Vector3f
      vec2_u = Eigen::Vector3f(-1, 0, 0) + Eigen::Vector3f(0, 1, 0) + Eigen::Vector3f(0, 0, 1);
  vec2_u.normalize();
  Eigen::Vector3f
      vec3_u = Eigen::Vector3f(1, 0, 0) + Eigen::Vector3f(0, -1, 0) + Eigen::Vector3f(0, 0, 1);
  vec3_u.normalize();
  Eigen::Vector3f
      vec4_u = Eigen::Vector3f(-1, 0, 0) + Eigen::Vector3f(0, -1, 0) + Eigen::Vector3f(0, 0, 1);
  vec4_u.normalize();

  VecVec3f shell_sub_pos;
  shell_sub_pos = {position_m + vec1_u * radius_m, position_m - vec1_u * radius_m,
                   position_m + vec2_u * radius_m, position_m - vec2_u * radius_m,
                   position_m + vec3_u * radius_m, position_m - vec3_u * radius_m,
                   position_m + vec4_u * radius_m, position_m - vec4_u * radius_m};
  for (VecVec3f::iterator it = shell_sub_pos.begin(); it != shell_sub_pos.end();
       ++it) {
    if (!isVoxelFree(*it))
      return false;
  }

  return true;
}

//   !!! took code snippets from DubinsSateSpace MotionValidator
template<typename FieldType>
bool CollisionCheckerM<FieldType>::isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3f &start_m,
                                                                         const Eigen::Vector3f &end_m,
                                                                         const float r_min_m,
                                                                         const float r_max_m)
                                                                         const {

   if (start_m == end_m)
    return true;
  /**
   * Set up the line-of-sight connection
   */
  const float seg_prec = planning_params_.skeleton_sample_precision; //
  // Precision at which
  // to sample the connection [m/voxel]

  VecVec3f segment_flight_corridor;
  const Eigen::Vector3f corridor_axis_u = (end_m - start_m) / (end_m - start_m).norm();
  const Eigen::Vector3f start_corridor_m = start_m - r_max_m * corridor_axis_u;
  const Eigen::Vector3f end_corridor_m = end_m + r_max_m * corridor_axis_u;

  const Eigen::Vector3f vec_seg_connection_m =
      end_corridor_m - start_corridor_m; // The vector in [m] connecting the start and end position
  const Eigen::Vector3f vec_seg_direction_u = vec_seg_connection_m
      / vec_seg_connection_m.norm(); // The vector in [m] connecting the start and end position
  const int num_axial_subpos =
      vec_seg_connection_m.norm() / seg_prec; // Number of sub points along the line to be checked

  if (!isSphereSkeletonFree(end_m, r_max_m)) {
    return false;
  }

  if (!isLineFree(start_corridor_m,
                 vec_seg_connection_m,
                 num_axial_subpos)) // Check if the line-of-sight connection is collision free
    return false;

  /**
   * Get cylinder extrema in horizontal and vertical direction
   */
  Eigen::Vector3f vec_z_u = Eigen::Vector3f(0, 0, 1);
  Eigen::Vector3f vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  Eigen::Vector3f vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3f(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3f(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3f(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }

  VecVec3f shell_main_pos;
  shell_main_pos =
      {start_corridor_m + vec_horizontal_u * r_max_m, start_corridor_m - vec_horizontal_u * r_max_m,
       start_corridor_m + vec_vertical_u * r_max_m, start_corridor_m - vec_vertical_u * r_max_m};
  for (VecVec3f::iterator it = shell_main_pos.begin();
       it != shell_main_pos.end(); ++it) {
    if (!isLineFree(*it, vec_seg_connection_m, num_axial_subpos))
      return false;
  }

  const int num_circ_shell_subpos = r_max_m * M_PI / (2 * seg_prec);

  if (num_circ_shell_subpos > 1) {
    AlignedQueueTupleVec3f circ_shell_pos; // Not sure yet
    circ_shell_pos.push(std::make_tuple(vec_vertical_u,
                                        vec_horizontal_u,
                                        1,
                                        num_circ_shell_subpos - 1));

    while (!circ_shell_pos.empty()) {
      std::tuple<Eigen::Vector3f, Eigen::Vector3f, int, int> x = circ_shell_pos.front();
      int mid = (std::get<2>(x) + std::get<3>(x)) / 2;
      Eigen::Vector3f vec1_u = (std::get<0>(x) + std::get<1>(x)) / 2;
      vec1_u.normalize();
      Eigen::Vector3f vec2_u = (std::get<0>(x) - std::get<1>(x)) / 2;
      vec2_u.normalize();

      VecVec3f circ_shell_sub_pos;
      circ_shell_sub_pos =
          {start_corridor_m + vec1_u * r_max_m, start_corridor_m - vec1_u * r_max_m,
           start_corridor_m + vec2_u * r_max_m, start_corridor_m - vec2_u * r_max_m};
      for (VecVec3f::iterator it = circ_shell_sub_pos.begin();
           it != circ_shell_sub_pos.end(); ++it) {
        if (!isLineFree(*it, vec_seg_connection_m, num_axial_subpos))
          return false;
      }

      circ_shell_pos.pop();

      if (std::get<2>(x) < mid)
        circ_shell_pos.push(std::make_tuple(std::get<0>(x), vec1_u, std::get<2>(x), mid - 1));
      if (std::get<3>(x) > mid)
        circ_shell_pos.push(std::make_tuple(vec1_u, std::get<1>(x), mid + 1, std::get<3>(x)));
    }
  }

  const int num_radial_subpos = r_max_m / seg_prec;

  if (num_radial_subpos > 1) {
    std::queue<std::pair<int, int>> radial_pos; // Not sure yet
    radial_pos.push(std::make_pair(1, num_radial_subpos - 1));

    while (!radial_pos.empty()) {
      std::pair<int, int> y = radial_pos.front();
      int mid = (y.first + y.second) / 2;
      float r_m = (float) mid / (float) num_radial_subpos * r_max_m;

      VecVec3f circ_main_pos;
      circ_main_pos =
          {start_corridor_m + vec_horizontal_u * r_m, start_corridor_m - vec_horizontal_u * r_m,
           start_corridor_m + vec_vertical_u * r_m, start_corridor_m - vec_vertical_u * r_m};
      for (VecVec3f::iterator it = circ_main_pos.begin();
           it != circ_main_pos.end(); ++it) {
        if (!isLineFree(*it, vec_seg_connection_m, num_axial_subpos))
          return false;
      }

      int num_circ_subpos = r_m * M_PI / (2 * seg_prec);

      if (num_circ_subpos > 1) {
       AlignedQueueTupleVec3f circ_sub_pos; // Not sure yet
        circ_sub_pos.push(std::make_tuple(vec_vertical_u,
                                          vec_horizontal_u,
                                          1,
                                          num_circ_subpos - 1));

        while (!circ_sub_pos.empty()) {
          std::tuple<Eigen::Vector3f, Eigen::Vector3f, int, int> x = circ_sub_pos.front();
          int mid = (std::get<2>(x) + std::get<3>(x)) / 2;
          Eigen::Vector3f vec1_u = (std::get<0>(x) + std::get<1>(x)) / 2;
          vec1_u.normalize();
          Eigen::Vector3f vec2_u = (std::get<0>(x) - std::get<1>(x)) / 2;
          vec2_u.normalize();

          VecVec3f sub_starts;
          sub_starts = {start_corridor_m + vec1_u * r_max_m, start_corridor_m - vec1_u * r_max_m,
                        start_corridor_m + vec2_u * r_max_m, start_corridor_m - vec2_u * r_max_m};
          for (VecVec3f::iterator it = sub_starts.begin();
               it != sub_starts.end(); ++it) {
            if (!isLineFree(*it, vec_seg_connection_m, num_axial_subpos))
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

/*
template<typename FieldType>
bool CollisionCheckerM<FieldType>::expandFlightCorridorSkeleton(Path<kDim>::Ptr path_m) const {
  Eigen::Vector3f start_m = Eigen::Vector3f(-1, -1, -1);
  Eigen::Vector3f end_m = Eigen::Vector3f(-1, -1, -1);
  double step_size = 0.05;

  for (auto it_i = path_m->states.begin(); it_i != path_m->states.end(); ++it_i) {
    bool corridor_safe = true;
    int iteration = 0;
    start_m = end_m;
    end_m = (*it_i).segment_end;
    if (start_m != Eigen::Vector3f(-1, -1, -1)) {
      while (corridor_safe) {
        if (checkSegmentFlightCorridorSkeleton(start_m,
                                               end_m,
                                               0,
                                               (*it_i).segment_radius
                                                   + (iteration + 1) * step_size))
          ++iteration;
        else
          corridor_safe = false;
      }
    }
    (*it_i).segment_radius += iteration * step_size;
    std::cout << "Segment Radius: " << (*it_i).segment_radius << std::endl;
  }
}

//========================================================//
//                DISTANCE SAMPLING INTERFACE             //
//========================================================//
template<typename FieldType>
bool CollisionCheckerM<FieldType>::checkVoxelDistance(const Eigen::Vector3f &position_m,
                                                         float radius_m) const {

  // Convert start and direction from [m] to [voxel]
  Eigen::Vector3f position_v = (position_m / voxel_dim_).cast<int>();

  // Fetch voxel block that contains the voxel position
  se::VoxelBlock<FieldType>
      *voxel_block = octree_ptr_->fetch(position_v.x(), position_v.y(), position_v.z());

  // Check if the voxel blocks containing the current voxel position is not allocated, i.e. unseen
  // Opposite does not imply that the start and end position are seen!
  if (treat_unknown_as_occupied_) {
    if (voxel_block == NULL) {
      return false;
    }
  } else {
    if (voxel_block == NULL) {
      return true;
    }
  }

  double voxel_distance =
      voxel_block->data(Eigen::Vector3f(position_v.x(), position_v.y(), position_v.z())).y;

  if (voxel_distance < radius_m) {
    return false;
  }

  return true;
}

template<typename FieldType>
bool CollisionCheckerM<FieldType>::checkLineDistance(const Eigen::Vector3f &start_m,
                                                        const Eigen::Vector3f &end_m,
                                                        float radius_m) const {

  // Set up the line-of-sight connection
  double seg_prec = voxel_dim_; // Precision at which to sample the connection [m/voxel]
  Eigen::Vector3f vec_seg_connection_m =
      end_m - start_m; // The vector in [m] connecting the start and end position
  int num_subpos =
      vec_seg_connection_m.norm() / seg_prec; // Number of sub points along the line to be checked

  if (num_subpos == 0) num_subpos = 1;
  std::queue<std::pair<int, int>> line_pos;
  line_pos.push(std::make_pair(1, num_subpos - 1));

  // repeatedly subdivide the path segment in the middle (and check the middle)
  while (!line_pos.empty()) {
    std::pair<int, int> x = line_pos.front();

    int mid = (x.first + x.second) / 2;

    // Compute midpoint
    Eigen::Vector3f
        position_m = ((double) mid / (double) num_subpos * vec_seg_connection_m + start_m);

    if (!checkVoxelDistance(position_m, radius_m)) {
      return false;
    }

    line_pos.pop();

    if (x.first < mid)
      line_pos.push(std::make_pair(x.first, mid - 1));
    if (x.second > mid)
      line_pos.push(std::make_pair(mid + 1, x.second));
  }

  return true;
}

template<typename FieldType>
void CollisionCheckerM<FieldType>::findMinLineDistance(const Eigen::Vector3f &start_m,
                                                          const Eigen::Vector3f &end_m,
                                                          double &min_distance_m) const {

  // Set up the line-of-sight connection
  double seg_prec = voxel_dim_; // Precision at which to sample the connection [m/voxel]
  Eigen::Vector3f vec_seg_connection_m =
      end_m - start_m; // The vector in [m] connecting the start and end position
  int n_subpos = vec_seg_connection_m.norm() / seg_prec
      + 1; // Number of sub points along the line to be checked
  min_distance_m = 2;
  double distance_m;
  for (int subpos_idx = 0; subpos_idx < n_subpos; subpos_idx++) {
    Eigen::Vector3f
        position_m = ((double) subpos_idx / (double) n_subpos * vec_seg_connection_m + start_m);
    getVoxelDistance(position_m, distance_m);
    if (distance_m < min_distance_m) min_distance_m = distance_m;
  }
}

template<typename FieldType>
void CollisionCheckerM<FieldType>::getVoxelDistance(const Eigen::Vector3f &position_m,
                                                       double &distance) const  {

  // Convert start and direction from [m] to [voxel]
  Eigen::Vector3f position_v = position_m / voxel_dim_;

  // Fetch voxel block that contains the voxel position
  se::VoxelBlock<FieldType>
      *voxel_block = octree_ptr_->fetch(position_v.x(), position_v.y(), position_v.z());

  // Check if the voxel blocks containing the current voxel position is not allocated, i.e. unseen
  // Opposite does not imply that the start and end position are seen!
  if (voxel_block == NULL) {
    distance = 0;
  } else {
    distance = voxel_block->data(Eigen::Vector3f(position_v.x(), position_v.y(), position_v.z())).y;
  }
}

template<typename FieldType>
double CollisionCheckerM<FieldType>::getVoxelDistance(const Eigen::Vector3f &position_m) const {

  double distance;
  getVoxelDistance(position_m, distance);
  return distance;

}

template<typename FieldType>
bool CollisionCheckerM<FieldType>::expandFlightCorridorDistance(Path<kDim>::Ptr path_m)const  {

  Eigen::Vector3f start_m = Eigen::Vector3f(-1, -1, -1);
  Eigen::Vector3f end_m = Eigen::Vector3f(-1, -1, -1);

  for (auto it_i = path_m->states.begin(); it_i != path_m->states.end(); ++it_i) {
    start_m = end_m;
    end_m = (*it_i).segment_end;
    double min_distance;
    if (start_m != Eigen::Vector3f(-1, -1, -1)) {
      findMinLineDistance(start_m, end_m, min_distance);
    }

    if (ompl_params_.guarante_all_control_pts_save_) {
      double distance_start_m;
      double distance_end_m;
      double distance_reduction = 0.75 * ompl_params_.min_control_point_radius_;

      getVoxelDistance(start_m, distance_start_m);
      getVoxelDistance(end_m, distance_end_m);

      double safety_distance = distance_start_m - distance_reduction;
      if (safety_distance < min_distance)
        min_distance = safety_distance;
      safety_distance = distance_end_m - distance_reduction;
      if (safety_distance < min_distance)
        min_distance = safety_distance;
    }

    (*it_i).segment_radius = min_distance;
  }
}
*/


} // namespace exploration
}// namespace se
#endif //EXPLORATION_CollisionCheckerM_HPP
