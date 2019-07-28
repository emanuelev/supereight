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


#ifndef EXPLORATION_PROBCOLLISIONCHECKER_HPP
#define EXPLORATION_PROBCOLLISIONCHECKER_HPP

#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>
#include <math.h>
#include <vector>

#include "se/utils/support_structs.hpp"
#include "se/occupancy_world.hpp"
#include "volume_shifter.hpp"
#include "se/planning_parameter.hpp"

#include "se/octree.hpp"
//#include "FoldedStatus.h"

//#include "state_uncertanty.hpp"
namespace se {
namespace exploration {
template<typename FieldType>
class ProbCollisionChecker {
 public:
  typedef std::shared_ptr<ProbCollisionChecker<FieldType> > Ptr;

  ProbCollisionChecker(const Octree<FieldType> &octree, const PlanningParameter &ompl_params);

  bool checkSegmentFlightCorridor(const Eigen::Vector3d start,
                                  const Eigen::Vector3d end,
                                  const double r_min,
                                  const double r_max);
  bool checkSegmentFlightCorridorSkeleton(const Eigen::Vector3d start_m,
                                          const Eigen::Vector3d end_m,
                                          const double r_min_m,
                                          const double r_max_m);
  bool checkSphere(const Eigen::Vector3d position_m, const double radius_m);
  bool checkSphereSkeleton(const Eigen::Vector3d position_m, const double radius_m);
  bool checkVolume(std::vector<Eigen::Vector3i> volume);
  bool checkLine(const Eigen::Vector3d start_m,
                 const Eigen::Vector3d connection_m,
                 const int num_subpos);
  bool checkVoxel(const Eigen::Vector3d position_m);
  bool checkLineDistance(const Eigen::Vector3d start_m,
                         const Eigen::Vector3d end_m,
                         double radius_m);
  bool checkVoxelDistance(const Eigen::Vector3d position_m, double radius_m);
  bool expandFlightCorridorDistance(Path<kDim>::Ptr path_m);
  bool expandFlightCorridorSkeleton(Path<kDim>::Ptr path_m);
  bool expandFlightCorridor(Path<kDim>::Ptr path_m);
  void findMinLineDistance(const Eigen::Vector3d start_m,
                           const Eigen::Vector3d end_m,
                           double &min_distance_m);
  void getVoxelDistance(const Eigen::Vector3d position_m, double &distance);
  double getVoxelDistance(const Eigen::Vector3d position_m);

 private:
  exploration::VolumeShifter::Ptr vs_ = NULL;
//  std::shared_ptr<OccupancyWorld> ow_ = NULL;
//  std::shared_ptr<se::Octree < FieldType>> octree_ = NULL;
  Octree<FieldType> octree_;
  // from occupancy world
  std::string id_;
  std::vector<key_t> alloc_list_;

  PlanningParameter ompl_params_;

  float voxel_dim_;
  bool treat_unknown_as_occupied_;
};

template<typename FieldType>
ProbCollisionChecker<FieldType>::ProbCollisionChecker(const Octree<FieldType> &octree,
                                                      const PlanningParameter &ompl_params)
    :
    octree_(octree),
    treat_unknown_as_occupied_(ompl_params.treat_unknown_as_occupied_),
    voxel_dim_(octree.voxelDim()),
    ompl_params_(ompl_params) {
  vs_ = std::unique_ptr<VolumeShifter>(new VolumeShifter(voxel_dim_,
                                                                      ompl_params.volume_precision_factor_));
}

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkVoxel(const Eigen::Vector3d position_m) {

  /**
   * Convert position from [m] to [voxel]
   */
  Eigen::Vector3i position_v = (position_m / voxel_dim_).cast<int>();

  se::VoxelBlock<FieldType>
      *voxel_block = octree_.fetch(position_v.x(), position_v.y(), position_v.z());

  /**
   * Check if the voxel blocks containing the current voxel position is not allocated, i.e. unseen
   */
  if (voxel_block != nullptr) {

    double voxel_occupancy =
        voxel_block->data(Eigen::Vector3i(position_v.x(), position_v.y(), position_v.z())).x;

    if (voxel_occupancy > 0.5) { // Surface boundary at 0.5 occupancy
      return false;
    }

    if (treat_unknown_as_occupied_) {
      if (voxel_occupancy == 0) {
        return false;
      }
    }
    return true;
  } else {
    return !treat_unknown_as_occupied_;
  }
}

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkLine(const Eigen::Vector3d start_m,
                                                const Eigen::Vector3d connection_m,
                                                const int num_subpos) {

  std::queue<std::pair<int, int>> line_pos;
  line_pos.push(std::make_pair(1, num_subpos - 1));

  // Repeatedly subdivide the path segment in the middle (and check the middle)
  while (!line_pos.empty()) {
    std::pair<int, int> x = line_pos.front();

    int mid = (x.first + x.second) / 2;

    // Compute midpoint
    Eigen::Vector3d position_m = ((double) mid / (double) num_subpos * connection_m + start_m);

    if (!checkVoxel(position_m))
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
template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkVolume(std::vector<Eigen::Vector3i> volume) {

  /**
   * Iterate through whole voxel vector
   */
  for (auto it_i = volume.begin(); it_i != volume.end(); ++it_i) {

    /**
     * Check all voxels that are within the current voxel block
     */
    se::VoxelBlock<FieldType> *voxel_block = octree_.fetch((*it_i).x(), (*it_i).y(), (*it_i).z());

    if (treat_unknown_as_occupied_ && voxel_block == nullptr) {
      return false;
    }

    Eigen::Vector3i block_coordinates = voxel_block->coordinates();

    for (std::vector<Eigen::Vector3i>::iterator it_j = volume.begin(); it_j != volume.end();
         ++it_j) {
      if (block_coordinates.x() <= (*it_j).x()
          && (*it_j).x() < (block_coordinates.x() + se::Octree<FieldType>::blockSide)
          && block_coordinates.y() <= (*it_j).y()
          && (*it_j).y() < (block_coordinates.y() + se::Octree<FieldType>::blockSide)
          && block_coordinates.z() <= (*it_j).z()
          && (*it_j).z() < (block_coordinates.z() + se::Octree<FieldType>::blockSide)) {

        float voxel_occupancy = voxel_block->data(*it_j).x;
        if (voxel_occupancy > 0.5) { // Surface boundary at 0.5 occupancy
          return false;
        }
        if (treat_unknown_as_occupied_) {
          if (voxel_occupancy == 0) {
            return false;
          }
        }
      }
    }

    /**
     * Erase all voxels that are within the current voxel block
     */
    volume.erase(std::remove_if(volume.begin(), volume.end(), [&](Eigen::Vector3i it_k) {
      return (block_coordinates.x() <= it_k.x()
          && it_k.x() < (block_coordinates.x() + se::Octree<FieldType>::blockSide)
          && block_coordinates.y() <= it_k.y()
          && it_k.y() < (block_coordinates.y() + se::Octree<FieldType>::blockSide)
          && block_coordinates.z() <= it_k.z()
          && it_k.z() < (block_coordinates.z() + se::Octree<FieldType>::blockSide));
    }), volume.end());
    --it_i;
  }

  return true;
}

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkSphere(const Eigen::Vector3d position_m,
                                                  const double radius_m) {

  /**
   * Sample sphere with volume shifter
   */
  std::vector<Eigen::Vector3i> sphere_volume;
  if (!vs_->generateSphereVolume(sphere_volume, position_m, radius_m)) {
    return false;
  }

  /**
   * Check sphere volume for collision
   */
  if (!checkVolume(sphere_volume)) {
    return false;
  }

  return true;
}

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkSegmentFlightCorridor(const Eigen::Vector3d start_m,
                                                                 const Eigen::Vector3d end_m,
                                                                 const double r_min,
                                                                 const double r_max) {

  /**
   * Check if the start and end position are in the allocated map
   * should not be able to plan out side as end position is a pose in the free space
   */

/*  if (!ow_->InMapBoundsMeter(start_m)) {
    LOG(WARNING) << "Start position is outside the map";
    return false;
  }

  if (!ow_->InMapBoundsMeter(end_m)) {
    LOG(WARNING) << "End position is outside the map";
    return false;
  }*/

  /**
   * Check if the voxel blocks containing the start and end position are not allocated, i.e. unseen
   * Opposite does not imply that the start and end position are seen!
   */
//  if (treat_unknown_as_occupied_) {
//    if (!ow_->isVoxelBlockAllocatedMeter(start_m)) {
//      LOG(WARNING) << "Start position is not allocated";
//      return false;
//    }
//
//    if (!ow_->isVoxelBlockAllocatedMeter(end_m)) {
//      LOG(WARNING) << "End position is not allocated";
//      return false;
//    }
//  }

  /**
   * Check if the start and end position fullfill the requirements of the given flight corridor radius
   * TODO: Check if checkSphere works for radius 0
   */
  if (!checkSphere(start_m, r_max)) {
    return false;
  }

  if (!checkSphere(end_m, r_max)) {
    return false;
  }

  /**
   * Generate std::vector containing all voxels (Eigen::Vector3i) to checked that are within the flight corridor
   */
  std::vector<Eigen::Vector3i> segment_flight_corridor;
  Eigen::Vector3d corridor_axis_u = (end_m - start_m) / (end_m - start_m).norm();
  Eigen::Vector3d start_corridor_m = start_m - r_max * corridor_axis_u;
  Eigen::Vector3d end_corridor_m = end_m + r_max * corridor_axis_u;
  if (!vs_->generateSegmentFlightCorridor(segment_flight_corridor,
                                          start_corridor_m,
                                          end_corridor_m,
                                          r_min,
                                          r_max)) {
    return false;
  }

  /**
   * Check the flight corridor for collision
   */
  if (!checkVolume(segment_flight_corridor)) {
    return false;
  }

  return true;
}

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::expandFlightCorridor(Path<kDim>::Ptr path_m) {
  Eigen::Vector3d start_m = Eigen::Vector3d(-1, -1, -1);
  Eigen::Vector3d end_m = Eigen::Vector3d(-1, -1, -1);
  double step_size = 0.05;

  for (std::vector<State<kDim>>::iterator it_i = path_m->states.begin();
       it_i != path_m->states.end(); ++it_i) {
    bool corridor_safe = true;
    int iteration = 0;
    start_m = end_m;
    end_m = (*it_i).segment_end;
    if (start_m != Eigen::Vector3d(-1, -1, -1)) {
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
}

//========================================================//
//                SKELETON SAMPLING INTERFACE             //
//========================================================//

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkSphereSkeleton(const Eigen::Vector3d position_m,
                                                          const double radius_m) {

  checkVoxel(position_m);

  std::vector<Eigen::Vector3d> shell_main_pos;
  shell_main_pos = {position_m + Eigen::Vector3d(1, 0, 0) * radius_m,
                    position_m - Eigen::Vector3d(1, 0, 0) * radius_m,
                    position_m + Eigen::Vector3d(0, 1, 0) * radius_m,
                    position_m - Eigen::Vector3d(0, 1, 0) * radius_m,
                    position_m + Eigen::Vector3d(0, 0, 1) * radius_m,
                    position_m - Eigen::Vector3d(0, 0, 1) * radius_m,};
  for (std::vector<Eigen::Vector3d>::iterator it = shell_main_pos.begin();
       it != shell_main_pos.end(); ++it) {
    if (!checkVoxel(*it))
      return false;
  }

  Eigen::Vector3d
      vec1_u = Eigen::Vector3d(1, 0, 0) + Eigen::Vector3d(0, 1, 0) + Eigen::Vector3d(0, 0, 1);
  vec1_u.normalize();
  Eigen::Vector3d
      vec2_u = Eigen::Vector3d(-1, 0, 0) + Eigen::Vector3d(0, 1, 0) + Eigen::Vector3d(0, 0, 1);
  vec2_u.normalize();
  Eigen::Vector3d
      vec3_u = Eigen::Vector3d(1, 0, 0) + Eigen::Vector3d(0, -1, 0) + Eigen::Vector3d(0, 0, 1);
  vec3_u.normalize();
  Eigen::Vector3d
      vec4_u = Eigen::Vector3d(-1, 0, 0) + Eigen::Vector3d(0, -1, 0) + Eigen::Vector3d(0, 0, 1);
  vec4_u.normalize();

  std::vector<Eigen::Vector3d> shell_sub_pos;
  shell_sub_pos = {position_m + vec1_u * radius_m, position_m - vec1_u * radius_m,
                   position_m + vec2_u * radius_m, position_m - vec2_u * radius_m,
                   position_m + vec3_u * radius_m, position_m - vec3_u * radius_m,
                   position_m + vec4_u * radius_m, position_m - vec4_u * radius_m};
  for (std::vector<Eigen::Vector3d>::iterator it = shell_sub_pos.begin(); it != shell_sub_pos.end();
       ++it) {
    if (!checkVoxel(*it))
      return false;
  }

  return true;
}

//   !!! took code snippets from DubinsSateSpace MotionValidator
template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkSegmentFlightCorridorSkeleton(const Eigen::Vector3d start_m,
                                                                         const Eigen::Vector3d end_m,
                                                                         const double r_min_m,
                                                                         const double r_max_m) {

  /**
   * Set up the line-of-sight connection
   */
  double seg_prec = ompl_params_.skeleton_sample_precision_; //
  // Precision at which
  // to sample the connection [m/voxel]

  std::vector<Eigen::Vector3i> segment_flight_corridor;
  Eigen::Vector3d corridor_axis_u = (end_m - start_m) / (end_m - start_m).norm();
  Eigen::Vector3d start_corridor_m = start_m - r_max_m * corridor_axis_u;
  Eigen::Vector3d end_corridor_m = end_m + r_max_m * corridor_axis_u;

  Eigen::Vector3d vec_seg_connection_m =
      end_corridor_m - start_corridor_m; // The vector in [m] connecting the start and end position
  Eigen::Vector3d vec_seg_direction_u = vec_seg_connection_m
      / vec_seg_connection_m.norm(); // The vector in [m] connecting the start and end position
  int num_axial_subpos =
      vec_seg_connection_m.norm() / seg_prec; // Number of sub points along the line to be checked

  if (!checkSphereSkeleton(end_m, r_max_m)) {
    return false;
  }

  if (!checkLine(start_corridor_m,
                 vec_seg_connection_m,
                 num_axial_subpos)) // Check if the line-of-sight connection is collision free
    return false;

  /**
   * Get cylinder extrema in horizontal and vertical direction
   */
  Eigen::Vector3d vec_z_u = Eigen::Vector3d(0, 0, 1);
  Eigen::Vector3d vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  Eigen::Vector3d vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3d(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3d(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3d(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }

  std::vector<Eigen::Vector3d> shell_main_pos;
  shell_main_pos =
      {start_corridor_m + vec_horizontal_u * r_max_m, start_corridor_m - vec_horizontal_u * r_max_m,
       start_corridor_m + vec_vertical_u * r_max_m, start_corridor_m - vec_vertical_u * r_max_m};
  for (std::vector<Eigen::Vector3d>::iterator it = shell_main_pos.begin();
       it != shell_main_pos.end(); ++it) {
    if (!checkLine(*it, vec_seg_connection_m, num_axial_subpos))
      return false;
  }

  int num_circ_shell_subpos = r_max_m * M_PI / (2 * seg_prec);

  if (num_circ_shell_subpos > 1) {
    std::queue<std::tuple<Eigen::Vector3d, Eigen::Vector3d, int, int>>
        circ_shell_pos; // Not sure yet
    circ_shell_pos.push(std::make_tuple(vec_vertical_u,
                                        vec_horizontal_u,
                                        1,
                                        num_circ_shell_subpos - 1));

    while (!circ_shell_pos.empty()) {
      std::tuple<Eigen::Vector3d, Eigen::Vector3d, int, int> x = circ_shell_pos.front();
      int mid = (std::get<2>(x) + std::get<3>(x)) / 2;
      Eigen::Vector3d vec1_u = (std::get<0>(x) + std::get<1>(x)) / 2;
      vec1_u.normalize();
      Eigen::Vector3d vec2_u = (std::get<0>(x) - std::get<1>(x)) / 2;
      vec2_u.normalize();

      std::vector<Eigen::Vector3d> circ_shell_sub_pos;
      circ_shell_sub_pos =
          {start_corridor_m + vec1_u * r_max_m, start_corridor_m - vec1_u * r_max_m,
           start_corridor_m + vec2_u * r_max_m, start_corridor_m - vec2_u * r_max_m};
      for (std::vector<Eigen::Vector3d>::iterator it = circ_shell_sub_pos.begin();
           it != circ_shell_sub_pos.end(); ++it) {
        if (!checkLine(*it, vec_seg_connection_m, num_axial_subpos))
          return false;
      }

      circ_shell_pos.pop();

      if (std::get<2>(x) < mid)
        circ_shell_pos.push(std::make_tuple(std::get<0>(x), vec1_u, std::get<2>(x), mid - 1));
      if (std::get<3>(x) > mid)
        circ_shell_pos.push(std::make_tuple(vec1_u, std::get<1>(x), mid + 1, std::get<3>(x)));
    }
  }

  int num_radial_subpos = r_max_m / seg_prec;

  if (num_radial_subpos > 1) {
    std::queue<std::pair<int, int>> radial_pos; // Not sure yet
    radial_pos.push(std::make_pair(1, num_radial_subpos - 1));

    while (!radial_pos.empty()) {
      std::pair<int, int> y = radial_pos.front();
      int mid = (y.first + y.second) / 2;
      double r_m = (double) mid / (double) num_radial_subpos * r_max_m;

      std::vector<Eigen::Vector3d> circ_main_pos;
      circ_main_pos =
          {start_corridor_m + vec_horizontal_u * r_m, start_corridor_m - vec_horizontal_u * r_m,
           start_corridor_m + vec_vertical_u * r_m, start_corridor_m - vec_vertical_u * r_m};
      for (std::vector<Eigen::Vector3d>::iterator it = circ_main_pos.begin();
           it != circ_main_pos.end(); ++it) {
        if (!checkLine(*it, vec_seg_connection_m, num_axial_subpos))
          return false;
      }

      int num_circ_subpos = r_m * M_PI / (2 * seg_prec);

      if (num_circ_subpos > 1) {
        std::queue<std::tuple<Eigen::Vector3d, Eigen::Vector3d, int, int>>
            circ_sub_pos; // Not sure yet
        circ_sub_pos.push(std::make_tuple(vec_vertical_u,
                                          vec_horizontal_u,
                                          1,
                                          num_circ_subpos - 1));

        while (!circ_sub_pos.empty()) {
          std::tuple<Eigen::Vector3d, Eigen::Vector3d, int, int> x = circ_sub_pos.front();
          int mid = (std::get<2>(x) + std::get<3>(x)) / 2;
          Eigen::Vector3d vec1_u = (std::get<0>(x) + std::get<1>(x)) / 2;
          vec1_u.normalize();
          Eigen::Vector3d vec2_u = (std::get<0>(x) - std::get<1>(x)) / 2;
          vec2_u.normalize();

          std::vector<Eigen::Vector3d> sub_starts;
          sub_starts = {start_corridor_m + vec1_u * r_max_m, start_corridor_m - vec1_u * r_max_m,
                        start_corridor_m + vec2_u * r_max_m, start_corridor_m - vec2_u * r_max_m};
          for (std::vector<Eigen::Vector3d>::iterator it = sub_starts.begin();
               it != sub_starts.end(); ++it) {
            if (!checkLine(*it, vec_seg_connection_m, num_axial_subpos))
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
bool ProbCollisionChecker<FieldType>::expandFlightCorridorSkeleton(Path<kDim>::Ptr path_m) {
  Eigen::Vector3d start_m = Eigen::Vector3d(-1, -1, -1);
  Eigen::Vector3d end_m = Eigen::Vector3d(-1, -1, -1);
  double step_size = 0.05;

  for (std::vector<State<kDim>>::iterator it_i = path_m->states.begin();
       it_i != path_m->states.end(); ++it_i) {
    bool corridor_safe = true;
    int iteration = 0;
    start_m = end_m;
    end_m = (*it_i).segment_end;
    if (start_m != Eigen::Vector3d(-1, -1, -1)) {
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
bool ProbCollisionChecker<FieldType>::checkVoxelDistance(const Eigen::Vector3d position_m,
                                                         double radius_m) {

  // Convert start and direction from [m] to [voxel]
  Eigen::Vector3i position_v = (position_m / voxel_dim_).cast<int>();

  // Fetch voxel block that contains the voxel position
  se::VoxelBlock<FieldType>
      *voxel_block = octree_.fetch(position_v.x(), position_v.y(), position_v.z());

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
      voxel_block->data(Eigen::Vector3i(position_v.x(), position_v.y(), position_v.z())).y;

  if (voxel_distance < radius_m) {
    return false;
  }

  return true;
}

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::checkLineDistance(const Eigen::Vector3d start_m,
                                                        const Eigen::Vector3d end_m,
                                                        double radius_m) {

  // Set up the line-of-sight connection
  double seg_prec = voxel_dim_; // Precision at which to sample the connection [m/voxel]
  Eigen::Vector3d vec_seg_connection_m =
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
    Eigen::Vector3d
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
void ProbCollisionChecker<FieldType>::findMinLineDistance(const Eigen::Vector3d start_m,
                                                          const Eigen::Vector3d end_m,
                                                          double &min_distance_m) {

  // Set up the line-of-sight connection
  double seg_prec = voxel_dim_; // Precision at which to sample the connection [m/voxel]
  Eigen::Vector3d vec_seg_connection_m =
      end_m - start_m; // The vector in [m] connecting the start and end position
  int n_subpos = vec_seg_connection_m.norm() / seg_prec
      + 1; // Number of sub points along the line to be checked
  min_distance_m = 2;
  double distance_m;
  for (int subpos_idx = 0; subpos_idx < n_subpos; subpos_idx++) {
    Eigen::Vector3d
        position_m = ((double) subpos_idx / (double) n_subpos * vec_seg_connection_m + start_m);
    getVoxelDistance(position_m, distance_m);
    if (distance_m < min_distance_m) min_distance_m = distance_m;
  }
}

template<typename FieldType>
void ProbCollisionChecker<FieldType>::getVoxelDistance(const Eigen::Vector3d position_m,
                                                       double &distance) {

  // Convert start and direction from [m] to [voxel]
  Eigen::Vector3d position_v = position_m / voxel_dim_;

  // Fetch voxel block that contains the voxel position
  se::VoxelBlock<FieldType>
      *voxel_block = octree_.fetch(position_v.x(), position_v.y(), position_v.z());

  // Check if the voxel blocks containing the current voxel position is not allocated, i.e. unseen
  // Opposite does not imply that the start and end position are seen!
  if (voxel_block == NULL) {
    distance = 0;
  } else {
    distance = voxel_block->data(Eigen::Vector3i(position_v.x(), position_v.y(), position_v.z())).y;
  }
}

template<typename FieldType>
double ProbCollisionChecker<FieldType>::getVoxelDistance(const Eigen::Vector3d position_m) {

  double distance;
  getVoxelDistance(position_m, distance);
  return distance;

}

template<typename FieldType>
bool ProbCollisionChecker<FieldType>::expandFlightCorridorDistance(Path<kDim>::Ptr path_m) {

  Eigen::Vector3d start_m = Eigen::Vector3d(-1, -1, -1);
  Eigen::Vector3d end_m = Eigen::Vector3d(-1, -1, -1);

  for (std::vector<State<kDim>>::iterator it_i = path_m->states.begin();
       it_i != path_m->states.end(); ++it_i) {
    start_m = end_m;
    end_m = (*it_i).segment_end;
    double min_distance;
    if (start_m != Eigen::Vector3d(-1, -1, -1)) {
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
} // namespace exploration
}// namespace se
#endif //EXPLORATION_PROBCOLLISIONCHECKER_HPP
