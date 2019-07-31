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

  CollisionCheckerV(const std::shared_ptr<Octree<FieldType> > &octree_ptr,
                       const PlanningParameter &ompl_params);

  bool checkSegmentFlightCorridor(const Eigen::Vector3i &start,
                                  const Eigen::Vector3i &end,
                                  const int r_min,
                                  const int r_max); // motion checker
  bool checkSegmentFlightCorridorSkeleton(const Eigen::Vector3i &start,
                                          const Eigen::Vector3i &end,
                                          const int r_min,
                                          const int r_max);
  bool isSphereCollisionFree(const Eigen::Vector3i &position_v, const int radius_v); // start end
  // goal
  bool checkVolume(VectorVec3i volume); // checkou flight segment corridor
  bool checkLine(const Eigen::Vector3i &start,
                 const Eigen::Vector3i &connection,
                 const int num_subpos); // check segment fligh corridor
  bool checkVoxel(const Eigen::Vector3i &position);
  bool checkLineDistance(const Eigen::Vector3i &start,
                         const Eigen::Vector3i &end,
                         int radius); // motion validator
  bool checkVoxelDistance(const Eigen::Vector3i &position, int radius); //motion validator


  bool expandFlightCorridorDistance(Path<kDim>::Ptr path_m);
  bool expandFlightCorridorSkeleton(Path<kDim>::Ptr path_m);
  bool expandFlightCorridor(Path<kDim>::Ptr path_m);

  void findMinLineDistance(const Eigen::Vector3d &start_m,
                           const Eigen::Vector3d &end_m,
                           double &min_distance_m); // expand flight segment corridor
  void getVoxelDistance(const Eigen::Vector3d &position_m, double &distance);
  double getVoxelDistance(const Eigen::Vector3d &position_m); // state validty checker

 private:
  exploration::VolumeShifter::Ptr vs_ = NULL;
  std::shared_ptr<Octree<FieldType> > octree_ptr_ = nullptr;
  // from occupancy world
  std::string id_;
  std::vector<key_t> alloc_list_;

  PlanningParameter ompl_params_;

  float voxel_dim_;
  bool treat_unknown_as_occupied_;
};





template<typename FieldType>
bool ProbCollisionChecker<FieldType>::isSphereCollisionFree(const Eigen::Vector3i &center,
                                                            const int radius_v) {

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
#endif //SUPEREIGHT_COLLISION_CHECKER_VOXEL_HPP
