//
// Created by anna on 28/06/19.
//

#ifndef SUPEREIGHT_COLLISION_CHECKER_HPP
#define SUPEREIGHT_COLLISION_CHECKER_HPP
// Functions from Project : Probabilistic Trajectory Planning Volume Shifter
// by Nils FUnk

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include <cstdlib>
#include <cmath>
#include "se/utils/eigen_utils.h"
#include "exploration_utils.hpp"

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
namespace se {
namespace exploration {

//typedef FieldType OFusion;
template<typename T>
class CollisionCheck {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionCheck(const Volume<T> &volume, Planning_Configuration planning_config, double res);

  int isSphereCollisionFree(const Eigen::Vector3i center);

 private:

  Volume<T> volume_;
  double res_;
  Planning_Configuration planning_config_;
};

template<typename T>
CollisionCheck<T>::CollisionCheck(const VolumeTemplate<T, se::Octree> &volume,
                                  Planning_Configuration planning_config,
                                  double res)
    :

    volume_(volume), planning_config_(planning_config), res_(res) {

}

// from voxblox utils planning_utils_inl.h


template<typename T>
int CollisionCheck<T>::isSphereCollisionFree(const Eigen::Vector3i center) {

  int radius_v = static_cast<int>(planning_config_.cand_view_safety_radius / res_); // m/(m/voxel)
  se::Node<T> *node = nullptr;
  se::VoxelBlock<T> *block = nullptr;
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
            volume_._map_index->fetch_octant(point_v.x(),
                                             point_v.y(),
                                             point_v.z(),
                                             node,
                                             is_voxel_block);
            prev_pos = point_v;
            if (is_voxel_block) {
              block = static_cast<se::VoxelBlock<T> *> (node);
            } else {
              if (volume_._map_index->get(se::keyops::decode(node->code_)).x >= 0.f) {
//                std::cout << " [secollision] collision at node "
//                          << se::keyops::decode(node->code_).format(InLine) << std::endl;
                return 0;
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
                return 0;
              }
            } else {
              volume_._map_index->fetch_octant(point_v.x(),
                                               point_v.y(),
                                               point_v.z(),
                                               node,
                                               is_voxel_block);
              if (is_voxel_block) {
                block = static_cast<se::VoxelBlock<T> *> (node);
                if (block->data(point_v).x >= 0.f) {
//                  std::cout << " [secollision] collision at " << point_v.format(InLine) << " plog "
//                            << block->data(point_v).x << std::endl;
                  return 0;
                }
              } else {
                block = nullptr;
                if (volume_._map_index->get(se::keyops::decode(node->code_)).x >= 0.f) {
//                  std::cout << " [secollision] collision at node "
//                            << se::keyops::decode(node->code_).format(InLine) << std::endl;
                  return 0;
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
  return 1;
}
} // namespace exploration

} // namespace se

#endif //SUPEREIGHT_COLLISION_CHECKER_HPP
