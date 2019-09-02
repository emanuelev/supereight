//
// Created by anna on 26/07/19.
//

#ifndef SUPEREIGHT_INIT_NEW_POSITION_HPP
#define SUPEREIGHT_INIT_NEW_POSITION_HPP

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
#include "candidate_view.hpp"
#include "exploration_utils.hpp"
template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
//typedef SE_FIELD_TYPE FieldType;
typedef std::set<key_t> set3i;
namespace se {

namespace exploration {

/**
 * @brief creates a std::map <morton code of voxel block, vector with voxel coord> for the voxels
 * belonging to a sphere
 * @param center [voxel coord]
 * @param clearance_radius [m]
 * @param map
 * @param[out[ block_voxel_map [std::map]
 */
template<typename FieldType>
static void getSphereAroundPoint(const Eigen::Vector3i &center,
                                 const float clearance_radius,
                                 const Octree<FieldType> &map,
                                 const float res,
                                 mapvec3i *block_voxel_map) {
  const int radius_v = static_cast<int>(clearance_radius / res); // m/(m/voxel)
  const int leaf_level = map.leaf_level();
  for (int x = -radius_v; x <= radius_v; x++) {
    for (int y = -radius_v; y <= radius_v; y++) {
      for (int z = -radius_v; z <= radius_v; z++) {
        Eigen::Vector3i point_offset_v(x, y, z);
        //check if point is inside the sphere radius
        if (point_offset_v.norm() <= radius_v) {

          // check to wich voxelblock the voxel belongs
          const Eigen::Vector3i point_v = point_offset_v + center;
          if (point_v.x() < 0 || point_v.y() < 0 || point_v.z() < 0 || point_v.x() >= map.size()
              || point_v.y() >= map.size() || point_v.z() >= map.size()) {
            continue;
          }
          const key_t morton_code =
              keyops::encode(point_v.x(), point_v.y(), point_v.z(), leaf_level, map.max_level());
          (*block_voxel_map)[morton_code].push_back(point_v);
        }
      }//z
    }//y
  }//x

}

/**
 * for all voxels in the block_voxel_map, set the voxel state from unknown to free
 * @param[in] map
 * @param[in] block_voxel_map <voxelblock morton code, vector with voxel coord of all voxels
 * belonging to the sphere
 */
template<typename FieldType>
static void setStateToFree(Octree<FieldType> &map, mapvec3i *block_voxel_map) {

  for (const auto &block : *block_voxel_map) {
    VoxelBlock<FieldType> *block_ptr = map.fetch(block.first);
    for (const auto &voxel : block.second) {
      // make handler with the current voxel
      VoxelBlockHandler<FieldType> handler = {block_ptr, voxel};
      auto data = handler.get();
      if (data.st == voxel_state::kUnknown || data.st == voxel_state::kFrontier) {
        data.st = voxel_state::kFree;
        data.x = THRESH_FREE_LOG;
        handler.set(data);
      }
    }
  }
}
/**
 * @brief At planning initialization, set all voxel's state  inside a sphere from
 * unknown to free,
 * NO UPDATE to probability, Frontiers not redetected
 * @param center
 * @param planning_config
 * @param volume
 */

template<typename FieldType>
static void initNewPosition(const Eigen::Matrix4f &pose,
                            const Planning_Configuration &planning_config,
                            mapvec3i *block_voxel_map,
                            Octree<FieldType> &map) {

  const float res = map.dim() / static_cast<float>(map.size());
  // create list with morton code (VB) and list of voxels belonging to the sphere
  const pose3D curr_pose = getCurrPose(pose, res);
  std::vector<se::key_t> alloc_list;
  getSphereAroundPoint(curr_pose.p.cast<int>(),
                       planning_config.clearance_radius,
                       map,
                       res,
                       block_voxel_map);
  // allocate the space
  for (const auto &block : *block_voxel_map) {
    alloc_list.push_back(block.first); // insert morton
  }
  map.allocate(alloc_list.data(), alloc_list.size());
  setStateToFree(map, block_voxel_map);
  // std::cout << "[se/setSphere] done." << std::endl;

}

}// namespace exploration
}// namespace se
#endif //SUPEREIGHT_INIT_NEW_POSITION_HPP
