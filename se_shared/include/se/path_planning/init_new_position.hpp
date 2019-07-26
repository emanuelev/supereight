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
#include "collision_checker.hpp"
#include "exploration_utils.hpp"
template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
typedef SE_FIELD_TYPE FieldType;
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
void getSphereAroundPoint(const Eigen::Vector3i & center, const float
clearance_radius, const Volume<FieldType> &map, mapvec3i *block_voxel_map){
  const int res = map._map_index->dim() / map._map_index->size();
  const int radius_v = static_cast<int>(clearance_radius / res); // m/(m/voxel)
  const int leaf_level = map._map_index->leaf_level();

  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;
  bool is_voxel_block;
  for (int x = -radius_v; x <= radius_v; x++) {
    for (int y = -radius_v; y <= radius_v; y++) {
      for (int z = -radius_v; z <= radius_v; z++) {
        Eigen::Vector3i point_offset_v(x, y, z);
        //check if point is inside the sphere radius
        if (point_offset_v.norm() <= radius_v) {
          // check to wich voxelblock the voxel belongs
          const Eigen::Vector3i point_v = point_offset_v + center;
          const key_t morton_code = map._map_index->hash(x,y,z, leaf_level);
          (*block_voxel_map)[morton_code].push_back(Eigen::Vector3i(x,y,z));


        }

      }//z
    }//y
  }//x

}
/**
 * @brief At planning initialization, set all voxel's state  inside a sphere from
 * unknown to free
 * @param center
 * @param planning_config
 * @param volume
 */
void initNewPosition(const Eigen::Vector3i & center, const Planning_Configuration
&planning_config, const Volume<FieldType> & map){

  // create list with morton code (VB) and list of voxels belonging to the sphere

  mapvec3i block_voxel_map;
  getSphereAroundPoint(center, planning_config.clearance_radius, map,
      &block_voxel_map );

  // update all the voxels in side it via node iterator and set them to voxel_state kfree

}


}// namespace exploration
}// namespace se
#endif //SUPEREIGHT_INIT_NEW_POSITION_HPP
