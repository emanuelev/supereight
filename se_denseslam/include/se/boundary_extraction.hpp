/**
 * Information-theoretic exploration, OMPL Motion Validator Skeleton.
 *
 * Copyright (C) 2019 Imperial College London.
 * Copyright (C) 2019 ETH Zürich.
 *
 * @file boundary_extraction.hpp
 * @author Anna Dai
 * @date August 22, 2019
 */

#ifndef SUPEREIGHT_BOUNDARY_EXTRACTION_HPP
#define SUPEREIGHT_BOUNDARY_EXTRACTION_HPP

#include <cstdlib>
#include <map>

#include "continuous/volume_template.hpp"
#include <se/octree.hpp>
#include <se/functors/data_handler.hpp>
#include <se/node_iterator.hpp>
#include <se/utils/eigen_utils.h>

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;

static inline void insertBlocksToMap(map3i &blocks_map, set3i *blocks) {
  if (blocks->size() == 0) return;

  for (auto it = blocks->begin(); it != blocks->end(); ++it) {
    const Eigen::Vector3i voxel_coord = se::keyops::decode(*it);
    blocks_map.emplace(*it, voxel_coord);
  }

//  std::cout << "[supereight/boundary] frontier maps size " << blocks_map.size() << std::endl;
}
static inline void insertBlocksToMap(map3i &blocks_map, mapvec3i *blocks) {
  if (blocks->size() == 0) return;
  for (auto it = blocks->begin(); it != blocks->end(); ++it) {
    const Eigen::Vector3i voxel_coord = se::keyops::decode(it->first);
    blocks_map.emplace(it->first, voxel_coord);
  }

//  std::cout << "[supereight/boundary] free maps size " << blocks_map.size() << std::endl;
}

/**
 * check if past frontier voxels have been updated and update the std::map
 * [uint64_t morton_code, Eigen::Vector3i coord]
 * Issue map wide function
 */
template<typename T>
void updateFrontierMap(const Volume<T> &volume, map3i &blocks_map) {
//  std::cout << "[supereight] frontier map size: before " << frontier_blocks_map.size()
//  <<std::endl;
  se::node_iterator<T> node_it(*(volume._map_index));
  for (auto it = blocks_map.begin(); it != blocks_map.end(); ++it) {
    // check if the occupancy probability of the frontier voxels has been updated
    // changes voxel states from frontier to free or occupied
    if (!node_it.deleteFrontierVoxelBlockviaMorton(it->first)) {
//      std::cout << "[supereight/boundary] no frontier in voxel block => erase" << std::endl;
      blocks_map.erase(it->first);
    }
  }
}

template<typename T>
void updateFrontierMap(const Volume<T> &volume,
                       map3i &blocks_map,
                       set3i *blocks,
                       const bool update_frontier_map) {

  Eigen::Vector3i lowerbound;
  Eigen::Vector3i upperbound;
  // check if the ones in the map
  if (update_frontier_map) {
    // std::cout << "update frontier map " <<std::endl;
    updateFrontierMap(volume, blocks_map);
  }

  // insert new frontier blocks to map
  insertBlocksToMap(blocks_map, blocks);
}

// level at leaf level
/**
 * @brief get the boundaries from the free map
 * @param octree_ptr_
 * @param blocks_map   morton code of voxelblocks with free voxels
 * @param lower_bound[out]
 * @param upper_bound[out]
 */
template<typename T>
static inline void getFreeMapBounds(const std::shared_ptr<se::Octree<T> > octree_ptr_,
                                    const map3i &blocks_map,
                                    Eigen::Vector3i &lower_bound,
                                    Eigen::Vector3i &upper_bound) {

  se::node_iterator<T> node_it(*octree_ptr_);
  auto it_beg = blocks_map.begin();
  auto it_end = blocks_map.end();
  Eigen::Vector3i lower_bound_tmp;
  Eigen::Vector3i upper_bound_tmp;

  key_t lower_bound_morton = it_beg->first;
  Eigen::Vector3i lower_block_coord = se::keyops::decode(lower_bound_morton);
  bool valid_lower = node_it.getFreeVoxel(lower_bound_morton, lower_block_coord);

  --it_end;
  key_t upper_bound_morton = it_end->first;
  Eigen::Vector3i upper_block_coord = se::keyops::decode(upper_bound_morton);
  bool valid_upper = node_it.getFreeVoxel(upper_bound_morton, upper_block_coord);

  // std::cout << "upper " << upper_block_coord.format(InLine) << " lower "<< lower_block_coord.format(InLine)
  // << std::endl;
  lower_bound = upper_block_coord;
  upper_bound = lower_block_coord;
  while (it_beg != it_end) {
    ++it_beg;
    lower_bound_morton = it_beg->first;
    lower_bound_tmp = se::keyops::decode(lower_bound_morton);
    valid_lower = node_it.getFreeVoxel(lower_bound_morton, lower_bound_tmp);

    --it_end;
    upper_bound_morton = it_end->first;
    upper_bound_tmp = se::keyops::decode(upper_bound_morton);
    valid_upper = node_it.getFreeVoxel(upper_bound_morton, upper_bound_tmp);
    // std::cout << "upper " << upper_bound_tmp.format(InLine) << " lower "<< lower_bound_tmp.format(InLine)
    // << std::endl;
    if (lower_bound_tmp.norm() < lower_bound.norm() && valid_lower) {
      // std::cout << "lower_bound from " << lower_bound.format(InLine) << " to "
      // << lower_bound_tmp.format(InLine) << std::endl;
      lower_bound = lower_bound_tmp;

    }
    if (upper_bound_tmp.norm() > upper_bound.norm() && valid_upper) {
      // std::cout << "upper_bound from " << upper_bound.format(InLine) << " to "
      // << upper_bound_tmp.format(InLine) << std::endl;
      upper_bound = upper_bound_tmp;
    }

  }
  upper_bound += Eigen::Vector3i(8, 8, 8);

}
#endif //SUPEREIGHT_BOUNDARY_EXTRACTION_HPP
