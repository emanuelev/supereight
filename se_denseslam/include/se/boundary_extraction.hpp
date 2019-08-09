//
// Created by anna on 12/06/19.
//

#ifndef SUPEREIGHT_BOUNDARY_EXTRACTION_HPP
#define SUPEREIGHT_BOUNDARY_EXTRACTION_HPP

#include <cstdlib>
#include <map>

#include "continuous/volume_template.hpp"
//#include "DenseSLAMSystem.h"
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
 * TODO how to use OMP
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
    updateFrontierMap(volume, blocks_map);
  }

  // insert new frontier blocks to map
  insertBlocksToMap(blocks_map, blocks);
}


// level at leaf level
template<typename T>
static inline void getFreeMapBounds(const std::shared_ptr<se::Octree<T> > octree_ptr_,
                      const map3i &blocks_map,
                      Eigen::Vector3i &lower_bound,
                      Eigen::Vector3i &upper_bound) {

  se::node_iterator<T> node_it(*octree_ptr_);
  auto it_beg = blocks_map.begin();
  auto it_end = blocks_map.end();


  lower_bound = Eigen::Vector3i(-1,-1,-1);

  while(lower_bound == Eigen::Vector3i(-1,-1,-1)) {
    const key_t lower_bound_morton = it_beg->first;
//    std::cout << "[se/boundary] " << lower_bound_morton << "coord "
//              << se::keyops::decode(lower_bound_morton).format(InLine) << std::endl;
    lower_bound = node_it.getFreeVoxel(lower_bound_morton);
//    std::cout << "[se/boundary] " << lower_bound.format(InLine) << std::endl;
    ++it_beg;
  }
//  std::cout << "[se/boundary] " << lower_bound.format(InLine) << std::endl;
  upper_bound = Eigen::Vector3i(-1,-1,-1);
  while(upper_bound == Eigen::Vector3i(-1,-1,-1)) {
    --it_end;
    const key_t upper_bound_morton = it_end->first;
//    std::cout << "[se/boundary] " << upper_bound_morton << "coord "
//              << se::keyops::decode(upper_bound_morton).format(InLine) << std::endl;
    upper_bound = node_it.getFreeVoxel(upper_bound_morton);
//    std::cout << "[se/boundary] " << upper_bound.format(InLine) << std::endl;
  }
}
#endif //SUPEREIGHT_BOUNDARY_EXTRACTION_HPP
