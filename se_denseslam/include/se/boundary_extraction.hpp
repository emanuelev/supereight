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


void insertBlocksToMap(map3i &blocks_map, set3i *blocks) {
  if (blocks->size() == 0) return;


  for (auto it = blocks->begin(); it != blocks->end(); ++it) {
    const Eigen::Vector3i voxel_coord = se::keyops::decode(*it);
    blocks_map.emplace(*it, voxel_coord);
  }
//  std::cout << "[supereight/boundary] frontier maps size " << blocks_map.size() << std::endl;
}
/**
 * check if past frontier voxels have been updated and update the std::map
 * [uint64_t morton_code, Eigen::Vector3i coord]
 * TODO how to use OMP
 * Issue map wide function
 */
template<typename T>
void updateFrontierMap(const Volume<T> &volume, map3i &frontier_blocks_map) {
//  std::cout << "[supereight] frontier map size: before " << frontier_blocks_map.size()
//  <<std::endl;
  se::node_iterator<T> node_it(*(volume._map_index));
  for (auto it = frontier_blocks_map.begin(); it != frontier_blocks_map.end(); ++it) {
    // check if the occupancy probability of the frontier voxels has been updated
    // changes voxel states from frontier to free or occupied
    if (!node_it.deleteFrontierVoxelBlockviaMorton(it->first)) {
//      std::cout << "[supereight/boundary] no frontier in voxel block => erase" << std::endl;
      frontier_blocks_map.erase(it->first);
    }
  }
//  std::cout << ", updated " << frontier_blocks_map.size() << std::endl;
}

template<typename T>
void updateFrontierMap(const Volume<T> &volume,
                       map3i &frontier_blocks_map,
                       set3i *frontier_blocks,
                       const bool update_frontier_map) {

  // check if the ones in the map
  if(update_frontier_map){
  updateFrontierMap(volume, frontier_blocks_map);
  }
  // insert new frontier blocks to map
  insertBlocksToMap(frontier_blocks_map, frontier_blocks);
}


#endif //SUPEREIGHT_BOUNDARY_EXTRACTION_HPP
