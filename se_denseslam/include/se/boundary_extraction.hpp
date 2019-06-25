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

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vec3i;
typedef std::map<uint64_t,
                 Eigen::Vector3i,
                 std::less<uint64_t>,
                 Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector3i> > > map3i;

void insertBlocksToMap(map3i &blocks_map, set3i *blocks) {
  if (blocks->size() != 0) {
    std::cout << "[supereight/boundary] adding " << blocks->size() << " new blocks "
                                                                      "" << std::endl;
  } else {
    std::cout << "[supereight/boundary] frontier blocks empty" << std::endl;
    return;
  }

  for (auto it = blocks->begin(); it != blocks->end(); ++it) {
//    uint64_t morton = compute_morton(it->x(), it->y(), it->z());
    Eigen::Vector3i voxel_coord = unpack_morton(*it);
    blocks_map.emplace(*it, voxel_coord);
  }
  std::cout << "[supereight/boundary] frontier maps size " << blocks_map.size() << std::endl;
}
/**
 * check if past frontier voxels have been updated and update the std::map
 * via voxel blocks / its morton code
 * TODO how to use OMP
 */
template<typename T>
void updateFrontierMap(const Volume<T> &volume, map3i &frontier_blocks_map) {
  std::cout << "[supereight] frontier map size: before " << frontier_blocks_map.size();
  se::node_iterator<T> node_it(*(volume._map_index));
  for (auto it = frontier_blocks_map.begin(); it != frontier_blocks_map.end(); ++it) {
    // check if the occupancy probability of the frontier voxels has been updated
    // changes voxel states from frontier to free or occupied
    if (!node_it.deleteFrontierVoxelBlockviaMorton(it->second)) {
//      std::cout << "[supereight/boundary] no frontier in voxel block => erase" << std::endl;
      frontier_blocks_map.erase(it->first);
    }
  }
  std::cout << ", updated " << frontier_blocks_map.size() << std::endl;
}

template<typename T>
void updateFrontierMap(const Volume<T> &volume,
                       map3i &frontier_blocks_map,
                       set3i *frontier_blocks) {
  // go through frontier map and check if they are actually frontiers

  // check if the ones in the map
  updateFrontierMap(volume, frontier_blocks_map);
  // insert new frontier blocks to map
  insertBlocksToMap(frontier_blocks_map, frontier_blocks);
}

template<typename T>
void updateOcclusionMap(const Volume<T> &volume,
                        map3i &occlusion_blocks_map,
                        set3i *occlusion_blocks) {
  se::node_iterator<T> node_it(*(volume._map_index));
  for (auto it = occlusion_blocks_map.begin(); it != occlusion_blocks_map.end(); ++it) {
    if (!node_it.deleteOcclusionVoxelsviaMorton(it->second)) {
      occlusion_blocks_map.erase(it->first);
    }
  }
  insertBlocksToMap(occlusion_blocks_map, occlusion_blocks);
}

#endif //SUPEREIGHT_BOUNDARY_EXTRACTION_HPP
