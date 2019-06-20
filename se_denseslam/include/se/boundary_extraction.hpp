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

void insertOcclusionBlocksToMap(map3i &occl_blocks_map, vec3i *occl_blocks) {
  if (occl_blocks->size() != 0) {
    std::cout << "[supereight/boundary] adding " << occl_blocks->size() << " new occlusion blocks "
                                                                           "" << std::endl;
  } else {
    std::cout << "[supereight/boundary] occlusion blocks empty" << std::endl;
    return;
  }

  for (auto it = occl_blocks->begin(); it != occl_blocks->end(); ++it) {
    uint64_t morton = compute_morton(it->x(), it->y(), it->z());
    occl_blocks_map.emplace(morton, *it);
  }
  std::cout << "[supereight/boundary] occlusion maps size " << occl_blocks_map.size() << std::endl;
}

void insertFrontierBlocksToMap(map3i &frontier_blocks_map, vec3i *frontier_blocks) {
  if (frontier_blocks->size() != 0) {
    std::cout << "[supereight/boundary] adding " << frontier_blocks->size()
              << " new frontier blocks "
                 "" << std::endl;
  } else {
    std::cout << "[supereight/boundary] frontier blocks empty" << std::endl;
    return;
  }

  for (auto it = frontier_blocks->begin(); it != frontier_blocks->end(); ++it) {
    uint64_t morton = compute_morton(it->x(), it->y(), it->z());
    frontier_blocks_map.emplace(morton, *it);
  }
  std::cout << "[supereight/boundary] frontier maps size " << frontier_blocks_map.size()
            << std::endl;
}
/**
 * Frontier voxels
 * if curr voxel is unknown, check if the neighbourgh voxels are allocated and free
 * TODO change vector to set to increase speed
 */
template<typename T>
void getFrontierBlocks(const Volume<T> &volume, vec3i *frontier_blocks) {

  std::cout << "[supereight] get Frontier blocks " << std::endl;
  uint64_t siblings_res[8];
  Eigen::Matrix<int, 4, 6> face_res;
  int level;
  int max_depth = log2(volume._map_index->size());
  bool is_frontier = false;
  // for each  voxel is there a neighbour voxel with more determine occ prob
  for (auto it = frontier_blocks->begin(); it != frontier_blocks->end(); ++it) {

    key_t octant_curr_voxel = compute_morton(it->x(), it->y(), it->z());
    // returns vector with the morton code of the neighbours
//    se::siblings(siblings_res, octant_curr_voxel, max_depth);
    se::one_neighbourhood(face_res, octant_curr_voxel, max_depth);
    for (int col = 0; col < 6; col++) {
      std::cout << "face coord " << face_res.block<3, 1>(0, col) << std::endl;
      if (volume._map_index->get(face_res.block<3, 1>(0, col)).y != 0.0
          && volume._map_index->get(face_res.block<3, 1>(0, col)).x < 0.0) {
        is_frontier = true;
      }
    }

//    // go through all sibling voxel
//    for (const auto &sibling_m : siblings_res) {
//      std::cout << "prob " << volume._map_index->get(sibling_m).x << "timestamp " << volume
//      ._map_index->get(sibling_m).y << std::endl;
//      if (volume._map_index->get(sibling_m).y != 0.0 && volume._map_index->get(sibling_m).x <0.0) {
//        is_frontier = true;
//      }
//    }

    if (!is_frontier && it != frontier_blocks->end()) {

      std::cout << "not frontier block" << std::endl;
    }
    is_frontier = false;

  }
}
/**
 * check if past frontier blocks have been updated and update the std::map
 * TODO how to use OMP
 */
template<typename T>
void updateFrontierMap(const Volume<T> &volume, map3i &frontier_blocks_map) {
  std::cout << "[supereight] frontier map size: before " << frontier_blocks_map.size();
  se::node_iterator<T> node_it(*(volume._map_index));
  for (auto it = frontier_blocks_map.begin(); it != frontier_blocks_map.end(); ++it) {
    // check if the occupancy probability of the block has been updated
    if (!node_it.hasFrontierVoxelBlockviaMorton(it->second)) {
      std::cout << "[supereight/boundary] no frontier in voxel block => erase" << std::endl;
      frontier_blocks_map.erase(it->first);
    }else{
      node_it.deleteFrontierVoxelBlockviaMorton(it->second);
    }
  }
  std::cout << ", updated " << frontier_blocks_map.size() << std::endl;
}

template<typename T>
void updateFrontierMap(const Volume<T> &volume,
                       map3i &frontier_blocks_map,
                       vec3i *frontier_blocks) {
  // go through frontier map and check if they are actually frontiers

//  getFrontierBlocks(volume, frontier_blocks);
  // check if the ones in the map
  updateFrontierMap(volume, frontier_blocks_map);
  // insert new frontier blocks to map
  insertFrontierBlocksToMap(frontier_blocks_map, frontier_blocks);
}

#endif //SUPEREIGHT_BOUNDARY_EXTRACTION_HPP
