/*
 * Copyright 2019 Sotiris Papatheodorou, Imperial College London
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __POST_PROCESSING_HPP
#define __POST_PROCESSING_HPP

#include <cmath>
#include <cstdio>
#include <iostream>
#include <set>

#include "se/octree.hpp"
#include "se/node_iterator.hpp"



typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vec3i;



template<typename T>
void count_voxels(se::Octree<T>& octree,
                  size_t&        free_voxels,
                  float&         free_voxel_volume,
                  size_t&        occupied_voxels,
                  float&         occupied_voxel_volume,
                  size_t&        free_nodes,
                  float&         free_node_volume,
                  size_t&        occupied_nodes,
                  float&         occupied_node_volume) {
  constexpr float voxel_state_threshold = 0.f;

  const int   octree_size   = octree.size();
  const float octree_dim    = octree.dim();
  const float octree_volume = std::pow(octree_dim, 3.f);
  const float voxel_dim     = octree.voxelDim();
  const float voxel_volume  = std::pow(voxel_dim, 3.f);

  // Get all unique VoxelBlock Morton codes.
  auto &vb_buffer = octree.getBlockBuffer();
  std::set<se::key_t> morton_set;
  for (size_t i = 0; i < vb_buffer.size(); ++i) {
    const se::key_t morton_code = vb_buffer[i]->code_;
    morton_set.insert(morton_code);
  }
  // Count occupied and free voxels.
  free_voxels = 0;
  occupied_voxels = 0;
  se::node_iterator<OFusion> vb_it(octree);
  for (const auto &explored : morton_set) {
    vec3i occupied_voxels_vec = vb_it.getOccupiedVoxels(0.f, explored);
    vec3i free_voxels_vec = vb_it.getFreeVoxels(explored);

    free_voxels += free_voxels_vec.size();
    occupied_voxels += occupied_voxels_vec.size();
  }
  free_voxel_volume = free_voxels * voxel_volume;
  occupied_voxel_volume = occupied_voxels * voxel_volume;

  // Count occupied and free Nodes and compute their volume.
  free_nodes = 0;
  occupied_nodes = 0;
  free_node_volume = 0.f;
  occupied_node_volume = 0.f;
  auto &node_buffer = octree.getNodesBuffer();
  for (size_t i = 0; i < node_buffer.size(); ++i) {
    se::Node<OFusion> node = *(node_buffer[i]);
    // Loop over all the node children.
    for (size_t j = 0; j < 8; ++j) {
      // Only look at children that have not been allocated at a lower level.
      if (node.child(j) == nullptr) {
        if (node.value_[j].x < voxel_state_threshold) {
          // Free node.
          free_nodes++;
          free_node_volume += std::pow(node.side_ >> 1, 3) * voxel_volume;
        } else if (node.value_[j].x > voxel_state_threshold) {
          // Occupied node.
          occupied_nodes++;
          occupied_node_volume += std::pow(node.side_ >> 1, 3) * voxel_volume;
        }
      }
    }
  }
}

#endif

