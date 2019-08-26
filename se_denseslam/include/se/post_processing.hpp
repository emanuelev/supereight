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

static const Eigen::IOFormat
    line_fmt (2, 0, " ", " ", "", "", "", "");

static constexpr float _voxel_state_threshold = 0.f;


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
  se::node_iterator<T> vb_it(octree);
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
    se::Node<T> node = *(node_buffer[i]);
    // Loop over all the node children.
    for (size_t j = 0; j < 8; ++j) {
      // Only look at children that have not been allocated at a lower level.
      if (node.child(j) == nullptr) {
        if (node.value_[j].x < _voxel_state_threshold) {
          // Free node.
          free_nodes++;
          free_node_volume += std::pow(node.side_ >> 1, 3) * voxel_volume;
        } else if (node.value_[j].x > _voxel_state_threshold) {
          // Occupied node.
          occupied_nodes++;
          occupied_node_volume += std::pow(node.side_ >> 1, 3) * voxel_volume;
        }
      }
    }
  }
}



template<typename T>
void crop_octree(se::Octree<T>& octree, const Eigen::Vector3f dim) {
  // Compute the min and max coordinates from the desired dimension. The map is
  // centered on [0 0 0]^T. Offset it so that it's centered inside the octree.
  // Increase the z offset so that the map's floor is halfway on the octree's z
  // axis.
  const Eigen::Vector3f map_offset (octree.dim() / 2.f, octree.dim() / 2.f,
      octree.dim() / 2.f + dim.z() / 2.f);
  const Eigen::Vector3f min_pos_m = -dim / 2.f + map_offset;
  const Eigen::Vector3f max_pos_m =  dim / 2.f + map_offset;
  // Convert limits to voxel coordinates.
  const Eigen::Vector3f min_pos_v_f = min_pos_m / octree.voxelDim();
  const Eigen::Vector3f max_pos_v_f = max_pos_m / octree.voxelDim();
  const Eigen::Vector3i min_pos_v = min_pos_v_f.cast<int>();
  const Eigen::Vector3i max_pos_v = max_pos_v_f.cast<int>();

  //std::cout << dim.format(line_fmt) << " m\n";
  //std::cout << min_pos_m.format(line_fmt) << " m  ->  "
  //          << min_pos_v.format(line_fmt) << " v\n";
  //std::cout << max_pos_m.format(line_fmt) << " m  ->  "
  //          << max_pos_v.format(line_fmt) << " v\n";

  // Iterate over all voxels.
  auto &vb_buffer = octree.getBlockBuffer();
  for (size_t i = 0; i < vb_buffer.size(); ++i) {
    se::VoxelBlock<T>* vb = vb_buffer[i];
    // Loop over all the single voxels.
    const Eigen::Vector3i first = vb->coordinates();
    const Eigen::Vector3i last = vb->coordinates() + Eigen::Vector3i::Constant(BLOCK_SIDE);
    for (int z = first.z(); z < last.z(); ++z) {
      for (int y = first.y(); y < last.y(); ++y) {
        for (int x = first.x(); x < last.x(); ++x) {

          // Get the child node coordinates.
          const Eigen::Vector3i child_v (x, y, z);
          if (   child_v.x()     < min_pos_v.x()
              || child_v.y()     < min_pos_v.y()
              || child_v.z()     < min_pos_v.z()
              || child_v.x() + 1 > max_pos_v.x()
              || child_v.y() + 1 > max_pos_v.y()
              || child_v.z() + 1 > max_pos_v.z()) {
            // Get the current data.
            auto val = vb->data(child_v);
            // Update occupancy and timestamp to unknown.
            val.x = _voxel_state_threshold;
            val.y = 0.0;
            // Set the new data.
            vb->data(child_v, val);
          }
        }
      }
    }
  }
  // Iterate over all nodes.
  auto &node_buffer = octree.getNodesBuffer();
  for (size_t i = 0; i < node_buffer.size(); ++i) {
    se::Node<T>* node = node_buffer[i];
    // Loop over all the node children.
    for (size_t j = 0; j < 8; ++j) {
      // Get the child node coordinates.
      const Eigen::Vector3i child_v = node->childCoordinates(j);
      const unsigned int child_side = node->side_ >> 1;
      if (   child_v.x()              < min_pos_v.x()
          || child_v.y()              < min_pos_v.y()
          || child_v.z()              < min_pos_v.z()
          || child_v.x() + child_side > max_pos_v.x()
          || child_v.y() + child_side > max_pos_v.y()
          || child_v.z() + child_side > max_pos_v.z()) {
        // Update occupancy and timestamp to unknown.
        node->value_[j].x = _voxel_state_threshold;
        node->value_[j].y = 0.0;
      }
    }
  }
}

#endif

