/**
 * Copyright (C) 2019 Imperial College London.
 * Copyright (C) 2019 ETH Zürich.
 *
 *
 *
 * @file exploration_progress_evaluation.cpp
 * @author Anna Dai
 * @author Sotiris Papatheodorou
 * @date August 23, 2019
 */


// TODO
// Compute volume using voxel/Node side length.

#include <cmath>
#include <cstdio>
#include <iostream>
#include <set>
#include <string>

#include "se/octree.hpp"
#include "se/node_iterator.hpp"

typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vec3i;



std::string parse_arguments(int argc, char** argv) {
  std::string filename;

  switch (argc) {
    // Map filename supplied.
    case 2:
      filename = std::string(argv[1]);
      break;

    // Show usage message.
    default:
      std::cout << "Usage: " << argv[0] << " FILENAME\n";
      exit(EXIT_FAILURE);
  }

  // Test if the file exists.
  FILE* fp = fopen(filename.c_str(), "r");
  if (fp == nullptr) {
    std::cout << "Error: file " << filename
        << " does not exist or is not accessible\n";
    exit(EXIT_FAILURE);
  } else {
    fclose(fp);
  }

  return filename;
}



int main(int argc, char** argv) {
  constexpr float voxel_state_threshold = 0.f;

  // Parse the input arguments.
  std::string filename = parse_arguments(argc, argv);

  // Initialize the Octree and load the saved map.
  std::shared_ptr<se::Octree<OFusion> > tree_
      = std::make_shared<se::Octree<OFusion> >();
  tree_->load(filename);

  // Get and show Octree info.
  const int   octree_size   = tree_->size();
  const float octree_dim    = tree_->dim();
  const float octree_volume = std::pow(octree_dim, 3.f);
  const float voxel_dim     = tree_->voxelDim();
  const float voxel_volume  = std::pow(voxel_dim, 3.f);
  std::cout << "Octree info -----------------------\n";
  std::cout << "Octree size:   " << octree_size   << " v\n";
  std::cout << "Octree dim:    " << octree_dim    << " m\n";
  std::cout << "Octree volume: " << octree_volume << " m^3\n";
  std::cout << "Voxel dim:     " << voxel_dim     << " m\n";
  std::cout << "Voxel volume:  " << voxel_volume  << " m^3\n";

  // Get all unique VoxelBlock Morton codes.
  auto &vb_buffer = tree_->getBlockBuffer();
  std::set<se::key_t> morton_set;
  for (size_t i = 0; i < vb_buffer.size(); ++i) {
    const se::key_t morton_code = vb_buffer[i]->code_;
    morton_set.insert(morton_code);
  }
  // Count occupied and free voxels.
  size_t free_voxels = 0;
  size_t occupied_voxels = 0;
  se::node_iterator<OFusion> vb_it(*tree_);
  for (const auto &explored : morton_set) {
    vec3i occupied_voxels_vec = vb_it.getOccupiedVoxels(0.f, explored);
    vec3i free_voxels_vec = vb_it.getFreeVoxels(explored);

    free_voxels += free_voxels_vec.size();
    occupied_voxels += occupied_voxels_vec.size();
  }
  float free_voxel_volume = free_voxels * voxel_volume;
  float occupied_voxel_volume = occupied_voxels * voxel_volume;

  // Count occupied and free Nodes and compute their volume.
  size_t free_nodes = 0;
  size_t occupied_nodes = 0;
  float free_node_volume = 0.f;
  float occupied_node_volume = 0.f;
  auto &node_buffer = tree_->getNodesBuffer();
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

  // Print results.
  std::cout << "Voxels ----------------------------\n";
  std::cout << "Free voxels:           " << free_voxels << std::endl;
  std::cout << "Occupied voxels:       " << occupied_voxels << std::endl;
  std::cout << "Explored voxels:       " << free_voxels + occupied_voxels << std::endl;
  std::cout << "Explored voxel volume: " << free_voxel_volume + occupied_voxel_volume << " m^3\n";
  std::cout << "Nodes -----------------------------\n";
  std::cout << "Free nodes:            " << free_nodes << std::endl;
  std::cout << "Occupied nodes:        " << occupied_nodes << std::endl;
  std::cout << "Explored nodes:        " << free_nodes + occupied_nodes << std::endl;
  std::cout << "Explored node volume:  " << free_node_volume + occupied_node_volume << " m^3\n";
  std::cout << "Results ---------------------------\n";
  std::cout << "Explored volume:       "
      << free_node_volume + occupied_node_volume
      + free_voxel_volume + occupied_voxel_volume << " m^3\n";

  exit(EXIT_SUCCESS);
}

