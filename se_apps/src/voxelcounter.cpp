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

  return filename;
}



int main(int argc, char** argv) {

  // Parse the input arguments.
  std::string filename = parse_arguments(argc, argv);

  // Initialize the Octree and load the saved map.
  std::shared_ptr<se::Octree<OFusion> > tree_
      = std::make_shared<se::Octree<OFusion> >();
  tree_->load(filename);

  // Get all unique Morton codes.
  auto &block_buffer_base = tree_->getBlockBuffer();
  std::set<se::key_t> morton_set;
  for (int i = 0; i < block_buffer_base.size(); i++) {
    const se::key_t morton_code = block_buffer_base[i]->code_;
    morton_set.insert(morton_code);
  }

  // Count occupied and free voxels.
  size_t free_voxels = 0;
  size_t occupied_voxels = 0;
  se::node_iterator<OFusion> node_it(*tree_);
  for (const auto &explored : morton_set) {
    vec3i occupied_voxels_vec = node_it.getOccupiedVoxels(0.f, explored);
    vec3i free_voxels_vec = node_it.getFreeVoxels(explored);

    free_voxels += free_voxels_vec.size();
    occupied_voxels += occupied_voxels_vec.size();
  }
  std::cout << "Total free voxels:     " << free_voxels << std::endl;
  std::cout << "Total occupied voxels: " << occupied_voxels << std::endl;
  std::cout << "Total explored voxels: " << free_voxels + occupied_voxels
      << std::endl;

  exit(EXIT_SUCCESS);
}

