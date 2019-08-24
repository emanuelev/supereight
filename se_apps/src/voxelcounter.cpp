/**
 * Copyright (C) 2019 Imperial College London.
 * Copyright (C) 2019 ETH Zürich.
 *
 *
 *
 * @file exploration_progress_evaluation.cpp
 * @author Anna Dai
 * @author Sotiris Papatheodorou
 * @date August 24, 2019
 */



#include <cstdio>
#include <iostream>
#include <string>

#include "se/octree.hpp"
#include "se/post_processing.hpp"



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
  std::shared_ptr<se::Octree<OFusion> > octree_
      = std::make_shared<se::Octree<OFusion> >();
  octree_->load(filename);

  // Get and show Octree info.
  const int   octree_size   = octree_->size();
  const float octree_dim    = octree_->dim();
  const float octree_volume = std::pow(octree_dim, 3.f);
  const float voxel_dim     = octree_->voxelDim();
  const float voxel_volume  = std::pow(voxel_dim, 3.f);
  std::cout << "Octree info -----------------------\n";
  std::cout << "Octree size:   " << octree_size   << " v\n";
  std::cout << "Octree dim:    " << octree_dim    << " m\n";
  std::cout << "Octree volume: " << octree_volume << " m^3\n";
  std::cout << "Voxel dim:     " << voxel_dim     << " m\n";
  std::cout << "Voxel volume:  " << voxel_volume  << " m^3\n";



  // Count the explored voxels.
  size_t free_voxels;
  float  free_voxel_volume;
  size_t occupied_voxels;
  float  occupied_voxel_volume;
  size_t free_nodes;
  float  free_node_volume;
  size_t occupied_nodes;
  float  occupied_node_volume;
  count_voxels(*octree_,
      free_voxels,     free_voxel_volume,
      occupied_voxels, occupied_voxel_volume,
      free_nodes,      free_node_volume,
      occupied_nodes,  occupied_node_volume);



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

