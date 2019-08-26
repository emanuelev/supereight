/**
 * Copyright (C) 2019 Imperial College London.
 *
 *
 *
 * @file mapcropper.cpp
 * @author Sotiris Papatheodorou
 * @date August 23, 2019
 */

// TODO
// Ensure the map dimensions are smaller or equal to the octree dimensions.


#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "se/octree.hpp"
#include "se/post_processing.hpp"


struct arguments {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string filename;
  Eigen::Vector3f map_dim;
};

struct arguments parse_arguments(int argc, char** argv) {
  struct arguments args;

  switch (argc) {
    // Map filename and dimensions supplied.
    case 5:
      args.filename = std::string(argv[1]);
      args.map_dim.x() = atof(argv[2]);
      args.map_dim.y() = atof(argv[3]);
      args.map_dim.z() = atof(argv[4]);
      break;

    // Show usage message.
    default:
      std::cout << "Usage: " << argv[0] << " FILENAME DIM_X DIM_Y DIM_Z\n";
      exit(EXIT_FAILURE);
  }

  // Test if the file exists.
  FILE* fp = fopen(args.filename.c_str(), "r");
  if (fp == nullptr) {
    std::cout << "Error: file " << args.filename
        << " does not exist or is not accessible\n";
    exit(EXIT_FAILURE);
  } else {
    fclose(fp);
  }

  return args;
}



int main(int argc, char** argv) {
  constexpr float voxel_state_threshold = 0.f;

  // Parse the input arguments.
  const struct arguments args = parse_arguments(argc, argv);

  // Initialize the Octree and load the saved map.
  std::shared_ptr<se::Octree<OFusion> > octree
      = std::make_shared<se::Octree<OFusion> >();
  octree->load(args.filename);

  // Get and show Octree info.
  const int   octree_size   = octree->size();
  const float octree_dim    = octree->dim();
  const float octree_volume = std::pow(octree_dim, 3.f);
  const float voxel_dim     = octree->voxelDim();
  const float voxel_volume  = std::pow(voxel_dim, 3.f);
  std::cout << "Octree info -----------------------\n";
  std::cout << "Octree size:   " << octree_size   << " x "
                                 << octree_size   << " x "
                                 << octree_size   << " voxels\n";
  std::cout << "Octree dim:    " << octree_dim    << " x "
                                 << octree_dim    << " x "
                                 << octree_dim    << " m\n";
  std::cout << "Octree volume: " << octree_volume << " m^3\n";
  std::cout << "Voxel dim:     " << voxel_dim     << " m\n";
  std::cout << "Voxel volume:  " << voxel_volume  << " m^3\n";

  // Show map info.
  std::cout << "Map info --------------------------\n";
  std::cout << args.map_dim.x() << " x "
            << args.map_dim.y() << " x "
            << args.map_dim.z() << " m\n";

  // Crop the map.
  crop_octree(*octree, args.map_dim);

  // Save the new map.
  std::string filename_cropped = args.filename;
  filename_cropped.erase(filename_cropped.end() - 4, filename_cropped.end());
  filename_cropped += "_cropped.bin";
  std::cout << "Saving cropped map in " << filename_cropped << "\n";
  octree->save(filename_cropped);

  exit(EXIT_SUCCESS);
}

