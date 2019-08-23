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


#include <string>

#include "se/octree.hpp"
#include "se/node_iterator.hpp"

typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vec3i;

int main(int argc, char ** argv) {

  std::shared_ptr<se::Octree<OFusion> > tree_
      = std::make_shared<se::Octree<OFusion> >();
  std::string filename = "/home/anna/exploration_ws/src/supereight/se_ompl/test/collision/w_box.bin";
  tree_->load(filename);
  std::cout << "file loaded" << std::endl;


  auto &block_buffer_base = tree_->getBlockBuffer();
  std::set<uint64_t > morton_set;
  for (int i = 0; i < block_buffer_base.size(); i++) {
    const key_t morton_code = block_buffer_base[i]->code_;
    morton_set.emplace(morton_code);
  }

  int free_voxels=0;
  int occupied_voxels = 0;
  se::node_iterator<OFusion> node_it(*tree_);
  for (const auto &explored : morton_set) {
    vec3i occupied_voxels_vec = node_it.getOccupiedVoxels(0.f, explored);
    vec3i free_voxels_vec = node_it.getFreeVoxels(explored);

    free_voxels += free_voxels_vec.size();
    occupied_voxels += occupied_voxels_vec.size();
  }
  std::cout <<"total free voxels " << free_voxels << std::endl;
  std::cout << "total occupied voxels " << occupied_voxels<< std::endl;

  exit(EXIT_SUCCESS);
}

