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



#include "se/octree.hpp"
#include "se/post_processing.hpp"

#include "gtest/gtest.h"


typedef OFusion testT;
typedef voxel_traits<testT>::value_type valT;


class TestOctree : public ::testing::Test {
  protected:
    virtual void SetUp() {
      free_voxels_     = 0;
      occupied_voxels_ = 0;
      free_nodes_      = 0;
      occupied_nodes_  = 0;
      free_voxel_volume_     = 0.f;
      occupied_voxel_volume_ = 0.f;
      free_node_volume_      = 0.f;
      occupied_node_volume_  = 0.f;

      octree_ = std::make_shared<se::Octree<testT> >();
      octree_->init(octree_size_, octree_dim_);

      const float single_voxel_volume = std::pow(octree_->voxelDim(), 3);

      // Allocate some VoxelBlocks.
      constexpr size_t num_voxel_blocks = 9+9+9;
      const Eigen::Vector3i blocks[num_voxel_blocks] = {
          { 0,  0,  0}, { 8,  0,  0}, {16,  0,  0},
          { 0,  8,  0}, { 8,  8,  0}, {16,  8,  0},
          { 0, 16,  0}, { 8, 16,  0}, {16, 16,  0},
          { 0,  0,  8}, { 8,  0,  8}, {16,  0,  8},
          { 0,  8,  8}, { 8,  8,  8}, {16,  8,  8},
          { 0, 16,  8}, { 8, 16,  8}, {16, 16,  8},
          { 0,  0, 16}, { 8,  0, 16}, {16,  0, 16},
          { 0,  8, 16}, { 8,  8, 16}, {16,  8, 16},
          { 0, 16, 16}, { 8, 16, 16}, {16, 16, 16},
      };
      se::key_t alloc_list[num_voxel_blocks];
      for (size_t i = 0; i < num_voxel_blocks; ++i) {
        alloc_list[i] = octree_->hash(blocks[i](0), blocks[i](1), blocks[i](2));
      }
      octree_->allocate(alloc_list, num_voxel_blocks);

      // Set the values of all the single voxels by iterating over all voxels
      // of each allocated VoxelBlock.
      for (size_t i = 0; i < num_voxel_blocks; ++i) {
        se::VoxelBlock<testT> * vblock
            = octree_->fetch(blocks[i].x(), blocks[i].y(), blocks[i].z());

        for (size_t z = 0; z < BLOCK_SIDE; ++z) {
          for (size_t y = 0; y < BLOCK_SIDE; ++y) {
            for (size_t x = 0; x < BLOCK_SIDE; ++x) {
              const Eigen::Vector3i vox = blocks[i] + Eigen::Vector3i(x, y, z);
              // The value of each voxel is proportional to the sum of its
              // coordinates and side length.
              valT val;
              if (vox.x() < vox.y() ) {
                val.x = val_;
                occupied_voxels_++;
                occupied_voxel_volume_ += single_voxel_volume;
              } else {
                val.x = -val_;
                free_voxels_++;
                free_voxel_volume_ += single_voxel_volume;
              }
              vblock->data(vox, val);
            }
          }
        }
      }

      // Allocate some Nodes. The fourth vector element is the node side.
      constexpr size_t num_nodes = 9+9+9;
      const Eigen::Vector4i nodes[num_nodes] = {
          {16, 16, 16, 16}, {32, 16, 16, 16}, {48, 16, 16, 16},
          {16, 32, 16, 16}, {32, 32, 16, 16}, {48, 32, 16, 16},
          {16, 48, 16, 16}, {32, 48, 16, 16}, {48, 48, 16, 16},
          {16, 16, 32, 16}, {32, 16, 32, 16}, {48, 16, 32, 16},
          {16, 32, 32, 16}, {32, 32, 32, 16}, {48, 32, 32, 16},
          {16, 48, 32, 16}, {32, 48, 32, 16}, {48, 48, 32, 16},
          {16, 16, 48, 16}, {32, 16, 48, 16}, {48, 16, 48, 16},
          {16, 32, 48, 16}, {32, 32, 48, 16}, {48, 32, 48, 16},
          {16, 48, 48, 16}, {32, 48, 48, 16}, {48, 48, 48, 16},
      };
      for (size_t i = 0; i< num_nodes; ++i) {
        const int node_level = std::log2(octree_->size() / nodes[i].w());
        se::Node<testT>* n = octree_->insert(
            nodes[i].x(), nodes[i].y(), nodes[i].z(), node_level);
        // Set the value of each child.
        for (size_t j = 0; j < 8; ++j) {
          const Eigen::Vector3i child_v = n->childCoordinates(j);
          // The value of each child is proportional to the sum of its
          // coordinates and side length.
          valT val;
          if (child_v.x() < child_v.y() ) {
            val.x = val_;
            if (n->child(j) == nullptr) {
              occupied_nodes_++;
              occupied_node_volume_ += std::pow(n->side_ / 2, 3) * single_voxel_volume;
            }
          } else {
            val.x = -val_;
            if (n->child(j) == nullptr) {
              free_nodes_++;
              free_node_volume_ += std::pow(n->side_ / 2, 3) * single_voxel_volume;
            }
          }
          n->value_[j] = val;
        }
      }

      // Export the list of allocated nodes and the created octree.
      //octree_->save("/tmp/test_map.bin");
      //octree_->writeAllocatedNodes("/tmp/test_nodes.txt");
    }

    static constexpr float val_ = 1.f;
    static constexpr int octree_size_ = 64;
    static constexpr float octree_dim_ = 10.f;

    std::shared_ptr<se::Octree<testT> > octree_;
    size_t free_voxels_;
    float  free_voxel_volume_;
    size_t occupied_voxels_;
    float  occupied_voxel_volume_;
    size_t free_nodes_;
    float  free_node_volume_;
    size_t occupied_nodes_;
    float  occupied_node_volume_;
};





TEST_F(TestOctree, CountVoxels) {
  // Number of voxels in the diagonal: 3^2 * BLOCK_SIDE^2 = 576
  // Total number of voxels:    3^3 * BLOCK_SIDE^3        = 13824
  // Number of free voxels:     (13824 - 576) / 2 + 576   = 7200
  // Number of occupied voxels: (13824 - 576) / 2         = 6624

  // Number of node children in the diagonal: 3^2 * 2^2   = 36
  // Total number of node children:           3^3 * 2^3   = 216
  // Number of free nodes:      (216 - 36) / 2 + 36 - 1   = 125
  // Number of occupied nodes:  (216 - 36) / 2            = 90

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

  EXPECT_EQ(      free_voxels,           free_voxels_);
  EXPECT_FLOAT_EQ(free_voxel_volume,     free_voxel_volume_);
  EXPECT_EQ(      occupied_voxels,       occupied_voxels_);
  EXPECT_FLOAT_EQ(occupied_voxel_volume, occupied_voxel_volume_);
  EXPECT_EQ(      free_nodes,            free_nodes_);
  EXPECT_FLOAT_EQ(free_node_volume,      free_node_volume_);
  EXPECT_EQ(      occupied_nodes,        occupied_nodes_);
  EXPECT_FLOAT_EQ(occupied_node_volume,  occupied_node_volume_);
}



TEST_F(TestOctree, CropOctreeExact) {
  // Crop the octree.
  const Eigen::Vector3f dim (64 * octree_->voxelDim(),
                             32 * octree_->voxelDim(),
                             16 * octree_->voxelDim());
  crop_octree(*octree_, dim);
  //octree_->save("/tmp/test_map_cropped.bin");
  //octree_->writeAllocatedNodes("/tmp/test_nodes_cropped.txt");

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

  // Compare with the exprected number of voxels.
  const float voxel_block_volume
      = std::pow(BLOCK_SIDE, 3.f) * std::pow(octree_->voxelDim(), 3.f);
  EXPECT_EQ(      free_voxels,           0);
  EXPECT_FLOAT_EQ(free_voxel_volume,     0.f);
  EXPECT_EQ(      occupied_voxels,       0);
  EXPECT_FLOAT_EQ(occupied_voxel_volume, 0.f);
  EXPECT_EQ(      free_nodes,            36);
  EXPECT_FLOAT_EQ(free_node_volume,      36 * voxel_block_volume);
  EXPECT_EQ(      occupied_nodes,        12);
  EXPECT_FLOAT_EQ(occupied_node_volume,  12 * voxel_block_volume);
}

