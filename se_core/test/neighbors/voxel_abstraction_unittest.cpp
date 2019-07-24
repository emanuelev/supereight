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


#include <cmath>

#include "octree.hpp"
#include "utils/math_utils.h"
#include "neighbors/voxel_abstraction.hpp"
#include "gtest/gtest.h"

typedef float testT;

// Create a voxel trait storing a single float value.
template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return 0.f; }
  static inline value_type initValue(){ return 1.f; }
};



// Initialize an octree and store some values inside.
class VoxelAbstractionTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      octree_.init(64, 1);

      // Allocate some VoxelBlocks.
      constexpr size_t num_voxel_blocks = 2;
      const Eigen::Vector3i blocks[num_voxel_blocks] = {{0, 0, 0},
          {32, 32, 32}};
      se::key_t alloc_list[num_voxel_blocks];
      for (size_t i = 0; i < num_voxel_blocks; ++i) {
        alloc_list[i] = octree_.hash(blocks[i](0), blocks[i](1), blocks[i](2));
      }
      octree_.allocate(alloc_list, num_voxel_blocks);

      // Set the values of some voxels.
      constexpr size_t num_voxels = 11;
      const Eigen::Vector3i voxels[num_voxels] =
          {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1},
           {0, 1, 1}, {1, 0, 1}, {1, 1, 0}, {2, 1, 1}, {1, 2, 1},
           {1, 1, 2}};
      for (size_t i = 0; i< num_voxels; ++i) {
        // The values start from value_increment_ and increase by
        // value_increment_ for each next voxel.
        octree_.set(voxels[i].x(), voxels[i].y(), voxels[i].z(),
            value_increment_ * (i + 1));
      }



      // Create some VoxelAbstrations.
      va_whole_map_.pos_ = Eigen::Vector3i(0, 0, 0);
      va_whole_map_.side_ = octree_.size();

      va_node_.pos_ = Eigen::Vector3i(32, 32, 32);
      va_node_.side_ = octree_.size() / 2;

      va_voxel_block_.pos_ = Eigen::Vector3i(0, 0, 0);
      va_voxel_block_.side_ = BLOCK_SIDE;

      va_single_voxel_.pos_ = Eigen::Vector3i(2, 1, 3);
      va_single_voxel_.side_ = 1;
    }

    se::Octree<testT> octree_;
    se::VoxelAbstration<testT> va_whole_map_;
    se::VoxelAbstration<testT> va_node_;
    se::VoxelAbstration<testT> va_voxel_block_;
    se::VoxelAbstration<testT> va_single_voxel_;
    static constexpr float value_increment_ = 0.05f;
};



// Get the center in map coordinates of some VoxelAbstrations.
TEST_F(VoxelAbstractionTest, Center) {
  // Compute the center of the whole map.
  const Eigen::Vector3f va_whole_map_center
      = va_whole_map_.center(octree_.voxelDim());
  EXPECT_FLOAT_EQ(va_whole_map_center.x(), octree_.dim() / 2);
  EXPECT_FLOAT_EQ(va_whole_map_center.y(), octree_.dim() / 2);
  EXPECT_FLOAT_EQ(va_whole_map_center.z(), octree_.dim() / 2);

  // Compute the center of a Node.
  const Eigen::Vector3f va_node_center
      = va_node_.center(octree_.voxelDim());
  EXPECT_FLOAT_EQ(va_node_center.x(), octree_.dim() / 2 + octree_.dim() / 4);
  EXPECT_FLOAT_EQ(va_node_center.y(), octree_.dim() / 2 + octree_.dim() / 4);
  EXPECT_FLOAT_EQ(va_node_center.z(), octree_.dim() / 2 + octree_.dim() / 4);

  // Compute the center of a VoxelBlock.
  const Eigen::Vector3f va_voxel_block_center
      = va_voxel_block_.center(octree_.voxelDim());
  EXPECT_FLOAT_EQ(va_voxel_block_center.x(), octree_.dim() / 16);
  EXPECT_FLOAT_EQ(va_voxel_block_center.y(), octree_.dim() / 16);
  EXPECT_FLOAT_EQ(va_voxel_block_center.z(), octree_.dim() / 16);

  // Compute the center of a single voxel.
  const Eigen::Vector3f va_single_voxel_center
      = va_single_voxel_.center(octree_.voxelDim());
  EXPECT_FLOAT_EQ(va_single_voxel_center.x(), 2 * octree_.voxelDim()
      + octree_.voxelDim() / 2);
  EXPECT_FLOAT_EQ(va_single_voxel_center.y(), 1 * octree_.voxelDim()
      + octree_.voxelDim() / 2);
  EXPECT_FLOAT_EQ(va_single_voxel_center.z(), 3 * octree_.voxelDim()
      + octree_.voxelDim() / 2);
}



// Get the level of some VoxelAbstrations.
TEST_F(VoxelAbstractionTest, Level) {
  const int octree_size = octree_.size();
  EXPECT_EQ(va_whole_map_.level(octree_size),    0);
  EXPECT_EQ(va_node_.level(octree_size),         1);
  EXPECT_EQ(va_voxel_block_.level(octree_size),  3);
  EXPECT_EQ(va_single_voxel_.level(octree_size), 6);
}



// Get the dimensions of some VoxelAbstrations.
TEST_F(VoxelAbstractionTest, Dim) {
  const float voxel_dim = octree_.voxelDim();
  EXPECT_EQ(va_whole_map_.dim(voxel_dim),    octree_.dim());
  EXPECT_EQ(va_node_.dim(voxel_dim),         octree_.dim() / 2);
  EXPECT_EQ(va_voxel_block_.dim(voxel_dim),  octree_.dim() / 8);
  EXPECT_EQ(va_single_voxel_.dim(voxel_dim), voxel_dim);
}



// Get the volume of some VoxelAbstrations.
TEST_F(VoxelAbstractionTest, Volume) {
  // Volume in voxels^3.
  EXPECT_EQ(va_whole_map_.volume(),
      static_cast<int>(std::pow(octree_.size(), 3)));
  EXPECT_EQ(va_node_.volume(),
      static_cast<int>(std::pow(octree_.size() / 2, 3)));
  EXPECT_EQ(va_voxel_block_.volume(),
      static_cast<int>(std::pow(octree_.size() / 8, 3)));
  EXPECT_EQ(va_single_voxel_.volume(), 1);

  // Volume in map units^3.
  const float voxel_dim = octree_.voxelDim();
  EXPECT_FLOAT_EQ(va_whole_map_.volume(voxel_dim),
      std::pow(octree_.dim(), 3));
  EXPECT_FLOAT_EQ(va_node_.volume(voxel_dim),
      std::pow(octree_.dim() / 2, 3));
  EXPECT_FLOAT_EQ(va_voxel_block_.volume(voxel_dim),
      std::pow(octree_.dim() / 8, 3));
  EXPECT_FLOAT_EQ(va_single_voxel_.volume(voxel_dim), std::pow(voxel_dim, 3));
}



// Create some VoxelAbstration containers.
TEST_F(VoxelAbstractionTest, Containers) {
  // Create std::array.
  se::VoxelAbstrationArray<testT, 2> arr;
  arr[0] = va_whole_map_;
  arr[1] = va_single_voxel_;
  EXPECT_EQ(arr.size(), 2);
  EXPECT_EQ(arr[0].side_, va_whole_map_.side_);
  EXPECT_EQ(arr[1].side_, va_single_voxel_.side_);

  // Create std::vector.
  se::VoxelAbstrationVector<testT> vec;
  vec.push_back(va_node_);
  vec.push_back(va_voxel_block_);
  EXPECT_EQ(vec.size(), 2);
  EXPECT_EQ(vec[0].side_, va_node_.side_);
  EXPECT_EQ(vec[1].side_, va_voxel_block_.side_);
}

