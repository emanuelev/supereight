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


#include "octree.hpp"
#include "utils/math_utils.h"
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
class OctreeTest : public ::testing::Test {
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
      for (size_t i = 0; i < num_allocated_voxels_; ++i) {
        // The values start from value_increment_ and increase by
        // value_increment_ for each next voxel.
        octree_.set(allocated_voxels_[i].x(),
                    allocated_voxels_[i].y(),
                    allocated_voxels_[i].z(),
                    value_increment_ * (i + 1));
      }

      // Set the values of some Nodes.
      // TODO set the values of some non-leaf nodes to properly test get on
      // unallocated voxels.
    }

    se::Octree<testT> octree_;
    static constexpr float value_increment_ = 0.05f;

    // Voxels that have been allocated and their value changed from the initial
    // value.
    static constexpr size_t num_allocated_voxels_ = 10;
    const Eigen::Vector3i allocated_voxels_[num_allocated_voxels_] =
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1},
         {0, 1, 1}, {1, 0, 1}, {1, 1, 0}, {2, 1, 1}, {1, 2, 1}};

    // Voxels that have not been allocated but the value of the lowest level
    // allocated parent Node has been changed from the initial value.
    static constexpr size_t num_unallocated_voxels_ = 5;
    const Eigen::Vector3i unallocated_voxels_[num_unallocated_voxels_] =
        {{16, 0, 1}, {3, 21, 8}, {5, 9, 1}, {18, 32, 8}, {45, 2, 55}};
    const Eigen::Vector3i parent_nodes_[num_unallocated_voxels_] =
        {{16, 0, 0}, {0, 16, 0}, {0, 8, 0}, {0, 32, 0}, {32, 0, 32}};
    const int parent_node_side_[num_unallocated_voxels_] =
        {8, 8, 4, 16, 16};
};



// Call get on allocated voxels.
TEST_F(OctreeTest, GetAllocated) {
  for (size_t i = 0; i < num_allocated_voxels_; ++i) {
    EXPECT_FLOAT_EQ(octree_.get(allocated_voxels_[i].x(),
                                allocated_voxels_[i].y(),
                                allocated_voxels_[i].z()),
                                value_increment_ * (i + 1));
  }
}



// Call get on unallocated voxels.
TEST_F(OctreeTest, GetUnallocated) {
  for (size_t i = 0; i < num_unallocated_voxels_; ++i) {
    EXPECT_FLOAT_EQ(octree_.get(unallocated_voxels_[i].x(),
                                unallocated_voxels_[i].y(),
                                unallocated_voxels_[i].z()),
                                voxel_traits<testT>::initValue());
  }
}



// Call get_fine on allocated voxels.
TEST_F(OctreeTest, GetFineAllocated) {
  for (size_t i = 0; i < num_allocated_voxels_; ++i) {
    EXPECT_FLOAT_EQ(octree_.get_fine(allocated_voxels_[i].x(),
                                     allocated_voxels_[i].y(),
                                     allocated_voxels_[i].z()),
                                     value_increment_ * (i + 1));
  }
}



// Call get_fine on unallocated voxels.
TEST_F(OctreeTest, GetFineUnallocated) {
  for (size_t i = 0; i < num_unallocated_voxels_; ++i) {
    EXPECT_FLOAT_EQ(octree_.get_fine(unallocated_voxels_[i].x(),
                                     unallocated_voxels_[i].y(),
                                     unallocated_voxels_[i].z()),
                                     voxel_traits<testT>::initValue());
  }
}



// Call get_lowest_as_voxel on allocated voxels.
TEST_F(OctreeTest, GetLowestAsVoxelAllocated) {
  for (size_t i = 0; i < num_allocated_voxels_; ++i) {
    se::VoxelAbstration<testT> va = octree_.getLowestAsVoxel(
        allocated_voxels_[i].x(),
        allocated_voxels_[i].y(),
        allocated_voxels_[i].z());
    EXPECT_EQ(va.pos_.x(),    allocated_voxels_[i].x());
    EXPECT_EQ(va.pos_.y(),    allocated_voxels_[i].y());
    EXPECT_EQ(va.pos_.z(),    allocated_voxels_[i].z());
    EXPECT_EQ(va.side_,       1);
    EXPECT_FLOAT_EQ(va.data_, value_increment_ * (i + 1));
  }
}



// Call get_lowest_as_voxel on unallocated voxels.
TEST_F(OctreeTest, GetLowestAsVoxelUnallocated) {
  for (size_t i = 0; i < num_unallocated_voxels_; ++i) {
    se::VoxelAbstration<testT> va = octree_.getLowestAsVoxel(
        unallocated_voxels_[i].x(),
        unallocated_voxels_[i].y(),
        unallocated_voxels_[i].z());
    EXPECT_EQ(va.pos_.x(),    parent_nodes_[i].x());
    EXPECT_EQ(va.pos_.y(),    parent_nodes_[i].y());
    EXPECT_EQ(va.pos_.z(),    parent_nodes_[i].z());
    EXPECT_EQ(va.side_,       parent_node_side_[i]);
    EXPECT_FLOAT_EQ(va.data_, voxel_traits<testT>::initValue());
  }
}

