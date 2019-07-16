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


#include <array>

#include "octree.hpp"
#include "utils/math_utils.h"
#include "neighbors/neighbor_gather.hpp"
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
class NeighborGatherTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      octree_.init(64, 1);

      // Allocate some VoxelBlocks.
      constexpr size_t num_voxel_blocks = 2;
      const Eigen::Vector3i blocks[num_voxel_blocks] =
          {{0, 0, 0},
           {octree_.blockSide, octree_.blockSide, octree_.blockSide}};
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
    }

    typedef se::Octree<testT> OctreeF;
    OctreeF octree_;
    static constexpr float value_increment_ = 0.05f;
};



// Get the face neighbor values of a voxel located on the interior of a voxel
// block.
TEST_F(NeighborGatherTest, GetFaceNeighborsLocal) {
  std::array<voxel_traits<testT>::value_type, 6> neighbor_values
      = octree_.get_face_neighbors(1, 1, 1);

  // Voxel -x (0, 1, 1).
  EXPECT_EQ(neighbor_values[0], 6 * value_increment_);

  // Voxel +x (2, 1, 1).
  EXPECT_EQ(neighbor_values[1], 9 * value_increment_);

  // Voxel -y (1, 0, 1).
  EXPECT_EQ(neighbor_values[2], 7 * value_increment_);

  // Voxel +y (1, 2, 1).
  EXPECT_EQ(neighbor_values[3], 10 * value_increment_);

  // Voxel -z (1, 1, 0).
  EXPECT_EQ(neighbor_values[4], 8 * value_increment_);

  // Voxel +z (1, 1, 2).
  EXPECT_EQ(neighbor_values[5], 11 * value_increment_);
}



// Get the face neighbor values of a voxel located on the corner of the volume.
// NOTE Neighbors outside the volume have a value of initValue().
TEST_F(NeighborGatherTest, GetFaceNeighborsVolumeCorner) {
  std::array<voxel_traits<testT>::value_type, 6> neighbor_values
      = octree_.get_face_neighbors(0, 0, 0);

  // Voxel -x (-1, 0, 0).
  EXPECT_EQ(neighbor_values[0], voxel_traits<testT>::initValue());

  // Voxel +x (1, 0, 0).
  EXPECT_EQ(neighbor_values[1], 2 * value_increment_);

  // Voxel -y (0, -1, 0).
  EXPECT_EQ(neighbor_values[2], voxel_traits<testT>::initValue());

  // Voxel +y (0, 1, 0).
  EXPECT_EQ(neighbor_values[3], 3 * value_increment_);

  // Voxel -z (0, 0, -1).
  EXPECT_EQ(neighbor_values[4], voxel_traits<testT>::initValue());

  // Voxel +z (0, 0, 1).
  EXPECT_EQ(neighbor_values[5], 4 * value_increment_);
}

