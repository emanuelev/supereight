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
class NeighborGatherTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      octree_.init(64, 1);

      // Allocate some VoxelBlocks.
      constexpr size_t num_voxel_blocks = 1;
      const Eigen::Vector3i blocks[num_voxel_blocks] = {{0, 0, 0}};
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

      // Set the child values of some Nodes.
      constexpr size_t num_nodes = 2;
      const Eigen::Vector3i nodes[num_nodes] = {{0, 0, 0}, {0, 0, 0}};
      const int node_levels[num_nodes] = {1, 2};
      for (size_t i = 0; i< num_nodes; ++i) {
        se::Node<testT>* n = octree_.fetch_octant(
            nodes[i].x(), nodes[i].y(), nodes[i].z(), node_levels[i]);
        for (size_t j = 0; j < 8; ++j) {
          n->value_[j] = value_increment_ * (j + 1);
        }
      }
    }

    typedef se::Octree<testT> OctreeF;
    OctreeF octree_;
    static constexpr float value_increment_ = 0.05f;
};



// Get the face neighbor values of a voxel located on the interior of a voxel
// block.
TEST_F(NeighborGatherTest, GetFaceNeighborValuesLocal) {
  // Safe version.
  std::array<voxel_traits<testT>::value_type, 7> neighbor_values_safe
      = octree_.get_face_neighbor_values<true>(1, 1, 1);
  // Unsafe version.
  std::array<voxel_traits<testT>::value_type, 7> neighbor_values_unsafe
      = octree_.get_face_neighbor_values<false>(1, 1, 1);

  // Voxel  0 (1, 1, 1).
  EXPECT_EQ(  neighbor_values_safe[0], 5 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[0], 5 * value_increment_);

  // Voxel -x (0, 1, 1).
  EXPECT_EQ(  neighbor_values_safe[1], 6 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[1], 6 * value_increment_);

  // Voxel +x (2, 1, 1).
  EXPECT_EQ(  neighbor_values_safe[2], 9 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[2], 9 * value_increment_);

  // Voxel -y (1, 0, 1).
  EXPECT_EQ(  neighbor_values_safe[3], 7 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[3], 7 * value_increment_);

  // Voxel +y (1, 2, 1).
  EXPECT_EQ(  neighbor_values_safe[4], 10 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[4], 10 * value_increment_);

  // Voxel -z (1, 1, 0).
  EXPECT_EQ(  neighbor_values_safe[5], 8 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[5], 8 * value_increment_);

  // Voxel +z (1, 1, 2).
  EXPECT_EQ(  neighbor_values_safe[6], 11 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[6], 11 * value_increment_);
}



// Get the face neighbor values of a voxel located on the corner of the volume.
// NOTE Neighbors outside the volume have a value of empty().
TEST_F(NeighborGatherTest, GetFaceNeighborValuesVolumeCorner) {
  // Safe version.
  std::array<voxel_traits<testT>::value_type, 7> neighbor_values_safe
      = octree_.get_face_neighbor_values<true>(0, 0, 0);
  // Unsafe version.
  std::array<voxel_traits<testT>::value_type, 7> neighbor_values_unsafe
      = octree_.get_face_neighbor_values<false>(0, 0, 0);

  // Voxel  0 (0, 0, 0), inside.
  EXPECT_EQ(  neighbor_values_safe[0], 1 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[0], 1 * value_increment_);

  // Voxel -x (-1, 0, 0), outside.
  EXPECT_EQ(  neighbor_values_safe[1], voxel_traits<testT>::empty());

  // Voxel +x (1, 0, 0), inside.
  EXPECT_EQ(  neighbor_values_safe[2], 2 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[2], 2 * value_increment_);

  // Voxel -y (0, -1, 0), outside.
  EXPECT_EQ(  neighbor_values_safe[3], voxel_traits<testT>::empty());

  // Voxel +y (0, 1, 0), inside.
  EXPECT_EQ(  neighbor_values_safe[4], 3 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[4], 3 * value_increment_);

  // Voxel -z (0, 0, -1), outside.
  EXPECT_EQ(  neighbor_values_safe[5], voxel_traits<testT>::empty());

  // Voxel +z (0, 0, 1), inside.
  EXPECT_EQ(  neighbor_values_safe[6], 4 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[6], 4 * value_increment_);
}



// Get the face neighbor values of a voxel located on the corner of a
// VoxelBlock and surrounded by unallocated VoxelBlocks.
// NOTE Unallocated neighbors have a value of initValue().
TEST_F(NeighborGatherTest, GetFaceNeighborValuesCornerUnallocated) {
  // Safe version.
  std::array<voxel_traits<testT>::value_type, 7> neighbor_values_safe
      = octree_.get_face_neighbor_values<true>(octree_.blockSide - 1,
      octree_.blockSide - 1, octree_.blockSide - 1);
  // Unsafe version.
  std::array<voxel_traits<testT>::value_type, 7> neighbor_values_unsafe
      = octree_.get_face_neighbor_values<false>(octree_.blockSide - 1,
      octree_.blockSide - 1, octree_.blockSide - 1);

  // Voxel  0 (7, 7, 7), allocated.
  EXPECT_EQ(  neighbor_values_safe[0], voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[0], voxel_traits<testT>::initValue());

  // Voxel -x (6, 7, 7), allocated.
  EXPECT_EQ(  neighbor_values_safe[1], voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[1], voxel_traits<testT>::initValue());

  // Voxel +x (8, 7, 7), unallocated.
  EXPECT_EQ(  neighbor_values_safe[2], voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[2], voxel_traits<testT>::initValue());

  // Voxel -y (7, 6, 7), allocated.
  EXPECT_EQ(  neighbor_values_safe[3], voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[3], voxel_traits<testT>::initValue());

  // Voxel +y (7, 8, 7), unallocated.
  EXPECT_EQ(  neighbor_values_safe[4], voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[4], voxel_traits<testT>::initValue());

  // Voxel -z (7, 7, 6), allocated.
  EXPECT_EQ(  neighbor_values_safe[5], voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[5], voxel_traits<testT>::initValue());

  // Voxel +z (7, 7, 8), unallocated.
  EXPECT_EQ(  neighbor_values_safe[6], voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[6], voxel_traits<testT>::initValue());
}



// Get the face neighbors of a voxel located on the interior of a voxel block.
TEST_F(NeighborGatherTest, GetFaceNeighborsLocal) {
  // Safe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_safe
      = octree_.getFaceNeighbors<true>(1, 1, 1);
  // Unsafe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_unsafe
      = octree_.getFaceNeighbors<false>(1, 1, 1);

  // Voxel  0 (1, 1, 1).
  EXPECT_EQ(  neighbor_values_safe[0].pos_,  Eigen::Vector3i(1, 1, 1));
  EXPECT_EQ(  neighbor_values_safe[0].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[0].data_, 5 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[0].pos_,  Eigen::Vector3i(1, 1, 1));
  EXPECT_EQ(neighbor_values_unsafe[0].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[0].data_, 5 * value_increment_);

  // Voxel -x (0, 1, 1).
  EXPECT_EQ(  neighbor_values_safe[1].pos_,  Eigen::Vector3i(0, 1, 1));
  EXPECT_EQ(  neighbor_values_safe[1].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[1].data_, 6 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[1].pos_,  Eigen::Vector3i(0, 1, 1));
  EXPECT_EQ(neighbor_values_unsafe[1].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[1].data_, 6 * value_increment_);

  // Voxel +x (2, 1, 1).
  EXPECT_EQ(  neighbor_values_safe[2].pos_,  Eigen::Vector3i(2, 1, 1));
  EXPECT_EQ(  neighbor_values_safe[2].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[2].data_, 9 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[2].pos_,  Eigen::Vector3i(2, 1, 1));
  EXPECT_EQ(neighbor_values_unsafe[2].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[2].data_, 9 * value_increment_);

  // Voxel -y (1, 0, 1).
  EXPECT_EQ(  neighbor_values_safe[3].pos_,  Eigen::Vector3i(1, 0, 1));
  EXPECT_EQ(  neighbor_values_safe[3].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[3].data_, 7 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[3].pos_,  Eigen::Vector3i(1, 0, 1));
  EXPECT_EQ(neighbor_values_unsafe[3].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[3].data_, 7 * value_increment_);

  // Voxel +y (1, 2, 1).
  EXPECT_EQ(  neighbor_values_safe[4].pos_,  Eigen::Vector3i(1, 2, 1));
  EXPECT_EQ(  neighbor_values_safe[4].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[4].data_, 10 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[4].pos_,  Eigen::Vector3i(1, 2, 1));
  EXPECT_EQ(neighbor_values_unsafe[4].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[4].data_, 10 * value_increment_);

  // Voxel -z (1, 1, 0).
  EXPECT_EQ(  neighbor_values_safe[5].pos_,  Eigen::Vector3i(1, 1, 0));
  EXPECT_EQ(  neighbor_values_safe[5].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[5].data_, 8 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[5].pos_,  Eigen::Vector3i(1, 1, 0));
  EXPECT_EQ(neighbor_values_unsafe[5].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[5].data_, 8 * value_increment_);

  // Voxel +z (1, 1, 2).
  EXPECT_EQ(  neighbor_values_safe[6].pos_,  Eigen::Vector3i(1, 1, 2));
  EXPECT_EQ(  neighbor_values_safe[6].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[6].data_, 11 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[6].pos_,  Eigen::Vector3i(1, 1, 2));
  EXPECT_EQ(neighbor_values_unsafe[6].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[6].data_, 11 * value_increment_);
}



// Get the face neighbors of a voxel located on the corner of the volume.
// NOTE Neighbors outside the volume have a value of empty().
TEST_F(NeighborGatherTest, GetFaceNeighborsVolumeCorner) {
  // Safe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_safe
      = octree_.getFaceNeighbors<true>(0, 0, 0);
  // Unsafe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_unsafe
      = octree_.getFaceNeighbors<false>(0, 0, 0);

  // Voxel  0 (0, 0, 0), inside.
  EXPECT_EQ(  neighbor_values_safe[0].pos_,  Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(  neighbor_values_safe[0].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[0].data_, 1 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[0].pos_,  Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(neighbor_values_unsafe[0].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[0].data_, 1 * value_increment_);

  // Voxel -x (-1, 0, 0), outside.
  EXPECT_EQ(  neighbor_values_safe[1].pos_,  Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(  neighbor_values_safe[1].side_, 0);
  EXPECT_EQ(  neighbor_values_safe[1].data_, voxel_traits<testT>::empty());

  // Voxel +x (1, 0, 0), inside.
  EXPECT_EQ(  neighbor_values_safe[2].pos_,  Eigen::Vector3i(1, 0, 0));
  EXPECT_EQ(  neighbor_values_safe[2].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[2].data_, 2 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[2].pos_,  Eigen::Vector3i(1, 0, 0));
  EXPECT_EQ(neighbor_values_unsafe[2].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[2].data_, 2 * value_increment_);

  // Voxel -y (0, -1, 0), outside.
  EXPECT_EQ(  neighbor_values_safe[3].pos_,  Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(  neighbor_values_safe[3].side_, 0);
  EXPECT_EQ(  neighbor_values_safe[3].data_, voxel_traits<testT>::empty());

  // Voxel +y (0, 1, 0), inside.
  EXPECT_EQ(  neighbor_values_safe[4].pos_,  Eigen::Vector3i(0, 1, 0));
  EXPECT_EQ(  neighbor_values_safe[4].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[4].data_, 3 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[4].pos_,  Eigen::Vector3i(0, 1, 0));
  EXPECT_EQ(neighbor_values_unsafe[4].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[4].data_, 3 * value_increment_);

  // Voxel -z (0, 0, -1), outside.
  EXPECT_EQ(  neighbor_values_safe[5].pos_,  Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(  neighbor_values_safe[5].side_, 0);
  EXPECT_EQ(  neighbor_values_safe[5].data_, voxel_traits<testT>::empty());

  // Voxel +z (0, 0, 1), inside.
  EXPECT_EQ(  neighbor_values_safe[6].pos_,  Eigen::Vector3i(0, 0, 1));
  EXPECT_EQ(  neighbor_values_safe[6].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[6].data_, 4 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[6].pos_,  Eigen::Vector3i(0, 0, 1));
  EXPECT_EQ(neighbor_values_unsafe[6].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[6].data_, 4 * value_increment_);
}



// Get the face neighbors of a voxel located on the corner of a VoxelBlock and
// surrounded by unallocated VoxelBlocks.
// NOTE Unallocated neighbors have a value of initValue().
TEST_F(NeighborGatherTest, GetFaceNeighborsCornerUnallocated) {
  // Safe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_safe
      = octree_.getFaceNeighbors<true>(octree_.blockSide - 1,
      octree_.blockSide - 1, octree_.blockSide - 1);
  // Unsafe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_unsafe
      = octree_.getFaceNeighbors<false>(octree_.blockSide - 1,
      octree_.blockSide - 1, octree_.blockSide - 1);

  // Voxel  0 (7, 7, 7), allocated.
  EXPECT_EQ(  neighbor_values_safe[0].pos_,  Eigen::Vector3i(7, 7, 7));
  EXPECT_EQ(  neighbor_values_safe[0].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[0].data_, voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[0].pos_,  Eigen::Vector3i(7, 7, 7));
  EXPECT_EQ(neighbor_values_unsafe[0].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[0].data_, voxel_traits<testT>::initValue());

  // Voxel -x (6, 7, 7), allocated.
  EXPECT_EQ(  neighbor_values_safe[1].pos_,  Eigen::Vector3i(6, 7, 7));
  EXPECT_EQ(  neighbor_values_safe[1].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[1].data_, voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[1].pos_,  Eigen::Vector3i(6, 7, 7));
  EXPECT_EQ(neighbor_values_unsafe[1].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[1].data_, voxel_traits<testT>::initValue());

  // Voxel +x (8, 7, 7), unallocated.
  EXPECT_EQ(  neighbor_values_safe[2].pos_,  Eigen::Vector3i(8, 0, 0));
  EXPECT_EQ(  neighbor_values_safe[2].side_, 8);
  EXPECT_EQ(  neighbor_values_safe[2].data_, 2 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[2].pos_,  Eigen::Vector3i(8, 0, 0));
  EXPECT_EQ(neighbor_values_unsafe[2].side_, 8);
  EXPECT_EQ(neighbor_values_unsafe[2].data_, 2 * value_increment_);

  // Voxel -y (7, 6, 7), allocated.
  EXPECT_EQ(  neighbor_values_safe[3].pos_,  Eigen::Vector3i(7, 6, 7));
  EXPECT_EQ(  neighbor_values_safe[3].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[3].data_, voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[3].pos_,  Eigen::Vector3i(7, 6, 7));
  EXPECT_EQ(neighbor_values_unsafe[3].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[3].data_, voxel_traits<testT>::initValue());

  // Voxel +y (7, 8, 7), unallocated.
  EXPECT_EQ(  neighbor_values_safe[4].pos_,  Eigen::Vector3i(0, 8, 0));
  EXPECT_EQ(  neighbor_values_safe[4].side_, 8);
  EXPECT_EQ(  neighbor_values_safe[4].data_, 3 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[4].pos_,  Eigen::Vector3i(0, 8, 0));
  EXPECT_EQ(neighbor_values_unsafe[4].side_, 8);
  EXPECT_EQ(neighbor_values_unsafe[4].data_, 3 * value_increment_);

  // Voxel -z (7, 7, 6), allocated.
  EXPECT_EQ(  neighbor_values_safe[5].pos_,  Eigen::Vector3i(7, 7, 6));
  EXPECT_EQ(  neighbor_values_safe[5].side_, 1);
  EXPECT_EQ(  neighbor_values_safe[5].data_, voxel_traits<testT>::initValue());
  EXPECT_EQ(neighbor_values_unsafe[5].pos_,  Eigen::Vector3i(7, 7, 6));
  EXPECT_EQ(neighbor_values_unsafe[5].side_, 1);
  EXPECT_EQ(neighbor_values_unsafe[5].data_, voxel_traits<testT>::initValue());

  // Voxel +z (7, 7, 8), unallocated.
  EXPECT_EQ(  neighbor_values_safe[6].pos_,  Eigen::Vector3i(0, 0, 8));
  EXPECT_EQ(  neighbor_values_safe[6].side_, 8);
  EXPECT_EQ(  neighbor_values_safe[6].data_, 5 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[6].pos_,  Eigen::Vector3i(0, 0, 8));
  EXPECT_EQ(neighbor_values_unsafe[6].side_, 8);
  EXPECT_EQ(neighbor_values_unsafe[6].data_, 5 * value_increment_);
}



// Get the face neighbors of a voxel located in a region that has not been
// allocated at the VoxelBlock level.
TEST_F(NeighborGatherTest, GetFaceNeighborsUnallocated) {
  // Safe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_safe
      = octree_.getFaceNeighbors<true>(octree_.blockSide,
      octree_.blockSide, octree_.blockSide);
  // Unsafe version.
  se::VoxelAbstrationArray<voxel_traits<testT>::value_type, 7> neighbor_values_unsafe
      = octree_.getFaceNeighbors<false>(octree_.blockSide,
      octree_.blockSide, octree_.blockSide);

  // Voxel  0 (8, 8, 8), allocated.
  EXPECT_EQ(  neighbor_values_safe[0].pos_,  Eigen::Vector3i(8, 8, 8));
  EXPECT_EQ(  neighbor_values_safe[0].side_, 8);
  EXPECT_EQ(  neighbor_values_safe[0].data_, 8 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[0].pos_,  Eigen::Vector3i(8, 8, 8));
  EXPECT_EQ(neighbor_values_unsafe[0].side_, 8);
  EXPECT_EQ(neighbor_values_unsafe[0].data_, 8 * value_increment_);

  // Voxel -x (7, 8, 8), unallocated.
  EXPECT_EQ(  neighbor_values_safe[1].pos_,  Eigen::Vector3i(0, 8, 8));
  EXPECT_EQ(  neighbor_values_safe[1].side_, 8);
  EXPECT_EQ(  neighbor_values_safe[1].data_, 7 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[1].pos_,  Eigen::Vector3i(0, 8, 8));
  EXPECT_EQ(neighbor_values_unsafe[1].side_, 8);
  EXPECT_EQ(neighbor_values_unsafe[1].data_, 7 * value_increment_);

  // Voxel +x (9, 8, 8), unallocated.
  EXPECT_EQ(  neighbor_values_safe[2].pos_,  Eigen::Vector3i(16, 0, 0));
  EXPECT_EQ(  neighbor_values_safe[2].side_, 16);
  EXPECT_EQ(  neighbor_values_safe[2].data_, 2 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[2].pos_,  Eigen::Vector3i(16, 0, 0));
  EXPECT_EQ(neighbor_values_unsafe[2].side_, 16);
  EXPECT_EQ(neighbor_values_unsafe[2].data_, 2 * value_increment_);

  // Voxel -y (8, 7, 8), unallocated.
  EXPECT_EQ(  neighbor_values_safe[3].pos_,  Eigen::Vector3i(8, 0, 8));
  EXPECT_EQ(  neighbor_values_safe[3].side_, 8);
  EXPECT_EQ(  neighbor_values_safe[3].data_, 6 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[3].pos_,  Eigen::Vector3i(8, 0, 8));
  EXPECT_EQ(neighbor_values_unsafe[3].side_, 8);
  EXPECT_EQ(neighbor_values_unsafe[3].data_, 6 * value_increment_);

  // Voxel +y (8, 9, 8), unallocated.
  EXPECT_EQ(  neighbor_values_safe[4].pos_,  Eigen::Vector3i(0, 16, 0));
  EXPECT_EQ(  neighbor_values_safe[4].side_, 16);
  EXPECT_EQ(  neighbor_values_safe[4].data_, 3 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[4].pos_,  Eigen::Vector3i(0, 16, 0));
  EXPECT_EQ(neighbor_values_unsafe[4].side_, 16);
  EXPECT_EQ(neighbor_values_unsafe[4].data_, 3 * value_increment_);

  // Voxel -z (8, 8, 7), unallocated.
  EXPECT_EQ(  neighbor_values_safe[5].pos_,  Eigen::Vector3i(8, 8, 0));
  EXPECT_EQ(  neighbor_values_safe[5].side_, 8);
  EXPECT_EQ(  neighbor_values_safe[5].data_, 4 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[5].pos_,  Eigen::Vector3i(8, 8, 0));
  EXPECT_EQ(neighbor_values_unsafe[5].side_, 8);
  EXPECT_EQ(neighbor_values_unsafe[5].data_, 4 * value_increment_);

  // Voxel +z (8, 8, 9), unallocated.
  EXPECT_EQ(  neighbor_values_safe[6].pos_,  Eigen::Vector3i(0, 0, 16));
  EXPECT_EQ(  neighbor_values_safe[6].side_, 16);
  EXPECT_EQ(  neighbor_values_safe[6].data_, 5 * value_increment_);
  EXPECT_EQ(neighbor_values_unsafe[6].pos_,  Eigen::Vector3i(0, 0, 16));
  EXPECT_EQ(neighbor_values_unsafe[6].side_, 16);
  EXPECT_EQ(neighbor_values_unsafe[6].data_, 5 * value_increment_);
}

