/*

Copyright 2016 Emanuele Vespa, Imperial College London 

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

*/
#include "octree.hpp"
#include "utils/math_utils.h"
#include "interpolation/interp_gather.hpp"
#include "gtest/gtest.h"
#include "node_iterator.hpp"

typedef float testT;

template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return 0.f; }
  static inline value_type initValue(){ return 1.f; }
};

class GatherTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      oct_.init(512, 5);
      const Eigen::Vector3i blocks[10] = {{56, 12, 254}, {87, 32, 423}, {128, 128, 128},
      {136, 128, 128}, {128, 136, 128}, {136, 136, 128}, 
      {128, 128, 136}, {136, 128, 136}, {128, 136, 136}, {136, 136, 136}};
      se::key_t alloc_list[10];
      for(int i = 0; i < 10; ++i) {
        alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2));
      }
      oct_.allocate(alloc_list, 10);
    }

  typedef se::Octree<testT> OctreeF;
  OctreeF oct_;
};

TEST_F(GatherTest, Init) {
  EXPECT_EQ(oct_.get(137, 138, 130), voxel_traits<testT>::initValue());
}

TEST_F(GatherTest, GatherLocal) {
  float points[8];
  const Eigen::Vector3i base = {136, 128, 136};
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, ZCrosses) {
  float points[8];
  const unsigned blockSize = se::VoxelBlock<testT>::side;
  const Eigen::Vector3i base = {132, 128, 135};
  unsigned int crossmask = ((base(0) % blockSize) == blockSize - 1 << 2) | 
                           ((base(1) % blockSize) == blockSize - 1 << 1) |
                            (base(2) % blockSize) == blockSize - 1;
  ASSERT_EQ(crossmask, 1);
  se::VoxelBlock<testT> * block = oct_.fetch(base(0), base(1), base(2));
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, YCrosses) {
  float points[8];
  const unsigned blockSize = se::VoxelBlock<testT>::side;
  const Eigen::Vector3i base = {132, 135, 132};
  unsigned int crossmask = ((base(0) % blockSize == blockSize - 1) << 2) | 
                           ((base(1) % blockSize == blockSize - 1) << 1) |
                            ((base(2) % blockSize) == blockSize - 1);
  ASSERT_EQ(crossmask, 2);
  se::VoxelBlock<testT> * block = oct_.fetch(base(0), base(1), base(2));
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, XCrosses) {
  float points[8];
  const unsigned blockSize = se::VoxelBlock<testT>::side;
  const Eigen::Vector3i base = {135, 132, 132};
  unsigned int crossmask = ((base(0) % blockSize == blockSize - 1) << 2) | 
                           ((base(1) % blockSize == blockSize - 1) << 1) |
                            ((base(2) % blockSize) == blockSize - 1);
  ASSERT_EQ(crossmask, 4);
  se::VoxelBlock<testT> * block = oct_.fetch(base(0), base(1), base(2));
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, YZCross) {
  float points[8];
  const unsigned blockSize = se::VoxelBlock<testT>::side;
  const Eigen::Vector3i base = {129, 135, 135};
  unsigned int crossmask = ((base(0) % blockSize == blockSize - 1) << 2) | 
                           ((base(1) % blockSize == blockSize - 1) << 1) |
                            ((base(2) % blockSize) == blockSize - 1);
  ASSERT_EQ(crossmask, 3);
  se::VoxelBlock<testT> * block = oct_.fetch(base(0), base(1), base(2));
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, XZCross) {
  float points[8];
  const unsigned blockSize = se::VoxelBlock<testT>::side;
  const Eigen::Vector3i base = {135, 131, 135};
  unsigned int crossmask = ((base(0) % blockSize == blockSize - 1) << 2) | 
                           ((base(1) % blockSize == blockSize - 1) << 1) |
                            ((base(2) % blockSize) == blockSize - 1);
  ASSERT_EQ(crossmask, 5);
  se::VoxelBlock<testT> * block = oct_.fetch(base(0), base(1), base(2));
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, XYCross) {
  float points[8];
  const unsigned blockSize = se::VoxelBlock<testT>::side;
  const Eigen::Vector3i base = {135, 135, 138};
  unsigned int crossmask = ((base(0) % blockSize == blockSize - 1) << 2) | 
                           ((base(1) % blockSize == blockSize - 1) << 1) |
                            ((base(2) % blockSize) == blockSize - 1);
  ASSERT_EQ(crossmask, 6);
  se::VoxelBlock<testT> * block = oct_.fetch(base(0), base(1), base(2));
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, AllCross) {
  float points[8];
  const unsigned blockSize = se::VoxelBlock<testT>::side;
  const Eigen::Vector3i base = {135, 135, 135};
  unsigned int crossmask = ((base(0) % blockSize == blockSize - 1) << 2) | 
                           ((base(1) % blockSize == blockSize - 1) << 1) |
                            ((base(2) % blockSize) == blockSize - 1);
  ASSERT_EQ(crossmask, 7);
  se::VoxelBlock<testT> * block = oct_.fetch(base(0), base(1), base(2));
  se::internal::gather_points(oct_, base, [](const auto& val){ return val; }, points);

  for(int i = 0; i < 8; ++i) {
    EXPECT_EQ(points[i], voxel_traits<testT>::initValue());
  }
}

TEST_F(GatherTest, FetchWithStack) {
  se::Node<testT>* stack[CAST_STACK_DEPTH] = {nullptr};
  int max_depth = se::math::log2_const(oct_.size());
  se::internal::fetch(stack, oct_.root(), max_depth, 
      Eigen::Vector3i(128, 136, 128));
  auto res = oct_.fetch(128, 136, 128);
  int l = 0;
  while(stack[l] && stack[l + 1] != nullptr) {
    ASSERT_TRUE(se::parent(stack[l + 1]->code_, max_depth) == stack[l]->code_);
    ++l;
  }
}

TEST_F(GatherTest, FetchNeighbourhood) {
  Eigen::Vector3i pos0(128,   0, 0); // (1, 0, 0) 
  Eigen::Vector3i pos1(0,   128, 0); // (0, 1, 0)
  Eigen::Vector3i pos2(128, 128, 0); // (1, 1, 0)
  Eigen::Vector3i base(64,   64, 0);
  oct_.insert(pos0.x(), pos0.y(), pos0.z(), 2);
  oct_.insert(pos1.x(), pos1.y(), pos1.z(), 2);
  oct_.insert(pos2.x(), pos2.y(), pos2.z(), 2);
  oct_.insert(base.x(), base.y(), base.z(), 3);

  se::Node<testT>* stack[CAST_STACK_DEPTH] = {nullptr};
  int max_depth = se::math::log2_const(oct_.size());
  auto base_ptr = se::internal::fetch(stack, oct_.root(), max_depth, base);
  auto n1 = se::internal::fetch_neighbour(stack, base_ptr, max_depth, 1);
  auto n2 = se::internal::fetch_neighbour(stack, base_ptr, max_depth, 2);
  auto n3 = se::internal::fetch_neighbour(stack, base_ptr, max_depth, 3);
  ASSERT_TRUE(se::keyops::decode(base_ptr->code_) == base);
  ASSERT_TRUE(se::keyops::decode(n1->code_) == pos0);
  ASSERT_TRUE(se::keyops::decode(n2->code_) == pos1);
  ASSERT_TRUE(se::keyops::decode(n3->code_) == pos2);

  std::fill(std::begin(stack), std::end(stack), nullptr);
  base_ptr = se::internal::fetch(stack, oct_.root(), max_depth, pos0); 
  auto failed = se::internal::fetch_neighbour(stack, base_ptr, max_depth, 1);
  ASSERT_TRUE(failed == nullptr);
}
