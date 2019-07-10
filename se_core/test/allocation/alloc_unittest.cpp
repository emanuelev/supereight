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
#include <random>
#include "octree.hpp"
#include "node.hpp"
#include "utils/math_utils.h"
#include "utils/morton_utils.hpp"
#include "gtest/gtest.h"

typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > vec3i;
template<>
struct voxel_traits<float> {
  typedef float value_type;
  static inline value_type empty() { return 0.f; }
  static inline value_type initValue() { return 0.f; }
};

TEST(AllocationTest, EmptySingleVoxel) {
  typedef se::Octree<float> OctreeF;
  OctreeF oct;
  oct.init(256, 5);
  const Eigen::Vector3i vox = {25, 65, 127};
  const se::key_t code = oct.hash(vox(0), vox(1), vox(2));
  se::key_t allocList[1] = {code};
  const float val = oct.get(vox(0), vox(1), vox(2));
  EXPECT_EQ(val, voxel_traits<float>::empty());
}

TEST(AllocationTest, SetSingleVoxel) {
  typedef se::Octree<float> OctreeF;
  OctreeF oct;
  oct.init(256, 5);
  const Eigen::Vector3i vox = {25, 65, 127};
  const se::key_t code = oct.hash(vox(0), vox(1), vox(2));
  se::key_t allocList[1] = {code};
  oct.allocate(allocList, 1);

  se::VoxelBlock<float> *block = oct.fetch(vox(0), vox(1), vox(2));
  float written_val = 2.f;
  block->data(vox, written_val);

  const float read_val = oct.get(vox(0), vox(1), vox(2));
  EXPECT_EQ(written_val, read_val);
}

TEST(AllocationTest, FetchOctant_NodetoVoxelblock) {
  typedef se::Octree<float> OctreeF;
  OctreeF oct;
  const int max_level = 8;
  const unsigned int block_side = 8;
  const int leaves_level = max_level - log2(block_side);
  const unsigned int size = std::pow(2, max_level);
  oct.init(size, 5);
  const Eigen::Vector3i vox = {25, 65, 127};
  const Eigen::Vector3i vox_block_coord = {24, 64, 120};
  const se::key_t code = oct.hash(vox(0), vox(1), vox(2));
  se::key_t allocList[1] = {code};
  oct.allocate(allocList, 1);

  se::Node<float> *node = nullptr;
  bool is_voxel_block;
  testing::internal::CaptureStdout();
  std::cout << "My test \n";
  oct.fetch_octant(vox(0), vox(1), vox(2), node, is_voxel_block);
  Eigen::Vector3i coord(0, 0, 0);

  std::cout << " code " << node->code_<< " side " << node->side_ <<  std::endl;
  if (is_voxel_block) {
    se::VoxelBlock<float> *block = static_cast<se::VoxelBlock<float> *> (node);
    coord = block->coordinates();
  }
  std::string output = testing::internal::GetCapturedStdout();
//  EXPECT_TRUE(false)<< output;
  ASSERT_EQ(vox_block_coord, coord);
}

TEST(AllocationTest, FetchOctant_NoVoxelblock) {
  // WHEN octree with voxel blocks are initialized
  typedef se::Octree<float> OctreeF;
  OctreeF oct;
  const int max_level = 8;
  const unsigned int block_side = 8;
  const int leaves_level = max_level - log2(block_side);
  const unsigned int size = std::pow(2, max_level);
  oct.init(size, 5);
  const Eigen::Vector3i vox = {25, 65, 127};
  const se::key_t code = oct.hash(vox(0), vox(1), vox(2));
  se::key_t allocList[1] = {code};
  oct.allocate(allocList, 1);

  const vec3i vox_test {{7,65,127}, {11, 65,127}, {18, 65, 127}};
  const std::vector<int> node_side  {32, 32, 16};

  // IF  fetching the octant of an allocated voxel in a voxel block
  se::Node<float> *node = nullptr;
  bool is_voxel_block;
  testing::internal::CaptureStdout();
  std::cout << "My test \n";
  int i = 0;
  for(auto vox_t : vox_test) {
    std::cout << "before ";
    oct.fetch_octant(vox_t(0),
                     vox_t(1),
                     vox_t(2),
                     node,
                     is_voxel_block);
    Eigen::Vector3i coord(0, 0, 0);
    std::cout << " code " << node->code_ << " side " << node->side_ << std::endl;
    if (is_voxel_block) {
      se::VoxelBlock<float> *block = static_cast<se::VoxelBlock<float> *> (node);
      coord = block->coordinates();
    }
    std::cout << "coord " << se::keyops::decode(node->code_)<< std::endl;
    ASSERT_EQ(node_side[i], node->side_);
    i++;
  }
  std::string output = testing::internal::GetCapturedStdout();
//  EXPECT_TRUE(false) << output;
}

TEST(AllocationTest, FetchOctant) {
  typedef se::Octree<float> OctreeF;
  OctreeF oct;
  const int max_level = 8;
  const unsigned int block_side = 8;
  const int leaves_level = max_level - log2(block_side);
  const unsigned int size = std::pow(2, max_level);
  oct.init(size, 5);
  const Eigen::Vector3i vox = {25, 65, 127};
  const se::key_t code = oct.hash(vox(0), vox(1), vox(2));
  se::key_t allocList[1] = {code};

  oct.allocate(allocList, 1);

  const int level = 3; /* 32 voxels per side */
  se::Node<float> *node = oct.fetch_octant(vox(0), vox(1), vox(2), level);
  se::key_t fetched_code = node->code_;

  const se::key_t gt_code = oct.hash(vox(0), vox(1), vox(2), level);
  ASSERT_EQ(fetched_code, gt_code);
}

TEST(AllocationTest, MortonPrefixMask) {

  const unsigned int max_bits = 21;
  const unsigned int block_side = 8;
  const unsigned int size = std::pow(2, max_bits);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0, size);

  constexpr int num_samples = 10;
  se::key_t keys[num_samples];
  se::key_t tempkeys[num_samples];
  Eigen::Vector3i coordinates[num_samples];

  for (int i = 0; i < num_samples; ++i) {
    const Eigen::Vector3i vox = {dis(gen), dis(gen), dis(gen)};
    coordinates[i] = Eigen::Vector3i(vox);
    const se::key_t code = compute_morton(vox(0), vox(1), vox(2));
    keys[i] = code;
  }

  const int max_level = log2(size);
  const int leaf_level = max_level - log2(block_side);
  const unsigned int shift = max_bits - max_level;
  int edge = size / 2;
  for (int level = 0; level <= leaf_level; level++) {
    const se::key_t mask = MASK[level + shift];
    compute_prefix(keys, tempkeys, num_samples, mask);
    for (int i = 0; i < num_samples; ++i) {
      const Eigen::Vector3i masked_vox = unpack_morton(tempkeys[i]);
      ASSERT_EQ(masked_vox(0) % edge, 0);
      ASSERT_EQ(masked_vox(1) % edge, 0);
      ASSERT_EQ(masked_vox(2) % edge, 0);
      const Eigen::Vector3i vox = coordinates[i];
      // printf("vox: %d, %d, %d\n", vox(0), vox(1), vox(2));
      // printf("masked level %d: %d, %d, %d\n", level, masked_vox(0), masked_vox(1), masked_vox(2) );
    }
    edge = edge / 2;
  }
}
