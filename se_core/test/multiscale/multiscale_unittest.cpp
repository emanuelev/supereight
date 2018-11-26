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
#include "octant_ops.hpp"
#include "node_iterator.hpp"
#include "utils/math_utils.h"
#include "gtest/gtest.h"
#include <random>

typedef float testT;

template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return 0.f; }
  static inline value_type initValue(){ return 1.f; }
};

class MultiscaleTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      oct_.init(512, 5);

    }

  typedef se::Octree<testT> OctreeF;
  OctreeF oct_;
};

TEST_F(MultiscaleTest, Init) {
  EXPECT_EQ(oct_.get(137, 138, 130), voxel_traits<testT>::initValue());
}

TEST_F(MultiscaleTest, PlainAlloc) {
  const Eigen::Vector3i blocks[2] = {{56, 12, 254}, {87, 32, 423}};
  se::key_t alloc_list[2];
  for(int i = 0; i < 2; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2));
  }
  oct_.allocate(alloc_list, 2);

  oct_.set(56, 12, 254, 3.f);

  EXPECT_EQ(oct_.get(56, 12, 254), 3.f);
  EXPECT_EQ(oct_.get(106, 12, 254), voxel_traits<testT>::initValue());
  EXPECT_NE(oct_.get(106, 12, 254), 3.f);
}

TEST_F(MultiscaleTest, ScaledAlloc) {
  const Eigen::Vector3i blocks[2] = {{200, 12, 25}, {87, 32, 423}};
  se::key_t alloc_list[2];
  for(int i = 0; i < 2; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2), 5);
  }

  oct_.allocate(alloc_list, 2);
  se::Node<testT>* n = oct_.fetch_octant(87, 32, 420, 5);
  ASSERT_TRUE(n != NULL);
  n->value_[0] = 10.f;
  EXPECT_EQ(oct_.get(87, 32, 420), 10.f);
}

TEST_F(MultiscaleTest, Iterator) {
  const Eigen::Vector3i blocks[1] = {{56, 12, 254}};
  se::key_t alloc_list[1];
  alloc_list[0] = oct_.hash(blocks[0](0), blocks[0](1), blocks[0](2));

  oct_.allocate(alloc_list, 1);
  se::node_iterator<testT> it(oct_);
  se::Node<testT> * node = it.next();
  for(int i = 512; node != nullptr; node = it.next(), i /= 2){
    const Eigen::Vector3i coords = se::keyops::decode(node->code_);
    const int side = node->side_;
    const se::Octree<testT>::value_type val = node->value_[0];
    EXPECT_EQ(side, i);
  }
}

TEST_F(MultiscaleTest, ChildrenMaskTest) {
  const Eigen::Vector3i blocks[10] = {{56, 12, 254}, {87, 32, 423}, {128, 128, 128},
    {136, 128, 128}, {128, 136, 128}, {136, 136, 128}, 
    {128, 128, 136}, {136, 128, 136}, {128, 136, 136}, {136, 136, 136}};
  se::key_t alloc_list[10];
  for(int i = 0; i < 10; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2), 5);
  }

  oct_.allocate(alloc_list, 10);
  const se::MemoryPool<se::Node<testT> >& nodes = oct_.getNodesBuffer();
  const size_t num_nodes = nodes.size();
  for(size_t i = 0; i < num_nodes; ++i) {
    se::Node<testT>* n = nodes[i];
    for(int c = 0; c < 8; ++c) {
      if(n->child(c)) {
        ASSERT_TRUE(n->children_mask_ & (1 << c));
      }
    }
  } 
}

TEST_F(MultiscaleTest, OctantAlloc) {
  const Eigen::Vector3i blocks[10] = {{56, 12, 254}, {87, 32, 423}, {128, 128, 128},
    {136, 128, 128}, {128, 136, 128}, {136, 136, 128}, 
    {128, 128, 136}, {136, 128, 136}, {128, 136, 136}, {136, 136, 136}};
  se::key_t alloc_list[10];
  for(int i = 0; i < 10; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2));
  }

  alloc_list[2] = alloc_list[2] | 3;
  alloc_list[9] = alloc_list[2] | 5;
  oct_.allocate(alloc_list, 10);
  se::Node<testT> * octant = oct_.fetch_octant(blocks[4](0), blocks[4](1),
      blocks[4](2), 3);
  ASSERT_TRUE(octant != NULL);
  octant = oct_.fetch_octant(blocks[9](0), blocks[9](1),
      blocks[9](2), 6);
  ASSERT_TRUE(octant == NULL);
}

TEST_F(MultiscaleTest, SingleInsert) {
  Eigen::Vector3i vox(32, 208, 44);
  const int side = se::VoxelBlock<testT>::side;
  se::VoxelBlock<testT> * n = oct_.insert(vox(0), vox(1), vox(2));
  Eigen::Vector3i coords = n->coordinates();
  Eigen::Vector3i rounded = side * (vox/side);
  ASSERT_TRUE(coords == rounded);
}

TEST_F(MultiscaleTest, MultipleInsert) {
  OctreeF tree;
  tree.init(1024, 10);
  const int side = se::VoxelBlock<testT>::side;
  const int max_depth = log2(tree.size());
  const int leaves_level = max_depth - log2(side);
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(1); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> dis(0, 1023);
  
  int num_tested = 0;
  for(int i = 1, edge = tree.size()/2; i <= leaves_level; ++i, edge = edge/2) { 
    for(int j = 0; j < 20; ++j) {
      Eigen::Vector3i vox(dis(gen), dis(gen), dis(gen));
      se::Node<testT> * n = tree.insert(vox(0), vox(1), vox(2), i);
      se::Node<testT> * n1 = tree.fetch_octant(vox(0), vox(1), vox(2), i);
      Eigen::Vector3i rounded = edge * (vox/edge);
      Eigen::Vector3i coords = se::keyops::decode(n1->code_);

      // Check expected coordinates
      ASSERT_TRUE(coords == rounded);
      // Should not have any children up to this level
      ASSERT_TRUE(n1->children_mask_ == 0);
      ++num_tested;
    }
  }
  std::cout << "tested " << num_tested << " nodes" << std::endl;
}
