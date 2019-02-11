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
#include <fstream>
#include <string>
#include <random>
#include "io/se_serialise.hpp"
#include "node.hpp"
#include "octree.hpp"
#include "gtest/gtest.h"

typedef float testT;
template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return 0.f; }
  static inline value_type initValue(){ return 10.f; }
};

class Occupancy;
template <>
struct voxel_traits<Occupancy> {
  typedef struct { 
    float x;
    double y;
  } value_type;
  static inline value_type empty(){ return {0.f, 0.}; }
  static inline value_type initValue(){ return {1.f, 0.}; }
};

TEST(SerialiseUnitTest, WriteReadNode) {
  std::string filename = "test.bin";
  {
    std::ofstream os (filename, std::ios::binary); 
    se::Node<testT> octant;
    octant.code_ = 24;
    octant.side_ = 256;
    for(int i = 0; i < 8; ++i)
      octant.value_[i] =  5.f;
    se::internal::serialise(os, octant);
  }

  {
    std::ifstream is(filename, std::ios::binary);
    se::Node<testT> octant;
    se::internal::deserialise(octant, is);
    ASSERT_EQ(octant.code_, 24);
    ASSERT_EQ(octant.side_, 256);
    for(int i = 0; i < 8; ++i)
      ASSERT_EQ(octant.value_[i], 5.f);
  }
}

TEST(SerialiseUnitTest, WriteReadBlock) {
  std::string filename = "test.bin";
  {
    std::ofstream os (filename, std::ios::binary); 
    se::VoxelBlock<testT> octant;
    octant.code_ = 24;
    octant.coordinates(Eigen::Vector3i(40, 48, 52));
    for(int i = 0; i < 512; ++i)
      octant.data(i, 5.f);
    se::internal::serialise(os, octant);
  }

  {
    std::ifstream is(filename, std::ios::binary);
    se::VoxelBlock<testT> octant;
    se::internal::deserialise(octant, is);
    ASSERT_EQ(octant.code_, 24);
    ASSERT_TRUE(octant.coordinates() == Eigen::Vector3i(40, 48, 52));
    for(int i = 0; i < 512; ++i)
      ASSERT_EQ(octant.data(i), 5.f);
  }
}

TEST(SerialiseUnitTest, WriteReadBlockStruct) {
  std::string filename = "test.bin";
  {
    std::ofstream os (filename, std::ios::binary); 
    se::VoxelBlock<Occupancy> octant;
    octant.code_ = 24;
    octant.coordinates(Eigen::Vector3i(40, 48, 52));
    for(int i = 0; i < 512; ++i)
      octant.data(i, {5.f, 2.});
    se::internal::serialise(os, octant);
  }

  {
    std::ifstream is(filename, std::ios::binary);
    se::VoxelBlock<Occupancy> octant;
    se::internal::deserialise(octant, is);
    ASSERT_EQ(octant.code_, 24);
    ASSERT_TRUE(octant.coordinates() == Eigen::Vector3i(40, 48, 52));
    for(int i = 0; i < 512; ++i) {
      auto data = octant.data(i);
      ASSERT_EQ(data.x, 5.f);
      ASSERT_EQ(data.y, 2.);
    }
  }
}

TEST(SerialiseUnitTest, SerialiseTree) {
  se::Octree<testT> tree;
  tree.init(1024, 10);
  const int side = se::VoxelBlock<testT>::side;
  const int max_depth = log2(tree.size());
  const int leaves_level = max_depth - log2(side);
  std::mt19937 gen(1); //Standard mersenne_twister_engine seeded with constant
  std::uniform_int_distribution<> dis(0, 1023);
  
  int num_tested = 0;
  for(int i = 1, edge = tree.size()/2; i <= leaves_level; ++i, edge = edge/2) { 
    for(int j = 0; j < 20; ++j) {
      Eigen::Vector3i vox(dis(gen), dis(gen), dis(gen));
      tree.insert(vox(0), vox(1), vox(2), i);
    }
  }
  std::string filename = "octree-test.bin";
  tree.save(filename);

  se::Octree<testT> tree_copy;
  tree_copy.load(filename);

  auto& node_buffer_base = tree.getNodesBuffer();
  auto& node_buffer_copy = tree_copy.getNodesBuffer();
  ASSERT_EQ(node_buffer_base.size(), node_buffer_copy.size());
  for(int i = 0; i < node_buffer_base.size(); ++i) {
    se::Node<testT> * n  = node_buffer_base[i];
    se::Node<testT> * n1 = node_buffer_copy[i];
    ASSERT_EQ(n->code_, n1->code_);
    ASSERT_EQ(n->children_mask_, n1->children_mask_);
  }

  auto& block_buffer_base = tree.getBlockBuffer();
  auto& block_buffer_copy = tree_copy.getBlockBuffer();
  ASSERT_EQ(block_buffer_base.size(), block_buffer_copy.size());
}

