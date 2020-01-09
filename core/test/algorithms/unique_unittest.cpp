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
#include "utils/morton_utils.hpp"
#include "algorithms/unique.hpp"
#include "gtest/gtest.h"
#include <algorithm>
#include <bitset>

typedef float testT;
typedef unsigned int MortonType;

template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return 0.f; }
  static inline value_type initValue(){ return 0.f; }
};

class UniqueTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      oct.init(1 << max_depth_, 10);
      const Eigen::Vector3i blocks[10] = {
        {56, 12, 12}, {56, 12, 15}, 
        {128, 128, 128},
        {128, 128, 125}, {128, 128, 127}, 
        {128, 136, 129}, 
        {128, 136, 127}, 
        {136, 128, 136}, 
        {128, 240, 136}, {128, 241, 136}};
      for(int i = 0; i < 10; ++i) { 
        keys[i] = oct.hash(blocks[i](0), blocks[i](1), blocks[i](2));
      }
    }
    
    MortonType keys[10];
    typedef se::Octree<testT> OctreeF;
    OctreeF oct;
    const int max_depth_ = 10;
};

class UniqueMultiscaleTest : public ::testing::Test {
  protected:
    virtual void SetUp() {

      oct.init(1 << max_depth_, 10);
      const int root_side = pow(2, max_depth_ - 4);
      const Eigen::Vector3i base(64, 0, 64);
      keys.push_back(oct.hash(base.x(), base.y(), base.z(), 4));
      keys.push_back(oct.hash(base.x() + root_side/2, base.y(), base.z(), 5));
      keys.push_back(oct.hash(base.x() + root_side/4, base.y(), base.z(), 5));
      keys.push_back(oct.hash(128, 24, 80, 5));
      std::sort(keys.begin(), keys.end()); 
    }
   
    std::vector<MortonType> keys;
    typedef se::Octree<testT> OctreeF;
    OctreeF oct;
    const int max_depth_ = 10;
};

TEST_F(UniqueTest, FilterDuplicates) {
  const int last = se::algorithms::unique(keys, 10);
  for(int i = 1; i < last; ++i) { 
    ASSERT_TRUE(keys[i] != keys[i-1]);
  }
}

TEST_F(UniqueMultiscaleTest, FilterAncestors) {
  const int last = se::algorithms::filter_ancestors(keys.data(), keys.size(), max_depth_);
  for(int i = 1; i < last; ++i) { 
    // std::cout << std::bitset<64>(keys[i]) << std::endl;
    // std::cout << std::bitset<64>(keys[i - 1]) << std::endl << std::endl;
    ASSERT_TRUE(keys[i] != keys[i-1]);
  }
  ASSERT_EQ(last, 3);
}

TEST_F(UniqueMultiscaleTest, FilterDuplicatesTillLevel) {
  /*
   * 0x1FFu extracts the last 9 bits of a morton number,
   * corresponding to the edge of a voxel block: 3*log2(se::VoxelBlock<T>::side)
   */
  const int last = se::algorithms::unique_multiscale(keys.data(), keys.size());
  for(int i = 1; i < last; ++i) { 
    // std::cout << std::bitset<64>(keys[i]) << std::endl;
    ASSERT_TRUE(keys[i] != keys[i-1]);
  }
  ASSERT_EQ(last, 3);
}
