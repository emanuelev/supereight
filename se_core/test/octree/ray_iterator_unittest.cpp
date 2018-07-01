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
#include "ray_iterator.hpp"
#include "gtest/gtest.h"
#include <vector>

typedef float testT;

template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return 0.f; }
  static inline value_type initValue(){ return 1.f; }
};
typedef se::Octree<testT> OctreeF;

class RayIteratorTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      oct_.init(512, 5);

      p_   = Eigen::Vector3f(1.5f, 1.5f, 1.5f);
      dir_ = Eigen::Vector3f(0.5, 0.5, 0.5).normalized();

      // ensure stepsize is big enough to get distinct blocks
      const float stepsize = 2 * (oct_.dim()/oct_.size() * OctreeF::blockSide);
      const float voxelsize = oct_.dim()/oct_.size();

      const int num_blocks = 4;
      float t = 0.6f;
      for(int i = 0; i < num_blocks; ++i, t += stepsize) {
        const Eigen::Vector3f tmp = p_ + t * dir_;
        const Eigen::Vector3i vox = ((p_ + t * dir_)/voxelsize).cast<int> ();

        // hash to VoxelBlocks
        se::key_t key = oct_.hash(vox(0), vox(1), vox(2)) ;
        alloc_list_.push_back(key);
      }

      oct_.allocate(alloc_list_.data(), alloc_list_.size());
    }
  OctreeF oct_;
  Eigen::Vector3f p_;
  Eigen::Vector3f dir_;
  std::vector<se::key_t> alloc_list_;
};

TEST_F(RayIteratorTest, FetchAlongRay) {
  se::ray_iterator<testT> it(oct_, p_, dir_, 0.4, 4.0f); 
  int i = 0;
  se::VoxelBlock<testT> * current;
  while(current = it.next()) {
    ASSERT_LT(i, alloc_list_.size());
    ASSERT_EQ(current->code_, alloc_list_[i]);
    i++; 
  }
  ASSERT_EQ(i, alloc_list_.size());
}
