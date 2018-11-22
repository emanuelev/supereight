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
#include <cmath>
#include "octree.hpp"
#include "utils/math_utils.h"
#include "gtest/gtest.h"
#include "functors/axis_aligned_functor.hpp"

typedef float testT;
template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return {0.f}; }
  static inline value_type initValue(){ return {1.f}; }
};

float test_fun(float x, float y, float z) {
  return se::math::sq(z) + std::sin(2*x + y);
}

class InterpolationTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      unsigned size = 512;
      float dim = 5.f;
      oct_.init(size, dim); // 5 meters

      const float center = 2.5f;
      const float radius = center + 0.5f; 

      const float voxelsize = oct_.dim()/oct_.size();
      const float inverse_voxelsize = 1.f/voxelsize;
      const int band = 1 * inverse_voxelsize;
      const Eigen::Vector3i offset = 
        Eigen::Vector3i::Constant(oct_.size()/2 - band/2);
      unsigned leaf_level = log2(size) - log2(se::Octree<testT>::blockSide);
      for(int z = 0; z < band; ++z) {
        for(int y = 0; y < band; ++y) {
          for(int x = 0; x < band; ++x) {
            const Eigen::Vector3i vox =  Eigen::Vector3i(x + offset(0), 
                y + offset(1), z + offset(2));
            alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2), leaf_level));
          }
        }
      }
      oct_.allocate(alloc_list.data(), alloc_list.size());
    }

  typedef se::Octree<testT> OctreeF;
  OctreeF oct_;
  std::vector<se::key_t> alloc_list;
};

TEST_F(InterpolationTest, Init) {

  auto initialise = [](auto& handler, const Eigen::Vector3i& v) {
    float data;
    data = test_fun(v(0), v(1), v(2));
    handler.set(data);
  }; 

  se::functor::axis_aligned_map(oct_, initialise);

  auto test = [](auto& handler, const Eigen::Vector3i& v) {
    auto data = handler.get();
    ASSERT_EQ(data, test_fun(v(0), v(1), v(2)));
  }; 
  se::functor::axis_aligned_map(oct_, test);

  // std::stringstream f;
  // f << "./analytical_function.vtk";
  // save3DSlice(oct_, Eigen::Vector3i(0, oct_.size()/2, 0),
  //     Eigen::Vector3i(oct_.size(), oct_.size()/2 + 1, oct_.size()), 
  //     Eigen::Vector3i(oct_.size()), f.str().c_str());
  // f.str("");
  // f.clear();
}

// TEST_F(InterpolationTest, InterpAtPoints) {
// 
//   auto test = [this](auto& handler, const Eigen::Vector3i& v) {
//     auto data = handler.get();
//     float interpolated = oct_.interp(make_float3(v(0), v(1), v(2)), [](const auto& val){ return val(0); });
//     ASSERT_EQ(data(0), interpolated);
//   }; 
// 
//   se::functor::axis_aligned<testT, Octree, decltype(test)> 
//     funct_test(oct_, test);
//   funct_test.apply();
// }
