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
#include "se/octree.hpp"
#include "se/node_iterator.hpp"
#include "se/node.hpp"
#include "se/utils/math_utils.h"
#include "se/utils/morton_utils.hpp"
#include "se/functors/axis_aligned_functor.hpp"
#include "se/config.h"
#include "se/planner_config.h"
#include "gtest/gtest.h"

#include "se/path_planning/candidate_view.hpp"
#include "se/path_planning/init_new_position.hpp"

typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > vec3i;

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
typedef struct {
  float x;
} testT;

template<>
struct voxel_traits<testT> {
  typedef testT value_type;
  static inline value_type empty() { return {0.f}; }
  static inline value_type initValue() { return {0.f}; }
};

class CandidateViewTest : public ::testing::Test {
 protected:
  virtual void SetUp() {

    unsigned size = 128;
    float dim = 12.8f;
    oct_.init(size, dim); // 10 meters

    const unsigned center = 6.4f;

    float voxelsize = oct_.dim() / oct_.size(); // 0.1m
    const float inverse_voxelsize = 1.f / voxelsize;
    const int band = 1 * inverse_voxelsize; // 10
    const Eigen::Vector3i offset = Eigen::Vector3i::Constant(oct_.size() / 2 - band / 2);
    unsigned leaf_level = log2(size) - log2(se::Octree<testT>::blockSide);
    for (int z = 0; z < band; ++z) {
      for (int y = 0; y < band; ++y) {
        for (int x = 0; x < band; ++x) {
          const Eigen::Vector3i vox = Eigen::Vector3i(x + offset(0), y + offset(1), z + offset(2));
          alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2), leaf_level));
        }
      }
    }
    oct_.allocate(alloc_list.data(), alloc_list.size());

  }

  Volume<testT> volume_;
  typedef se::Octree<testT> OctreeF;
  OctreeF oct_;
  std::vector<se::key_t> alloc_list;
  Planning_Configuration planning_config;
  Configuration config;
  float voxelsize;

};

TEST_F(CandidateViewTest, WeightFunction) {
  //GIVEN: a cand view object
  Eigen::Matrix4f pose;
  pose << 1.f, 0.f, 0.f, 3.f, 0.f, 1.f, 0.f, 3.f, 0.f, 0.f, 1.f, 3.f, 0.f, 0.f, 0.f, 1.f;
  se::exploration::CandidateView<testT>
      cand_view(volume_, planning_config, voxelsize, config, pose);
  bool hit_unknown[3] = {false, false, false};
  float t_hit[3]={0.f, 0.f , 0.f};
  float prob_log[3] = {0.f, 3.f, 0.f};
  float ray_length[3] = {0.f, 1.f, 2.f};
  float ray_max = 4.f;
  float ratio = 1.f;
  float weight[3];
  //WHEN: check the tanh function , that when prob_log = 0 , unknown voxel is hit

  testing::internal::CaptureStdout();



  weight[0]= cand_view.getIGWeight_tanh(ray_max, ratio, ray_length[0], prob_log[0], t_hit[0],
      hit_unknown[0]);

  weight[1]= cand_view.getIGWeight_tanh(ray_max, ratio, ray_length[1], prob_log[1], t_hit[1],
                                        hit_unknown[1]);
 // when a ray hits a unknown voxel at 2m . the unknown voxel should have the weight of tanh(4)
  weight[2]= cand_view.getIGWeight_tanh(ray_max, ratio, ray_length[2], prob_log[2], t_hit[2],
                                        hit_unknown[2]);

  //THEN: the tanh function should be applied
  EXPECT_FLOAT_EQ(tanh(4), weight[0]);
  EXPECT_TRUE(hit_unknown[0]);
  EXPECT_FLOAT_EQ(0.f, t_hit[0]); //


  EXPECT_FLOAT_EQ(1.f, weight[1]);
  EXPECT_FALSE(hit_unknown[1]);
  EXPECT_FLOAT_EQ(0.f, t_hit[1]);

  EXPECT_FLOAT_EQ(tanh(4), weight[2]);
  EXPECT_TRUE(hit_unknown[2]);
  EXPECT_FLOAT_EQ(2.f, t_hit[2]);

  std::string output = testing::internal::GetCapturedStdout();
//  EXPECT_TRUE(false) << output;
}
TEST_F(CandidateViewTest, EntropyCalculation_occ) {
  //GIVEN:
// set map value
// TODO make this work
//  std::pair<double, float> ig = cand_view.getInformationGain(volume_,
//                                                             Eigen::Vector3f(1.f, 0.f, 0.f),
//                                                             0.1f,
//                                                             4.1f,
//                                                             0.1,
//                                                             Eigen::Vector3f(3.f, 3.f, 3.f));
//  std::cout << ig.first << ig.second << std::endl;
  auto set_to_five = [](auto &handler, const Eigen::Vector3i &) {
    handler.set({5.f});
  };
  se::functor::axis_aligned_map(oct_,
                                set_to_five,
                                Eigen::Vector3i::Constant(0),
                                Eigen::Vector3i::Constant(51));

  //WHEN

  //THEN
}
// TODO fix this test!
TEST_F(CandidateViewTest, getSphereAroundPoint ) {
  //GIVEN:
  // Allocated block: {56, 8, 248};
  mapvec3i block_voxel_map;
  const Eigen::Vector3i center = {60, 12, 252};
  const float radius = 0.2f;// [m] 2 voxels
  //WHEN
  /* Update bottom left corner as occupied */
  se::VoxelBlock<testT> *block = oct_.fetch(56, 8, 248);
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  testT low;
  low.x = 2.f;
  testT high;
  high.x = 10.f;
  blockSide = (int) se::VoxelBlock<testT>::side;
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {
        if ((blockCoord(0)) < x && x < (xlast - 4) && (blockCoord(1)) < y && y < (ylast - 4)
            && (blockCoord(2)) < z && z < (zlast - 4)) {
          block->data(Eigen::Vector3i(x, y, z), low);
        } else {
          block->data(Eigen::Vector3i(x, y, z), high);
        }
      }
    }
  }
  se::exploration::getSphereAroundPoint(center, radius, oct_, block_voxel_map);

  //THEN
}

