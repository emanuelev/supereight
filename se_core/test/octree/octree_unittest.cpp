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
#include "utils/math_utils.h"
#include "gtest/gtest.h"
#include "octant_ops.hpp"
#include <octree.hpp>
#include <bitset>
#include <io/ply_io.hpp>
#include <algorithms/balancing.hpp>

typedef float testT;

template <>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty(){ return 0.f; }
  static inline value_type initValue(){ return 1.f; }
};

TEST(Octree, OctantFaceNeighbours) {
  const Eigen::Vector3i octant = {112, 80, 160};
  const unsigned int max_depth = 8;
  const unsigned int leaves_depth = 5;
  const se::key_t code = 
    se::keyops::encode(octant(0), octant(1), octant(2), leaves_depth, max_depth);
  const unsigned int side = 8;
  const Eigen::Vector3i faces[6] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, 
    {0, 0, -1}, {0, 0, 1}};
  for(int i = 0; i < 6; ++i) {
    const Eigen::Vector3i neighbour = octant + side * faces[i];
    const Eigen::Vector3i computed = se::face_neighbour(code, i, leaves_depth, max_depth); 
    ASSERT_EQ(neighbour(0), computed(0)); 
    ASSERT_EQ(neighbour(1), computed(1)); 
    ASSERT_EQ(neighbour(2), computed(2)); 
  }
}

TEST(Octree, OctantDescendant) {
  const unsigned max_depth = 8;
  Eigen::Vector3i octant = {110, 80, 159};
  se::key_t code = 
    se::keyops::encode(octant(0), octant(1), octant(2), 5, max_depth);
  se::key_t ancestor = 
    se::keyops::encode(96, 64, 128, 3, max_depth);
  ASSERT_EQ(true , se::descendant(code, ancestor, max_depth)); 

  ancestor = se::keyops::encode(128, 64, 64, 3, max_depth);
  ASSERT_FALSE(se::descendant(code, ancestor, max_depth)); 
}

TEST(Octree, OctantParent) {
  const int max_depth = 8;
  Eigen::Vector3i octant = {112, 80, 160};
  se::key_t code = 
    se::keyops::encode(octant(0), octant(1), octant(2), 5, max_depth);
  se::key_t p = se::parent(code, max_depth);
  ASSERT_EQ(se::keyops::code(code), se::keyops::code(p));
  ASSERT_EQ(4, p & SCALE_MASK);

  code = p;
  p = se::parent(code, max_depth); 
  ASSERT_EQ(3, se::keyops::level(p));
  ASSERT_EQ(p, se::keyops::encode(96, 64, 160, 3, max_depth));

  code = p;
  p = se::parent(code, max_depth); 
  ASSERT_EQ(2, se::keyops::level(p));
  ASSERT_EQ(p, se::keyops::encode(64, 64, 128, 2, max_depth));
}

TEST(Octree, FarCorner) {
  /*
   * The far corner should always be "exterior", meaning that moving one step
   * in the outward direction (i.e. away from the center) in *any* direction 
   * implies leaving the parent octant. For simplicity here the corners
   * individually, but this can be done programmatically testing this property.
   * TODO: change this test case to be more exhaustive.
   */

  const int max_depth = 5;
  const int level = 2;

  /* First child */
  const se::key_t cell0 = 
    se::keyops::encode(16, 16, 16, level, max_depth);
  const Eigen::Vector3i fc0 = se::far_corner(cell0, level, max_depth);
  ASSERT_EQ(fc0(0), 16);
  ASSERT_EQ(fc0(1), 16);
  ASSERT_EQ(fc0(2), 16);

  /* Second child */
  const se::key_t cell1 = se::keyops::encode(24, 16, 16, level, max_depth);
  const Eigen::Vector3i fc1 = se::far_corner(cell1, level, max_depth);
  ASSERT_EQ(fc1(0), 32);
  ASSERT_EQ(fc1(1), 16);
  ASSERT_EQ(fc1(2), 16);

  /* Third child */
  const se::key_t cell2 = se::keyops::encode(16, 24, 16, level, max_depth);
  const Eigen::Vector3i fc2 = se::far_corner(cell2, level, max_depth);
  ASSERT_EQ(fc2(0), 16);
  ASSERT_EQ(fc2(1), 32);
  ASSERT_EQ(fc2(2), 16);

  /* Fourth child */
  const se::key_t cell3 = se::keyops::encode(24, 24, 16, level, max_depth);
  const Eigen::Vector3i fc3 = se::far_corner(cell3, level, max_depth);
  ASSERT_EQ(fc3(0), 32);
  ASSERT_EQ(fc3(1), 32);
  ASSERT_EQ(fc3(2), 16);

  /* Fifth child */
  const se::key_t cell4 = se::keyops::encode(24, 24, 16, level, max_depth);
  const Eigen::Vector3i fc4 = se::far_corner(cell4, level, max_depth);
  ASSERT_EQ(fc4(0), 32);
  ASSERT_EQ(fc4(1), 32);
  ASSERT_EQ(fc4(2), 16);

  /* sixth child */
  const se::key_t cell5 = se::keyops::encode(16, 16, 24, level, max_depth);
  const Eigen::Vector3i fc5 = se::far_corner(cell5, level, max_depth);
  ASSERT_EQ(fc5(0), 16);
  ASSERT_EQ(fc5(1), 16);
  ASSERT_EQ(fc5(2), 32);

  /* seventh child */
  const se::key_t cell6 = se::keyops::encode(24, 16, 24, level, max_depth);
  const Eigen::Vector3i fc6 = se::far_corner(cell6, level, max_depth);
  ASSERT_EQ(fc6(0), 32);
  ASSERT_EQ(fc6(1), 16);
  ASSERT_EQ(fc6(2), 32);

  /* eight child */
  const se::key_t cell7 = se::keyops::encode(24, 24, 24, level, max_depth);
  const Eigen::Vector3i fc7 = se::far_corner(cell7, level, max_depth);
  ASSERT_EQ(fc7(0), 32);
  ASSERT_EQ(fc7(1), 32);
  ASSERT_EQ(fc7(2), 32);
}

TEST(Octree, InnerOctantExteriorNeighbours) {
  const int max_depth = 5;
  const int level = 2;
  const int side = 1 << (max_depth - level);
  const se::key_t cell = se::keyops::encode(16, 16, 16, level, max_depth);
  se::key_t N[7];
  se::exterior_neighbours(N, cell, level, max_depth);
  const se::key_t p = se::parent(cell, max_depth);
  
  const se::key_t neighbours_gt[7] = 
    {se::keyops::encode(15, 16, 16, level, max_depth),
     se::keyops::encode(16, 15, 16, level, max_depth),
     se::keyops::encode(15, 15, 16, level, max_depth),
     se::keyops::encode(16, 16, 15, level, max_depth),
     se::keyops::encode(15, 16, 15, level, max_depth),
     se::keyops::encode(16, 15, 15, level, max_depth),
     se::keyops::encode(15, 15, 15, level, max_depth)};
  for(int i = 0; i < 7; ++i) {
    // std::bitset<64> c(N[i]);
    // std::bitset<64> a(p);
    // std::cout << a << std::endl;
    // std::cout << c << std::endl << std::endl;
    ASSERT_EQ(neighbours_gt[i], N[i]);
    ASSERT_FALSE(se::parent(N[i], max_depth) == p);
    // std::cout << (unpack_morton(N[i] & ~SCALE_MASK)) << std::endl;
  }
}

TEST(Octree, EdgeOctantExteriorNeighbours) {
  const int max_depth = 5;
  const int size = 1 << max_depth;
  const int level = 2;
  const se::key_t cell = se::keyops::encode(0, 16, 16, level, max_depth);
  se::key_t N[7];
  se::exterior_neighbours(N, cell, level, max_depth);
  const se::key_t p = se::parent(cell, max_depth);
  
  for(int i = 0; i < 7; ++i) {
    const Eigen::Vector3i corner = unpack_morton(N[i] & ~SCALE_MASK);
    const int res = ((corner.array() >= Eigen::Vector3i::Constant(0).array()) 
     && (corner.array() <= Eigen::Vector3i::Constant(size - 1).array())).all();
    ASSERT_TRUE(res);
  }
}

TEST(Octree, OctantSiblings) {
  const int max_depth = 5;
  const unsigned size = std::pow(2, 5);
  const int level = 2;
  const se::key_t cell = se::keyops::encode(16, 16, 16, level, max_depth);
  se::key_t s[8];
  se::siblings(s, cell, max_depth);

  const int childidx = se::child_id(cell, level, max_depth);
  ASSERT_EQ(s[childidx], cell);
  const Eigen::Vector3i p = se::keyops::decode(cell); 
  for(int i = 0; i < 8; ++i) {
    // std::cout << (unpack_morton(s[i] & ~SCALE_MASK)) << std::endl;
    const Eigen::Vector3i po = se::keyops::decode(se::parent(s[i], max_depth));
    ASSERT_TRUE(se::parent(s[i], max_depth) == se::parent(cell, max_depth));
    ASSERT_TRUE(p.x() == po.x() && p.y() == po.y() && p.z() == po.z());
  }
}

TEST(Octree, OctantOneNeighbours) {
  const int max_depth = 8;
  const int level = 5;
  const unsigned size = std::pow(2, max_depth);
  Eigen::Matrix<int, 4, 6> N;
  Eigen::Vector3i pos;
  
  // Inside cube
  //
  pos << 127, 56, 3;
  se::one_neighbourhood(N, se::keyops::encode(pos.x(), pos.y(), pos.z(), 
        level, max_depth), max_depth);
  ASSERT_TRUE((N.array() >= 0).all() && (N.array() < size).all());

  
  // At edge cube
  //
  pos << size-1, 56, 3;
  se::one_neighbourhood(N, se::keyops::encode(pos.x(), pos.y(), pos.z(), 
        level, max_depth), max_depth);
  ASSERT_TRUE((N.array() >= 0).all() && (N.array() < size).all());
}

TEST(Octree, BalanceTree) {
  const int max_depth = 8;
  const int level = 5;

  se::Octree<testT> oct;
  oct.init(1 << max_depth, 5);
  Eigen::Vector3i vox(32, 208, 44);
  oct.insert(vox.x(), vox.y(), vox.z());
  vox += Eigen::Vector3i(50, 12, 100);
  oct.insert(vox.x(), vox.y(), vox.z());
  se::print_octree("./oct.ply", oct);
  se::balance(oct);
  se::print_octree("./oct-balanced.ply", oct);
}
