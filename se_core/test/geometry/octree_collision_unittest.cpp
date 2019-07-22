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
#include "geometry/octree_collision.hpp"
#include "geometry/aabb_collision.hpp"
#include "utils/morton_utils.hpp"
#include "octree.hpp"
#include "octant_ops.hpp"
#include "node_iterator.hpp"
#include "functors/axis_aligned_functor.hpp"
#include "gtest/gtest.h"
#include "utils/eigen_utils.h"

using namespace se::geometry;
typedef float testT;

template<>
struct voxel_traits<testT> {
  typedef float value_type;
  static inline value_type empty() { return 0.f; }
  static inline value_type initValue() { return 1.f; }
};

collision_status test_voxel(const voxel_traits<testT>::value_type &val) {
  if (val == voxel_traits<testT>::initValue()) return collision_status::unseen;
  if (val == 10.f) return collision_status::empty;
  return collision_status::occupied;
};

class OctreeCollisionTest : public ::testing::Test {
 protected:
  virtual void SetUp() {

    oct_.init(256, 5);
    const Eigen::Vector3i blocks[1] = {{56, 12, 254}};
    se::key_t alloc_list[1];
    alloc_list[0] = oct_.hash(blocks[0](0), blocks[0](1), blocks[0](2));
    oct_.allocate(alloc_list, 1);

    auto set_to_ten = [](auto &handler, const Eigen::Vector3i &coords) {
      if ((coords.array() >= Eigen::Vector3i(48, 0, 240).array()).all()) {
        handler.set(10.f);
      }
    };
    se::functor::axis_aligned_map(oct_, set_to_ten);
  }

  typedef se::Octree<testT> OctreeF;
  OctreeF oct_;
};

TEST_F(OctreeCollisionTest, TotallyUnseen) {

  se::node_iterator<testT> it(oct_);
  se::Node<testT> *node = it.next();
  for (int i = 256; node != nullptr; node = it.next(), i /= 2) {
    const Eigen::Vector3i coords = se::keyops::decode(node->code_);
    const int side = node->side_;
    const se::Octree<testT>::value_type val = (node->value_[0]);
    printf("se::Node's coordinates: (%d, %d, %d), side %d, value %.2f\n",
           coords(0),
           coords(1),
           coords(2),
           side,
           val);
    EXPECT_EQ(side, i);
  }

  const Eigen::Vector3i test_bbox = {23, 0, 100};
  const Eigen::Vector3i width = {2, 2, 2};

  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  ASSERT_EQ(collides, collision_status::unseen);
}

TEST_F(OctreeCollisionTest, PartiallyUnseen) {
  const Eigen::Vector3i test_bbox = {47, 0, 239};
  const Eigen::Vector3i width = {6, 6, 6};
  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  ASSERT_EQ(collides, collision_status::occupied);
}

TEST_F(OctreeCollisionTest, Empty) {
  const Eigen::Vector3i test_bbox = {49, 1, 242};
  const Eigen::Vector3i width = {1, 1, 1};
  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  ASSERT_EQ(collides, collision_status::empty);
}

TEST_F(OctreeCollisionTest, Collision) {
  const Eigen::Vector3i test_bbox = {54, 10, 249};
  const Eigen::Vector3i width = {5, 5, 3};

  auto update = [](auto &handler, const Eigen::Vector3i &coords) {
    handler.set(2.f);
  };
  se::functor::axis_aligned_map(oct_, update);

  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  ASSERT_EQ(collides, collision_status::occupied);
}

TEST_F(OctreeCollisionTest, CollisionFreeLeaf) {
  // GIVEN this octree
  // Allocated block: {56, 8, 248};
  const Eigen::Vector3i test_bbox = {61, 13, 253};
  const Eigen::Vector3i width = {3, 3, 3};

  //sphere
  const Eigen::Vector3i center = {61, 13, 253};
  const int radius = 1;

  /* Update bottom left corner as occupied */
  se::VoxelBlock<testT> *block = oct_.fetch(56, 12, 254);
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  blockSide = (int) se::VoxelBlock<testT>::side;
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {
        if ((blockCoord(0)) < x && x < (xlast - 4) && (blockCoord(1)) < y && y < (ylast - 4)
            && (blockCoord(2)) < z && z < (zlast - 4)) {
          block->data(Eigen::Vector3i(x, y, z), 2.f);
        } else {
          block->data(Eigen::Vector3i(x, y, z), 10.f);
        }
      }
    }
  }
 // WHEN checking for collision in the upper right corner
  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  const collision_status collidesSphere = isSphereCollisionFree(oct_, center, radius);

// THEN the space should be collision free
  ASSERT_EQ(collides, collision_status::empty);
  ASSERT_EQ(collidesSphere, collision_status::empty);

}

TEST_F(OctreeCollisionTest, CollisionLeaf) {
  // GIVEN this octree
  // Allocated block: {56, 8, 248};
  const Eigen::Vector3i test_bbox = {57, 9, 249};
  const Eigen::Vector3i width = {5, 5, 5};

  // sphere
  const Eigen::Vector3i center = {60, 11, 251};
  const int radius = 2;
  /* Update the center of the voxel block as occupied */
  se::VoxelBlock<testT> *block = oct_.fetch(56, 12, 254);
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  blockSide = (int) se::VoxelBlock<testT>::side;
  testing::internal::CaptureStdout();
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {
        if ((blockCoord(0) + 1) < x && x < (xlast - 2) && (blockCoord(1) + 1) < y && y < (ylast - 2)
            && (blockCoord(2) + 1) < z && z < (zlast - 2)) {
          block->data(Eigen::Vector3i(x, y, z), 2.f);
        } else {
          block->data(Eigen::Vector3i(x, y, z), 10.f);
        }
      }
    }
  }
 // WHEN checking for collision in the voxel block
  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  const collision_status collidesSphere = isSphereCollisionFree(oct_, center, radius);
  std::string output = testing::internal::GetCapturedStdout();
//  EXPECT_TRUE(false) << output;
// THEN: the collision status should be occupied, although some voxels are free
  ASSERT_EQ(collides, collision_status::occupied);
  ASSERT_EQ(collidesSphere, collision_status::occupied);
}
TEST_F(OctreeCollisionTest, NoSphereYesBox) {
  // GIVEN this octree
  // Allocated block: {56, 8, 248};
  // bottom left corner of the box
  const Eigen::Vector3i test_bbox = {57, 9, 249};
  const Eigen::Vector3i width = {5, 5, 5};

  // sphere
  const Eigen::Vector3i center = test_bbox + Eigen::Vector3i(2, 2, 2);
  const int radius = 2;


  /* Update a small bottom left corner as occupied */
  se::VoxelBlock<testT> *block = oct_.fetch(56, 12, 254);
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  blockSide = (int) se::VoxelBlock<testT>::side;
  testing::internal::CaptureStdout();
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {
        if (x < (xlast - 6) && y < (ylast - 6) && z < (zlast - 6)) {
          block->data(Eigen::Vector3i(x, y, z), 2.f);
          std::cout << "fill " << Eigen::Vector3i(x, y, z).format(InLine) << " status 2.f"
                    << std::endl;
        } else {
          block->data(Eigen::Vector3i(x, y, z), 10.f);
        }
      }
    }
  }
// THEN checking for collision
  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  const collision_status collidesSphere = isSphereCollisionFree(oct_, center, radius);
  std::string output = testing::internal::GetCapturedStdout();
//  EXPECT_TRUE(false) << output;

// THEN : the corner of the bounding box is occupied, whereas the sphere doesn't collide with it
  ASSERT_EQ(collides, collision_status::occupied);

  ASSERT_EQ(collidesSphere, collision_status::empty);
}

TEST_F(OctreeCollisionTest, AcrossBlocksOccupied1) {
  // Allocated block: {56, 8, 248};
  // bottom left corner of the box
  const Eigen::Vector3i test_bbox = {55, 7, 247};
  const Eigen::Vector3i width = {5, 5, 5};

  // sphere
  const Eigen::Vector3i center = {56, 8, 248};
  const int radius = 2;


  /* Update a small bottom left corner as occupied */
  se::VoxelBlock<testT> *block = oct_.fetch(56, 12, 254);
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  blockSide = (int) se::VoxelBlock<testT>::side;
  testing::internal::CaptureStdout();
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {
        if (x < (xlast - 6) && y < (ylast - 6) && z < (zlast - 6)) {
          block->data(Eigen::Vector3i(x, y, z), 2.f);
        } else {
          block->data(Eigen::Vector3i(x, y, z), 10.f);
        }
      }
    }
  }

  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  const collision_status collidesSphere = isSphereCollisionFree(oct_, center, radius);
  std::string output = testing::internal::GetCapturedStdout();
//  EXPECT_TRUE(false) << output;
// THEN : box and sphere collide partially with occupied space, hence return occupied
  ASSERT_EQ(collides, collision_status::occupied);

  ASSERT_EQ(collidesSphere, collision_status::occupied);
}

TEST_F(OctreeCollisionTest, AcrossBlocksOccupied2) {
  // Allocated block: {56, 8, 248};
  // bottom left corner of the box
  const Eigen::Vector3i test_bbox = {62, 14, 254};
  const Eigen::Vector3i width = {5, 5, 5};

  // sphere
  const Eigen::Vector3i center = test_bbox + Eigen::Vector3i(2, 2, 2);
  const int radius = 2;


  /* Update a small bottom left corner as occupied */
  se::VoxelBlock<testT> *block = oct_.fetch(56, 12, 254);
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  blockSide = (int) se::VoxelBlock<testT>::side;
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {
        if (x < (xlast - 6) && y < (ylast - 6) && z < (zlast - 6)) {
          block->data(Eigen::Vector3i(x, y, z), 2.f);
        } else {
          block->data(Eigen::Vector3i(x, y, z), 10.f);
        }
      }
    }
  }
 // WHEN checking the sphere and blocks
  const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
  const collision_status collidesSphere = isSphereCollisionFree(oct_, center, radius);

// THEN : because neighbour nodes are allocated but not updated, unseen is treated as occupied
// for path planning
  ASSERT_EQ(collides, collision_status::occupied);

  ASSERT_EQ(collidesSphere, collision_status::occupied);
}
TEST_F(OctreeCollisionTest, AcrossBlocksFree) {
  // GIVEN: a map which is set to all free , except a small cube
  // Allocated block: {56, 8, 248};
  // bottom left corner of the box
  const Eigen::Vector3i test_bbox = {62, 14, 254};
  const Eigen::Vector3i width = {5, 5, 5};

  // sphere
  const Eigen::Vector3i center = test_bbox + Eigen::Vector3i(2, 2, 2);
  const int radius = 2;
 // set map to free
  auto set_to_ten = [](auto &handler, const Eigen::Vector3i &coords) {
    if ((coords.array() >= Eigen::Vector3i(0,0,0).array()).all()) {
      handler.set(10.f);
    }
  };
  se::functor::axis_aligned_map(oct_, set_to_ten);
  /* Update a small bottom left corner as occupied */
  se::VoxelBlock<testT> *block = oct_.fetch(56, 12, 254);
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  blockSide = (int) se::VoxelBlock<testT>::side;
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {
        if (x < (xlast - 6) && y < (ylast - 6) && z < (zlast - 6)) {
          block->data(Eigen::Vector3i(x, y, z), 2.f);
        } else {
          block->data(Eigen::Vector3i(x, y, z), 10.f);
        }
      }
    }
  }
// WHEN: checking box and sphere which map across different free voxel blocks
const collision_status collides = collides_with(oct_, test_bbox, width, test_voxel);
const collision_status collidesSphere = isSphereCollisionFree(oct_, center, radius);

//output;
// THEN : the collision status should be empty
EXPECT_EQ(collides, collision_status::empty);

EXPECT_EQ(collidesSphere, collision_status::empty);
}
