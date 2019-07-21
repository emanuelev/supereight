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
#ifndef OCTREE_COLLISION_HPP
#define OCTREE_COLLISION_HPP
#include "../node.hpp"
#include "../octree.hpp"
#include "../utils/eigen_utils.h"
#include "aabb_collision.hpp"

namespace se {
namespace geometry {
enum class collision_status {
  occupied, unseen, empty
};
static std::ostream &operator<<(std::ostream &os, const collision_status &dt) {
  return os << static_cast<int>(dt);
}
/*! \brief Implements a simple state machine to update the collision status.
 * The importance order is given as follows in ascending order: 
 * Empty, Unseen, Occupied.
 * \param previous_status
 * \param new_status 
 */
inline collision_status update_status(const collision_status previous_status,
                                      const collision_status new_status) {
  switch (previous_status) {
    case collision_status::unseen:
        return collision_status::occupied;
      break;
    case collision_status::occupied:return previous_status;
      break;
    default:return new_status;
      break;
  }
}

/*! \brief Perform a collision test for each voxel value in the input voxel 
 * block. The test function test takes as input a voxel value and returns a
 * collision_status. This is used to distinguish between seen-empty voxels and
 * occupied voxels.
 * \param block voxel block of type FieldType
 * \param test function that takes a voxel and returns a collision_status value
 */
template<typename FieldType, typename TestVoxelF>
collision_status collides_with(const se::VoxelBlock<FieldType> *block,
                               const Eigen::Vector3i bbox,
                               const Eigen::Vector3i side,
                               TestVoxelF test) {
  collision_status status = collision_status::empty;
  const Eigen::Vector3i blockCoord = block->coordinates();
  int x, y, z, blockSide;
  blockSide = (int) se::VoxelBlock<FieldType>::side;
  int xlast = blockCoord(0) + blockSide;
  int ylast = blockCoord(1) + blockSide;
  int zlast = blockCoord(2) + blockSide;
  Eigen::Vector3i center = bbox + side / 2;

  for (z = blockCoord(2); z < zlast; ++z) {
    for (y = blockCoord(1); y < ylast; ++y) {
      for (x = blockCoord(0); x < xlast; ++x) {

        typename se::VoxelBlock<FieldType>::value_type value;
        const Eigen::Vector3i vox{x, y, z};
        if (!geometry::aabb_aabb_collision(bbox, side, vox, Eigen::Vector3i::Constant(1))) continue;
        value = block->data(Eigen::Vector3i(x, y, z));
        status = update_status(status, test(value));
//        std::cout << "[se/collision] vb " << vox.format(InLine) << " status " << status
//                  << std::endl;
        if(status == collision_status::occupied){return status;}
      }
    }
  }
  return status;
}

/*! \brief Perform a collision test between the input octree map and the 
 * input axis aligned bounding box bbox of extension side. The test function 
 * test takes as input a voxel value and returns a collision_status. This is
 * used to distinguish between seen-empty voxels and occupied voxels.
 * \param map octree map
 * \param bbox test bounding box lower bottom corner 
 * \param side extension in number of voxels of the bounding box
 * \param test function that takes a voxel and returns a collision_status value
 */

template<typename FieldType, typename TestVoxelF>
collision_status collides_with(const Octree<FieldType> &map,
                               const Eigen::Vector3i bbox,
                               const Eigen::Vector3i side,
                               TestVoxelF test) {

  typedef struct stack_entry {
    se::Node<FieldType> *node_ptr;
    Eigen::Vector3i coordinates;
    int side;
    typename se::Node<FieldType>::value_type parent_val;
  } stack_entry;

  stack_entry stack[Octree<FieldType>::max_depth * 8 + 1];
  size_t stack_idx = 0;

  se::Node<FieldType> *node = map.root();
  if (!node) return collision_status::unseen;

  stack_entry current;
  current.node_ptr = node;
  current.side = map.size();
  current.coordinates = {0, 0, 0};
  stack[stack_idx++] = current;
  collision_status status = collision_status::empty;

  while (stack_idx != 0) {
    node = current.node_ptr;

    if (node->isLeaf()) {
      status = collides_with(static_cast<se::VoxelBlock<FieldType> *>(node), bbox, side, test);
//      std::cout << "[bbox] node leaf" << se::keyops::decode(node->code_).format(InLine)
//      << " status "<< status << std::endl;
      if(status == collision_status::occupied){return status;}
    }

    if (node->children_mask_ == 0) {
      current = stack[--stack_idx];
      continue;
    }

    for (int i = 0; i < 8; ++i) {
      se::Node<FieldType> *child = node->child(i);
      stack_entry child_descr;
      child_descr.node_ptr = NULL;
      child_descr.side = current.side / 2;
      child_descr.coordinates =
          Eigen::Vector3i(current.coordinates(0) + child_descr.side * ((i & 1) > 0),
                          current.coordinates(1) + child_descr.side * ((i & 2) > 0),
                          current.coordinates(2) + child_descr.side * ((i & 4) > 0));

      const bool overlaps = geometry::aabb_aabb_collision(bbox,
                                                          side,
                                                          child_descr.coordinates,
                                                          Eigen::Vector3i::Constant(child_descr.side));

      if (overlaps && child != NULL) {
        child_descr.node_ptr = child;
        child_descr.parent_val = node->value_[0];
        stack[stack_idx++] = child_descr;
      } else if (overlaps && child == NULL) {
        status = update_status(status, test(node->value_[0]));
//        std::cout << "[bbox] node " << se::keyops::decode(node->code_).format(InLine) << " status "
//                  << status << std::endl;
        if(status == collision_status::occupied){return status;}
      }
    }
    current = stack[--stack_idx];
  }
  return status;
}




/**
 * used only in unit test, same logic as in path planning folder
 * @tparam FieldType
 * @param map
 * @param center
 * @param radius
 * @return
 */
template<typename FieldType>
collision_status isSphereCollisionFree(const Octree<FieldType> &map,
                                       const Eigen::Vector3i center,
                                       const int radius) {

  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;
  bool is_voxel_block;
  Eigen::Vector3i prev_pos(0, 0, 0);
  for (int x = -radius; x <= radius; x++) {
    for (int y = -radius; y <= radius; y++) {
      for (int z = -radius; z <= radius; z++) {
        Eigen::Vector3i point_offset_v(x, y, z);
        //check if point is inside the sphere radius
//        std::cout << "sphere norm " << point_offset_v.norm() <<std::endl;
        if (point_offset_v.norm() <= radius) {
          // check if voxelblock is allocated or only node
          Eigen::Vector3i point_v = point_offset_v + center;
          // first round
          if (node == nullptr || block == nullptr) {
            map.fetch_octant(point_v.x(), point_v.y(), point_v.z(), node, is_voxel_block);
            prev_pos = point_v;
            if (is_voxel_block) {
              block = static_cast<se::VoxelBlock<FieldType> *> (node);
            } else {
              if (map.get(se::keyops::decode(node->code_)) < 10.f) {
                std::cout << " [secollision] collision at node "
                          << se::keyops::decode(node->code_).format(InLine) << std::endl;
                return collision_status::occupied;
              }
            }
          } else {
            // if true keep old voxelblock pointer and fetch
            // else get new voxel block
            if ((point_v.x() / BLOCK_SIDE) == (prev_pos.x() / BLOCK_SIDE)
                && ((point_v.y()) / BLOCK_SIDE) == (prev_pos.y() / BLOCK_SIDE)
                && ((point_v.z()) / BLOCK_SIDE) == (prev_pos.z() / BLOCK_SIDE)) {
              if (block->data(point_v) < 10.f) {
                std::cout << " [secollision] collision at " << point_v.format(InLine) << " plog "
                          << block->data(point_v) << std::endl;
                return collision_status::occupied;
              }
            } else {
              map.fetch_octant(point_v.x(), point_v.y(), point_v.z(), node, is_voxel_block);
              if (is_voxel_block) {
                block = static_cast<se::VoxelBlock<FieldType> *> (node);
                if (block->data(point_v) < 10.f) {
                  std::cout << " [secollision] collision at " << point_v.format(InLine) << " plog "
                            << block->data(point_v) << std::endl;
                  return collision_status::occupied;
                }
              } else {
                block = nullptr;
                if (map.get(se::keyops::decode(node->code_)) < 10.f) {
                  std::cout << " [secollision] collision at node "
                            << se::keyops::decode(node->code_).format(InLine) << std::endl;
                  return collision_status::occupied;
                }
              }

            }
          }
          prev_pos = point_v;
        }

      }
    }
  }
//  std::cout << "[se/collision_checker] sphere radius " << planning_config_.cand_view_safety_radius
//            << " [m] =  " << radius_v << " voxels around center " << pos_v.format(InLine) << std::endl;
  return collision_status::empty;
}

}
}
#endif
