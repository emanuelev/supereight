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

#ifndef NODE_ITERATOR_H
#define NODE_ITERATOR_H
#include "octree.hpp"
#include <Eigen/Dense>
#include "functors/data_handler.hpp"
namespace se {

/*! \brief Iterate through all the nodes (first Node and then VoxelBlock nodes)
 * of the Octree.
 */
template<typename T>
class node_iterator {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vec3i;
  node_iterator(const Octree<T> &m) : map_(m) {
    state_ = BRANCH_NODES;
    last = 0;
  };

  /*! \brief Get a pointer to the next node in the Octree.
   * Starting from the root node, each time this function is called will return
   * a pointer to the next non-leaf (Node) node. Once all non-leaf nodes have
   * been iterated through, it will start iterating through the leaf nodes
   * (VoxelBlock).
   *
   * \return A pointer to the next node. Returns nullptr if all nodes have been
   * iterated through.
   */
  Node<T> *next() {
    switch (state_) {
      case BRANCH_NODES:
        if (last < map_.nodes_buffer_.size()) {
          Node<T> *n = map_.nodes_buffer_[last++];
          return n;
        } else {
          last = 0;
          state_ = LEAF_NODES;
          return next();
        }
        break;
      case LEAF_NODES:
        if (last < map_.block_buffer_.size()) {
          VoxelBlock<T> *n = map_.block_buffer_[last++];
          return n;
          /* the above int init required due to odr-use of static member */
        } else {
          last = 0;
          state_ = FINISHED;
          return nullptr;
        }
        break;
      case FINISHED:return nullptr;
    }
    return nullptr;
  }

  vec3i getOccupiedVoxels(float threshold = 0.5) {
    vec3i occupiedVoxels;
    occupiedVoxels.clear();

    for (int block_idx = 0; block_idx < map_.block_buffer_.size(); block_idx++) {
      VoxelBlock<T> *block = map_.block_buffer_[block_idx];
      const Eigen::Vector3i blockCoord = block->coordinates();

      int x, y, z;
      int xlast = blockCoord(0) + BLOCK_SIDE;
      int ylast = blockCoord(1) + BLOCK_SIDE;
      int zlast = blockCoord(2) + BLOCK_SIDE;
      for (z = blockCoord(2); z < zlast; ++z) {
        for (y = blockCoord(1); y < ylast; ++y) {
          for (x = blockCoord(0); x < xlast; ++x) {
            typename VoxelBlock<T>::value_type value;
            const Eigen::Vector3i vox{x, y, z};
            value = block->data(Eigen::Vector3i(x, y, z));
            if (value.x >= threshold) {
              occupiedVoxels.push_back(vox);
            }
          }
        }
      }
    }
    return occupiedVoxels;
  }
  // for rviz viauslization
  vec3i getFreeVoxels(const uint64_t morton) {
    vec3i freeVoxels;
    freeVoxels.clear();

    typename VoxelBlock<T>::value_type value;
    const Eigen::Vector3i blockCoord = keyops::decode(morton);
    const int level = keyops::level(morton);

    Node<T> *node = nullptr;
    bool is_block = false;
    map_.fetch_octant(blockCoord(0), blockCoord(1), blockCoord(2), node, is_block);
    if (!is_block) {
      freeVoxels.push_back(blockCoord);
      freeVoxels.push_back(Eigen::Vector3i(0, 0, 0));
    } else {
      VoxelBlock<T> *block = static_cast< VoxelBlock<T> *> (node);
      const int xlast = blockCoord(0) + BLOCK_SIDE;
      const int ylast = blockCoord(1) + BLOCK_SIDE;
      const int zlast = blockCoord(2) + BLOCK_SIDE;
      for (int z = blockCoord(2); z < zlast; ++z) {
        for (int y = blockCoord(1); y < ylast; ++y) {
          for (int x = blockCoord(0); x < xlast; ++x) {
            const Eigen::Vector3i vox{x, y, z};
            value = block->data(vox);
            if (value.st == voxel_state::kFree) {
              freeVoxels.push_back(vox);
            }
          }
        }
      }
    }
    return freeVoxels;
  }

// for boundary check
  bool getFreeVoxel(const key_t morton, Eigen::Vector3i &free_voxel) {

    typename VoxelBlock<T>::value_type value;

    Node<T> *node = nullptr;
    bool is_block = false;
    map_.fetch_octant(free_voxel(0), free_voxel(1), free_voxel(2), node, is_block);
    const Eigen::Vector3i blockCoord = free_voxel;
    if (!is_block) {

      // std::cout << "[se/nodeit] node " << free_voxel.format(InLine) << std::endl;
      return true;
    } else {
      VoxelBlock<T> *block = static_cast< VoxelBlock<T> *> (node);
      const int xlast = blockCoord(0) + BLOCK_SIDE;
      const int ylast = blockCoord(1) + BLOCK_SIDE;
      const int zlast = blockCoord(2) + BLOCK_SIDE;
      for (int z = blockCoord(2); z < zlast; ++z) {
        for (int y = blockCoord(1); y < ylast; ++y) {
          for (int x = blockCoord(0); x < xlast; ++x) {
            const Eigen::Vector3i vox{x, y, z};
            value = block->data(vox);
            if (value.x < 0.f) {
              free_voxel = vox;
              // std::cout << "[se/nodeit] block " << free_voxel.format(InLine) << std::endl;
              return true;
            }
          }
        }
      }
    }
    // std::cout <<"[se/nodeit] vb with no free voxel " << std::endl;
    return false;
  }
  // rviz visualization
  vec3i getOccupiedVoxels(float threshold, const uint64_t morton) {
    vec3i occupiedVoxels;
    occupiedVoxels.clear();
    const Eigen::Vector3i blockCoord = keyops::decode(morton);
    const VoxelBlock<T> *block = map_.fetch(morton);

    typename VoxelBlock<T>::value_type value;
    const int xlast = blockCoord(0) + BLOCK_SIDE;
    const int ylast = blockCoord(1) + BLOCK_SIDE;
    const int zlast = blockCoord(2) + BLOCK_SIDE;
#pragma omp parallel for
    for (int z = blockCoord(2); z < zlast; ++z) {
      for (int y = blockCoord(1); y < ylast; ++y) {
        for (int x = blockCoord(0); x < xlast; ++x) {
          const Eigen::Vector3i vox{x, y, z};
//          float prob = map_.get(x, y, z).x;
          value = block->data(vox);
// TODO use state
          if (value.x > threshold) {
#pragma omp critical
            occupiedVoxels.push_back(vox);
          }
        }
      }
    }
    return occupiedVoxels;
  }
  /**
   *
   * @param threshold upper and lower range for frontier voxel probability
   * @param blockCoord of frontier voxel block
   * @return vector with all frontier voxels
   */
  vec3i getFrontierVoxels(const uint64_t morton) {
    vec3i frontierVoxels;
    frontierVoxels.clear();

    typename VoxelBlock<T>::value_type value;
    const Eigen::Vector3i blockCoord = keyops::decode(morton);
    const int level = keyops::level(morton);

    Node<T> *node = nullptr;
    bool is_block = false;
    map_.fetch_octant(blockCoord(0), blockCoord(1), blockCoord(2), node, is_block);
    if (is_block) {
      VoxelBlock<T> *block = static_cast< VoxelBlock<T> *> (node);
      const int xlast = blockCoord(0) + BLOCK_SIDE;
      const int ylast = blockCoord(1) + BLOCK_SIDE;
      const int zlast = blockCoord(2) + BLOCK_SIDE;
#pragma omp parallel for
      for (int z = blockCoord(2); z < zlast; ++z) {
        for (int y = blockCoord(1); y < ylast; ++y) {
          for (int x = blockCoord(0); x < xlast; ++x) {

            const Eigen::Vector3i vox{x, y, z};
            value = block->data(vox);
            if (value.st == voxel_state::kFrontier) {
#pragma omp critical
              frontierVoxels.push_back(vox);
            }
          }
        }
      }
    } else {
      // std::cout << " node frontier " << blockCoord.format(InLine) << std::endl;
      frontierVoxels.push_back(blockCoord);
      frontierVoxels.push_back(Eigen::Vector3i(0, 0, 0));
    }
    return frontierVoxels;
  }

/**
 * check if frontier voxels are in the voxel block
 * @param blockCoord
 * @return
 */
  bool hasFrontierVoxelBlockviaMorton(const Eigen::Vector3i &blockCoord) {

//    VoxelBlock<T>* block = map_.fetch(blockCoord(0), blockCoord(1), blockCoord(2));
    bool has_frontier = false;
    int xlast = blockCoord(0) + BLOCK_SIDE;
    int ylast = blockCoord(1) + BLOCK_SIDE;
    int zlast = blockCoord(2) + BLOCK_SIDE;

    for (int z = blockCoord(2); z < zlast; ++z) {
      for (int y = blockCoord(1); y < ylast; ++y) {
        for (int x = blockCoord(0); x < xlast; ++x) {
          if (map_.get(x, y, z).st == voxel_state::kFrontier) {

            return true;

          }

        }
      }
    }

    return false;
  }
/**
 * update all frontier voxels, targeting the frontier voxel just above floor
 * @param blockCoord bottom left voxel block (node) voxel
 * @return
 */
  bool deleteFrontierVoxelBlockviaMorton(const key_t morton) {
//    VoxelBlock<T>* block = map_.fetch(blockCoord(0), blockCoord(1), blockCoord(2));
    bool has_frontier = false;

    const Eigen::Vector3i blockCoord = keyops::decode(morton);
    Node<T> *node = nullptr;
    bool is_block = false;
    // returned node could be at any level, but currently only frontier node at leaf level
    map_.fetch_octant(blockCoord(0), blockCoord(1), blockCoord(2), node, is_block);

    if (is_block) {
      VoxelBlock<T> *block = static_cast< VoxelBlock<T> *> (node);
      const int xlast = blockCoord(0) + BLOCK_SIDE;
      const int ylast = blockCoord(1) + BLOCK_SIDE;
      const int zlast = blockCoord(2) + BLOCK_SIDE;
      for (int z = blockCoord(2); z < zlast; ++z) {
        for (int y = blockCoord(1); y < ylast; ++y) {
          for (int x = blockCoord(0); x < xlast; ++x) {
            // make handler with the current voxel
            VoxelBlockHandler<T> handler = {block, Eigen::Vector3i(x, y, z)};
            auto data = handler.get();
//          std::cout << "[se/node_it] state "<< map_.get(x,y,z).st << std::endl;
            if (data.st == voxel_state::kFrontier && !handler.isFrontier(map_)) {
//            std::cout << " [supereight/node it] voxel is a frontier" << std::endl;
              // check for the curr voxel if it a Frontier / has an unknown voxel next to it
              if (data.x <= THRESH_FREE) {
                data.st = voxel_state::kFree;
//              std::cout << "[superegiht/node_it] frontier=> free , voxel " << x << " " << y << " "
//                        << z << std::endl;
              } else if (data.x >= THRESH_OCC) {
                data.st = voxel_state::kOccupied;
              } else {
                data.st = voxel_state::kUnknown;
              }
              // std::cout << "frontier update state " << data.st << std::endl;
              handler.set(data);

            } else if (handler.isFrontier(map_)) {
              has_frontier = true;
            }

          }
        }
      }
    } else {
      std::cout << "delete node frontier" << std::endl;
      // data handler set parent_node ->value[idx];
      // TODO include when we consider frontier nodes at lower than leaf level
      const int level = keyops::level(morton);
//      Node<T> *parent_node =
//          map_.fetch_octant(blockCoord.x(), blockCoord.y(), blockCoord.z(), level - 1);
//      const int edge = (map_.leaf_level()-level ) * BLOCK_SIDE;
//      const int idx = ((blockCoord.x() & edge) > 0) + 2 * ((blockCoord.y() & edge) > 0)
//          + 4 * ((blockCoord.z() & edge) > 0);
      const int childid = child_id(morton, level, map_.max_level());

//      NodeHandler<T> handler = {parent_node, idx}; // pointer to parent node and idx of this node
      NodeHandler<T> handler = {node, childid}; // pointer to parent node and idx of this node
      auto data = handler.get();

      if (data.st == voxel_state::kFrontier && !handler.isFrontier(map_)) {
        if (data.x <= THRESH_FREE) {
          data.st = voxel_state::kFree;
//              std::cout << "[superegiht/node_it] frontier=> free , voxel " << x << " " << y << " "
//                        << z << std::endl;
        } else if (data.x >= THRESH_OCC) {
          data.st = voxel_state::kOccupied;
        }
        handler.set(data);

      } else if (handler.isFrontier(map_)) {
        has_frontier = true;
      }
    }
    return has_frontier;
  }

 private:
  typedef enum ITER_STATE {
    BRANCH_NODES, LEAF_NODES, FINISHED
  } ITER_STATE;

  const Octree<T> &map_;
  ITER_STATE state_;
  size_t last;
};
}
#endif
