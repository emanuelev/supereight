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
#include "Eigen/Dense"
#include "functors/data_handler.hpp"
namespace se {

template<typename T>
class node_iterator {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vec3i;
  node_iterator(const Octree<T> &m) : map_(m) {
    state_ = BRANCH_NODES;
    last = 0;
  };

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

  vec3i getSurfaceVoxels(float threshold = 0.25) {
    vec3i surfaceVoxels;
    surfaceVoxels.clear();

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
            if (value.x >= -threshold && value.x <= threshold) {
              surfaceVoxels.push_back(vox);
            }
          }
        }
      }
    }
    return surfaceVoxels;
  }

  vec3i getOccupiedVoxels(float threshold, const uint64_t morton) {
    vec3i occupiedVoxels;
    occupiedVoxels.clear();
  Eigen::Vector3i blockCoord = unpack_morton(morton);
//    VoxelBlock<T>* block = map_.fetch(blockCoord(0), blockCoord(1), blockCoord(2));

    int xlast = blockCoord(0) + BLOCK_SIDE;
    int ylast = blockCoord(1) + BLOCK_SIDE;
    int zlast = blockCoord(2) + BLOCK_SIDE;
#pragma omp parallel for
    for (int z = blockCoord(2); z < zlast; ++z) {
      for (int y = blockCoord(1); y < ylast; ++y) {
        for (int x = blockCoord(0); x < xlast; ++x) {
          typename VoxelBlock<T>::value_type value;
          const Eigen::Vector3i vox{x, y, z};
          float prob = se::math::getProbFromLog(map_.get(x, y, z).x);
//          value = block->data(Eigen::Vector3i(x, y, z));
// TODO use state
          if (prob >= threshold) {
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
  vec3i getFrontierVoxels(float threshold, const uint64_t morton) {
    vec3i frontierVoxels;
    frontierVoxels.clear();

    Eigen::Vector3i blockCoord = unpack_morton(morton);
    int xlast = blockCoord(0) + BLOCK_SIDE;
    int ylast = blockCoord(1) + BLOCK_SIDE;
    int zlast = blockCoord(2) + BLOCK_SIDE;
#pragma omp parallel for
    for (int z = blockCoord(2); z < zlast; ++z) {
      for (int y = blockCoord(1); y < ylast; ++y) {
        for (int x = blockCoord(0); x < xlast; ++x) {

          const Eigen::Vector3i vox{x, y, z};
          if (map_.get(x, y, z).st == voxel_state::kFrontier) {
//          value = block->data(Eigen::Vector3i(x, y, z));
#pragma omp critical
            frontierVoxels.push_back(vox);
          }
        }
      }
    }
    return frontierVoxels;
  }

  vec3i getOccludedVoxels(float threshold, const uint64_t morton) {
    vec3i occludedVoxels;
    occludedVoxels.clear();

    Eigen::Vector3i blockCoord = unpack_morton(morton);
    int xlast = blockCoord(0) + BLOCK_SIDE;
    int ylast = blockCoord(1) + BLOCK_SIDE;
    int zlast = blockCoord(2) + BLOCK_SIDE;
#pragma omp parallel for
    for (int z = blockCoord(2); z < zlast; ++z) {
      for (int y = blockCoord(1); y < ylast; ++y) {
        for (int x = blockCoord(0); x < xlast; ++x) {

          const Eigen::Vector3i vox{x, y, z};
          if (map_.get(x, y, z).st == voxel_state::kOccluded) {
//          value = block->data(Eigen::Vector3i(x, y, z));
#pragma omp critical
            occludedVoxels.push_back(vox);
          }
        }
      }
    }
    return occludedVoxels;
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
  bool deleteFrontierVoxelBlockviaMorton(const Eigen::Vector3i &blockCoord) {
//    VoxelBlock<T>* block = map_.fetch(blockCoord(0), blockCoord(1), blockCoord(2));
    bool has_frontier = false;

    int xlast = blockCoord(0) + BLOCK_SIDE;
    int ylast = blockCoord(1) + BLOCK_SIDE;
    int zlast = blockCoord(2) + BLOCK_SIDE;
    se::VoxelBlock<T> *block = map_.fetch(blockCoord(0), blockCoord(1), blockCoord(2));
    for (int z = blockCoord(2); z < zlast; ++z) {
      for (int y = blockCoord(1); y < ylast; ++y) {
        for (int x = blockCoord(0); x < xlast; ++x) {
          // make handler with the current voxel
          VoxelBlockHandler<T> handler = {block, Eigen::Vector3i(x, y, z)};
          auto data = handler.get();
//          std::cout << "[se/node_it] state "<< map_.get(x,y,z).st << std::endl;
          if (map_.get(x, y, z).st == voxel_state::kFrontier && !handler.isFrontier(map_)) {
//            std::cout << " [supereight/node it] voxel is a frontier" << std::endl;
            // check for the curr voxel if it a Frontier / has an unknown voxel next to it
            if (data.x <= THRESH_FREE) {
              data.st = voxel_state::kFree;
//              std::cout << "[superegiht/node_it] frontier=> free , voxel " << x << " " << y << " "
//                        << z << std::endl;
            } else if (data.x >= THRESH_OCC) {
              data.st = voxel_state::kOccupied;
            }
            handler.set(data);
            has_frontier = true;
          }

        }
      }
    }
    return has_frontier;
  }

   /**
   * update all frontier voxels, targeting the frontier voxel just above floor
   * @param blockCoord bottom left voxel block (node) voxel
   * @return
   */
  bool deleteOcclusionVoxelsviaMorton(const Eigen::Vector3i &blockCoord) {
//    VoxelBlock<T>* block = map_.fetch(blockCoord(0), blockCoord(1), blockCoord(2));
    bool has_occlusion = false;

    int xlast = blockCoord(0) + BLOCK_SIDE;
    int ylast = blockCoord(1) + BLOCK_SIDE;
    int zlast = blockCoord(2) + BLOCK_SIDE;
    se::VoxelBlock<T> *block = map_.fetch(blockCoord(0), blockCoord(1), blockCoord(2));
    for (int z = blockCoord(2); z < zlast; ++z) {
      for (int y = blockCoord(1); y < ylast; ++y) {
        for (int x = blockCoord(0); x < xlast; ++x) {
          // make handler with the current voxel
          VoxelBlockHandler<T> handler = {block, Eigen::Vector3i(x, y, z)};
          auto data = handler.get();
          if (map_.get(x, y, z).st == voxel_state::kOccluded) {
            if (data.x <= THRESH_FREE) {
              data.st = voxel_state::kFree;
              std::cout << "[superegiht/node_it] frontier=> free , voxel " << x << " " << y << " "
                        << z << std::endl;
            } else if (data.x >= THRESH_OCC) {
              data.st = voxel_state::kOccupied;
            }
            handler.set(data);
            has_occlusion = true;
          }

        }
      }
    }
    return has_occlusion;
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
