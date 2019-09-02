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

#ifndef PROJECTIVE_FUNCTOR_HPP
#define PROJECTIVE_FUNCTOR_HPP
#include <functional>
#include <vector>
#include <Eigen/StdVector>

#include <sophus/se3.hpp>
#include "../utils/math_utils.h"
#include "../algorithms/filter.hpp"
#include "../node.hpp"
#include "../functors/data_handler.hpp"

namespace se {
namespace functor {
template<typename FieldType, template<typename FieldT> class MapT, typename UpdateF>
class projective_functor {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  projective_functor(MapT<FieldType> &map,
                     UpdateF &f,
                     const Sophus::SE3f &Tcw,
                     const Eigen::Matrix4f &K,
                     const Eigen::Vector2i &framesize)
      :
      _map(map), _function(f), _Tcw(Tcw), _K(K), _frame_size(framesize) {
  }

  /*! \brief Get all the blocks that are active or inside the camera
   * frustum. The blocks are stored in projective_functor::_active_list.
   */
  void build_active_list() {
    using namespace std::placeholders;
    /* Retrieve the active list */
    const se::MemoryPool<se::VoxelBlock<FieldType> > &block_array = _map.getBlockBuffer();

    /* Predicates definition */
    const Eigen::Matrix4f Tcw = _Tcw.matrix();
    const float voxel_size = _map.dim() / _map.size();
    /* Check if a given voxel block projects into the image
     * WARNING: Returns true even if only the coorner cooresponding to the block coordinates
     *          is within the image.
     *          Returns false if entire block is within the frustum apart from the corner
     *          corresponding to the block coordinates.
     */
    auto in_frustum_predicate = std::bind(algorithms::in_frustum<se::VoxelBlock<FieldType>>,
                                          _1,
                                          voxel_size,
                                          _K * _Tcw.matrix(),
                                          _frame_size);
    /* Check if a given voxel block is active */
    auto is_active_predicate = [](const se::VoxelBlock<FieldType> *b) {
      return b->active();
    };

    /* Adds all voxel blocks to the active list that are either active OR within the frustum */
    algorithms::filter(_active_list, block_array, is_active_predicate, in_frustum_predicate);
  }

  /*! \brief Get all the Nodes that are inside the camera frustum. The Nodes
   * are stored in projective_functor::_in_frustum_node_list.
   */
  void build_in_frustum_node_list() {
    const se::MemoryPool<se::Node<FieldType> >& node_array
        = _map.getNodesBuffer();

    /* Predicates definition */
    const Eigen::Matrix4f Tcw = _Tcw.matrix();
    const float voxel_size = _map.dim() / _map.size();
    auto in_frustum_predicate = std::bind(
        algorithms::in_frustum<se::Node<FieldType>>, std::placeholders::_1,
        voxel_size, _K * _Tcw.matrix(), _frame_size);

    algorithms::filter(_in_frustum_node_list, node_array, in_frustum_predicate);
  }

  void update_block(se::VoxelBlock<FieldType> *block, const float voxel_size) {
    /* Is this the VoxelBlock center? */
    const Eigen::Vector3i blockCoord = block->coordinates();
    /* Change of the voxel position in camera frame when x is increased by 1 in world frame */
    const Eigen::Vector3f delta = _Tcw.rotationMatrix() * Eigen::Vector3f(voxel_size, 0, 0);
    /* Change of the pixel coordinate when x is increased by 1 in world frame */
    const Eigen::Vector3f cameraDelta = _K.topLeftCorner<3, 3>() * delta;
    bool is_visible = false;

    unsigned int y, z, blockSide;
    blockSide = se::VoxelBlock<FieldType>::side; // 8
    unsigned int ylast = blockCoord(1) + blockSide;
    unsigned int zlast = blockCoord(2) + blockSide;

    block->occupancyUpdated(false);
    // goes through 8 by 8 block
    for (z = blockCoord(2); z < zlast; ++z) {
      for (y = blockCoord(1); y < ylast; ++y) {
        /* This variable has nothing to do with pix but is the side starting point of the side
           of a given voxel block , [voxel coord]*/
        Eigen::Vector3i pix = Eigen::Vector3i(blockCoord(0), y, z);
        // Starting voxel in world frame for the iteration over a voxel line (x in [0; blockSide])
        // in world frame
        Eigen::Vector3f start = _Tcw
            * Eigen::Vector3f((pix(0)) * voxel_size, (pix(1)) * voxel_size, (pix(2)) * voxel_size);
        // Starting "pixel" (depth * [u, v, 1]) for the interation over a voxel line
        // (x in [0, blockSide])
        Eigen::Vector3f camerastart = _K.topLeftCorner<3, 3>() * start;
#pragma omp simd
        for (unsigned int x = 0; x < blockSide; ++x) {
          pix(0) = x + blockCoord(0);
          // Compute "pixel" (depth * [u, v, 1]) for a new x
          const Eigen::Vector3f camera_voxel = camerastart + (x * cameraDelta);
          // Compute voxel position in camera frame for a new x in world frame [m]
          const Eigen::Vector3f pos = start + (x * delta);
          if (pos(2) < 0.0001f) continue;

          const float inverse_depth = 1.f / camera_voxel(2);
          // Compute the actual image coordinates ([u, v, 1]) from pixel (depth * [u, v, 1])
          const Eigen::Vector2f pixel = Eigen::Vector2f(camera_voxel(0) * inverse_depth + 0.5f,
                                                        camera_voxel(1) * inverse_depth + 0.5f);
          // Check if the image coordinates are within the image
          if (pixel(0) < 0.5f || pixel(0) > _frame_size(0) - 1.5f || pixel(1) < 0.5f
              || pixel(1) > _frame_size(1) - 1.5f)
            continue;

          // Set voxel to visible to later active the block
          is_visible = true;
          // Update the voxel. pix is curr 3D block coord
          VoxelBlockHandler<FieldType> handler = {block, pix};
          _function(handler, pix, pos, pixel, _map);
        }
      }
    }
    block->active(is_visible); // TODO: Blocks are activate during allocation anyway
  }

  void update_node(se::Node<FieldType> *node, const float voxel_size) {
    // Extract the coordinates of the node (bottom, left, front corner) in [vox]
    const Eigen::Vector3i voxel = Eigen::Vector3i(unpack_morton(node->code_));
    /* Compute the change in position in [m] in x, y, z, when increasing the position
       by half the node size in [m] in all directions */
    const Eigen::Vector3f
        delta = _Tcw.rotationMatrix() * Eigen::Vector3f::Constant(0.5f * voxel_size * node->side_);
    /* Compute the change in "pixel" coordinates (depth * [u, v, 1]) when increasing the position
       by half the node size in [m] in all directions */
    const Eigen::Vector3f delta_c = _K.topLeftCorner<3, 3>() * delta;
    /* Compute starting position in camera frame for the iteration over all 8 corners*/
    Eigen::Vector3f base_cam = _Tcw * (voxel_size * voxel.cast<float>());
    /* Compute starting "pixel" (depth * [u, v, 1]) for the iteration over all 8 corners*/
    Eigen::Vector3f basepix_hom = _K.topLeftCorner<3, 3>() * base_cam;

    /* Iterate over the Node children. */
#pragma omp simd
    /* Iterate over all 8 corners */
    for (int i = 0; i < 8; ++i) {
      const Eigen::Vector3i dir = Eigen::Vector3i((i & 1) > 0, (i & 2) > 0, (i & 4) > 0);
      /* Compute ith corner in camera frame correspoinding to the ith node corner */
      const Eigen::Vector3f vox_cam = base_cam + dir.cast<float>().cwiseProduct(delta);
      /* Compute ith "pixel" (depth * [u, v, 1]) for the ith node corner */
      const Eigen::Vector3f pix_hom = basepix_hom + dir.cast<float>().cwiseProduct(delta_c);

      /* Ignore voxel if less than 0.1mm away from the camera */
      if (vox_cam(2) < 0.0001f) continue;

      /* Compute image coordinates ([u, v, 1]) */
      const float inverse_depth = 1.f / pix_hom(2);
      const Eigen::Vector2f pixel =
          Eigen::Vector2f(pix_hom(0) * inverse_depth + 0.5f, pix_hom(1) * inverse_depth + 0.5f);
      /* Check if the corner projects into the image */
      if (pixel(0) < 0.5f || pixel(0) > _frame_size(0) - 1.5f || pixel(1) < 0.5f
          || pixel(1) > _frame_size(1) - 1.5f)
        continue;

      /* Get handler for ith child of the given node */
      NodeHandler<FieldType> handler = {node, i};

      /* Update the ith child of the given node */
      _function(handler, voxel + dir, vox_cam, pixel, _map); // voxel + dir seems wrong;
      // should be voxel + dir * node->side_ / 2
    }
  }

  /**
   * Update all active nodes/voxels
   */
  void apply() {
//    /* Update the leaf Octree nodes (VoxelBlock). */
//    build_active_list();
//    const float voxel_size = _map.dim() / _map.size();
//    size_t list_size = _active_list.size();
// #pragma omp parallel for
//    for (unsigned int i = 0; i < list_size; ++i) {
//      update_block(_active_list[i], voxel_size);
//    }
//    _active_list.clear();

    /* Update the intermediate Octree nodes (Node). */
    build_in_frustum_node_list();
    const float voxel_size = _map.dim() / _map.size();
#pragma omp parallel for
    for (unsigned int i = 0; i < _in_frustum_node_list.size(); ++i) {
      update_node(_in_frustum_node_list[i], voxel_size);
    }
    _in_frustum_node_list.clear();

  }

 private:
  MapT<FieldType> &_map;
  UpdateF _function;
  Sophus::SE3f _Tcw;
  Eigen::Matrix4f _K;
  Eigen::Vector2i _frame_size;
  std::vector<se::VoxelBlock<FieldType> *> _active_list;
  std::vector<se::Node<FieldType> *> _in_frustum_node_list;
};

/*! \brief Create a projective_functor and call projective_functor::apply.
 *
 * Update all active nodes/voxels using the provided function
 * @param[in] map         Octree
 * @param[in] Tcw         World to camera transformation
 * @param[in] K           Intrinsic camera parameter
 * @param[in] framesize   Size of the depth image
 * @param[in] funct       Function to update the node/voxel values
 */
template<typename FieldType, template<typename FieldT> class MapT, typename UpdateF>
void projective_map(MapT<FieldType> &map,
                    const Sophus::SE3f &Tcw,
                    const Eigen::Matrix4f &K,
                    const Eigen::Vector2i &framesize,
                    UpdateF &funct) {
  projective_functor<FieldType, MapT, UpdateF> it(map, funct, Tcw, K, framesize);
  it.apply();
}
}
}
#endif
