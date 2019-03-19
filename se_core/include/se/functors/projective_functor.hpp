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

#include <sophus/se3.hpp>
#include "../utils/math_utils.h"
#include "../algorithms/filter.hpp"
#include "../node.hpp"
#include "../functors/data_handler.hpp"

namespace se {
namespace functor {
  template <typename FieldType, template <typename FieldT> class MapT, 
            typename UpdateF>
  class projective_functor {

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      projective_functor(MapT<FieldType>& map, UpdateF f, const Sophus::SE3f& T_cw, 
          const Eigen::Matrix4f& K, const Eigen::Vector2i frame_size) :
        map_(map), function_(f), T_cw_(T_cw), K_(K), frame_size_(frame_size) {
      } 

      void buildactive_list_() {
        using namespace std::placeholders;
        /* Retrieve the active list */ 
        const se::MemoryPool<se::VoxelBlock<FieldType> >& block_array = 
          map_.getBlockBuffer();

        /* Predicates definition */
        const Eigen::Matrix4f T_cw = T_cw_.matrix();
        const float voxel_size = map_.dim()/map_.size();
        auto in_frustum_predicate = 
          std::bind(algorithms::in_frustum<se::VoxelBlock<FieldType>>, _1, 
              voxel_size, K_*T_cw, frame_size_); 
        auto is_active_predicate = [](const se::VoxelBlock<FieldType>* b) {
          return b->active();
        };

        algorithms::filter(active_list_, block_array, is_active_predicate, 
            in_frustum_predicate);
      }

      void update_block(se::VoxelBlock<FieldType> * block, 
                        const float voxel_size) {
        const Eigen::Vector3i block_coord = block->coordinates();
        const Eigen::Vector3f delta = T_cw_.rotationMatrix() * Eigen::Vector3f(voxel_size, 0, 0);
        const Eigen::Vector3f camera_delta = K_.topLeftCorner<3,3>() * delta;
        bool is_visible = false;

        unsigned int y, z, block_side;
        block_side = se::VoxelBlock<FieldType>::side;
        unsigned int ylast = block_coord(1) + block_side;
        unsigned int zlast = block_coord(2) + block_side;

        for(z = block_coord(2); z < zlast; ++z)
          for (y = block_coord(1); y < ylast; ++y){
            Eigen::Vector3i pix = Eigen::Vector3i(block_coord(0), y, z);
            Eigen::Vector3f start = T_cw_ * Eigen::Vector3f((pix(0)) * voxel_size, 
                (pix(1)) * voxel_size, (pix(2)) * voxel_size);
            Eigen::Vector3f camera_start = K_.topLeftCorner<3,3>() * start;
#pragma omp simd
            for (unsigned int x = 0; x < block_side; ++x){
              pix(0) = x + block_coord(0);
              const Eigen::Vector3f camera_voxel = camera_start + (x*camera_delta);
              const Eigen::Vector3f pos = start + (x*delta);
              if (pos(2) < 0.0001f) continue;

              const float inverse_depth = 1.f / camera_voxel(2);
              const Eigen::Vector2f pixel = Eigen::Vector2f(
                  camera_voxel(0) * inverse_depth + 0.5f,
                  camera_voxel(1) * inverse_depth + 0.5f);
              if (pixel(0) < 0.5f || pixel(0) > frame_size_(0) - 1.5f || 
                  pixel(1) < 0.5f || pixel(1) > frame_size_(1) - 1.5f) continue;
              is_visible = true;

              VoxelBlockHandler<FieldType> handler = {block, pix};
              function_(handler, pix, pos, pixel);
            }
          }
        block->active(is_visible);
      }

      void update_node(se::Node<FieldType> * node, const float voxel_size) { 
        const Eigen::Vector3i voxel = Eigen::Vector3i(unpack_morton(node->code_));
        const Eigen::Vector3f delta = T_cw_.rotationMatrix() * Eigen::Vector3f::Constant(0.5f * voxel_size * node->side_);
        const Eigen::Vector3f delta_c = K_.topLeftCorner<3,3>() * delta;
        Eigen::Vector3f base_cam = T_cw_ * (voxel_size * voxel.cast<float> ());
        Eigen::Vector3f basepix_hom = K_.topLeftCorner<3,3>() * base_cam;

#pragma omp simd
        for(int i = 0; i < 8; ++i) {
          const Eigen::Vector3i dir =  Eigen::Vector3i((i & 1) > 0, (i & 2) > 0, (i & 4) > 0);
          const Eigen::Vector3f vox_cam = base_cam + dir.cast<float>().cwiseProduct(delta); 
          const Eigen::Vector3f pix_hom = basepix_hom + dir.cast<float>().cwiseProduct(delta_c); 

          if (vox_cam(2) < 0.0001f) continue;
          const float inverse_depth = 1.f / pix_hom(2);
          const Eigen::Vector2f pixel = Eigen::Vector2f(
              pix_hom(0) * inverse_depth + 0.5f,
              pix_hom(1) * inverse_depth + 0.5f);
          if (pixel(0) < 0.5f || pixel(0) > frame_size_(0) - 1.5f || 
              pixel(1) < 0.5f || pixel(1) > frame_size_(1) - 1.5f) continue;

          NodeHandler<FieldType> handler = {node, i};
          function_(handler, voxel + dir, vox_cam, pixel);
        }
      }

      void apply() {

        buildactive_list_();
        const float voxel_size = map_.dim() / map_.size();
        size_t list_size = active_list_.size();
#pragma omp parallel for
        for(unsigned int i = 0; i < list_size; ++i){
          update_block(active_list_[i], voxel_size);
        }
        active_list_.clear();

        auto& nodes_list = map_.getNodesBuffer();
        list_size = nodes_list.size();
#pragma omp parallel for
          for(unsigned int i = 0; i < list_size; ++i){
            update_node(nodes_list[i], voxel_size);
         }
      }

    private:
      MapT<FieldType>& map_; 
      UpdateF function_; 
      Sophus::SE3f T_cw_;
      Eigen::Matrix4f K_;
      Eigen::Vector2i frame_size_;
      std::vector<se::VoxelBlock<FieldType>*> active_list_;
  };

  template <typename FieldType, template <typename FieldT> class MapT, 
            typename UpdateF>
  void projective_map(MapT<FieldType>& map, const Sophus::SE3f& T_cw, 
          const Eigen::Matrix4f& K, const Eigen::Vector2i frame_size,
          UpdateF funct) {

    projective_functor<FieldType, MapT, UpdateF> 
      it(map, funct, T_cw, K, frame_size);
    it.apply();
  }
}
}
#endif
