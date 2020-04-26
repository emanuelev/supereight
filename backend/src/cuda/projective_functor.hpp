/*
    Copyright 2016 Emanuele Vespa, Imperial College London
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include "kernels.hpp"

#include <supereight/algorithms/filter.hpp>
#include <supereight/functors/data_handler.hpp>
#include <supereight/node.hpp>
#include <supereight/utils/math_utils.h>

#include <functional>
#include <sophus/se3.hpp>
#include <vector>

namespace se {
namespace functor {

template<typename FieldType, template<typename> typename BufferT,
    template<typename, template<typename> typename> class MapT,
    typename UpdateF>
class projective_functor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    projective_functor(MapT<FieldType, BufferT>& map, UpdateF f,
        const Sophus::SE3f& Tcw, const Eigen::Matrix4f& K,
        const Eigen::Vector2i framesize)
        : _map(map), _function(f), _Tcw(Tcw), _K(K), _frame_size(framesize) {}

    void update_node(se::Node<FieldType>* node, const float voxel_size) {
        const Eigen::Vector3i voxel =
            Eigen::Vector3i(unpack_morton(node->code_));
        const Eigen::Vector3f delta = _Tcw.rotationMatrix() *
            Eigen::Vector3f::Constant(0.5f * voxel_size * node->side_);
        const Eigen::Vector3f delta_c = _K.topLeftCorner<3, 3>() * delta;
        Eigen::Vector3f base_cam    = _Tcw * (voxel_size * voxel.cast<float>());
        Eigen::Vector3f basepix_hom = _K.topLeftCorner<3, 3>() * base_cam;

#pragma omp simd
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3i dir =
                Eigen::Vector3i((i & 1) > 0, (i & 2) > 0, (i & 4) > 0);
            const Eigen::Vector3f vox_cam =
                base_cam + dir.cast<float>().cwiseProduct(delta);
            const Eigen::Vector3f pix_hom =
                basepix_hom + dir.cast<float>().cwiseProduct(delta_c);

            if (vox_cam(2) < 0.0001f) continue;
            const float inverse_depth = 1.f / pix_hom(2);
            const Eigen::Vector2f pixel =
                Eigen::Vector2f(pix_hom(0) * inverse_depth + 0.5f,
                    pix_hom(1) * inverse_depth + 0.5f);
            if (pixel(0) < 0.5f || pixel(0) > _frame_size(0) - 1.5f ||
                pixel(1) < 0.5f || pixel(1) > _frame_size(1) - 1.5f)
                continue;

            NodeHandler<FieldType> handler = {node, i};
            _function(handler, voxel + dir, vox_cam, pixel);
        }
    }

    void apply() {
        updateBlocks(_map, _function, _Tcw, _K, _frame_size);

        const float voxel_size = _map.dim() / _map.size();
        auto& nodes_list       = _map.getNodesBuffer();
        list_size              = nodes_list.used();
#pragma omp parallel for
        for (unsigned int i = 0; i < list_size; ++i) {
            update_node(nodes_list[i], voxel_size);
        }
    }

private:
    MapT<FieldType, BufferT>& _map;
    UpdateF _function;
    Sophus::SE3f _Tcw;
    Eigen::Matrix4f _K;
    Eigen::Vector2i _frame_size;
    std::vector<se::VoxelBlock<FieldType>*> _active_list;
};

template<typename FieldType, template<typename> typename BufferT,
    template<typename, template<typename> typename> class MapT,
    typename UpdateF>
void projective_map(MapT<FieldType, BufferT>& map, const Sophus::SE3f& Tcw,
    const Eigen::Matrix4f& K, const Eigen::Vector2i framesize, UpdateF funct) {
    projective_functor<FieldType, BufferT, MapT, UpdateF> it(
        map, funct, Tcw, K, framesize);
    it.apply();
}

} // namespace functor
} // namespace se
