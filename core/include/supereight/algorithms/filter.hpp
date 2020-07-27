/*
 * Copyright 2016 Emanuele Vespa, Imperial College London
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * */

#ifndef ACTIVE_LIST_HPP
#define ACTIVE_LIST_HPP

#include "../node.hpp"
#include "../utils/math_utils.h"
#include "../utils/morton_utils.hpp"

#include <supereight/shared/commons.h>

namespace se {
namespace algorithms {

template<typename VoxelBlockType>
SE_DEVICE_FUNC static inline bool in_frustum(const VoxelBlockType* v,
    float voxelSize, const Eigen::Matrix4f& camera,
    const Eigen::Vector2i& frameSize) {
    constexpr static int side = VoxelBlockType::side;

    // clang-format off

    constexpr static int offset_data[] = {
         0,    side, 0,    side, 0,    side, 0,    side,
         0,    0,    side, side, 0,    0,    side, side,
         0,    0,    0,    0,    side, side, side, side,
         0,    0,    0,    0,    0,    0,    0,    0
    };

    const auto offsets = Eigen::Matrix<int, 4, 8>{offset_data};

    // clang-format on

    Eigen::Matrix<float, 4, 8> v_camera = camera *
        Eigen::Vector4f(voxelSize, voxelSize, voxelSize, 1.f).asDiagonal() *
        (offsets.colwise() + v->coordinates().homogeneous())
            .template cast<float>();

    v_camera.row(0).array() /= v_camera.row(2).array();
    v_camera.row(1).array() /= v_camera.row(2).array();

    return ((v_camera.row(0).array() >= 0.f &&
                v_camera.row(0).array() < frameSize.x()) &&
        (v_camera.row(1).array() >= 0.f &&
            v_camera.row(1).array() < frameSize.y()))
        .any();
}

template<typename VoxelBlockType>
SE_DEVICE_FUNC static inline bool in_frustum2(const VoxelBlockType* v,
    float voxel_size, const Eigen::Matrix4f& camera,
    const Eigen::Vector2i& frame_size) {
    constexpr static int side = VoxelBlockType::side;
    Eigen::Vector3i coords    = v->coordinates();

    for (int z = 0; z <= side; z += side) {
        for (int y = 0; y <= side; y += side) {
            for (int x = 0; x <= side; x += side) {
                Eigen::Vector3i corner = coords + Eigen::Vector3i(x, y, z);
                Eigen::Vector4f camera_corner =
                    camera * (voxel_size * corner.cast<float>()).homogeneous();

                camera_corner(0) /= camera_corner(2);
                camera_corner(1) /= camera_corner(2);

                if (camera_corner(0) >= 0.f &&
                    camera_corner(0) < frame_size.x() &&
                    camera_corner(1) >= 0.f &&
                    camera_corner(1) < frame_size.y())
                    return true;
            }
        }
    }

    return false;
}

template<typename VoxelBlockType>
SE_DEVICE_FUNC static inline bool in_frustum3(const VoxelBlockType* v,
    float voxel_size, const Eigen::Matrix4f& camera,
    const Eigen::Vector2i& frame_size) {
    constexpr static int side = VoxelBlockType::side;
    Eigen::Vector3i coords    = v->coordinates();
    Eigen::Vector3f center    = voxel_size *
        (coords + Eigen::Vector3i::Constant(side / 2)).template cast<float>();
    Eigen::Vector4f camera_center = camera * center.homogeneous();

    camera_center(0) /= camera_center(2);
    camera_center(1) /= camera_center(2);

    if (camera_center(0) >= 0.f && camera_center(0) < frame_size.x() &&
        camera_center(1) >= 0.f && camera_center(1) < frame_size.y())
        return true;

    return false;
}

template<typename ValueType, typename P>
SE_DEVICE_FUNC bool satisfies(const ValueType& el, P predicate) {
    return predicate(el);
}

template<typename ValueType, typename P, typename... Ps>
SE_DEVICE_FUNC bool satisfies(const ValueType& el, P predicate, Ps... others) {
    return predicate(el) || satisfies(el, others...);
}

#ifdef _OPENMP
template<typename BlockType, template<typename> class BufferT,
    typename... Predicates>
void filter(std::vector<BlockType*>& out, const BufferT<BlockType>& block_array,
    Predicates... ps) {
    std::vector<BlockType*> temp;
    int num_elem = block_array.used();
    temp.resize(num_elem);

    int* thread_start = new int[omp_get_max_threads()];
    int* thread_end   = new int[omp_get_max_threads()];
    int spawn_threads = -1;
#pragma omp parallel
    {
        int threadid    = omp_get_thread_num();
        int num_threads = omp_get_num_threads();
        int my_start    = thread_start[threadid] =
            (threadid) *num_elem / num_threads;
        int my_end = (threadid + 1) * num_elem / num_threads;
        int count  = 0;
        for (int i = my_start; i < my_end; ++i) {
            if (satisfies(block_array[i], ps...)) {
                temp[my_start + count] = block_array[i];
                count++;
            }
        }
        /* Store the actual end */
        thread_end[threadid] = count;
        if (threadid == 0) spawn_threads = num_threads;
    }

    int total = 0;
    for (int i = 0; i < spawn_threads; ++i) { total += thread_end[i]; }
    out.resize(total);
    /* Copy the first */
    std::memcpy(out.data(), temp.data(), sizeof(BlockType*) * thread_end[0]);
    int copied = thread_end[0];
    /* Copy the rest */
    for (int i = 1; i < spawn_threads; ++i) {
        std::memcpy(out.data() + copied, temp.data() + thread_start[i],
            sizeof(BlockType*) * thread_end[i]);
        copied += thread_end[i];
    }
}

#else
template<typename BlockType, template<typename> class BufferT,
    typename... Predicates>
void filter(std::vector<BlockType*>& out, const BufferT<BlockType>& block_array,
    Predicates... ps) {
    for (unsigned int i = 0; i < block_array.used(); ++i) {
        if (satisfies(block_array[i], ps...)) { out.push_back(block_array[i]); }
    }
}
#endif
} // namespace algorithms
} // namespace se
#endif
