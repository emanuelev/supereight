/*
 *
 * Copyright 2016 Emanuele Vespa, Imperial College London
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
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
 *
 * */

#pragma once

#include <supereight/backend/fields.hpp>
#include <supereight/shared/commons.h>
#include <supereight/utils/math_utils.h>

namespace se {

/* Compute step size based on distance travelled along the ray */
SE_DEVICE_FUNC
static inline float compute_step_size(
    const float dist_travelled, const float band, const float voxel_size) {
    float new_step_size;
    float half = band * 0.5f;
    if (dist_travelled < band) {
        new_step_size = voxel_size;
    } else if (dist_travelled < band + half) {
        new_step_size = 10.f * voxel_size;
    } else {
        new_step_size = 30.f * voxel_size;
    }
    return new_step_size;
}

/* Compute octree level given a step size */
SE_DEVICE_FUNC
static inline int step_to_depth(
    const float step, const int max_depth, const float voxel_size) {
    return static_cast<int>(floorf(std::log2f(voxel_size / step)) + max_depth);
}

template<typename OctreeT, typename HashType, typename IncF>
inline void voxel_traits<OFusion>::buildAllocationList(
    HashType* allocation_list, int reserved, IncF get_idx,
    const OctreeT& octree, const Eigen::Vector3f& world_vertex,
    const Eigen::Vector3f& direction, const Eigen::Vector3f& camera_pos,
    float depth_sample, int max_depth, int block_depth, float voxel_size,
    float inverse_voxel_size, float noise_factor) {
    int cur_depth   = max_depth;
    float step_size = voxel_size;

    const float sigma = se::math::clamp(
        noise_factor * se::math::sq(depth_sample), 2 * voxel_size, 0.05f);
    const float band             = 2 * sigma;
    const Eigen::Vector3f origin = world_vertex - (band * 0.5f) * direction;
    const float dist             = (camera_pos - origin).norm();
    Eigen::Vector3f step         = direction * step_size;

    Eigen::Vector3f voxel_pos = origin;
    float travelled           = 0.f;
    for (; travelled < dist; travelled += step_size) {
        Eigen::Vector3f voxel_scaled =
            (voxel_pos * inverse_voxel_size).array().floor();
        if (octree.inBounds(voxel_scaled)) {
            auto voxel = voxel_scaled.cast<int>();
            auto node_ptr =
                octree.fetch_octant(voxel.x(), voxel.y(), voxel.z(), cur_depth);

            if (!node_ptr) {
                HashType k = octree.hash(voxel.x(), voxel.y(), voxel.z(),
                    std::min(cur_depth, block_depth));
                int idx    = get_idx();

                if (idx < reserved) { allocation_list[idx] = k; }
            } else if (cur_depth >= block_depth) {
                static_cast<typename OctreeT::block_type*>(node_ptr)->active(
                    true);
            }
        }

        step_size = compute_step_size(travelled, band, voxel_size);
        cur_depth = step_to_depth(step_size, max_depth, voxel_size);

        step = direction * step_size;
        voxel_pos += step;
    }
}

} // namespace se
