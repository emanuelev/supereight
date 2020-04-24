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
 * */
#pragma once

#include <supereight/node.hpp>
#include <supereight/utils/math_utils.h>
#include <supereight/utils/morton_utils.hpp>

namespace se {

template<typename OctreeT, typename HashType>
static void buildAllocationList_SDF(HashType* allocation_list, int reserved,
    std::atomic<int>& voxel_count, const OctreeT& octree,
    const Eigen::Vector3f& world_vertex, const Eigen::Vector3f& direction,
    const Eigen::Vector3f& camera_pos, float depth_sample, int max_depth,
    int block_depth, float voxel_size, float inverse_voxel_size, float mu) {
    const float band             = mu * 2;
    const Eigen::Vector3f origin = world_vertex - (band * 0.5f) * direction;
    const int num_steps          = ceil(band * inverse_voxel_size);
    const Eigen::Vector3f step   = (direction * band) / num_steps;

    Eigen::Vector3f voxel_pos = origin;
    for (int i = 0; i < num_steps; i++) {
        Eigen::Vector3f voxel_scaled =
            (voxel_pos * inverse_voxel_size).array().floor();
        if (octree.inBounds(voxel_scaled)) {
            auto voxel     = voxel_scaled.cast<int>();
            auto block_ptr = octree.fetch(voxel.x(), voxel.y(), voxel.z());

            if (!block_ptr) {
                HashType k =
                    octree.hash(voxel.x(), voxel.y(), voxel.z(), block_depth);
                int idx = voxel_count++;

                if (idx < reserved) {
                    allocation_list[idx] = k;
                } else {
                    break;
                }
            } else {
                block_ptr->active(true);
            }
        }

        voxel_pos += step;
    }
}

template<typename OctreeT, typename HashType>
int voxel_traits<SDF>::buildAllocationList(HashType* allocation_list,
    int reserved, const OctreeT& octree, const Eigen::Matrix4f& pose,
    const Eigen::Matrix4f& K, const float* depth_map,
    const Eigen::Vector2i& image_size, float mu) {
    const Eigen::Matrix4f inv_P = pose * K.inverse();

    const int max_depth = log2(octree.size());
    const int block_depth =
        log2(octree.size()) - se::math::log2_const(OctreeT::blockSide);

    const float voxel_size         = octree.dim() / octree.size();
    const float inverse_voxel_size = 1.f / voxel_size;

    std::atomic<int> voxel_count;

    const Eigen::Vector3f camera_pos = pose.topRightCorner<3, 1>();
    voxel_count                      = 0;
#pragma omp parallel for
    for (int y = 0; y < image_size.y(); ++y) {
        for (int x = 0; x < image_size.x(); ++x) {
            const float depth_sample = depth_map[x + y * image_size.x()];
            if (depth_sample == 0) continue;

            Eigen::Vector3f world_vertex = (inv_P *
                Eigen::Vector3f((x + 0.5f) * depth_sample,
                    (y + 0.5f) * depth_sample, depth_sample)
                    .homogeneous())
                                               .head<3>();
            Eigen::Vector3f direction =
                (camera_pos - world_vertex).normalized();

            buildAllocationList_SDF(allocation_list, reserved, voxel_count,
                octree, world_vertex, direction, camera_pos, depth_sample,
                max_depth, block_depth, voxel_size, inverse_voxel_size, mu);
        }
    }

    int final_count = voxel_count;
    return final_count >= reserved ? reserved : final_count;
}

} // namespace se
