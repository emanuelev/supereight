/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of
 Manchester. Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.


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

#pragma once

#include "../common/field_impls.hpp"

#include <supereight/shared/commons.h>
#include <supereight/shared/perfstats.h>
#include <supereight/shared/timings.h>
#include <supereight/utils/math_utils.h>

#include <supereight/image/image.hpp>
#include <supereight/ray_iterator.hpp>

#include <supereight/backend/memory_pool_cuda.hpp>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace se {

template<typename OctreeT, typename HashType>
static void buildAllocationListKernel(HashType* allocation_list, int reserved,
    int& allocation_count, const OctreeT& octree, const Eigen::Matrix4f& pose,
    const Eigen::Matrix4f& K, const float* depth_map,
    const Eigen::Vector2i& image_size, float mu) {
    const Eigen::Matrix4f inv_P = pose * K.inverse();

    const int max_depth = log2(octree.size());
    const int block_depth =
        log2(octree.size()) - se::math::log2_const(OctreeT::blockSide);

    const float voxel_size         = octree.dim() / octree.size();
    const float inverse_voxel_size = 1.f / voxel_size;

    std::atomic<int> voxel_count;

    Eigen::Vector2i computation_size = image_size;

    const Eigen::Vector3f camera_pos = pose.topRightCorner<3, 1>();
    voxel_count                      = 0;
#pragma omp parallel for
    for (int y = 0; y < computation_size.y(); ++y) {
        for (int x = 0; x < computation_size.x(); ++x) {
            const float depth_sample = depth_map[x + y * computation_size.x()];
            if (depth_sample == 0) continue;

            Eigen::Vector3f world_vertex = (inv_P *
                Eigen::Vector3f((x + 0.5f) * depth_sample,
                    (y + 0.5f) * depth_sample, depth_sample)
                    .homogeneous())
                                               .head<3>();
            Eigen::Vector3f direction =
                (camera_pos - world_vertex).normalized();

            voxel_traits<typename OctreeT::value_type>::buildAllocationList(
                allocation_list, reserved, voxel_count, octree, world_vertex,
                direction, camera_pos, depth_sample, max_depth, block_depth,
                voxel_size, inverse_voxel_size, mu);
        }
    }

    int final_count  = voxel_count;
    allocation_count = final_count >= reserved ? reserved : final_count;
}

template<typename FieldType, template<typename> class BufferT,
    template<typename, template<typename> class> class OctreeT>
static void renderVolumeKernel(const OctreeT<FieldType, BufferT>& octree,
    unsigned char* out, // RGBW packed
    const Eigen::Vector2i& depthSize, const Eigen::Matrix4f& view,
    const float nearPlane, const float farPlane, const float mu,
    const float step, const float largestep, const Eigen::Vector3f& light,
    const Eigen::Vector3f& ambient, bool render,
    const se::Image<Eigen::Vector3f>& vertex,
    const se::Image<Eigen::Vector3f>& normal) {
    TICK();
    int y;
#pragma omp parallel for shared(out), private(y)
    for (y = 0; y < depthSize.y(); y++) {
        for (int x = 0; x < depthSize.x(); x++) {
            Eigen::Vector4f hit;
            Eigen::Vector3f test, surfNorm;
            const int idx = (x + depthSize.x() * y) * 4;

            if (render) {
                const Eigen::Vector3f dir =
                    (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f))
                        .normalized();
                const Eigen::Vector3f transl = view.topRightCorner<3, 1>();
                se::ray_iterator<FieldType, BufferT> ray(
                    octree, transl, dir, nearPlane, farPlane);
                ray.next();
                const float t_min = ray.tmin(); /* Get distance to the first
                                                   intersected block */
                hit = t_min > 0.f
                    ? voxel_traits<FieldType>::raycast(octree, transl, dir,
                          t_min, ray.tmax(), mu, step, largestep)
                    : Eigen::Vector4f::Constant(0.f);
                if (hit.w() > 0) {
                    test     = hit.head<3>() * octree.size() / octree.dim();
                    surfNorm = octree.grad(
                        test, [](const auto& val) { return val.x; });

                    // Invert normals if SDF
                    if (voxel_traits<FieldType>::invert_normals)
                        surfNorm *= -1.f;
                } else {
                    surfNorm = Eigen::Vector3f(INVALID, 0, 0);
                }
            } else {
                test     = vertex[x + depthSize.x() * y];
                surfNorm = normal[x + depthSize.x() * y];
            }

            if (surfNorm.x() != INVALID && surfNorm.norm() > 0) {
                const Eigen::Vector3f diff = (test - light).normalized();
                const Eigen::Vector3f dir  = Eigen::Vector3f::Constant(
                    fmaxf(surfNorm.normalized().dot(diff), 0.f));
                Eigen::Vector3f col = dir + ambient;
                se::math::clamp(col, Eigen::Vector3f::Constant(0.f),
                    Eigen::Vector3f::Constant(1.f));
                col *= 255.f;
                out[idx + 0] = col.x();
                out[idx + 1] = col.y();
                out[idx + 2] = col.z();
                out[idx + 3] = 0;
            } else {
                out[idx + 0] = 255;
                out[idx + 1] = 0;
                out[idx + 2] = 0;
                out[idx + 3] = 0;
            }
        }
    }
    TOCK("renderVolumeKernel", depthSize.x * depthSize.y);
}

} // namespace se
