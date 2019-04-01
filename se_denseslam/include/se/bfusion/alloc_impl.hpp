/*
 *
 * Copyright 2016 Emanuele Vespa, Imperial College London
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * */
#ifndef BFUSION_ALLOC_H
#define BFUSION_ALLOC_H
#include <se/utils/math_utils.h>

/* Compute step size based on distance travelled along the ray */
static inline float compute_stepsize(const float dist_travelled,
                                     const float band,
                                     const float voxel_size) {
  float new_stepsize;
  float half = band * 0.5f;
  if (dist_travelled < band) {
    new_stepsize = voxel_size;
  } else if (dist_travelled < band + half) {
    new_stepsize = 10.f * voxel_size;
  } else {
    new_stepsize = 30.f * voxel_size;
  }
  return new_stepsize;
}

/* Compute octree level given a step size */
static inline int step_to_depth(const float step,
                                const int max_depth,
                                const float voxel_size) {
  return static_cast<int>(floorf(std::log2f(voxel_size/step)) + max_depth);
}

template <typename FieldType,
          template <typename> class OctreeT,
          typename HashType,
          typename StepF, typename DepthF>
size_t buildOctantList(HashType*              allocation_list,
                       size_t                 reserved,
                       OctreeT<FieldType>&    map_index,
                       const Eigen::Matrix4f& T_wc,
                       const Eigen::Matrix4f& K,
                       const float*           depth_map,
                       const Eigen::Vector2i& image_size,
                       const float            voxel_size,
                       StepF                  compute_stepsize,
                       DepthF                 step_to_depth,
                       const float            noise_factor) {

  const float inverse_voxel_size = 1.f/voxel_size;
  Eigen::Matrix4f inv_K = K.inverse();
  const Eigen::Matrix4f inv_P = T_wc * inv_K;
  const int size = map_index.size();
  const int max_depth = log2(size);
  const int leaves_depth = max_depth - se::math::log2_const(OctreeT<FieldType>::blockSide);

#ifdef _OPENMP
  std::atomic<unsigned int> voxel_count;
  std::atomic<unsigned int> leaves_count;
#else
  unsigned int voxel_count;
#endif

  const Eigen::Vector3f camera_pos = T_wc.topRightCorner<3, 1>();
  voxel_count = 0;
#pragma omp parallel for
  for (int y = 0; y < image_size.y(); ++y) {
    for (int x = 0; x < image_size.x(); ++x) {
      if(depth_map[x + y*image_size.x()] == 0) {
        continue;
      }
      int tree_depth = max_depth;
      float stepsize = voxel_size;
      const float depth = depth_map[x + y*image_size.x()];
      Eigen::Vector3f world_vertex = (inv_P * Eigen::Vector3f((x + 0.5f) * depth,
            (y + 0.5f) * depth, depth).homogeneous()).head<3>();

      Eigen::Vector3f direction = (camera_pos - world_vertex).normalized();
      const float sigma = se::math::clamp(noise_factor * se::math::sq(depth), 2 * voxel_size, 0.05f);
      const float band = 2 * sigma;
      const Eigen::Vector3f origin = world_vertex - (band * 0.5f) * direction;
      const float dist = (camera_pos - origin).norm();
      Eigen::Vector3f step = direction*stepsize;

      Eigen::Vector3f voxel_pos = origin;
      float travelled = 0.f;
      for (; travelled < dist; travelled += stepsize) {

        Eigen::Vector3f voxel_scaled = (voxel_pos * inverse_voxel_size).array().floor();
        if ((voxel_scaled.x() < size)
            && (voxel_scaled.y() < size)
            && (voxel_scaled.z() < size)
            && (voxel_scaled.x() >= 0)
            && (voxel_scaled.y() >= 0)
            && (voxel_scaled.z() >= 0)) {
          const Eigen::Vector3i voxel = voxel_scaled.cast<int>();
          auto node_ptr = map_index.fetch_octant(voxel.x(), voxel.y(), voxel.z(),
              tree_depth);
          if (!node_ptr) {
            HashType k = map_index.hash(voxel.x(), voxel.y(), voxel.z(),
                std::min(tree_depth, leaves_depth));
            unsigned int idx = voxel_count++;
            if(idx < reserved) {
              allocation_list[idx] = k;
            }
          } else if (tree_depth >= leaves_depth) {
            static_cast<se::VoxelBlock<FieldType>*>(node_ptr)->active(true);
          }
        }

        stepsize = compute_stepsize(travelled, band, voxel_size);
        tree_depth = step_to_depth(stepsize, max_depth, voxel_size);

        step = direction*stepsize;
        voxel_pos +=step;
      }
    }
  }
  return (size_t) voxel_count >= reserved ? reserved : (size_t) voxel_count;
}
#endif
