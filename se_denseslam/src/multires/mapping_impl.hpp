/*
 *
 * Copyright 2019 Emanuele Vespa, Imperial College London
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
 * */
#ifndef MULTIRES_MAPPING_HPP
#define MULTIRES_MAPPING_HPP
#include <se/node.hpp>
#include <se/image/image.hpp>

namespace se {
  namespace multires {

/**
 * Update the subgrids of a voxel block starting from a given scale up 
 * to a maximum scale.
 *
 * \param[in] block VoxelBlock to be updated 
 * \param[in] scale scale from which propagate up voxel values
*/
template <typename T>
void propagate_up(se::VoxelBlock<T>* block, const int scale) {
  const Eigen::Vector3i base = block->coordinates();
  const int side = se::VoxelBlock<T>::side;
  for(int curr_scale = scale; curr_scale < se::math::log2_const(side) - 1; ++curr_scale) {
    const int stride = 1 << (curr_scale + 1);
    for(int z = 0; z < side; z += stride)
      for(int y = 0; y < side; y += stride)
        for(int x = 0; x < side; x += stride) {
          const Eigen::Vector3i curr = base + Eigen::Vector3i(x, y, z);

          float mean = 0;
          int num_samples = 0;
          float weight = 0;
          for(int k = 0; k < stride; k += stride/2)
            for(int j = 0; j < stride; j += stride/2 )
              for(int i = 0; i < stride; i += stride/2) {
                auto tmp = block->data(curr + Eigen::Vector3i(i, j , k), curr_scale);
                mean += tmp.x;
                weight += tmp.y;
                num_samples++;
              }
          mean /= num_samples;
          weight /= num_samples;
          auto data = block->data(curr, curr_scale + 1);
          data.x = mean;
          data.y = weight;
          data.delta   = 0;
          data.delta_y = 0;
          block->data(curr, curr_scale + 1, data);
        }
  }
}

/**
 * Update the subgrids of a voxel block starting from a given scale 
 * down to the finest grid.
 *
 * \param[in] block VoxelBlock to be updated 
 * \param[in] scale scale from which propagate down voxel values
*/
template <typename T>
void propagate_down(se::VoxelBlock<T>* block, const int scale) {
  const Eigen::Vector3i base = block->coordinates();
  const int side = se::VoxelBlock<T>::side;
  for(int curr_scale = scale; curr_scale > 0; --curr_scale) {
    const int stride = 1 << curr_scale;
    for(int z = 0; z < side; z += stride)
      for(int y = 0; y < side; y += stride)
        for(int x = 0; x < side; x += stride) {
          const Eigen::Vector3i parent = base + Eigen::Vector3i(x, y, z);
          auto data = block->data(parent, curr_scale);
          const int half_step = stride / 2;
          for(int k = 0; k < stride; k += half_step)
            for(int j = 0; j < stride; j += half_step)
              for(int i = 0; i < stride; i += half_step) {
                const Eigen::Vector3i vox = parent + Eigen::Vector3i(i, j , k);
                auto curr = block->data(vox, curr_scale - 1);
                auto new_x = curr.x + data.delta;
                auto new_y = curr.y + data.delta_y;
                curr.delta = new_x - curr.x;
                curr.delta_y = new_y - curr.y;
                curr.x  +=  data.delta;
                curr.y  +=  data.delta_y;
                block->data(vox, curr_scale - 1, curr);
              }
          data.delta_y = 0; 
          block->data(parent, curr_scale, data);
        }
  }
}

/**
 * Compute a TSDF update for a given block at a given scale.
 *
 * \param[in] block VoxelBlock to be updated 
 * \param[in] Tcw World-to-camera SE3 transform 
 * \param[in] K camera intrinsics matrix 
 * \param[in] voxel_size voxel resolution per side
 * \param[in] offset three dimensional offset of the sampling point compared to 
 * the voxel position. E.g. if voxels are cell centered, offset = (0.5f, 0.5f, 0.5f) 
 * \param[in] depth input depthmap.
 * \param[in] mu truncation bandwidth
 * \param[in] maxweight maxium weight for a voxel
 * \param[in] scale scale at which the fusion should happen, 
 * relative to scale 0 which indicates the finest subgrid. E.g. if blocks are 
 * made of 8^3 voxel, a scale of 1 means that the fusion will occur at the 4^3
 * grid.
 */
template <typename T>
void update_block (se::VoxelBlock<T>* block, 
                   const Sophus::SE3f& Tcw,
                   const Eigen::Matrix4f& K,
                   const float voxel_size,
                   const Eigen::Vector3f& offset,
                   const se::Image<float>& depth, 
                   const float mu,
                   const int maxweight,
                   const int scale) {
  const int side = se::VoxelBlock<T>::side;
  const int stride = 1 << scale;
  const Eigen::Vector3i base = block->coordinates();
  const Eigen::Vector3f delta = Tcw.rotationMatrix() * Eigen::Vector3f(stride * voxel_size, 0, 0);
  const Eigen::Vector3f cameraDelta = K.topLeftCorner<3,3>() * delta;
  for(int z = 0; z < side; z += stride)
    for(int y = 0; y < side; y += stride) {
      Eigen::Vector3i pix = base + Eigen::Vector3i(0, y, z);
      Eigen::Vector3f start = Tcw * (voxel_size * (pix.cast<float>() + offset));
      Eigen::Vector3f camerastart = K.topLeftCorner<3,3>() * start;
      for(int x = 0; x < side; x += stride, pix.x() += stride) {
        const Eigen::Vector3f camera_voxel = camerastart + (x*cameraDelta);
        const Eigen::Vector3f pos = start + (x*delta);
        if (pos.z() < 0.0001f) continue;

        const float inverse_depth = 1.f / camera_voxel.z();
        const Eigen::Vector2f pixel = Eigen::Vector2f(
            camera_voxel.x() * inverse_depth + 0.5f,
            camera_voxel.y() * inverse_depth + 0.5f);
        if (pixel.x() < 0.5f || pixel.x() > depth.width() - 1.5f || 
            pixel.y() < 0.5f || pixel.y() > depth.height() - 1.5f) continue;
        block->active(true);

        const Eigen::Vector2i px = pixel.cast<int>();
        const float depthSample = depth[px.x() + depth.width()*px.y()];
        // continue on invalid depth measurement
        if (depthSample <=  0) continue;

        // Update the TSDF
        const float diff = (depthSample - pos.z())
          * std::sqrt( 1 + se::math::sq(pos.x() / pos.z()) 
          + se::math::sq(pos.y() / pos.z()));
        if (diff > -mu) {
          const float sdf = fminf(1.f, diff / mu);
          auto data = block->data(pix, scale);
          data.x = se::math::clamp(
              (static_cast<float>(data.y) * data.x + sdf) / (static_cast<float>(data.y) + 1.f),
              -1.f,
              1.f);
          data.y = fminf(data.y + 1, maxweight);
          block->data(pix, scale, data);
        }
      }
    }
}
}
}
#endif

