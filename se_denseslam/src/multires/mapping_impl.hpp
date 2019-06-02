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
#include <se/octree.hpp>
#include <se/image/image.hpp>
#include <se/volume_traits.hpp>
#include <se/algorithms/filter.hpp>
#include <se/functors/for_each.hpp>

namespace se {
  namespace multires {
    /**
     * \brief Computes the scale corresponding to the back-projected pixel size 
     * in voxel space
     * \param[in] vox centroid coordinates of the test voxel
     * \param[in] twc translational component of the camera position 
     * \param[in] scale_pix unitary pixel side after application of inverse 
     * calibration matrix
     * \param[in] voxelsize size of a voxel side in meters 
     * \param[out] scale scale from which propagate up voxel values
     */

    inline float compute_scale(const Eigen::Vector3f& vox, 
        const Eigen::Vector3f& twc,
        const Eigen::Matrix3f& Rcw,
        const float scaled_pix,
        const float voxelsize,
        const int max_scale) {
      const float dist = (Rcw*(voxelsize*vox - twc)).z();
      const float pix_size = dist * scaled_pix;
      int scale = std::min(std::max(0, int(log2(pix_size/voxelsize + 0.5f))), 
                           max_scale);
      return scale;
    }

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
  for(int curr_scale = scale; curr_scale < se::math::log2_const(side); ++curr_scale) {
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
                if (tmp.y != 0) {
                  mean += tmp.x;
                  weight += tmp.y;
                  num_samples++;
                }
              }
          auto data = block->data(curr, curr_scale + 1);

          if(num_samples != 0) {
            mean /= num_samples;
            weight /= num_samples;
            data.x = mean;
            data.y = ceil(weight);
          } else {
            data = voxel_traits<T>::initValue();
          }

          data.delta   = 0;
          data.delta_y = 0;
          block->data(curr, curr_scale + 1, data);
        }
  }
}

template <typename FieldSelector>
float interp(const se::Octree<MultiresSDF>& octree,
             const se::VoxelBlock<MultiresSDF>* block,
             const Eigen::Vector3i& vox,
             const int scale,
             FieldSelector select) {

  // Compute base point in parent block
  const int side   = se::VoxelBlock<MultiresSDF>::side >> (scale + 1); 
  const int stride = 1 << (scale + 1);

  const Eigen::Vector3f& offset = se::Octree<MultiresSDF>::_offset;
  Eigen::Vector3i base = stride * (vox.cast<float>()/stride - offset).cast<int>().cwiseMax(Eigen::Vector3i::Constant(0));
  base = (base.array() == side - 1).select(base - Eigen::Vector3i::Constant(1), base);

  float points[8];
  internal::gather_points(octree, block->coordinates() + base, scale + 1, 
      select, points);

  const Eigen::Vector3f vox_f  = vox.cast<float>() + offset * (stride/2);
  const Eigen::Vector3f base_f = base.cast<float>() + offset*(stride);
  const Eigen::Vector3f factor = (vox_f - base_f) / stride;

  const float v_000 = points[0] * (1 - factor.x()) + points[1] * factor.x();
  const float v_001 = points[2] * (1 - factor.x()) + points[3] * factor.x();
  const float v_010 = points[4] * (1 - factor.x()) + points[5] * factor.x();
  const float v_011 = points[6] * (1 - factor.x()) + points[7] * factor.x();

  const float v_0 = v_000 * (1 - factor.y()) + v_001 * (factor.y());
  const float v_1 = v_010 * (1 - factor.y()) + v_011 * (factor.y());

  const float val = v_0 * (1 - factor.z()) + v_1 * factor.z(); 
  return val;
}

/**
 * Update the subgrids of a voxel block starting from a given scale 
 * down to the finest grid.
 *
 * \param[in] block VoxelBlock to be updated 
 * \param[in] scale scale from which propagate down voxel values
*/
template <typename T>
void propagate_down(const se::Octree<T>& map, 
                    se::VoxelBlock<T>* block, 
                    const int scale,
                    const int min_scale) {
  const Eigen::Vector3i base = block->coordinates();
  const int side = se::VoxelBlock<T>::side;
  for(int curr_scale = scale; curr_scale > min_scale; --curr_scale) {
    const int stride = 1 << curr_scale;
    for(int z = 0; z < side; z += stride)
      for(int y = 0; y < side; y += stride)
        for(int x = 0; x < side; x += stride) {
          const Eigen::Vector3i parent = base + Eigen::Vector3i(x, y, z);
          auto data = block->data(parent, curr_scale);
          const int half_step = stride / 2;
          for(int k = 0; k < stride; k += half_step) {
            for(int j = 0; j < stride; j += half_step) {
              for(int i = 0; i < stride; i += half_step) {
                const Eigen::Vector3i vox = parent + Eigen::Vector3i(i, j , k);
                auto curr = block->data(vox, curr_scale - 1);
                if(curr.y == 0) {
                  curr.x = se::math::clamp(interp(map, block, vox - base, curr_scale - 1, 
                      [](const auto& val) { return val.x; }), -1.f, 1.f);
                  curr.y = data.y;
                  curr.delta   = 0;
                  curr.delta_y = 0;
                } else {
                  curr.x  +=  data.delta;
                  curr.y  +=  data.delta_y;
                  curr.delta = data.delta;
                  curr.delta_y = data.delta_y;
                }
                block->data(vox, curr_scale - 1, curr);
              }
            }
          }
          data.delta   = 0;
          data.delta_y = 0;
          block->data(parent, curr_scale, data);
        }
  }
}


struct multires_block_update {
  multires_block_update(
                  const se::Octree<MultiresSDF>& octree,
                  const Sophus::SE3f& T,
                  const Eigen::Matrix4f& calib,
                  const float vsize,
                  const Eigen::Vector3f& off,
                  const se::Image<float>& d, 
                  const float m,
                  const int mw) : 
                  map(octree),
                  Tcw(T), 
                  K(calib), 
                  voxel_size(vsize),
                  offset(off),
                  depth(d),
                  mu(m),
                  maxweight(mw) {}

  const se::Octree<MultiresSDF>& map;
  const Sophus::SE3f& Tcw;
  const Eigen::Matrix4f& K;
  float voxel_size;
  const Eigen::Vector3f& offset;
  const se::Image<float>& depth; 
  float mu; 
  int maxweight;

  void operator()(se::VoxelBlock<MultiresSDF>* block) {
    const float scaled_pix = (K.inverse() * 
        (Eigen::Vector3f(1, 0 ,1) - Eigen::Vector3f(0, 0, 1)).homogeneous()).x();
    constexpr int side = se::VoxelBlock<MultiresSDF>::side;
    const Eigen::Vector3i base = block->coordinates();
    const int last_scale = block->current_scale();
    int scale = compute_scale((base + Eigen::Vector3i::Constant(side/2)).cast<float>(),
        Tcw.inverse().translation(), Tcw.rotationMatrix(), scaled_pix, voxel_size, se::math::log2_const(side));
    scale = std::max(last_scale - 1, scale);
    if(last_scale > scale) propagate_down(map, block, last_scale, scale);
    block->current_scale(scale);
    const int stride = 1 << scale;
    bool visible = false;

    const Eigen::Vector3f delta = Tcw.rotationMatrix() * Eigen::Vector3f(voxel_size, 0, 0);
    const Eigen::Vector3f cameraDelta = K.topLeftCorner<3,3>() * delta;
    for(int z = 0; z < side; z += stride)
      for(int y = 0; y < side; y += stride) {
        Eigen::Vector3i pix = base + Eigen::Vector3i(0, y, z);
        Eigen::Vector3f start = Tcw * (voxel_size * (pix.cast<float>() + stride*offset));
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
          visible = true;
          const Eigen::Vector2i px = pixel.cast<int>();
          const float depthSample = depth[px.x() + depth.width()*px.y()];
          // continue on invalid depth measurement
          if (depthSample <=  0) continue;

          // Update the TSDF
          const float diff = (depthSample - pos.z())
            * std::sqrt( 1 + se::math::sq(pos.x() / pos.z()) 
                + se::math::sq(pos.y() / pos.z()));
          if (diff > -mu) {
            const float sdf = fminf(1.f, diff/mu);
            auto data = block->data(pix, scale);
            auto tmp = data.x;
            data.x = se::math::clamp(
                (static_cast<float>(data.y) * data.x + sdf) / (static_cast<float>(data.y) + 1.f),
                -1.f,
                 1.f);
            data.delta = (data.x - tmp)/(data.y + 1);
            data.y = fminf(data.y + 1, maxweight);
            data.delta_y++;
            block->data(pix, scale, data);
          }
        }
      }
    propagate_up(block, scale);
    block->active(visible);
  }
};

template <typename T>
void propagate(se::VoxelBlock<T>* block) {
  propagate_up(block, block->current_scale());
  // propagate_down(block, block->current_scale());
}

template <typename T>
void integrate(se::Octree<T>& , const Sophus::SE3f& , const
    Eigen::Matrix4f& , float , const Eigen::Vector3f& , const
    se::Image<float>& , float , int ) {
}

template <>void integrate(se::Octree<MultiresSDF>& map, const Sophus::SE3f& Tcw, const
    Eigen::Matrix4f& K, float voxelsize, const Eigen::Vector3f& offset, const
    se::Image<float>& depth, float mu, int maxweight) {
      // Filter visible blocks
      using namespace std::placeholders;
      std::vector<se::VoxelBlock<MultiresSDF>*> active_list;  
      auto& block_array = map.getBlockBuffer();
      auto is_active_predicate = [](const se::VoxelBlock<MultiresSDF>* b) {
        return b->active();
      };
      const Eigen::Vector2i framesize(depth.width(), depth.height());
      const Eigen::Matrix4f Pcw = K*Tcw.matrix();
      auto in_frustum_predicate = 
        std::bind(se::algorithms::in_frustum<se::VoxelBlock<MultiresSDF>>, _1, 
            voxelsize, Pcw, framesize); 
      se::algorithms::filter(active_list, block_array, is_active_predicate, 
          in_frustum_predicate);
      struct multires_block_update funct(map, Tcw, K, voxelsize,
          offset, depth, mu, maxweight);
      se::functor::internal::parallel_for_each(active_list, funct);
 }
}
}
#endif
