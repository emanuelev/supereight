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
 * */

#ifndef MULTIRES_BFUSION_MAPPING_HPP
#define MULTIRES_BFUSION_MAPPING_HPP
#include <cstdlib>
#include <Eigen/StdVector>
#include <atomic>
#include <omp.h>
#include <map>

#include <se/node.hpp>
#include <se/functors/projective_functor.hpp>
#include <se/constant_parameters.h>
#include <se/image/image.hpp>
#include "../bfusion/bspline_lookup.cc"
#include <se/octree.hpp>
#include <se/volume_traits.hpp>
#include <se/algorithms/filter.hpp>
#include <se/functors/for_each.hpp>

namespace se {
  namespace multires {
    namespace ofusion {

      /**
       * Perform bilinear interpolation on a depth image. See
       * https://en.wikipedia.org/wiki/Bilinear_interpolation for more details.
       *
       * \param[in] depth The depth image to interpolate.
       * \param[in] proj The coordinates on the image at which the interpolation
       * should be computed.
       * \return The value of the interpolated depth at proj.
       */
      float interpDepth(const se::Image<float>& depth, const Eigen::Vector2f& proj) {
        // https://en.wikipedia.org/wiki/Bilinear_interpolation

        // Pixels version
        const float x1 = (floorf(proj.x()));
        const float y1 = (floorf(proj.y() + 1));
        const float x2 = (floorf(proj.x() + 1));
        const float y2 = (floorf(proj.y()));

        const float d11 = depth(int(x1), int(y1));
        const float d12 = depth(int(x1), int(y2));
        const float d21 = depth(int(x2), int(y1));
        const float d22 = depth(int(x2), int(y2));

        if ( d11 == 0.f || d12 == 0.f || d21 == 0.f || d22 == 0.f )
          return 0.f;

        const float f11 = 1.f / d11;
        const float f12 = 1.f / d12;
        const float f21 = 1.f / d21;
        const float f22 = 1.f / d22;

        // Filtering version
        const float d =  1.f /
                         ( (   f11 * (x2 - proj.x()) * (y2 - proj.y())
                               + f21 * (proj.x() - x1) * (y2 - proj.y())
                               + f12 * (x2 - proj.x()) * (proj.y() - y1)
                               + f22 * (proj.x() - x1) * (proj.y() - y1)
                           ) / ((x2 - x1) * (y2 - y1))
                         );

        static const float interp_thresh = 0.05f;
        if (fabs(d - d11) < interp_thresh
            && fabs(d - d12) < interp_thresh
            && fabs(d - d21) < interp_thresh
            && fabs(d - d22) < interp_thresh) {
          return d;
        } else {
          return depth(int(proj.x() + 0.5f), int(proj.y() + 0.5f));
        }
      }

      /**
       * Compute the value of the q_cdf spline using a lookup table. This implements
       * equation (7) from \cite VespaRAL18.
       *
       * \param[in] t Where to compute the value of the spline at.
       * \return The value of the spline.
       */
      static inline float bspline_memoized(float t) {
        float value = 0.f;
        constexpr float inverseRange = 1/6.f;
        if (t >= -3.0f && t <= 3.0f) {
          unsigned int idx = ((t + 3.f)*inverseRange)*(bspline_num_samples - 1) + 0.5f;
          return bspline_lookup[idx];
        } else if(t > 3) {
          value = 1.f;
        }
        return value;
      }

      /**
       * Compute the occupancy probability along the ray from the camera. This
       * implements equation (6) from \cite VespaRAL18.
       *
       * \param[in] val The point on the ray at which the occupancy probability is
       * computed. The point is expressed using the ray parametric equation.
       * \param[in]
       * \return The occupancy probability.
       */
      static inline float H(const float val, const float) {
        const float Q_1 = bspline_memoized(val);
        const float Q_2 = bspline_memoized(val - 3);
        return Q_1 - Q_2 * 0.5f;
      }

      /**
       * Perform a log-odds update of the occupancy probability. This implements
       * equations (8) and (9) from \cite VespaRAL18.
       */
      static inline float updateLogs(const float prior, const float sample) {
        return (prior + log2(sample / (1.f - sample)));
      }

      /**
       * Weight the occupancy by the time since the last update, acting as a
       * forgetting factor. This implements equation (10) from \cite VespaRAL18.
       */
      static inline float applyWindow(const float occupancy,
                                      const float,
                                      const float delta_t,
                                      const float tau) {
        float fraction = 1.f / (1.f + (delta_t / tau));
        fraction = std::max(0.5f, fraction);
        return occupancy * fraction;
      }

      /**
       * Up-propagate the mean and maximum occupuancy of a voxel block starting scale 0 to voxel block scale
       *
       * \param[in] block VoxelBlock to be updated
       * \param[in] scale Scale to start the up-progation from
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
                float x_max = BOTTOM_CLAMP;
                voxel_state st_max = voxel_state::kFree;
                for(int k = 0; k < stride; k += stride/2)
                  for(int j = 0; j < stride; j += stride/2 )
                    for(int i = 0; i < stride; i += stride/2) {
                      auto tmp = block->data(curr + Eigen::Vector3i(i, j , k), curr_scale);
                      if (tmp.x > x_max)
                        x_max = tmp.x;
                      if (tmp.st > st_max)
                        st_max = tmp.st;
                    }
                auto data = block->data(curr, curr_scale + 1);
                data.x = x_max;
                data.st = st_max;
                block->data(curr, curr_scale + 1, data);
              }
        }
      }

      /**
       * Up-propagate the maximum occupuancy of a node.
       * Note only the x and x_max are equivalent at node level.
       *
       * \param[in] node      Node to be up-propagated
       * \param[in] max_level Max level of the tree
       * \param[in] timestamp Current frame number
      */
      template <typename T>
      void propagate_up(se::Node<T>* node, const int max_level,
                        const unsigned timestamp) {
        node->timestamp(timestamp);

        if(!node->parent()) {
          return;
        }

        float x_max = BOTTOM_CLAMP;
        voxel_state st_max = voxel_state::kFree;
        for(int i = 0; i < 8; ++i) {
          const auto& tmp = node->value_[i];
          if (tmp.x > x_max)
            x_max = tmp.x;
          if (tmp.st > st_max)
            st_max = tmp.st;
        }

        const unsigned int id = se::child_id(node->code_,
                                             se::keyops::level(node->code_), max_level);
        auto& data = node->parent()->value_[id];
        data.x = x_max;
        data.st = st_max;
      }

      bool isFrontier(const se::Octree<FieldType> &map, se::VoxelBlock<OFusion>* block, const Eigen::Vector3i voxel) {
        VecVec3i face_neighbour_voxel(6);

        face_neighbour_voxel[0] << voxel.x() - 1, voxel.y(), voxel.z();
        face_neighbour_voxel[1] << voxel.x() + 1, voxel.y(), voxel.z();
        face_neighbour_voxel[2] << voxel.x(), voxel.y() - 1, voxel.z();
        face_neighbour_voxel[3] << voxel.x(), voxel.y() + 1, voxel.z();
        face_neighbour_voxel[4] << voxel.x(), voxel.y(), voxel.z() - 1;
        face_neighbour_voxel[5] << voxel.x(), voxel.y(), voxel.z() + 1;
        for (const auto &face_voxel : face_neighbour_voxel) {

          // map boarder check, don't want the drone to fly there.
          if(face_voxel.x() < 0|| face_voxel.y() < 0 || face_voxel.z() <0 ||
             face_voxel.x() >= map.size() || face_voxel.y() >= map.size() || face_voxel.z() >= map.size()){
            return false;
          }

          // check if face voxel is inside same voxel block
          if ((voxel.x() / BLOCK_SIDE) == (face_voxel.x() / BLOCK_SIDE)
              && (voxel.y() / BLOCK_SIDE) == (face_voxel.y() / BLOCK_SIDE)
              && (voxel.z() / BLOCK_SIDE) == (face_voxel.z() / BLOCK_SIDE)) {
            // CASE 1: same voxel block
            // std::cout << "prob " << _block->data(face_voxel).x << " state " << _block->data(face_voxel).st << std::endl; _block->data(face_voxel).x == 0.f &&
            if (block->data(face_voxel).st==voxel_state::kUnknown )
              return true;
          } else {
            // not same voxel block => check if neighbour is a voxel block
            se::Node<FieldType> *node = nullptr;
            bool is_voxel_block;
            map.fetch_octant(face_voxel(0), face_voxel(1), face_voxel(2), node, is_voxel_block);
            // CASE 2: not same voxelblock but is a voxel
            if (is_voxel_block) {
              // neighbour is a voxel block block->data(face_voxel).x == 0.f  &&
              se::VoxelBlock<FieldType> *block = static_cast<se::VoxelBlock<FieldType> *> (node);
              // std::cout << "prob " << block->data(face_voxel).x << " state " << block->data(face_voxel).st << std::endl;
              if (block->data(face_voxel).st==voxel_state::kUnknown )
                return true;

              // CASE 3: not same voxelblock but belongs to a node
              // TODO take value from node or say the curr voxel is unknown?
              // currently just take node state
              // the node can be at a much higher level

            } else {
              // in case the neighbour node is also not in the same parent
              const key_t octant = se::keyops::encode(face_voxel.x(), face_voxel.y(), face_voxel.z(),
                                                      map.leaf_level(), map.max_level());
              const unsigned int id = se::child_id(octant, map.leaf_level(), map.max_level());
              auto& data = node->parent()->value_[id];
              if (data.st==voxel_state::kUnknown){
                return false;
              }
            }
          }
        }
        return false;
      }

      struct multires_block_update {
        multires_block_update(
            const se::Octree<OFusion>& octree,
            const Sophus::SE3f& T,
            const Eigen::Matrix4f& calib,
            const float vsize,
            const Eigen::Vector3f& off,
            const se::Image<float>& d,
            const int f,
            const float m) :
            map(octree),
            Tcw(T),
            K(calib),
            voxel_size(vsize),
            offset(off),
            depth(d),
            frame(f),
            mu(m) {};

        multires_block_update(
            const se::Octree<OFusion>& octree,
            const Sophus::SE3f& T,
            const Eigen::Matrix4f& calib,
            const float vsize,
            const Eigen::Vector3f& off,
            const se::Image<float>& d,
            const int f,
            const float m,
            set3i *updated_blocks,
            set3i *free_blocks
            ) :
            map(octree),
            Tcw(T),
            K(calib),
            voxel_size(vsize),
            offset(off),
            depth(d),
            frame(f),
            updated_blocks_(updated_blocks),
            free_blocks_(free_blocks) {};

        const se::Octree<OFusion>& map;
        const Sophus::SE3f& Tcw;
        const Eigen::Matrix4f& K;
        float voxel_size;
        const Eigen::Vector3f& offset;
        const se::Image<float>& depth;
        const int frame;
        float mu;

        set3i *updated_blocks_ = new set3i;
        set3i *free_blocks_ = new set3i;



        void operator()(se::VoxelBlock<OFusion>* block) {
          constexpr int side = se::VoxelBlock<OFusion>::side;
          const Eigen::Vector3i base = block->coordinates();
          const int scale = 0;
          const int stride = 1 << scale;
          bool visible = false;
          block->current_scale(scale);

          const Eigen::Vector3f delta = Tcw.rotationMatrix() * Eigen::Vector3f(voxel_size, 0, 0);
          const Eigen::Vector3f cameraDelta = K.topLeftCorner<3, 3>() * delta;
          for (int z = 0; z < side; z += stride) {
            for (int y = 0; y < side; y += stride) {
              Eigen::Vector3i pix = base + Eigen::Vector3i(0, y, z);
              Eigen::Vector3f start = Tcw * (voxel_size * (pix.cast<float>() + stride * offset));
              Eigen::Vector3f camerastart = K.topLeftCorner<3, 3>() * start;
              for (int x = 0; x < side; x += stride, pix.x() += stride) {
                const Eigen::Vector3f camera_voxel = camerastart + (x * cameraDelta);
                const Eigen::Vector3f pos = start + (x * delta);
                if (pos.z() < 0.0001f) continue;

                const float inverse_depth = 1.f / camera_voxel.z();
                const Eigen::Vector2f pixel = Eigen::Vector2f(
                    camera_voxel.x() * inverse_depth + 0.5f,
                    camera_voxel.y() * inverse_depth + 0.5f);
                if (pixel.x() < 0.5f || pixel.x() > depth.width() - 1.5f ||
                    pixel.y() < 0.5f || pixel.y() > depth.height() - 1.5f)
                  continue;
                visible = true;
                const Eigen::Vector2i px = pixel.cast<int>();
                const float depthSample = depth[px.x() + depth.width() * px.y()];
                // continue on invalid depth measurement
                if (depthSample <= 0) continue;

                // Compute the occupancy probability for the current measurement.
                const float diff = (pos.z() - depthSample);
                                   // * std::sqrt( 1 + se::math::sq(pos.x() / pos.z()) + se::math::sq(pos.y() / pos.z()));
                float sigma = se::math::clamp(mu * se::math::sq(pos.z()),
                                              0.5f * voxel_size, 0.75f * voxel_size);
                float sample = H(diff/sigma, pos.z());
                if (sample == 0.5f)
                  continue;
                sample = se::math::clamp(sample, 0.03f, 0.97f);

                auto data = block->data(pix, scale);
                float prev_occ = se::math::getProbFromLog(data.x);
                const bool is_voxel = true;
                const key_t morton_code_child = block->code_;
                // Update the occupancy probability

                // NOTE: Time dependency is deactivated
                const double delta_t = (double)(frame - data.y) / 30;
                data.x = applyWindow(data.x, SURF_BOUNDARY, delta_t, CAPITAL_T);
                data.x = se::math::clamp(updateLogs(data.x, sample), BOTTOM_CLAMP, TOP_CLAMP);
                data.y = frame;

                float prob = se::math::getProbFromLog(data.x);
                if (data.x > SURF_BOUNDARY) {
                  data.st = voxel_state::kOccupied;
                } else if (data.x < SURF_BOUNDARY) {
                  data.st = voxel_state::kFree;
#pragma omp critical
                    free_blocks_->insert(morton_code_child);
                }
                bool voxelOccupied = prev_occ <= 0.5 && prob > 0.5;
                bool voxelFreed = prev_occ >= 0.5 && prob < 0.5;
                if (voxelOccupied || voxelFreed) { // Isn't the && data.st != voxel_state::kFrontier redundant as it
                                                   // get's most likely overwritten above (i.e. data.x != SURF_BOUNDARY)
#pragma omp critical
                  {
                    updated_blocks_->insert(morton_code_child);
                  }
                }
                block->data(pix, scale, data);
              }
            }
          }
          propagate_up(block, scale);
          block->active(visible);
        }
      };

      struct frontier_update {
        frontier_update(const se::Octree<OFusion>& octree, set3i *frontier_blocks, const int ceiling_height_v) :
            map(octree),
            frontier_blocks_(frontier_blocks),
            ceiling_height_v_(ceiling_height_v){};
        const se::Octree<OFusion>& map;
        set3i *frontier_blocks_ = new set3i;
        const int ceiling_height_v_;

        void operator()(se::VoxelBlock<OFusion>* block) {
          constexpr int side = se::VoxelBlock<OFusion>::side;
          const key_t morton_code_child = block->code_;

          if (block->data(VoxelBlock<OFusion>::buff_size - 1).st == voxel_state::kOccupied) {
            return;
          }
            else if (block->data(VoxelBlock<OFusion>::buff_size - 1).st == voxel_state::kFree) {
// #pragma omp critical
//             {
              for (int i = 0; i < 2 ; i++) {
                for (int y = 0; y < side; y++) {
                  for (int z = 0; z < side; z++) {
                    Eigen::Vector3i pix = block->coordinates() + Eigen::Vector3i(i*(side-1),y,z);
                    auto data = block->data(pix);
                    if (data.st == voxel_state::kFree && isFrontier(map, block, pix) && pix.z() < ceiling_height_v_) {
#pragma omp critical
                      frontier_blocks_->insert(morton_code_child);
                      data.st = voxel_state::kFrontier;
                      block->data(pix, 0, data);
                    }
                  }
                }
              }
              for (int x = 1; x < side-1 ; x++) {
                for (int j = 0; j < 2 ; j++) {
                  for (int z = 0; z < side; z++) {
                    Eigen::Vector3i pix = block->coordinates() + Eigen::Vector3i(x,j*(side-1),z);
                    auto data = block->data(pix);
                    if (data.st == voxel_state::kFree && isFrontier(map, block, pix)&& pix.z() < ceiling_height_v_) {
#pragma omp critical
                      frontier_blocks_->insert(morton_code_child);
                      data.st = voxel_state::kFrontier;
                      block->data(pix, 0, data);
                    }
                  }
                }
              }
              for (int x = 1; x < side-1 ; x++) {
                for (int y = 1; y < side-1; y++) {
                  for (int k = 0; k < 2 ; k++) {

                    Eigen::Vector3i pix = block->coordinates() + Eigen::Vector3i(x,y,k*(side-1));
                    auto data = block->data(pix);
                    if (data.st == voxel_state::kFree && isFrontier(map, block, pix)&& pix.z() < ceiling_height_v_) {
#pragma omp critical
                      frontier_blocks_->insert(morton_code_child);
                      data.st = voxel_state::kFrontier;
                      block->data(pix, 0, data);
                    }
                  }
                }
              }
            // }
            return;
          }
#pragma omp critical
          {

            for (int x = 0; x < side ; x++) {
              for (int y = 0; y < side; y++) {
                for (int z = 0; z < side; z++) {
                  Eigen::Vector3i pix = block->coordinates() + Eigen::Vector3i(x,y,z);
                  auto data = block->data(pix);
                  if (data.st == voxel_state::kFree && isFrontier(map, block, pix)&& pix.z() < ceiling_height_v_) {
                    frontier_blocks_->insert(morton_code_child);
                    data.st = voxel_state::kFrontier;
                    block->data(pix, 0, data);
                  }
                }
              }
            }
          }
        }
      };

      template <typename T>
      void integrate(se::Octree<T>& , const Sophus::SE3f& , const
      Eigen::Matrix4f& , float , const Eigen::Vector3f& , const
                     se::Image<float>& , float , const unsigned) {
      }

      template <typename T>
      void integrate(se::Octree<T>& , const Sophus::SE3f& , const
      Eigen::Matrix4f& , float , const Eigen::Vector3f& , const
                     se::Image<float>& , float , const unsigned,
                     const int, set3i*, set3i*, set3i*) {
      }

      template <>void integrate(se::Octree<OFusion>& map, const Sophus::SE3f& Tcw, const
      Eigen::Matrix4f& K, float voxelsize, const Eigen::Vector3f& offset, const
                                se::Image<float>& depth, float mu, const unsigned frame) {
        // Filter active blocks / blocks in frustum from the block buffer
        using namespace std::placeholders;
        std::vector<se::VoxelBlock<OFusion> *> active_list;
        auto &block_array = map.getBlockBuffer();
        auto is_active_predicate = [](const se::VoxelBlock<OFusion> *b) {
          return b->active();
        };
        const Eigen::Vector2i framesize(depth.width(), depth.height());
        const Eigen::Matrix4f Pcw = K * Tcw.matrix();
        auto in_frustum_predicate =
            std::bind(se::algorithms::in_frustum<se::VoxelBlock<OFusion>>, _1,
                      voxelsize, Pcw, framesize);
        se::algorithms::filter(active_list, block_array, is_active_predicate,
                               in_frustum_predicate);

        // Update voxel block values
        struct multires_block_update funct(map, Tcw, K, voxelsize,
                                           offset, depth, frame, mu);
        se::functor::internal::parallel_for_each(active_list, funct);

        std::deque<Node<OFusion> *> prop_list;
        std::mutex deque_mutex;

        // Up propagate voxel block max values to first node level
        for (const auto &b : active_list) {
          if (b->parent()) {
            prop_list.push_back(b->parent());
            const unsigned int id = se::child_id(b->code_,
                                                 se::keyops::level(b->code_), map.max_level());
            auto data = b->data(b->coordinates(), se::math::log2_const(se::VoxelBlock<OFusion>::side));
            auto &parent_data = b->parent()->value_[id];
            parent_data = data;
          }
        }

        // Up propagate node max values to root level
        while (!prop_list.empty()) {
          Node<OFusion> *n = prop_list.front();
          prop_list.pop_front();
          if (n->timestamp() == frame) continue;
          propagate_up(n, map.max_level(), frame);
          if (n->parent()) prop_list.push_back(n->parent());
        }
      }

      template <>void integrate(se::Octree<OFusion>& map, const Sophus::SE3f& Tcw, const
      Eigen::Matrix4f& K, float voxelsize, const Eigen::Vector3f& offset, const
                                se::Image<float>& depth, float mu, const unsigned frame,
                                const int ceiling_height_v, set3i* updated_blocks, set3i* free_blocks,
                                set3i* frontier_blocks) {
        // Filter active blocks / blocks in frustum from the block buffer
        using namespace std::placeholders;
        std::vector<se::VoxelBlock<OFusion> *> active_list;
        auto &block_array = map.getBlockBuffer();
        auto is_active_predicate = [](const se::VoxelBlock<OFusion> *b) {
          return b->active();
        };
        const Eigen::Vector2i framesize(depth.width(), depth.height());
        const Eigen::Matrix4f Pcw = K * Tcw.matrix();
        auto in_frustum_predicate =
            std::bind(se::algorithms::in_frustum<se::VoxelBlock<OFusion>>, _1,
                      voxelsize, Pcw, framesize);
        se::algorithms::filter(active_list, block_array, is_active_predicate,
                               in_frustum_predicate);

        // Update voxel block values
        struct multires_block_update functMultires(map, Tcw, K, voxelsize,
                                           offset, depth, frame, mu, updated_blocks, free_blocks);
        se::functor::internal::parallel_for_each(active_list, functMultires);
        struct frontier_update functFrontier(map, frontier_blocks, ceiling_height_v);
        se::functor::internal::parallel_for_each(active_list, functFrontier);

        std::deque<Node<OFusion> *> prop_list;
        std::mutex deque_mutex;

        // Up propagate voxel block max values to first node level
        for (const auto &b : active_list) {
          if (b->parent()) {
            prop_list.push_back(b->parent());
            const unsigned int id = se::child_id(b->code_,
                                                 se::keyops::level(b->code_), map.max_level());
            auto data = b->data(b->coordinates(), se::math::log2_const(se::VoxelBlock<OFusion>::side));
            auto &parent_data = b->parent()->value_[id];
            parent_data = data;
          }
        }

        // Up propagate node max values to root level
        while (!prop_list.empty()) {
          Node<OFusion> *n = prop_list.front();
          prop_list.pop_front();
          if (n->timestamp() == frame) continue;
          propagate_up(n, map.max_level(), frame);
          if (n->parent()) prop_list.push_back(n->parent());
        }
      }
    }
  }
}
#endif
