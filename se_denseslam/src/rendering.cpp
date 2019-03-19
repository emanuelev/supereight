/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

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
#include <se/utils/math_utils.h>
#include <se/commons.h>
#include <timings.h>
#include <tuple>

#include <sophus/se3.hpp>
#include <se/continuous/volume_template.hpp>
#include <se/image/image.hpp>
#include <se/ray_iterator.hpp>

/* Raycasting implementations */ 
#include "bfusion/rendering_impl.hpp"
#include "kfusion/rendering_impl.hpp"

template<typename T>
void raycastKernel(const Volume<T>& volume, se::Image<Eigen::Vector3f>& vertex,
   se::Image<Eigen::Vector3f>& normal,
   const Eigen::Matrix4f& view, const float near_plane, const float far_plane,
   const float mu, const float step, const float large_step) {
  TICK();
  int y;
#pragma omp parallel for shared(normal, vertex), private(y)
  for (y = 0; y < vertex.height(); y++)
#pragma simd
    for (int x = 0; x < vertex.width(); x++) {

      Eigen::Vector2i pos(x, y);
      const Eigen::Vector3f dir = 
        (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
      const Eigen::Vector3f transl = view.topRightCorner<3, 1>();
      se::ray_iterator<T> ray(*volume.map_index_, transl, dir, near_plane, far_plane);
      ray.next();
      const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
      const Eigen::Vector4f hit = t_min > 0.f ? 
        raycast(volume, transl, dir, t_min, ray.tmax(), mu, step, large_step) :
        Eigen::Vector4f::Constant(0.f);
      if(hit.w() > 0.0) {
        vertex[x + y * vertex.width()] = hit.head<3>();
        Eigen::Vector3f depth_size = volume.grad(hit.head<3>(),
            [](const auto& val){ return val.x; });
        if (depth_size.norm() == 0) {
          //normal[pos] = normalize(depth_size); // APN added
          normal[pos.x() + pos.y() * normal.width()] = Eigen::Vector3f(INVALID, 0, 0);
        } else {
          // Invert normals if SDF 
          normal[pos.x() + pos.y() * normal.width()] = std::is_same<T, SDF>::value ?
            (-1.f * depth_size).normalized() : depth_size.normalized();
        }
      } else {
        vertex[pos.x() + pos.y() * vertex.width()] = Eigen::Vector3f::Constant(0);
        normal[pos.x() + pos.y() * normal.width()] = Eigen::Vector3f(INVALID, 0, 0);
      }
    }
  TOCK("raycastKernel", input_size.x * input_size.y);
}

// void renderNormalKernel(uchar3* out, const float3* normal, uint2 normalSize) {
// 	TICK();
// 	unsigned int y;
// #pragma omp parallel for shared(out), private(y)
// 	for (y = 0; y < normalSize.y; y++)
// 		for (unsigned int x = 0; x < normalSize.x; x++) {
// 			unsigned int pos = (x + y * normalSize.x);
// 			float3 n = normal[pos];
// 			if (n.x == -2) {
// 				out[pos] = make_uchar3(0, 0, 0);
// 			} else {
// 				n = normalize(n);
// 				out[pos] = make_uchar3(n.x * 128 + 128, n.y * 128 + 128,
// 						n.z * 128 + 128);
// 			}
// 		}
// 	TOCK("renderNormalKernel", normalSize.x * normalSize.y);
// }

void renderDepthKernel(unsigned char* out, float * depth, 
    const Eigen::Vector2i& depth_size, const float near_plane,
    const float far_plane) {
	TICK();

	float range_scale = 1 / (far_plane - near_plane);

	int y;
#pragma omp parallel for \
        shared(out), private(y)
	for (y = 0; y < depth_size.y(); y++) {
		int row_offset = y * depth_size.x();
		for (int x = 0; x < depth_size.x(); x++) {

			unsigned int pos = row_offset + x;
      unsigned int idx = pos * 4;

      if (depth[pos] < near_plane) {
        out[idx + 0] = 255;
        out[idx + 1] = 255;
        out[idx + 2] = 255;
        out[idx + 3] = 0;
      }
      else if (depth[pos] > far_plane) {
        out[idx + 0] = 0;
        out[idx + 1] = 0;
        out[idx + 2] = 0;
        out[idx + 3] = 0;
      }
      else {
        const float d = (depth[pos] - near_plane) * range_scale;
        unsigned char rgbw[4];
        gs2rgb(d, rgbw);
        out[idx + 0] = rgbw[0];
        out[idx + 1] = rgbw[1];
        out[idx + 2] = rgbw[2];
        out[idx + 3] = rgbw[3];
      }
		}
	}
	TOCK("renderDepthKernel", depth_size.x * depth_size.y);
}

void renderTrackKernel(unsigned char* out, 
    const TrackData* data, 
    const Eigen::Vector2i& out_size) {
  TICK();

  int y;
#pragma omp parallel for \
  shared(out), private(y)
  for (y = 0; y < out_size.y(); y++)
    for (int x = 0; x < out_size.x(); x++) {
      const int pos = x + out_size.x()*y;
      const int idx = pos * 4;
      switch (data[pos].result) {
        case 1:
          out[idx + 0] = 128;
          out[idx + 1] = 128;
          out[idx + 2] = 128;
          out[idx + 3] = 0;
          break;
        case -1:
          out[idx + 0] = 0;
          out[idx + 1] = 0;
          out[idx + 2] = 0;
          out[idx + 3] = 0;
          break;
        case -2:
          out[idx + 0] = 255;
          out[idx + 1] = 0;
          out[idx + 2] = 0;
          out[idx + 3] = 0;
          break;
        case -3:
          out[idx + 0] = 0;
          out[idx + 1] = 255;
          out[idx + 2] = 0;
          out[idx + 3] = 0;
          break;
        case -4:
          out[idx + 0] = 0;
          out[idx + 1] = 0;
          out[idx + 2] = 255;
          out[idx + 3] = 0;
          break;
        case -5:
          out[idx + 0] = 255;
          out[idx + 1] = 255;
          out[idx + 2] = 0;
          out[idx + 3] = 0;
          break;
        default:
          out[idx + 0] = 255;
          out[idx + 1] = 128;
          out[idx + 2] = 128;
          out[idx + 3] = 0;
          break;
      }
    }
  TOCK("renderTrackKernel", out_size.x * out_size.y);
}

template <typename T>
void renderVolumeKernel(const Volume<T>& volume, 
    unsigned char* out, // RGBW packed
    const Eigen::Vector2i& depth_size,
    const Eigen::Matrix4f& view, 
    const float near_plane,
    const float far_plane,
    const float mu,
		const float step, 
    const float large_step,
    const Eigen::Vector3f& light,
		const Eigen::Vector3f& ambient, 
    bool render, 
    const se::Image<Eigen::Vector3f>& vertex, 
    const se::Image<Eigen::Vector3f>& normal) {
  TICK();
  int y;
#pragma omp parallel for shared(out), private(y)
  for (y = 0; y < depth_size.y(); y++) {
    for (int x = 0; x < depth_size.x(); x++) {
      Eigen::Vector4f hit;
      Eigen::Vector3f test, depth_size;
      const int idx = (x + depth_size.x()*y) * 4;

      if(render) {
        const Eigen::Vector3f dir = 
          (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
        const Eigen::Vector3f transl = view.topRightCorner<3, 1>();
        se::ray_iterator<typename Volume<T>::field_type> ray(*volume.map_index_, 
            transl, dir, near_plane, far_plane);
        ray.next();
        const float t_min = ray.tmin(); /* Get distance to the first intersected block */
        hit = t_min > 0.f ? 
          raycast(volume, transl, dir, t_min, ray.tmax(), mu, step, large_step) :
          Eigen::Vector4f::Constant(0.f);
        if (hit.w() > 0) {
          test = hit.head<3>();
          depth_size = volume.grad(test, [](const auto& val){ return val.x; });

          // Invert normals if SDF 
          depth_size = std::is_same<T, SDF>::value ? -1.f * depth_size : depth_size;
        } else {
          depth_size = Eigen::Vector3f(INVALID, 0, 0);
        }
      }
      else {
        test = vertex[x + depth_size.x()*y];
        depth_size = normal[x + depth_size.x()*y];
      }

      if (depth_size.x() != INVALID && depth_size.norm() > 0) {
        const Eigen::Vector3f diff = (test - light).normalized();
        const Eigen::Vector3f dir = Eigen::Vector3f::Constant(fmaxf(depth_size.normalized().dot(diff), 0.f));
        Eigen::Vector3f col = dir + ambient;
        se::math::clamp(col, Eigen::Vector3f::Constant(0.f), Eigen::Vector3f::Constant(1.f));
        col *=  255.f;
        out[idx + 0] = col.x();
        out[idx + 1] = col.y();
        out[idx + 2] = col.z();
        out[idx + 3] = 0;
      } else {
        out[idx + 0] = 0;
        out[idx + 1] = 0;
        out[idx + 2] = 0;
        out[idx + 3] = 0;
      }
    }
  }
  TOCK("renderVolumeKernel", depth_size.x * depth_size.y);
}

