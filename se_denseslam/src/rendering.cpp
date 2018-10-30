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
#include <math_utils.h>
#include "timings.h"
#include <se/continuous/volume_template.hpp>
#include <tuple>
#include <sophus/se3.hpp>

/* Raycasting implementations */ 
#include "bfusion/rendering_impl.hpp"
#include "kfusion/rendering_impl.hpp"

template<typename T>
void raycastKernel(const Volume<T>& volume, float3* vertex, float3* normal, uint2 inputSize, 
    const Matrix4 view, const float nearPlane, const float farPlane, 
    const float mu, const float step, const float largestep) {
  TICK();

  unsigned int y;
#pragma omp parallel for shared(normal, vertex), private(y)
  for (y = 0; y < inputSize.y; y++)
#pragma simd
    for (unsigned int x = 0; x < inputSize.x; x++) {

      uint2 pos = make_uint2(x, y);
      const Eigen::Vector3f dir = 
        (to_eigen(view).topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
      const float3 transl = get_translation(view);
      se::ray_iterator<typename Volume<T>::field_type> ray(*volume._map_index,
          Eigen::Vector3f(transl.x, transl.y, transl.z), dir, nearPlane, farPlane);
      ray.next();
      const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
      const float4 hit = t_min > 0.f ? 
        raycast(volume, transl, make_float3(dir(0), dir(1), dir(2)), t_min, ray.tmax(), mu, step, largestep) : 
        make_float4(0.f);
      if(hit.w > 0.0) {
        vertex[pos.x + pos.y * inputSize.x] = make_float3(hit);
        Eigen::Vector3f tmp = volume.grad(make_float3(hit), 
            [](const auto& val){ return val.x; });
        float3 surfNorm = make_float3(tmp(0), tmp(1), tmp(2));
        if (length(surfNorm) == 0) {
          //normal[pos] = normalize(surfNorm); // APN added
          normal[pos.x + pos.y * inputSize.x] = make_float3(INVALID, 0, 0);
        } else {
          // Invert normals if SDF 
          normal[pos.x + pos.y * inputSize.x] = std::is_same<T, SDF>::value ?
            normalize(-1.f * surfNorm) : normalize(surfNorm);
        }
      } else {
        vertex[pos.x + pos.y * inputSize.x] = make_float3(0);
        normal[pos.x + pos.y * inputSize.x] = make_float3(INVALID, 0, 0);
      }
    }
  TOCK("raycastKernel", inputSize.x * inputSize.y);
}

void renderNormalKernel(uchar3* out, const float3* normal, uint2 normalSize) {
	TICK();
	unsigned int y;
#pragma omp parallel for \
        shared(out), private(y)
	for (y = 0; y < normalSize.y; y++)
		for (unsigned int x = 0; x < normalSize.x; x++) {
			uint pos = (x + y * normalSize.x);
			float3 n = normal[pos];
			if (n.x == -2) {
				out[pos] = make_uchar3(0, 0, 0);
			} else {
				n = normalize(n);
				out[pos] = make_uchar3(n.x * 128 + 128, n.y * 128 + 128,
						n.z * 128 + 128);
			}
		}
	TOCK("renderNormalKernel", normalSize.x * normalSize.y);
}

void renderDepthKernel(uchar4* out, float * depth, uint2 depthSize,
		const float nearPlane, const float farPlane) {
	TICK();

	float rangeScale = 1 / (farPlane - nearPlane);

	unsigned int y;
#pragma omp parallel for \
        shared(out), private(y)
	for (y = 0; y < depthSize.y; y++) {
		int rowOffeset = y * depthSize.x;
		for (unsigned int x = 0; x < depthSize.x; x++) {

			unsigned int pos = rowOffeset + x;

			if (depth[pos] < nearPlane)
				out[pos] = make_uchar4(255, 255, 255, 0); // The forth value is a padding in order to align memory
			else {
				if (depth[pos] > farPlane)
					out[pos] = make_uchar4(0, 0, 0, 0); // The forth value is a padding in order to align memory
				else {
					const float d = (depth[pos] - nearPlane) * rangeScale;
					out[pos] = gs2rgb(d);
				}
			}
		}
	}
	TOCK("renderDepthKernel", depthSize.x * depthSize.y);
}

void renderTrackKernel(uchar4* out, const TrackData* data, uint2 outSize) {
  TICK();

  unsigned int y;
#pragma omp parallel for \
  shared(out), private(y)
  for (y = 0; y < outSize.y; y++)
    for (unsigned int x = 0; x < outSize.x; x++) {
      uint pos = x + y * outSize.x;
      switch (data[pos].result) {
        case 1:
          out[pos] = make_uchar4(128, 128, 128, 0);  // ok	 GREY
          break;
        case -1:
          out[pos] = make_uchar4(0, 0, 0, 0);      // no input BLACK
          break;
        case -2:
          out[pos] = make_uchar4(255, 0, 0, 0);        // not in image RED
          break;
        case -3:
          out[pos] = make_uchar4(0, 255, 0, 0);    // no correspondence GREEN
          break;
        case -4:
          out[pos] = make_uchar4(0, 0, 255, 0);        // to far away BLUE
          break;
        case -5:
          out[pos] = make_uchar4(255, 255, 0, 0);     // wrong normal YELLOW
          break;
        default:
          out[pos] = make_uchar4(255, 128, 128, 0);
          break;
      }
    }
  TOCK("renderTrackKernel", outSize.x * outSize.y);
}

template <typename T>
void renderVolumeKernel(const Volume<T>& volume, uchar4* out, const uint2 depthSize, const Matrix4 view, 
    const float nearPlane, const float farPlane, const float mu,
		const float step, const float largestep, const float3 light,
		const float3 ambient, bool render, const float3 * vertex, 
    const float3 * normal) {
  TICK();
  unsigned int y;
#pragma omp parallel for shared(out), private(y)
  for (y = 0; y < depthSize.y; y++) {
    for (unsigned int x = 0; x < depthSize.x; x++) {
      float4 hit;
      float3 test, surfNorm;

      if(render) {
        const Eigen::Vector3f dir = 
          (to_eigen(view).topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
        const float3 transl = get_translation(view);
        se::ray_iterator<typename Volume<T>::field_type> ray(*volume._map_index, 
            Eigen::Vector3f(transl.x, transl.y, transl.z), dir, nearPlane, 
            farPlane);
        ray.next();
        const float t_min = ray.tmin(); /* Get distance to the first intersected block */
        hit = t_min > 0.f ? 
          raycast(volume, transl, make_float3(dir(0), dir(1), dir(2)), t_min, ray.tmax(), mu, step, largestep) : 
          make_float4(0.f);
        if (hit.w > 0) {
          test = make_float3(hit);
          Eigen::Vector3f tmp = volume.grad(make_float3(hit), 
              [](const auto& val){ return val.x; });
          surfNorm = make_float3(tmp(0), tmp(1), tmp(2));

          // Invert normals if SDF 
          surfNorm = std::is_same<T, SDF>::value ? -1.f * surfNorm : surfNorm;
        } else {
          out[x + depthSize.x*y] = make_uchar4(0, 0, 0, 0); // The forth value is a padding to align memory
        }
      }
      else {
        test = vertex[x + depthSize.x*y];
        surfNorm = normal[x + depthSize.x*y];
      }

      if (surfNorm.x != INVALID && length(surfNorm) > 0) {
        const float3 diff = normalize(test - light);
        const float dir = fmaxf(dot(normalize(surfNorm), diff),
            0.f);
        const float3 col = clamp(make_float3(dir) + ambient, 0.f,
            1.f) * 255;
        out[x + depthSize.x*y] = make_uchar4(col.x, col.y, col.z, 0); // The forth value is a padding to align memory
      } else {
        out[x + depthSize.x*y] = make_uchar4(0, 0, 0, 0); // The forth value is a padding to align memory
      }
    }
  }
  TOCK("renderVolumeKernel", depthSize.x * depthSize.y);
}

