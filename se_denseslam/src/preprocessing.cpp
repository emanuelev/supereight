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
#include "timings.h"
#include <math_utils.h>

#include <functional>
#include <se/image/image.hpp>

void bilateralFilterKernel(float* out, const float* in, uint2 size,
		const float * gaussian, float e_d, int r) {
	TICK()
		uint y;
		float e_d_squared_2 = e_d * e_d * 2;
#pragma omp parallel for \
	    shared(out),private(y)   
		for (y = 0; y < size.y; y++) {
			for (uint x = 0; x < size.x; x++) {
				uint pos = x + y * size.x;
				if (in[pos] == 0) {
					out[pos] = 0;
					continue;
				}

				float sum = 0.0f;
				float t = 0.0f;

				const float center = in[pos];

				for (int i = -r; i <= r; ++i) {
					for (int j = -r; j <= r; ++j) {
						uint2 curPos = make_uint2(clamp(x + i, 0u, size.x - 1),
								clamp(y + j, 0u, size.y - 1));
						const float curPix = in[curPos.x + curPos.y * size.x];
						if (curPix > 0) {
							const float mod = sq(curPix - center);
							const float factor = gaussian[i + r]
									* gaussian[j + r]
									* expf(-mod / e_d_squared_2);
							t += factor * curPix;
							sum += factor;
						}
					}
				}
				out[pos] = t / sum;
			}
		}
		TOCK("bilateralFilterKernel", size.x * size.y);
}

  void depth2vertexKernel(se::Image<Eigen::Vector3f>& vertex, 
                         const se::Image<float>& depth,
                         const Eigen::Matrix4f invK) {
	TICK();
	int x, y;
#pragma omp parallel for \
         shared(vertex), private(x, y)
	for (y = 0; y < depth.height(); y++) {
		for (x = 0; x < depth.width(); x++) {

			if (depth[x + y * depth.width()] > 0) {
				vertex[x + y * depth.width()] = (depth[x + y * depth.width()]
						* invK * Eigen::Vector4f(x, y, 1.f, 0.f)).head<3>();
        }
			else {
				vertex[x + y * depth.width()] = Eigen::Vector3f::Constant(0);
			}
		}
	}
	TOCK("depth2vertexKernel", imageSize.x * imageSize.y);
}

template <bool NegY>
void vertex2normalKernel(se::Image<Eigen::Vector3f>&  out, 
                         const se::Image<Eigen::Vector3f>& in) {
	TICK();
	int x, y;
  int width = in.width();
  int height = in.height();
#pragma omp parallel for \
        shared(out), private(x,y)
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			const Eigen::Vector2i pleft = Eigen::Vector2i(max(int(x) - 1, 0), y);
			const Eigen::Vector2i pright = Eigen::Vector2i(min(x + 1, (int) width - 1),
					y);

      // Swapped to match the left-handed coordinate system of ICL-NUIM
      Eigen::Vector2i pup, pdown;
      if(NegY) {
        pup = Eigen::Vector2i(x, max(int(y) - 1, 0));
        pdown = Eigen::Vector2i(x, min(y + 1, ((int) height) - 1));
      } else {
        pdown = Eigen::Vector2i(x, max(int(y) - 1, 0));
        pup = Eigen::Vector2i(x, min(y + 1, ((int) height) - 1));
      }

			const Eigen::Vector3f left  = in[pleft.x() + width * pleft.y()];
			const Eigen::Vector3f right = in[pright.x() + width * pright.y()];
			const Eigen::Vector3f up    = in[pup.x() + width * pup.y()];
			const Eigen::Vector3f down  = in[pdown.x() + width * pdown.y()];

			if (left.z() == 0 || right.z() == 0 || up.z() == 0 || down.z() == 0) {
				out[x + y * width].x() = INVALID;
				continue;
			}
			const Eigen::Vector3f dxv = right - left;
			const Eigen::Vector3f dyv = up - down;
      out[x + y * width] =  dxv.cross(dyv).normalized();
		}
	}
	TOCK("vertex2normalKernel", width * height);
}

void mm2metersKernel(se::Image<float> out, const se::Image<ushort> in) {
	TICK();
	// Check for unsupported conditions
	if ((in.width() < out.width()) || in.height() < out.height()) {
		std::cerr << "Invalid ratio." << std::endl;
		exit(1);
	}
	if ((in.width() % out.width() != 0) || (in.height() % out.height() != 0)) {
		std::cerr << "Invalid ratio." << std::endl;
		exit(1);
	}
	if ((in.width() / out.width() != in.height() / out.height())) {
		std::cerr << "Invalid ratio." << std::endl;
		exit(1);
	}

	int ratio = in.width() / out.width();
	int y;
#pragma omp parallel for \
        shared(out), private(y)
	for (y = 0; y < out.height(); y++)
		for (int x = 0; x < out.width(); x++) {
			out[x + out.width() * y] = in[x * ratio + in.width() * y * ratio]
					/ 1000.0f;
		}
	TOCK("mm2metersKernel", outSize.x * outSize.y);
}

void halfSampleRobustImageKernel(float* out, const float* in, uint2 inSize,
		const float e_d, const int r) {
	TICK();
	uint2 outSize = make_uint2(inSize.x / 2, inSize.y / 2);
	unsigned int y;
#pragma omp parallel for \
        shared(out), private(y)
	for (y = 0; y < outSize.y; y++) {
		for (unsigned int x = 0; x < outSize.x; x++) {
			uint2 pixel = make_uint2(x, y);
			const uint2 centerPixel = 2 * pixel;

			float sum = 0.0f;
			float t = 0.0f;
			const float center = in[centerPixel.x
					+ centerPixel.y * inSize.x];
			for (int i = -r + 1; i <= r; ++i) {
				for (int j = -r + 1; j <= r; ++j) {
					uint2 cur = make_uint2(
							clamp(
									make_int2(centerPixel.x + j,
											centerPixel.y + i), make_int2(0),
									make_int2(2 * outSize.x - 1,
											2 * outSize.y - 1)));
					float current = in[cur.x + cur.y * inSize.x];
					if (fabsf(current - center) < e_d) {
						sum += 1.0f;
						t += current;
					}
				}
			}
			out[pixel.x + pixel.y * outSize.x] = t / sum;
		}
	}
	TOCK("halfSampleRobustImageKernel", outSize.x * outSize.y);
}

