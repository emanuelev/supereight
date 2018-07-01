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

void depth2vertexKernel(float3* vertex, const float * depth, uint2 imageSize,
		const Matrix4 invK) {
	TICK();
	unsigned int x, y;
#pragma omp parallel for \
         shared(vertex), private(x, y)
	for (y = 0; y < imageSize.y; y++) {
		for (x = 0; x < imageSize.x; x++) {

			if (depth[x + y * imageSize.x] > 0) {
				vertex[x + y * imageSize.x] = depth[x + y * imageSize.x]
						* (rotate(invK, make_float3(x, y, 1.f)));
        }
			else {
				vertex[x + y * imageSize.x] = make_float3(0);
			}
		}
	}
	TOCK("depth2vertexKernel", imageSize.x * imageSize.y);
}

template <typename FieldType, bool NegY>
void vertex2normalKernel(float3 * out, const float3 * in, uint2 imageSize) {
	TICK();
	unsigned int x, y;
#pragma omp parallel for \
        shared(out), private(x,y)
	for (y = 0; y < imageSize.y; y++) {
		for (x = 0; x < imageSize.x; x++) {
			const uint2 pleft = make_uint2(max(int(x) - 1, 0), y);
			const uint2 pright = make_uint2(min(x + 1, (int) imageSize.x - 1),
					y);

      // Swapped to match the left-handed coordinate system of ICL-NUIM
      uint2 pup, pdown;
      if(NegY) {
        pup = make_uint2(x, max(int(y) - 1, 0));
        pdown = make_uint2(x, min(y + 1, ((int) imageSize.y) - 1));
      } else {
        pdown = make_uint2(x, max(int(y) - 1, 0));
        pup = make_uint2(x, min(y + 1, ((int) imageSize.y) - 1));
      }

			const float3 left = in[pleft.x + imageSize.x * pleft.y];
			const float3 right = in[pright.x + imageSize.x * pright.y];
			const float3 up = in[pup.x + imageSize.x * pup.y];
			const float3 down = in[pdown.x + imageSize.x * pdown.y];

			if (left.z == 0 || right.z == 0 || up.z == 0 || down.z == 0) {
				out[x + y * imageSize.x].x = INVALID;
				continue;
			}
			const float3 dxv = right - left;
			const float3 dyv = up - down;
      if(std::is_same<FieldType, SDF>::value) {
        out[x + y * imageSize.x] =  normalize(cross(dyv, dxv)); // switched dx and dy to get factor -1
      }
      else if(std::is_same<FieldType, OFusion>::value) {
        out[x + y * imageSize.x] =  normalize(cross(dxv, dyv)); // switched dx and dy to get factor -1
      }
		}
	}
	TOCK("vertex2normalKernel", imageSize.x * imageSize.y);
}

void mm2metersKernel(float * out, uint2 outSize, const ushort * in,
		uint2 inSize) {
	TICK();
	// Check for unsupported conditions
	if ((inSize.x < outSize.x) || (inSize.y < outSize.y)) {
		std::cerr << "Invalid ratio." << std::endl;
		exit(1);
	}
	if ((inSize.x % outSize.x != 0) || (inSize.y % outSize.y != 0)) {
		std::cerr << "Invalid ratio." << std::endl;
		exit(1);
	}
	if ((inSize.x / outSize.x != inSize.y / outSize.y)) {
		std::cerr << "Invalid ratio." << std::endl;
		exit(1);
	}

	int ratio = inSize.x / outSize.x;
	unsigned int y;
#pragma omp parallel for \
        shared(out), private(y)
	for (y = 0; y < outSize.y; y++)
		for (unsigned int x = 0; x < outSize.x; x++) {
			out[x + outSize.x * y] = in[x * ratio + inSize.x * y * ratio]
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

