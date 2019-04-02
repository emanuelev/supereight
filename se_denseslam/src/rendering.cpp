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

#include <se/rendering.hpp>

void renderDepthKernel(unsigned char*         out,
                       const float*           depth,
                       const Eigen::Vector2i& depth_size,
                       const float            near_plane,
                       const float            far_plane) {
  TICK();

  const float range_scale = 1 / (far_plane - near_plane);

#pragma omp parallel for
  for (int y = 0; y < depth_size.y(); y++) {
    const int row_offset = y * depth_size.x();
    for (int x = 0; x < depth_size.x(); x++) {

      // Linear pixel index.
      const unsigned int pos = row_offset + x;
      // Linear byte index.
      const unsigned int idx = pos * 4;

      if (depth[pos] < near_plane) {
        out[idx + 0] = 255;
        out[idx + 1] = 255;
        out[idx + 2] = 255;
        out[idx + 3] = 0;
      } else if (depth[pos] > far_plane) {
        out[idx + 0] = 0;
        out[idx + 1] = 0;
        out[idx + 2] = 0;
        out[idx + 3] = 0;
      } else {
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
  TOCK("renderDepthKernel", depth_size.x() * depth_size.y());
}

void renderTrackKernel(unsigned char*         out,
                       const TrackData*       data,
                       const Eigen::Vector2i& out_size) {
  TICK();

#pragma omp parallel for
  for (int y = 0; y < out_size.y(); y++) {
    for (int x = 0; x < out_size.x(); x++) {
      // Linear pixel index.
      const int pos = x + out_size.x()*y;
      // Linear byte index.
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
  }
  TOCK("renderTrackKernel", out_size.x() * out_size.y());
}

