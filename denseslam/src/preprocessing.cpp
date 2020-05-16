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
#include <supereight/shared/timings.h>
#include <supereight/utils/math_utils.h>

#include <functional>
#include <supereight/memory/image.hpp>

void bilateralFilterKernel(se::Image<float>& out, const se::Image<float>& in,
    const std::vector<float>& gaussian, float e_d, int r) {
    if ((in.width() != out.width()) || in.height() != out.height()) {
        std::cerr << "input/output image sizes differ." << std::endl;
        exit(1);
    }

    TICK()
    const int width  = in.width();
    const int height = in.height();
    int y;
    float e_d_squared_2 = e_d * e_d * 2;
#pragma omp parallel for shared(out), private(y)
    for (y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            unsigned int pos = x + y * width;
            if (in[pos] == 0) {
                out[pos] = 0;
                continue;
            }

            float sum = 0.0f;
            float t   = 0.0f;

            const float center = in[pos];

            for (int i = -r; i <= r; ++i) {
                for (int j = -r; j <= r; ++j) {
                    Eigen::Vector2i curPos =
                        Eigen::Vector2i(se::math::clamp(x + i, 0, width - 1),
                            se::math::clamp(y + j, 0, height - 1));
                    const float curPix = in[curPos.x() + curPos.y() * width];
                    if (curPix > 0) {
                        const float mod    = se::math::sq(curPix - center);
                        const float factor = gaussian[i + r] * gaussian[j + r] *
                            expf(-mod / e_d_squared_2);
                        t += factor * curPix;
                        sum += factor;
                    }
                }
            }
            out[pos] = t / sum;
        }
    }
    TOCK("bilateralFilterKernel", width * height);
}

void mm2metersKernel(se::Image<float>& out, const unsigned short* in,
    const Eigen::Vector2i& inputSize) {
    TICK();
    // Check for unsupported conditions
    if ((inputSize.x() < out.width()) || inputSize.y() < out.height()) {
        std::cerr << "Invalid ratio." << std::endl;
        exit(1);
    }
    if ((inputSize.x() % out.width() != 0) ||
        (inputSize.y() % out.height() != 0)) {
        std::cerr << "Invalid ratio." << std::endl;
        exit(1);
    }
    if ((inputSize.x() / out.width() != inputSize.y() / out.height())) {
        std::cerr << "Invalid ratio." << std::endl;
        exit(1);
    }

    int ratio = inputSize.x() / out.width();
    int y;
#pragma omp parallel for shared(out), private(y)
    for (y = 0; y < out.height(); y++)
        for (int x = 0; x < out.width(); x++) {
            out[x + out.width() * y] =
                in[x * ratio + inputSize.x() * y * ratio] / 1000.0f;
        }
    TOCK("mm2metersKernel", outSize.x * outSize.y);
}
