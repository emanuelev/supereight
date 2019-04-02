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

#ifndef __PREPROCESSING_HPP
#define __PREPROCESSING_HPP

#include <functional>

#include <timings.h>
#include <perfstats.h>
#include <se/commons.h>
#include <se/utils/math_utils.h>
#include <se/image/image.hpp>

void bilateralFilterKernel(se::Image<float>&         out,
                           const se::Image<float>&   in,
                           const std::vector<float>& gaussian,
                           const float               e_d,
                           const int                 radius);

void depth2vertexKernel(se::Image<Eigen::Vector3f>& vertex,
                        const se::Image<float>&     depth,
                        const Eigen::Matrix4f&      inv_K);

/**
 * NegY should only be true when reading an ICL-NUIM dataset which has a
 * left-handed coordinate system (the y focal length will be negative).
 */
template <bool NegY>
void vertex2normalKernel(se::Image<Eigen::Vector3f>&       out,
                         const se::Image<Eigen::Vector3f>& in);

void mm2metersKernel(se::Image<float>&      out,
                     const unsigned short*  in,
                     const Eigen::Vector2i& input_size);

void halfSampleRobustImageKernel(se::Image<float>&       out,
                                 const se::Image<float>& in,
                                 const float             e_d,
                                 const int               r);

#endif

