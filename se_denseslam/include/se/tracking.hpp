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

#ifndef __TRACKING_HPP
#define __TRACKING_HPP

#include <timings.h>
#include <perfstats.h>
#include <se/commons.h>
#include <se/utils/math_utils.h>
#include <se/image/image.hpp>

void new_reduce(int                    blockIndex,
                float*                 out,
                TrackData*             J,
                const Eigen::Vector2i& Jsize,
                const Eigen::Vector2i& size);

void reduceKernel(float*                 out,
                  TrackData*             J,
                  const Eigen::Vector2i& Jsize,
		          const Eigen::Vector2i& size);

void trackKernel(TrackData*                        output,
                 const se::Image<Eigen::Vector3f>& inVertex,
                 const se::Image<Eigen::Vector3f>& inNormal,
                 const se::Image<Eigen::Vector3f>& refVertex,
                 const se::Image<Eigen::Vector3f>& refNormal,
                 const Eigen::Matrix4f&            Ttrack,
                 const Eigen::Matrix4f&            view,
                 const float                       dist_threshold,
                 const float                       normal_threshold);

bool updatePoseKernel(Eigen::Matrix4f& pose,
                      const float*     output,
		              const float      icp_threshold);

bool checkPoseKernel(Eigen::Matrix4f&       pose,
                     Eigen::Matrix4f&       oldPose,
                     const float*           output,
                     const Eigen::Vector2i& imageSize,
                     const float            track_threshold);

#endif

