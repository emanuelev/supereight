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

#ifndef BFUSION_MAPPING_HPP
#define BFUSION_MAPPING_HPP

#include <se/node.hpp>
#include <se/functors/projective_functor.hpp>
#include <se/constant_parameters.h>
#include <se/image/image.hpp>
#include "bspline_lookup.cc"

float interpDepth(const se::Image<float>& depth, const Eigen::Vector2f proj) {
  // https://en.wikipedia.org/wiki/Bilinear_interpolation

  // Pixels version
  const float x1 = (floorf(proj.x()));
  const float y1 = (floorf(proj.y() + 1));
  const float x2 = (floorf(proj.x() + 1));
  const float y2 = (floorf(proj.y()));

  // Half pixels
  // const float x1 = (float) (int(proj.x - 0.5f)) + 0.5f;
  // const float y1 = (float) (int(proj.y + 0.5f)) + 0.5f;
  // const float x2 = (float) (int(proj.x + 0.5f)) + 0.5f;
  // const float y2 = (float) (int(proj.y - 0.5f)) + 0.5f;

  const float d11 = depth[int(x1) +  depth.width()*int(y1)];
  const float d12 = depth[int(x1) +  depth.width()*int(y2)];
  const float d21 = depth[int(x2) +  depth.width()*int(y1)];
  const float d22 = depth[int(x2) +  depth.width()*int(y2)];
  
  if( d11 == 0.f || d12 == 0.f || d21 == 0.f || d22 == 0.f ) return 0.f;

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
  if (fabs(d - d11) < interp_thresh && fabs(d - d12) < interp_thresh &&
      fabs(d - d21) < interp_thresh && fabs(d - d22) < interp_thresh) 
    return d;
  else 
    return depth[int(proj.x() + 0.5f) + depth.width()*int(proj.y()+0.5f)];

  // Non-Filtering version
  // return  1.f / 
  //         ( (   f11 * (x2 - proj.x) * (y2 - proj.y)
  //             + f21 * (proj.x - x1) * (y2 - proj.y)
  //             + f12 * (x2 - proj.x) * (proj.y - y1)
  //             + f22 * (proj.x - x1) * (proj.y - y1)
  //           ) / ((x2 - x1) * (y2 - y1))
  //         );
}

static inline float bspline(float t){
  float value = 0.f;
  if(t >= -3.0f && t <= -1.0f) {
    value = std::pow((3 + t), 3)/48.0f;   
  } else if( t > -1 && t <= 1) {
    value = 0.5f + (t*(3 + t)*(3 - t))/24.f;
  } else if(t > 1 && t <= 3){
    value = 1 - std::pow((3 - t), 3)/48.f;
  } else if(t > 3) {
    value = 1.f;
  }
  return value;
}

static inline float H(const float val){
  const float Q_1 = bspline(val);
  const float Q_2 = bspline(val - 3);
  return Q_1 - Q_2 * 0.5f;
}

static const double const_offset =  0.0000001f;
const float scale_factor = (1.f - (farPlane - nearPlane) * const_offset); 

static inline float const_offset_integral(float t){
  float value = 0.f;
  if (nearPlane <= t && t <= farPlane)
      return (t - nearPlane) * const_offset;
  else if (farPlane < t)
      return (farPlane - nearPlane) * const_offset;
  return value;
}

static inline float bspline_memoized(float t){
  float value = 0.f;
  constexpr float inverseRange = 1/6.f;
  if(t >= -3.0f && t <= 3.0f) {
    unsigned int idx = ((t + 3.f)*inverseRange)*(bspline_num_samples - 1) + 0.5f;
    return bspline_lookup[idx];
  } 
  else if(t > 3) {
    value = 1.f;
  }
  return value;
}

static inline float HNew(const float val,const  float ){
  const float Q_1 = bspline_memoized(val)    ; // * scale_factor + const_offset_integral(d_xr      );
  const float Q_2 = bspline_memoized(val - 3); // * scale_factor + const_offset_integral(d_xr - 3.f);
  return Q_1 - Q_2 * 0.5f;
}

static inline float updateLogs(const float prior, const float sample){
  // return (prior + se::math::clamp(log2(sample / (1.f - sample)), -100, 100));
  return (prior + log2(sample / (1.f - sample)));
}

static inline float applyWindow(const float occupancy, const float , 
    const float delta_t, const float tau){
  float fraction = 1.f / (1.f + (delta_t / tau));
  fraction = std::max(0.5f,fraction);
  return occupancy * fraction;
}

struct bfusion_update {

  template <typename DataHandlerT>
  void operator()(DataHandlerT& handler, const Eigen::Vector3i&, 
      const Eigen::Vector3f& pos, const Eigen::Vector2f& pixel) {

    const Eigen::Vector2i px = pixel.cast <int> ();
    const float depthSample = depth[px(0) + depthSize(0)*px(1)];
    if (depthSample <=  0) return;

    const float diff = (pos(2) - depthSample)
      * std::sqrt( 1 + se::math::sq(pos(0) / pos(2)) + se::math::sq(pos(1) / pos(2)));
    float sigma = se::math::clamp(noiseFactor * se::math::sq(pos(2)), 0.005f, 0.05f);
    float sample = HNew(diff/sigma, pos(2));
    if(sample == 0.5f) return;
    sample = se::math::clamp(sample, 0.03f, 0.97f);
    auto data = handler.get();
    const double delta_t = timestamp - data.y;
    data.x = applyWindow(data.x, SURF_BOUNDARY, delta_t, CAPITAL_T);
    data.x = se::math::clamp(updateLogs(data.x, sample), BOTTOM_CLAMP, TOP_CLAMP);
    data.y = timestamp;
    handler.set(data);
  } 

  bfusion_update(const float * d, const Eigen::Vector2i framesize, float n, 
      float t): depth(d), depthSize(framesize), noiseFactor(n), timestamp(t){};

  const float * depth;
  Eigen::Vector2i depthSize;
  float noiseFactor;
  float timestamp;
};
#endif
