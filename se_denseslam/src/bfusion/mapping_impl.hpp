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
 * Struct to hold the data and perform the update of the map from a single
 * depth frame.
 */
struct bfusion_update {
  const float* depth;
  Eigen::Vector2i depthSize;
  float noiseFactor;
  float timestamp;
  float voxelsize;

  bfusion_update(const float*           d,
                 const Eigen::Vector2i& framesize,
                 float                  n,
                 float                  t,
                 float                  vs)
    : depth(d), depthSize(framesize), noiseFactor(n),
  timestamp(t), voxelsize(vs) {};

  template <typename DataHandlerT>
  void operator()(DataHandlerT&          handler,
                  const Eigen::Vector3i&,
                  const Eigen::Vector3f& pos,
                  const Eigen::Vector2f& pixel) {

    const Eigen::Vector2i px = pixel.cast <int> ();
    const float depthSample = depth[px.x() + depthSize.x()*px.y()];
    // Return on invalid depth measurement
    if (depthSample <=  0)
      return;

    // Compute the occupancy probability for the current measurement.
    const float diff = (pos.z() - depthSample);
    float sigma = se::math::clamp(noiseFactor * se::math::sq(pos.z()),
        2*voxelsize, 0.05f);
    float sample = H(diff/sigma, pos.z());
    if (sample == 0.5f)
      return;
    sample = se::math::clamp(sample, 0.03f, 0.97f);

    auto data = handler.get();

    // Update the occupancy probability
    const double delta_t = timestamp - data.y;
    data.x = applyWindow(data.x, SURF_BOUNDARY, delta_t, CAPITAL_T);
    data.x = se::math::clamp(updateLogs(data.x, sample), BOTTOM_CLAMP, TOP_CLAMP);
    data.y = timestamp;

    handler.set(data);
  }

};
#endif
