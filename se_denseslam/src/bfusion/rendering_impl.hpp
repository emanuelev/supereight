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

#include <se/utils/math_utils.h>
#include <type_traits>

inline Eigen::Vector4f raycast(const Volume<OFusion> &volume,
                               const Eigen::Vector3f &origin,
                               const Eigen::Vector3f &direction,
                               const float tnear,
                               const float tfar,
                               const float,
                               const float step,
                               const float) {

  auto select_occupancy = [](const auto &val) { return val.x; };
  // march from camera away
  if (tnear < tfar) {
    float t = tnear; // closer bound to camera
    float stepsize = step;
    // occupancy prob in log2
    float f_t = volume.interp(origin + direction * t, select_occupancy);
    float f_tt = 0;
    // check if current pos is free
    if (f_t <= SURF_BOUNDARY) {
      for (; t < tfar; t += stepsize) {
        const Eigen::Vector3f pos = origin + direction * t;
        Volume<OFusion>::value_type data = volume.get(pos);
        // check if the voxel has been updated before
        if (data.x > -100.f && data.y > 0.f) {
          f_tt = volume.interp(origin + direction * t, select_occupancy);
        }
        // log2(p_occ)>0 , current pos is occupied
        if (f_tt > SURF_BOUNDARY)
          break;
        f_t = f_tt;
      }
      if (f_tt > SURF_BOUNDARY) {
        // got it, calculate accurate intersection
        t = t - stepsize * (f_tt - SURF_BOUNDARY) / (f_tt - f_t);
        Eigen::Vector4f res = (origin + direction * t).homogeneous();
        // normalized distance between camera and surface
        res.w() = t;
        return res;
      }
    }
  }
  return Eigen::Vector4f::Constant(0);
}
//
//inline bool raycastOcclusionVoxel(const Volume<OFusion>& volume,
//                                  const Eigen::Vector3f& origin,
//                                  const Eigen::Vector3f& direction,
//                                  const float            tnear,
//                                  const float            tfar,
//                                  const float,
//                                  const float            step,
//                                  const float,
//                                  std::unordered_set<uint64_t> &occlusion_voxel,
//                                  const float inverseVoxelSize) {
//
//  if (tnear < tfar) {
//    // t = starting distance after surface
//    float t = tnear+step;
//    float stepsize = step;
//
//    //march along the ray from surface away
//    for (; t < tfar; t += stepsize) {
//      const Eigen::Vector3f pos =  origin + direction * t;
//      // get info from map
//      Volume<OFusion>::value_type data = volume.get(pos);
//      // using log odds prob as -100 is a threshold for free space
//      // if valid add morton code to occluded voxel set
//      if (data.x > -100.f && data.y > 0.f) {
//        Eigen::Vector4f occl = (origin + direction * t).homogeneous();
////          std::cout << "occlusion "<< res << std::endl;
//        Eigen::Vector3i  occl_scaled = (inverseVoxelSize * occl.head<3>()).cast<int>();
//        occlusion_voxel.insert(compute_morton(occl_scaled.x(), occl_scaled.y(), occl_scaled.z()));
//
//      }
//
//    }
//
//  }
//  return true;
//}