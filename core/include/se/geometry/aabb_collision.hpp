/*
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
#ifndef AABB_COLLISION_HPP
#define AABB_COLLISION_HPP
#include <cmath>
#include "../utils/math_utils.h"

namespace se {
namespace geometry {
  inline int axis_overlap(int a, const int a_edge, 
       int b, const int b_edge) {
    /* Half plane intersection test */
    a = a + (a_edge/2);
    b = b + (b_edge/2);
    return (std::abs(b - a) > (a_edge + b_edge)/2) ? 0 : 1;
  }

  inline int axis_overlap(float a, const float a_edge, 
       float b, const float b_edge) {
    /* Half plane intersection test */
    a = a + (a_edge/2);
    b = b + (b_edge/2);
    return (std::fabs(b - a) > (a_edge + b_edge)/2) ? 0 : 1;
  }

  inline int axis_contained(float a, const float a_edge, 
       float b, const float b_edge) {
    /* Segment a includes segment b */
    return (a < b) && ((a + a_edge) > (b + b_edge)); 
  }


  inline int aabb_aabb_collision(const Eigen::Vector3i a, const Eigen::Vector3i a_edge, 
      const Eigen::Vector3i b, const Eigen::Vector3i b_edge){

    return axis_overlap(a(0), a_edge(0), b(0), b_edge(0)) && 
           axis_overlap(a(1), a_edge(1), b(1), b_edge(1)) && 
           axis_overlap(a(2), a_edge(2), b(2), b_edge(2));
  }

  inline int aabb_aabb_inclusion(const Eigen::Vector3i a, const Eigen::Vector3i a_edge, 
      const Eigen::Vector3i b, const Eigen::Vector3i b_edge){
    /* Box a contains box b */
    return axis_contained(a(0), a_edge(0), b(0), b_edge(0)) && 
           axis_contained(a(1), a_edge(1), b(1), b_edge(1)) && 
           axis_contained(a(2), a_edge(2), b(2), b_edge(2));
  }
}
}
#endif
