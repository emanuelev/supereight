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
#ifndef SE_MATH_HELPER_H
#define SE_MATH_HELPER_H
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#define SOPHUS_DISABLE_ENSURES
/* 
 * When compiling in debug mode Eigen compilation fails 
 * due to -Wunused-parameter. Disable it if compiling with GCC.
 */
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#if __GNUC__ > 6
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#endif
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#pragma GCC diagnostic pop
#else
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#endif


namespace se {
  namespace math {
    template <typename T>
    static inline T fracf(const T& v) {
        return v - v.array().floor().matrix();
      }

    template <typename T>
    static inline T floorf(const T& v) {
        return v.array().floor();
      }

    template <typename T>
    static inline T fabs(const T& v) {
        return v.cwiseAbs();
      }

    template <typename Scalar>
    static inline Scalar sq(Scalar a) {
        return a*a;
      };

    template <typename Scalar>
    static inline bool in(const Scalar v, const Scalar a, 
          const Scalar b) {
        return v >= a && v <= b;
      }

    constexpr int log2_const(int n){
      return (n < 2 ? 0 : 1 + log2_const(n/2));
    }

    static inline Eigen::Matrix4f toMatrix4f(const Eigen::Vector3f& trans) {
      Eigen::Matrix4f se3_mat;  
      se3_mat << 1.f ,  0.f ,  0.f , trans.x(), 
              0.f ,  1.f ,  0.f , trans.y(), 
              0.f ,  0.f ,  1.f , trans.z(), 
              0.f ,  0.f ,  0.f ,  1.f;
      return se3_mat;
    }


    template <typename T>
      static inline typename std::enable_if<std::is_arithmetic<T>::value, T>::type
      clamp(const T& f, const T& a, const T& b) {
        return std::max(a, std::min(f, b));
      }

    static inline void clamp(Eigen::Ref<Eigen::VectorXf> res, const Eigen::Ref<const Eigen::VectorXf> a, 
        const Eigen::Ref<Eigen::VectorXf> b) {
      res = (res.array() < a.array()).select(a, res);
      res = (res.array() >= b.array()).select(b, res);
    } 

    template <typename R, typename A, typename B>
      static inline void clamp(Eigen::MatrixBase<R>& res, const Eigen::MatrixBase<A>& a, 
          const Eigen::MatrixBase<B>& b) {
        res = res.array().max(a.array());
        res = res.array().min(b.array());
      }
  }
}
#endif
