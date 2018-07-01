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

#include "se_common.h"

namespace se {
  namespace math {
    template <typename T>
      T fracf(const T& v) {
        return v - v.array().floor().matrix();
      }

    template <typename T>
      T floorf(const T& v) {
        return v.array().floor();
      }

    template <typename T>
    inline T fabs(const T& v) {
        return v.cwiseAbs();
      }

    template <typename Scalar>
      inline Scalar sq(Scalar a) {
        return a*a;
      };

    template <typename Scalar>
    inline bool in(const Scalar v, const Scalar a, 
          const Scalar b) {
        return v >= a && v <= b;
      }

    constexpr int log2_const(int n){
      return (n < 2 ? 0 : 1 + log2_const(n/2));
    }
  }
}

#endif
