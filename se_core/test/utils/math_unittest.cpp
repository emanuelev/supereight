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

#include <se/utils/math_utils.h>
#include <Eigen/Core>
#include "gtest/gtest.h"

TEST(EigenUtils, ClampFixVec3) {
  Eigen::Vector3i base{0, 20, -1};
  Eigen::Vector3i min{0, 0, 0};
  Eigen::Vector3i max{10, 10, 10};
  se::math::clamp(base, min, max);

  ASSERT_TRUE(base.x() >= 0 && base.x() <= 10);
  ASSERT_TRUE(base.y() >= 0 && base.y() <= 10);
  ASSERT_TRUE(base.z() >= 0 && base.z() <= 10);
}

TEST(EigenUtils, ClampFixVec2) {
  Eigen::Vector2f base{-100.f, 34.f};
  Eigen::Vector2f min{0.f, 0.f};
  Eigen::Vector2f max{20.f, 10.f};
  se::math::clamp(base, min, max);

  ASSERT_TRUE(base.x() >= min.x() && base.x() <= max.x());
  ASSERT_TRUE(base.y() >= min.y() && base.y() <= max.y());
}
