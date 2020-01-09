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

#include <limits>
#include <image/image.hpp>
#include <random>
#include "gtest/gtest.h"

TEST(ImageTest, Constructor) {
  const int width  = 640;
  const int height = 480;
  const float init_val = std::numeric_limits<float>::lowest();
  se::Image<float> img(width, height, init_val);

  for(std::size_t i = 0; i < img.size(); ++i) {
    ASSERT_EQ(img[i], init_val);
  } 
}

TEST(ImageTest, Accessor) {
  const int width  = 640;
  const int height = 480;
  const float init_val = std::numeric_limits<float>::lowest();
  se::Image<float> img(width, height, init_val);

  std::random_device rd{};
  std::mt19937 gen{rd()};
  gen.seed(1);

  // Populate
  std::normal_distribution<float> dis { 0.f, 1.f };
  for(std::size_t i = 0; i < img.size(); ++i) {
    img[i] = dis(gen); 
  } 

  size_t i = 0;
  for(int y = 0; y < img.height(); ++y) {
    for(int x = 0; x < img.width(); ++x) {
      ASSERT_EQ(img[i++], img(x, y));
    }
  }
}
