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

#ifndef OCTREE_CONFIG_H
#define OCTREE_CONFIG_H

#include <cstdint>
#include "utils/math_utils.h"

#define BLOCK_SIDE 8
#define MAX_BITS 21
#define CAST_STACK_DEPTH 23
#define SCALE_MASK ((se::key_t)0x1FF)

namespace se {
typedef uint64_t key_t; 
//   typedef long long int morton_type; 
}

/*
 * Mask generated with:  
   MASK[0] = 0x7000000000000000,
   for(int i = 1; i < 21; ++i) {
   MASK[i] = MASK[i-1] | (MASK[0] >> (i*3));
   std::bitset<64> b(MASK[i]);
   std::cout << std::hex << b.to_ullong() << std::endl;
   }
 *
*/
constexpr uint64_t MASK[] = {
  0x7000000000000000,
  0x7e00000000000000,
  0x7fc0000000000000,
  0x7ff8000000000000,
  0x7fff000000000000,
  0x7fffe00000000000,
  0x7ffffc0000000000,
  0x7fffff8000000000,
  0x7ffffff000000000,
  0x7ffffffe00000000,
  0x7fffffffc0000000,
  0x7ffffffff8000000,
  0x7fffffffff000000,
  0x7fffffffffe00000,
  0x7ffffffffffc0000,
  0x7fffffffffff8000,
  0x7ffffffffffff000,
  0x7ffffffffffffe00,
  0x7fffffffffffffc0,
  0x7ffffffffffffff8,
  0x7fffffffffffffff
};

#endif
