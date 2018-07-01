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

#ifndef ALGO_MAPPING_HPP
#define ALGO_MAPPING_HPP
#include <node.hpp>

inline float3 voxelToPos(const int3 p, const float voxelSize){
  return make_float3(p.x * voxelSize, p.y * voxelSize, p.z * voxelSize);
}

namespace algorithms {

  template <typename T>
    void integratePass(se::VoxelBlock<T> ** blockList, unsigned int list_size, 
        const float * depth, uint2 depthSize, const float voxelSize, 
        const Matrix4 invTrack, const Matrix4 K, const float mu, 
        const float maxweight, const int current_frame) {

#pragma omp parallel for
      for(unsigned int i = 0; i < list_size; ++i){
        integrate(blockList[i], depth, depthSize, voxelSize,
            invTrack, K, mu, maxweight);
        blockList[i]->timestamp(current_frame);
      }
    }

  /*
   * MemoryBufferType is an instance of the memory allocator class
   */
  template <typename MemoryBufferType, typename UpdateFunctor>
    void integratePass(MemoryBufferType&  nodes_list, unsigned int list_size, 
        UpdateFunctor f) {

#pragma omp parallel for
      for(unsigned int i = 0; i < list_size; ++i){
        f(nodes_list[i]);
      }
    }
}
#endif
