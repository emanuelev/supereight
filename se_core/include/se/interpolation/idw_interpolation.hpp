/*
    Copyright 2019 Emanuele Vespa, Imperial College London 
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

#ifndef IDW_INTERP_HPP
#define IDW_INTERP_HPP
#include "interp_gather.hpp"
#include "../octree_defines.h"
namespace se {
  namespace internal {
template <typename Precision, typename FieldType, 
         template<typename FieldT> class MapIndex,
         class FieldSelector>
static inline std::pair<Precision, bool>
    idw_interp(const MapIndex<FieldType>& map, const Eigen::Vector3f& pos, 
        FieldSelector select) {
      std::pair<Precision, Eigen::Vector3i> samples[8];

      Node<FieldType> * stack[CAST_STACK_DEPTH] = {};
      int max_depth = se::math::log2_const(map.size());
      auto base_ptr = se::internal::fetch(stack, map.root(), max_depth, 
          pos.cast<int>());
      std::cout << "base octant: \n" << se::keyops::decode(base_ptr->code_) << std::endl;

      for(int i = 1; i < 7; ++i) {
        samples[i] = se::internal::fetch_neighbour_sample<Precision>(stack, base_ptr, 
            max_depth, i, select);
        std::cout << "direction: " << i << std::endl;
        std::cout << "sample " << i << samples[i].first << std::endl;
        std::cout << "coords: \n" << samples[i].second << std::endl << std::endl;
      }
      return std::make_pair(samples[0].first, true);
    }
  }
}
#endif
