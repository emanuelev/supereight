/*
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
 * */

#ifndef UNIQUE_HPP
#define UNIQUE_HPP
#include <se/octant_ops.hpp>

namespace se {
namespace algorithms {
  template <typename T>
    inline int unique(T* keys, int num_keys){
      int end = 1;
      if(num_keys < 2) return end;
      for (int i = 1; i < num_keys; ++i){
        if(keys[i] != keys[i-1]){
          keys[end] = keys[i];
          ++end;
        }
      }
      return end;
    }

  template <typename KeyT>
    inline int unique_multiscale(KeyT* keys, int num_keys,
      const KeyT level_mask){
      int end = 1;
      const KeyT key_mask = ~level_mask; 
      if(num_keys < 2) return end;
      KeyT prev_key = keys[0] & key_mask;
      KeyT prev_level = keys[0] & level_mask;
      for (int i = 1; i < num_keys; ++i){
        const KeyT key = keys[i] & key_mask;
        const KeyT level = keys[i] & level_mask;
        if(key != prev_key){
          keys[end] = keys[i];
          ++end;
        } else if(level > prev_level) { 
          /* end does not advance but previous entry is overwritten */
          keys[end-1] = keys[i];
        }
        prev_key = key;
        prev_level = level;
      }
      return end;
    }

  template <typename KeyT>
    inline int unique_multiscale(KeyT* keys, int num_keys,
        const KeyT , const unsigned current_level){
      int e = 0;
      for (int i = 1; i < num_keys; ++i){
        const KeyT level = se::keyops::level(keys[i]);
        if(level >= current_level) {
          if(se::keyops::code(keys[i]) != se::keyops::code(keys[e])){
            keys[++e] = keys[i];
          } else if(se::keyops::level(keys[i]) > se::keyops::level(keys[e])) { 
            /* end does not advance but previous entry is overwritten */
            keys[e] = keys[i];
          }
        }
      }
      return e + 1;
    }
}
}
#endif
