/*
 * Copyright 2016 Emanuele Vespa, Imperial College London 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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

#ifndef ACTIVE_LIST_HPP
#define ACTIVE_LIST_HPP

#include "../utils/math_utils.h" 
#include "../node.hpp"
#include "../utils/memory_pool.hpp"
#include "../utils/morton_utils.hpp"

namespace se {
namespace algorithms {

  template <typename VoxelBlockType>
    static inline bool in_frustum(const VoxelBlockType* v, float voxelSize, 
        const Eigen::Matrix4f& camera, const Eigen::Vector2i& frameSize) {

      const int side = VoxelBlockType::side;
      const static Eigen::Matrix<int, 4, 8> offsets = 
        (Eigen::Matrix<int, 4, 8>() << 0, side, 0   , side, 0   , side, 0   , side,
                                       0, 0   , side, side, 0   , 0   , side, side,
                                       0, 0   , 0   , 0   , side, side, side, side,
                                       0, 0   , 0   , 0   , 0   , 0   , 0   , 0   ).finished();

      Eigen::Matrix<float, 4, 8> v_camera =  
        camera *  
        Eigen::Vector4f(voxelSize, voxelSize, voxelSize, 1.f).asDiagonal() * 
         (offsets.colwise() + v->coordinates().homogeneous()).template cast<float>();
      v_camera.row(0).array() /= v_camera.row(2).array();
      v_camera.row(1).array() /= v_camera.row(2).array();
      return ((v_camera.row(0).array() >= 0.f && v_camera.row(0).array() < frameSize.x()) && 
       (v_camera.row(1).array() >= 0.f && v_camera.row(1).array() < frameSize.y())).any();
    }

  template <typename ValueType, typename P>
    bool satisfies(const ValueType& el, P predicate) {
      return predicate(el);
    }

  template <typename ValueType, typename P, typename... Ps>
    bool satisfies(const ValueType& el, P predicate, Ps... others) {
      return predicate(el) || satisfies(el, others...);
    }

#ifdef _OPENMP
  template <typename BlockType, typename... Predicates>
    void filter(std::vector<BlockType *>& out,
        const se::MemoryPool<BlockType>& block_array, Predicates... ps) {

      std::vector<BlockType *> temp;
      int num_elem = block_array.size();
      temp.resize(num_elem);

      int * thread_start = new int[omp_get_max_threads()];
      int * thread_end = new int[omp_get_max_threads()];
      int spawn_threads;
#pragma omp parallel
      {
        int threadid = omp_get_thread_num(); 
        int num_threads = omp_get_num_threads();
        int my_start = thread_start[threadid] = (threadid) * num_elem / num_threads;
        int my_end   = (threadid+1) * num_elem / num_threads;
        int count = 0;
        for(int i = my_start; i < my_end; ++i) {
          if(satisfies(block_array[i], ps...)){
            temp[my_start + count] = block_array[i];
            count++;
          }
        } 
        /* Store the actual end */
        thread_end[threadid] = count;
        if(threadid == 0) spawn_threads = num_threads;
      }
      
      int total = 0;
      for(int i = 0; i < spawn_threads; ++i) {
        total += thread_end[i];
      }
      out.resize(total);
      /* Copy the first */
      std::memcpy(out.data(), temp.data(), sizeof(BlockType *) * thread_end[0]);
      int copied = thread_end[0];
      /* Copy the rest */
      for(int i = 1; i < spawn_threads; ++i) {
        std::memcpy(out.data() + copied, 
            temp.data() + thread_start[i], sizeof(BlockType *) * thread_end[i]);
        copied += thread_end[i];
      }
    }

#else
  template <typename BlockType, typename... Predicates>
    void filter(std::vector<BlockType *>& out,
        const se::MemoryPool<BlockType>& block_array, Predicates... ps) {
      for(unsigned int i = 0; i < block_array.size(); ++i) {
        if(satisfies(block_array[i], ps...)){
          out.push_back(block_array[i]);
        }
      } 
    }
#endif
}
}
#endif
