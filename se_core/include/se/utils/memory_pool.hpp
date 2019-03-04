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

#ifndef MEM_POOL_H
#define MEM_POOL_H

#include <iostream>
#include <vector>
#include <atomic>
#include <mutex>

namespace se {
template <typename BlockType>
  class MemoryPool {
    public:
      MemoryPool(){
        current_block_ = 0;
        num_pages_ = 0;
        reserved_ = 0;
      }

      ~MemoryPool(){
        for(auto&& i : pages_){
          delete [] i;
        }
      }

      size_t size() const { return current_block_; };

      BlockType* operator[](const size_t i) const {
        const int page_idx = i / pagesize_;
        const int ptr_idx = i % pagesize_;
        return pages_[page_idx] + (ptr_idx);
      }

      void reserve(const size_t n){
        bool requires_realloc = (current_block_ + n) > reserved_;
        if(requires_realloc) expand(n);
      }

      BlockType * acquire_block(){
        // Fetch-add returns the value before increment
        int current = current_block_.fetch_add(1);
        const int page_idx = current / pagesize_;
        const int ptr_idx = current % pagesize_;
        BlockType * ptr = pages_[page_idx] + (ptr_idx);
        return ptr;
      }

    private:
      size_t reserved_;
      std::atomic<unsigned int> current_block_;
      const int pagesize_ = 1024; // # of blocks per page
      int num_pages_;
      std::vector<BlockType *> pages_;

      void expand(const size_t n){

        // std::cout << "Allocating " << n << " blocks" << std::endl;
        const int new_pages = std::ceil(n/pagesize_);
        for(int p = 0; p <= new_pages; ++p){
          pages_.push_back(new BlockType[pagesize_]);
          ++num_pages_;
          reserved_ += pagesize_;
        }
        // std::cout << "Reserved " << reserved_ << " blocks" << std::endl;
      }

      // Disabling copy-constructor
      MemoryPool(const MemoryPool& m);
  };
}
#endif
