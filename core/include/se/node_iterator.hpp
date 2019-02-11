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

#ifndef NODE_ITERATOR_H
#define NODE_ITERATOR_H
#include "octree.hpp"
#include "Eigen/Dense"

namespace se {

template <typename T>
class node_iterator {

  public:

  node_iterator(const Octree<T>& m): map_(m){
    state_ = BRANCH_NODES;
    last = 0;
  };

  Node<T> *  next() {
    switch(state_) {
      case BRANCH_NODES:
        if(last < map_.nodes_buffer_.size()) {
          Node<T>* n = map_.nodes_buffer_[last++];
          return n;
        } else {
          last = 0;
          state_ = LEAF_NODES; 
          return next();
        }
        break;
      case LEAF_NODES:
        if(last < map_.block_buffer_.size()) {
          VoxelBlock<T>* n = map_.block_buffer_[last++];
          return n;
              /* the above int init required due to odr-use of static member */
        } else {
          last = 0;
          state_ = FINISHED; 
          return nullptr;
        }
        break;
      case FINISHED:
        return nullptr;
    }
  }

  private:
  typedef enum ITER_STATE {
    BRANCH_NODES,
    LEAF_NODES,
    FINISHED
  } ITER_STATE;

  const Octree<T>& map_;
  ITER_STATE state_;
  size_t last;
};
}
#endif
