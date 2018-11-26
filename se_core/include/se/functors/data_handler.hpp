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
#ifndef DATA_HANDLER_HPP
#define DATA_HANDLER_HPP
#include "../utils/math_utils.h"
#include "../node.hpp"

template <typename SpecialisedHandlerT, typename NodeT>
class DataHandlerBase {
  typename NodeT::value_type get() {
    return static_cast<SpecialisedHandlerT *>(this)->get();
  } 
  void set(const typename NodeT::value_type& val) {
    static_cast<SpecialisedHandlerT *>(this)->set(val);
  }
};

template<typename FieldType>
class VoxelBlockHandler : 
  DataHandlerBase<VoxelBlockHandler<FieldType>, se::VoxelBlock<FieldType> > {

public:
  VoxelBlockHandler(se::VoxelBlock<FieldType>* ptr, Eigen::Vector3i v) : 
    _block(ptr), _voxel(v) {}

  typename se::VoxelBlock<FieldType>::value_type get() {
    return _block->data(_voxel);
  }

  void set(const typename se::VoxelBlock<FieldType>::value_type& val) {
    _block->data(_voxel, val);
  }

  private:
    se::VoxelBlock<FieldType> * _block;  
    Eigen::Vector3i _voxel;
};

template<typename FieldType>
class NodeHandler: DataHandlerBase<NodeHandler<FieldType>, se::Node<FieldType> > {
  public:
    NodeHandler(se::Node<FieldType>* ptr, int i) : _node(ptr), _idx(i) {}

    typename se::Node<FieldType>::value_type get() {
      return _node->value_[_idx];
    }

    void set(const typename se::Node<FieldType>::value_type& val) {
      _node->value_[_idx] = val;
    }

  private:
    se::Node<FieldType> * _node; 
    int _idx; 
};


#endif
