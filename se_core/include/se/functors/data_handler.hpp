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
#include "../octree.hpp"

template<typename SpecialisedHandlerT, typename NodeT>
class DataHandlerBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typename NodeT::value_type get() {
    return static_cast<SpecialisedHandlerT *>(this)->get();
  }
  void set(const typename NodeT::value_type &val) {
    static_cast<SpecialisedHandlerT *>(this)->set(val);
  }

  virtual Eigen::Vector3i getNodeCoordinates() {
  };

  virtual void occupancyUpdated(const bool o) {
  };

  virtual bool occupancyUpdated() {
  };
  virtual bool isFrontier() {
  };

  virtual key_t  get_mortoncode(){};

  virtual int get_childidx(){};

  virtual NodeT * get_node(){};
};

template<typename FieldType>
class VoxelBlockHandler : DataHandlerBase<VoxelBlockHandler<FieldType>,
                                          se::VoxelBlock<FieldType> > {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vec3i;

  VoxelBlockHandler(se::VoxelBlock<FieldType> *ptr, Eigen::Vector3i v) : _block(ptr), _voxel(v) {}

  typename se::VoxelBlock<FieldType>::value_type get() {
    return _block->data(_voxel);
  }

  void set(const typename se::VoxelBlock<FieldType>::value_type &val) {
    _block->data(_voxel, val);
  }

  Eigen::Vector3i getNodeCoordinates() {
    return _block->coordinates();
  }

  void occupancyUpdated(const bool o) {
    _block->occupancyUpdated(o);
  }

  bool occupancyUpdated() {
    return _block->occupancyUpdated();
  }

  bool isFrontier(const se::Octree<FieldType> &map) {
    vec3i face_neighbour_voxel(6);
    face_neighbour_voxel[0] << _voxel.x() - 1, _voxel.y(), _voxel.z();
    face_neighbour_voxel[1] << _voxel.x() + 1, _voxel.y(), _voxel.z();
    face_neighbour_voxel[2] << _voxel.x(), _voxel.y() - 1, _voxel.z();
    face_neighbour_voxel[3] << _voxel.x(), _voxel.y() + 1, _voxel.z();
    face_neighbour_voxel[4] << _voxel.x(), _voxel.y(), _voxel.z() - 1;
    face_neighbour_voxel[5] << _voxel.x(), _voxel.y(), _voxel.z() + 1;

    for (const auto &face_voxel : face_neighbour_voxel) {

//     if(map.get(face_voxel).st == voxel_state::kUnknown){
//       return true;
//     }
      // TODO change to octree allocation . currently fix grid

      // check if face voxel is inside same voxel block
      if (((_voxel.x() + 1) / BLOCK_SIDE) == ((face_voxel.x() + 1) / BLOCK_SIDE)
          && ((_voxel.y() + 1) / BLOCK_SIDE) == ((face_voxel.y() + 1) / BLOCK_SIDE)
          && ((_voxel.z() + 1) / BLOCK_SIDE) == ((face_voxel.z() + 1) / BLOCK_SIDE)) {
        // same voxel block
        if( map.get(face_voxel).st == voxel_state::kUnknown)
          return true;
      } else {
        // not same voxel block => check if neighbour is a voxel block
        se::Node<FieldType> *node = nullptr;
        bool is_voxel_block;
        map.fetch_octant(face_voxel(0), face_voxel(1), face_voxel(2), node, is_voxel_block);

        if (is_voxel_block) {
          // neighbour is a voxel block
          se::VoxelBlock<FieldType> *block = static_cast<se::VoxelBlock<FieldType> *> (node);
          if( block->data(face_voxel).st == voxel_state::kUnknown)
            return true;
//          return block->data(face_voxel).st == voxel_state::kUnknown;
        } else {
          // neighbour is a node
          if (map.get(se::keyops::decode(node->code_)).st == voxel_state::kUnknown)
            return true;
        }
      }
    }
    return false;
  }

  key_t  get_mortoncode(){
    return _block->code_;
  }
// only for nodes
  int get_childidx(){
    return -1;
  };

  se::VoxelBlock<FieldType> * get_node(){
    return _block;
  }
 private:
  se::VoxelBlock<FieldType> *_block;
  Eigen::Vector3i _voxel;
};

template<typename FieldType>
class NodeHandler : DataHandlerBase<NodeHandler<FieldType>, se::Node<FieldType> > {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NodeHandler(se::Node<FieldType> *ptr, int i) : _node(ptr), _idx(i) {}

  typename se::Node<FieldType>::value_type get() {
    return _node->value_[_idx];
  }

  void set(const typename se::Node<FieldType>::value_type &val) {
    _node->value_[_idx] = val;
  }

  Eigen::Vector3i getNodeCoordinates() {
    return Eigen::Vector3i(0, 0, 0);
  }

  void occupancyUpdated(const bool o) {
    _node->occupancyUpdated(o);
  }

  bool occupancyUpdated() {
    return _node->occupancyUpdated();
  }
  bool isFrontier(const se::Octree<FieldType> &map) {
    return false;
  }

  key_t  get_mortoncode(){
    return _node->code_;
  };

  int get_childidx(){
    return _idx;
  }

  se::Node<FieldType> * get_node(){
    return  _node;
  }

 private:
  se::Node<FieldType> *_node;
  int _idx;
};

#endif
