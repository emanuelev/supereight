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

/*! \brief Provide a generic way to get and set the data of both intermediate
 * (Node) and leaf (VoxelBlock) Octree nodes.
 */
template <typename SpecialisedHandlerT, typename NodeT>
class DataHandlerBase {
  typename NodeT::value_type get() {
    return static_cast<SpecialisedHandlerT *>(this)->get();
  }

  void set(const typename NodeT::value_type& val) {
    static_cast<SpecialisedHandlerT *>(this)->set(val);
  }
};

/*! \brief Get and set the data of a single voxel.
 */
template<typename FieldType>
class VoxelBlockHandler :
  DataHandlerBase<VoxelBlockHandler<FieldType>, se::VoxelBlock<FieldType> > {

public:
  /*!
   * \param[in] ptr Pointer to the VoxelBlock.
   * \param[in] v Coordinates of the voxel to get/set the data of. The elements
   * of v must be in the in the interval [0, BLOCK_SIDE-1].
   */
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

/*! \brief Get and set the data of a child of an intermediate (Node) Octree
 * node.
 */
template<typename FieldType>
class NodeHandler: DataHandlerBase<NodeHandler<FieldType>, se::Node<FieldType> > {
  public:
    /*!
     * \param[in] ptr Pointer to the Node.
     * \param[in] i Index of the child node to get/set the data of. i must be
     * in the in the interval [0, 7].
     */
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
