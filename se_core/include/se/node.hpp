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

#ifndef NODE_H
#define NODE_H

#include <atomic>
#include <ctime>

#include <Eigen/Dense>

#include "io/se_serialise.hpp"
#include "octant_ops.hpp"
#include "octree_defines.h"
#include "se/volume_traits.hpp"
#include "utils/eigen_utils.h"
#include "utils/math_utils.h"
#include "utils/memory_pool.hpp"
#include "utils/morton_utils.hpp"
#include "voxel_traits.hpp"

namespace se {
/*! \brief A non-leaf node of the Octree. Each Node has 8 children.
 */
template <typename T>
class Node {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef voxel_traits<T> traits_type;
    typedef typename traits_type::value_type value_type;
    value_type empty() const { return traits_type::empty(); }
    value_type init_val() const { return traits_type::initValue(); }

    value_type value_[8];
    key_t code_;
    unsigned int side_;
    unsigned char children_mask_;
    unsigned int timestamp_;

    const Eigen::Vector3i child_offsets_[8] =
    {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
      {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};

    Node() {
      code_ = 0;
      side_ = 0;
      timestamp_ = -1;
      for (unsigned int i = 0; i < 8; i++) {
        value_[i]     = init_val();
        parent_ptr_ = NULL;
        child_ptr_[i] = nullptr;
      }
    }

    virtual ~Node(){};

    Node *& child(const int x, const int y, const int z) {
      return child_ptr_[x + y*2 + z*4];
    };

    Node *& child(const int offset){
      return child_ptr_[offset];
    }

    Node *& parent() {
      return parent_ptr_;
    }

    value_type data(const int offset)const{
      return value_[offset];
    }

    unsigned int timestamp() { return timestamp_; }
    unsigned int timestamp(unsigned int t) { return timestamp_ = t; }

    virtual bool isLeaf(){ return false; }
    /**
     *
     * @param offset = idx
     * @return bottom left corner coordinate of the node
     */
    Eigen::Vector3i childCoordinates(const int offset) const {
      // Get the Node coordinates from the Morton code and add the child
      // offset.
      return keyops::decode(code_) + (side_ >> 1) * child_offsets_[offset];
    }

    void occupancyUpdated(const bool o) { occupancyUpdated_ = o; }
    bool occupancyUpdated() { return occupancyUpdated_; }

  protected:
    Node *parent_ptr_;
    Node *child_ptr_[8];
  private:
    friend std::ofstream& internal::serialise <> (std::ofstream& out, Node& node);
    friend void internal::deserialise <> (Node& node, std::ifstream& in);
    bool occupancyUpdated_;
};

/*! \brief A leaf node of the Octree. Each VoxelBlock contains BLOCK_SIDE^3
 * voxels.
 */
template <typename T>
class VoxelBlock: public Node<T> {

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef voxel_traits<T> traits_type;
  // returns the voxel type struct
    typedef typename traits_type::value_type value_type;
    static constexpr unsigned int side = BLOCK_SIDE;
    static constexpr unsigned int sideSq = side*side;
    static constexpr unsigned int sideCb = side*side*side;

    static constexpr value_type empty() {
      return traits_type::empty();
    }

    static constexpr value_type initValue() {
      return traits_type::initValue();
    }

    VoxelBlock(){
      coordinates_ = Eigen::Vector3i::Constant(0);
      current_scale_ = 0;
      min_scale_ = -1;
      for (unsigned int i = 0; i < buff_size; i++)
        voxel_block_[i] = initValue();
    }

    bool isLeaf(){ return true; }

    Eigen::Vector3i coordinates() const { return coordinates_; }
    void coordinates(const Eigen::Vector3i& c){ coordinates_ = c; }
    //\brief Retrieves voxel value at coordinates (x,y,z)

    value_type data(const Eigen::Vector3i& pos) const;
    void data(const Eigen::Vector3i& pos, const value_type& value);

    value_type data(const Eigen::Vector3i& pos, const int level) const;
    void data(const Eigen::Vector3i& pos, const int level, const value_type& value);

    value_type data(const int i) const;
    void data(const int i, const value_type& value);

    int current_scale() { return current_scale_; }
    void current_scale(const int s) { current_scale_ = s; }

    int min_scale() { return min_scale_; }
    void min_scale(const int s) { min_scale_ = s; }

    void active(const bool a){ active_ = a; }
    bool active() const { return active_; }


    value_type * getBlockRawPtr(){ return voxel_block_; }
    static constexpr int size(){ return sizeof(VoxelBlock<T>); }

  private:
    VoxelBlock(const VoxelBlock&) = delete;
    Eigen::Vector3i coordinates_; // bottom left corner of the block
    int current_scale_;
    int min_scale_;
    static constexpr size_t compute_buff_size() {
      size_t size = 0;
      unsigned int s = side;
      while(s >= 1) {
        size += s * s * s;
        s = s >> 1;
      }
      return size;
    }
    static constexpr size_t buff_size = compute_buff_size();
    value_type voxel_block_[buff_size]; // Brick of data.
    bool active_;

    friend std::ofstream& internal::serialise <> (std::ofstream& out,
        VoxelBlock& node);
    friend std::ofstream& internal::serialiseMultilevel <> (std::ofstream& out,
    VoxelBlock& node);
    friend void internal::deserialise <> (VoxelBlock& node, std::ifstream& in);
    friend void internal::deserialiseMultilevel <> (VoxelBlock& node, std::ifstream& in);
};

template <typename T>
inline typename VoxelBlock<T>::value_type
VoxelBlock<T>::data(const Eigen::Vector3i& pos) const {
  Eigen::Vector3i offset = pos - coordinates_;
  const value_type& data = voxel_block_[offset(0) + offset(1)*side +
                                         offset(2)*sideSq];
  return data;
}

template <typename T>
inline typename VoxelBlock<T>::value_type
VoxelBlock<T>::data(const Eigen::Vector3i& pos, const int level) const {
  Eigen::Vector3i relative_pos = pos - coordinates_;
  int offset = 0;
  int l = 0;
  int num_voxels = sideCb;
  while(l < level) {
    offset += num_voxels;
    num_voxels /= 8;
    ++l;
  }
  const int local_size = side / (1 << level);
  relative_pos = relative_pos / (1 << level);
  return voxel_block_[offset + relative_pos.x() +
                      relative_pos.y()*local_size +
                      relative_pos.z()*se::math::sq(local_size)];
}

template <typename T>
inline void VoxelBlock<T>::data(const Eigen::Vector3i& pos,
                                const value_type &value){
  Eigen::Vector3i offset = pos - coordinates_;
  voxel_block_[offset(0) + offset(1)*side + offset(2)*sideSq] = value;
}

template <typename T>
inline void VoxelBlock<T>::data(const Eigen::Vector3i& pos, const int level,
                                const value_type &value){
  Eigen::Vector3i relative_pos = pos - coordinates_;
  int offset = 0;
  int l = 0;
  int num_voxels = sideCb;
  while(l < level) {
    offset += num_voxels;
    num_voxels /= 8;
    ++l;
  }

  const int local_size = side / (1 << level);
  relative_pos = relative_pos / (1 << level);
  voxel_block_[offset + relative_pos.x() +
               relative_pos.y()*local_size +
               relative_pos.z()*se::math::sq(local_size)] = value;
}

template <typename T>
inline typename VoxelBlock<T>::value_type
VoxelBlock<T>::data(const int i) const {
  const value_type& data = voxel_block_[i];
  return data;
}

template <typename T>
inline void VoxelBlock<T>::data(const int i, const value_type &value){
  voxel_block_[i] = value;
}


} //namespace se
#endif
