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

#include "io/se_serialise.hpp"
#include "octree_defines.h"
#include "utils/math_utils.h"
#include "voxel_traits.hpp"

#include <supereight/shared/commons.h>

#include <atomic>
#include <time.h>

namespace se {
template<typename T>
class Node {
public:
    typedef voxel_traits<T> traits_type;
    typedef typename traits_type::value_type value_type;

    SE_DEVICE_FUNC
    value_type empty() const { return traits_type::empty(); }

    SE_DEVICE_FUNC
    value_type init_val() const { return traits_type::initValue(); }

    value_type value_[8];
    key_t code_;
    unsigned int side_;
    unsigned char children_mask_;

    SE_DEVICE_FUNC
    Node() {
        code_          = 0;
        side_          = 0;
        children_mask_ = 0;
        for (unsigned int i = 0; i < 8; i++) {
            value_[i]     = init_val();
            child_ptr_[i] = NULL;
        }
    }

    SE_DEVICE_FUNC
    virtual ~Node(){};

    SE_DEVICE_FUNC
    Node*& child(const int x, const int y, const int z) {
        return child_ptr_[x + y * 2 + z * 4];
    };

    SE_DEVICE_FUNC
    Node*& child(const int offset) { return child_ptr_[offset]; }

    SE_DEVICE_FUNC
    virtual bool isLeaf() { return false; }

protected:
    Node* child_ptr_[8];

private:
    friend std::ofstream& internal::serialise<>(std::ofstream& out, Node& node);
    friend void internal::deserialise<>(Node& node, std::ifstream& in);
};

template<typename T>
class VoxelBlock : public Node<T> {
public:
    typedef voxel_traits<T> traits_type;
    typedef typename traits_type::value_type value_type;
    static constexpr unsigned int side   = BLOCK_SIDE;
    static constexpr unsigned int sideSq = side * side;

    SE_DEVICE_FUNC
    static constexpr value_type empty() { return traits_type::empty(); }

    SE_DEVICE_FUNC
    static constexpr value_type initValue() { return traits_type::initValue(); }

    SE_DEVICE_FUNC
    VoxelBlock() {
        coordinates_ = Eigen::Vector3i::Constant(0);
        for (unsigned int i = 0; i < side * sideSq; i++)
            voxel_block_[i] = initValue();
    }

    SE_DEVICE_FUNC
    bool isLeaf() { return true; }

    SE_DEVICE_FUNC
    Eigen::Vector3i coordinates() const { return coordinates_; }

    SE_DEVICE_FUNC
    void coordinates(const Eigen::Vector3i& c) { coordinates_ = c; }

    SE_DEVICE_FUNC
    value_type data(const Eigen::Vector3i& pos) const;

    SE_DEVICE_FUNC
    void data(const Eigen::Vector3i& pos, const value_type& value);

    SE_DEVICE_FUNC
    value_type data(const int i) const;

    SE_DEVICE_FUNC
    void data(const int i, const value_type& value);

    SE_DEVICE_FUNC
    void active(const bool a) { active_ = a; }

    SE_DEVICE_FUNC
    bool active() const { return active_; }

    SE_DEVICE_FUNC
    value_type* getBlockRawPtr() { return voxel_block_; }

    SE_DEVICE_FUNC
    static constexpr int size() { return sizeof(VoxelBlock<T>); }

private:
    VoxelBlock(const VoxelBlock&) = delete;
    Eigen::Vector3i coordinates_;
    value_type voxel_block_[side * sideSq]; // Brick of data.
    bool active_;

    friend std::ofstream& internal::serialise<>(
        std::ofstream& out, VoxelBlock& node);
    friend void internal::deserialise<>(VoxelBlock& node, std::ifstream& in);
};

template<typename T>
inline typename VoxelBlock<T>::value_type VoxelBlock<T>::data(
    const Eigen::Vector3i& pos) const {
    Eigen::Vector3i offset = pos - coordinates_;
    const value_type& data =
        voxel_block_[offset(0) + offset(1) * side + offset(2) * sideSq];
    return data;
}

template<typename T>
inline void VoxelBlock<T>::data(
    const Eigen::Vector3i& pos, const value_type& value) {
    Eigen::Vector3i offset = pos - coordinates_;
    voxel_block_[offset(0) + offset(1) * side + offset(2) * sideSq] = value;
}

template<typename T>
inline typename VoxelBlock<T>::value_type VoxelBlock<T>::data(
    const int i) const {
    const value_type& data = voxel_block_[i];
    return data;
}

template<typename T>
inline void VoxelBlock<T>::data(const int i, const value_type& value) {
    voxel_block_[i] = value;
}
} // namespace se
#endif
