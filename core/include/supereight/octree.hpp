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

#pragma once

#include <algorithm>
#include <cstring>
#include <queue>

#if defined(_OPENMP) && !defined(__clang__)
#include <parallel/algorithm>
#endif

#include <supereight/algorithms/unique.hpp>
#include <supereight/geometry/aabb_collision.hpp>
#include <supereight/interpolation/interp_gather.hpp>

#include <supereight/node.hpp>
#include <supereight/octant_ops.hpp>
#include <supereight/octree_defines.h>

#include <supereight/utils/math_utils.h>
#include <supereight/utils/morton_utils.hpp>

#include <supereight/voxel_traits.hpp>

namespace se {

template<typename T, template<typename> class BufferT>
class ray_iterator;

template<typename T, template<typename> class BufferT>
class node_iterator;

template<typename T, template<typename> class BufferT>
class Octree {
public:
    using traits_type = voxel_traits<T>;
    using value_type  = typename traits_type::value_type;

    using node_type  = Node<T>;
    using block_type = VoxelBlock<T>;

    SE_DEVICE_FUNC
    value_type empty() const { return traits_type::empty(); }

    SE_DEVICE_FUNC
    value_type init_val() const { return traits_type::initValue(); }

    // Compile-time constant expressions
    // # of voxels per side in a voxel block
    static constexpr unsigned int blockSide = BLOCK_SIDE;
    // maximum tree depth in bits
    static constexpr unsigned int max_depth = ((sizeof(key_t) * 8) / 3);
    // Tree depth at which blocks are found
    static constexpr unsigned int block_depth =
        max_depth - math::log2_const(BLOCK_SIDE);

    SE_DEVICE_FUNC
    Octree(){};

    SE_DEVICE_FUNC
    ~Octree() {}

    /*! \brief Initialises the octree attributes
     * \param size number of voxels per side of the cube
     * \param dim cube extension per side, in meter
     */
    SE_DEVICE_FUNC
    void init(int size, float dim);

    SE_DEVICE_FUNC
    inline int size() const { return size_; }

    SE_DEVICE_FUNC
    inline float dim() const { return dim_; }

    SE_DEVICE_FUNC
    inline float voxelSize() const { return voxel_size_; }

    SE_DEVICE_FUNC
    inline float inverseVoxelSize() const { return inverse_voxel_size_; }

    SE_DEVICE_FUNC
    inline Node<T>* root() const { return root_; }

    /*! \brief Retrieves voxel value at coordinates (x,y,z), if not present it
     * allocates it. This method is not thread safe.
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     */
    SE_DEVICE_FUNC
    void set(const int x, const int y, const int z, const value_type val);

    /*! \brief Retrieves voxel value at coordinates (x,y,z)
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     */
    SE_DEVICE_FUNC
    value_type get(const int x, const int y, const int z) const;

    SE_DEVICE_FUNC
    value_type get_fine(const int x, const int y, const int z) const;

    /*! \brief Fetch the voxel block at which contains voxel  (x,y,z)
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     */
    SE_DEVICE_FUNC
    VoxelBlock<T>* fetch(const int x, const int y, const int z) const;

    /*! \brief Fetch the octant (x,y,z) at level depth
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     * \param depth maximum depth to be searched
     */
    SE_DEVICE_FUNC
    Node<T>* fetch_octant(
        const int x, const int y, const int z, const int depth) const;

    /*! \brief Insert the octant at (x,y,z). Not thread safe.
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     * \param depth target insertion level
     */
    SE_DEVICE_ONLY_FUNC
    Node<T>* insert(const int x, const int y, const int z, const int depth);

    /*! \brief Insert the octant (x,y,z) at maximum resolution. Not thread safe.
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     */
    SE_DEVICE_ONLY_FUNC
    VoxelBlock<T>* insert(const int x, const int y, const int z);

    /*! \brief Insert the octant (x,y,z). Allocates exactly one octant, requires
     * parents to be allocated. Thread safe.
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     */
    SE_DEVICE_ONLY_FUNC
    void insert_one(const key_t key, const int target_level);

    /*! \brief Interp voxel value at voxel position  (x,y,z)
     * \param pos three-dimensional coordinates in which each component belongs
     * to the interval [0, size]
     * \return signed distance function value at voxel position (x, y, z)
     */
    template<typename FieldSelect>
    SE_DEVICE_FUNC float interp(
        const Eigen::Vector3f& pos, FieldSelect f) const;

    /*! \brief Compute the gradient at voxel position  (x,y,z)
     * \param pos three-dimensional coordinates in which each component belongs
     * to the interval [0, size]
     * \return gradient at voxel position pos
     */
    SE_DEVICE_FUNC
    Eigen::Vector3f grad(const Eigen::Vector3f& pos) const;

    template<typename FieldSelect>
    SE_DEVICE_FUNC Eigen::Vector3f grad(
        const Eigen::Vector3f& pos, FieldSelect selector) const;

    /*! \brief Get the list of allocated block. If the active switch is set to
     * true then only the visible blocks are retrieved.
     * \param blocklist output vector of allocated blocks
     * \param active boolean switch. Set to true to retrieve visible, allocated
     * blocks, false to retrieve all allocated blocks.
     */
    void getBlockList(std::vector<VoxelBlock<T>*>& blocklist, bool active);

    SE_DEVICE_FUNC
    BufferT<VoxelBlock<T>>& getBlockBuffer() { return block_buffer_; };

    SE_DEVICE_FUNC
    BufferT<Node<T>>& getNodesBuffer() { return nodes_buffer_; };

    /*! \brief Computes the morton code of the block containing voxel
     * at coordinates (x,y,z)
     * \param x x coordinate in interval [0, size]
     * \param y y coordinate in interval [0, size]
     * \param z z coordinate in interval [0, size]
     */
    SE_DEVICE_ONLY_FUNC
    key_t hash(const int x, const int y, const int z) const {
        return keyops::encode(x, y, z, block_depth_, max_depth_);
    }

    SE_DEVICE_ONLY_FUNC
    key_t hash(const int x, const int y, const int z, key_t scale) const {
        return keyops::encode(x, y, z, scale, max_depth_);
    }

    SE_DEVICE_FUNC
    bool inBounds(const Eigen::Vector3f& pos) const {
        return pos.x() < size() && pos.y() < size() && pos.z() < size() &&
            pos.x() >= 0 && pos.y() >= 0 && pos.z() >= 0;
    }

    /*! \brief allocate a set of voxel blocks via their positional key
     * \param keys collection of voxel block keys to be allocated (i.e. their
     * morton number)
     * \param number of keys in the keys array
     */
    SE_DEVICE_ONLY_FUNC
    bool allocate(key_t* keys, int num_elem);

    SE_DEVICE_ONLY_FUNC
    void save(const std::string& filename);

    SE_DEVICE_ONLY_FUNC
    void load(const std::string& filename);

    SE_DEVICE_FUNC
    int maxDepth() const { return max_depth_; }

    SE_DEVICE_FUNC
    int blockDepth() const { return block_depth_; }

    /*! \brief Counts the number of blocks allocated
     * \return number of voxel blocks allocated
     */
    int leavesCount();

    /*! \brief Counts the number of internal nodes
     * \return number of internal nodes
     */
    int nodeCount();

    void printMemStats(){
        // memory.printStats();
    };

private:
    Node<T>* root_;

    int size_;
    float dim_;

    float voxel_size_;
    float inverse_voxel_size_;

    int max_depth_;
    int block_depth_;

    BufferT<VoxelBlock<T>> block_buffer_;
    BufferT<Node<T>> nodes_buffer_;

    friend class ray_iterator<T, BufferT>;
    friend class node_iterator<T, BufferT>;

    // Allocation specific variables
    key_t* keys_at_level_;
    int reserved_;

    // Private implementation of cached methods
    SE_DEVICE_FUNC
    value_type get(
        const int x, const int y, const int z, VoxelBlock<T>* cached) const;

    SE_DEVICE_FUNC
    value_type get(const Eigen::Vector3f& pos, VoxelBlock<T>* cached) const;

    // Parallel allocation of a given tree level for a set of input keys.
    // Pre: levels above target_level must have been already allocated
    bool allocate_level(key_t* keys, int num_tasks, int target_level);

    void reserveBuffers(const int n);

    // General helpers

    int leavesCountRecursive(Node<T>*);
    int nodeCountRecursive(Node<T>*);
    void getActiveBlockList(Node<T>*, std::vector<VoxelBlock<T>*>& blocklist);
    void getAllocatedBlockList(
        Node<T>*, std::vector<VoxelBlock<T>*>& blocklist);

    void deleteNode(Node<T>** node);
    void deallocateTree() { deleteNode(&root_); }
};

template<typename T, template<typename> class BufferT>
inline typename Octree<T, BufferT>::value_type Octree<T, BufferT>::get(
    const Eigen::Vector3f& p, VoxelBlock<T>* cached) const {
    const Eigen::Vector3i pos =
        (p.homogeneous() * Eigen::Vector4f::Constant(inverse_voxel_size_))
            .template head<3>()
            .template cast<int>();

    if (cached != NULL) {
        Eigen::Vector3i lower = cached->coordinates();
        Eigen::Vector3i upper =
            lower + Eigen::Vector3i::Constant(blockSide - 1);
        const int contained =
            ((pos.array() >= lower.array()) * (pos.array() <= upper.array()))
                .all();
        if (contained) { return cached->data(pos); }
    }

    Node<T>* n = root_;
    if (!n) { return empty(); }

    // Get the block.

    unsigned edge = size_ >> 1;
    for (; edge >= blockSide; edge = edge >> 1) {
        n = n->child(
            (pos(0) & edge) > 0, (pos(1) & edge) > 0, (pos(2) & edge) > 0);
        if (!n) { return empty(); }
    }

    // Get the element in the voxel block
    return static_cast<VoxelBlock<T>*>(n)->data(pos);
}

template<typename T, template<typename> class BufferT>
inline void Octree<T, BufferT>::set(
    const int x, const int y, const int z, const value_type val) {
    Node<T>* n = root_;
    if (!n) { return; }

    unsigned edge = size_ >> 1;
    for (; edge >= blockSide; edge = edge >> 1) {
        Node<T>* tmp = n->child((x & edge) > 0, (y & edge) > 0, (z & edge) > 0);
        if (!tmp) { return; }
        n = tmp;
    }

    static_cast<VoxelBlock<T>*>(n)->data(Eigen::Vector3i(x, y, z), val);
}

template<typename T, template<typename> class BufferT>
inline typename Octree<T, BufferT>::value_type Octree<T, BufferT>::get(
    const int x, const int y, const int z) const {
    Node<T>* n = root_;
    if (!n) { return init_val(); }

    unsigned edge = size_ >> 1;
    for (; edge >= blockSide; edge = edge >> 1) {
        const int childid =
            ((x & edge) > 0) + 2 * ((y & edge) > 0) + 4 * ((z & edge) > 0);
        Node<T>* tmp = n->child(childid);
        if (!tmp) { return n->value_[childid]; }
        n = tmp;
    }

    return static_cast<VoxelBlock<T>*>(n)->data(Eigen::Vector3i(x, y, z));
}

template<typename T, template<typename> class BufferT>
inline typename Octree<T, BufferT>::value_type Octree<T, BufferT>::get_fine(
    const int x, const int y, const int z) const {
    Node<T>* n = root_;
    if (!n) { return init_val(); }

    unsigned edge = size_ >> 1;
    for (; edge >= blockSide; edge = edge >> 1) {
        const int childid =
            ((x & edge) > 0) + 2 * ((y & edge) > 0) + 4 * ((z & edge) > 0);
        Node<T>* tmp = n->child(childid);
        if (!tmp) { return init_val(); }
        n = tmp;
    }

    return static_cast<VoxelBlock<T>*>(n)->data(Eigen::Vector3i(x, y, z));
}

template<typename T, template<typename> class BufferT>
inline typename Octree<T, BufferT>::value_type Octree<T, BufferT>::get(
    const int x, const int y, const int z, VoxelBlock<T>* cached) const {
    if (cached != NULL) {
        const Eigen::Vector3i pos   = Eigen::Vector3i(x, y, z);
        const Eigen::Vector3i lower = cached->coordinates();
        const Eigen::Vector3i upper =
            lower + Eigen::Vector3i::Constant(blockSide - 1);
        const int contained =
            ((pos.array() >= lower.array()) && (pos.array() <= upper.array()))
                .all();
        if (contained) { return cached->data(Eigen::Vector3i(x, y, z)); }
    }

    Node<T>* n = root_;
    if (!n) { return init_val(); }

    unsigned edge = size_ >> 1;
    for (; edge >= blockSide; edge = edge >> 1) {
        n = n->child((x & edge) > 0, (y & edge) > 0, (z & edge) > 0);
        if (!n) { return init_val(); }
    }

    return static_cast<VoxelBlock<T>*>(n)->data(Eigen::Vector3i(x, y, z));
}

template<typename T, template<typename> class BufferT>
void Octree<T, BufferT>::deleteNode(Node<T>** node) {
    if (*node) {
        for (int i = 0; i < 8; i++) {
            if ((*node)->child(i)) { deleteNode(&(*node)->child(i)); }
        }
        if (!(*node)->isLeaf()) {
            delete *node;
            *node = NULL;
        }
    }
}

template<typename T, template<typename> class BufferT>
void Octree<T, BufferT>::init(int size, float dim) {
    size_ = size;
    dim_  = dim;

    voxel_size_         = dim / size;
    inverse_voxel_size_ = size / dim;

    max_depth_   = log2(size);
    block_depth_ = max_depth_ - se::math::log2_const(blockSide);

    nodes_buffer_.reserve(1);

    root_        = nodes_buffer_.acquire();
    root_->side_ = size;

    reserved_ = 1024;

    keys_at_level_ = new key_t[reserved_];
    std::memset(keys_at_level_, 0, reserved_);
}

template<typename T, template<typename> class BufferT>
inline VoxelBlock<T>* Octree<T, BufferT>::fetch(
    const int x, const int y, const int z) const {
    Node<T>* n = root_;
    if (!n) { return NULL; }

    // Get the block.
    unsigned edge = size_ / 2;
    for (; edge >= blockSide; edge /= 2) {
        n = n->child((x & edge) > 0u, (y & edge) > 0u, (z & edge) > 0u);
        if (!n) { return NULL; }
    }
    return static_cast<VoxelBlock<T>*>(n);
}

template<typename T, template<typename> class BufferT>
inline Node<T>* Octree<T, BufferT>::fetch_octant(
    const int x, const int y, const int z, const int depth) const {
    Node<T>* n = root_;
    if (!n) { return NULL; }

    // Get the block.
    unsigned edge = size_ / 2;
    for (int d = 1; edge >= blockSide && d <= depth; edge /= 2, ++d) {
        n = n->child((x & edge) > 0u, (y & edge) > 0u, (z & edge) > 0u);
        if (!n) { return NULL; }
    }
    return n;
}

template<typename T, template<typename> class BufferT>
SE_DEVICE_ONLY_FUNC Node<T>* Octree<T, BufferT>::insert(
    const int x, const int y, const int z, const int depth) {
    // Make sure we have enough space on buffers

    /*
    if (depth >= block_depth_) {
        block_buffer_.reserve(block_buffer_.used() + 1);
        nodes_buffer_.reserve(nodes_buffer_.used() + block_depth_);
    } else {
        nodes_buffer_.reserve(nodes_buffer_.used() + depth);
    }
    */

    Node<T>* n = root_;
    // Should not happen if octree has been initialised properly
    if (!n) {
        root_        = nodes_buffer_.acquire();
        root_->code_ = 0;
        root_->side_ = size_;
        n            = root_;
    }

    key_t key                = keyops::encode(x, y, z, depth, max_depth_);
    const unsigned int shift = MAX_BITS - max_depth_ - 1;

    unsigned edge = size_ / 2;
    for (int d = 1; edge >= blockSide && d <= depth; edge /= 2, ++d) {
        const int childid =
            ((x & edge) > 0) + 2 * ((y & edge) > 0) + 4 * ((z & edge) > 0);

        // std::cout << "Level: " << d << std::endl;
        Node<T>* tmp = n->child(childid);
        if (!tmp) {
            const key_t prefix = keyops::code(key) & MASK[d + shift];
            if (edge == blockSide) {
                tmp = block_buffer_.acquire();
                static_cast<VoxelBlock<T>*>(tmp)->coordinates(
                    Eigen::Vector3i(unpack_morton(prefix)));
                static_cast<VoxelBlock<T>*>(tmp)->active(true);
                static_cast<VoxelBlock<T>*>(tmp)->code_ = prefix | d;
                n->children_mask_ = n->children_mask_ | (1 << childid);
            } else {
                tmp               = nodes_buffer_.acquire();
                tmp->code_        = prefix | d;
                tmp->side_        = edge;
                n->children_mask_ = n->children_mask_ | (1 << childid);
                // std::cout << "coords: "
                //   << keyops::decode(keyops::code(tmp->code_)) << std::endl;
            }
            n->child(childid) = tmp;
        }
        n = tmp;
    }
    return n;
}

template<typename T, template<typename> class BufferT>
SE_DEVICE_ONLY_FUNC VoxelBlock<T>* Octree<T, BufferT>::insert(
    const int x, const int y, const int z) {
    return static_cast<VoxelBlock<T>*>(insert(x, y, z, max_depth_));
}

template<typename T, template<typename> class BufferT>
template<typename FieldSelector>
float Octree<T, BufferT>::interp(
    const Eigen::Vector3f& pos, FieldSelector select) const {
    const Eigen::Vector3i base   = math::floorf(pos).cast<int>();
    const Eigen::Vector3f factor = math::fracf(pos);
    const Eigen::Vector3i lower  = base.cwiseMax(Eigen::Vector3i::Constant(0));

    float points[8];
    gather_points(*this, lower, select, points);

    return (
        ((points[0] * (1 - factor(0)) + points[1] * factor(0)) *
                (1 - factor(1)) +
            (points[2] * (1 - factor(0)) + points[3] * factor(0)) * factor(1)) *
            (1 - factor(2)) +
        ((points[4] * (1 - factor(0)) + points[5] * factor(0)) *
                (1 - factor(1)) +
            (points[6] * (1 - factor(0)) + points[7] * factor(0)) * factor(1)) *
            factor(2));
}

template<typename T, template<typename> class BufferT>
Eigen::Vector3f Octree<T, BufferT>::grad(const Eigen::Vector3f& pos) const {
    Eigen::Vector3i base   = Eigen::Vector3i(math::floorf(pos).cast<int>());
    Eigen::Vector3f factor = math::fracf(pos);
    Eigen::Vector3i lower_lower = (base - Eigen::Vector3i::Constant(1))
                                      .cwiseMax(Eigen::Vector3i::Constant(0));
    Eigen::Vector3i lower_upper = base.cwiseMax(Eigen::Vector3i::Constant(0));
    Eigen::Vector3i upper_lower =
        (base + Eigen::Vector3i::Constant(1))
            .cwiseMin(Eigen::Vector3i::Constant(size_) -
                Eigen::Vector3i::Constant(1));
    Eigen::Vector3i upper_upper =
        (base + Eigen::Vector3i::Constant(2))
            .cwiseMin(Eigen::Vector3i::Constant(size_) -
                Eigen::Vector3i::Constant(1));
    Eigen::Vector3i& lower = lower_upper;
    Eigen::Vector3i& upper = upper_lower;

    Eigen::Vector3f gradient;

    VoxelBlock<T>* n = fetch(base(0), base(1), base(2));
    gradient(0)      = (((get(upper_lower(0), lower(1), lower(2), n)(0) -
                        get(lower_lower(0), lower(1), lower(2), n)(0)) *
                           (1 - factor(0)) +
                       (get(upper_upper(0), lower(1), lower(2), n)(0) -
                           get(lower_upper(0), lower(1), lower(2), n)(0)) *
                           factor(0)) *
                          (1 - factor(1)) +
                      ((get(upper_lower(0), upper(1), lower(2), n)(0) -
                           get(lower_lower(0), upper(1), lower(2), n)(0)) *
                              (1 - factor(0)) +
                          (get(upper_upper(0), upper(1), lower(2), n)(0) -
                              get(lower_upper(0), upper(1), lower(2), n)(0)) *
                              factor(0)) *
                          factor(1)) *
            (1 - factor(2)) +
        (((get(upper_lower(0), lower(1), upper(2), n)(0) -
              get(lower_lower(0), lower(1), upper(2), n)(0)) *
                 (1 - factor(0)) +
             (get(upper_upper(0), lower(1), upper(2), n)(0) -
                 get(lower_upper(0), lower(1), upper(2), n)(0)) *
                 factor(0)) *
                (1 - factor(1)) +
            ((get(upper_lower(0), upper(1), upper(2), n)(0) -
                 get(lower_lower(0), upper(1), upper(2), n)(0)) *
                    (1 - factor(0)) +
                (get(upper_upper(0), upper(1), upper(2), n)(0) -
                    get(lower_upper(0), upper(1), upper(2), n)(0)) *
                    factor(0)) *
                factor(1)) *
            factor(2);

    gradient(1) = (((get(lower(0), upper_lower(1), lower(2), n)(0) -
                        get(lower(0), lower_lower(1), lower(2), n)(0)) *
                           (1 - factor(0)) +
                       (get(upper(0), upper_lower(1), lower(2), n)(0) -
                           get(upper(0), lower_lower(1), lower(2), n)(0)) *
                           factor(0)) *
                          (1 - factor(1)) +
                      ((get(lower(0), upper_upper(1), lower(2), n)(0) -
                           get(lower(0), lower_upper(1), lower(2), n)(0)) *
                              (1 - factor(0)) +
                          (get(upper(0), upper_upper(1), lower(2), n)(0) -
                              get(upper(0), lower_upper(1), lower(2), n)(0)) *
                              factor(0)) *
                          factor(1)) *
            (1 - factor(2)) +
        (((get(lower(0), upper_lower(1), upper(2), n)(0) -
              get(lower(0), lower_lower(1), upper(2), n)(0)) *
                 (1 - factor(0)) +
             (get(upper(0), upper_lower(1), upper(2), n)(0) -
                 get(upper(0), lower_lower(1), upper(2), n)(0)) *
                 factor(0)) *
                (1 - factor(1)) +
            ((get(lower(0), upper_upper(1), upper(2), n)(0) -
                 get(lower(0), lower_upper(1), upper(2), n)(0)) *
                    (1 - factor(0)) +
                (get(upper(0), upper_upper(1), upper(2), n)(0) -
                    get(upper(0), lower_upper(1), upper(2), n)(0)) *
                    factor(0)) *
                factor(1)) *
            factor(2);

    gradient(2) = (((get(lower(0), lower(1), upper_lower(2), n)(0) -
                        get(lower(0), lower(1), lower_lower(2), n)(0)) *
                           (1 - factor(0)) +
                       (get(upper(0), lower(1), upper_lower(2), n)(0) -
                           get(upper(0), lower(1), lower_lower(2), n)(0)) *
                           factor(0)) *
                          (1 - factor(1)) +
                      ((get(lower(0), upper(1), upper_lower(2), n)(0) -
                           get(lower(0), upper(1), lower_lower(2), n)(0)) *
                              (1 - factor(0)) +
                          (get(upper(0), upper(1), upper_lower(2), n)(0) -
                              get(upper(0), upper(1), lower_lower(2), n)(0)) *
                              factor(0)) *
                          factor(1)) *
            (1 - factor(2)) +
        (((get(lower(0), lower(1), upper_upper(2), n)(0) -
              get(lower(0), lower(1), lower_upper(2), n)(0)) *
                 (1 - factor(0)) +
             (get(upper(0), lower(1), upper_upper(2), n)(0) -
                 get(upper(0), lower(1), lower_upper(2), n)(0)) *
                 factor(0)) *
                (1 - factor(1)) +
            ((get(lower(0), upper(1), upper_upper(2), n)(0) -
                 get(lower(0), upper(1), lower_upper(2), n)(0)) *
                    (1 - factor(0)) +
                (get(upper(0), upper(1), upper_upper(2), n)(0) -
                    get(upper(0), upper(1), lower_upper(2), n)(0)) *
                    factor(0)) *
                factor(1)) *
            factor(2);

    return (0.5f * voxel_size_) * gradient;
}

template<typename T, template<typename> class BufferT>
template<typename FieldSelector>
Eigen::Vector3f Octree<T, BufferT>::grad(
    const Eigen::Vector3f& pos, FieldSelector select) const {
    Eigen::Vector3i base   = Eigen::Vector3i(math::floorf(pos).cast<int>());
    Eigen::Vector3f factor = math::fracf(pos);
    Eigen::Vector3i lower_lower = (base - Eigen::Vector3i::Constant(1))
                                      .cwiseMax(Eigen::Vector3i::Constant(0));
    Eigen::Vector3i lower_upper = base.cwiseMax(Eigen::Vector3i::Constant(0));
    Eigen::Vector3i upper_lower =
        (base + Eigen::Vector3i::Constant(1))
            .cwiseMin(Eigen::Vector3i::Constant(size_) -
                Eigen::Vector3i::Constant(1));
    Eigen::Vector3i upper_upper =
        (base + Eigen::Vector3i::Constant(2))
            .cwiseMin(Eigen::Vector3i::Constant(size_) -
                Eigen::Vector3i::Constant(1));
    Eigen::Vector3i& lower = lower_upper;
    Eigen::Vector3i& upper = upper_lower;

    Eigen::Vector3f gradient;

    VoxelBlock<T>* n = fetch(base(0), base(1), base(2));
    gradient(0) =
        (((select(get(upper_lower(0), lower(1), lower(2), n)) -
              select(get(lower_lower(0), lower(1), lower(2), n))) *
                 (1 - factor(0)) +
             (select(get(upper_upper(0), lower(1), lower(2), n)) -
                 select(get(lower_upper(0), lower(1), lower(2), n))) *
                 factor(0)) *
                (1 - factor(1)) +
            ((select(get(upper_lower(0), upper(1), lower(2), n)) -
                 select(get(lower_lower(0), upper(1), lower(2), n))) *
                    (1 - factor(0)) +
                (select(get(upper_upper(0), upper(1), lower(2), n)) -
                    select(get(lower_upper(0), upper(1), lower(2), n))) *
                    factor(0)) *
                factor(1)) *
            (1 - factor(2)) +
        (((select(get(upper_lower(0), lower(1), upper(2), n)) -
              select(get(lower_lower(0), lower(1), upper(2), n))) *
                 (1 - factor(0)) +
             (select(get(upper_upper(0), lower(1), upper(2), n)) -
                 select(get(lower_upper(0), lower(1), upper(2), n))) *
                 factor(0)) *
                (1 - factor(1)) +
            ((select(get(upper_lower(0), upper(1), upper(2), n)) -
                 select(get(lower_lower(0), upper(1), upper(2), n))) *
                    (1 - factor(0)) +
                (select(get(upper_upper(0), upper(1), upper(2), n)) -
                    select(get(lower_upper(0), upper(1), upper(2), n))) *
                    factor(0)) *
                factor(1)) *
            factor(2);

    gradient(1) =
        (((select(get(lower(0), upper_lower(1), lower(2), n)) -
              select(get(lower(0), lower_lower(1), lower(2), n))) *
                 (1 - factor(0)) +
             (select(get(upper(0), upper_lower(1), lower(2), n)) -
                 select(get(upper(0), lower_lower(1), lower(2), n))) *
                 factor(0)) *
                (1 - factor(1)) +
            ((select(get(lower(0), upper_upper(1), lower(2), n)) -
                 select(get(lower(0), lower_upper(1), lower(2), n))) *
                    (1 - factor(0)) +
                (select(get(upper(0), upper_upper(1), lower(2), n)) -
                    select(get(upper(0), lower_upper(1), lower(2), n))) *
                    factor(0)) *
                factor(1)) *
            (1 - factor(2)) +
        (((select(get(lower(0), upper_lower(1), upper(2), n)) -
              select(get(lower(0), lower_lower(1), upper(2), n))) *
                 (1 - factor(0)) +
             (select(get(upper(0), upper_lower(1), upper(2), n)) -
                 select(get(upper(0), lower_lower(1), upper(2), n))) *
                 factor(0)) *
                (1 - factor(1)) +
            ((select(get(lower(0), upper_upper(1), upper(2), n)) -
                 select(get(lower(0), lower_upper(1), upper(2), n))) *
                    (1 - factor(0)) +
                (select(get(upper(0), upper_upper(1), upper(2), n)) -
                    select(get(upper(0), lower_upper(1), upper(2), n))) *
                    factor(0)) *
                factor(1)) *
            factor(2);

    gradient(2) =
        (((select(get(lower(0), lower(1), upper_lower(2), n)) -
              select(get(lower(0), lower(1), lower_lower(2), n))) *
                 (1 - factor(0)) +
             (select(get(upper(0), lower(1), upper_lower(2), n)) -
                 select(get(upper(0), lower(1), lower_lower(2), n))) *
                 factor(0)) *
                (1 - factor(1)) +
            ((select(get(lower(0), upper(1), upper_lower(2), n)) -
                 select(get(lower(0), upper(1), lower_lower(2), n))) *
                    (1 - factor(0)) +
                (select(get(upper(0), upper(1), upper_lower(2), n)) -
                    select(get(upper(0), upper(1), lower_lower(2), n))) *
                    factor(0)) *
                factor(1)) *
            (1 - factor(2)) +
        (((select(get(lower(0), lower(1), upper_upper(2), n)) -
              select(get(lower(0), lower(1), lower_upper(2), n))) *
                 (1 - factor(0)) +
             (select(get(upper(0), lower(1), upper_upper(2), n)) -
                 select(get(upper(0), lower(1), lower_upper(2), n))) *
                 factor(0)) *
                (1 - factor(1)) +
            ((select(get(lower(0), upper(1), upper_upper(2), n)) -
                 select(get(lower(0), upper(1), lower_upper(2), n))) *
                    (1 - factor(0)) +
                (select(get(upper(0), upper(1), upper_upper(2), n)) -
                    select(get(upper(0), upper(1), lower_upper(2), n))) *
                    factor(0)) *
                factor(1)) *
            factor(2);

    return (0.5f * voxel_size_) * gradient;
}

template<typename T, template<typename> class BufferT>
int Octree<T, BufferT>::leavesCount() {
    return leavesCountRecursive(root_);
}

template<typename T, template<typename> class BufferT>
int Octree<T, BufferT>::leavesCountRecursive(Node<T>* n) {
    if (!n) return 0;

    if (n->isLeaf()) { return 1; }

    int sum = 0;

    for (int i = 0; i < 8; i++) { sum += leavesCountRecursive(n->child(i)); }

    return sum;
}

template<typename T, template<typename> class BufferT>
int Octree<T, BufferT>::nodeCount() {
    return nodeCountRecursive(root_);
}

template<typename T, template<typename> class BufferT>
int Octree<T, BufferT>::nodeCountRecursive(Node<T>* node) {
    if (!node) { return 0; }

    int n = 1;
    for (int i = 0; i < 8; ++i) {
        n += (n ? nodeCountRecursive((node)->child(i)) : 0);
    }
    return n;
}

#include <bitset>

template<typename T, template<typename> class BufferT>
void Octree<T, BufferT>::reserveBuffers(const int n) {
    if (n > reserved_) {
        // std::cout << "Reserving " << n << " entries in allocation buffers" <<
        // std::endl;
        delete[] keys_at_level_;
        keys_at_level_ = new key_t[n];
        reserved_      = n;
    }
    block_buffer_.reserve(block_buffer_.used() + n);
}

template<typename T, template<typename> class BufferT>
SE_DEVICE_ONLY_FUNC bool Octree<T, BufferT>::allocate(
    key_t* keys, int num_elem) {
#if defined(_OPENMP) && !defined(__clang__)
    __gnu_parallel::sort(keys, keys + num_elem);
#else
    std::sort(keys, keys + num_elem);
#endif

    /*
    key_t* unique_keys = new key_t[num_elem];
    std::memcpy(unique_keys, keys, num_elem * sizeof(key_t));
    key_t* end = std::unique(unique_keys, unique_keys + num_elem);
    std::cout << "num_elem: " << num_elem
              << ", num unique: " << end - unique_keys << "\n";
    */

    num_elem = algorithms::filter_ancestors(keys, num_elem, max_depth_);
    reserveBuffers(num_elem);

    int last_elem = 0;
    bool success  = false;

    const unsigned int shift = MAX_BITS - max_depth_ - 1;
    for (int level = 1; level <= block_depth_; level++) {
        const key_t mask = MASK[level + shift] | SCALE_MASK;
        compute_prefix(keys, keys_at_level_, num_elem, mask);
        last_elem = algorithms::unique_multiscale(keys_at_level_, num_elem);
        // std::cout << "unique at level " << level << ": " << last_elem <<
        // "\n";
        success = allocate_level(keys_at_level_, last_elem, level);
    }
    return success;
}

template<typename T, template<typename> class BufferT>
bool Octree<T, BufferT>::allocate_level(
    key_t* keys, int num_tasks, int target_level) {
    nodes_buffer_.reserve(nodes_buffer_.used() + num_tasks);

    // #pragma omp parallel for
    for (int i = 0; i < num_tasks; i++) {
        Node<T>** n = &root_;
        key_t myKey = keyops::code(keys[i]);
        int myLevel = keyops::level(keys[i]);
        if (myLevel < target_level) continue;

        int edge = size_ / 2;
        for (int level = 1; level <= target_level; ++level) {
            int index       = child_id(myKey, level, max_depth_);
            Node<T>* parent = *n;
            n               = &(*n)->child(index);

            if (!(*n)) {
                if (level == block_depth_) {
                    *n          = block_buffer_.acquire();
                    (*n)->side_ = edge;
                    static_cast<VoxelBlock<T>*>(*n)->coordinates(
                        Eigen::Vector3i(unpack_morton(myKey)));
                    static_cast<VoxelBlock<T>*>(*n)->active(true);
                    static_cast<VoxelBlock<T>*>(*n)->code_ = myKey | level;
                    parent->children_mask_ =
                        parent->children_mask_ | (1 << index);
                } else {
                    *n          = nodes_buffer_.acquire();
                    (*n)->code_ = myKey | level;
                    (*n)->side_ = edge;
                    parent->children_mask_ =
                        parent->children_mask_ | (1 << index);
                }
            }
            edge /= 2;
        }
    }
    return true;
}

template<typename T, template<typename> class BufferT>
SE_DEVICE_ONLY_FUNC void Octree<T, BufferT>::insert_one(
    const key_t key, const int target_level) {
    Node<T>** node = &root_;

    key_t code = keyops::code(key);
    int edge   = size_ / 2;

    for (int level = 1; level <= target_level - 1; ++level) {
        int index = child_id(code, level, max_depth_);
        node      = &(*node)->child(index);

        edge /= 2;
    }

    int index       = child_id(code, target_level, max_depth_);
    Node<T>* parent = *node;
    node            = &(*node)->child(index);

    // Already allocated?!
    if (*node) return;

    if (target_level == block_depth_) {
        *node       = block_buffer_.acquire();
        auto* block = static_cast<VoxelBlock<T>*>(*node);

        block->coordinates(Eigen::Vector3i(unpack_morton(code)));
        block->active(true);
    } else {
        *node = nodes_buffer_.acquire();
    }

    (*node)->code_ = code | target_level;
    (*node)->side_ = edge;

    parent->children_mask_ = parent->children_mask_ | (1 << index);
}

template<typename T, template<typename> class BufferT>
void Octree<T, BufferT>::getBlockList(
    std::vector<VoxelBlock<T>*>& blocklist, bool active) {
    Node<T>* n = root_;
    if (!n) return;
    if (active)
        getActiveBlockList(n, blocklist);
    else
        getAllocatedBlockList(n, blocklist);
}

template<typename T, template<typename> class BufferT>
void Octree<T, BufferT>::getActiveBlockList(
    Node<T>* n, std::vector<VoxelBlock<T>*>& blocklist) {
    using tNode = Node<T>;
    if (!n) return;
    std::queue<tNode*> q;
    q.push(n);
    while (!q.empty()) {
        tNode* node = q.front();
        q.pop();

        if (node->isLeaf()) {
            VoxelBlock<T>* block = static_cast<VoxelBlock<T>*>(node);
            if (block->active()) blocklist.push_back(block);
            continue;
        }

        for (int i = 0; i < 8; ++i) {
            if (node->child(i)) q.push(node->child(i));
        }
    }
}

template<typename T, template<typename> class BufferT>
void Octree<T, BufferT>::getAllocatedBlockList(
    Node<T>*, std::vector<VoxelBlock<T>*>& blocklist) {
    for (int i = 0; i < block_buffer_.used(); ++i) {
        blocklist.push_back(block_buffer_[i]);
    }
}

template<typename T, template<typename> class BufferT>
SE_DEVICE_ONLY_FUNC void Octree<T, BufferT>::save(const std::string& filename) {
    std::ofstream os(filename, std::ios::binary);
    os.write(reinterpret_cast<char*>(&size_), sizeof(size_));
    os.write(reinterpret_cast<char*>(&dim_), sizeof(dim_));

    size_t n = nodes_buffer_.used();
    os.write(reinterpret_cast<char*>(&n), sizeof(size_t));
    for (size_t i = 0; i < n; ++i) internal::serialise(os, *nodes_buffer_[i]);

    n = block_buffer_.used();
    os.write(reinterpret_cast<char*>(&n), sizeof(size_t));
    for (size_t i = 0; i < n; ++i) internal::serialise(os, *block_buffer_[i]);
}

template<typename T, template<typename> class BufferT>
SE_DEVICE_ONLY_FUNC void Octree<T, BufferT>::load(const std::string& filename) {
    std::cout << "Loading octree from disk... " << filename << std::endl;
    std::ifstream is(filename, std::ios::binary);
    int size;
    float dim;
    const int side       = se::VoxelBlock<T>::side;
    const int side_cubed = side * side * side;

    is.read(reinterpret_cast<char*>(&size), sizeof(size));
    is.read(reinterpret_cast<char*>(&dim), sizeof(dim));

    init(size, dim);

    size_t n = 0;
    is.read(reinterpret_cast<char*>(&n), sizeof(size_t));
    nodes_buffer_.reserve(nodes_buffer_.used() + n);
    std::cout << "Reading " << n << " nodes " << std::endl;
    for (size_t i = 0; i < n; ++i) {
        Node<T> tmp;
        internal::deserialise(tmp, is);
        Eigen::Vector3i coords = keyops::decode(tmp.code_);
        Node<T>* n =
            insert(coords(0), coords(1), coords(2), keyops::level(tmp.code_));
        std::memcpy(n->value_, tmp.value_, sizeof(tmp.value_));
    }

    is.read(reinterpret_cast<char*>(&n), sizeof(size_t));
    std::cout << "Reading " << n << " blocks " << std::endl;
    for (size_t i = 0; i < n; ++i) {
        VoxelBlock<T> tmp;
        internal::deserialise(tmp, is);
        Eigen::Vector3i coords = tmp.coordinates();
        VoxelBlock<T>* n       = static_cast<VoxelBlock<T>*>(
            insert(coords(0), coords(1), coords(2), keyops::level(tmp.code_)));
        std::memcpy(n->getBlockRawPtr(), tmp.getBlockRawPtr(),
            side_cubed * sizeof(*(tmp.getBlockRawPtr())));
    }
}
} // namespace se
