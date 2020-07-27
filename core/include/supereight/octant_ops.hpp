/*
    Copyright 2016 Emanuele Vespa, Imperial College London
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef OCTANT_OPS_HPP
#define OCTANT_OPS_HPP
#include "octree_defines.h"
#include "utils/math_utils.h"
#include "utils/morton_utils.hpp"

#include <supereight/shared/commons.h>

#include <Eigen/Dense>
#include <bitset>
#include <iostream>

namespace se {
namespace keyops {

SE_DEVICE_FUNC
inline se::key_t code(const se::key_t key) { return key & ~SCALE_MASK; }

SE_DEVICE_FUNC
inline int level(const se::key_t key) { return key & SCALE_MASK; }

SE_DEVICE_ONLY_FUNC
inline se::key_t encode(const int x, const int y, const int z, const int level,
    const int max_depth) {
    const int offset = MAX_BITS - max_depth + level - 1;
    return (compute_morton(x, y, z) & MASK[offset] & ~SCALE_MASK) | level;
}

SE_DEVICE_FUNC
inline Eigen::Vector3i decode(const se::key_t key) {
    return unpack_morton(key & ~SCALE_MASK);
}
} // namespace keyops

/*
 * Algorithm 5 of p4est paper: https://epubs.siam.org/doi/abs/10.1137/100791634
 */
SE_DEVICE_FUNC
inline Eigen::Vector3i face_neighbour(const se::key_t o,
    const unsigned int face, const unsigned int l,
    const unsigned int max_depth) {
    Eigen::Vector3i coords  = se::keyops::decode(o);
    const unsigned int side = 1 << (max_depth - l);
    coords(0) = coords(0) + ((face == 0) ? -side : (face == 1) ? side : 0);
    coords(1) = coords(1) + ((face == 2) ? -side : (face == 3) ? side : 0);
    coords(2) = coords(2) + ((face == 4) ? -side : (face == 5) ? side : 0);
    return {coords(0), coords(1), coords(2)};
}

/*
 * \brief Return true if octant is a descendant of ancestor
 * \param octant
 * \param ancestor
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_ONLY_FUNC
inline bool descendant(
    se::key_t octant, se::key_t ancestor, const int max_depth) {
    const int level = se::keyops::level(ancestor);
    const int idx   = MAX_BITS - max_depth + level - 1;
    ancestor        = se::keyops::code(ancestor);
    octant          = se::keyops::code(octant) & MASK[idx];
    return (ancestor ^ octant) == 0;
}

/*
 * \brief Computes the parent's morton code of a given octant
 * \param octant
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_ONLY_FUNC
inline se::key_t parent(const se::key_t& octant, const int max_depth) {
    const int level = se::keyops::level(octant) - 1;
    const int idx   = MAX_BITS - max_depth + level - 1;
    return (octant & MASK[idx]) | level;
}

/*
 * \brief Computes the octants's id in its local brotherhood
 * \param octant
 * \param level of octant
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_FUNC
inline int child_id(se::key_t octant, const int level, const int max_depth) {
    int shift = max_depth - level;
    octant    = se::keyops::code(octant) >> shift * 3;
    int idx   = (octant & 0x01) | (octant & 0x02) | (octant & 0x04);
    return idx;
}

/*
 * \brief Computes the octants's corner which is not shared with its siblings
 * \param octant
 * \param level of octant
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_FUNC
inline Eigen::Vector3i far_corner(
    const se::key_t octant, const int level, const int max_depth) {
    const unsigned int side           = 1 << (max_depth - level);
    const int idx                     = child_id(octant, level, max_depth);
    const Eigen::Vector3i coordinates = se::keyops::decode(octant);
    return Eigen::Vector3i(coordinates(0) + (idx & 1) * side,
        coordinates(1) + ((idx & 2) >> 1) * side,
        coordinates(2) + ((idx & 4) >> 2) * side);
}

/*
 * \brief Computes the non-sibling neighbourhood around an octants. In the
 * special case in which the octant lies on an edge, neighbour are duplicated
 * as movement outside the enclosing cube is forbidden.
 * \param result 7-vector containing the neighbours
 * \param octant
 * \param level of octant
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_ONLY_FUNC
inline void exterior_neighbours(se::key_t result[7], const se::key_t octant,
    const int level, const int max_depth) {
    const int idx       = child_id(octant, level, max_depth);
    Eigen::Vector3i dir = Eigen::Vector3i(
        (idx & 1) ? 1 : -1, (idx & 2) ? 1 : -1, (idx & 4) ? 1 : -1);
    Eigen::Vector3i base = far_corner(octant, level, max_depth);
    dir(0) =
        se::math::in(base(0) + dir(0), 0, (1 << max_depth) - 1) ? dir(0) : 0;
    dir(1) =
        se::math::in(base(1) + dir(1), 0, (1 << max_depth) - 1) ? dir(1) : 0;
    dir(2) =
        se::math::in(base(2) + dir(2), 0, (1 << max_depth) - 1) ? dir(2) : 0;

    result[0] = se::keyops::encode(
        base(0) + dir(0), base(1) + 0, base(2) + 0, level, max_depth);
    result[1] = se::keyops::encode(
        base(0) + 0, base(1) + dir(1), base(2) + 0, level, max_depth);
    result[2] = se::keyops::encode(
        base(0) + dir(0), base(1) + dir(1), base(2) + 0, level, max_depth);
    result[3] = se::keyops::encode(
        base(0) + 0, base(1) + 0, base(2) + dir(2), level, max_depth);
    result[4] = se::keyops::encode(
        base(0) + dir(0), base(1) + 0, base(2) + dir(2), level, max_depth);
    result[5] = se::keyops::encode(
        base(0) + 0, base(1) + dir(1), base(2) + dir(2), level, max_depth);
    result[6] = se::keyops::encode(
        base(0) + dir(0), base(1) + dir(1), base(2) + dir(2), level, max_depth);
}

/*
 * \brief Computes the six face neighbours of an octant. These are stored in an
 * 4x6 matrix in which each column represents the homogeneous coordinates of a
 * neighbouring octant. The neighbours along the x axis come first, followed by
 * neighbours along the y axis and finally along the z axis. All coordinates are
 * clamped to be in the range between [0, max_size] where max size is given
 * by pow(2, max_depth).
 * \param res 4x6 matrix containing the neighbours
 * \param octant octant coordinates
 * \param level level of the octant
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_FUNC
static inline void one_neighbourhood(Eigen::Ref<Eigen::Matrix<int, 4, 6>> res,
    const Eigen::Vector3i& octant, const int level, const int max_depth) {
    const Eigen::Vector3i base = octant;
    const int size             = 1 << max_depth;
    const int step             = 1 << (max_depth - level);
    Eigen::Matrix<int, 4, 6> cross;
    res << -step, step, 0, 0, 0, 0, 0, 0, -step, step, 0, 0, 0, 0, 0, 0, -step,
        step, 0, 0, 0, 0, 0, 0;
    res.colwise() += base.homogeneous();
    res = res.unaryExpr(
        [size](const int a) { return std::max(std::min(a, size - 1), 0); });
}

/*
 * \brief Computes the six face neighbours of an octant. These are stored in an
 * 4x6 matrix in which each column represents the homogeneous coordinates of a
 * neighbouring octant. The neighbours along the x axis come first, followed by
 * neighbours along the y axis and finally along the z axis. All coordinates are
 * clamped to be in the range between [0, max_size] where max size is given
 * by pow(2, max_depth).
 * \param res 4x6 matrix containing the neighbours
 * \param octant octant key
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_FUNC
static inline void one_neighbourhood(Eigen::Ref<Eigen::Matrix<int, 4, 6>> res,
    const se::key_t octant, const int max_depth) {
    one_neighbourhood(
        res, se::keyops::decode(octant), se::keyops::level(octant), max_depth);
}

/*
 * \brief Computes the morton number of all siblings around an octant,
 * including itself.
 * \param result 8-vector containing the neighbours
 * \param octant
 * \param max_depth max depth of the tree on which the octant lives
 */
SE_DEVICE_ONLY_FUNC
inline void siblings(
    se::key_t result[8], const se::key_t octant, const int max_depth) {
    const int level   = (octant & SCALE_MASK);
    const int shift   = 3 * (max_depth - level);
    const se::key_t p = parent(octant, max_depth) + 1; // set-up next level
    for (int i = 0; i < 8; ++i) { result[i] = p | (i << shift); }
}
} // namespace se
#endif
