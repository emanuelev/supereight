/*
 * Copyright 2019 Sotiris Papatheodorou, Imperial College London
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __VOXEL_ABSTRACTION_HPP
#define __VOXEL_ABSTRACTION_HPP

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace se {
  /*! \brief Allow abstracting voxels/VoxelBlocks/Nodes.
   *
   * This class allows handling voxels, VoxelBlocks and Nodes in a uniform
   * manner, as voxels with coordinates, dimensions and associated data.
   */
  template <typename T>
  class VoxelAbstration {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      typedef voxel_traits<T> traits_type;
      typedef typename traits_type::value_type value_type;
      value_type empty() const { return traits_type::empty(); }
      value_type initValue() const { return traits_type::initValue(); }

      /*! \brief The position of the VoxelAbstration's vertex that is closest
       * to the origin.
       *
       * It will have the smallest x, y and z coordinates among the
       * VoxelAbstration's 8 vertices.
       */
      Eigen::Vector3i pos_;

      /*! \brief The length of the VoxelAbstration's size in voxels.
       *
       * It allows determining its level in the octree.
       */
      unsigned int side_;

      /*! \brief The data stored in the Voxel.
       */
      value_type data_;

      /*! \brief The edge length in voxels of the Octree this VoxelAbstration
       * is part of.
       */
      int octree_size_;

      /*! \brief The single voxel edge length in map units of the Octree this
       * VoxelAbstration is part of.
       */
      float voxel_dim_;



      /*! \brief Initialize a VoxelAbstration for an invalid Octree.
       *
       * \warning If you use this constructor, the variables octree_size_ and
       * voxel_dim_ will have to be initialized manually for the
       * VoxelAbstration instance to be valid. Otherwise some of the class
       * methods will return invalid results.
       */
      VoxelAbstration()
          : octree_size_(0),
            voxel_dim_(0) {
        pos_         = Eigen::Vector3i::Zero();
        side_        = 0;
        data_        = initValue();
      }

      /*! \brief Initialize a VoxelAbstration for a specific Octree.
       *
       * \param[in] octree_size Number of voxels per map edge. Can be obtained
       * by calling Octree::size().
       * \param[in] octree_dim Dimension of the Octree in map units. Can be
       * obtained by calling Octree::dim().
       */
      VoxelAbstration(const int octree_size, const float octree_dim)
          : octree_size_(octree_size),
            voxel_dim_(octree_dim / octree_size) {
        pos_         = Eigen::Vector3i::Zero();
        side_        = 0;
        data_        = initValue();
      }

      /*! \brief Return the VoxelAbstration's center in map coordinates.
       */
      inline Eigen::Vector3f center() const {
        // Compute the voxel position in map coordinates.
        const Eigen::Vector3f pos_m = voxel_dim_ * pos_.cast<float>();
        // Compute the offset to the voxel center in map coordinates.
        const Eigen::Vector3f offset_m = Eigen::Vector3f::Constant(
            voxel_dim_ * side_ / 2.f);
        return pos_m + offset_m;
      }

      /*! \brief Return the Octree level the VoxelAbstration is located at.
       *
       * The root is at level zero and the individual voxels at level
       * log2(Octree::size()).
       */
      inline int level() const {
        return log2(octree_size_ / side_);
      }

      /*! \brief Return the VoxelAbstration's dimensions (edge length) in map
       * units.
       */
      inline float dim() const {
        return side_ * voxel_dim_;
      }

      /*! \brief Return the VoxelAbstration's volume in voxels^3.
       *
       * \return The VoxelAbstration's volume in voxels^3.
       */
      inline int volumeInVoxels() const {
        return side_ * side_ * side_;
      }

      /*! \brief Return the VoxelAbstration's volume in map units^3.
       */
      inline float volume() const {
        return std::pow(voxel_dim_, 3.f) * volumeInVoxels();
      }
  };



  /*! \brief Array containing VoxelAbstration elements.
   *
   * Used like this: se::VoxelAbstrationArray<value_type, 3>
   */
  template <typename T, size_t N>
  using VoxelAbstrationArray = std::array<VoxelAbstration<T>, N>;

  /*! \brief Vector containing VoxelAbstration elements.
   *
   * Used like this: se::VoxelAbstrationVector<value_type>
   */
  template <typename T>
  using VoxelAbstrationVector = std::vector<VoxelAbstration<T>,
      Eigen::aligned_allocator<VoxelAbstration<T> > >;
}

#endif

