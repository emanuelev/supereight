/*
    Copyright (c) 2009-2011, NVIDIA Corporation
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
#ifndef SE_RAY_ITERATOR_HPP
#define SE_RAY_ITERATOR_HPP
#include "octree.hpp"
#include "Eigen/Dense"

/*****************************************************************************
 *
 *
 * Ray iterator implementation
 *
 * A modified version of the ray-caster introduced in the paper:
 * https://research.nvidia.com/publication/efficient-sparse-voxel-octrees
 *
 * Original code available at:
 * https://code.google.com/p/efficient-sparse-voxel-octrees/
 *
 * 
*****************************************************************************/

template <typename T>
class se::ray_iterator {

  public:
    ray_iterator(const Octree<T>& m, const Eigen::Vector3f& origin, 
        const Eigen::Vector3f& direction, float nearPlane, float farPlane) : map_(m) {

      pos_ = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
      idx_ = 0;
      parent_ = map_.root_;
      child_ = NULL;
      scale_exp2_ = 0.5f;
      scale_ = CAST_STACK_DEPTH-1;
      min_scale_ = CAST_STACK_DEPTH - log2(m.size_/Octree<T>::blockSide);
      static const float epsilon = exp2f(-log2(map_.size_));
      voxelSize_ = map_.dim_/map_.size_;
      state_ = INIT; 

      for(int i = 0 ; i < CAST_STACK_DEPTH; ++i)
        stack[i] = {0, 0, 0};

      direction_(0) = fabsf(direction(0)) < epsilon ? 
        copysignf(epsilon, direction(0)) : direction(0);
      direction_(1) = fabsf(direction(1)) < epsilon ? 
        copysignf(epsilon, direction(1)) : direction(1);
      direction_(2) = fabsf(direction(2)) < epsilon ? 
        copysignf(epsilon, direction(2)) : direction(2);

      /* Scaling the origin to resides between coordinates [1,2] */
      const Eigen::Vector3f scaled_origin = origin/map_.dim_ + 
        Eigen::Vector3f::Constant(1.f);

      /* Precomputing the ray coefficients */
      t_coef_ = -1.f * math::fabs(direction_).cwiseInverse();
      t_bias_ = t_coef_.cwiseProduct(scaled_origin);


      /* Build the octrant mask to to mirror the coordinate system such that
       * each ray component points in negative coordinates. The octree is 
       * assumed to reside at coordinates [1, 2]
       * 
       */
      octant_mask_ = 7;
      if(direction_(0) > 0.0f) octant_mask_ ^=1, t_bias_(0) = 3.0f * t_coef_(0) - t_bias_(0);
      if(direction_(1) > 0.0f) octant_mask_ ^=2, t_bias_(1) = 3.0f * t_coef_(1) - t_bias_(1);
      if(direction_(2) > 0.0f) octant_mask_ ^=4, t_bias_(2) = 3.0f * t_coef_(2) - t_bias_(2);

      /* Find the min-max t ranges. */
      t_min_ = fmaxf(
          fmaxf(2.0f * t_coef_(0) - t_bias_(0), 2.0f * t_coef_(1) - t_bias_(1)), 2.0f * t_coef_(2) - t_bias_(2));
      t_max_ = fminf(fminf(t_coef_(0) - t_bias_(0), t_coef_(1) - t_bias_(1)), t_coef_(2) - t_bias_(2));
      h_ = t_max_;
      t_min_ = t_min_init_ = fmaxf(t_min_, nearPlane/map_.dim_);
      t_max_ = t_max_init_ = fminf(t_max_, farPlane/map_.dim_ );

      /*
       * Initialise the ray position
       */
      if (1.5f * t_coef_(0) - t_bias_(0) > t_min_) idx_^= 1, pos_(0) = 1.5f;
      if (1.5f * t_coef_(1) - t_bias_(1) > t_min_) idx_^= 2, pos_(1) = 1.5f;
      if (1.5f * t_coef_(2) - t_bias_(2) > t_min_) idx_^= 4, pos_(2) = 1.5f;

    };

    /* 
     * Advance the ray.
     */
    inline void advance_ray() {

      int step_mask = 0;

      step_mask = (t_corner_(0) <= tc_max_) | 
        ((t_corner_(1) <= tc_max_) << 1) | ((t_corner_(2) <= tc_max_) << 2);
      pos_(0) -= scale_exp2_ * bool(step_mask & 1);
      pos_(1) -= scale_exp2_ * bool(step_mask & 2);
      pos_(2) -= scale_exp2_ * bool(step_mask & 4);

      t_min_ = tc_max_;
      idx_ ^= step_mask;

      // POP if bits flips disagree with ray direction

      if ((idx_ & step_mask) != 0) {

        // Get the different bits for each component.
        // This is done by xoring the bit patterns of the new and old pos
        // (float_as_int reinterprets a floating point number as int,
        // it is a sort of reinterpret_cast). This work because the volume has
        // been scaled between [1, 2]. Still digging why this is the case. 

        unsigned int differing_bits = 0;
        if ((step_mask & 1) != 0) differing_bits |= __float_as_int(pos_(0)) ^ __float_as_int(pos_(0) + scale_exp2_);
        if ((step_mask & 2) != 0) differing_bits |= __float_as_int(pos_(1)) ^ __float_as_int(pos_(1) + scale_exp2_);
        if ((step_mask & 4) != 0) differing_bits |= __float_as_int(pos_(2)) ^ __float_as_int(pos_(2) + scale_exp2_);

        // Get the scale at which the two differs. Here's there are different subtlelties related to how fp are stored.
        // MIND BLOWN: differing bit (i.e. the MSB) extracted using the 
        // exponent part of the fp representation. 
        scale_ = (__float_as_int((float)differing_bits) >> 23) - 127; // position of the highest bit
        scale_exp2_ = __int_as_float((scale_ - CAST_STACK_DEPTH + 127) << 23); // exp2f(scale - s_max)
        struct stack_entry&  e = stack[scale_];
        parent_ = e.parent;
        t_max_ = e.t_max;

        // Round cube position and extract child slot index.

        int shx = __float_as_int(pos_(0)) >> scale_;
        int shy = __float_as_int(pos_(1)) >> scale_;
        int shz = __float_as_int(pos_(2)) >> scale_;
        pos_(0) = __int_as_float(shx << scale_);
        pos_(1) = __int_as_float(shy << scale_);
        pos_(2) = __int_as_float(shz << scale_);
        idx_  = (shx & 1) | ((shy & 1) << 1) | ((shz & 1) << 2);

        h_ = 0.0f;
        child_ = NULL;

      }
    }

    /* 
     * Descend the hiararchy and compute the next child position.
     */
    inline void descend() {
      float tv_max = fminf(t_max_, tc_max_);
      float half = scale_exp2_ * 0.5f;
      Eigen::Vector3f t_center = half * t_coef_ + t_corner_;

      // Descend to the first child if the resulting t-span is non-empty.

      if (tc_max_ < h_) {
        stack[scale_] = {scale_, parent_, t_max_};
      }

      h_ = tc_max_;
      parent_ = child_;

      idx_ = 0;
      scale_--;
      scale_exp2_ = half;
      idx_ ^= (t_center(0) > t_min_) ? 1 : 0;
      idx_ ^= (t_center(1) > t_min_) ? 2 : 0;
      idx_ ^= (t_center(2) > t_min_) ? 4 : 0;

      pos_(0) += scale_exp2_ * bool(idx_ & 1);
      pos_(1) += scale_exp2_ * bool(idx_ & 2);
      pos_(2) += scale_exp2_ * bool(idx_ & 4);

      t_max_ = tv_max;
      child_ = NULL;
    }

    /*
     * Returns the next leaf along the ray direction.
     */

    VoxelBlock<T>* next() {

      if(state_ == ADVANCE) advance_ray();
      else if (state_ == FINISHED) return nullptr;

      while (scale_ < CAST_STACK_DEPTH) {
        t_corner_ = pos_.cwiseProduct(t_coef_) - t_bias_;
        tc_max_ = fminf(fminf(t_corner_(0), t_corner_(1)), t_corner_(2));

        child_ = parent_->child(idx_ ^ octant_mask_ ^ 7);

        if (scale_ == min_scale_ && child_ != NULL){
          state_ = ADVANCE;
          return static_cast<VoxelBlock<T> *>(child_); 
        } else if (child_ != NULL && t_min_ <= t_max_){  // If the child is valid, descend the tree hierarchy.
          descend();
          continue;
        }
        advance_ray();
      }
      return nullptr;
    }

    /*
     * \brief Returns the minimum distance in meters to be travelled along
     * the ray to intersect the voxel cube.
     */
    float tmin() { return t_min_init_ * map_.dim_; }

    /*
     * \brief Returns the minimum distance in meters to be travelled along
     * the ray to exit the voxel cube.
     */
    float tmax() { return t_max_init_ * map_.dim_; }

    /*
     * \brief Returns the minimum distance in meters to be travelled along
     * the ray to reach the currently intersected leaf.
     */
    float tcmin() { return t_min_ * map_.dim_; }

    /*
     * \brief Returns the minimum distance in meters to be travelled along
     * the ray to exit the currently intersected grid.
     */
    float tcmax() { return tc_max_ * map_.dim_; }

  private:
    struct stack_entry {
      int scale;
      Node<T> * parent;
      float t_max;
    };

    typedef enum STATE {
      INIT,
      ADVANCE,
      FINISHED
    } STATE;

    const Octree<T>& map_;
    float voxelSize_; 
    Eigen::Vector3f origin_;
    Eigen::Vector3f direction_;
    Eigen::Vector3f t_coef_;
    Eigen::Vector3f t_bias_;
    struct stack_entry stack[CAST_STACK_DEPTH];
    Node<T> * parent_;
    Node<T> * child_;
    int idx_;
    Eigen::Vector3f pos_;
    int scale;
    int min_scale_;
    float scale_exp2_;
    float t_min_;
    float t_min_init_;
    float t_max_;
    float t_max_init_;
    float tc_max_;
    Eigen::Vector3f t_corner_;
    float h_;
    int octant_mask_;
    int scale_;
    STATE state_;
};
#endif
