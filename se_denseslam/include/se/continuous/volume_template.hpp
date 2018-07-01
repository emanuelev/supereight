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
#ifndef VOLUME_TEMPLATE_H
#define VOLUME_TEMPLATE_H

#include <iostream>
#include <memory>
#include <se/voxel_traits.hpp>
#include <se/utils/memory_pool.hpp>
#include <type_traits>
#include <cstring>

template <typename T>
class Void {};

/**
 * Continuous volume abstraction
 * Sparse, dynamically allocated storage accessed through the 
 * appropriate indexer (octree/hash table).
 * */ 
template <typename FieldType, template<typename> class DiscreteMapT> 
class VolumeTemplate {

  public:
    typedef voxel_traits<FieldType> traits_type;
    typedef typename traits_type::value_type value_type;
    typedef FieldType field_type;

    VolumeTemplate(){};
    VolumeTemplate(unsigned int s, float d, DiscreteMapT<FieldType>* m) :
      _map_index(m) {
        _size = s;
        _dim = d;
      };

    inline float3 pos(const uint3 & p) const {
      static const float voxelSize = _dim/_size;
      return make_float3(p.x * voxelSize, p.y * voxelSize, p.z * voxelSize);
    }

    void set(const uint3 & , const value_type& ) {}

    value_type operator[](const float3 & p) const {
      const float inverseVoxelSize = _size/_dim;
      const int3 scaled_pos = make_int3(make_float3((p.x * inverseVoxelSize),
          (p.y * inverseVoxelSize), (p.z * inverseVoxelSize)));
      return _map_index->get(scaled_pos.x, scaled_pos.y, scaled_pos.z);
    }

    value_type get(const float3 & p) const {
      const float inverseVoxelSize = _size/_dim;
      const int3 scaled_pos = make_int3(make_float3((p.x * inverseVoxelSize),
          (p.y * inverseVoxelSize), (p.z * inverseVoxelSize)));
      return _map_index->get_fine(scaled_pos.x, scaled_pos.y, scaled_pos.z);
    }

    value_type operator[](const uint3 p) const {
      return _map_index->get(p.x, p.y, p.z);
    }

    template <typename FieldSelector>
    float interp(const float3 & pos, FieldSelector select) const {
      const float inverseVoxelSize = _size / _dim;
      const Eigen::Vector3f scaled_pos((pos.x * inverseVoxelSize),
          (pos.y * inverseVoxelSize),
          (pos.z * inverseVoxelSize));
      return _map_index->interp(scaled_pos, select);
    }

    template <typename FieldSelector>
    Eigen::Vector3f grad(const float3 & pos, FieldSelector select) const {

      const float inverseVoxelSize = _size / _dim;
      const Eigen::Vector3f scaled_pos((pos.x * inverseVoxelSize),
          (pos.y * inverseVoxelSize),
          (pos.z * inverseVoxelSize));
      return _map_index->grad(scaled_pos, select);
    }

    unsigned int _size;
    float _dim;
    std::vector<se::key_t> _allocationList;
    DiscreteMapT<FieldType> * _map_index; 

  private:

    inline uint3 pos(const float3 & p) const {
      static const float inverseVoxelSize = _size/_dim;
      return make_uint3(p.x * inverseVoxelSize, p.y * inverseVoxelSize, 
          p.z * inverseVoxelSize);
    }
};
#endif
