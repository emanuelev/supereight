/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of
 Manchester. Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.


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

#include <supereight/backend/buffer_cuda.hpp>
#include <supereight/backend/fields.hpp>
#include <supereight/backend/memory_pool_cuda.hpp>
#include <supereight/octree.hpp>

#include <Eigen/Dense>

namespace se {

int buildAllocationList(BufferAccessorCUDA<se::key_t> allocation_list,
    const Octree<FieldType, MemoryPoolCUDA>& octree, int* voxel_count,
    const Eigen::Matrix4f& pose, const Eigen::Matrix4f& K,
    BufferAccessorCUDA<float> depth, const Eigen::Vector2i& frame_size,
    float mu);

void allocate(Octree<FieldType, MemoryPoolCUDA>& octree,
    BufferAccessorCUDA<se::key_t> allocation_list, int allocation_list_used,
    BufferCUDA<se::key_t>& keys_at_level, int* keys_at_level_used,
    BufferCUDA<std::uint8_t>& temp_storage);

} // namespace se
