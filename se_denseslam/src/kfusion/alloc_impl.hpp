/*
 *
 * Copyright 2016 Emanuele Vespa, Imperial College London 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 * */
#ifndef SDF_ALLOC_H
#define SDF_ALLOC_H
#include <math_utils.h> 
#include <se/node.hpp>
#include <se/utils/morton_utils.hpp>

/* 
 * \brief Given a depth map and camera matrix it computes the list of 
 * voxels intersected but not allocated by the rays around the measurement m in
 * a region comprised between m +/- band. 
 * \param allocationList output list of keys corresponding to voxel blocks to
 * be allocated
 * \param reserved allocated size of allocationList
 * \param map_index indexing structure used to index voxel blocks 
 * \param pose camera extrinsics matrix
 * \param K camera intrinsics matrix
 * \param depthmap input depth map
 * \param imageSize dimensions of depthmap
 * \param size discrete extent of the map, in number of voxels
 * \param voxelSize spacing between two consegutive voxels, in metric space
 * \param band maximum extent of the allocating region, per ray
 */
template <typename FieldType, template <typename> class OctreeT, typename HashType>
unsigned int buildAllocationList(HashType * allocationList, size_t reserved,
    OctreeT<FieldType>& map_index, const Matrix4 &pose, const Matrix4& K, 
    const float *depthmap, const Eigen::Vector2i& imageSize, 
    const unsigned int size,  const float voxelSize, const float band) {

  const float inverseVoxelSize = 1/voxelSize;
  const unsigned block_scale = log2(size) - log2_const(se::VoxelBlock<FieldType>::side);

  Matrix4 invK = inverse(K);
  const Matrix4 kPose = pose * invK;
  

#ifdef _OPENMP
  std::atomic<unsigned int> voxelCount;
#else
  unsigned int voxelCount;
#endif

  int x, y;
  const float3 camera = get_translation(pose);
  const int numSteps = ceil(band*inverseVoxelSize);
  voxelCount = 0;
#pragma omp parallel for \
  private(y)
  for (y = 0; y < imageSize.y(); y++) {
    for (x = 0; x < imageSize.x(); x++) {
      if(depthmap[x + y*imageSize.x()] == 0)
        continue;
      const float depth = depthmap[x + y*imageSize.x()];
      float3 worldVertex = (kPose * make_float3((x + 0.5f) * depth, 
            (y + 0.5f) * depth, depth));

      float3 direction = normalize(camera - worldVertex);
      const float3 origin = worldVertex - (band * 0.5f) * direction;
      const float3 step = (direction*band)/numSteps;

      int3 voxel;
      float3 voxelPos = origin;
      for(int i = 0; i < numSteps; i++){
        float3 voxelScaled = floorf(voxelPos * inverseVoxelSize);
        if((voxelScaled.x < size) && (voxelScaled.y < size) &&
            (voxelScaled.z < size) && (voxelScaled.x >= 0) &&
            (voxelScaled.y >= 0) && (voxelScaled.z >= 0)){
          voxel = make_int3(voxelScaled);
          se::VoxelBlock<FieldType> * n = map_index.fetch(voxel.x, voxel.y, voxel.z);
          if(!n){
            HashType k = map_index.hash(voxel.x, voxel.y, voxel.z, 
                block_scale);
            unsigned int idx = ++voxelCount;
            if(idx < reserved) {
              allocationList[idx] = k;
            } else
              break;
          }
          else {
            n->active(true); 
          }
        }
        voxelPos +=step;
      }
    }
  }
  const uint written = voxelCount;
  return written >= reserved ? reserved : written;
}

#endif
