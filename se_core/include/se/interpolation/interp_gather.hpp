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
#ifndef INTERP_GATHER_H
#define INTERP_GATHER_H
#include "../node.hpp"

namespace se {

/*
 * Interpolation's point gather offsets
 */

static const Eigen::Vector3i interp_offsets[8] = 
  {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, 
   {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};

template <typename FieldType, typename FieldSelector>
inline void gather_local(const se::VoxelBlock<FieldType>* block, const Eigen::Vector3i& base, 
    FieldSelector select, float points[8]) {

  if(!block) {
    points[0] = select(se::VoxelBlock<FieldType>::empty());
    points[1] = select(se::VoxelBlock<FieldType>::empty());
    points[2] = select(se::VoxelBlock<FieldType>::empty());
    points[3] = select(se::VoxelBlock<FieldType>::empty());
    points[4] = select(se::VoxelBlock<FieldType>::empty());
    points[5] = select(se::VoxelBlock<FieldType>::empty());
    points[6] = select(se::VoxelBlock<FieldType>::empty());
    points[7] = select(se::VoxelBlock<FieldType>::empty());
    return;
  }

  points[0] = select(block->data(base + interp_offsets[0]));
  points[1] = select(block->data(base + interp_offsets[1]));
  points[2] = select(block->data(base + interp_offsets[2]));
  points[3] = select(block->data(base + interp_offsets[3]));
  points[4] = select(block->data(base + interp_offsets[4]));
  points[5] = select(block->data(base + interp_offsets[5]));
  points[6] = select(block->data(base + interp_offsets[6]));
  points[7] = select(block->data(base + interp_offsets[7]));
  return;
}

template <typename FieldType, typename FieldSelector>
inline void gather_4(const se::VoxelBlock<FieldType>* block, const Eigen::Vector3i& base, 
    FieldSelector select, const unsigned int offsets[4], float points[8]) {

  if(!block) {
    points[offsets[0]] = select(se::VoxelBlock<FieldType>::empty());
    points[offsets[1]] = select(se::VoxelBlock<FieldType>::empty());
    points[offsets[2]] = select(se::VoxelBlock<FieldType>::empty());
    points[offsets[3]] = select(se::VoxelBlock<FieldType>::empty());
    return;
  }

  points[offsets[0]] = select(block->data(base + interp_offsets[offsets[0]]));
  points[offsets[1]] = select(block->data(base + interp_offsets[offsets[1]]));
  points[offsets[2]] = select(block->data(base + interp_offsets[offsets[2]]));
  points[offsets[3]] = select(block->data(base + interp_offsets[offsets[3]]));
  return;
}

template <typename FieldType, typename FieldSelector>
inline void gather_2(const se::VoxelBlock<FieldType>* block, 
    const Eigen::Vector3i& base, FieldSelector select, 
    const unsigned int offsets[2], float points[8]) {

  if(!block) {
    points[offsets[0]] = select(se::VoxelBlock<FieldType>::empty());
    points[offsets[1]] = select(se::VoxelBlock<FieldType>::empty());
    return;
  }

  points[offsets[0]] = select(block->data(base + interp_offsets[offsets[0]]));
  points[offsets[1]] = select(block->data(base + interp_offsets[offsets[1]]));
  return;
}

template <typename FieldType, template<typename FieldT> class MapIndex,
         class FieldSelector>
inline void gather_points(const MapIndex<FieldType>& fetcher, 
    const Eigen::Vector3i& base, 
    FieldSelector select, float points[8]) {
 
  unsigned int blockSize =  se::VoxelBlock<FieldType>::side;
  unsigned int crossmask = ((base(0) % blockSize == blockSize - 1) << 2) | 
                           ((base(1) % blockSize == blockSize - 1) << 1) |
                           ((base(2) % blockSize) == blockSize - 1);

  switch(crossmask) {
    case 0: /* all local */
      {
        se::VoxelBlock<FieldType> * block = fetcher.fetch(base(0), base(1), base(2));
        gather_local(block, base, select, points);
      }
      break;
    case 1: /* z crosses */
      {
        const unsigned int offs1[4] = {0, 1, 2, 3};
        const unsigned int offs2[4] = {4, 5, 6, 7};
        se::VoxelBlock<FieldType> * block = fetcher.fetch(base(0), base(1), base(2));
        gather_4(block, base, select, offs1, points);
        const Eigen::Vector3i base1 = base + interp_offsets[offs2[0]];
        block = fetcher.fetch(base1(0), base1(1), base1(2));
        gather_4(block, base, select, offs2, points);
      }
      break;
    case 2: /* y crosses */ 
      {
        const unsigned int offs1[4] = {0, 1, 4, 5};
        const unsigned int offs2[4] = {2, 3, 6, 7};
        se::VoxelBlock<FieldType> * block = fetcher.fetch(base(0), base(1), base(2));
        gather_4(block, base, select, offs1, points);
        const Eigen::Vector3i base1 = base + interp_offsets[offs2[0]];
        block = fetcher.fetch(base1(0), base1(1), base1(2));
        gather_4(block, base, select, offs2, points);
      }
      break;
    case 3: /* y, z cross */ 
      {
        const unsigned int offs1[2] = {0, 1};
        const unsigned int offs2[2] = {2, 3};
        const unsigned int offs3[2] = {4, 5};
        const unsigned int offs4[2] = {6, 7};
        const Eigen::Vector3i base2 = base + interp_offsets[offs2[0]];
        const Eigen::Vector3i base3 = base + interp_offsets[offs3[0]];
        const Eigen::Vector3i base4 = base + interp_offsets[offs4[0]];
        se::VoxelBlock<FieldType> * block = fetcher.fetch(base(0), base(1), base(2));
        gather_2(block, base, select, offs1, points);
        block = fetcher.fetch(base2(0), base2(1), base2(2));
        gather_2(block, base, select, offs2, points);
        block = fetcher.fetch(base3(0), base3(1), base3(2));
        gather_2(block, base, select, offs3, points);
        block = fetcher.fetch(base4(0), base4(1), base4(2));
        gather_2(block, base, select, offs4, points);
      }
      break;
    case 4: /* x crosses */ 
      {
        const unsigned int offs1[4] = {0, 2, 4, 6};
        const unsigned int offs2[4] = {1, 3, 5, 7};
        se::VoxelBlock<FieldType> * block = fetcher.fetch(base(0), base(1), base(2));
        gather_4(block, base, select, offs1, points);
        const Eigen::Vector3i base1 = base + interp_offsets[offs2[0]];
        block = fetcher.fetch(base1(0), base1(1), base1(2));
        gather_4(block, base, select, offs2, points);
      }
      break;
    case 5: /* x,z cross */ 
      {
        const unsigned int offs1[2] = {0, 2};
        const unsigned int offs2[2] = {1, 3};
        const unsigned int offs3[2] = {4, 6};
        const unsigned int offs4[2] = {5, 7};
        const Eigen::Vector3i base2 = base + interp_offsets[offs2[0]];
        const Eigen::Vector3i base3 = base + interp_offsets[offs3[0]];
        const Eigen::Vector3i base4 = base + interp_offsets[offs4[0]];
        se::VoxelBlock<FieldType> * block = fetcher.fetch(base(0), base(1), base(2));
        gather_2(block, base, select, offs1, points);
        block = fetcher.fetch(base2(0), base2(1), base2(2));
        gather_2(block, base, select, offs2, points);
        block = fetcher.fetch(base3(0), base3(1), base3(2));
        gather_2(block, base, select, offs3, points);
        block = fetcher.fetch(base4(0), base4(1), base4(2));
        gather_2(block, base, select, offs4, points);
      }
      break;
    case 6: /* x,y cross */ 
      {
        const unsigned int offs1[2] = {0, 4};
        const unsigned int offs2[2] = {1, 5};
        const unsigned int offs3[2] = {2, 6};
        const unsigned int offs4[2] = {3, 7};
        const Eigen::Vector3i base2 = base + interp_offsets[offs2[0]];
        const Eigen::Vector3i base3 = base + interp_offsets[offs3[0]];
        const Eigen::Vector3i base4 = base + interp_offsets[offs4[0]];
        se::VoxelBlock<FieldType> * block = fetcher.fetch(base(0), base(1), base(2));
        gather_2(block, base, select, offs1, points);
        block = fetcher.fetch(base2(0), base2(1), base2(2));
        gather_2(block, base, select, offs2, points);
        block = fetcher.fetch(base3(0), base3(1), base3(2));
        gather_2(block, base, select, offs3, points);
        block = fetcher.fetch(base4(0), base4(1), base4(2));
        gather_2(block, base, select, offs4, points);
      }
      break;

    case 7:
      {
        Eigen::Vector3i vox[8];
        vox[0] = base + interp_offsets[0];
        vox[1] = base + interp_offsets[1];
        vox[2] = base + interp_offsets[2];
        vox[3] = base + interp_offsets[3];
        vox[4] = base + interp_offsets[4];
        vox[5] = base + interp_offsets[5];
        vox[6] = base + interp_offsets[6];
        vox[7] = base + interp_offsets[7];

        points[0] = select(fetcher.get_fine(vox[0](0), vox[0](1), vox[0](2)));
        points[1] = select(fetcher.get_fine(vox[1](0), vox[1](1), vox[1](2)));
        points[2] = select(fetcher.get_fine(vox[2](0), vox[2](1), vox[2](2)));
        points[3] = select(fetcher.get_fine(vox[3](0), vox[3](1), vox[3](2)));
        points[4] = select(fetcher.get_fine(vox[4](0), vox[4](1), vox[4](2)));
        points[5] = select(fetcher.get_fine(vox[5](0), vox[5](1), vox[5](2)));
        points[6] = select(fetcher.get_fine(vox[6](0), vox[6](1), vox[6](2)));
        points[7] = select(fetcher.get_fine(vox[7](0), vox[7](1), vox[7](2)));
      }
      break;
  }
}
}
#endif
