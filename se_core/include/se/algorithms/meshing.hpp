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

#ifndef MESHING_HPP
#define MESHING_HPP
#include "../octree.hpp"
#include "edge_tables.h"

namespace se {
namespace meshing {
  enum status : uint8_t {
    OUTSIDE = 0x0,
    UNKNOWN = 0xFE, // 254
    INSIDE = 0xFF, // 255
  };

  void savePointCloudPly(const std::vector<Triangle>& mesh, 
      const char* filename, const Eigen::Vector3f init_pose) {
    std::stringstream points;
    int num_points = 0;
    for(int i = 0; i < mesh.size(); ++i ){
      const Triangle& t = mesh[i];
      for(int j = 0; j < 3; ++j) {
        points << t.vertexes[j].x() - init_pose.x() << " " 
               << t.vertexes[j].y() - init_pose.y() << " " 
               << t.vertexes[j].z() - init_pose.z() << std::endl; 
        num_points++;
      }
    }   

    {
      std::ofstream f;
      f.open(std::string(filename).c_str());

      f << "ply" << std::endl;
      f << "format ascii 1.0" << std::endl;
      f << "comment octree structure" << std::endl;
      f << "element vertex " << num_points <<  std::endl;
      f << "property float x" << std::endl;
      f << "property float y" << std::endl;
      f << "property float z" << std::endl;
      f << "end_header" << std::endl;
      f << points.str();
    }
  }

  template <typename Map, typename FieldSelector>
    inline Eigen::Vector3f compute_intersection(const Map& volume, FieldSelector select,
        const Eigen::Vector3i& source, const Eigen::Vector3i& dest){
      const float voxelSize = volume.dim()/volume.size(); 
      Eigen::Vector3f s = Eigen::Vector3f(source(0) * voxelSize, source(1) * voxelSize, source(2) * voxelSize);
      Eigen::Vector3f d = Eigen::Vector3f(dest(0) * voxelSize, dest(1) * voxelSize, dest(2) * voxelSize);
      float v1 = select(volume.get_fine(source(0), source(1), source(2)));
      float v2 = select(volume.get_fine(dest(0), dest(1), dest(2))); 
      return s + (0.0 - v1)*(d - s)/(v2-v1);
    }

  template <typename Map, typename FieldSelector>
    inline Eigen::Vector3f interp_vertexes(const Map& volume, FieldSelector select, 
        const unsigned x, const unsigned y, const unsigned z, const int edge){
      switch(edge){
        case 0:  return compute_intersection(volume, select, Eigen::Vector3i(x,   y, z),     
                     Eigen::Vector3i(x+1, y, z));
        case 1:  return compute_intersection(volume, select, Eigen::Vector3i(x+1, y, z),     
                     Eigen::Vector3i(x+1, y, z+1));
        case 2:  return compute_intersection(volume, select, Eigen::Vector3i(x+1, y, z+1),   
                     Eigen::Vector3i(x, y, z+1));
        case 3:  return compute_intersection(volume, select, Eigen::Vector3i(x,   y, z),     
                     Eigen::Vector3i(x, y, z+1));
        case 4:  return compute_intersection(volume, select, Eigen::Vector3i(x,   y+1, z),   
                     Eigen::Vector3i(x+1, y+1, z));
        case 5:  return compute_intersection(volume, select, Eigen::Vector3i(x+1, y+1, z),   
                     Eigen::Vector3i(x+1, y+1, z+1));
        case 6:  return compute_intersection(volume, select, Eigen::Vector3i(x+1, y+1, z+1), 
                     Eigen::Vector3i(x, y+1, z+1));
        case 7:  return compute_intersection(volume, select, Eigen::Vector3i(x,   y+1, z),   
                     Eigen::Vector3i(x,   y+1, z+1));

        case 8:  return compute_intersection(volume, select, Eigen::Vector3i(x,   y, z),     
                     Eigen::Vector3i(x,   y+1, z));
        case 9:  return compute_intersection(volume, select, Eigen::Vector3i(x+1, y, z),     
                     Eigen::Vector3i(x+1, y+1, z));
        case 10: return compute_intersection(volume, select, Eigen::Vector3i(x+1, y, z+1),   
                     Eigen::Vector3i(x+1, y+1, z+1));
        case 11: return compute_intersection(volume, select, Eigen::Vector3i(x,   y, z+1),   
                     Eigen::Vector3i(x,   y+1, z+1));
      }
      return Eigen::Vector3f::Constant(0);
    }

  template <typename FieldType, typename PointT>
    inline void gather_points( const se::VoxelBlock<FieldType>* cached, PointT points[8], 
        const int x, const int y, const int z) {
      points[0] = cached->data(Eigen::Vector3i(x, y, z)); 
      points[1] = cached->data(Eigen::Vector3i(x+1, y, z));
      points[2] = cached->data(Eigen::Vector3i(x+1, y, z+1));
      points[3] = cached->data(Eigen::Vector3i(x, y, z+1));
      points[4] = cached->data(Eigen::Vector3i(x, y+1, z));
      points[5] = cached->data(Eigen::Vector3i(x+1, y+1, z));
      points[6] = cached->data(Eigen::Vector3i(x+1, y+1, z+1));
      points[7] = cached->data(Eigen::Vector3i(x, y+1, z+1));
    }

  template <typename FieldType, template <typename FieldT> class MapT, typename PointT>
  inline void gather_points(const MapT<FieldType>& volume, PointT points[8], 
                 const int x, const int y, const int z) {
               points[0] = volume.get_fine(x, y, z); 
               points[1] = volume.get_fine(x+1, y, z);
               points[2] = volume.get_fine(x+1, y, z+1);
               points[3] = volume.get_fine(x, y, z+1);
               points[4] = volume.get_fine(x, y+1, z);
               points[5] = volume.get_fine(x+1, y+1, z);
               points[6] = volume.get_fine(x+1, y+1, z+1);
               points[7] = volume.get_fine(x, y+1, z+1);
             }

  template <typename FieldType, template <typename FieldT> class MapT,
  typename InsidePredicate>
  uint8_t compute_index(const MapT<FieldType>& volume, 
  const se::VoxelBlock<FieldType>* cached, InsidePredicate inside,
  const unsigned x, const unsigned y, const unsigned z){
    unsigned int blockSize =  se::VoxelBlock<FieldType>::side;
    unsigned int local = ((x % blockSize == blockSize - 1) << 2) | 
      ((y % blockSize == blockSize - 1) << 1) |
      ((z % blockSize) == blockSize - 1);

    typename MapT<FieldType>::value_type points[8];
    if(!local) gather_points(cached, points, x, y, z);
    else gather_points(volume, points, x, y, z);

    uint8_t index = 0;

    if(points[0].y == 0.f) return 0;
    if(points[1].y == 0.f) return 0;
    if(points[2].y == 0.f) return 0;
    if(points[3].y == 0.f) return 0;
    if(points[4].y == 0.f) return 0;
    if(points[5].y == 0.f) return 0;
    if(points[6].y == 0.f) return 0;
    if(points[7].y == 0.f) return 0;

    if(inside(points[0])) index |= 1;
    if(inside(points[1])) index |= 2;
    if(inside(points[2])) index |= 4;
    if(inside(points[3])) index |= 8;
    if(inside(points[4])) index |= 16;
    if(inside(points[5])) index |= 32;
    if(inside(points[6])) index |= 64;
    if(inside(points[7])) index |= 128;
    // std::cerr << std::endl << std::endl;

    return index;
  }

  inline bool checkVertex(const Eigen::Vector3f v, const int dim){
    return (v(0) <= 0 || v(1) <=0 || v(2) <= 0 || v(0) > dim || v(1) > dim || v(2) > dim);
  }

}
namespace algorithms {
  template <typename FieldType, typename FieldSelector, 
            typename InsidePredicate, typename TriangleType>
    void marching_cube(Octree<FieldType>& volume, FieldSelector select, 
        InsidePredicate inside, std::vector<TriangleType>& triangles)
    {

      using namespace meshing;
      std::stringstream points, polygons;
      std::vector<se::VoxelBlock<FieldType>*> blocklist;
      std::mutex lck;
      const int size = volume.size();
      const int dim = volume.dim();
      volume.getBlockList(blocklist, false);
      std::cout << "Blocklist size: " << blocklist.size() << std::endl;
      

#pragma omp parallel for
      for(size_t i = 0; i < blocklist.size(); i++){
        se::VoxelBlock<FieldType> * leaf = static_cast<se::VoxelBlock<FieldType> *>(blocklist[i]);  
        int edge = se::VoxelBlock<FieldType>::side;
        int x, y, z ; 
        const Eigen::Vector3i& start = leaf->coordinates();
        const Eigen::Vector3i top = 
          (leaf->coordinates() + Eigen::Vector3i::Constant(edge)).cwiseMin(
              Eigen::Vector3i::Constant(size-1));
        for(x = start(0); x < top(0); x++){
          for(y = start(1); y < top(1); y++){
            for(z = start(2); z < top(2); z++){

              uint8_t index = meshing::compute_index(volume, leaf, inside, x, y, z);

                int * edges = triTable[index]; 
              for(unsigned int e = 0; edges[e] != -1 && e < 16; e += 3){
                Eigen::Vector3f v1 = interp_vertexes(volume, select, x, y, z, edges[e]);
                Eigen::Vector3f v2 = interp_vertexes(volume, select, x, y, z, edges[e+1]);
                Eigen::Vector3f v3 = interp_vertexes(volume, select, x, y, z, edges[e+2]);
                if(checkVertex(v1, dim) || checkVertex(v2, dim) || checkVertex(v3, dim)) continue;
                Triangle temp = Triangle();
                temp.vertexes[0] = v1;
                temp.vertexes[1] = v2;
                temp.vertexes[2] = v3;
                lck.lock();
                triangles.push_back(temp);
                lck.unlock();
              }
            }
          }
        }
      }
    }
}
}
#endif
