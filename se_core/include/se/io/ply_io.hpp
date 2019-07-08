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

#ifndef PLY_IO_HPP
#define PLY_IO_HPP
#include <fstream>
#include <sstream>
#include <se/octree.hpp>
#include <se/node_iterator.hpp>
#include <Eigen/Dense>

/*! \file */
namespace se {
  template <typename T>
  /*! \brief Write the octree structure to file in ply format. 
   * Aggregated voxel blocks are written as a single octant of size block_side^3.
   * \param filename output filename
   * \param map octree map
   */
    void print_octree(const char* filename, const se::Octree<T>& map) {
      std::stringstream ss_vertex;
      std::stringstream ss_faces;
      se::node_iterator<T> it(map);
      se::Node<T>* n = it.next();
      const int depth = std::log2(map.size());
      int vertex_num = 0;
      int faces_num  = 0;
      while(n) {
        const Eigen::Vector3i corner = se::keyops::decode(n->code_);
        const int side = 1 << (depth - se::keyops::level(n->code_));

        Eigen::Vector3f vertices[8];
        vertices[0] = corner.cast<float>(); 
        vertices[1] = (corner + Eigen::Vector3i(side, 0, 0)).cast<float>(); 
        vertices[2] = (corner + Eigen::Vector3i(0, side, 0)).cast<float>(); 
        vertices[3] = (corner + Eigen::Vector3i(side, side, 0)).cast<float>(); 
        vertices[4] = (corner + Eigen::Vector3i(0, 0, side)).cast<float>(); 
        vertices[5] = (corner + Eigen::Vector3i(side, 0, side)).cast<float>(); 
        vertices[6] = (corner + Eigen::Vector3i(0, side, side)).cast<float>(); 
        vertices[7] = (corner + Eigen::Vector3i(side, side, side)).cast<float>(); 

        for(int i = 0; i < 8; ++i) {
          ss_vertex << vertices[i].x() << " " << vertices[i].y() << " " << vertices[i].z() << std::endl;
        }

        ss_faces << "4 " << vertex_num << " " << vertex_num + 1 
                 << " "  << vertex_num + 3 << " " << vertex_num + 2 << std::endl;

        ss_faces << "4 " << vertex_num + 1 << " " << vertex_num + 5 
                 << " "  << vertex_num + 7 << " " << vertex_num + 3 << std::endl;

        ss_faces << "4 " << vertex_num + 5 << " " << vertex_num + 7 
                 << " "  << vertex_num + 6 << " " << vertex_num + 4 << std::endl;

        ss_faces << "4 " << vertex_num << " " << vertex_num + 2
                 << " "  << vertex_num + 6 << " " << vertex_num + 4 << std::endl;

        ss_faces << "4 " << vertex_num << " " << vertex_num + 1
                 << " "  << vertex_num + 5 << " " << vertex_num + 4 << std::endl;

        ss_faces << "4 " << vertex_num + 2 << " " << vertex_num + 3
                 << " "  << vertex_num + 7 << " " << vertex_num + 6 << std::endl;

        vertex_num += 8;
        faces_num  += 6;
        n = it.next();
      }

      {
        std::ofstream f;
        f.open(std::string(filename).c_str());

        f << "ply" << std::endl;
        f << "format ascii 1.0" << std::endl;
        f << "comment octree structure" << std::endl;
        f << "element vertex " << vertex_num <<  std::endl;
        f << "property float x" << std::endl;
        f << "property float y" << std::endl;
        f << "property float z" << std::endl;
        f << "element face " << faces_num << std::endl;
        f << "property list uchar int vertex_index" << std::endl;
        f << "end_header" << std::endl;
        f << ss_vertex.str();
        f << ss_faces.str();
      }
    }

  template <typename T>
    void savePointCloudPly(const T* in, const int num_points, 
        const char* filename, const Eigen::Vector3f init_pose) {
      std::stringstream points;
      for(int i = 0; i < num_points; ++i ){
        points << in[i].x() - init_pose.x() 
          << " " << in[i].y()  - init_pose.y() << " " 
          << in[i].z() - init_pose.z() << std::endl; 
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
}
#endif
