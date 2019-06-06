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

#ifndef VTK_IO_H
#define VTK_IO_H
#include <fstream>
#include <sstream>
#include <iostream>
#include <se/utils/math_utils.h>
#include <algorithm>

template <typename T>
void savePointCloud(const T* in, const int num_points, 
    const char* filename, const Eigen::Vector3f init_pose){
  std::stringstream points;

  for(int i = 0; i < num_points; ++i ){
    points << in[i].x() - init_pose.x() 
           << " " << in[i].y()  - init_pose.y() << " " 
           << in[i].z() - init_pose.z() << std::endl; 
  }   

  std::ofstream f;
  f.open(std::string(filename).append(".vtk").c_str());
  f << "# vtk DataFile Version 1.0" << std::endl;
  f << "vtk mesh generated from KFusion" << std::endl;
  f << "ASCII" << std::endl;
  f << "DATASET POLYDATA" << std::endl;

  f << "POINTS " << num_points << " FLOAT" << std::endl;
  f << points.str();
  f.close();
}

template <typename MapType, typename FieldSelector>
void save3DSlice(const MapType& in, const Eigen::Vector3i lower, 
    const Eigen::Vector3i upper, FieldSelector select, const int scale, const char* filename){
  std::stringstream x_coordinates, y_coordinates, z_coordinates, scalars;
  std::ofstream f;
  f.open(filename);
 
  const int stride = 1 << scale;
  const int dimX = std::max(1, (upper.x() - lower.x()) / stride);
  const int dimY = std::max(1, (upper.y() - lower.y()) / stride);
  const int dimZ = std::max(1, (upper.z() - lower.z()) / stride);

  f << "# vtk DataFile Version 1.0" << std::endl;
  f << "vtk mesh generated from KFusion" << std::endl;
  f << "ASCII" << std::endl;
  f << "DATASET RECTILINEAR_GRID" << std::endl;
  f << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;

  for(int x = lower.x(); x < upper.x(); x += stride)
    x_coordinates << x << " ";  
  for(int y = lower.y(); y < upper.y(); y += stride)
    y_coordinates << y << " ";  
  for(int z = lower.z(); z < upper.z(); z += stride)
    z_coordinates << z << " ";  

  for(int z = lower.z(); z < upper.z(); z += stride)
    for(int y = lower.y(); y < upper.y(); y += stride)
      for(int x = lower.x(); x < upper.x(); x += stride) {
        float data = in.get_fine(x, y, z, scale).x;
        scalars << data  << std::endl;
      }

  f << "X_COORDINATES " << dimX << " int " << std::endl;
  f << x_coordinates.str() << std::endl;

  f << "Y_COORDINATES " << dimY << " int " << std::endl;
  f << y_coordinates.str() << std::endl;

  f << "Z_COORDINATES " << dimZ << " int " << std::endl;
  f << z_coordinates.str() << std::endl;

  f << "POINT_DATA " << dimX*dimY*dimZ << std::endl;
  f << "SCALARS scalars float 1" << std::endl;
  f << "LOOKUP_TABLE default" << std::endl;
  f << scalars.str() << std::endl;
  f.close();
} 
#endif
