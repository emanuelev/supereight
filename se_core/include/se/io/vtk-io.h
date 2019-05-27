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
    const Eigen::Vector3i upper, FieldSelector select, const char* filename_dist,
    const char* filename_scale = NULL) {
  std::stringstream x_coordinates, y_coordinates, z_coordinates, scalars, scales;
  std::ofstream f_dist;
  f_dist.open(filename_dist);

  std::ofstream f_scale;
  f_scale.open(filename_scale);

  const int dimX = upper.x() - lower.x();
  const int dimY = upper.y() - lower.y();
  const int dimZ = upper.z() - lower.z();

  for (int x = lower.x(); x < upper.x(); ++x) {
    x_coordinates << x << " ";
  }
  for (int y = lower.y(); y < upper.y(); ++y) {
    y_coordinates << y << " ";
  }
  for (int z = lower.z(); z < upper.z(); ++z) {
    z_coordinates << z << " ";
  }

  for (int z = lower.z(); z < upper.z(); ++z)
    for (int y = lower.y(); y < upper.y(); ++y)
      for (int x = lower.x(); x < upper.x(); ++x) {
        auto data = in.interp(Eigen::Vector3f(x, y, z), select);
        scalars << data.first << std::endl;
        scales << data.second << std::endl;
      }

  f_dist << "# vtk DataFile Version 1.0" << std::endl;
  f_dist << "vtk mesh generated from KFusion" << std::endl;
  f_dist << "ASCII" << std::endl;
  f_dist << "DATASET RECTILINEAR_GRID" << std::endl;
  f_dist << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;

  f_dist << "X_COORDINATES " << dimX << " int " << std::endl;
  f_dist << x_coordinates.str() << std::endl;

  f_dist << "Y_COORDINATES " << dimY << " int " << std::endl;
  f_dist << y_coordinates.str() << std::endl;

  f_dist << "Z_COORDINATES " << dimZ << " int " << std::endl;
  f_dist << z_coordinates.str() << std::endl;

  f_dist << "POINT_DATA " << dimX*dimY*dimZ << std::endl;
  f_dist << "SCALARS scalars float 1" << std::endl;
  f_dist << "LOOKUP_TABLE default" << std::endl;
  f_dist << scalars.str() << std::endl;
  f_dist.close();

  if (filename_scale != NULL) {
    f_scale << "# vtk DataFile Version 1.0" << std::endl;
    f_scale << "vtk mesh generated from KFusion" << std::endl;
    f_scale << "ASCII" << std::endl;
    f_scale << "DATASET RECTILINEAR_GRID" << std::endl;
    f_scale << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;

    f_scale << "X_COORDINATES " << dimX << " int " << std::endl;
    f_scale << x_coordinates.str() << std::endl;

    f_scale << "Y_COORDINATES " << dimY << " int " << std::endl;
    f_scale << y_coordinates.str() << std::endl;

    f_scale << "Z_COORDINATES " << dimZ << " int " << std::endl;
    f_scale << z_coordinates.str() << std::endl;

    f_scale << "POINT_DATA " << dimX*dimY*dimZ << std::endl;
    f_scale << "SCALARS scalars float 1" << std::endl;
    f_scale << "LOOKUP_TABLE default" << std::endl;
    f_scale << scales.str() << std::endl;
    f_scale.close();
  }

} 
#endif
