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
#include <lodepng.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <se/utils/math_utils.h>
#include <algorithm>

//http://stackoverflow.com/questions/236129/split-a-string-in-c
static inline void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

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

// 
// Function to save a slice of the 3D volume to vtk file. The convention
// for the iteration limits is that it expets them in the format [lower, upper),
// i.e. the upper bound is not included in the set. 
//
// template <typename T>
// void save3DSlice(const T* in, const uint3 lower, const uint3 upper, 
//     const uint3 size, const char* filename){
//   std::stringstream x_coordinates, y_coordinates, z_coordinates, scalars;
//   std::ofstream f;
//   f.open(filename);
//  
//   const int dimX = upper.x - lower.x;
//   const int dimY = upper.y - lower.y;
//   const int dimZ = upper.z - lower.z;
// 
//   f << "# vtk DataFile Version 1.0" << std::endl;
//   f << "vtk mesh generated from KFusion" << std::endl;
//   f << "ASCII" << std::endl;
//   f << "DATASET RECTILINEAR_GRID" << std::endl;
//   f << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;
// 
//   for(int x = lower.x; x < upper.x; ++x)
//     x_coordinates << x << " ";  
//   for(int y = lower.y; y < upper.y; ++y)
//     y_coordinates << y << " ";  
//   for(int z = lower.z; z < upper.z; ++z)
//     z_coordinates << z << " ";  
// 
//   for(int z = lower.z; z < upper.z; ++z)
//     for(int y = lower.y; y < upper.y; ++y)
//       for(int x = lower.x; x < upper.x; ++x) {
//         const float data = in[x + y*size.x + z*size.x*size.y].x * 0.00003051944088f;
//         scalars << data  << std::endl;
//       }
// 
//   f << "X_COORDINATES " << dimX << " int " << std::endl;
//   f << x_coordinates.str() << std::endl;
// 
//   f << "Y_COORDINATES " << dimY << " int " << std::endl;
//   f << y_coordinates.str() << std::endl;
// 
//   f << "Z_COORDINATES " << dimZ << " int " << std::endl;
//   f << z_coordinates.str() << std::endl;
// 
//   f << "POINT_DATA " << dimX*dimY*dimZ << std::endl;
//   f << "SCALARS scalars float 1" << std::endl;
//   f << "LOOKUP_TABLE default" << std::endl;
//   f << scalars.str() << std::endl;
//   f.close();
// } 

template <typename MapType>
void save3DSlice(const MapType& in, const Eigen::Vector3i lower, 
    const Eigen::Vector3i upper, 
    const Eigen::Vector3i, const char* filename){
  std::stringstream x_coordinates, y_coordinates, z_coordinates, scalars;
  std::ofstream f;
  f.open(filename);
 
  const int dimX = upper.x() - lower.x();
  const int dimY = upper.y() - lower.y();
  const int dimZ = upper.z() - lower.z();

  f << "# vtk DataFile Version 1.0" << std::endl;
  f << "vtk mesh generated from KFusion" << std::endl;
  f << "ASCII" << std::endl;
  f << "DATASET RECTILINEAR_GRID" << std::endl;
  f << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;

  for(int x = lower.x(); x < upper.x(); ++x)
    x_coordinates << x << " ";  
  for(int y = lower.y(); y < upper.y(); ++y)
    y_coordinates << y << " ";  
  for(int z = lower.z(); z < upper.z(); ++z)
    z_coordinates << z << " ";  

  for(int z = lower.z(); z < upper.z(); ++z)
    for(int y = lower.y(); y < upper.y(); ++y)
      for(int x = lower.x(); x < upper.x(); ++x) {
        const float data = in.get(x, y, z).x;
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

template <typename MapType, typename MapOp>
void save3DSlice(const MapType& in, MapOp op, const Eigen::Vector3i lower, 
    const Eigen::Vector3i upper, 
    const Eigen::Vector3i, const char* filename){
  std::stringstream x_coordinates, y_coordinates, z_coordinates, scalars;
  std::ofstream f;
  f.open(filename);
 
  const int dimX = upper.x() - lower.x();
  const int dimY = upper.y() - lower.y();
  const int dimZ = upper.z() - lower.z();

  f << "# vtk DataFile Version 1.0" << std::endl;
  f << "vtk mesh generated from KFusion" << std::endl;
  f << "ASCII" << std::endl;
  f << "DATASET RECTILINEAR_GRID" << std::endl;
  f << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;

  for(int x = lower.x(); x < upper.x(); ++x)
    x_coordinates << x << " ";  
  for(int y = lower.y(); y < upper.y(); ++y)
    y_coordinates << y << " ";  
  for(int z = lower.z(); z < upper.z(); ++z)
    z_coordinates << z << " ";  

  for(int z = lower.z(); z < upper.z(); ++z)
    for(int y = lower.y(); y < upper.y(); ++y)
      for(int x = lower.x(); x < upper.x(); ++x) {
        const float data = op(in, x, y, z);
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

template <typename BlockList>
void saveBlockList(const BlockList& in, const Eigen::Vector3i shift, const char* filename){

  std::stringstream x_coordinates, y_coordinates, z_coordinates, scalars;
  std::ofstream f;
  f.open(filename);

  std::stringstream points;
  std::stringstream cells;
  int voxel_count = 0;

  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > coords;
  int num_blocks = in.size();
  for(int i = 0; i < num_blocks; ++i) {
    auto * block = in[i];
    coords.push_back(block->coordinates());
  }

  auto comp = [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) -> bool {
      return (a.z() < b.z() && a.y() < b.y() && a.x() < b.x());
      };


  std::sort(coords.begin(), coords.end(), comp);

  Eigen::Vector3i min_elem = coords[0];

  std::cout << "Min element: " << min_elem.x() << ", " << min_elem.y() << ", " 
    << min_elem.z() <<  std::endl;
  std::cout << "Shift: " << shift.x() << ", " << shift.y() << ", " 
    << shift.z() <<  std::endl;

  const int step = 8;
  for(unsigned int i = 0; i < coords.size(); ++i) {
    // shift it to the origin
    const Eigen::Vector3i voxel = coords[i] - shift;
    points << voxel.x()      << " " << voxel.y() << " " <<  voxel.z()   << std::endl;
    points << voxel.x()+step << " " << voxel.y() << " " <<  voxel.z()   << std::endl;
    points << voxel.x()      << " " << voxel.y()+step << " " <<  voxel.z()  << std::endl;
    points << voxel.x()+step << " " << voxel.y()+step << " " <<  voxel.z()   << std::endl;
    points << voxel.x()   << " " << voxel.y()   << " " <<  voxel.z()+step << std::endl;
    points << voxel.x()+step << " " << voxel.y()   << " " <<  voxel.z()+step << std::endl;
    points << voxel.x()   << " " << voxel.y()+step << " " <<  voxel.z()+step << std::endl;
    points << voxel.x()+step << " " << voxel.y()+step << " " <<  voxel.z()+step << std::endl;

    cells << "8";
    for(int w = 0; w < 8; ++w) cells << " " << (voxel_count*8)+w;
    cells << std::endl;
    voxel_count++;
  }

  // Writing output vtk.
  int num_points = voxel_count * 8;
  int num_cells = voxel_count + num_points; 

  f << "# vtk DataFile Version 1.0" << std::endl;
  f << "vtk mesh" << std::endl;
  f << "ASCII" << std::endl;
  f << "DATASET UNSTRUCTURED_GRID" << std::endl;
  f << "POINTS " << num_points << " FLOAT" << std::endl; 
  f << points.str() << std::endl; 
  f << "CELLS " << voxel_count << " " << num_cells << std::endl; 
  f << cells.str() << std::endl;
  f << "CELL_TYPES " << voxel_count << std::endl;
  std::stringstream types;
  for(int i = 0; i < voxel_count; i++) types << "11\n";
  f << types.str() << std::endl;
  f.close();
} 

void printNormals(const se::Image<Eigen::Vector3f> in, const unsigned int xdim, 
                 const unsigned int ydim, const char* filename) {
  unsigned char* image = new unsigned char [xdim * ydim * 4];
  for(unsigned int y = 0; y < ydim; ++y)
    for(unsigned int x = 0; x < xdim; ++x){
      const Eigen::Vector3f n = in[x + y*xdim];
      image[4 * xdim * y + 4 * x + 0] = (n.x()/2 + 0.5) * 255; 
      image[4 * xdim * y + 4 * x + 1] = (n.y()/2 + 0.5) * 255;
      image[4 * xdim * y + 4 * x + 2] = (n.z()/2 + 0.5) * 255;
      image[4 * xdim * y + 4 * x + 3] = 255;
    }
  lodepng_encode32_file(std::string(filename).append(".png").c_str(),
      image, xdim, ydim);
} 

// void parseGTFile(const std::string& filename, std::vector<Matrix4>& poses){
// 
//   std::ifstream file;
//   file.open(filename.c_str());
// 
//   if(!file.is_open()) {
//     std::cout << "Failed to open GT file " << filename << std::endl;
//     return;
//   }
// 
//   std::string line;
//   while (getline(file,line))
//   {
//     std::vector<std::string> data;
//     split(line, ' ', data);
// 
//     if (data.size() != 8)
//     {
//       continue;
//     }
//     const float3 trans = make_float3(std::stof(data[1]), std::stof(data[2]),
//         std::stof(data[3]));
//     const float4 quat = make_float4(std::stof(data[4]), std::stof(data[5]),
//         std::stof(data[6]), std::stof(data[7]));
//     poses.push_back(toMatrix4(quat, trans));
//   }
// }
#endif
