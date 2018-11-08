/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

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

#include <se/DenseSLAMSystem.h>
#include <se/ray_iterator.hpp>
#include <se/algorithms/meshing.hpp>
#include <se/geometry/octree_collision.hpp>
#include <se/vtk-io.h>
#include "timings.h"
#include <perfstats.h>
#include "preprocessing.cpp"
#include "tracking.cpp"
#include "rendering.cpp"
#include "bfusion/mapping_impl.hpp"
#include "kfusion/mapping_impl.hpp"
#include "bfusion/alloc_impl.hpp"
#include "kfusion/alloc_impl.hpp"


extern PerfStats Stats;

// For debugging purposes, will be deleted once done.
std::vector<Matrix4> poses;

DenseSLAMSystem::DenseSLAMSystem(uint2              inputSize,
                                 uint3              volumeResolution,
                                 float3             volumeDimensions,
                                 float3             initPose,
                                 std::vector<int> & pyramid,
                                 Configuration      config)
  : DenseSLAMSystem(inputSize, volumeResolution, volumeDimensions,
        toMatrix4(initPose), pyramid, config) { }

DenseSLAMSystem::DenseSLAMSystem(uint2              inputSize,
                                 uint3              volumeResolution,
                                 float3             volumeDimensions,
                                 Matrix4            initPose,
                                 std::vector<int> & pyramid,
                                 Configuration      config)
  : computation_size_(make_uint2(inputSize.x, inputSize.y)),
    vertex_(computation_size_.x, computation_size_.y),
    normal_(computation_size_.x, computation_size_.y),
    float_depth_(computation_size_.x, computation_size_.y),
    tracking_result_(computation_size_.x, computation_size_.y) {

    this->init_pose_ = getPosition();
    this->volume_dimension_ = volumeDimensions;
    this->volume_resolution_ = volumeResolution;
    this->mu_ = config.mu;
    pose_ = initPose;

    this->iterations_.clear();
    for (std::vector<int>::iterator it = pyramid.begin();
        it != pyramid.end(); it++) {
      this->iterations_.push_back(*it);
    }

    viewPose_ = &pose_;

    if (getenv("KERNEL_TIMINGS"))
      print_kernel_timing = true;

    // internal buffers to initialize
    reduction_output_.resize(8 * 32);

    for (unsigned int i = 0; i < iterations_.size(); ++i) {
      int downsample = 1 << i;
      scaled_depth_.push_back(se::Image<float>(computation_size_.x / downsample,
            computation_size_.y / downsample));

      input_vertex_.push_back(se::Image<float3>(computation_size_.x / downsample,
            computation_size_.y / downsample));

      input_normal_.push_back(se::Image<float3>(computation_size_.x / downsample,
            computation_size_.y / downsample));
    }

    // ********* BEGIN : Generate the gaussian *************
    size_t gaussianS = radius * 2 + 1;
    gaussian_.reserve(gaussianS);
    int x;
    for (unsigned int i = 0; i < gaussianS; i++) {
      x = i - 2;
      gaussian_[i] = expf(-(x * x) / (2 * delta * delta));
    }

    // ********* END : Generate the gaussian *************

    if (config_.groundtruth_file != "") {
      parseGTFile(config_.groundtruth_file, poses);
      std::cout << "Parsed " << poses.size() << " poses" << std::endl;
    }

    discrete_vol_ptr_ = std::make_shared<se::Octree<FieldType> >();
    discrete_vol_ptr_->init(volume_resolution_.x, volume_dimension_.x);
    volume_ = Volume<FieldType>(volume_resolution_.x, volume_dimension_.x,
        discrete_vol_ptr_.get());
}

bool DenseSLAMSystem::preprocessing(const ushort * inputDepth,
                                    const uint2    inputSize,
                                    const bool     filterInput) {

    // Scale the depth frame to the size used internaly for computation and
    // convert the depth data from millimeters to meters.
    mm2metersKernel(float_depth_.data(), computation_size_,
        inputDepth, inputSize);

    // Optionally pass the depth frame through a bilateral filter to smooth the
    // data.
    if (filterInput) {
        bilateralFilterKernel(scaled_depth_[0].data(), float_depth_.data(),
            computation_size_, gaussian_.data(), e_delta, radius);
    }
    else {
      std::memcpy(scaled_depth_[0].data(), float_depth_.data(),
          sizeof(float) * computation_size_.x * computation_size_.y);
    }
    return true;
}

bool DenseSLAMSystem::tracking(float4 k,
                               float  icp_threshold,
                               uint   tracking_rate,
                               uint   frame) {

  if (frame % tracking_rate != 0)
    return false;

  // Legacy ground truth pose loading, to be removed.
  if(!poses.empty()) {
    old_pose_ = this->pose_;
    this->pose_ = poses[frame];
    this->pose_.data[0].w = (this->pose_.data[0].w - poses[0].data[0].w) + this->init_pose_.x;
    this->pose_.data[1].w = (this->pose_.data[1].w - poses[0].data[1].w) + this->init_pose_.y;
    this->pose_.data[2].w = (this->pose_.data[2].w - poses[0].data[2].w) + this->init_pose_.z;
    return true;
  }

  // Half sample the input depth maps into the pyramid levels.
  for (unsigned int i = 1; i < iterations_.size(); ++i) {
    halfSampleRobustImageKernel(scaled_depth_[i].data(),
        scaled_depth_[i - 1].data(),
        make_uint2(computation_size_.x / (int) pow(2, i - 1),
            computation_size_.y / (int) pow(2, i - 1)), e_delta * 3, 1);
  }

  // Prepare the 3D information from the input depth maps for all pyramid
  // levels.
  uint2 localimagesize = computation_size_;
  for (unsigned int i = 0; i < iterations_.size(); ++i) {
    Matrix4 invK = getInverseCameraMatrix(k / float(1 << i));

    // Generate a 3D point for each depth frame pixel (vertex map) in
    // input_vertex_.
    depth2vertexKernel(input_vertex_[i].data(), scaled_depth_[i].data(),
        localimagesize, invK);

    // Generate a normal vector for each 3D point (normal map) in
    // input_normal_.
    if (k.y < 0)
      vertex2normalKernel<true>(input_normal_[i].data(),
          input_vertex_[i].data(), localimagesize);
    else
      vertex2normalKernel<false>(input_normal_[i].data(),
          input_vertex_[i].data(), localimagesize);

    localimagesize = make_uint2(localimagesize.x / 2, localimagesize.y / 2);
  }

  old_pose_ = pose_;
  const Matrix4 projectReference = getCameraMatrix(k) * inverse(raycast_pose_);

  // Loop over the pyramid levels starting from the lowest resolution one.
  for (int level = iterations_.size() - 1; level >= 0; --level) {
    uint2 localimagesize = make_uint2(
        computation_size_.x / (int) pow(2, level),
        computation_size_.y / (int) pow(2, level));
    // Repeat multiple times per level.
    for (int i = 0; i < iterations_[level]; ++i) {

      trackKernel(tracking_result_.data(), input_vertex_[level].data(),
          input_normal_[level].data(), localimagesize,
          vertex_.data(), normal_.data(), computation_size_,
          pose_, projectReference, dist_threshold, normal_threshold);

      reduceKernel(reduction_output_.data(), tracking_result_.data(),
          computation_size_, localimagesize);

      if (updatePoseKernel(pose_, reduction_output_.data(), icp_threshold))
        break;
    }
  }
  return checkPoseKernel(pose_, old_pose_, reduction_output_.data(),
      computation_size_, track_threshold);
}

bool DenseSLAMSystem::raycasting(float4 k,
                                 float  mu,
                                 uint   frame) {

  bool doRaycast = false;

  if(frame > 2) {
    raycast_pose_ = pose_;
    float step = volume_dimension_.x / volume_resolution_.x;
    // Raycast to create the vertex (vertex_) and normal (normal_) maps from
    // the current pose.
    raycastKernel(volume_, vertex_.data(), normal_.data(), computation_size_,
        raycast_pose_ * getInverseCameraMatrix(k), nearPlane, farPlane, mu,
        step, step*BLOCK_SIDE);
    doRaycast = true;
  }
  return doRaycast;
}

bool DenseSLAMSystem::integration(float4 k,
                                  uint   integration_rate,
                                  float  mu,
                                  uint   frame) {

  // Integrate only if ground truth is used or if tracking was successful.
  bool doIntegrate = poses.empty() ? checkPoseKernel(pose_, old_pose_,
      reduction_output_.data(), computation_size_, track_threshold) : true;

  if ((doIntegrate && ((frame % integration_rate) == 0)) || (frame <= 3)) {

    float voxelsize =  volume_._dim/volume_._size;
    int num_vox_per_pix = volume_._dim/((se::VoxelBlock<FieldType>::side)*voxelsize);
    size_t total = num_vox_per_pix * computation_size_.x * computation_size_.y;
    allocation_list_.reserve(total);

    // Allocation for octree.
    unsigned int allocated = 0;
    if(std::is_same<FieldType, SDF>::value) {
      allocated  = buildAllocationList(allocation_list_.data(),
          allocation_list_.capacity(),
          *volume_._map_index, pose_, getCameraMatrix(k), float_depth_.data(),
          computation_size_, volume_._size,
          voxelsize, 2*mu);
    } else if(std::is_same<FieldType, OFusion>::value) {
      allocated = buildOctantList(allocation_list_.data(), allocation_list_.capacity(),
          *volume_._map_index,
          pose_, getCameraMatrix(k), float_depth_.data(), computation_size_, voxelsize,
          compute_stepsize, step_to_depth, 6*mu);
    }

    volume_._map_index->allocate(allocation_list_.data(), allocated);

    // Update octree data.
    if (std::is_same<FieldType, SDF>::value) {
      struct sdf_update funct(float_depth_.data(),
          Eigen::Vector2i(computation_size_.x, computation_size_.y), mu, 100);
      se::functor::projective_map(*volume_._map_index,
          to_sophus(pose_).inverse(),
          to_eigen(getCameraMatrix(k)),
          Eigen::Vector2i(computation_size_.x, computation_size_.y),
          funct);
    } else if (std::is_same<FieldType, OFusion>::value) {

      float timestamp = (1.f/30.f)*frame;
      struct bfusion_update funct(float_depth_.data(),
          Eigen::Vector2i(computation_size_.x, computation_size_.y), mu, timestamp);

      se::functor::projective_map(*volume_._map_index,
          to_sophus(pose_).inverse(),
          to_eigen(getCameraMatrix(k)),
          Eigen::Vector2i(computation_size_.x, computation_size_.y),
          funct);
    }

    // if(frame % 15 == 0) {
    //   std::stringstream f;
    //   f << "./slices/integration_" << frame << ".vtk";
    //   save3DSlice(volume_._map_index, make_int3(0, volume_._size/2, 0),
    //       make_int3(volume_._size, volume_._size/2 + 1, volume_._size), make_int3(volume_._size), f.str().c_str());
    //   f.str("");
    //   f.clear();
    //   }

    // f << "./slices/collision_" << frame << ".vtk";
    // save3DSlice(volume_._map_index, [](const Octree<FieldType>& map,
    //       const int x, const int y, const int z) {
    //       const int3 bbox = make_int3(x, y, z);
    //       const int3 side = make_int3(1);
    //       auto test = [](const Octree<FieldType>::value_type & val) {
    //       if(val.x == 0.f) return collision_status::unseen;
    //       auto test = [](const Octree<FieldType>::value_type & val) {
    //       if(val.x == 0.f) return collision_status::unseen;
    //       if(val.x < 5) return collision_status::empty;
    //       return collision_status::occupied;
    //       };
    //       return (float) collides_with(map, bbox, side, test);
    //     }, // end lambda
    //     make_int3(0, volume_._size/2, 0),
    //     make_int3(volume_._size, volume_._size/2 + 1, volume_._size), make_int3(volume_._size), f.str().c_str());
    doIntegrate = true;
  } else {
    doIntegrate = false;
  }

  return doIntegrate;
}

void DenseSLAMSystem::dump_volume(std::string ) {

}

void DenseSLAMSystem::renderVolume(uchar4 * out,
                                   uint2    outputSize,
                                   int      frame,
                                   int      raycast_rendering_rate,
                                   float4   k,
                                   float    largestep) {
  if (frame % raycast_rendering_rate == 0) {
    const float step = volume_dimension_.x / volume_resolution_.x;
    renderVolumeKernel(volume_, out, outputSize,
        *(this->viewPose_) * getInverseCameraMatrix(k), nearPlane,
        farPlane * 2.0f, mu_, step, largestep,
        get_translation(*(this->viewPose_)), ambient,
        !compareMatrix4(*(this->viewPose_), raycast_pose_), vertex_.data(),
        normal_.data());
  }
}

void DenseSLAMSystem::renderTrack(uchar4 * out, uint2 outputSize) {
  renderTrackKernel(out, tracking_result_.data(), outputSize);
}

void DenseSLAMSystem::renderDepth(uchar4 * out, uint2 outputSize) {
  renderDepthKernel(out, float_depth_.data(), outputSize, nearPlane, farPlane);
}

void DenseSLAMSystem::dump_mesh(const std::string filename){

  std::vector<Triangle> mesh;
  auto inside = [](const Volume<FieldType>::value_type& val) {
    // meshing::status code;
    // if(val.y == 0.f) 
    //   code = meshing::status::UNKNOWN;
    // else 
    //   code = val.x < 0.f ? meshing::status::INSIDE : meshing::status::OUTSIDE;
    // return code;
    // std::cerr << val.x << " ";
    return val.x < 0.f;
  };

  auto select = [](const Volume<FieldType>::value_type& val) {
    return val.x;
  };

  se::algorithms::marching_cube(*volume_._map_index, select, inside, mesh);
  writeVtkMesh(filename.c_str(), mesh);
}
