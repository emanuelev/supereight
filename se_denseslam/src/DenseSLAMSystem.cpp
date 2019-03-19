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
static bool print_kernel_timing = false;

DenseSLAMSystem::DenseSLAMSystem(const Eigen::Vector2i& input_size,
                                 const Eigen::Vector3i& volume_resolution,
                                 const Eigen::Vector3f& volume_dimensions,
			                           const Eigen::Vector3f& init_pose,
                                 std::vector<int> & pyramid,
                                 const Configuration& config):
      DenseSLAMSystem(input_size, volume_resolution, volume_dimensions,
          se::math::toMatrix4f(init_pose), pyramid, config) { }

DenseSLAMSystem::DenseSLAMSystem(const Eigen::Vector2i& input_size,
                                 const Eigen::Vector3i& volume_resolution,
                                 const Eigen::Vector3f& volume_dimensions,
                                 const Eigen::Matrix4f& init_pose,
                                 std::vector<int> & pyramid,
                                 const Configuration& config) :
  computation_size_(input_size),
  vertex_(computation_size_.x(), computation_size_.y()),
  normal_(computation_size_.x(), computation_size_.y()),
  float_depth_(computation_size_.x(), computation_size_.y())
  {

    this->init_pose_ = init_pose.block<3,1>(0,3);
    this->volume_dimension_ = volume_dimensions;
    this->volume_resolution_ = volume_resolution;
    this->mu_ = config.mu;
    pose_ = init_pose;
    raycast_pose_ = init_pose;

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
    tracking_result_.resize(computation_size_.x() * computation_size_.y());

    for (unsigned int i = 0; i < iterations_.size(); ++i) {
      int downsample = 1 << i;
      scaled_depth_.push_back(se::Image<float>(computation_size_.x() / downsample,
            computation_size_.y() / downsample));

      input_vertex_.push_back(se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
            computation_size_.y() / downsample));

      input_normal_.push_back(se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
            computation_size_.y() / downsample));
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

    discrete_vol_ptr_ = std::make_shared<se::Octree<FieldType> >();
    discrete_vol_ptr_->init(volume_resolution_.x(), volume_dimension_.x());
    volume_ = Volume<FieldType>(volume_resolution_.x(), volume_dimension_.x(),
        discrete_vol_ptr_.get());
}

bool DenseSLAMSystem::preprocessing(const unsigned short * input_depth,
    const Eigen::Vector2i& input_size, const bool filter_input){

    mm2metersKernel(float_depth_, input_depth, input_size);
    if(filter_input){
        bilateralFilterKernel(scaled_depth_[0], float_depth_, gaussian_,
            e_delta, radius);
    }
    else {
      std::memcpy(scaled_depth_[0].data(), float_depth_.data(),
          sizeof(float) * computation_size_.x() * computation_size_.y());
    }
	return true;
}

bool DenseSLAMSystem::tracking(const Eigen::Vector4f& k,
    float icp_threshold, unsigned tracking_rate, unsigned frame) {

	if (frame % tracking_rate != 0)
		return false;

	// half sample the input depth maps into the pyramid levels
	for (unsigned int i = 1; i < iterations_.size(); ++i) {
		halfSampleRobustImageKernel(scaled_depth_[i], scaled_depth_[i - 1], e_delta * 3, 1);
	}

	// prepare the 3D information from the input depth maps
  Eigen::Vector2i local_image_size = computation_size_;
	for (unsigned int i = 0; i < iterations_.size(); ++i) {
    Eigen::Matrix4f invK = getInverseCameraMatrix(k / float(1 << i));
		depth2vertexKernel(input_vertex_[i], scaled_depth_[i], invK);
    if(k.y() < 0)
      vertex2normalKernel<true>(input_normal_[i], input_vertex_[i]);
    else
      vertex2normalKernel<false>(input_normal_[i], input_vertex_[i]);
		local_image_size /= 2;;
	}

	old_pose_ = pose_;
	const Eigen::Matrix4f projectReference = getCameraMatrix(k) * raycast_pose_.inverse();

	for (int level = iterations_.size() - 1; level >= 0; --level) {
    Eigen::Vector2i local_image_size(
				computation_size_.x() / (int) pow(2, level),
				computation_size_.y() / (int) pow(2, level));
		for (int i = 0; i < iterations_[level]; ++i) {

      trackKernel(tracking_result_.data(), input_vertex_[level], input_normal_[level],
          vertex_, normal_, pose_, projectReference,
          dist_threshold, normal_threshold);

			reduceKernel(reduction_output_.data(), tracking_result_.data(), computation_size_,
					local_image_size);

			if (updatePoseKernel(pose_, reduction_output_.data(), icp_threshold))
				break;

		}
	}
	return checkPoseKernel(pose_, old_pose_, reduction_output_.data(),
      computation_size_, track_threshold);
}

bool DenseSLAMSystem::raycasting(const Eigen::Vector4f& k, float mu, unsigned int frame) {

  bool do_raycast = false;

  if(frame > 2) {
    raycast_pose_ = pose_;
    float step = volume_dimension_.x() / volume_resolution_.x();
    raycastKernel(volume_, vertex_, normal_,
        raycast_pose_ * getInverseCameraMatrix(k), near_plane,
        far_plane, mu, step, step*BLOCK_SIDE);
    do_raycast = true;
  }
  return do_raycast;
}

bool DenseSLAMSystem::integration(const Eigen::Vector4f& k, unsigned int integration_rate,
    float mu, unsigned int frame) {

  if (((frame % integration_rate) == 0) || (frame <= 3)) {

    float voxel_size =  volume_.extent_/volume_.size_;
    int num_vox_per_pix = volume_.extent_/((se::VoxelBlock<FieldType>::side)*voxel_size);
    size_t total = num_vox_per_pix * computation_size_.x() *
      computation_size_.y();
    allocation_list_.reserve(total);

    unsigned int allocated = 0;
    if(std::is_same<FieldType, SDF>::value) {
     allocated  = buildAllocationList(allocation_list_.data(),
         allocation_list_.capacity(),
        *volume_.map_index_, pose_, getCameraMatrix(k), float_depth_.data(),
        computation_size_, volume_.size_,
      voxel_size, 2*mu);
    } else if(std::is_same<FieldType, OFusion>::value) {
     allocated = buildOctantList(allocation_list_.data(), allocation_list_.capacity(),
         *volume_.map_index_,
         pose_, getCameraMatrix(k), float_depth_.data(), computation_size_, voxel_size,
         compute_stepsize, step_to_depth, 6*mu);
    }

    volume_.map_index_->allocate(allocation_list_.data(), allocated);

    if(std::is_same<FieldType, SDF>::value) {
      struct sdf_update funct(float_depth_.data(),
          Eigen::Vector2i(computation_size_.x(), computation_size_.y()), mu, 100);
      se::functor::projective_map(*volume_.map_index_,
          Sophus::SE3f(pose_).inverse(),
          getCameraMatrix(k),
          Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
          funct);
    } else if(std::is_same<FieldType, OFusion>::value) {

      float timestamp = (1.f/30.f)*frame;
      struct bfusion_update funct(float_depth_.data(),
          Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
          mu, timestamp, voxel_size);

      se::functor::projective_map(*volume_.map_index_,
          Sophus::SE3f(pose_).inverse(),
          getCameraMatrix(k),
          Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
          funct);
    }

    // if(frame % 15 == 0) {
    //   std::stringstream f;
    //   f << "./slices/integration_" << frame << ".vtk";
    //   save3DSlice(*volume_.map_index_, Eigen::Vector3i(0, 200, 0),
    //       Eigen::Vector3i(volume_.size_, 201, volume_.size_),
    //       Eigen::Vector3i::Constant(volume_.size_), f.str().c_str());
    //   f.str("");
    //   f.clear();
    // }
  } else {
    return false;
  }
  return true;
}

void DenseSLAMSystem::dumpVolume(std::string ) {

}

void DenseSLAMSystem::renderVolume(unsigned char* out,
    const Eigen::Vector2i& output_size,
    int frame,
		int raycast_rendering_rate,
    const Eigen::Vector4f& k,
    float large_step) {

	if (frame % raycast_rendering_rate == 0) {
    const float step = volume_dimension_.x() / volume_resolution_.x();
		renderVolumeKernel(volume_, out, output_size,
	*(this->viewPose_) * getInverseCameraMatrix(k), near_plane,
	far_plane * 2.0f, mu_, step, large_step,
        this->viewPose_->topRightCorner<3, 1>(), ambient,
        !(this->viewPose_->isApprox(raycast_pose_)), vertex_,
        normal_);
  }
}

void DenseSLAMSystem::renderTrack(unsigned char* out,
    const Eigen::Vector2i& output_size) {
        renderTrackKernel(out, tracking_result_.data(), output_size);
}

void DenseSLAMSystem::renderDepth(unsigned char* out,
    const Eigen::Vector2i& output_size) {
        renderDepthKernel(out, float_depth_.data(), output_size, near_plane, far_plane);
}

void DenseSLAMSystem::dumpMesh(const std::string filename){

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

  se::algorithms::marching_cube(*volume_.map_index_, select, inside, mesh);
  writeVtkMesh(filename.c_str(), mesh);
}
