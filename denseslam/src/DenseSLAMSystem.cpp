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

#include <supereight/algorithms/meshing.hpp>
#include <supereight/denseslam/DenseSLAMSystem.h>
#include <supereight/geometry/octree_collision.hpp>
#include <supereight/ray_iterator.hpp>
#include <supereight/shared/perfstats.h>
#include <supereight/shared/timings.h>

#include <supereight/tracking/tracker.hpp>

#include "preprocessing.cpp"
#include "rendering.cpp"

#include "bfusion/alloc_impl.hpp"
#include "bfusion/mapping_impl.hpp"

#include "kfusion/alloc_impl.hpp"
#include "kfusion/mapping_impl.hpp"

DenseSLAMSystem::DenseSLAMSystem(const Eigen::Vector2i& inputSize,
    const Eigen::Vector3i& volumeResolution,
    const Eigen::Vector3f& volumeDimensions, const Eigen::Vector3f& initPose,
    std::vector<int>& pyramid, const Configuration& config)
    : DenseSLAMSystem(inputSize, volumeResolution, volumeDimensions,
          se::math::toMatrix4f(initPose), pyramid, config) {}

DenseSLAMSystem::DenseSLAMSystem(const Eigen::Vector2i& inputSize,
    const Eigen::Vector3i& volumeResolution,
    const Eigen::Vector3f& volumeDimensions, const Eigen::Matrix4f& initPose,
    std::vector<int>& pyramid, const Configuration& config)
    : computation_size_(inputSize),
      vertex_(computation_size_.x(), computation_size_.y()),
      normal_(computation_size_.x(), computation_size_.y()),
      float_depth_(computation_size_.x(), computation_size_.y()),
      float_depth_filtered_(computation_size_.x(), computation_size_.y()),
      tracker_(pyramid, computation_size_, initPose) {
    volume_dimension_  = volumeDimensions.x();
    volume_resolution_ = volumeResolution.y();
    mu_                = config.mu;

    raycast_pose_ = initPose;

    // Generate the gaussian
    size_t gaussian_size = radius * 2 + 1;
    gaussian_.reserve(gaussian_size);
    for (unsigned i = 0; i < gaussian_size; i++) {
        int x        = i - 2;
        gaussian_[i] = expf(-(x * x) / (2 * delta * delta));
    }

    octree_ = std::make_shared<se::Octree<FieldType>>();
    octree_->init(volume_resolution_, volume_dimension_);
}

bool DenseSLAMSystem::preprocessing(const unsigned short* inputDepth,
    const Eigen::Vector2i& inputSize, const bool filterInput) {
    mm2metersKernel(float_depth_, inputDepth, inputSize);

    if (filterInput) {
        bilateralFilterKernel(
            float_depth_filtered_, float_depth_, gaussian_, e_delta, radius);
    } else {
        std::memcpy(float_depth_filtered_.data(), float_depth_.data(),
            sizeof(float) * computation_size_.x() * computation_size_.y());
    }

    return true;
}

bool DenseSLAMSystem::tracking(const Eigen::Vector4f& k, float icp_threshold,
    unsigned tracking_rate, unsigned frame) {
    if (frame % tracking_rate != 0) return false;
    return tracker_.track(k, float_depth_filtered_, icp_threshold,
        raycast_pose_, vertex_, normal_);
}

bool DenseSLAMSystem::raycasting(
    const Eigen::Vector4f& k, float mu, unsigned int frame) {
    if (frame <= 2) return false;

    raycast_pose_ = tracker_.getPose();
    float step    = volume_dimension_ / volume_resolution_;

    raycastKernel(*octree_, vertex_, normal_,
        raycast_pose_ * getInverseCameraMatrix(k), nearPlane, farPlane, mu,
        step, step * BLOCK_SIDE);

    return true;
}

bool DenseSLAMSystem::integration(const Eigen::Vector4f& k,
    unsigned int integration_rate, float mu, unsigned int frame) {
    if (frame > 3 && frame % integration_rate != 0) return false;

    float voxelsize = volume_dimension_ / volume_resolution_;
    int num_vox_per_pix =
        volume_dimension_ / ((se::VoxelBlock<FieldType>::side) * voxelsize);
    size_t total =
        num_vox_per_pix * computation_size_.x() * computation_size_.y();
    allocation_list_.reserve(total);

    unsigned int allocated = 0;
    if (std::is_same<FieldType, SDF>::value) {
        allocated = buildAllocationList(allocation_list_.data(),
            allocation_list_.capacity(), *octree_, tracker_.getPose(),
            getCameraMatrix(k), float_depth_.data(), computation_size_,
            volume_resolution_, voxelsize, 2 * mu);
    } else if (std::is_same<FieldType, OFusion>::value) {
        allocated = buildOctantList(allocation_list_.data(),
            allocation_list_.capacity(), *octree_, tracker_.getPose(),
            getCameraMatrix(k), float_depth_.data(), computation_size_,
            voxelsize, compute_stepsize, step_to_depth, mu);
    }

    octree_->allocate(allocation_list_.data(), allocated);

    if (std::is_same<FieldType, SDF>::value) {
        struct sdf_update funct(float_depth_.data(),
            Eigen::Vector2i(computation_size_.x(), computation_size_.y()), mu,
            100);

        se::functor::projective_map(*octree_,
            Sophus::SE3f(tracker_.getPose()).inverse(), getCameraMatrix(k),
            Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
            funct);
    } else if (std::is_same<FieldType, OFusion>::value) {
        float timestamp = (1.f / 30.f) * frame;
        struct bfusion_update funct(float_depth_.data(),
            Eigen::Vector2i(computation_size_.x(), computation_size_.y()), mu,
            timestamp, voxelsize);

        se::functor::projective_map(*octree_,
            Sophus::SE3f(tracker_.getPose()).inverse(), getCameraMatrix(k),
            Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
            funct);
    }

    /*
    if (frame % 15 == 0) {
        std::stringstream f;
        f << "./slices/integration_" << frame << ".vtk";

        save3DSlice(*volume_._map_index, Eigen::Vector3i(0, 200, 0),
            Eigen::Vector3i(volume_._size, 201, volume_._size),
            Eigen::Vector3i::Constant(volume_._size), f.str().c_str());

        f.str("");
        f.clear();
    }
    */

    return true;
}

void DenseSLAMSystem::dump_volume(std::string) {}

void DenseSLAMSystem::renderVolume(unsigned char* out,
    const Eigen::Vector2i& outputSize, int frame, int raycast_rendering_rate,
    const Eigen::Vector4f& k, float largestep) {
    if (frame % raycast_rendering_rate != 0) return;

    Eigen::Matrix4f view_pose = getViewPose();
    const float step          = volume_dimension_ / volume_resolution_;
    renderVolumeKernel(*octree_, out, outputSize,
        view_pose * getInverseCameraMatrix(k), nearPlane, farPlane * 2.0f, mu_,
        step, largestep, view_pose.topRightCorner<3, 1>(), ambient,
        !(view_pose.isApprox(raycast_pose_)), vertex_, normal_);
}

void DenseSLAMSystem::renderTrack(
    unsigned char* out, const Eigen::Vector2i& outputSize) {
    tracker_.renderTrack(out, outputSize);
}

void DenseSLAMSystem::renderDepth(
    unsigned char* out, const Eigen::Vector2i& outputSize) {
    renderDepthKernel(
        out, float_depth_.data(), outputSize, nearPlane, farPlane);
}

void DenseSLAMSystem::dump_mesh(const std::string filename) {
    std::vector<Triangle> mesh;
    auto inside = [](const auto& val) {
        // meshing::status code;
        // if(val.y == 0.f)
        //   code = meshing::status::UNKNOWN;
        // else
        //   code = val.x < 0.f ? meshing::status::INSIDE :
        //   meshing::status::OUTSIDE;
        // return code;
        // std::cerr << val.x << " ";

        return val.x < 0.f;
    };

    auto select = [](const auto& val) { return val.x; };

    se::algorithms::marching_cube(*octree_, select, inside, mesh);
    writeVtkMesh(filename.c_str(), mesh);
}
