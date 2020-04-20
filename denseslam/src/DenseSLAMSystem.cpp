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
      tracker_(pyramid, computation_size_, initPose),
      backend_(volumeResolution.x(), volumeDimensions.x()) {
    volume_dimension_  = volumeDimensions.x();
    volume_resolution_ = volumeResolution.x();
    mu_                = config.mu;

    raycast_pose_ = initPose;

    // Generate the gaussian
    size_t gaussian_size = radius * 2 + 1;
    gaussian_.reserve(gaussian_size);
    for (unsigned i = 0; i < gaussian_size; i++) {
        int x        = i - 2;
        gaussian_[i] = expf(-(x * x) / (2 * delta * delta));
    }
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
    backend_.raycast(vertex_, normal_, k, raycast_pose_, mu);

    return true;
}

bool DenseSLAMSystem::integration(const Eigen::Vector4f& k,
    unsigned int integration_rate, float mu, unsigned int frame) {
    if (frame > 3 && frame % integration_rate != 0) return false;

    backend_.integrate(
        float_depth_, k, tracker_.getPose(), computation_size_, mu, frame);

    return true;
}

void DenseSLAMSystem::dump_volume(std::string) {}

void DenseSLAMSystem::renderVolume(unsigned char* out,
    const Eigen::Vector2i& outputSize, int frame, int raycast_rendering_rate,
    const Eigen::Vector4f& k, float largestep) {
    if (frame % raycast_rendering_rate != 0) return;

    Eigen::Matrix4f view_pose = getViewPose();
    backend_.render(out, outputSize, k, view_pose, largestep, mu_, vertex_,
        normal_, raycast_pose_);
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
    /*
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
    */
}
