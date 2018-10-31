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

#ifndef _KERNELS_
#define _KERNELS_

#include <cstdlib>
#include <se/commons.h>
#include <iostream>
#include <memory>
#include <perfstats.h>
#include <timings.h>
#include <se/config.h>
#include <se/octree.hpp>
#include <se/image/image.hpp>
#include "volume_traits.hpp"
#include "continuous/volume_template.hpp"

/*
 * Use SE_FIELD_TYPE macro to define the DenseSLAMSystem instance.
 */
typedef SE_FIELD_TYPE FieldType;
template <typename T>
using Volume = VolumeTemplate<T, se::Octree>;

class DenseSLAMSystem {

  private:
    uint2 computation_size_;
    Matrix4 pose_;
    Matrix4 *viewPose_;
    float3 volume_dimension_;
    uint3 volume_resolution_;
    std::vector<int> iterations_;
    bool tracked_;
    bool integrated_;
    float3 init_pose_;
    float mu_;
    bool need_render_ = false;
    Configuration config_;

    // input once
    std::vector<float> gaussian_;

    // inter-frame
    se::Image<Eigen::Vector3f> vertex_;
    se::Image<Eigen::Vector3f> normal_;

    std::vector<se::key_t> allocation_list_;
    std::shared_ptr<se::Octree<FieldType> > discrete_vol_ptr_;
    Volume<FieldType> volume_;

    // intra-frame
    std::vector<float> reduction_output_;
    std::vector<se::Image<float>  > scaled_depth_;
    std::vector<se::Image<Eigen::Vector3f> > input_vertex_;
    std::vector<se::Image<Eigen::Vector3f> > input_normal_;
    se::Image<float> float_depth_;
    std::vector<TrackData>  tracking_result_;
    Matrix4 old_pose_;
    Matrix4 raycast_pose_;

  public:

    /**
     * Constructor using the initial camera position.
     *
     * \param[in] inputSize The size (width and height) of the input frames.
     * \param[in] volume_resolution_ The x, y and z resolution of the
     * reconstructed volume in voxels.
     * \param[in] volume_dimension_ The x, y and z dimensions of the
     * reconstructed volume in meters.
     * \param[in] initPose The x, y and z coordinates of the initial camera
     * position. The camera orientation is assumed to be aligned with the axes.
     * \param[in] pyramid TODO See ::Configuration.pyramid for more details.
     * \param[in] config_ The pipeline options.
     */
    DenseSLAMSystem(uint2              inputSize,
                    uint3              volume_resolution_,
                    float3             volume_dimension_,
                    float3             initPose,
                    std::vector<int> & pyramid,
                    Configuration      config_);
    /**
     * Constructor using the initial camera position.
     *
     * \param[in] inputSize The size (width and height) of the input frames.
     * \param[in] volume_resolution_ The x, y and z resolution of the
     * reconstructed volume in voxels.
     * \param[in] volume_dimension_ The x, y and z dimensions of the
     * reconstructed volume in meters.
     * \param[in] initPose The initial camera pose encoded in a 4x4 matrix.
     * \param[in] pyramid TODO See ::Configuration.pyramid for more details.
     * \param[in] config_ The pipeline options.
     */
    DenseSLAMSystem(uint2              inputSize,
                    uint3              volume_resolution_,
                    float3             volume_dimension_,
                    Matrix4            initPose,
                    std::vector<int> & pyramid,
                    Configuration      config_);

    /**
     * Preprocess a single depth measurement frame and add it to the pipeline.
     * This is the first stage of the pipeline.
     *
     * \param[in] inputDepth Pointer to the depth frame.
     * \param[in] inputSize Size of the depth frame in pixels (width and
     * height).
     * \param[in] filterInput Whether to filter the frame using a bilateral
     * filter to reduce the measurement noise.
     * \return true (does not fail).
     */
    bool preprocessing(const ushort * inputDepth,
                       const uint2    inputSize,
                       const bool     filterInput);

    /*
     * TODO Implement this.
     */
    bool preprocessing(const ushort * inputDepth,
                       const uchar3 * inputRGB,
                       const uint2    inputSize,
                       const bool     filterInput);

    /**
     * Update the camera pose. Create a 3D reconstruction from the current
     * depth frame and compute a transformation using ICP. The 3D
     * reconstruction of the current frame is matched to the 3D reconstruction
     * obtained from all of the previous frames. This is the second stage of
     * the pipeline.
     *
     * \param[in] k The intrinsic camera parameters. See
     * ::Configuration.camera for details.
     * \param[in] icp_threshold The ICP convergence threshold.
     * \param[in] tracking_rate Process a frame every tracking_rate frames.
     * \param[in] frame The index of the current frame (starts from 0).
     * \return true if the camera pose was updated and false if it wasn't.
     */
    bool tracking(float4 k,
                  float  icp_threshold,
                  uint   tracking_rate,
                  uint   frame);

    /**
     * Integrate the 3D reconstruction resulting from the current frame to the
     * existing reconstruction. This is the third stage of the pipeline.
     *
     * \param[in] k The intrinsic camera parameters. See
     * ::Configuration.camera for details.
     * \param[in] integration_rate Integrate a 3D reconstruction every
     * integration_rate frames. Should not be less than the tracking_rate used
     * in tracking().
     * \param[in] mu TSDF truncation bound. See ::Configuration.mu for more
     * details.
     * \param[in] frame The index of the current frame (starts from 0).
     * \return true if the current 3D reconstruction was added to the octree
     * and false if it wasn't.
     */
    bool integration(float4 k,
                     uint   integration_rate,
                     float  mu,
                     uint   frame);

    /**
     * Raycast the 3D reconstruction after integration to update the values of
     * the TSDF. This is the fourth stage of the pipeline.
     *
     * @note Raycast is not performed on the first 3 frames (those with an
     * index up to 2).
     *
     * \param[in] k The intrinsic camera parameters. See
     * ::Configuration.camera for details.
     * \param[in] mu TSDF truncation bound. See ::Configuration.mu for more
     * details.
     * \param[in] frame The index of the current frame (starts from 0).
     * \return true if raycasting was performed and false if it wasn't.
     */
    bool raycasting(float4 k,
                    float  mu,
                    uint   frame);

    /*
     * TODO Implement this.
     */
    void dump_volume(const std::string filename);

    /*
     * TODO Document this.
     */
    void dump_mesh(const std::string filename);

    /**
     * Render the current 3D reconstruction.
     *
     * \param[out] out A pointer to an array containing the rendered frame.
     * The array must be allocated before calling this function. The x, y and
     * z members of each element of the array contain the R, G and B values of
     * the image respectively. The w member of each element of the array is
     * always 0 and is used for padding.
     * \param[in] outputSize The dimensions of the output array (width and
     * height in pixels).
     * \param[in] frame The index of the current frame (starts from 0).
     * \param[in] rate Render the 3D reconstruction every rate frames.
     * \param[in] k The intrinsic camera parameters. See
     * ::Configuration.camera for details.
     * \param[in] mu TSDF truncation bound. See ::Configuration.mu for more
     * details.
     */
    void renderVolume(uchar4 *    out,
                      const uint2 outputSize,
                      int         frame,
                      int         rate,
                      float4      k,
                      float       mu);

    /**
     * Render the output of the tracking algorithm. The meaning of the colors is as follows:
     * | Color  | Meaning |
     * | ------ | ------- |
     * | grey   | Successful tracking. |
     * | black  | No input data. |
     * | red    | Not in image. |
     * | green  | No correspondence. |
     * | blue   | Too far away. |
     * | yellow | Wrong normal. |
     * | orange | Tracking not performed. |
     *
     * \param[out] out A pointer to an array containing the rendered frame.
     * The array must be allocated before calling this function. The x, y and
     * z members of each element of the array contain the R, G and B values of
     * the image respectively. The w member of each element of the array is
     * always 0 and is used for padding.
     * \param[in] outputSize The dimensions of the output array (width and
     * height in pixels).
     */
    void renderTrack(uchar4 *    out,
                     const uint2 outputSize);

    /**
     * Render the current depth frame. The frame is rendered before
     * preprocessing while taking into account the values of ::nearPlane and
     * ::farPlane. Regions closer to the camera than ::nearPlane appear white
     * and regions further than ::farPlane appear black.
     *
     * \param[out] out A pointer to an array containing the rendered frame.
     * The array must be allocated before calling this function. The x, y and
     * z members of each element of the array contain the R, G and B values of
     * the image respectively. The w member of each element of the array is
     * always 0 and is used for padding.
     * \param[in] outputSize The dimensions of the output array (width and
     * height in pixels).
     */
    void renderDepth(uchar4* out,
                     uint2   outputSize);

    //
    // Getters
    //

    /*
     * TODO Document this.
     */
    void getMap(std::shared_ptr<se::Octree<FieldType> >& out) {
      out = discrete_vol_ptr_;
    }

    /*
     * TODO Document this.
     */
    bool getTracked() {
      return (tracked_);
    }

    /*
     * TODO Document this.
     */
    bool getIntegrated() {
      return (integrated_);
    }

    /**
     * Get the current camera position.
     *
     * \return A vector containing the x, y and z coordinates of the camera.
     */
    float3 getPosition() {
      //std::cerr << "InitPose =" << _initPose.x << "," << _initPose.y  <<"," << _initPose.z << "    ";
      //std::cerr << "pose =" << pose.data[0].w << "," << pose.data[1].w  <<"," << pose.data[2].w << "    ";
      float xt = pose_.data[0].w - init_pose_.x;
      float yt = pose_.data[1].w - init_pose_.y;
      float zt = pose_.data[2].w - init_pose_.z;
      return (make_float3(xt, yt, zt));
    }

    /**
     * Get the initial camera position.
     *
     * \return A vector containing the x, y and z coordinates of the camera.
     */
    float3 getInitPos(){
      return init_pose_;
    }

    /**
     * Get the current camera pose.
     *
     * \return The current camera pose encoded in a 4x4 matrix.
     */
    Matrix4 getPose() {
      return pose_;
    }

    /**
     * Set the camera pose used to render the 3D reconstruction.
     *
     * \param[in] value The desired camera pose encoded in a 4x4 matrix.
     */
    void setViewPose(Matrix4 *value = NULL) {
      if (value == NULL){
        viewPose_ = &pose_;
        need_render_ = false;
      }
      else {
        viewPose_ = value;
        need_render_ = true;
      }
    }

    /**
     * Get the camera pose used to render the 3D reconstruction. The default
     * is the current frame's camera pose.
     *
     * \return The current camera pose encoded in a 4x4 matrix.
     */
    Matrix4 *getViewPose() {
      return (viewPose_);
    }

    /**
     * Get the dimensions of the reconstructed volume in meters.
     *
     * \return A vector containing the x, y and z dimensions of the volume.
     */
    float3 getModelDimensions() {
      return (volume_dimension_);
    }

    /**
     * Get the resolution of the reconstructed volume in voxels.
     *
     * \return A vector containing the x, y and z resolution of the volume.
     */
    uint3 getModelResolution() {
      return (volume_resolution_);
    }

    /**
     * Get the resolution used when processing frames in the pipeline in
     * pixels.
     *
     * \return A vector containing the frame width and height.
     */
    uint2 getComputationResolution() {
      return (computation_size_);
    }
};

/**
 * Synchronize CPU and GPU.
 *
 * @note This function does nothing in the C++ implementation.
 */
void synchroniseDevices();

#endif
