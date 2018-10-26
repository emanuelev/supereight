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
    se::Image<float3> vertex_;
    se::Image<float3> normal_;

    std::vector<se::key_t> allocation_list_;
    std::shared_ptr<se::Octree<FieldType> > discrete_vol_ptr_;
    Volume<FieldType> volume_;

    // intra-frame
    std::vector<float> reduction_output_;
    std::vector<se::Image<float>  > scaled_depth_;
    std::vector<se::Image<float3> > input_vertex_;
    std::vector<se::Image<float3> > input_normal_;
    se::Image<float> float_depth_;
    se::Image<TrackData>  tracking_result_;
    Matrix4 old_pose_;
    Matrix4 raycast_pose_;

  public:

    DenseSLAMSystem(uint2 inputSize, uint3 volume_resolution_,
        float3 volume_dimension_, float3 initPose, std::vector<int> & pyramid,
        Configuration config_);

    DenseSLAMSystem(uint2 inputSize, uint3 volume_resolution_,
        float3 volume_dimension_, Matrix4 initPose, std::vector<int> & pyramid,
        Configuration config_);

    void languageSpecificConstructor();

    ~DenseSLAMSystem();

    bool preprocessing(const ushort * inputDepth, const uint2 inputSize, 
        const bool filterInput);

    bool preprocessing(const ushort * inputDepth, const uchar3 * inputRGB,
        const uint2 inputSize, const bool filterInput);

    bool tracking(float4 k, float icp_threshold, uint tracking_rate,
        uint frame);
    bool raycasting(float4 k, float mu, uint frame);
    bool integration(float4 k, uint integration_rate, float mu, uint frame);

    void dump_volume(const std::string filename);
    void dump_mesh(const std::string filename);

    void renderVolume(uchar4 * out, const uint2 outputSize, int frame, int rate,
        float4 k, float mu);
    void renderTrack(uchar4 * out, const uint2 outputSize);
    void renderDepth(uchar4* out, uint2 outputSize);

    //
    // Getters
    //

    void getMap(std::shared_ptr<se::Octree<FieldType> >& out) {
      out = discrete_vol_ptr_;
    }

    bool getTracked() {
      return (tracked_);
    }

    bool getIntegrated() {
      return (integrated_);
    }

    float3 getPosition() {
      //std::cerr << "InitPose =" << _initPose.x << "," << _initPose.y  <<"," << _initPose.z << "    ";
      //std::cerr << "pose =" << pose.data[0].w << "," << pose.data[1].w  <<"," << pose.data[2].w << "    ";
      float xt = pose_.data[0].w - init_pose_.x;
      float yt = pose_.data[1].w - init_pose_.y;
      float zt = pose_.data[2].w - init_pose_.z;
      return (make_float3(xt, yt, zt));
    }

    float3 getInitPos(){
      return init_pose_;
    }

    Matrix4 getPose() {
      return pose_;
    }

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

    Matrix4 *getViewPose() {
      return (viewPose_);
    }

    float3 getModelDimensions() {
      return (volume_dimension_);
    }

    uint3 getModelResolution() {
      return (volume_resolution_);
    }

    uint2 getComputationResolution() {
      return (computation_size_);
    }
};

void synchroniseDevices(); // Synchronise CPU and GPU

#endif
