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

#ifndef CONFIG_H
#define CONFIG_H

#include <math_utils.h>
#include <vector>
#include <string>

struct Configuration {

  //
  // KFusion configuration parameters
  // Command line arguments are parsed in default_parameters.h
  //

  /**
   * The ratio of the input frame size over the frame size used internally.
   * Values greater than 1 result in the input frames being downsampled
   * before processing. Valid values are 1, 2, 4 and 8.
   * <br>\em Default: 1
   */
  int compute_size_ratio;

  /**
   * Perform tracking on a frame every tracking_rate frames.
   * <br>\em Default: 1
   */
  int tracking_rate;

  /**
   * Integrate a 3D reconstruction every integration_rate frames. Should not
   * be less than tracking_rate.
   * <br>\em Default: 2
   */
  int integration_rate;

  /**
   * Render the 3D reconstruction every rendering_rate frames.
   * <br>\em Default: 4
   */
  int rendering_rate;

  /**
   * The x, y and z resolution of the reconstructed volume in voxels.
   * <br>\em Default: (256, 256, 256)
   */
  Eigen::Vector3i volume_resolution;

  /**
   * The x, y and z dimensions of the reconstructed volume in meters.
   * <br>\em Default: (2, 2, 2)
   */
  Eigen::Vector3f volume_size;

  int voxel_block_size;

  /*
   * TODO
   * <br>\em Default: (0.5, 0.5, 0)
   */
  Eigen::Vector3f initial_pos_factor;

  /**
   * TODO
   * <br>\em Default: (10, 5, 4)
   */
  std::vector<int> pyramid;

  /*
   * TODO
   * <br>\em Default: ""
   */
  std::string dump_volume_file;

  /*
   * TODO
   * <br>\em Default: ""
   */
  std::string input_file;

  /*
   * TODO
   * <br>\em Default: ""
   */
  std::string log_file;

  /*
   * TODO
   * <br>\em Default: ""
   */
  std::string groundtruth_file;

  /**
   * The intrinsic camera parameters. camera.x, camera.y, camera.z and
   * camera.w are the x-axis focal length, y-axis focal length, horizontal
   * resolution (pixels) and vertical resolution (pixels) respectively.
   */
  Eigen::Vector4f camera;

  /**
   * Whether the default intrinsic camera parameters have been overriden.
   */
  bool camera_overrided;

  /**
   * The TSDF truncation bound. Values of the TSDF are assumed to be in the
   * interval ±mu. See Section 3.3 of \cite NewcombeISMAR2011 for more
   * details.
   *  <br>\em Default: 0.1
   */
  float mu;

  /*
   * TODO
   *
   * @note Must be non-negative.
   *
   * <br>\em Default: 0
   */
  int fps;

  /*
   * TODO
   * <br>\em Default: false
   */
  bool blocking_read;

  /**
   * The ICP convergence threshold.
   * <br>\em Default: 1e-5
   */
  float icp_threshold;

  /**
   * Whether to hide the GUI. Hiding the GUI results in faster operation.
   * <br>\em Default: false
   */
  bool no_gui;

  /*
   * TODO
   * <br>\em Default: false
   */
  bool render_volume_fullsize;

  /**
   * Whether to filter the depth input frames using a bilateral filter.
   * Filtering using a bilateral filter helps to reduce the measurement
   * noise.
   * <br>\em Default: false
   */
  bool bilateralFilter;

  /* UNUSED */
  bool colouredVoxels;

  /*
   * UNUSED
   * <br>\em Default: false
   */
  bool multiResolution;

  /*
   * UNUSED
   * <br>\em Default: false
   */
  bool bayesian;
};

#endif
