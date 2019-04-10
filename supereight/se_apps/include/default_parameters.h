/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef DEFAULT_PARAMETERS_H_
#define DEFAULT_PARAMETERS_H_

#include <str_utils.h>
#include <se/utils/math_utils.h>
#include <vector>
#include <sstream>
#include <getopt.h>

#include <se/constant_parameters.h>
#include <se/config.h>
#include <Eigen/Dense>

////////////////////////// RUNTIME PARAMETERS //////////////////////

#define DEFAULT_ITERATION_COUNT 3
static const int default_iterations[DEFAULT_ITERATION_COUNT] = { 10, 5, 4 };

const float default_mu = 0.1f;
const bool default_blocking_read = false;
const int default_fps = 0;
const float default_icp_threshold = 1e-5;
const int default_compute_size_ratio = 1;
const int default_integration_rate = 2;
const int default_rendering_rate = 4;
const int default_tracking_rate = 1;
const Eigen::Vector3i default_volume_resolution(256, 256, 256);
const Eigen::Vector3f default_volume_size(2.f, 2.f, 2.f);
const Eigen::Vector3f default_initial_pos_factor(0.5f, 0.5f, 0.0f);
const bool default_no_gui = false;
const bool default_render_volume_fullsize = false;
const bool default_bilateralFilter = false;
const std::string default_dump_volume_file = "";
const std::string default_input_file = "";
const std::string default_log_file = "";
const int default_coloured_voxels = false;
const int default_multi_resolution = false;
const bool default_bayesian = false;
const std::string default_groundtruth_file = "";
const Eigen::Matrix4f default_gt_transform = Eigen::Matrix4f::Identity();

inline std::string pyramid2str(std::vector<int> v) {
  std::ostringstream ss;
  for (std::vector<int>::iterator it = v.begin(); it != v.end(); it++)
    ss << *it << " ";
  return ss.str();

}

static std::string short_options = "a:qc:d:f:g:G:hi:l:m:k:o:p:r:s:t:v:y:z:FC:M";

static struct option long_options[] =
{
  {"quaternion",         required_argument, 0, 'a'},
  {"block-read",         no_argument, 0, 'b'},
  {"compute-size-ratio", required_argument, 0, 'c'},
  {"dump-volume",        required_argument, 0, 'd'},
  {"invert-y",           no_argument, 0, 'e'},
  {"fps",                required_argument, 0, 'f'},
  {"input-file",         required_argument, 0, 'i'},
  {"camera",             required_argument, 0, 'k'},
  {"icp-threshold",      required_argument, 0, 'l'},
  {"log-file",           required_argument, 0, 'o'},
  {"mu",                 required_argument, 0, 'm'},
  {"init-pose",          required_argument, 0, 'p'},
  {"no-gui",             no_argument,       0, 'q'},
  {"integration-rate",   required_argument, 0, 'r'},
  {"volume-size",        required_argument, 0, 's'},
  {"tracking-rate",      required_argument, 0, 't'},
  {"volume-resolution",  required_argument, 0, 'v'},
  {"pyramid-levels",     required_argument, 0, 'y'},
  {"rendering-rate",     required_argument, 0, 'z'},
  {"voxel-block-size",   required_argument, 0, 'B'},
  {"bilateral-filter",   no_argument, 0, 'F'},
  {"coloured-voxels",    no_argument, 0, 'C'},
  {"multi-res",          no_argument, 0, 'M'},
  {"bayesian",           no_argument, 0, 'h'},
  {"ground-truth",       required_argument, 0, 'g'},
  {"gt-transform",       required_argument, 0, 'G'},
  {0, 0, 0, 0}
};

inline
void print_arguments() {
  std::cerr << "-b  (--block-read)                        : default is False: Block on read " << std::endl;
  std::cerr << "-c  (--compute-size-ratio)                : default is " << default_compute_size_ratio << "   (same size)      " << std::endl;
  std::cerr << "-e  (--invert-y)                          : default is False: Block on read " << std::endl;
  std::cerr << "-d  (--dump-volume) <filename>            : Output volume file              " << std::endl;
  std::cerr << "-f  (--fps)                               : default is " << default_fps       << std::endl;
  std::cerr << "-F  (--bilateral-filter                   : default is disabled"               << std::endl;
  std::cerr << "-h  (--bayesian                           : default is disabled"               << std::endl;
  std::cerr << "-i  (--input-file) <filename>             : Input camera file               " << std::endl;
  std::cerr << "-k  (--camera)                            : default is defined by input     " << std::endl;
  std::cerr << "-l  (--icp-threshold)                     : default is " << default_icp_threshold << std::endl;
  std::cerr << "-o  (--log-file) <filename>               : default is stdout               " << std::endl;
  std::cerr << "-m  (--mu)                                : default is " << default_mu << "               " << std::endl;
  std::cerr << "-p  (--init-pose)                         : default is " << default_initial_pos_factor.x() << "," << default_initial_pos_factor.y() << "," << default_initial_pos_factor.z() << "     " << std::endl;
  std::cerr << "-q  (--no-gui)                            : default is to display gui"<<std::endl;
  std::cerr << "-r  (--integration-rate)                  : default is " << default_integration_rate << "     " << std::endl;
  std::cerr << "-s  (--volume-size)                       : default is " << default_volume_size.x() << "," << default_volume_size.y() << "," << default_volume_size.z() << "      " << std::endl;
  std::cerr << "-t  (--tracking-rate)                     : default is " << default_tracking_rate << "     " << std::endl;
  std::cerr << "-v  (--volume-resolution)                 : default is " << default_volume_resolution.x() << "," << default_volume_resolution.y() << "," << default_volume_resolution.z() << "    " << std::endl;
  std::cerr << "-y  (--pyramid-levels)                    : default is 10,5,4     " << std::endl;
  std::cerr << "-z  (--rendering-rate)                    : default is " << default_rendering_rate << std::endl;
  std::cerr << "-g  (--ground-truth) <filename>           : Ground truth file" << std::endl;
  std::cerr << "-G  (--gt-transform) tx,ty,tz,qx,qy,qz,qw : Ground truth pose tranform (translation and/or rotation)" << std::endl;
}

inline Eigen::Vector3f atof3(char * optarg) {
  Eigen::Vector3f res;
  std::istringstream dotargs(optarg);
  std::string s;
  if (getline(dotargs, s, ',')) {
    res.x() = atof(s.c_str());
  } else
    return res;
  if (getline(dotargs, s, ',')) {
    res.y() = atof(s.c_str());
  } else {
    res.y() = res.x();
    res.z() = res.y();
    return res;
  }
  if (getline(dotargs, s, ',')) {
    res.z() = atof(s.c_str());
  } else {
    res.z() = res.y();
  }
  return res;
}

inline Eigen::Vector3i atoi3(char * optarg) {
  Eigen::Vector3i res;
  std::istringstream dotargs(optarg);
  std::string s;
  if (getline(dotargs, s, ',')) {
    res.x() = atoi(s.c_str());
  } else
    return res;
  if (getline(dotargs, s, ',')) {
    res.y() = atoi(s.c_str());
  } else {
    res.y() = res.x();
    res.z() = res.y();
    return res;
  }
  if (getline(dotargs, s, ',')) {
    res.z() = atoi(s.c_str());
  } else {
    res.z() = res.y();
  }
  return res;
}

inline Eigen::Vector4f atof4(char * optarg) {
  Eigen::Vector4f res;
  std::istringstream dotargs(optarg);
  std::string s;
  if (getline(dotargs, s, ',')) {
    res.x() = atof(s.c_str());
  } else
    return res;
  if (getline(dotargs, s, ',')) {
    res.y() = atof(s.c_str());
  } else {
    res.y() = res.x();
    res.z() = res.y();
    res.w() = res.z();
    return res;
  }
  if (getline(dotargs, s, ',')) {
    res.z() = atof(s.c_str());
  } else {
    res.z() = res.y();
    res.w() = res.z();
    return res;
  }
  if (getline(dotargs, s, ',')) {
    res.w() = atof(s.c_str());
  } else {
    res.w() = res.z();
  }
  return res;
}

Configuration parseArgs(unsigned int argc, char ** argv) {

  Configuration config;

  config.compute_size_ratio = default_compute_size_ratio;
  config.integration_rate = default_integration_rate;
  config.tracking_rate = default_tracking_rate;
  config.rendering_rate = default_rendering_rate;
  config.volume_resolution = default_volume_resolution;
  config.volume_size = default_volume_size;
  config.initial_pos_factor = default_initial_pos_factor;
  //initial_pose_quant.setIdentity();
  //invert_y = false;

  config.dump_volume_file = default_dump_volume_file;
  config.input_file = default_input_file;
  config.log_file = default_log_file;
  config.groundtruth_file = default_groundtruth_file;
  config.gt_transform = default_gt_transform;

  config.mu = default_mu;
  config.fps = default_fps;
  config.blocking_read = default_blocking_read;
  config.icp_threshold = default_icp_threshold;
  config.no_gui = default_no_gui;
  config.render_volume_fullsize = default_render_volume_fullsize;
  config.camera_overrided = false;
  config.bilateralFilter = default_bilateralFilter;
  config.bayesian = default_bayesian;
  config.coloured_voxels = default_coloured_voxels;

  config.pyramid.clear();
  for (int i = 0; i < DEFAULT_ITERATION_COUNT; i++) {
    config.pyramid.push_back(default_iterations[i]);
  }

  int c;
  int option_index = 0;
  int flagErr = 0;
  std::vector<std::string> tokens;
  Eigen::Vector3f gt_transform_tran;
  Eigen::Quaternionf gt_transform_quat;
  while ((c = getopt_long(argc, argv, short_options.c_str(), long_options,
          &option_index)) != -1)
    switch (c) {
      case 'a':
        {
          //float4 vals = atof4(optarg);
          //initial_pose_quant = Eigen::Quaternionf(vals.w, vals.x, vals.y, vals.z);

          //std::cerr << "update quaternion rotation to " << config.initial_pose_quant.x() << ","
          //<< config.initial_pose_quant.y() << "," << config.initial_pose_quant.z() << ","
          //<< config.initial_pose_quant.w() << std::endl;
          break;
        }
      case 'b':
        config.blocking_read = true;
        std::cerr << "activate blocking read" << std::endl;
        break;
      case 'c':  //   -c  (--compute-size-ratio)
        config.compute_size_ratio = atoi(optarg);
        std::cerr << "update compute_size_ratio to "
          << config.compute_size_ratio << std::endl;
        if ((config.compute_size_ratio != 1)
            && (config.compute_size_ratio != 2)
            && (config.compute_size_ratio != 4)
            && (config.compute_size_ratio != 8)) {
          std::cerr
            << "ERROR: --compute-size-ratio (-c) must be 1, 2 ,4 or 8  (was "
            << optarg << ")\n";
          flagErr++;
        }
        break;
      case 'd':
        config.dump_volume_file = optarg;
        std::cerr << "update dump_volume_file to "
          << config.dump_volume_file << std::endl;
        break;
      case 'e':
        //config.invert_y = true;
        //std::cerr << "Inverting Y axis (ICL-NUIM Fix)" << std::endl;
        break;

      case 'f':  //   -f  (--fps)
        config.fps = atoi(optarg);
        std::cerr << "update fps to " << config.fps << std::endl;

        if (config.fps < 0) {
          std::cerr << "ERROR: --fps (-f) must be >= 0 (was "
            << optarg << ")\n";
          flagErr++;
        }
        break;
      case 'g': // -g (--ground-truth)
        config.groundtruth_file = optarg;
        std::cerr << "using the groundtruth file " << config.groundtruth_file
          << std::endl;
        break;
      case 'G': // -G (--gt-transform)
        // Split argument into substrings
        tokens = splitString(optarg, ',');
        switch (tokens.size()) {
          case 3:
            // Translation
            gt_transform_tran = Eigen::Vector3f(std::stof(tokens[0]),
                std::stof(tokens[1]), std::stof(tokens[2]));
            config.gt_transform.topRightCorner<3,1>() = gt_transform_tran;
            break;
          case 4:
            // Rotation
            // Create a quaternion and get the equivalent rotation matrix
            gt_transform_quat = Eigen::Quaternionf(std::stof(tokens[3]),
                std::stof(tokens[0]), std::stof(tokens[1]), std::stof(tokens[2]));
            config.gt_transform.block<3,3>(0,0) = gt_transform_quat.toRotationMatrix();
            break;
          case 7:
            // Translation and rotation
            gt_transform_tran = Eigen::Vector3f(std::stof(tokens[0]),
                std::stof(tokens[1]), std::stof(tokens[2]));
            gt_transform_quat = Eigen::Quaternionf(std::stof(tokens[6]),
                std::stof(tokens[3]), std::stof(tokens[4]), std::stof(tokens[5]));
            config.gt_transform.topRightCorner<3,1>() = gt_transform_tran;
            config.gt_transform.block<3,3>(0,0) = gt_transform_quat.toRotationMatrix();
            break;
          default:
            std::cerr << "Invalid number of parameters for argument gt-transform. Valid parameters are:\n"
                << "3 parameters (translation): tx,ty,tz\n"
                << "4 parameters (rotation in quaternion form): qx,qy,qz,qw\n"
                << "7 parameters (translation and rotation): tx,ty,tz,qx,qy,qz,qw"
                << std::endl;
            flagErr++;
            break;
        }
        std::cerr << "using the groundtruth transform\n"
          << config.gt_transform << std::endl;
        break;
      case 'h': // -h (--bayesian)
        config.bayesian = true;
        break;
      case 'i':    //   -i  (--input-file)
        config.input_file = optarg;
        std::cerr << "update input_file to " << config.input_file
          << std::endl;
        struct stat st;
        if (stat(config.input_file.c_str(), &st) != 0) {
          std::cerr << "ERROR: --input-file (-i) does not exist (was "
            << config.input_file << ")\n";
          flagErr++;
        }
        break;
      case 'k':    //   -k  (--camera)
        config.camera = atof4(optarg);
        config.camera_overrided = true;
        std::cerr << "update camera to " << config.camera.x() << ","
          << config.camera.y() << "," << config.camera.z() << ","
          << config.camera.w() << std::endl;
        break;
      case 'o':    //   -o  (--log-file)
        config.log_file = optarg;
        std::cerr << "update log_file to " << config.log_file
          << std::endl;
        break;
      case 'l':  //   -l (--icp-threshold)
        config.icp_threshold = atof(optarg);
        std::cerr << "update icp_threshold to " << config.icp_threshold
          << std::endl;
        break;
      case 'm':   // -m  (--mu)
        config.mu = atof(optarg);
        std::cerr << "update mu to " << config.mu << std::endl;
        break;
      case 'p':    //   -p  (--init-pose)
        config.initial_pos_factor = atof3(optarg);
        std::cerr << "update init_poseFactors to "
          << config.initial_pos_factor.x() << ","
          << config.initial_pos_factor.y() << ","
          << config.initial_pos_factor.z() << std::endl;
        break;
      case 'q':
        config.no_gui = true;
        break;
      case 'r':    //   -r  (--integration-rate)
        config.integration_rate = atoi(optarg);
        std::cerr << "update integration_rate to "
          << config.integration_rate << std::endl;
        if (config.integration_rate < 1) {
          std::cerr
            << "ERROR: --integration-rate (-r) must >= 1 (was "
            << optarg << ")\n";
          flagErr++;
        }
        break;
      case 's':    //   -s  (--map-size)
        config.volume_size = atof3(optarg);
        std::cerr << "update map_size to " << config.volume_size.x()
          << "mx" << config.volume_size.y() << "mx"
          << config.volume_size.z() << "m" << std::endl;
        if ((config.volume_size.x() <= 0) || (config.volume_size.y() <= 0)
            || (config.volume_size.z() <= 0)) {
          std::cerr
            << "ERROR: --volume-size (-s) all dimensions must > 0 (was "
            << optarg << ")\n";
          flagErr++;
        }
        break;
      case 't':    //   -t  (--tracking-rate)
        config.tracking_rate = atof(optarg);
        std::cerr << "update tracking_rate to " << config.tracking_rate
          << std::endl;
        break;
      case 'z':    //   -z  (--rendering-rate)
        config.rendering_rate = atof(optarg);
        std::cerr << "update rendering_rate to " << config.rendering_rate
          << std::endl;
        break;
      case 'v':    //   -v  (--volumetric-size)
        config.volume_resolution = atoi3(optarg);
        std::cerr << "update volumetric_size to "
          << config.volume_resolution.x() << "x"
          << config.volume_resolution.y() << "x"
          << config.volume_resolution.z() << std::endl;
        if ((config.volume_resolution.x() <= 0)
            || (config.volume_resolution.y() <= 0)
            || (config.volume_resolution.z() <= 0)) {
          std::cerr
            << "ERROR: --volume-size (-s) all dimensions must > 0 (was "
            << optarg << ")\n";
          flagErr++;
        }

        break;
      case 'y': {
                  std::istringstream dotargs(optarg);
                  std::string s;
                  config.pyramid.clear();
                  while (getline(dotargs, s, ',')) {
                    config.pyramid.push_back(atof(s.c_str()));
                  }
                }
                std::cerr << "update pyramid levels to " << pyramid2str(config.pyramid)
                  << std::endl;
                break;
      case 'F':
                config.bilateralFilter = true;
                std::cerr << "using bilateral filter" << std::endl;
                break;
      case 'C':
                config.coloured_voxels = true;
                std::cerr << "using coloured voxels" << std::endl;
                break;
      case 'M':
                config.multi_resolution = true;
                std::cerr << "using multi-resolution integration" << std::endl;
                break;
      case 0:
      case '?':
                std::cerr << "Unknown option character -" << char(optopt)
                  << " or bad usage.\n";
                print_arguments();
                exit(0);
      default:
                std::cerr << "GetOpt abort.";
                flagErr = true;
    }

  if (flagErr) {
    std::cerr << "Exited due to " << flagErr << " error"
      << (flagErr == 1 ? "" : "s")
      << " in command line options\n";
    exit(1);
  }
  return config;
}

#endif /* DEFAULT_PARAMETERS_H_ */
