/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */
// #include <DenseSLAMSystem.h>
#include <str_utils.h>
#include <se/commons.h>
#include <interface.h>
#include <se/constant_parameters.h>
#include <se/config.h>
#include <stdint.h>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <sstream>

DepthReader *createReader(Configuration *config, std::string filename) {
  DepthReader *reader = NULL;
  if (filename == "")
    filename = config->input_file;
  if ((filename.length() > 4)
      && (filename.substr(filename.length() - 4, 4) == ".scf")) {
    std::cerr << "====== Opening scene configuration file " << filename
      << "\n";
    bool new_file = false;
    std::string line;
    std::ifstream infile(filename.c_str());
    std::vector<std::string> path = splitString(filename, '/');
    std::string rpath = "";
    if (path.size() > 1)
      for (int i = 0; i < path.size() - 1; i++)
        rpath = rpath + path[i] + "/";
    while (std::getline(infile, line)) {
      if (line.substr(0, 1) != "#") {
        std::vector<std::string> key_value = splitString(line, '=');

        if (key_value.size() > 1) {

          std::string key = key_value[0];
          std::transform(key.begin(), key.end(), key.begin(),
              ::tolower);
          std::vector<std::string> values = splitString(key_value[1], '\"');
          std::string value;

          if (values.size() > 1)
            value = values[1];
          else
            value = values[0];
          if (key == "volume-resolution") {
            std::vector<std::string> dims = splitString(value, ',');

            if (dims.size() == 3) {
              config->volume_resolution.x() = ::atoi(
                  dims[0].c_str());
              config->volume_resolution.y() = ::atoi(
                  dims[1].c_str());
              config->volume_resolution.z() = ::atoi(
                  dims[2].c_str());
            } else {
              if (dims.size() == 0)
                config->volume_resolution.x() = 256;
              else
                config->volume_resolution.x() = ::atoi(
                    dims[0].c_str());
              config->volume_resolution.y() = config->volume_size.x();
              config->volume_resolution.z() = config->volume_size.x();
            }
            std::cout << "volumetric-size: "
              << config->volume_resolution.x() << "x"
              << config->volume_resolution.y() << "x"
              << config->volume_resolution.z() << std::endl;
            continue;
          }

          if (key == "volume-size") {
            std::vector<std::string> dims = splitString(value, ',');

            if (dims.size() == 3) {
              config->volume_size.x() = ::atof(dims[0].c_str());
              config->volume_size.y() = ::atof(dims[1].c_str());
              config->volume_size.z() = ::atof(dims[2].c_str());
            } else {
              if (dims.size() == 0)
                config->volume_size.x() = 2.0;
              else {
                config->volume_size.x() = ::atof(dims[0].c_str());
                config->volume_size.y() = config->volume_size.x();
                config->volume_size.z() = config->volume_size.x();
              }
            }
            std::cout << "volume-size: " << config->volume_size.x()
              << "x" << config->volume_size.y() << "x"
              << config->volume_size.z() << std::endl;
            continue;
          }

          if (key == "initial-position") {
            std::vector<std::string> dims = splitString(value, ',');
            if (dims.size() == 3) {
              config->initial_pos_factor.x() = ::atof(
                  dims[0].c_str());
              config->initial_pos_factor.y() = ::atof(
                  dims[1].c_str());
              config->initial_pos_factor.z() = ::atof(
                  dims[2].c_str());
              std::cout << "initial-position: "
                << config->initial_pos_factor.x() << ", "
                << config->initial_pos_factor.y() << ", "
                << config->initial_pos_factor.z()
                << std::endl;
            } else {
              std::cerr
                << "ERROR: initial-position  specified with incorrect data. (was "
                << value << ") Should be \"x, y, z\""
                << std::endl;
            }
            continue;
          }
          if (key == "camera") {
            std::vector<std::string> dims = splitString(value, ',');
            if (dims.size() == 4) {
              config->camera.x() = ::atof(dims[0].c_str());
              config->camera.y() = ::atof(dims[1].c_str());
              config->camera.z() = ::atof(dims[2].c_str());
              config->camera.w() = ::atof(dims[3].c_str());
              config->camera_overrided = true;
              std::cout << "camera: " << config->camera.x() << ","
                << config->camera.y() << ","
                << config->camera.z() << ","
                << config->camera.w() << std::endl;
            } else {
              std::cerr
                << "ERROR: camera specified with incorrect data. (was "
                << value << ") Should be \"x, y, z, w\""
                << std::endl;
            }

          }

          if (key == "input-file") {
            if (value.substr(0, 1) != "/") {
              value = rpath + value;
            }
            config->input_file = value;
            filename = value;
            std::cout << "input-file: " << config->input_file
              << std::endl;
            new_file = true;
            continue;
          }
        }
      }
    }
  }

  // Create reader configuration from general configuration.
  ReaderConfiguration reader_config;
  reader_config.fps = config->fps;
  reader_config.blocking_read = config->blocking_read;
  reader_config.data_path = config->input_file;
  reader_config.groundtruth_path = config->groundtruth_file;
  reader_config.transform = config->gt_transform;

  struct stat st;
  lstat(filename.c_str(), &st);

  if (filename == "") {
#ifdef DO_OPENNI
    //This is for openni from a camera
    reader = new OpenNIDepthReader(reader_config);
    if(!(reader->camera_open)) {
      delete reader;
      reader=NULL;
    }
#else
    reader = NULL;
#endif
  } else if (S_ISDIR(st.st_mode)) {
    // ICL-NUIM reader
    reader = new SceneDepthReader(reader_config);
  }
#ifdef DO_OPENNI
  else if(filename.substr(filename.length()-4, 4)==".oni") {
    //This is for openni from a file
    reader = new OpenNIDepthReader(reader_config);
  }
#endif
  else if (filename.substr(filename.length() - 4, 4) == ".raw") {
    // Slambench 1.0 raw reader
    reader = new RawDepthReader(reader_config);
  } else {
    std::cerr << "Unrecognised file format file not loaded\n";
    reader = NULL;
  }
  if (reader && reader->isValid() == false) {
    delete reader;
    reader = NULL;
  }
  return reader;

}

