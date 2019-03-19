/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#ifdef __APPLE__
    #include <mach/clock.h>
    #include <mach/mach.h>
#endif

#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <str_utils.h>
#include <Eigen/Dense>
#include <thirdparty/cutil_math.h>

#include "sys/stat.h"

enum ReaderType {
  READER_RAW, READER_SCENE, READER_OPENNI
};

struct ReaderConfiguration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int fps;
  bool blocking_read;
  std::string data_path;
  std::string groundtruth_path;
  Eigen::Matrix4f transform;
};

class DepthReader {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~DepthReader() { }

    virtual bool readNextDepthFrame(float * depth_map)= 0;

    inline bool readNextDepthFrame(unsigned short int * U_int_depth_map) {
      return readNextDepthFrame(NULL, U_int_depth_map);
    }

    virtual bool readNextDepthFrame(uchar3*              raw_rgb,
                                    unsigned short int * depth_map) = 0;

    virtual bool readNextData(uchar3*          ,
                              uint16_t*        ,
                              Eigen::Matrix4f& ) {return false;};

    virtual Eigen::Vector4f getK() = 0;

    virtual uint2 getinputSize() = 0;

    virtual void restart()=0;

    bool isValid() {
      return camera_open;
    }

    int getFrameNumber() {
      return (frame_);
    }

    virtual ReaderType getType() =0;

    void get_next_frame() {

      //		std::cout << "\rNext frame " << std::setw(10) << frame_ << " ";
      //
      //		if (frame_ % 2) {
      //			fflush(stdout);
      //		}

      if (fps_ == 0) {
        frame_++;
        return;
      }

#ifdef __APPLE__
      clock_serv_t cclock;
      mach_timespec_t clock_data;
      host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
      clock_get_time(cclock, &clock_data);
      mach_port_deallocate(mach_task_self(), cclock);
#else
      struct timespec clock_data;
      clock_gettime(CLOCK_MONOTONIC, &clock_data);
#endif
      double current_frame = clock_data.tv_sec
        + clock_data.tv_nsec / 1000000000.0;
      static double first_frame = current_frame;

      int frame = std::ceil((current_frame - first_frame) * (double) fps_);
      frame_ = frame;

      double tpf = 1.0 / fps_;
      double ttw = ((double) frame_ * tpf - current_frame + first_frame);
      if (blocking_read_) {
        if (ttw > 0)
          usleep(1000000.0 * ttw);
      }

    }

    inline bool readNextPose(Eigen::Matrix4f& pose) {
      std::string line;
      while (true) {
        std::getline(gt_file_,line);
        // EOF reached
        if (!gt_file_.good()) {
          return false;
        }
        // Ignore comment lines
        if (line[0] == '#') {
          continue;
        }
        // Data line read, split on spaces
        const std::vector<std::string> line_data = splitString(line, ' ');
        const size_t num_cols = line_data.size();
        if (num_cols < 7) {
          std::cout << "Invalid ground truth file format."
            << "Expected line format: ... tx ty tz qx qy qz qw" << std::endl;
          return false;
        }
        // Read the last 7 columns
        Eigen::Vector3f tran (std::stof(line_data[num_cols-7]),
                              std::stof(line_data[num_cols-6]),
                              std::stof(line_data[num_cols-5]));
        Eigen::Quaternionf quat (std::stof(line_data[num_cols-1]),
                                 std::stof(line_data[num_cols-4]),
                                 std::stof(line_data[num_cols-3]),
                                 std::stof(line_data[num_cols-2]));
        pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = quat.toRotationMatrix();
        pose.block<3,1>(0,3) = tran;
        pose_num_++;
        // Apply the transform to the pose
        pose = transform_ * pose;
        return true;
      }
    }

    bool camera_active;
    bool camera_open;
  protected:
    int frame_;
    size_t pose_num_;
    int fps_;
    bool blocking_read_;
    std::string data_path_;
    std::string groundtruth_path_;
    std::ifstream gt_file_;
    Eigen::Matrix4f transform_;
};

static const float scene_k[3][3] = { { 481.20, 0.00, 319.50 }, { 0.00, -480.00,
  239.50 }, { 0.00, 0.00, 1.00 } };

static const int scene_width_ = 640;
static const int scene_height_ = 480;
static const float u0_ = scene_k[0][2];
static const float v0_ = scene_k[1][2];
static const float focal_x_ = scene_k[0][0];
static const float focal_y_ = scene_k[1][1];

class SceneDepthReader: public DepthReader {
  private:

    std::string _dir;
    uint2 in_size_;

  public:
    ~SceneDepthReader() { };

    SceneDepthReader(const ReaderConfiguration& config)
      : SceneDepthReader(config.data_path, config.fps, config.blocking_read){ }

    SceneDepthReader(std::string dir, int fps, bool blocking_read) :
      DepthReader(), _dir(dir), in_size_(make_uint2(640, 480)) {
        std::cerr << "No such directory " << dir << std::endl;
        struct stat st;
        lstat(dir.c_str(), &st);
        if (S_ISDIR(st.st_mode)) {
          camera_open = true;
          camera_active = true;
          frame_ = -1;
          fps_ = fps;
          blocking_read_ = blocking_read;
        } else {
          std::cerr << "No such directory " << dir << std::endl;
          camera_open = false;
          camera_active = false;
        }

      }
    ;

    ReaderType getType() {
      return (READER_SCENE);
    }

    inline Eigen::Vector4f getK() {
      return Eigen::Vector4f(481.20, 480.00, 319.50, 239.50);
    }

    inline uint2 getinputSize() {
      return in_size_;
    }

    inline void restart() {
      frame_ = 0;
    }

    inline bool readNextDepthFrame(uchar3*, unsigned short int * depth_map) {

      float* float_depth_map = (float*) malloc(
          in_size_.x * in_size_.y * sizeof(float));
      bool res = readNextDepthFrame(float_depth_map);

      for (unsigned int i = 0; i < in_size_.x * in_size_.y; i++) {
        depth_map[i] = float_depth_map[i] * 1000.0f;
      }
      free(float_depth_map);
      return res;

    }

    inline bool readNextDepthFrame(float * depth_map) {

      std::ostringstream filename;
      get_next_frame();
      filename << this->_dir << "/scene_00_" << std::setfill('0')
        << std::setw(4) << frame_ << ".depth";

      std::ifstream source;
      source.open(filename.str().c_str(), std::ios_base::in);

      if (!source) {
        std::cerr << "Can't open Data from " << filename.str().c_str()
          << "!\n";
        return 0;
      }
      uint index = 0;
      while (source.good()) {
        float d;
        source >> d;
        if (scene_width_ * scene_height_ <= index)
          continue;
        depth_map[index] = d;
        index++;
      }

      for (int v = 0; v < scene_height_; v++) {
        for (int u = 0; u < scene_width_; u++) {
          float u_u0_by_fx = (u - u0_) / focal_x_;
          float v_v0_by_fy = (v - v0_) / focal_y_;

          depth_map[u + v * scene_width_] = depth_map[u + v * scene_width_]
            / std::sqrt(
                u_u0_by_fx * u_u0_by_fx
                + v_v0_by_fy * v_v0_by_fy + 1);

        }
      }
      return index > 0;
    }

};

/**
 * Reader for Slambench 1.0 datasets.
 */
class RawDepthReader: public DepthReader {
  private:
    FILE* raw_file_ptr_;
    uint2 in_size_;

  public:
    /**
     * Constructor using the ::ReaderConfiguration struct.
     */
    RawDepthReader(const ReaderConfiguration& config)
      : DepthReader() {
        // Copy configuration
        data_path_ = config.data_path;
        groundtruth_path_ = config.groundtruth_path;
        transform_ = config.transform;
        // Open ground truth association file if supplied
        if (groundtruth_path_ != "") {
          gt_file_.open(groundtruth_path_.c_str());
          if(!gt_file_.is_open()) {
            std::cout << "Failed to open ground truth association file "
              << groundtruth_path_ << std::endl;
            raw_file_ptr_ = NULL;
            return;
          }
          pose_num_ = -1;
        }
        // Open raw file
        const std::string raw_filename = data_path_;
        raw_file_ptr_ = fopen(raw_filename.c_str(), "rb");
        const size_t res = fread(&(in_size_), sizeof(in_size_), 1, raw_file_ptr_);
        camera_open = false;
        camera_active = false;
        if (res != 1) {
          std::cerr << "Invalid Raw file " << raw_filename << std::endl;
          return;
        } else {
          camera_open = true;
          camera_active = true;
          frame_ = -1;
          fps_ = config.fps;
          blocking_read_ = config.blocking_read;
          fseeko(raw_file_ptr_, 0, SEEK_SET);
        }
      };

    /**
     * Old style constructor. Does not support ground truth loading.
     *
     * @deprecated Might be removed in the future.
     */
    RawDepthReader(std::string filename, int fps, bool blocking_read) :
      DepthReader(), raw_file_ptr_(fopen(filename.c_str(), "rb")) {

        size_t res = fread(&(in_size_), sizeof(in_size_), 1, raw_file_ptr_);
        camera_open = false;
        camera_active = false;
        if (res != 1) {
          std::cerr << "Invalid Raw file." << std::endl;

        } else {
          camera_open = true;
          camera_active = true;
          frame_ = -1;
          fps_ = fps;
          blocking_read_ = blocking_read;
          fseeko(raw_file_ptr_, 0, SEEK_SET);
        }
      };

    /**
     * Get the type of the reader.
     *
     * \return ReaderType::READER_RAW
     */
    ReaderType getType() {
      return (READER_RAW);
    }

    inline bool readNextDepthFrame(uchar3*              raw_rgb,
                                   unsigned short int * depth_map) {

      int total = 0;
      int expected_size = 0;
      unsigned int new_image_size[2];

      get_next_frame();

      //		if (frame_ < 2135)
      //		{
      //			frame_ = 2135;
      //		}
      //
      //		std::cout << "Frame: " << frame_ << std::endl;


#ifdef LIGHT_RAW // This LightRaw mode is used to get smaller raw files
      unsigned int size_of_frame = (sizeof(unsigned int) * 2 + in_size_.x * in_size_.y * sizeof(unsigned short int) );
#else
      off_t size_of_frame = (sizeof(unsigned int) * 4
          + in_size_.x * in_size_.y * sizeof(unsigned short int)
          + in_size_.x * in_size_.y * sizeof(uchar3));
#endif
      // std::cout << "Seek: " << size_of_frame * frame_ << std::endl;

      fseeko(raw_file_ptr_, size_of_frame * frame_, SEEK_SET);

      total += fread(&(new_image_size), sizeof(new_image_size), 1, raw_file_ptr_);

      if (depth_map) {
        total += fread(depth_map, sizeof(unsigned short int),
            new_image_size[0] * new_image_size[1], raw_file_ptr_);
        expected_size += 1 + new_image_size[0] * new_image_size[1];
      } else {
        total += new_image_size[0] * new_image_size[1];
        fseeko(raw_file_ptr_,
            new_image_size[0] * new_image_size[1]
            * sizeof(unsigned short int), SEEK_CUR);
        expected_size += 1 + new_image_size[0] * new_image_size[1];
      }

#ifdef LIGHT_RAW // This LightRaw mode is used to get smaller raw files

      if (raw_rgb) {
        raw_rgb[0].x = 0;
      } else {
      }

#else
      total += fread(&(new_image_size), sizeof(new_image_size), 1, raw_file_ptr_);

      if (raw_rgb) {
        total += fread(raw_rgb, sizeof(uchar3),
            new_image_size[0] * new_image_size[1], raw_file_ptr_);
        expected_size += 1 + new_image_size[0] * new_image_size[1];
      } else {
        total += new_image_size[0] * new_image_size[1];
        fseeko(raw_file_ptr_, new_image_size[0] * new_image_size[1] * sizeof(uchar3),
            SEEK_CUR);
        expected_size += 1 + new_image_size[0] * new_image_size[1];
      }
#endif

      if (total != expected_size) {
        std::cout << "End of file" << (total == 0 ? "" : "(garbage found)")
          << "." << std::endl;
        return false;
      } else {
        return true;
      }
    }

    /**
     * Restart the reader from the beginning.
     */
    inline void restart() {
      frame_ = -1;
      pose_num_ = -1;
      rewind(raw_file_ptr_);
      if (gt_file_.is_open())
        gt_file_.seekg(0, gt_file_.beg);
    }

    inline bool readNextDepthFrame(float * depth_map) {

      unsigned short int* U_int_depth_map = (unsigned short int*) malloc(
          in_size_.x * in_size_.y * sizeof(unsigned short int));
      bool res = readNextDepthFrame(NULL, U_int_depth_map);

      for (unsigned int i = 0; i < in_size_.x * in_size_.y; i++) {
        depth_map[i] = (float) U_int_depth_map[i] / 1000.0f;
      }
      free(U_int_depth_map);
      return res;
    }

    /**
     * Read the RGB, depth and ground truth pose data corresponding to the next
     * measurement.
     *
     * \param[out] rgb_image The RGB frame.
     * \param[out] depth_image The depth frame.
     * \param[out] pose The ground truth pose.
     * \return true on success, false on failure to read.
     */
    inline bool readNextData(uchar3*          rgb_image,
                             uint16_t*        depth_image,
                             Eigen::Matrix4f& pose) {
      bool res;
      // pose_num_ is incremented inside readNextPose()
      res = readNextPose(pose);
      // frame_ is incremented inside readNextDepthFrame()
      res = res && readNextDepthFrame(rgb_image, depth_image);
      return res;
    }

    /**
     * Returns the dimensions of the frames read.
     */
    inline uint2 getinputSize() {
      return in_size_;
    }

    /**
     * Returns a vector with the camera matrix parameters. The `x`, `y`, `z`
     * and `w` elements are the x-axis focal length, y-axis focal length,
     * x-axis optical center and y-axis optical center.
     */
    inline Eigen::Vector4f getK() {
      return Eigen::Vector4f(531.15, 531.15, 640 / 2, 480 / 2);
    }

};

#ifdef DO_OPENNI
#include <OpenNI.h>

class MyDepthFrameAllocator: public openni::VideoStream::FrameAllocator {
  private:
    uint16_t *depth_buffers_;

  public:
    MyDepthFrameAllocator(uint16_t *depth_buffers) :
      depth_buffers_(depth_buffers) {
      }
    void *allocateFrameBuffer(int ) {
      return depth_buffers_;
    }
    void freeFrameBuffer(void *) {
    }
};

class MyColorFrameAllocator: public openni::VideoStream::FrameAllocator {
  private:
    uchar3 *rgb_buffer_;

  public:
    MyColorFrameAllocator(uchar3 *rgb_buffer) {
      rgb_buffer_ = rgb_buffer;
    }
    void *allocateFrameBuffer(int ) {
      return rgb_buffer_;
    }
    void freeFrameBuffer(void *) {
    }
};

class OpenNIDepthReader: public DepthReader {
  private:
    FILE* pFile_;
    uint2 in_size_;

    openni::Device device;
    openni::VideoStream depth;
    openni::VideoStream rgb;
    openni::VideoFrameRef depth_frame;
    openni::VideoFrameRef color_frame;

    uint16_t * depth_image;
    uchar3 * rgb_image;

    openni::Status rc;

  public:
    ~OpenNIDepthReader() {
      if(device.isValid()) {
        //std::cout << "Stopping depth stream..." << std::endl;

        depth.stop();
        //std::cout << "Stopping rgb stream..." << std::endl;
        rgb.stop();
        //std::cout << "Destroying depth stream..." << std::endl;
        depth.destroy();
        // std::cout << "Destroying rgb stream..." << std::endl;
        rgb.destroy();
        //std::cout << "Closing device..." << std::endl;
        device.close();

        //std::cout << "Shutting down OpenNI..." << std::endl;
        openni::OpenNI::shutdown();

        //std::cout << "Done!" << std::endl;
        camera_open = false;
        camera_active = false;
      }

    };

    ReaderType getType() {
      return(READER_OPENNI);
    }

    OpenNIDepthReader(const ReaderConfiguration& config)
      : OpenNIDepthReader(config.data_path, config.fps, config.blocking_read){ }

    OpenNIDepthReader(std::string filename, int fps, bool blocking_read) :
      DepthReader(), pFile_(fopen(filename.c_str(), "rb")) {

        camera_active = false;
        camera_open = false;

        rc = openni::OpenNI::initialize();

        if (rc != openni::STATUS_OK) {
          return;
        }

        if (filename == "") {
          rc = device.open(openni::ANY_DEVICE);
        } else {
          rc = device.open(filename.c_str());
          //std::cerr<<"Opened an oni file\n";
        }

        if (rc != openni::STATUS_OK) {
          std::cout << "No kinect device found. " << camera_active << " " << camera_open << "\n";
          return;
        }

        if (device.getSensorInfo(openni::SENSOR_DEPTH) != NULL) {
          rc = depth.create(device, openni::SENSOR_DEPTH);

          if (rc != openni::STATUS_OK) {
            std::cout << "Couldn't create depth stream" << std::endl << openni::OpenNI::getExtendedError() << std::endl;
            //exit(3);
            return;
          } else {
            rc = depth.start();
            if (rc != openni::STATUS_OK) {
              printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
              depth.destroy();
              //exit(3);
              return;
            }
            depth.stop();

          }

          rc = rgb.create(device, openni::SENSOR_COLOR);

          if (rc != openni::STATUS_OK) {
            std::cout << "Couldn't create color stream" << std::endl << openni::OpenNI::getExtendedError() << std::endl;
            //exit(3);
            return;
          }

        }

        device.getSensorInfo(openni::SENSOR_DEPTH)->getSupportedVideoModes();

        if(!device.isFile())
        {
          openni::VideoMode newDepthMode;
          newDepthMode.setFps(fps == 0 ? 30 : fps);
          newDepthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
          newDepthMode.setResolution(640,480);

          rc = depth.setVideoMode(newDepthMode);
          if(rc != openni::STATUS_OK)
          {
            std::cout << "Could not set videomode" << std::endl;
            //exit(3);
            return;
          }

          openni::VideoMode newRGBMode;
          newRGBMode.setFps(fps == 0 ? 30 : fps);
          newRGBMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
          newRGBMode.setResolution(640,480);

          rc = rgb.setVideoMode(newRGBMode);
          if(rc != openni::STATUS_OK)
          {
            std::cout << "Could not set videomode" << std::endl;

            return;
          }
        }

        openni::VideoMode depthMode = depth.getVideoMode();
        openni::VideoMode color_mode = rgb.getVideoMode();

        in_size_.x = depthMode.getResolutionX();
        in_size_.y = depthMode.getResolutionY();

        if (color_mode.getResolutionX() != in_size_.x || color_mode.getResolutionY() != in_size_.y) {
          std::cout << "Incorrect rgb resolution: " << color_mode.getResolutionX() << " " << color_mode.getResolutionY() << std::endl;
          //exit(3);
          return;
        }

        depth_image = (uint16_t*) malloc(in_size_.x * in_size_.y * sizeof(uint16_t));
        rgb_image = (uchar3*) malloc(in_size_.x * in_size_.y * sizeof(uchar3));

        rgb.setMirroringEnabled(false);
        depth.setMirroringEnabled(false);

        device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

        if(device.isFile())
        {
          device.getPlaybackControl()->setRepeatEnabled(false);
          depth_frame.release();
          color_frame.release();
          device.getPlaybackControl()->setSpeed(-1); // Set the playback in a manual mode i.e. read a frame whenever the application requests it
        }

        // The allocators must survive this initialization function.
        MyDepthFrameAllocator *depth_alloc = new MyDepthFrameAllocator(depth_image);
        MyColorFrameAllocator *color_alloc = new MyColorFrameAllocator(rgb_image);

        // Use allocators to have OpenNI write directly into our buffers
        depth.setFrameBuffersAllocator(depth_alloc);
        depth.start();

        rgb.setFrameBuffersAllocator(color_alloc);
        rgb.start();

        camera_open =true;
        camera_active =true;
        frame_=-1;
        fps_ = fps;
        blocking_read_ = blocking_read;

      };

    inline bool readNextDepthFrame(uchar3* raw_rgb, unsigned short int * depth_map) {

      rc = depth.readFrame(&depth_frame);

      if (rc != openni::STATUS_OK) {
        std::cout << "Wait failed" << std::endl;
        exit(1);
      }

      rc = rgb.readFrame(&color_frame);
      if (rc != openni::STATUS_OK) {
        std::cout << "Wait failed" << std::endl;
        exit(1);
      }

      if (depth_frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM
          && depth_frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM) {
        std::cout << "Unexpected frame format" << std::endl;
        exit(1);
      }

      if (color_frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_RGB888) {
        std::cout << "Unexpected frame format" << std::endl;
        exit(1);
      }

      if (raw_rgb) {
        memcpy(raw_rgb,rgb_image,in_size_.x * in_size_.y*sizeof(uchar3));
      }
      if (depth_map) {
        memcpy(depth_map,depth_image,in_size_.x * in_size_.y*sizeof(uint16_t));
      }

      get_next_frame ();

      return true;

    }

    inline void restart() {
      frame_=-1;
      //FIXME : how to rewind OpenNI ?

    }

    inline bool readNextDepthFrame(float * depth_map) {

      unsigned short int* U_int_depth_map = (unsigned short int*) malloc(in_size_.x * in_size_.y * sizeof(unsigned short int));
      bool res = readNextDepthFrame(NULL,U_int_depth_map);

      for (unsigned int i = 0; i < in_size_.x * in_size_.y; i++) {
        depth_map[i] = (float) U_int_depth_map[i] / 1000.0f;
      }
      free(U_int_depth_map);
      return res;
    }

    inline uint2 getinputSize() {
      return in_size_;
    }

    inline Eigen::Vector4f getK() {
      return Eigen::Vector4f(481.2, 480, 640/2, 480/2);
    }

};

#else
class OpenNIDepthReader: public DepthReader {
  public:
    OpenNIDepthReader(const ReaderConfiguration& config)
      : OpenNIDepthReader(config.data_path, config.fps, config.blocking_read){ }

    OpenNIDepthReader(std::string, int, bool) {
      std::cerr << "OpenNI Library Not found." << std::endl;
      camera_open = false;
      camera_active = false;
    }

    bool readNextDepthFrame(float * ) { return false;
    }

    bool readNextDepthFrame(uchar3* , unsigned short int * ) {
      return false;
    }

    Eigen::Vector4f getK() {
      return Eigen::Vector4f::Constant(0.f);
    } 

    uint2 getinputSize() {
      return make_uint2(0);
    }

    void restart() {
    }

    ReaderType getType() {
      return (READER_OPENNI);
    }
};
#endif /* DO_OPENNI*/

#endif /* INTERFACE_H_ */
