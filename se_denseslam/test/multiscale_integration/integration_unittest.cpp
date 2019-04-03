#include <se/octant_ops.hpp>
#include <se/octree.hpp>
#include <se/algorithms/balancing.hpp>
#include <se/functors/axis_aligned_functor.hpp>
#include <se/algorithms/filter.hpp>
#include <se/io/vtk-io.h>
#include <se/io/ply_io.hpp>
#include <sophus/se3.hpp>
#include <se/utils/math_utils.h>
#include "se/node.hpp"
#include "se/functors/data_handler.hpp"
#include <random>
#include <functional>
#include <gtest/gtest.h>
#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>

typedef struct ESDF{
  float x;
  float delta;
  int   y;
  int   delta_y;
} ESDF;

template <>
struct voxel_traits<ESDF> {
  typedef ESDF value_type;
  static inline value_type empty(){ return     {0.f, 0.f, 0, 0}; }
  static inline value_type initValue(){ return {1.f, 0.f, 0, 0}; }
};

struct camera_parameter {
public:
  camera_parameter() {};

  camera_parameter(float pixel_size, float focal_length,
      Eigen::Vector2i image_size, Eigen::Matrix4f Twc)
    : pixel_size_(pixel_size), focal_length_(focal_length),
    image_size_(image_size), Twc_(Twc) {
    f_ = focal_length/pixel_size;
    K_ << f_, 0,  image_size.x()/2, 0,
          0,  f_, image_size.y()/2, 0,
          0,  0,  1, 0,
          0,  0,  0, 1;
  };

  float pixel_size() {return pixel_size_;}
  float focal_length() {return focal_length_;}
  float f() {return f_;}
  Eigen::Vector2i image_size() {return image_size_;}
  Eigen::Matrix4f Twc() {return Twc_;}
  Eigen::Vector3f twc() {return Twc_.topRightCorner(3,1);}
  Eigen::Matrix3f Rwc() {return Twc_.topLeftCorner(3,3);};
  Eigen::Matrix4f K() {return K_;}

private:
  float pixel_size_; // [m]
  float focal_length_; // [m]
  float f_; //[vox]
  Eigen::Vector2i image_size_;
  Eigen::Matrix4f Twc_;
  Eigen::Matrix4f K_;
};

struct sphere {
public:
  sphere(Eigen::Vector3f center, float radius)
    : center_(center), radius_(radius) {};

  sphere(camera_parameter camera_parameter, Eigen::Vector2f center_angle,
      float center_distance, float radius)
      : radius_(radius) {
    Eigen::Matrix3f Rwc = camera_parameter.Rwc();
    Eigen::Vector3f twc = camera_parameter.twc();
    float dist_y = std::sin(center_angle.x())*center_distance;
    float dist_x = std::cos(center_angle.x())*std::sin(center_angle.y())*center_distance;
    float dist_z = std::cos(center_angle.x())*std::cos(center_angle.y())*center_distance;
    Eigen::Vector3f dist(dist_x, dist_y, dist_z);
    center_ = Rwc*dist + twc;
  };

  Eigen::Vector3f center() {return center_;}
  float radius() {return radius_;}

private:
  Eigen::Vector3f center_;
  float radius_;
};

struct ray {
public:
  ray(camera_parameter& camera_parameter)
      : direction_(Eigen::Vector3f(-1.f,-1.f,-1.f)),
        origin_(camera_parameter.twc()),
        Rwc_(camera_parameter.Rwc()) {
  focal_length_in_pixel_ = camera_parameter.f();
  offset_ = camera_parameter.image_size()/2;
  };

  void operator()(int pixel_u, int pixel_v) {
    direction_.x() = -offset_.x() + 0.5 + pixel_u;
    direction_.y() = -offset_.y() + 0.5 + pixel_v;
    direction_.z() = focal_length_in_pixel_;
    direction_.normalize();
    direction_ = Rwc_*direction_;
  };

  Eigen::Vector3f origin() {return  origin_;}
  Eigen::Vector3f direction() {return direction_;}

private:
  float focal_length_in_pixel_;
  Eigen::Vector2i offset_;
  Eigen::Vector3f origin_;
  Eigen::Matrix3f Rwc_;
  Eigen::Vector3f direction_;
};

struct sphere_intersection {
public:
  sphere_intersection(sphere sphere_1, sphere sphere_2)
      : sphere_1_(sphere_1), sphere_2_(sphere_2) {};

   float operator()(ray& ray) {
    Eigen::Vector3f oc_1 = ray.origin() - sphere_1_.center();
    float a_1 = ray.direction().dot(ray.direction());
    float b_1 = 2.0 * oc_1.dot(ray.direction());
    float c_1 = oc_1.dot(oc_1) - sphere_1_.radius()*sphere_1_.radius();
    float discriminant_1 = b_1*b_1 - 4*a_1*c_1;

    Eigen::Vector3f oc_2 = ray.origin() - sphere_2_.center();
    float a_2 = ray.direction().dot(ray.direction());
    float b_2 = 2.0 * oc_2.dot(ray.direction());
    float c_2 = oc_2.dot(oc_2) - sphere_2_.radius()*sphere_2_.radius();
    float discriminant_2 = b_2*b_2 - 4*a_2*c_2;

    if(discriminant_1 < 0 && discriminant_2 < 0){
      return 20;
    } else if(discriminant_1 < 0 && discriminant_2 >= 0) {
      return (-b_2 - sqrt(discriminant_2))/(2.0*a_2);
    } else if(discriminant_1 >= 0 && discriminant_2 < 0) {
      return (-b_1 - sqrt(discriminant_1))/(2.0*a_1);
    } else {
      float dist_1 = (-b_1 - sqrt(discriminant_1))/(2.0*a_1);
      float dist_2 = (-b_2 - sqrt(discriminant_2))/(2.0*a_2);
      return std::min(dist_1, dist_2);
    }
  };

private:
  sphere sphere_1_;
  sphere sphere_2_;
};

void generateDepthImage(float* depth_image, camera_parameter camera_parameter,
    sphere& sphere_close, sphere& sphere_far) {
  sphere_intersection si(sphere_close, sphere_far);
  ray ray(camera_parameter);
  int image_width = camera_parameter.image_size().x();
  int image_height = camera_parameter.image_size().y();
  for (int u = 0; u < image_width; u++) {
    for (int v = 0; v < image_height; v++) {
      ray(u,v);
      float range = si(ray);
      float regularisation = std::sqrt(1 + se::math::sq(std::abs(u - image_width/2) / camera_parameter.f()) + se::math::sq(std::abs(v - image_height/2) / camera_parameter.f()));
      float depth = range/regularisation;
      static std::mt19937 gen{1};
      std::normal_distribution<> d(0, 0.004*depth*depth);
      depth_image[u + v*camera_parameter.image_size().x()] = depth;// + d(gen);
    }
  }

  cv::Mat depth_image_cv(camera_parameter.image_size().y(), camera_parameter.image_size().x(), CV_16UC1);
  for (uint u = 0; u < camera_parameter.image_size().x(); u++) {
    for (uint v = 0; v < camera_parameter.image_size().y(); v++) {
      uint16_t depth_int = static_cast<uint16_t>(depth_image[u + v*camera_parameter.image_size().x()] * 1000);
      depth_image_cv.at<uint16_t>(v, u) = depth_int;
    }
  }

  const std::string depth_image_filename = "/home/nils/workspace_ptp/catkin_ws/src/probabilistic_trajectory_planning_ros/ext/probabilistic_trajectory_planning/src/ext/supereight/se_denseslam/test/out/depth_image.png";
  cv::imwrite(depth_image_filename, depth_image_cv);

}


class MultiscaleIntegrationTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    size_ = 512;                              // 512 x 512 x 512 voxel^3
    voxel_size_ = 0.005;                      // 5 mm/voxel
    float dim_ = size_ * voxel_size_;         // [m^3]
    oct_.init(size_, dim_);
    Eigen::Vector2i image_size(640, 480);    // width x height
    Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
    camera_pose.topLeftCorner<3,3>() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    camera_pose.topRightCorner<3,1>() = Eigen::Vector3f(-0.25*size_, size_/2, size_/2)*voxel_size_;
    camera_parameter_ = camera_parameter(0.006, 1.95, image_size, camera_pose);

    // Generate depth image
    depth_image_ =
        (float *) malloc(sizeof(float) * image_size.x() * image_size.y());

    // Allocate spheres in world frame
    sphere sphere_close = sphere(voxel_size_*Eigen::Vector3f(size_*6/7, size_*3/4, size_/2), 0.3f);
    sphere sphere_far = sphere(voxel_size_*Eigen::Vector3f(size_*6/7, size_*1/4, size_/2), 0.5f);

    // Allocate spheres relative to camera
//    sphere sphere_close(camera_parameter_, Eigen::Vector2f(0.0, 0.5), 5.f, 1.f);
//    sphere sphere_far(camera_parameter_, Eigen::Vector2f(0.0, -0.1), 5.f, 1.f);

    generateDepthImage(depth_image_, camera_parameter_, sphere_close, sphere_far);

    const int side = se::VoxelBlock<ESDF>::side;
    for(int z = side/2; z < size_; z += side) {
      for(int y = side/2; y < size_; y += side) {
        for(int x = side/2; x < size_; x += side) {
          const Eigen::Vector3i vox(x, y, z);
          alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2)));
        }
      }
    }
    oct_.allocate(alloc_list.data(), alloc_list.size());
  }

  float* depth_image_;
  camera_parameter camera_parameter_;

  typedef se::Octree<ESDF> OctreeT;
  OctreeT oct_;
  int size_;
  float voxel_size_;
  float dim_;
  std::vector<se::VoxelBlock<ESDF>*> active_list_;

private:
  std::vector<se::key_t> alloc_list;
};

struct calculate_scale {
public:
  calculate_scale(float voxelsize, camera_parameter camera_parameter) {
    voxelsize_ = voxelsize;
    pixel_size_ = camera_parameter.pixel_size();
    focal_length_ = camera_parameter.focal_length();
    camera_position_ = camera_parameter.twc();
  }

  int operator()(Eigen::Vector3i base, const int side) {
    float distance = (voxelsize_*(base.cast<float>() +
        Eigen::Vector3f(0.5*(side + 1), 0.5*(side + 1), 0.5*(side + 1)))
        - camera_position_).norm();
    float uncertainty = pixel_size_/focal_length_*distance;
    int scale = std::max(0,int(log2(uncertainty/voxelsize_) + 1));
    return std::min(scale, 3);
  }

private:
  float voxelsize_;
  float pixel_size_;
  float focal_length_;
  Eigen::Vector3f camera_position_;
};

template <typename T>
void propagate_down(se::VoxelBlock<T>* block, const int scale) {
  const Eigen::Vector3i base = block->coordinates();
  const int side = se::VoxelBlock<T>::side;
  for(int curr_scale = scale; curr_scale > 0; --curr_scale) {
    const int stride = 1 << curr_scale;
    for (int z = 0; z < side; z += stride)
      for (int y = 0; y < side; y += stride)
        for (int x = 0; x < side; x += stride) {
          const Eigen::Vector3i parent = base + Eigen::Vector3i(x, y, z);
          auto data = block->data(parent, curr_scale);
          for (int k = 0; k < stride; ++k)
            for (int j = 0; j < stride; ++j)
              for (int i = 0; i < stride; ++i) {
                const Eigen::Vector3i vox = parent + Eigen::Vector3i(i, j, k);
                auto curr = block->data(vox, curr_scale - 1);
                curr.x += data.delta;
                curr.y += data.delta_y;
                block->data(vox, curr_scale - 1, curr);
              }
          data.delta_y = 0;
          block->data(parent, curr_scale, data);
        }
  }
}

template <typename T>
void propagate_up(se::VoxelBlock<T>* block, const int scale) {
  const Eigen::Vector3i base = block->coordinates();
  const int side = se::VoxelBlock<T>::side;
  for(int curr_scale = scale; curr_scale < se::math::log2_const(side) - 1; ++curr_scale) {
    const int stride = 1 << (curr_scale + 1);
    for (int z = 0; z < side; z += stride)
      for (int y = 0; y < side; y += stride)
        for (int x = 0; x < side; x += stride) {
          const Eigen::Vector3i curr = base + Eigen::Vector3i(x, y, z);

          float mean = 0;
          int num_samples = 0;
          float weight = 0;
          for (int k = 0; k < stride; ++k)
            for (int j = 0; j < stride; ++j)
              for (int i = 0; i < stride; ++i) {
                auto tmp = block->data(curr + Eigen::Vector3i(i, j, k), curr_scale);
                mean += tmp.x;
                weight += tmp.y;
                num_samples++;
              }
          mean /= num_samples;
          weight /= num_samples;
          auto data = block->data(curr, curr_scale + 1);
          data.x = mean;
          data.y = weight;
          data.delta = 0;
          data.delta_y = 0;
          block->data(curr, curr_scale + 1, data);
        }
  }
}

template <typename T>
void foreach(float voxelsize, std::vector<se::VoxelBlock<T>*> active_list,
             camera_parameter camera_parameter, float* depth_image) {
  const int n = active_list.size();
  calculate_scale calculate_scale(voxelsize, camera_parameter);
  for(int i = 0; i < n; ++i) {
    se::VoxelBlock<T>* block = active_list[i];
    const Eigen::Vector3i base = block->coordinates();
    const int side = se::VoxelBlock<T>::side;

    const Eigen::Matrix4f Tcw = (camera_parameter.Twc()).inverse();
    const Eigen::Matrix3f Rcw = Tcw.topLeftCorner<3,3>();
    const Eigen::Vector3f tcw = Tcw.topRightCorner<3,1>();
    const Eigen::Matrix4f K = camera_parameter.K();
    const Eigen::Vector2i image_size = camera_parameter.image_size();

    // Calculate the maximum uncertainty possible
    int scale = calculate_scale(base, side);
    float stride = std::max(int(pow(2,scale)),1);
    for(float z = stride/2; z < side; z += stride) {
      for (float y = stride/2; y < side; y += stride) {
        for (float x = stride/2; x < side; x += stride) {
          const Eigen::Vector3f node_w = base.cast<float>() + Eigen::Vector3f(x, y, z);
          const Eigen::Vector3f node_c = Rcw * (voxelsize * node_w)+ tcw;
          auto data = block->data(node_w.cast<int>(), scale);
          if (node_c.z() < 0.0001f)
            continue;
          const Eigen::Vector3f pixel_homo = K.topLeftCorner<3, 3>() * node_c;
          const float inverse_depth = 1.f / pixel_homo.z();
          const Eigen::Vector2f pixel = Eigen::Vector2f(
              pixel_homo.x() * inverse_depth + 0.5f,
              pixel_homo.y() * inverse_depth + 0.5f);
          if (pixel(0) < 0.5f || pixel(0) > image_size.x() - 1.5f ||
              pixel(1) < 0.5f || pixel(1) > image_size.y() - 1.5f)
            continue;
          float mu = 0.1;
          float depth = depth_image[int(pixel.x()) + image_size.x()*int(pixel.y())];
          const float diff = (depth - node_c.z())
                             * std::sqrt( 1 + se::math::sq(node_c.x() / node_c.z()) + se::math::sq(node_c.y() / node_c.z()));
          if (diff > -mu) {
            const float sample = fminf(2.f, diff / mu);
            data.delta = (sample - data.x)/(data.y + 1);
            data.delta_y++;
            data.x = (data.x * data.y + sample)/(data.y + 1);
            data.y = data.y + 1;
            block->data(node_w.cast<int>(), scale, data);
          }
        }
      }
    }
    if(scale > 0)
      propagate_down(block, scale);
    if(scale < log2(side))
      propagate_up(block, scale);
  }
}

template <typename T>
std::vector<se::VoxelBlock<ESDF>*> buildActiveList(se::Octree<T>& map, camera_parameter camera_parameter, float voxel_size) {
  const se::MemoryPool<se::VoxelBlock<ESDF> >& block_array =
      map.getBlockBuffer();
  for(unsigned int i = 0; i < block_array.size(); ++i) {
    block_array[i]->active(false);
  }

  const Eigen::Matrix4f K = camera_parameter.K();
  const Eigen::Matrix4f Twc = camera_parameter.Twc();
  const Eigen::Matrix4f Tcw = (camera_parameter.Twc()).inverse();
  std::vector<se::VoxelBlock<ESDF>*> active_list;
  auto in_frustum_predicate =
      std::bind(se::algorithms::in_frustum<se::VoxelBlock<ESDF>>, std::placeholders::_1,
                voxel_size, K*Tcw, camera_parameter.image_size());
  se::algorithms::filter(active_list, block_array, in_frustum_predicate);
  return active_list;
}

TEST_F(MultiscaleIntegrationTest, Integration) {
  active_list_ = buildActiveList(oct_, camera_parameter_, voxel_size_);
  foreach(voxel_size_, active_list_, camera_parameter_, depth_image_);
  std::stringstream f;
  f << "/home/nils/workspace_ptp/catkin_ws/src/probabilistic_trajectory_planning_ros/ext/probabilistic_trajectory_planning/src/ext/supereight/se_denseslam/test/out/scaled_integration.vtk";
  save3DSlice(oct_,
              Eigen::Vector3i(0, 0, oct_.size()/2),
              Eigen::Vector3i(oct_.size(), oct_.size(), oct_.size()/2 + 1),
              [](const auto& val) { return val.x; }, f.str().c_str());
  free(depth_image_);
}

