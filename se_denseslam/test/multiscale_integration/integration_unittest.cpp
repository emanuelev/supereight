#include <se/octant_ops.hpp>
#include <se/octree.hpp>
#include <se/algorithms/balancing.hpp>
#include <se/functors/axis_aligned_functor.hpp>
#include <se/algorithms/filter.hpp>
#include <se/volume_traits.hpp>
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

// Truncation distance and maximum weight
#define MAX_DIST 2.f
#define MAX_WEIGHT 5
#define MU 0.1f

// Fusion level,
// 0-3 are the according levels in the voxel_block
// 4 is multilevel fusion
#define SCALE 0

// Returned distance when ray doesn't intersect sphere
#define SENSOR_LIMIT 20

// Camera movement,
// 0 = linear movement away from single sphere
// 1 = circular movement around two spheres
#define MOVEMENT 0

// Number of frames to move from start to end position
#define FRAMES 16

// Activate (1) and deactivate (0) depth dependent noise
#define NOISE 0

struct camera_parameter {
public:
  camera_parameter() {};

  camera_parameter(float pixel_size_mm, float focal_length_mm,
      Eigen::Vector2i image_size, Eigen::Matrix4f Twc)
    : pixel_size_mm_(pixel_size_mm), focal_length_mm_(focal_length_mm),
    image_size_(image_size), Twc_(Twc) {
    focal_length_pix_ = focal_length_mm/pixel_size_mm;
    K_ << focal_length_pix_, 0                , image_size.x()/2, 0,
          0                , focal_length_pix_, image_size.y()/2, 0,
          0                , 0                , 1               , 0,
          0                , 0                , 0               , 1;
  };

  float pixel_size_mm() {return pixel_size_mm_;}
  float focal_length_mm() {return focal_length_mm_;}
  float focal_length_pix() {return focal_length_pix_;}
  void setPose(Eigen::Matrix4f Twc) {Twc_ = Twc;}
  Eigen::Vector2i imageSize() {return image_size_;}
  Eigen::Matrix4f Twc() {return Twc_;}
  Eigen::Vector3f twc() {return Twc_.topRightCorner(3,1);}
  Eigen::Matrix3f Rwc() {return Twc_.topLeftCorner(3,3);};
  Eigen::Matrix4f K() {return K_;}

private:
  float pixel_size_mm_; // [mm]
  float focal_length_mm_; // [mm]
  float focal_length_pix_; //[vox]
  Eigen::Vector2i image_size_;
  Eigen::Matrix4f Twc_;
  Eigen::Matrix4f K_;
};

struct sphere {
public:
  sphere() {};
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
        Rwc_(camera_parameter.Rwc()),
        focal_length_pix_(camera_parameter.focal_length_pix()),
        offset_(camera_parameter.imageSize()/2) {};

  void operator()(int pixel_u, int pixel_v) {
    direction_.x() = -offset_.x() + 0.5 + pixel_u;
    direction_.y() = -offset_.y() + 0.5 + pixel_v;
    direction_.z() = focal_length_pix_;
    direction_.normalize();
    direction_ = Rwc_*direction_;
  };

  Eigen::Vector3f origin() {return  origin_;}
  Eigen::Vector3f direction() {return direction_;}

private:
  float focal_length_pix_;
  Eigen::Vector2i offset_;
  Eigen::Vector3f origin_;
  Eigen::Matrix3f Rwc_;
  Eigen::Vector3f direction_;
};

struct sphere_intersection {
public:
  sphere_intersection() {};
  sphere_intersection(std::vector<sphere> spheres)
      : spheres_(spheres) {};

  float operator()(ray& ray) {
    float dist(SENSOR_LIMIT);
    for (std::vector<sphere>::iterator sphere = spheres_.begin(); sphere != spheres_.end(); ++sphere) {
      Eigen::Vector3f oc = ray.origin() - sphere->center();
      float a = ray.direction().dot(ray.direction());
      float b = 2.0 * oc.dot(ray.direction());
      float c = oc.dot(oc) - sphere->radius()*sphere->radius();
      float discriminant = b*b - 4*a*c;
      if (discriminant >= 0) {
        float dist_tmp = (-b - sqrt(discriminant))/(2.0*a);
        if (dist_tmp < dist)
          dist = dist_tmp;
      }
    }
    return dist;
  };

private:
  std::vector<sphere> spheres_;
};

struct generate_depth_image {
public:
  generate_depth_image() {};
  generate_depth_image(float* depth_image, std::vector<sphere>& spheres)
    : depth_image_(depth_image),
      si_(sphere_intersection(spheres)) {};

  void operator()(camera_parameter camera_parameter) {
    float focal_length_pix = camera_parameter.focal_length_pix();
    ray ray(camera_parameter);
    int image_width = camera_parameter.imageSize().x();
    int image_height = camera_parameter.imageSize().y();
    for (int u = 0; u < image_width; u++) {
      for (int v = 0; v < image_height; v++) {
        ray(u,v);
        float range = si_(ray);
        float regularisation = std::sqrt(1 + se::math::sq(std::abs(u + 0.5 - image_width/2) / focal_length_pix)
                                           + se::math::sq(std::abs(v + 0.5 - image_height/2) / focal_length_pix));
        float depth = range/regularisation;
        if(NOISE) {
          static std::mt19937 gen{1};
          std::normal_distribution<> d(0, 0.004*depth*depth);
          depth_image_[u + v*camera_parameter.imageSize().x()] = depth + d(gen);
        }
        else
          depth_image_[u + v*camera_parameter.imageSize().x()] = depth;
      }
    }
  }

private:
  float* depth_image_;
  sphere_intersection si_;
};

struct calculate_scale {
public:
  calculate_scale(float voxelsize, camera_parameter camera_parameter) {
    voxelsize_ = voxelsize;
    pixel_size_mm_ = camera_parameter.pixel_size_mm();
    focal_length_mm_ = camera_parameter.focal_length_mm();
    camera_position_ = camera_parameter.twc();
  }

  int operator()(Eigen::Vector3i base, const int side) {
    float distance = (voxelsize_*(base.cast<float>() +
        Eigen::Vector3f(0.5*(side + 1), 0.5*(side + 1), 0.5*(side + 1)))
        - camera_position_).norm();
    float uncertainty = pixel_size_mm_/focal_length_mm_*distance;
    int scale = std::max(0,int(log2(uncertainty/voxelsize_) + 1));
    return std::min(scale, 1);
  }

private:
  float voxelsize_;
  float pixel_size_mm_;
  float focal_length_mm_;
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

          float virt_sample = data.delta*float(data.y - 1) + data.x;
          typedef voxel_traits<T> traits_type;
          typedef typename traits_type::value_type value_type;
          value_type curr_list[8];
          Eigen::Vector3i vox_list[8];
          float delta_sum(0);

          for (int k = 0; k < stride; k += stride/2)
            for (int j = 0; j < stride; j += stride/2)
              for (int i = 0; i < stride; i += stride/2) {
                int idx = 1*(2*i/stride) + 2*(2*j/stride) + 4*(2*k/stride);
                vox_list[idx] = parent + Eigen::Vector3i(i, j, k);
                auto curr = block->data(vox_list[idx], curr_scale -1);
                // Calculate non normalized child delta
                curr.delta = virt_sample - curr.x;
                delta_sum += curr.delta;
                curr_list[idx] = curr;
              }

          for (int i = 0; i < 8; i++) {
            // Update delta_x
            if (delta_sum != 0)
              curr_list[i].delta = data.delta*curr_list[i].delta/delta_sum*8;

            // Update x
            curr_list[i].x = curr_list[i].y == 0 ? data.x :
                curr_list[i].x += curr_list[i].delta;

            // Update weight (with 0 <= y <= MAX_WEIGHT)
            curr_list[i].y = curr_list[i].y == 0 ? data.y :
                             std::min(curr_list[i].y + data.delta_y, MAX_WEIGHT);
            curr_list[i].delta_y =data.delta_y;

            block->data(vox_list[i], curr_scale - 1, curr_list[i]);
          }

          data.delta = 0;
          data.delta_y = 0;
          block->data(parent, curr_scale, data);
        }
  }
}

template <typename T>
void propagate_up(se::VoxelBlock<T>* block, const int scale) {
  const Eigen::Vector3i base = block->coordinates();
  const int side = se::VoxelBlock<T>::side;
  for(int curr_scale = scale; curr_scale < se::math::log2_const(side); ++curr_scale) {
    const int stride = 1 << (curr_scale + 1);
    for (int z = 0; z < side; z += stride)
      for (int y = 0; y < side; y += stride)
        for (int x = 0; x < side; x += stride) {
          const Eigen::Vector3i curr = base + Eigen::Vector3i(x, y, z);

          float mean = 0;
          int num_samples = 0;
          float weight = 0;
          for (int k = 0; k < stride; k += stride/2)
            for (int j = 0; j < stride; j += stride/2)
              for (int i = 0; i < stride; i += stride/2) {
                auto tmp = block->data(curr + Eigen::Vector3i(i, j, k), curr_scale);
                if (tmp.y != 0) {
                  mean += tmp.x;
                  weight += tmp.y;
                  num_samples++;
                }
              }

          auto data = block->data(curr, curr_scale + 1);

          if (num_samples != 0) {
            // Update SDF value to mean of its children
            mean /= num_samples;
            data.x = mean;

            // Update weight (round up if > 0.5, round down otherwise)
            weight /= num_samples;
            data.y = ceil(weight);
          } else {
            data = voxel_traits<MultiresSDF>::initValue();
          }

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
    const Eigen::Vector2i image_size = camera_parameter.imageSize();

    // Calculate the maximum uncertainty possible
    int scale = calculate_scale(base, side);
    if (SCALE != 4)
      scale = SCALE;
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
              pixel_homo.x() * inverse_depth,
              pixel_homo.y() * inverse_depth);
          if (pixel(0) < 0.5f || pixel(0) > image_size.x() - 1.5f ||
              pixel(1) < 0.5f || pixel(1) > image_size.y() - 1.5f)
            continue;

          float depth = depth_image[int(pixel.x()) + image_size.x()*int(pixel.y())];
          const float diff = (depth - node_c.z()) * std::sqrt( 1 + se::math::sq(node_c.x() / node_c.z()) + se::math::sq(node_c.y() / node_c.z()));
          if (diff > -MU) {
            const float sample = fminf(MAX_DIST, diff);

            // Make sure that the max weight isn't greater than MAX_WEIGHT (i.e. y + 1)
            data.y = std::min(data.y, MAX_WEIGHT - 1);

            // Update SDF value
            data.delta = (sample - data.x)/(data.y + 1);
            data.x = (data.x * data.y + sample)/(data.y + 1);

            // Update weight
            data.delta_y++;
            data.y = data.y + 1;

            block->data(node_w.cast<int>(), scale, data);
          }
        }
      }
    }

    propagate_down(block, scale);
    propagate_up(block, 0);
  }
}

template <typename T>
std::vector<se::VoxelBlock<MultiresSDF>*> buildActiveList(se::Octree<T>& map, camera_parameter camera_parameter, float voxel_size) {
  const se::MemoryPool<se::VoxelBlock<MultiresSDF> >& block_array =
      map.getBlockBuffer();
  for(unsigned int i = 0; i < block_array.size(); ++i) {
    block_array[i]->active(false);
  }

  const Eigen::Matrix4f K = camera_parameter.K();
  const Eigen::Matrix4f Twc = camera_parameter.Twc();
  const Eigen::Matrix4f Tcw = (camera_parameter.Twc()).inverse();
  std::vector<se::VoxelBlock<MultiresSDF>*> active_list;
  auto in_frustum_predicate =
      std::bind(se::algorithms::in_frustum<se::VoxelBlock<MultiresSDF>>, std::placeholders::_1,
                voxel_size, K*Tcw, camera_parameter.imageSize());
  se::algorithms::filter(active_list, block_array, in_frustum_predicate);
  return active_list;
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
    camera_parameter_ = camera_parameter(0.006, 1.95, image_size, camera_pose);

    const int side = se::VoxelBlock<MultiresSDF>::side;
    for(int z = side/2; z < size_; z += side) {
      for(int y = side/2; y < size_; y += side) {
        for(int x = side/2; x < size_; x += side) {
          const Eigen::Vector3i vox(x, y, z);
          alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2)));
        }
      }
    }
    oct_.allocate(alloc_list.data(), alloc_list.size());

    // Generate depth image
    depth_image_ =
        (float *) malloc(sizeof(float) * image_size.x() * image_size.y());

    std::vector<sphere> spheres;

    // Allocate spheres in world frame
    if(MOVEMENT == 0) {
      spheres.push_back(sphere(voxel_size_*Eigen::Vector3f(size_*1/2, size_*1/2, size_/2), 0.5f));
    } else {
      sphere sphere_close = sphere(voxel_size_*Eigen::Vector3f(size_*1/8, size_*2/3, size_/2), 0.3f);
      sphere sphere_far = sphere(voxel_size_*Eigen::Vector3f(size_*7/8, size_*1/3, size_/2), 0.3f);
      spheres.push_back(sphere_close);
      spheres.push_back(sphere_far);
    }
    generate_depth_image_ = generate_depth_image(depth_image_, spheres);
  }

  float* depth_image_;
  camera_parameter camera_parameter_;

  typedef se::Octree<MultiresSDF> OctreeT;
  OctreeT oct_;
  int size_;
  float voxel_size_;
  float dim_;
  std::vector<se::VoxelBlock<MultiresSDF>*> active_list_;
  generate_depth_image generate_depth_image_;

private:
  std::vector<se::key_t> alloc_list;
};

TEST_F(MultiscaleIntegrationTest, Integration) {
  int frames = FRAMES;
  for (int frame = 0; frame < frames; frame++) {
    Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f Rbc;
    Rbc << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    double angle = 0;
    if(MOVEMENT == 1) { // Move camera from -45deg to +45deg in FRAMES steps
      angle = float(frame)/float(frames) * 2 * M_PI / 4 - 2 * M_PI / 8;
    }

    Eigen::Matrix3f Rwb;
    Rwb <<  std::cos(angle), -std::sin(angle), 0,
            std::sin(angle),  std::cos(angle), 0,
                          0,                0, 1;
    camera_pose.topLeftCorner<3,3>()  = Rwb*Rbc;

    if(MOVEMENT == 0)
      camera_pose.topRightCorner<3,1>() = (Rwb*Eigen::Vector3f(-(size_/2 + frame*size_/8), 0, size_/2) + Eigen::Vector3f(size_/2, size_/2, 0))*voxel_size_;
    else
      camera_pose.topRightCorner<3,1>() = (Rwb*Eigen::Vector3f(-(size_/2 + 16*size_/8), 0, size_/2) + Eigen::Vector3f(size_/2, size_/2, 0))*voxel_size_;

    camera_parameter_.setPose(camera_pose);
    generate_depth_image_(camera_parameter_);
    active_list_ = buildActiveList(oct_, camera_parameter_, voxel_size_);
    foreach(voxel_size_, active_list_, camera_parameter_, depth_image_);
    std::stringstream f;

    if(MOVEMENT == 0)
      f << "/home/nils/workspace_ptp/catkin_ws/src/probabilistic_trajectory_planning_ros/ext/probabilistic_trajectory_planning/src/ext/supereight/se_denseslam/test/out/scale_"  + std::to_string(SCALE) + "-linear_back_move-" + std::to_string(frame) + ".vtk";
    else
      f << "/home/nils/workspace_ptp/catkin_ws/src/probabilistic_trajectory_planning_ros/ext/probabilistic_trajectory_planning/src/ext/supereight/se_denseslam/test/out/scale_"  + std::to_string(SCALE) + "-circular_move-" + std::to_string(frame) + ".vtk";

    save3DSlice(oct_,
                Eigen::Vector3i(0, 0, oct_.size()/2),
                Eigen::Vector3i(oct_.size(), oct_.size(), oct_.size()/2 + 1),
                [](const auto& val) { return val.x; }, f.str().c_str());
  }
  free(depth_image_);

}

