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
#define SCALE 4

// Returned distance when ray doesn't intersect sphere
#define SENSOR_LIMIT 20

// Number of frames to move from start to end position
#define FRAMES 16

// Activate (1) and deactivate (0) depth dependent noise
#define NOISE 0

// Box intersection
#define RIGHT	0
#define LEFT	1
#define MIDDLE	2

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

struct obstacle {
  virtual float intersect(ray& ray) = 0;
};

struct sphere_obstacle : obstacle {
public:
  sphere_obstacle() {};
  sphere_obstacle(Eigen::Vector3f center, float radius)
  : center_(center), radius_(radius) {};

  sphere_obstacle(camera_parameter camera_parameter, Eigen::Vector2f center_angle,
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

  float intersect(ray& ray) {
    float dist(SENSOR_LIMIT);
    Eigen::Vector3f oc = ray.origin() - center_;
    float a = ray.direction().dot(ray.direction());
    float b = 2.0 * oc.dot(ray.direction());
    float c = oc.dot(oc) - radius_*radius_;
    float discriminant = b*b - 4*a*c;
    if (discriminant >= 0) {
      float dist_tmp = (-b - sqrt(discriminant))/(2.0*a);
      if (dist_tmp < dist)
        dist = dist_tmp;
    }
    return dist;
  };

  Eigen::Vector3f center() {return center_;}
  float radius() {return radius_;}

private:
  Eigen::Vector3f center_;
  float radius_;
};

struct box_obstacle : obstacle {
public:
  box_obstacle() {};
  box_obstacle(Eigen::Vector3f center, float depth, float width, float height)
  : center_(center), dim_(Eigen::Vector3f(depth, width, height)) {
    min_corner_ = center - Eigen::Vector3f(depth, width, height);
    max_corner_ = center + Eigen::Vector3f(depth, width, height);
  };

  box_obstacle(Eigen::Vector3f center, Eigen::Vector3f dim)
  : center_(center), dim_(dim) {
    min_corner_ = center - dim/2;
    max_corner_ = center + dim/2;
  };

  float intersect(ray& ray) {
    float dist(SENSOR_LIMIT);
    /*
    Fast Ray-Box Intersection
    by Andrew Woo
    from "Graphics Gems", Academic Press, 1990
    */
    int num_dim = 3;
    Eigen::Vector3f hit_point = -1*Eigen::Vector3f::Ones();				/* hit point */
    {
      bool inside = true;
      Eigen::Vector3i quadrant;
      int which_plane;
      Eigen::Vector3f max_T;
      Eigen::Vector3f candidate_plane;

      /* Find candidate planes; this loop can be avoided if
         rays cast all from the eye(assume perpsective view) */
      for (int i = 0; i < num_dim; i++)
        if(ray.origin()[i] < min_corner_[i]) {
          quadrant[i] = LEFT;
          candidate_plane[i] = min_corner_[i];
          inside = false;
        }else if (ray.origin()[i] > max_corner_[i]) {
          quadrant[i] = RIGHT;
          candidate_plane[i] = max_corner_[i];
          inside = false;
        }else	{
          quadrant[i] = MIDDLE;
        }

      /* Ray origin inside bounding box */
      if(inside)	{
        return 0;
      }

      /* Calculate T distances to candidate planes */
      for (int i = 0; i < num_dim; i++)
        if (quadrant[i] != MIDDLE && ray.direction()[i] !=0.)
          max_T[i] = (candidate_plane[i]-ray.origin()[i]) / ray.direction()[i];
        else
          max_T[i] = -1.;

      /* Get largest of the max_T's for final choice of intersection */
      which_plane = 0;
      for (int i = 1; i < num_dim; i++)
        if (max_T[which_plane] < max_T[i])
          which_plane = i;

      /* Check final candidate actually inside box */
      if (max_T[which_plane] < 0.f) return dist;
      for (int i = 0; i < num_dim; i++)
        if (which_plane != i) {
          hit_point[i] = ray.origin()[i] + max_T[which_plane] *ray.direction()[i];
          if (hit_point[i] < min_corner_[i] || hit_point[i] > max_corner_[i])
            return dist;
        } else {
          hit_point[i] = candidate_plane[i];
        }

      dist = (hit_point - ray.origin()).norm();
      return dist;
    }
  };

  Eigen::Vector3f center() {return center_;}
  Eigen::Vector3f dim() {return dim_;}
  Eigen::Vector3f min_corner() {return min_corner_;}
  Eigen::Vector3f max_corner() {return max_corner_;}

private:
  Eigen::Vector3f center_;
  Eigen::Vector3f dim_;
  Eigen::Vector3f min_corner_;
  Eigen::Vector3f max_corner_;
};

struct generate_depth_image {
public:
  generate_depth_image() {};
  generate_depth_image(float* depth_image, std::vector<obstacle*> obstacles)
      : depth_image_(depth_image), obstacles_(obstacles) {};

  void operator()(camera_parameter camera_parameter) {
    float focal_length_pix = camera_parameter.focal_length_pix();
    ray ray(camera_parameter);
    int image_width = camera_parameter.imageSize().x();
    int image_height = camera_parameter.imageSize().y();

    for (int u = 0; u < image_width; u++) {
      for (int v = 0; v < image_height; v++) {
        ray(u,v);
        float dist(SENSOR_LIMIT);
        for (std::vector<obstacle*>::iterator obstacle = obstacles_.begin(); obstacle != obstacles_.end(); ++obstacle) {
          float dist_tmp = (*obstacle)->intersect(ray);
          if (dist_tmp < dist)
            dist = dist_tmp;
        }

        float regularisation = std::sqrt(1 + se::math::sq(std::abs(u + 0.5 - image_width/2) / focal_length_pix)
                                         + se::math::sq(std::abs(v + 0.5 - image_height/2) / focal_length_pix));
        float depth = dist/regularisation;
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
  std::vector<obstacle*> obstacles_;

};

inline float compute_scale(const Eigen::Vector3f& vox, 
                    const Eigen::Vector3f& twc,
                    const float scaled_pix,
                    const float voxelsize) {
  const float dist = (voxelsize * vox - twc).norm();
  const float pix_size = dist * scaled_pix;
  int scale = std::min(std::max(0, int(log2(pix_size/voxelsize) + 1)),
                       3);
  return scale;
}

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

          int idx = 0;
          for (int k = 0; k < stride; k += stride/2)
            for (int j = 0; j < stride; j += stride/2)
              for (int i = 0; i < stride; i += stride/2) {
                vox_list[idx] = parent + Eigen::Vector3i(i, j, k);
                auto curr = block->data(vox_list[idx], curr_scale -1);
                // Calculate non normalized child delta
                curr.delta = virt_sample - curr.x;
                delta_sum += curr.delta;
                curr_list[idx] = curr;
                ++idx;
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
  for(int i = 0; i < n; ++i) {
    se::VoxelBlock<T>* block = active_list[i];
    const Eigen::Vector3i base = block->coordinates();
    const int side = se::VoxelBlock<T>::side;

    const Eigen::Matrix4f Tcw = (camera_parameter.Twc()).inverse();
    const Eigen::Matrix3f Rcw = Tcw.topLeftCorner<3,3>();
    const Eigen::Vector3f tcw = Tcw.topRightCorner<3,1>();
    const Eigen::Matrix4f K = camera_parameter.K();
    const Eigen::Vector2i image_size = camera_parameter.imageSize();
    const float scaled_pix = (camera_parameter.K().inverse() * (Eigen::Vector3f(1, 0 ,1) - Eigen::Vector3f(0, 0, 1)).homogeneous()).x();

    // Calculate the maximum uncertainty possible
    int scale = compute_scale((base + Eigen::Vector3i::Constant(side/2)).cast<float>(),
                               tcw, scaled_pix, voxelsize);
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

class MultiscaleTSDFMovingCameraTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    size_ = 512;                              // 512 x 512 x 512 voxel^3
    voxel_size_ = 0.005;                      // 5 mm/voxel
    dim_ = size_ * voxel_size_;         // [m^3]
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

TEST_F(MultiscaleTSDFMovingCameraTest, SphereTranslation) {
  std::vector<obstacle*> spheres;

  // Allocate spheres in world frame
  spheres.push_back(new sphere_obstacle(voxel_size_*Eigen::Vector3f(size_*1/2, size_*1/2, size_/2), 0.5f));
  generate_depth_image_ = generate_depth_image(depth_image_, spheres);

  int frames = FRAMES;
  for (int frame = 0; frame < frames; frame++) {
    Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f Rbc;
    Rbc << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    Eigen::Matrix3f Rwb = Eigen::Matrix3f::Identity();

    camera_pose.topLeftCorner<3,3>()  = Rwb*Rbc;

    camera_pose.topRightCorner<3,1>() = (Rwb*Eigen::Vector3f(-(size_/2 + frame*size_/8), 0, size_/2) + Eigen::Vector3f(size_/2, size_/2, 0))*voxel_size_;

    camera_parameter_.setPose(camera_pose);
    generate_depth_image_(camera_parameter_);
    active_list_ = buildActiveList(oct_, camera_parameter_, voxel_size_);
    foreach(voxel_size_, active_list_, camera_parameter_, depth_image_);
    std::stringstream f;

    f << "./out/scale_"  + std::to_string(SCALE) + "-sphere-linear_back_move-" + std::to_string(frame) + ".vtk";

    save3DSlice(oct_,
                Eigen::Vector3i(0, 0, oct_.size()/2),
                Eigen::Vector3i(oct_.size(), oct_.size(), oct_.size()/2 + 1),
                [](const auto& val) { return val.x; }, f.str().c_str());
  }

  for (std::vector<obstacle*>::iterator sphere = spheres.begin(); sphere != spheres.end(); ++sphere) {
    free(*sphere);
  }
  free(depth_image_);

}

TEST_F(MultiscaleTSDFMovingCameraTest, SphereRotation) {
  std::vector<obstacle*> spheres;

  // Allocate spheres in world frame
  sphere_obstacle* sphere_close = new sphere_obstacle(voxel_size_*Eigen::Vector3f(size_*1/8, size_*2/3, size_/2), 0.3f);
  sphere_obstacle* sphere_far   = new sphere_obstacle(voxel_size_*Eigen::Vector3f(size_*7/8, size_*1/3, size_/2), 0.3f);
  spheres.push_back(sphere_close);
  spheres.push_back(sphere_far);

  generate_depth_image_ = generate_depth_image(depth_image_, spheres);

  int frames = FRAMES;
  for (int frame = 0; frame < frames; frame++) {
    Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();

    Eigen::Matrix3f Rbc;
    Rbc << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    float angle = float(frame)/float(frames) * 2 * M_PI / 4 - 2 * M_PI / 8;
    Eigen::Matrix3f Rwb;
    Rwb <<  std::cos(angle), -std::sin(angle), 0,
        std::sin(angle),  std::cos(angle), 0,
        0,                0, 1;

    camera_pose.topLeftCorner<3,3>()  = Rwb*Rbc;

    camera_pose.topRightCorner<3,1>() = (Rwb*Eigen::Vector3f(-(size_/2 + 16*size_/8), 0, size_/2) + Eigen::Vector3f(size_/2, size_/2, 0))*voxel_size_;

    camera_parameter_.setPose(camera_pose);
    generate_depth_image_(camera_parameter_);
    active_list_ = buildActiveList(oct_, camera_parameter_, voxel_size_);
    foreach(voxel_size_, active_list_, camera_parameter_, depth_image_);
    std::stringstream f;

    f << "./out/scale_"  + std::to_string(SCALE) + "-sphere-rotational_move-" + std::to_string(frame) + ".vtk";

    save3DSlice(oct_,
                Eigen::Vector3i(0, 0, oct_.size()/2),
                Eigen::Vector3i(oct_.size(), oct_.size(), oct_.size()/2 + 1),
                [](const auto& val) { return val.x; }, f.str().c_str());
  }

  for (std::vector<obstacle*>::iterator sphere = spheres.begin(); sphere != spheres.end(); ++sphere) {
    free(*sphere);
  }
  free(depth_image_);

}

TEST_F(MultiscaleTSDFMovingCameraTest, BoxTranslation) {
  std::vector<obstacle*> boxes;

  // Allocate boxes in world frame
  boxes.push_back(new box_obstacle(voxel_size_*Eigen::Vector3f(size_*1/2, size_*1/4, size_/2), voxel_size_*Eigen::Vector3f(size_*1/4, size_*1/4, size_/4)));
  boxes.push_back(new box_obstacle(voxel_size_*Eigen::Vector3f(size_*1/2, size_*3/4, size_/2), voxel_size_*Eigen::Vector3f(size_*1/4, size_*1/4, size_/4)));
  generate_depth_image_ = generate_depth_image(depth_image_, boxes);

  int frames = FRAMES;
  for (int frame = 0; frame < frames; frame++) {
    Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f Rbc;
    Rbc << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    Eigen::Matrix3f Rwb = Eigen::Matrix3f::Identity();

    camera_pose.topLeftCorner<3,3>()  = Rwb*Rbc;

    camera_pose.topRightCorner<3,1>() = (Rwb*Eigen::Vector3f(-(size_/2 + frame*size_/8), 0, size_/2) + Eigen::Vector3f(size_/2, size_/2, 0))*voxel_size_;

    camera_parameter_.setPose(camera_pose);
    generate_depth_image_(camera_parameter_);
    active_list_ = buildActiveList(oct_, camera_parameter_, voxel_size_);
    foreach(voxel_size_, active_list_, camera_parameter_, depth_image_);
    std::stringstream f;

    f << "./out/scale_"  + std::to_string(SCALE) + "-box-linear_back_move-" + std::to_string(frame) + ".vtk";

    save3DSlice(oct_,
                Eigen::Vector3i(0, 0, oct_.size()/2),
                Eigen::Vector3i(oct_.size(), oct_.size(), oct_.size()/2 + 1),
                [](const auto& val) { return val.x; }, f.str().c_str());
  }

  for (std::vector<obstacle*>::iterator box = boxes.begin(); box != boxes.end(); ++box) {
    free(*box);
  }
  free(depth_image_);

}

TEST_F(MultiscaleTSDFMovingCameraTest, SphereBoxTranslation) {
  std::vector<obstacle*> obstacles;

  // Allocate boxes in world frame
  obstacles.push_back(new box_obstacle(voxel_size_*Eigen::Vector3f(size_*1/2, size_*1/4, size_/2), voxel_size_*Eigen::Vector3f(size_*1/4, size_*1/4, size_/4)));
  obstacles.push_back(new sphere_obstacle(voxel_size_*Eigen::Vector3f(size_*1/2, size_*1/2, size_/2), 0.5f));
  generate_depth_image_ = generate_depth_image(depth_image_, obstacles);

  int frames = FRAMES;
  for (int frame = 0; frame < frames; frame++) {
    Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f Rbc;
    Rbc << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    Eigen::Matrix3f Rwb = Eigen::Matrix3f::Identity();

    camera_pose.topLeftCorner<3,3>()  = Rwb*Rbc;

    camera_pose.topRightCorner<3,1>() = (Rwb*Eigen::Vector3f(-(size_/2 + frame*size_/8), 0, size_/2) + Eigen::Vector3f(size_/2, size_/2, 0))*voxel_size_;

    camera_parameter_.setPose(camera_pose);
    generate_depth_image_(camera_parameter_);
    active_list_ = buildActiveList(oct_, camera_parameter_, voxel_size_);
    foreach(voxel_size_, active_list_, camera_parameter_, depth_image_);
    std::stringstream f;

    f << "./out/scale_"  + std::to_string(SCALE) + "-sphere-and-box-linear_back_move-" + std::to_string(frame) + ".vtk";

    save3DSlice(oct_,
                Eigen::Vector3i(0, 0, oct_.size()/2),
                Eigen::Vector3i(oct_.size(), oct_.size(), oct_.size()/2 + 1),
                [](const auto& val) { return val.x; }, f.str().c_str());
  }

  for (std::vector<obstacle*>::iterator obstacle = obstacles.begin(); obstacle != obstacles.end(); ++obstacle) {
    free(*obstacle);
  }
  free(depth_image_);
}
