#include <se/octant_ops.hpp>
#include <se/octree.hpp>
#include <se/algorithms/balancing.hpp>
#include <se/functors/axis_aligned_functor.hpp>
#include <se/functors/for_each.hpp>
#include <se/io/vtk-io.h>
#include <se/io/ply_io.hpp>
#include "../../src/multires/mapping_impl.hpp"
#include <random>
#include <functional>
#include <gtest/gtest.h>

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

float sphere_dist(const Eigen::Vector3f& p, const Eigen::Vector3f& C, 
    const float radius) {
  const float dist = (p - C).norm();
  return dist - radius;
}

float sphere_dist_noisy(const Eigen::Vector3f& p, const Eigen::Vector3f& C, 
    const float radius) {
  static std::mt19937 gen{1}; 
  std::normal_distribution<> d(0, 2);
  const float dist = (p - C).norm();
  return (dist - radius) + d(gen);
}

template <typename T>
void update_block (se::VoxelBlock<T>* block, 
                    const Eigen::Vector3f& center,
                    const float radius,
                    const int scale) {       
  const Eigen::Vector3i base = block->coordinates();
  const int side = se::VoxelBlock<T>::side;
  const int stride = 1 << scale;
  for(int z = 0; z < side; z += stride)
    for(int y = 0; y < side; y += stride)
      for(int x = 0; x < side; x += stride) {
        const Eigen::Vector3i vox = base + Eigen::Vector3i(x, y, z);
        auto data = block->data(vox, scale);
        const float sample = sphere_dist_noisy(
            vox.cast<float>() + float(stride) * Eigen::Vector3f::Constant(0.5f), 
            center, radius);
        data.delta = (sample - data.x)/(data.y + 1);
        data.delta_y++;
        data.x = (data.x * data.y + sample)/(data.y + 1);
        data.y = data.y + 1;
        block->data(vox, scale, data);
      }
}

class MultiscaleTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      unsigned size = 256;
      float dim = 5.f;
      oct_.init(size, dim); // 5 meters

      center_ = size >> 1;
      radius_ = size >> 2;
      const Eigen::Vector3f C(center_, center_, center_);

      for(int z = center_ - radius_; z < (center_ + radius_); ++z) {
        for(int y = center_ - radius_; y < (center_ + radius_); ++y) {
          for(int x = center_ - radius_; x < (center_ + radius_); ++x) {
            const Eigen::Vector3i vox(x, y, z);
            const float dist = fabs(sphere_dist(vox.cast<float>(), C, radius_));
            if(dist > 20.f && dist < 25.f) {
              alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2)));
            }
          }
        }
      }
      oct_.allocate(alloc_list.data(), alloc_list.size());
    }

  typedef se::Octree<ESDF> OctreeT;
  OctreeT oct_;
  int center_;
  int radius_;
  std::vector<se::key_t> alloc_list;
};

TEST_F(MultiscaleTest, Fusion) {
  Eigen::Vector3f center = Eigen::Vector3f::Constant(center_);
  int scale = 0;
  float radius = this->radius_;
  auto update_op = [&center, &scale, radius](se::VoxelBlock<ESDF>* b) {
    update_block(b, center, radius, scale);
  };

  for(int i = 0; i < 5; ++i) {
    se::functor::internal::parallel_for_each(oct_.getBlockBuffer(), update_op);
    auto op = [](se::VoxelBlock<ESDF>* b) { se::multires::propagate_up(b, 0); };
    se::functor::internal::parallel_for_each(oct_.getBlockBuffer(), op);

    {
      std::stringstream f;
      f << "./out/sphere-interp-" << i << ".vtk";
      save3DSlice(oct_, Eigen::Vector3i(0, oct_.size()/2, 0),
          Eigen::Vector3i(oct_.size(), oct_.size()/2 + 1, oct_.size()), 
          [](const auto& val) { return val.x; }, f.str().c_str());
    }
  }
  
  // update captured lambda parameters
  center += Eigen::Vector3f::Constant(10.f);
  scale = 2;
  for(int i = 5; i < 10; ++i) {
    se::functor::internal::parallel_for_each(oct_.getBlockBuffer(), update_op);
    auto op = [&scale](se::VoxelBlock<ESDF>* b) { se::multires::propagate_down(b, scale); };
    se::functor::internal::parallel_for_each(oct_.getBlockBuffer(), op);

    {
      std::stringstream f;
      f << "./out/sphere-interp-" << i << ".vtk";
      save3DSlice(oct_, Eigen::Vector3i(0, oct_.size()/2, 0),
          Eigen::Vector3i(oct_.size(), oct_.size()/2 + 1, oct_.size()), 
          [](const auto& val) { return val.x; }, f.str().c_str());
    }
  }
  se::print_octree("./out/test-sphere.ply", oct_);
}
