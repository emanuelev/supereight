//
// Created by anna on 28/06/19.
//

#ifndef SUPEREIGHT_COLLISION_CHECKER_HPP
#define SUPEREIGHT_COLLISION_CHECKER_HPP
// Functions from Project : Probabilistic Trajectory Planning Volume Shifter
// by Nils FUnk

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include <cstdlib>
#include <cmath>
#include <se/utils/eigen_utils.h>

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
namespace se {
namespace exploration {

template<typename T>
class CollisionCheck {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionCheck(const Volume<T> &volume, Planning_Configuration planning_config, double res);
  void getSphereAroundPoint(const Eigen::Vector3i &center, float radius_m, VectorVec3i *voxel_list);
  bool checkVolume(const VectorVec3i &volume);

  bool isSphereCollisionFree(const Eigen::Vector3i pos_v);

 private:

  Volume<T> volume_;
  double res_;
  Planning_Configuration planning_config_;
};

template<typename T>
CollisionCheck<T>::CollisionCheck(const VolumeTemplate<T, se::Octree> &volume,
                                  Planning_Configuration planning_config,
                                  double res)
    :

    volume_(volume), planning_config_(planning_config), res_(res) {

}

// from voxblox utils planning_utils_inl.h

template<typename T>
bool CollisionCheck<T>::isSphereCollisionFree(const Eigen::Vector3i pos_v) {
  int radius_v = static_cast<int>(planning_config_.cand_view_safety_radius / res_);// m/(m/voxel)
  radius_v *=radius_v;
  for (int x = -radius_v; x <= radius_v; x++) {
    for (int y = -radius_v; y <= radius_v; y++) {
      for (int z = -radius_v; z <= radius_v; z++) {
        Eigen::Vector3i point_offset_v(x, y, z);
        //check if point is inside the sphere radius
        if (point_offset_v.squaredNorm() <= radius_v) {


          // check if voxelblock is allocated or only node
          Eigen::Vector3i point_v = point_offset_v + pos_v;
          // check if voxel exists
          if (volume_._map_index->fetch(point_v.x(), point_v.y(), point_v.z()) != NULL) {
            if (volume_._map_index->get(point_v).x > 0) {
              return false;
            }
            // TODO make it to octree
            /*          se::VoxelBlock<T>
              *block = volume_._map_index->fetch_octant(point_vox.x(), point_vox.y(), point_vox.z());
            // voxel block allocated
            if(block !=NULL){

              std::cout << "[se/collision_checker] get sphere point " << point_vox.format(InLine)
                        <<std::endl;*/
          } else {
//            std::cout << "[se/collision_check] sphere voxel not allocated" << std::endl;
            return false;
          }

        }
      }
    }
  }
//  std::cout << "[se/collision_checker] sphere radius " << planning_config_.cand_view_safety_radius
//            << " [m] =  " << radius_v << " voxels around center " << pos_v.format(InLine) << std::endl;
  return true;
}
template<typename T>
void CollisionCheck<T>::getSphereAroundPoint(const Eigen::Vector3i &center,
                                             float radius_m,
                                             VectorVec3i *voxel_list) {
  const float radius_in_voxels = radius_m / res_; // m / (m/ voxel)
  std::cout << "[se/collision_checker] sphere radius " << radius_m << " [m] =  " << radius_in_voxels
            << " voxels" << " around center " << center.format(InLine) << std::endl;
  for (float x = -radius_in_voxels; x <= radius_in_voxels; x++) {
    for (float y = -radius_in_voxels; y <= radius_in_voxels; y++) {
      for (float z = -radius_in_voxels; z <= radius_in_voxels; z++) {
        Eigen::Vector3f point_exact(x, y, z);
        //check if point is inside the sphere radius
        if (point_exact.norm() <= radius_in_voxels) {
          Eigen::Vector3i point_offset_vox(std::floor(point_exact.x()),
                                           std::floor(point_exact.y()),
                                           std::floor(point_exact.z()));
          // check if voxelblock is allocated or only node
          Eigen::Vector3i point_vox = point_offset_vox + center;
          // check if voxel exists
          if (volume_._map_index->fetch(point_vox.x(), point_vox.y(), point_vox.z()) != NULL) {
            // TODO make it to octree
            /*          se::VoxelBlock<T>
              *block = volume_._map_index->fetch_octant(point_vox.x(), point_vox.y(), point_vox.z());
            // voxel block allocated
            if(block !=NULL){

              std::cout << "[se/collision_checker] get sphere point " << point_vox.format(InLine)
                        <<std::endl;*/
            (*voxel_list).push_back(point_vox);
          } else {
            std::cout << "sphere voxel not allocated" << std::endl;
          }

        }
      }
    }
  }
}
} // namespace exploration

} // namespace se

#endif //SUPEREIGHT_COLLISION_CHECKER_HPP
