/**
 * Probabilistic Trajectory Planning, Volume Shifter.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @file VolumeShifter.hpp
 *
 *
 * @author Nils Funk
 * @date July 11, 2018
 */

#ifndef VOLUMESHIFTER_HPP
#define VOLUMESHIFTER_HPP

#include <memory>
#include <iostream>
#include <glog/logging.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace se {
namespace exploration {
class VolumeShifter {

 public:
  typedef std::unique_ptr<VolumeShifter> Ptr;

  VolumeShifter(const float voxel_dim, const int volume_precision_factor);
  VolumeShifter();

  // Fills segment_flight_corridor vector with all voxels that are within a cylinder with a hole from start to end (height = norm(end-start)) and the given radia r_min and r_max
  bool generateSegmentFlightCorridor(std::vector<Eigen::Vector3i> &segment_flight_corridor,
                                     const Eigen::Vector3f &start,
                                     const Eigen::Vector3f &end,
                                      float r_min,
                                      float r_max);

  std::vector<Eigen::Vector3i> getSegmentFlightCorridor(const Eigen::Vector3f &start,
                                                        const Eigen::Vector3f &end,
                                                         float r_min,
                                                         float r_max);

  bool generateSphereVolume(std::vector<Eigen::Vector3i> &sphere_volume,
                            const Eigen::Vector3f & position,
                            const float radius);

  std::vector<Eigen::Vector3i> getSphereVolume(Eigen::Vector3f position, double radius);

  // Transforms a point vector by a given transformation
  bool transformPts(std::vector<Eigen::Vector3i> &transformed_pts,
                    std::vector<Eigen::Vector3f> pts,
                    const Eigen::Transform<float, 3, Eigen::Affine> &transformation);

  // Rotates a point vector by a given rotation
  bool rotatePts(std::vector<Eigen::Vector3i> &rotated_pts,
                 std::vector<Eigen::Vector3f> pts,
                 const Eigen::AngleAxis<float> &rotation);

  // Translates a point vector by a given translation
  bool translatePts(std::vector<Eigen::Vector3i> &translated_pts,
                    std::vector<Eigen::Vector3f> pts,
                    const Eigen::Translation3f &translation);

 private:
  float voxel_dim_;
  float volume_precision_;

  // plan in [voxel] and not in [m]
  bool plan_in_voxel = true;
};
} // namespace exploration
}
#endif //VOLUMESHIFTER_HPP
