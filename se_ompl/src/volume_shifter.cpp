/**
 * Probabilistic Trajectory Planning Volume Shifter.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @todo LICENSE
 * 
 * @file file VolumeShifter.hpp
 * @author Nils Funk
 * @date June, 2018
 */

#include <se/ompl/volume_shifter.hpp>
#include <ctime>
#include <omp.h>

static inline bool cmpPts(const Eigen::Vector3i &lhs, const Eigen::Vector3i &rhs) {
  if (lhs.x() < rhs.x()) return true;
  if (rhs.x() < lhs.x()) return false;
  if (lhs.y() < rhs.y()) return true;
  if (rhs.y() < lhs.y()) return false;
  return lhs.z() < rhs.z();
}

namespace se {
namespace exploration {

VolumeShifter::VolumeShifter(float voxel_dim, int volume_precision_factor) {
  voxel_dim_ = voxel_dim;
  volume_precision_ = 1.f / float(volume_precision_factor);
};

VolumeShifter::VolumeShifter() {
  voxel_dim_ = 1;
  volume_precision_ = voxel_dim_ / 2;
};

/// Fills segment_flight_corridor vector with all voxels that are within a cylinder with a hole from start to end (height = norm(end-start)) and the given radia r_min and r_max
bool VolumeShifter::generateSegmentFlightCorridor(std::vector<Eigen::Vector3i> &segment_flight_corridor,
                                                  const Eigen::Vector3f &start,
                                                  const Eigen::Vector3f &end,
                                                  float r_min,
                                                  float r_max) {
  std::vector<Eigen::Vector3f> vol_pts;

  // Convert all inputs to voxel coordinates
  r_min = r_min / voxel_dim_;
  r_max = r_max / voxel_dim_;
  const Eigen::Vector3f start_v = start / voxel_dim_;
  const Eigen::Vector3f end_v = end / voxel_dim_;

  /// Define path axis and length
  Eigen::Vector3f path_seg_axis = end_v - start_v;
  float path_seg_length = path_seg_axis.norm();
  path_seg_axis.normalize();

  /// Generate points in axis alligned cylinder volume
  for (float z = 0; z < path_seg_length + volume_precision_; z = z + volume_precision_) {
    if (z > path_seg_length)
      z = path_seg_length;
    vol_pts.push_back(Eigen::Vector3f(r_max, 0, z));
    vol_pts.push_back(Eigen::Vector3f(-r_max, 0, z));
    for (float x = r_min; x < r_max; x = x + volume_precision_) {
      vol_pts.push_back(Eigen::Vector3f(x, 0, z));

      float y_max = (sqrt(pow(r_max, 2) - pow(x, 2)));
      if (y_max != 0) {
        vol_pts.push_back(Eigen::Vector3f(x, -y_max, z));
        vol_pts.push_back(Eigen::Vector3f(x, y_max, z));
      }

      if (x != 0) {
        vol_pts.push_back(Eigen::Vector3f(-x, y_max, z));
        vol_pts.push_back(Eigen::Vector3f(-x, -y_max, z));
        vol_pts.push_back(Eigen::Vector3f(-x, 0, z));
      }

      for (float y = volume_precision_; y < y_max; y = y + volume_precision_) {
        if (x == 0) {
          vol_pts.push_back(Eigen::Vector3f(0, y, z));
          vol_pts.push_back(Eigen::Vector3f(0, -y, z));
          continue;
        }
        vol_pts.push_back(Eigen::Vector3f(x, y, z));
        vol_pts.push_back(Eigen::Vector3f(-x, y, z));
        vol_pts.push_back(Eigen::Vector3f(x, -y, z));
        vol_pts.push_back(Eigen::Vector3f(-x, -y, z));
      }
    }
  }

  /// Compute rotation and translation and transform points
  Eigen::Translation<float, 3> translation(start_v);
  Eigen::Vector3f z_axis(0, 0, 1);
  float rot_angle = acos(z_axis.dot(path_seg_axis));

  if (0 <= rot_angle && rot_angle < (0.005 * M_PI)) {
    translatePts(segment_flight_corridor, vol_pts, translation);
  } else if ((0.995 * M_PI) < rot_angle && rot_angle <= M_PI) {
    Eigen::Vector3f rot_axis(0, 1, 0);
    Eigen::Transform<float, 3, Eigen::Affine>
        transformation = translation * Eigen::AngleAxisf(rot_angle, rot_axis);
    transformPts(segment_flight_corridor, vol_pts, transformation);
  } else {
    Eigen::Vector3f rot_axis = z_axis.cross(path_seg_axis) / (sin(rot_angle));
    Eigen::Transform<float, 3, Eigen::Affine>
        transformation = translation* Eigen::AngleAxisf(rot_angle, rot_axis);
    transformPts(segment_flight_corridor, vol_pts, transformation);
  }

  return true;
}

std::vector<Eigen::Vector3i> VolumeShifter::getSegmentFlightCorridor(const Eigen::Vector3f &start,
                                                                     const Eigen::Vector3f &end,
                                                                     float r_min,
                                                                     float r_max) {
  std::vector<Eigen::Vector3i> segment_flight_corridor;
  generateSegmentFlightCorridor(segment_flight_corridor, start, end, r_min, r_max);
  return segment_flight_corridor;
}

bool VolumeShifter::generateSphereVolume(std::vector<Eigen::Vector3i> &sphere_volume,
                                         const Eigen::Vector3f &position_m,
                                         const float radius_m) {

  std::vector<Eigen::Vector3f> vol_pts;
  Eigen::Vector3f position_v = position_m / voxel_dim_;
  float radius_v = radius_m / voxel_dim_;

  // Generate points in axis allined sphere volume
  for (float z = -radius_v; z <= radius_v; z = z + volume_precision_) {
    double x_max = std::sqrt(radius_v * radius_v - z * z);
    vol_pts.push_back(Eigen::Vector3f(x_max, 0, z));
    vol_pts.push_back(Eigen::Vector3f(-x_max, 0, z));
    for (float x = 0; x < x_max; x = x + volume_precision_) {
      vol_pts.push_back(Eigen::Vector3f(x, 0, z));

      float y_max = std::sqrt(radius_v * radius_v - x * x - z * z);
      if (y_max != 0) {
        vol_pts.push_back(Eigen::Vector3f(x, -y_max, z));
        vol_pts.push_back(Eigen::Vector3f(x, y_max, z));
      }

      if (x != 0) {
        vol_pts.push_back(Eigen::Vector3f(-x, y_max, z));
        vol_pts.push_back(Eigen::Vector3f(-x, -y_max, z));
        vol_pts.push_back(Eigen::Vector3f(-x, 0, z));
      }

      for (float y = volume_precision_; y < y_max; y = y + volume_precision_) {
        if (x == 0) {
          vol_pts.push_back(Eigen::Vector3f(0, y, z));
          vol_pts.push_back(Eigen::Vector3f(0, -y, z));
          continue;
        }
        vol_pts.push_back(Eigen::Vector3f(x, y, z));
        vol_pts.push_back(Eigen::Vector3f(-x, y, z));
        vol_pts.push_back(Eigen::Vector3f(x, -y, z));
        vol_pts.push_back(Eigen::Vector3f(-x, -y, z));
      }
    }
  }
  Eigen::Translation<float, 3> translation(position_v);
  translatePts(sphere_volume, vol_pts, translation);

  return true;
}

std::vector<Eigen::Vector3i> VolumeShifter::getSphereVolume(Eigen::Vector3f position,
                                                            double radius) {
  std::vector<Eigen::Vector3i> sphere_volume;
  generateSphereVolume(sphere_volume, position, (float) radius);
  return sphere_volume;
}

bool VolumeShifter::translatePts(std::vector<Eigen::Vector3i> &translated_pts,
                  std::vector<Eigen::Vector3f> pts,
                  const Eigen::Translation3f &translation) {
//    LOG(INFO) << "Translate segment corridor";
  for (auto it = pts.begin(); it != pts.end(); ++it) {
    Eigen::Vector3f translated_pt_d = translation * (*it);
    if (translated_pt_d.x() < 0) translated_pt_d.x() -= 1;
    if (translated_pt_d.y() < 0) translated_pt_d.y() -= 1;
    if (translated_pt_d.z() < 0) translated_pt_d.z() -= 1;
    Eigen::Vector3i translated_pt_i = translated_pt_d.cast<int>();
    translated_pts.push_back(translated_pt_i);
  }
  std::sort(translated_pts.begin(), translated_pts.end(), cmpPts);
  translated_pts.erase(unique(translated_pts.begin(), translated_pts.end()), translated_pts.end());
};

bool VolumeShifter::rotatePts(std::vector<Eigen::Vector3i> &rotated_pts,
                              std::vector<Eigen::Vector3f> pts,
                              const Eigen::AngleAxis<float> &rotation) {
  LOG(INFO) << "Rotate segment corridor";
  for (auto it = pts.begin(); it != pts.end(); ++it) {

    Eigen::Vector3f rotated_pt_d = rotation * (*it);
    if (rotated_pt_d.x() < 0) rotated_pt_d.x() -= 1;
    if (rotated_pt_d.y() < 0) rotated_pt_d.y() -= 1;
    if (rotated_pt_d.z() < 0) rotated_pt_d.z() -= 1;
    Eigen::Vector3i rotated_pt_i = rotated_pt_d.cast<int>();
    rotated_pts.push_back(rotated_pt_i);
  }
  std::sort(rotated_pts.begin(), rotated_pts.end(), cmpPts);
  rotated_pts.erase(unique(rotated_pts.begin(), rotated_pts.end()), rotated_pts.end());
};

bool VolumeShifter::transformPts(std::vector<Eigen::Vector3i> &transformed_pts,
                                 std::vector<Eigen::Vector3f> pts,
                                 const Eigen::Transform<float, 3, Eigen::Affine> &transformation) {
//    LOG(INFO) << "Transform segment corridor";
  //TODO: #pragma omp parallel for
  for (auto it = pts.begin(); it != pts.end(); ++it) {
    Eigen::Vector3f transformed_pt_d = transformation * *it;
    if (transformed_pt_d.x() < 0) transformed_pt_d.x() -= 1;
    if (transformed_pt_d.y() < 0) transformed_pt_d.y() -= 1;
    if (transformed_pt_d.z() < 0) transformed_pt_d.z() -= 1;
    Eigen::Vector3i transformed_pt_i = transformed_pt_d.cast<int>();
    //TODO: #pragma omp critical
    transformed_pts.push_back(transformed_pt_i);
  }
  std::sort(transformed_pts.begin(), transformed_pts.end(), cmpPts);
  transformed_pts.erase(unique(transformed_pts.begin(), transformed_pts.end()),
                        transformed_pts.end());
};
};

}