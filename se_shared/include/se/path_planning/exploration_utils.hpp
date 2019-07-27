//
// Created by anna on 11/07/19.
//

#ifndef SUPEREIGHT_EXPLORATION_UTILS_HPP
#define SUPEREIGHT_EXPLORATION_UTILS_HPP

#include <set>
#include <map>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <iterator>
#include <type_traits>

#include <Eigen/StdVector>

#include "lodepng.h"

namespace se {
namespace exploration {
struct pose3D {
  Eigen::Vector3f p;
  Eigen::Quaternionf q;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  pose3D() {
    p = {0.0, 0.0, 0.0};
    q = {1.0, 0.0, 0.0, 0.0};
  }
  pose3D(Eigen::Vector3f point, Eigen::Quaternionf quat) : p(point), q(quat) {}
};

static inline std::ostream &operator<<(std::ostream &os, const pose3D &pose) {
  return os << pose.p.x() << pose.p.y() << pose.p.z() << pose.q.x() << pose.q.y() << pose.q.z()
            << pose.q.z();
}
static inline std::istream &operator>>(std::istream &input, pose3D &pose) {
  input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y() >> pose.q.z()
        >> pose.q.w();
  // Normalize the quaternion to account for precision loss due to
  // serialization.
  pose.q.normalize();
  return input;
}

typedef std::vector<pose3D, Eigen::aligned_allocator<pose3D> > posevector;

typedef AlignedVector<std::pair<pose3D, double>> VectorPairPoseDouble;

struct eulerAngles {
  float yaw, pitch, roll;
};

// source https://cs.stanford.edu/~acoates/quaternion.h
// TODO write tests
/**
 * @brief Euler Angles to Quaternion
 * @param yaw [rad]
 * @param pitch [rad]
 * @param roll  [rad]
 * @return quaternion
 */
static inline Eigen::Quaternionf toQuaternion(float yaw, float pitch, float roll)
// yaw (Z),
// pitch (Y),
// roll (X)
{

  // Abbreviations for the various angular functions
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  Eigen::Quaternionf q;
  q.w() = cy * cp * cr + sy * sp * sr;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = sy * cp * sr + cy * sp * cr;
  q.z() = sy * cp * cr - cy * sp * sr;

  return q;
}

static inline eulerAngles toEulerAngles(Eigen::Quaternionf q) {
  eulerAngles angles;

  // roll (x-axis rotation)
  float sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  float cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    angles.pitch = asin(sinp);

  // yaw (z-axis rotation)
  float siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  float cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  angles.yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}
/** @brief Returns an equivalent euler angle representation of
 * this quaternion.
 * @return Euler angles in roll-pitch-yaw order.
 */
static inline eulerAngles toEulerAngles2(Eigen::Quaternionf q) {
  eulerAngles euler;
  const static float PI_OVER_2 = M_PI * 0.5;
  const static float EPSILON = 1e-10;
  float sqw, sqx, sqy, sqz;

  // quick conversion to Euler angles to give tilt to user
  sqw = q.w() * q.w();
  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();

  euler.pitch = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
  if (PI_OVER_2 - fabs(euler.pitch) > EPSILON) {
    euler.yaw = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()), sqx - sqy - sqz + sqw);
    euler.roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), sqw - sqx - sqy + sqz);
  } else {
    // compute heading from local 'down' vector
    euler.yaw = atan2(2 * q.y() * q.z() - 2 * q.x() * q.w(), 2 * q.x() * q.z() + 2 * q.y() * q.w());
    euler.roll = 0.0;

    // If facing down, reverse yaw
    if (euler.pitch < 0)
      euler.yaw = M_PI - euler.yaw;
  }
  return euler;
}

/**
 * @brief calculate entropy
 * @param prob_log logarithmic occupancy probability
 * @return entropy
 */
static inline double getEntropy(float prob_log) {
  float prob = se::math::getProbFromLog(prob_log);
  double temp_entropy = 0.0f;
  if (prob == 0.0 || (1 - prob) == 0.0) {
    return 0.0;
  }
  temp_entropy = -prob * log2(prob) - (1 - prob) * log2(1 - prob);
  return temp_entropy;

}

static inline bool isSameBlock(Eigen::Vector3i voxel, Eigen::Vector3i face_voxel) {
  return (voxel.x() / BLOCK_SIDE) == (face_voxel.x() / BLOCK_SIDE)
      && (voxel.y() / BLOCK_SIDE) == (face_voxel.y() / BLOCK_SIDE)
      && (voxel.z() / BLOCK_SIDE) == (face_voxel.z() / BLOCK_SIDE);
}

static inline bool saveMatrixToDepthImage(const Eigen::MatrixXd matrix,
                                          const int cand_num,
                                          const bool is_depth) {

  const int w = matrix.cols();
  const int h = matrix.rows();
  const double max_val = matrix.maxCoeff();
  const double min_val = matrix.minCoeff();
  const float diff = (max_val - min_val);
  uint16_t *input_depth = (uint16_t *) malloc(matrix.size() * sizeof(uint16_t));
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      input_depth[u + v * w] =
          static_cast<uint16_t >(65535 * (1 - ((matrix(v, u) - min_val) / diff)));
    }
  }

  char filename[80];
  if (is_depth) {
    const std::string
        s = "/home/anna/Data/cand_views/cand_" + std::to_string(cand_num) + "_depth_img.png";
    std::strcpy(filename, s.c_str());
  } else {

    const std::string s = "/home/anna/Data/cand_views/cand_" + std::to_string(cand_num) + "_IG_img"
                                                                                          ".png";
    std::strcpy(filename, s.c_str());
  }
  lodepng_encode_file(filename, (unsigned char *) input_depth, w, h, LCT_GREY, 16);

  return true;
}

// transforms current pose from matrix4f to position and orientation (quaternion)
static pose3D getCurrPose(const Eigen::Matrix4f &pose, const float res){
  pose3D curr_pose;
  curr_pose.q = se::math::rot_mat_2_quat(pose.block<3, 3>(0, 0));
  curr_pose.p = pose.block<3, 1>(0, 3) / res;
  return curr_pose;
}


} //exploration
}//namespace se
#endif //SUPEREIGHT_EXPLORATION_UTILS_HPP
