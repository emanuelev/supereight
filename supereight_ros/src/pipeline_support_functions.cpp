//
// Created by anna on 11/04/19.
//

#include "supereight_ros/pipeline_support_functions.hpp"

namespace se {
double calculateAlpha(int64_t pre_time_stamp,
                      int64_t post_time_stamp,
                      int64_t img_time_stamp) {
  double alpha = (double) (img_time_stamp - pre_time_stamp)
      / (post_time_stamp - pre_time_stamp);
  return alpha;
}

Eigen::Vector3d interpolateVector(const Eigen::Vector3d &pre_vector3D,
                                  const Eigen::Vector3d &post_vector3D,
                                  double alpha) {
  return pre_vector3D + alpha * (post_vector3D - pre_vector3D);
}

Eigen::Quaterniond interpolateOrientation(const Eigen::Quaterniond
                                          &pre_orientation,
                                          const Eigen::Quaterniond &post_orientation,
                                          double
                                          alpha) {
  Eigen::Quaterniond
      int_orientation = pre_orientation.slerp(alpha, post_orientation);
  return int_orientation;
}

Eigen::Matrix4d interpolatePose(
    const geometry_msgs::TransformStamped &pre_transformation,
    const geometry_msgs::TransformStamped &post_transformation,
    int64_t img_time_stamp) {

  double alpha =
      calculateAlpha(ros::Time(pre_transformation.header.stamp).toNSec(),
                     ros::Time(post_transformation.header.stamp).toNSec(),
                     img_time_stamp);

  Eigen::Vector3d pre_translation(pre_transformation.transform.translation.x,
                                  pre_transformation.transform.translation.y,
                                  pre_transformation.transform.translation.z);
  Eigen::Vector3d post_translation(post_transformation.transform.translation.x,
                                   post_transformation.transform.translation.y,
                                   post_transformation.transform.translation.z);

  Eigen::Quaterniond pre_rotation(pre_transformation.transform.rotation.w,
                                  pre_transformation.transform.rotation.x,
                                  pre_transformation.transform.rotation.y,
                                  pre_transformation.transform.rotation.z);
  Eigen::Quaterniond post_rotation(post_transformation.transform.rotation.w,
                                   post_transformation.transform.rotation.x,
                                   post_transformation.transform.rotation.y,
                                   post_transformation.transform.rotation.z);

  Eigen::Vector3d inter_translation =
      interpolateVector(pre_translation, post_translation, alpha);
  Eigen::Quaterniond inter_rotation =
      interpolateOrientation(pre_rotation, post_rotation, alpha);

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 1>(0, 3) = inter_translation;
  pose.block<3, 3>(0, 0) = inter_rotation.toRotationMatrix();

  return pose;
}

/* Taken from https://github.com/ethz-asl/volumetric_mapping */
std_msgs::ColorRGBA percentToColor(double h)  {
  /* Helen's note: direct copy from OctomapProvider. */
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  /* blend over HSV-values (more colors) */

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  /* if i is even */
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}
}//namespace se