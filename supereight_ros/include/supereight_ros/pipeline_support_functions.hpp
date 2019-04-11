//
// Created by anna on 11/04/19.
//

#ifndef SUPEREIGHT_PIPELINE_SUPPORT_FUNCTIONS_HPP
#define SUPEREIGHT_PIPELINE_SUPPORT_FUNCTIONS_HPP

#include <Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/ColorRGBA.h>
namespace se {

/**
 * @brief       Calculates the fraction a given sample is located between the closest pre and post sample
 *
 * @param[in]   pre_time_stamp    Closest pre vicon time stamp to image
 * @param[in]   post_time_stamp   Closest post vicon time stamp to image
 * @param[in]   img_time_stamp    Image time stamp
 *
 * @return      Fraction
 */
double calculateAlpha(int64_t pre_time_stamp,
                      int64_t post_time_stamp,
                      int64_t img_time_stamp);

/**
 * @brief      Linear 3D interpolation
 *
 * @param[in]  pre_vector3D  The pre vector 3D
 * @param[in]  post_vector3D      The post vector 3D
 * @param[in]  alpha              Fraction
 *
 * @return     Interpolated translation
 */
Eigen::Vector3d interpolateVector(const Eigen::Vector3d &pre_vector3D,
                                  const Eigen::Vector3d &post_vector3D,
                                  double alpha);

/**
 * @brief      Slerp interpolation for quaterniond
 *
 * @param[in]  pre_orientation       The previous orientation
 * @param[in]  post_orientation      The post orientation
 * @param[in]  alpha                 Fraction
 *
 * @return     Interpolated orientation
 */
Eigen::Quaterniond interpolateOrientation(const Eigen::Quaterniond
                                          &pre_orientation,
                                          const Eigen::Quaterniond &post_orientation,
                                          double alpha);

/**
 * @brief      Interpolation for transformations
 *
 * @param[in]  pre_transformation       The previous transformation
 * @param[in]  post_transformation      The post transformation
 * @param[in]  alpha                    Fraction
 *
 * @return     Interpolated transformation
 */
Eigen::Matrix4d interpolatePose(
    const geometry_msgs::TransformStamped &pre_transformation,
    const geometry_msgs::TransformStamped &post_transformation,
    int64_t img_time_stamp);

/* Taken from https://github.com/ethz-asl/volumetric_mapping */
std_msgs::ColorRGBA percentToColor(double h) ;
}// namespace se
#endif //SUPEREIGHT_PIPELINE_SUPPORT_FUNCTIONS_HPP
