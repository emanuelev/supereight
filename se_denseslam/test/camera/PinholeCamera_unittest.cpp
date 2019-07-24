/*
 * Copyright (C) 2019 Sotiris Papatheodorou
 */

// On Eigen before 3.3.6 GCC shows this warning:
// warning: argument 1 value ‘X’ exceeds maximum object size Y [-Walloc-size-larger-than=]

// TODO
// - find out why there is a small difference in project.
// - test pointInFrustumInf and sphereInFrustumInf

#include <cctype>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "gtest/gtest.h"

#include "str_utils.h"
#include "camera.hpp"


// Tolerance in pixels when comparing projected point coordinates.
#define TOLERANCE_PX 0.00015
// Tolerance in meters when comparing backprojected point coordinates.
#define TOLERANCE_M 0.000001


typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >
    Vector3fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
    Vector2fVector;



class PinholeCameraTest : public ::testing::Test {
 protected:
  int readTestResults(const std::string&    filename,
                      PinholeCamera&        camera,
                      Vector3fVector&       points_c,
                      Vector2fVector&       points_px,
                      std::vector<uint8_t>& valid_projection,
                      std::vector<uint8_t>& point_in_frustum,
                      std::vector<float>&   radii,
                      std::vector<uint8_t>& sphere_in_frustum) {
    // Open file for reading.
    std::ifstream infile (filename.c_str(), std::ifstream::in);

    // Read file line by line.
    std::string line;
    while (true) {
      std::getline(infile, line);

      // EOF reached
      if (not infile.good()) {
        return 1;
      }

      // Ignore comment lines
      if ((line[0] == '#') and isalpha(line[2])){
        continue;
      }

      // Read camera parameters.
      if ((line[0] == '#') and not isalpha(line[2])){
        const std::vector<std::string> camera_data = splitString(line, ' ', true);
        const size_t num_camera_cols = camera_data.size();
        constexpr size_t desired_camera_cols = 9;
        if (num_camera_cols != desired_camera_cols) {
          std::cerr << "Invalid test point file format. Expected "
              << desired_camera_cols << " columns: "
              << "width(px) height(px) fx(px) fy(px) cx(px) cy(px) near_plane(m) far_plane(m)"
              << "\n but got " << num_camera_cols << ": " << line << std::endl;
          return 1;
        }
        const float width      = std::stoi(camera_data[1]);
        const float height     = std::stoi(camera_data[2]);
        const float fx         = std::stof(camera_data[3]);
        const float fy         = std::stof(camera_data[4]);
        const float cx         = std::stof(camera_data[5]);
        const float cy         = std::stof(camera_data[6]);
        const float near_plane = std::stof(camera_data[7]);
        const float far_plane  = std::stof(camera_data[8]);
        camera = PinholeCamera(width, height, fx, fy, cx, cy, near_plane, far_plane);
        continue;
      }

      // Data line read, split on spaces
      const std::vector<std::string> line_data = splitString(line, ' ', true);
      const size_t num_cols = line_data.size();
      constexpr size_t required_cols = 9;
      if (num_cols < required_cols) {
        std::cerr << "Invalid test point file format. Expected "
            << required_cols << " columns: "
            << "x_camera(m) y_camera(m) z_camera(m) x_image(px) y_image(px) valid_projection point_in_frustum sphere_radius(m) sphere_in_frustum ..."
            << "\n but got " << num_cols << ": " << line << std::endl;
        return 1;
      }

      // Read the point coordinates in the camera frame.
      const Eigen::Vector3f tmp_point_c (std::stof(line_data[0]),
          std::stof(line_data[1]), std::stof(line_data[2]));
      points_c.push_back(tmp_point_c);

      // Read the point coordinates in the image frame.
      const Eigen::Vector2f tmp_point_px (std::stof(line_data[3]),
          std::stof(line_data[4]));
      points_px.push_back(tmp_point_px);

      // Read the point projection result.
      valid_projection.push_back(std::stoi(line_data[5]));

      // Read the point in frustum test result.
      point_in_frustum.push_back(std::stoi(line_data[6]));

      // Read the sphere radius.
      radii.push_back(std::stof(line_data[7]));

      // Read the sphere in frustum test result.
      sphere_in_frustum.push_back(std::stoi(line_data[8]));
    }

    return 0;
  }



  void SetUp() override {
    const std::string test_data_directory = TEST_DATA_DIR;

    // Read the test points for the HD camera.
    readTestResults(test_data_directory + "/camera_hd.txt", camera_hd,
        points_c_hd, points_px_hd, valid_projection_hd, point_in_frustum_hd,
        radii_hd, sphere_in_frustum_hd);

    // Read the test points for the ROS camera.
    readTestResults(test_data_directory + "/camera_ros.txt", camera_ros,
        points_c_ros, points_px_ros, valid_projection_ros, point_in_frustum_ros,
        radii_ros, sphere_in_frustum_ros);
  }



  // HD camera.
  const int width_hd        = 1280;
  const int height_hd       = 720;
  const float fx_hd         = 1373.f;
  const float fy_hd         = 1373.f;
  const float cx_hd         = 640.f;
  const float cy_hd         = 360.f;
  const float near_plane_hd = 0.4f;
  const float far_plane_hd  = 4.f;
  PinholeCamera        camera_hd;
  Vector3fVector       points_c_hd;
  Vector2fVector       points_px_hd;
  std::vector<float>   radii_hd;
  std::vector<uint8_t> valid_projection_hd;
  std::vector<uint8_t> point_in_frustum_hd;
  std::vector<uint8_t> sphere_in_frustum_hd;

  // ROS camera.
  const int width_ros        = 640;
  const int height_ros       = 480;
  const float fx_ros         = 525.f;
  const float fy_ros         = 525.f;
  const float cx_ros         = 319.5f;
  const float cy_ros         = 239.5f;
  const float near_plane_ros = 0.4f;
  const float far_plane_ros  = 4.f;
  PinholeCamera        camera_ros;
  Vector3fVector       points_c_ros;
  Vector2fVector       points_px_ros;
  std::vector<float>   radii_ros;
  std::vector<uint8_t> valid_projection_ros;
  std::vector<uint8_t> point_in_frustum_ros;
  std::vector<uint8_t> sphere_in_frustum_ros;
};



TEST_F(PinholeCameraTest, Constructor) {
  const Eigen::Vector2f center_hd = camera_hd.center();
  EXPECT_EQ(camera_hd.width(),           width_hd);
  EXPECT_EQ(camera_hd.height(),          height_hd);
  EXPECT_EQ(camera_hd.size(),            width_hd * height_hd);
  EXPECT_FLOAT_EQ(camera_hd.nearPlane(), near_plane_hd);
  EXPECT_FLOAT_EQ(camera_hd.farPlane(),  far_plane_hd);
  EXPECT_FLOAT_EQ(center_hd.x(),         cx_hd);
  EXPECT_FLOAT_EQ(center_hd.y(),         cy_hd);

  const Eigen::Vector2f center_ros = camera_ros.center();
  EXPECT_EQ(camera_ros.width(),           width_ros);
  EXPECT_EQ(camera_ros.height(),          height_ros);
  EXPECT_EQ(camera_ros.size(),            width_ros * height_ros);
  EXPECT_FLOAT_EQ(camera_ros.nearPlane(), near_plane_hd);
  EXPECT_FLOAT_EQ(camera_ros.farPlane(),  far_plane_hd);
  EXPECT_FLOAT_EQ(center_ros.x(),         cx_ros);
  EXPECT_FLOAT_EQ(center_ros.y(),         cy_ros);
}



TEST_F(PinholeCameraTest, ProjectPoint) {
  // HD camera.
  for (size_t i = 0; i < points_c_hd.size(); ++i) {
    // Test the projection status.
    Eigen::Vector2f point_px;
    const bool status = camera_hd.projectPoint(points_c_hd[i], point_px);
    ASSERT_EQ(status, valid_projection_hd[i]);

    // Test the projected point.
    if (status) {
      EXPECT_NEAR(points_px_hd[i].x(), point_px.x(), TOLERANCE_PX);
      EXPECT_NEAR(points_px_hd[i].y(), point_px.y(), TOLERANCE_PX);
    }
  }

  // ROS camera.
  for (size_t i = 0; i < points_c_ros.size(); ++i) {
    // Test the projection status.
    Eigen::Vector2f point_px;
    const bool status = camera_ros.projectPoint(points_c_ros[i], point_px);
    ASSERT_EQ(status, valid_projection_ros[i]);

    // Test the projected point.
    if (status) {
      EXPECT_NEAR(points_px_ros[i].x(), point_px.x(), TOLERANCE_PX);
      EXPECT_NEAR(points_px_ros[i].y(), point_px.y(), TOLERANCE_PX);
    }
  }
}



TEST_F(PinholeCameraTest, BackprojectPoint) {
  // HD camera.
  for (size_t i = 0; i < points_px_hd.size(); ++i) {
    Eigen::Vector3f point_c;
    camera_hd.backProjectPoint(points_px_hd[i], point_c);

    // Multiply with known z.
    point_c *= points_c_hd[i].z();

    // Test the projected point.
    EXPECT_NEAR(points_c_hd[i].x(), point_c.x(), TOLERANCE_M);
    EXPECT_NEAR(points_c_hd[i].y(), point_c.y(), TOLERANCE_M);
    EXPECT_NEAR(points_c_hd[i].z(), point_c.z(), TOLERANCE_M);
  }

  // ROS camera.
  for (size_t i = 0; i < points_px_ros.size(); ++i) {
    Eigen::Vector3f point_c;
    camera_ros.backProjectPoint(points_px_ros[i], point_c);

    // Multiply with known z.
    point_c *= points_c_ros[i].z();

    // Test the projected point.
    EXPECT_NEAR(points_c_ros[i].x(), point_c.x(), TOLERANCE_M);
    EXPECT_NEAR(points_c_ros[i].y(), point_c.y(), TOLERANCE_M);
    EXPECT_NEAR(points_c_ros[i].z(), point_c.z(), TOLERANCE_M);
  }
}



TEST_F(PinholeCameraTest, IsVisible) {
  // HD camera.
  for (size_t i = 0; i < points_c_hd.size(); ++i) {
    // Test the projection status.
    const bool visible = camera_hd.isVisible(points_c_hd[i]);
    EXPECT_EQ(visible, valid_projection_hd[i]);
  }

  // ROS camera.
  for (size_t i = 0; i < points_c_ros.size(); ++i) {
    // Test the projection visible.
    const bool visible = camera_ros.isVisible(points_c_ros[i]);
    EXPECT_EQ(visible, valid_projection_ros[i]);
  }
}



TEST_F(PinholeCameraTest, PointInFrustum) {
  // HD camera.
  for (size_t i = 0; i < points_c_hd.size(); ++i) {
    // Test the projection in.
    const bool in = camera_hd.pointInFrustum(points_c_hd[i]);
    EXPECT_EQ(in, point_in_frustum_hd[i]);
  }

  // ROS camera.
  for (size_t i = 0; i < points_c_ros.size(); ++i) {
    // Test the projection in.
    const bool in = camera_ros.pointInFrustum(points_c_ros[i]);
    EXPECT_EQ(in, point_in_frustum_ros[i]);
  }
}



TEST_F(PinholeCameraTest, SphereInFrustum) {
  // HD camera.
  for (size_t i = 0; i < points_c_hd.size(); ++i) {
    // Test the projection in.
    const bool in = camera_hd.sphereInFrustum(points_c_hd[i], radii_hd[i]);
    EXPECT_EQ(in, sphere_in_frustum_hd[i]);
  }

  // ROS camera.
  for (size_t i = 0; i < points_c_ros.size(); ++i) {
    // Test the projection in.
    const bool in = camera_ros.sphereInFrustum(points_c_ros[i], radii_ros[i]);
    EXPECT_EQ(in, sphere_in_frustum_ros[i]);
  }
}



TEST_F(PinholeCameraTest, Project_BackProject_Point) {
  // HD camera.
  for (size_t i = 0; i < points_c_hd.size(); ++i) {
    // Project.
    Eigen::Vector2f point_px;
    const bool status = camera_hd.projectPoint(points_c_hd[i], point_px);
    ASSERT_EQ(status, valid_projection_hd[i]);

    // Back-project.
    Eigen::Vector3f point_c;
    camera_hd.backProjectPoint(point_px, point_c);

    // Multiply with known z.
    point_c *= points_c_hd[i].z();

    // Compare original and backprojected point.
    EXPECT_NEAR(points_c_hd[i].x(), point_c.x(), TOLERANCE_M);
    EXPECT_NEAR(points_c_hd[i].y(), point_c.y(), TOLERANCE_M);
    EXPECT_NEAR(points_c_hd[i].z(), point_c.z(), TOLERANCE_M);
  }

  // ROS camera.
  for (size_t i = 0; i < points_c_ros.size(); ++i) {
    // Project.
    Eigen::Vector2f point_px;
    const bool status = camera_ros.projectPoint(points_c_ros[i], point_px);
    ASSERT_EQ(status, valid_projection_ros[i]);

    // Back-project.
    Eigen::Vector3f point_c;
    camera_ros.backProjectPoint(point_px, point_c);

    // Multiply with known z.
    point_c *= points_c_ros[i].z();

    // Compare original and backprojected point.
    EXPECT_NEAR(points_c_ros[i].x(), point_c.x(), TOLERANCE_M);
    EXPECT_NEAR(points_c_ros[i].y(), point_c.y(), TOLERANCE_M);
    EXPECT_NEAR(points_c_ros[i].z(), point_c.z(), TOLERANCE_M);
  }
}



TEST_F(PinholeCameraTest, BackProject_Project_Point) {
  // HD camera.
  for (size_t i = 0; i < points_px_hd.size(); ++i) {
    // Back-project.
    Eigen::Vector3f point_c;
    camera_hd.backProjectPoint(points_px_hd[i], point_c);

    // Project.
    Eigen::Vector2f point_px;
    const bool status = camera_hd.projectPoint(point_c, point_px);
    ASSERT_EQ(status, valid_projection_hd[i]);

    // Compare original and projected point.
    if (status) {
      EXPECT_NEAR(points_px_hd[i].x(), point_px.x(), TOLERANCE_PX);
      EXPECT_NEAR(points_px_hd[i].y(), point_px.y(), TOLERANCE_PX);
    }
  }

  // ROS camera.
  for (size_t i = 0; i < points_px_ros.size(); ++i) {
    // Back-project.
    Eigen::Vector3f point_c;
    camera_ros.backProjectPoint(points_px_ros[i], point_c);

    // Project.
    Eigen::Vector2f point_px;
    const bool status = camera_ros.projectPoint(point_c, point_px);
    ASSERT_EQ(status, valid_projection_ros[i]);

    // Compare original and projected point.
    if (status) {
      EXPECT_NEAR(points_px_ros[i].x(), point_px.x(), TOLERANCE_PX);
      EXPECT_NEAR(points_px_ros[i].y(), point_px.y(), TOLERANCE_PX);
    }
  }
}

