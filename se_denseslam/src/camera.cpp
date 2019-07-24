/*
 * Copyright (C) 2019 Sotiris Papatheodorou
 */

#include <se/camera.hpp>


PinholeCamera::PinholeCamera()
  : PinholeCamera(640, 480, 525.f, 525.f, 319.5f, 239.5f, 0.4f, 4.f) {
}



PinholeCamera::PinholeCamera(const int   width,
                             const int   height,
                             const float fx,
                             const float fy,
                             const float cx,
                             const float cy,
                             const float near_plane,
                             const float far_plane)
    : width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        near_plane_(near_plane), far_plane_(far_plane),
        frustum_vertices_(num_frustum_vertices_, Eigen::Vector4f::Ones()),
        frustum_normals_(num_frustum_normals_, Eigen::Vector4f::Zero()) {
  computeFrustumVertices();
  computeFrustumNormals();
}



PinholeCamera::PinholeCamera(const int              width,
                             const int              height,
                             const Eigen::Vector4f& k,
                             const float            near_plane,
                             const float            far_plane)
    : PinholeCamera(width, height, k.x(), k.y(), k.z(), k.w(),
        near_plane, far_plane) {
}



bool PinholeCamera::projectPoint(const Eigen::Vector3f& point,
                                 Eigen::Vector2f&       point_px) const {
  // The point is behind or on the camera.
  if (point.z() <= 0.f) {
    return false;
  }

  // Project the point. Round coordinates to nearest integer.
  point_px.x() = fx_ * (point.x() / point.z()) + cx_;
  point_px.y() = fy_ * (point.y() / point.z()) + cy_;

  // Test if the point is inside the image.
  return pointInImage(point_px);
}



void PinholeCamera::backProjectPoint(const Eigen::Vector2f& point_px,
                                     Eigen::Vector3f&       point) const {
  point.x() = (point_px.x() - cx_) / fx_;
  point.y() = (point_px.y() - cy_) / fy_;
  point.z() = 1.f;
}



bool PinholeCamera::isVisible(const Eigen::Vector3f& point) const {
  Eigen::Vector2f tmp_image_point;
  return projectPoint(point, tmp_image_point);
}



bool PinholeCamera::pointInFrustum(const Eigen::Vector3f& point) const {
  for (size_t i = 0; i < num_frustum_normals_; ++i) {
    // Compute the signed distance between the point and the plane.
    const double distance = point.homogeneous().dot(frustum_normals_[i]);
    if (distance < 0) {
      // A negative distance means that the point is located on the opposite
      // halfspace than the one the plane normal is pointing towards.
      return false;
    }
  }
  return true;
}



bool PinholeCamera::pointInFrustumInf(const Eigen::Vector3f& point) const {
  for (size_t i = 0; i < num_frustum_normals_; ++i) {
    // Skip the far plane normal.
    if (i != 1) {
      // Compute the signed distance between the point and the plane.
      const double distance = point.homogeneous().dot(frustum_normals_[i]);
      if (distance < 0) {
        // A negative distance means that the point is located on the opposite
        // halfspace than the one the plane normal is pointing towards.
        return false;
      }
    }
  }
  return true;
}



bool PinholeCamera::sphereInFrustum(const Eigen::Vector3f& center,
                                    const float            radius) const {
  for (size_t i = 0; i < num_frustum_normals_; ++i) {
    // Compute the signed distance between the point and the plane.
    const double distance = center.homogeneous().dot(frustum_normals_[i]);
    if (distance < -radius) {
      // Instead of testing for negative distance as in
      // PinholeCamera::pointInFrustum, test for distance smaller than -radius
      // so that the test is essentially performed on the plane offset by
      // radius.
      return false;
    }
  }
  return true;
}



bool PinholeCamera::sphereInFrustumInf(const Eigen::Vector3f& center,
                                       const float            radius) const {
  for (size_t i = 0; i < num_frustum_normals_; ++i) {
    // Skip the far plane normal.
    if (i != 1) {
      // Compute the signed distance between the point and the plane.
      const double distance = center.homogeneous().dot(frustum_normals_[i]);
      if (distance < -radius) {
        // Instead of testing for negative distance as in
        // PinholeCamera::pointInFrustum, test for distance smaller than -radius
        // so that the test is essentially performed on the plane offset by
        // radius.
        return false;
      }
    }
  }
  return true;
}



bool PinholeCamera::pointInImage(const Eigen::Vector2f& point_px) const {
  if ((point_px.x() < 0.f)
      || (point_px.x() > width_)
      || (point_px.y() < 0.f)
      || (point_px.y() > height_)) {
    return false;
  } else {
    return true;
  }
}



void PinholeCamera::computeFrustumVertices() {
  Eigen::Vector3f tmp_vector;

  // Back-project the frame corners to get the frustum vertices.
  // Top left.
  backProjectPoint(Eigen::Vector2f(0.f, 0.f), tmp_vector);
  frustum_vertices_[0].head<3>() = tmp_vector;
  frustum_vertices_[4].head<3>() = tmp_vector;
  // Top right.
  backProjectPoint(Eigen::Vector2f(width_, 0.f), tmp_vector);
  frustum_vertices_[1].head<3>() = tmp_vector;
  frustum_vertices_[5].head<3>() = tmp_vector;
  // Bottom right.
  backProjectPoint(Eigen::Vector2f(width_, height_), tmp_vector);
  frustum_vertices_[2].head<3>() = tmp_vector;
  frustum_vertices_[6].head<3>() = tmp_vector;
  // Bottom left.
  backProjectPoint(Eigen::Vector2f(0.f, height_), tmp_vector);
  frustum_vertices_[3].head<3>() = tmp_vector;
  frustum_vertices_[7].head<3>() = tmp_vector;

  // Scale the frustum vertices with the appropriate depth for near and far
  // plane vertices.
  // Near plane vertices.
  for (size_t i = 0; i < num_frustum_vertices_ / 2; ++i) {
    frustum_vertices_[i].head<3>() *= near_plane_;
  }
  // Far plane vertices.
  for (size_t i = num_frustum_vertices_ / 2; i < num_frustum_vertices_; ++i) {
    frustum_vertices_[i].head<3>() *= far_plane_;
  }

  //printFrustumVertices();
}



void PinholeCamera::computeFrustumNormals() {
  // Near plane vector.
  frustum_normals_[0] = Eigen::Vector4f(0.f, 0.f, 1.f, -near_plane_);
  // Far plane vector.
  frustum_normals_[1] = Eigen::Vector4f(0.f, 0.f, -1.f, far_plane_);
  // Left plane vector.
  frustum_normals_[2].head<3>() = planeNormal(
      frustum_vertices_[4], frustum_vertices_[0], frustum_vertices_[3]);
  // Right plane vector.
  frustum_normals_[3].head<3>() = planeNormal(
      frustum_vertices_[1], frustum_vertices_[5], frustum_vertices_[6]);
  // Bottom plane vector.
  frustum_normals_[4].head<3>() = planeNormal(
      frustum_vertices_[7], frustum_vertices_[3], frustum_vertices_[2]);
  // Top plane vector.
  frustum_normals_[5].head<3>() = planeNormal(
      frustum_vertices_[5], frustum_vertices_[1], frustum_vertices_[0]);

  // The w vector component corresponds to the distance of the plane from the
  // origin. It should be 0 for all planes other than the near and far planes.
  // The vectors where initialized to 0 so nothing needs to be done.

  //printFrustumNormals();
}



void PinholeCamera::printFrustumVertices() const {
  for (const auto& vertex : frustum_vertices_) {
    std::cout << vertex.x() << " ";
  }
  std::cout << "\n";
  for (const auto& vertex : frustum_vertices_) {
    std::cout << vertex.y() << " ";
  }
  std::cout << "\n";
  for (const auto& vertex : frustum_vertices_) {
    std::cout << vertex.z() << " ";
  }
  std::cout << "\n";
}



void PinholeCamera::printFrustumNormals() const {
  for (const auto& normal : frustum_normals_) {
    std::cout << normal.x() << " ";
  }
  std::cout << "\n";
  for (const auto& normal : frustum_normals_) {
    std::cout << normal.y() << " ";
  }
  std::cout << "\n";
  for (const auto& normal : frustum_normals_) {
    std::cout << normal.z() << " ";
  }
  std::cout << "\n";
  for (const auto& normal : frustum_normals_) {
    std::cout << normal.w() << " ";
  }
  std::cout << "\n";
}



Eigen::Vector3f PinholeCamera::planeNormal(const Eigen::Vector4f& v1,
                                           const Eigen::Vector4f& v2,
                                           const Eigen::Vector4f& v3) const {
  // Temporary tangent vectors.
  const Eigen::Vector4f t1 = v2 - v1;
  const Eigen::Vector4f t2 = v3 - v2;

  // Compute cross product.
  const Eigen::Vector3f c = t1.head<3>().cross(t2.head<3>());

  // Return unit length vector.
  return c / c.norm();
}

