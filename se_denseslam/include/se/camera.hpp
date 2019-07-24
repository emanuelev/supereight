/*
 * Copyright (C) 2019 Sotiris Papatheodorou
 */

#ifndef _CAMERA_HPP
#define _CAMERA_HPP

#include <cstdint>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>


constexpr uint8_t num_frustum_vertices_ = 8;
constexpr uint8_t num_frustum_normals_ = 6;

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >
    Vector4fVector;

class PinholeCamera {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int width_;
    int height_;
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float near_plane_;
    float far_plane_;
    PinholeCamera();

    PinholeCamera(const int   width,
                  const int   height,
                  const float fx,
                  const float fy,
                  const float cx,
                  const float cy,
                  const float near_plane,
                  const float far_plane);

    PinholeCamera(const int              width,
                  const int              height,
                  const Eigen::Vector4f& k,
                  const float            near_plane,
                  const float            far_plane);

    inline int width() const {
      return width_;
    }

    inline int height() const {
      return height_;
    }

    inline size_t size() const {
      return width_ * height_;
    }

    inline float nearPlane() const {
      return near_plane_;
    }

    inline float farPlane() const {
      return far_plane_;
    }

    inline Eigen::Vector2f center() const {
      return Eigen::Vector2f(cx_, cy_);
    }
    /**
     * Project a 3D point in camera coordinates to the image plane. Returns
     * true on successful projection.
     */
    bool projectPoint(const Eigen::Vector3f& point,
                      Eigen::Vector2f&       point_px) const;

    /**
     * Back-project from the image plane to a 3D point in camera coordinates.
     */
    void backProjectPoint(const Eigen::Vector2f& point_px,
                          Eigen::Vector3f&       point) const;

    /**
     * Test whether a 3D point in camera coordinates is visible by projecting
     * it on the image plane.
     */
    bool isVisible(const Eigen::Vector3f& point) const;

    /**
     * Test whether a 3D point in camera coordinates is visible by testing if
     * it is inside the camera frustum.
     */
    bool pointInFrustum(const Eigen::Vector3f& point) const;

    /**
     * Test whether a 3D point in camera coordinates is visible by testing if
     * it is inside the camera frustum. The difference from
     * PinholeCamera::pointInFrustum is that in this function it is assumed
     * that the far plane is at infinity.
     */
    bool pointInFrustumInf(const Eigen::Vector3f& point) const;

    /**
     * Test whether a sphere  in camera coordinates is visible by testing if
     * its center is inside the camera frustum offest outwards by the sphere's
     * radius. This is a quick test that in some rare cases may return a sphere
     * as being visible although it isn't.
     */
    bool sphereInFrustum(const Eigen::Vector3f& center,
                         const float            radius) const;

    /**
     * Test whether a sphere  in camera coordinates is visible by testing if
     * its center is inside the camera frustum offest outwards by the sphere's
     * radius. This is a quick test that in some rare cases may return a sphere
     * as being visible although it isn't. The difference from
     * PinholeCamera::sphereInFrustum is that in this function it is assumed
     * that the far plane is at infinity.
     */
    bool sphereInFrustumInf(const Eigen::Vector3f& center,
                            const float            radius) const;

  private:
    Vector4fVector frustum_vertices_;
    Vector4fVector frustum_normals_;

    /**
     * Test whether a point in pixel coordinates is inside the image.
     */
    bool pointInImage(const Eigen::Vector2f& point_px) const;

    /**
     * Compute the 8 camera frustum vertrices in camera coordinates. The
     * indices in the PinholeCamera::frustum_vertices_ vector correspond to the
     * following vertices:
     * - 0: top left near
     * - 1: top right near
     * - 2: bottom right near
     * - 3: bottom left near
     * - 4: top left far
     * - 5: top right far
     * - 6: bottom right far
     * - 7: bottom left far
     */
    void computeFrustumVertices();

    /**
     * Compute the 6 camera frustum inwards pointing unit normal vectors for
     * the frustum planes in camera coordinates. The
     * indices in the PinholeCamera::frustum_normals_ vector correspond to the
     * following normal vectors/planes:
     * - 0: near plane vector
     * - 1: far plane vector
     * - 2: left plane vector
     * - 3: right plane vector
     * - 4: bottom plane vector
     * - 5: top plane vector
     */
    void computeFrustumNormals();

    void printFrustumVertices() const;

    void printFrustumNormals() const;

    /**
     * Compute the plane normal vector as (v2 - v1) x (v3 - v2), where x
     * denotes the vector cross product.
     */
    Eigen::Vector3f planeNormal(const Eigen::Vector4f& v1,
                                const Eigen::Vector4f& v2,
                                const Eigen::Vector4f& v3) const;
};

#endif

