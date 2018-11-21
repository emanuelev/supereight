#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#define SOPHUS_DISABLE_ENSURES
/* 
 * When compiling in debug mode Eigen compilation fails 
 * due to -Wunused-parameter. Disable it if compiling with GCC.
 */
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#if __GNUC__ > 6
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#endif
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#pragma GCC diagnostic pop
#else
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#endif



inline float sq(float r) {
	return r * r;
}

inline Eigen::Matrix4f toMatrix4f(const Eigen::Vector3f& trans) {
  Eigen::Matrix4f se3_mat;  
  se3_mat << 1.f ,  0.f ,  0.f , trans.x(), 
             0.f ,  1.f ,  0.f , trans.y(), 
             0.f ,  0.f ,  1.f , trans.z(), 
             0.f ,  0.f ,  0.f ,  1.f;
  return se3_mat;
}

constexpr int log2_const(int n){
  return (n < 2 ? 0 : 1 + log2_const(n/2));
}

template <typename T>
inline T clamp(const T& f, const T& a, const T& b) {
	return std::max(a, std::min(f, b));
}
static inline void clamp(Eigen::Ref<Eigen::VectorXf> res, const Eigen::Ref<const Eigen::VectorXf> a, 
          const Eigen::Ref<Eigen::VectorXf> b) {
  res = (res.array() < a.array()).select(a, res);
  res = (res.array() >= b.array()).select(b, res);
} 

template <typename R, typename A, typename B>
static inline void clamp(Eigen::MatrixBase<R>& res, const Eigen::MatrixBase<A>& a, 
          const Eigen::MatrixBase<B>& b) {
 res = res.array().max(a.array());
 res = res.array().min(b.array());
} 
#endif
