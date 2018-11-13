#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "thirdparty/vector_types.h"
#include "thirdparty/cutil_math.h"


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


////////////////////////////////////////////////////////////////////////////////
// ceilf - missing from cutil_math.h
////////////////////////////////////////////////////////////////////////////////

inline __host__     __device__ float2 ceilf(float2 v) {
	return make_float2(ceilf(v.x), ceilf(v.y));
}
inline __host__     __device__ float3 ceilf(float3 v) {
	return make_float3(ceilf(v.x), ceilf(v.y), ceilf(v.z));
}
inline __host__     __device__ float4 ceilf(float4 v) {
	return make_float4(ceilf(v.x), ceilf(v.y), ceilf(v.z), ceilf(v.w));
}

inline __host__ __device__ bool operator==(const float3 a, float b){
  return((a.x == b) && (a.y == b) && (a.z == b));
}

inline __host__ __device__ bool in(const unsigned int value, const unsigned int lower, 
               const unsigned int upper){
  return value >= lower && value <= upper;
}

inline __host__ __device__ bool in(const int value, const int lower, 
               const int upper){
  return value >= lower && value <= upper;
}

inline __host__     __device__ uchar3 operator*(const uchar3 a, float v) {
	return make_uchar3(a.x * v, a.y * v, a.z * v);
}

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

static inline std::ostream& operator<<(std::ostream& os, const uint3& val) {
  os << "(" << val.x << ", " << val.y << ", " << val.z << ")";
  return os;
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
