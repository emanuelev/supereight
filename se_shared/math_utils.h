#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <iostream>
#include <cmath>
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

typedef struct sMatrix4 {
	float4 data[4];
} Matrix4;

inline __host__ __device__ float3 get_translation(const Matrix4& view) {
	return make_float3(view.data[0].w, view.data[1].w, view.data[2].w);
}

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

inline float4 operator*(const Matrix4 & M, const float4 & v) {
	return make_float4(dot(M.data[0], v), dot(M.data[1], v), dot(M.data[2], v),
			dot(M.data[3], v));
}

inline float sq(float r) {
	return r * r;
}

inline Matrix4 outer(const float4& a, const float4& b){
  Matrix4 mat;
  mat.data[0] = make_float4(a.x*b.x, a.x*b.y, a.x*b.z, a.x*b.w);
  mat.data[1] = make_float4(a.y*b.x, a.y*b.y, a.y*b.z, a.y*b.w);
  mat.data[2] = make_float4(a.z*b.x, a.z*b.y, a.z*b.z, a.z*b.w);
  mat.data[3] = make_float4(a.w*b.x, a.w*b.y, a.w*b.z, a.w*b.w);
  return mat;
}

inline __host__      __device__ float3 operator*(const Matrix4 & M,
		const float3 & v) {
	return make_float3(dot(make_float3(M.data[0]), v) + M.data[0].w,
			dot(make_float3(M.data[1]), v) + M.data[1].w,
			dot(make_float3(M.data[2]), v) + M.data[2].w);
}

inline float3 rotate(const Matrix4 & M, const float3 & v) {
	return make_float3(dot(make_float3(M.data[0]), v),
			dot(make_float3(M.data[1]), v), dot(make_float3(M.data[2]), v));
}

// Converting quaternion and trans to SE3 matrix 
// Following the implementation provided in TUM scripts.
inline Matrix4 toMatrix4(float4 quat, const float3& trans) {
  const float n = dot(quat, quat);
  quat = quat*(sqrtf(2.0/n));
  Matrix4 mat = outer(quat, quat);
  Matrix4 se3_mat;
  se3_mat.data[0] = make_float4(1.0-mat.data[1].y - mat.data[2].z, mat.data[0].y - mat.data[2].w,     mat.data[0].z + mat.data[1].w, trans.x);
  se3_mat.data[1] = make_float4(mat.data[0].y + mat.data[2].w, 1.0-mat.data[0].x - mat.data[2].z,     mat.data[1].z - mat.data[0].w, trans.y);
  se3_mat.data[2] = make_float4(mat.data[0].z - mat.data[1].w,     mat.data[1].z + mat.data[0].w, 1.0-mat.data[0].x - mat.data[1].y, trans.z);
  se3_mat.data[3] = make_float4(0.0, 0.0, 0.0, 1.0);
  return se3_mat;
}

// Converting quaternion and trans to SE3 matrix 
// Following the implementation provided in TUM scripts.
inline Matrix4 toMatrix4(const float3& trans) {
  Matrix4 se3_mat;
  se3_mat.data[0] = {1.f, 0.f, 0.f, trans.x};
  se3_mat.data[1] = {0.f, 1.f, 0.f, trans.y};
  se3_mat.data[2] = {0.f, 0.f, 1.f, trans.z};
  se3_mat.data[3] = {0.f, 0.f, 0.f, 1.f};
  return se3_mat;
}

constexpr int log2_const(int n){
  return (n < 2 ? 0 : 1 + log2_const(n/2));
}

static inline std::ostream& operator<<(std::ostream& os, const uint3& val) {
  os << "(" << val.x << ", " << val.y << ", " << val.z << ")";
  return os;
}

#endif
