#ifndef CUTIL_HELPER_H
#define CUTIL_HELPER_H
////////////////////////////////////////////////////////////////////////////////
// ceilf - missing from cutil_math.h
////////////////////////////////////////////////////////////////////////////////


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

inline __host__     __device__ float2 ceilf(float2 v) {
  return make_float2(ceilf(v.x), ceilf(v.y));
}
inline __host__     __device__ float3 ceilf(float3 v) {
  return make_float3(ceilf(v.x), ceilf(v.y), ceilf(v.z));
}
inline __host__     __device__ float4 ceilf(float4 v) {
  return make_float4(ceilf(v.x), ceilf(v.y), ceilf(v.z), ceilf(v.w));
}

inline __host__     __device__ uchar3 operator*(const uchar3 a, float v) {
	return make_uchar3(a.x * v, a.y * v, a.z * v);
}

static inline std::ostream& operator<<(std::ostream& os, const uint3& val) {
  os << "(" << val.x << ", " << val.y << ", " << val.z << ")";
  return os;
}
#endif
