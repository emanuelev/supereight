/*
 * Copyright 1993-2012 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO LICENSEE: 
 *
 * This source code and/or documentation ("Licensed Deliverables") are
 * subject to NVIDIA intellectual property rights under U.S. and
 * international Copyright laws.
 *
 * These Licensed Deliverables contained herein is PROPRIETARY and
 * CONFIDENTIAL to NVIDIA and is being provided under the terms and
 * conditions of a form of NVIDIA software license agreement by and
 * between NVIDIA and Licensee ("License Agreement") or electronically
 * accepted by Licensee.  Notwithstanding any terms or conditions to
 * the contrary in the License Agreement, reproduction or disclosure
 * of the Licensed Deliverables to any third party without the express
 * written consent of NVIDIA is prohibited.
 *
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, NVIDIA MAKES NO REPRESENTATION ABOUT THE
 * SUITABILITY OF THESE LICENSED DELIVERABLES FOR ANY PURPOSE.  IT IS
 * PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY OF ANY KIND.
 * NVIDIA DISCLAIMS ALL WARRANTIES WITH REGARD TO THESE LICENSED
 * DELIVERABLES, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY
 * SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 * WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THESE LICENSED DELIVERABLES.
 *
 * U.S. Government End Users.  These Licensed Deliverables are a
 * "commercial item" as that term is defined at 48 C.F.R. 2.101 (OCT
 * 1995), consisting of "commercial computer software" and "commercial
 * computer software documentation" as such terms are used in 48
 * C.F.R. 12.212 (SEPT 1995) and is provided to the U.S. Government
 * only as a commercial end item.  Consistent with 48 C.F.R.12.212 and
 * 48 C.F.R. 227.7202-1 through 227.7202-4 (JUNE 1995), all
 * U.S. Government End Users acquire the Licensed Deliverables with
 * only those rights set forth herein.
 *
 * Any use of the Licensed Deliverables in individual and commercial
 * software must include, in the user documentation and internal
 * comments to the code, the above Disclaimer and U.S. Government End
 * Users Notice.
 */

#if !defined(__VECTOR_TYPES_H__)
#define __VECTOR_TYPES_H__

/*******************************************************************************
 *                                                                              *
 *                                                                              *
 *                                                                              *
 *******************************************************************************/

/*******************************************************************************
 *                                                                              *
 *                                                                              *
 *                                                                              *
 *******************************************************************************/

struct __device_builtin__char1 {
	signed char x;
};

struct __device_builtin__uchar1 {
	unsigned char x;
};

struct __device_builtin__char2 {
	signed char x, y;
};

struct __device_builtin__uchar2 {
	unsigned char x, y;
};

struct __device_builtin__char3 {
	signed char x, y, z;
};

struct __device_builtin__uchar3 {
	unsigned char x, y, z;
};

struct __device_builtin__char4 {
	signed char x, y, z, w;
};

struct __device_builtin__uchar4 {
	unsigned char x, y, z, w;
};

struct __device_builtin__short1 {
	short x;
};

struct __device_builtin__ushort1 {
	unsigned short x;
};

struct __device_builtin__short2 {
	short x, y;
};

struct __device_builtin__ushort2 {
	unsigned short x, y;
};

struct __device_builtin__short3 {
	short x, y, z;
};

struct __device_builtin__ushort3 {
	unsigned short x, y, z;
};

struct __device_builtin__short4 {
	short x, y, z, w;
};


//__cuda_builtin_vector_align8(short4, short x; short y; short z; short w;);
//__cuda_builtin_vector_align8(ushort4, unsigned short x; unsigned short y; unsigned short z; unsigned short w;);

struct __device_builtin__int1 {
	int x;
};
struct __device_builtin__int2 {
	int x;
	int y;
};

struct __device_builtin__uint1 {
	unsigned int x;
};
struct __device_builtin__uint2 {
	unsigned int x;
	unsigned int y;
};

//__cuda_builtin_vector_align8(int2, int x; int y;);
//__cuda_builtin_vector_align8(uint2, unsigned int x; unsigned int y;);

struct __device_builtin__int3 {
	int x, y, z;
};

struct __device_builtin__uint3 {
	unsigned int x, y, z;
    bool operator<(const __device_builtin__uint3& rhs) const{
        if( x < rhs.x) return true;
        else if( y < rhs.y) return true;
        else if( z < rhs.z) return true;
        return false;
    }

};

struct __device_builtin__int4 {
	int x, y, z, w;
};

struct __device_builtin__uint4 {
	unsigned int x, y, z, w;
};

struct __device_builtin__long1 {
	long int x;
};

struct __device_builtin__ulong1 {
	unsigned long x;
};

struct __device_builtin__long2 {
	long int x, y;
};

struct __device_builtin__ulong2 {
	unsigned long int x, y;
};

struct __device_builtin__long3 {
	long int x, y, z;
};

struct __device_builtin__ulong3 {
	unsigned long int x, y, z;
};

struct __device_builtin__long4 {
	long int x, y, z, w;
};

struct __device_builtin__ulong4 {
	unsigned long int x, y, z, w;
};

struct __device_builtin__float1 {
	float x;
};

struct __device_builtin__float2 {
	float x;
	float y;
};

struct __device_builtin__float3 {
	float x, y, z;
};

struct __device_builtin__float4 {
	float x, y, z, w;
};

struct __device_builtin__longlong1 {
	long long int x;
};

struct __device_builtin__ulonglong1 {
	unsigned long long int x;
};

struct __device_builtin__longlong2 {
	long long int x, y;
};

struct __device_builtin__ulonglong2 {
	unsigned long long int x, y;
};

struct __device_builtin__longlong3 {
	long long int x, y, z;
};

struct __device_builtin__ulonglong3 {
	unsigned long long int x, y, z;
};

struct __device_builtin__longlong4 {
	long long int x, y, z, w;
};

struct __device_builtin__ulonglong4 {
	unsigned long long int x, y, z, w;
};

struct __device_builtin__double1 {
	double x;
};

struct __device_builtin__double2 {
	double x, y;
};

struct __device_builtin__double3 {
	double x, y, z;
};

struct __device_builtin__double4 {
	double x, y, z, w;
};

/*******************************************************************************
 *                                                                              *
 *                                                                              *
 *                                                                              *
 *******************************************************************************/

typedef struct __device_builtin__char1 char1;
typedef struct __device_builtin__uchar1 uchar1;
typedef struct __device_builtin__char2 char2;
typedef struct __device_builtin__uchar2 uchar2;
typedef struct __device_builtin__char3 char3;
typedef struct __device_builtin__uchar3 uchar3;
typedef struct __device_builtin__char4 char4;
typedef struct __device_builtin__uchar4 uchar4;
typedef struct __device_builtin__short1 short1;
typedef struct __device_builtin__ushort1 ushort1;
typedef struct __device_builtin__short2 short2;
typedef struct __device_builtin__ushort2 ushort2;
typedef struct __device_builtin__short3 short3;
typedef struct __device_builtin__ushort3 ushort3;
typedef struct __device_builtin__short4 short4;
typedef struct __device_builtin__ushort4 ushort4;
typedef struct __device_builtin__int1 int1;
typedef struct __device_builtin__uint1 uint1;
typedef struct __device_builtin__int2 int2;
typedef struct __device_builtin__uint2 uint2;
typedef struct __device_builtin__int3 int3;
typedef struct __device_builtin__uint3 uint3;
typedef struct __device_builtin__int4 int4;
typedef struct __device_builtin__uint4 uint4;
typedef struct __device_builtin__long1 long1;
typedef struct __device_builtin__ulong1 ulong1;
typedef struct __device_builtin__long2 long2;
typedef struct __device_builtin__ulong2 ulong2;
typedef struct __device_builtin__long3 long3;
typedef struct __device_builtin__ulong3 ulong3;
typedef struct __device_builtin__long4 long4;
typedef struct __device_builtin__ulong4 ulong4;
typedef struct __device_builtin__float1 float1;
typedef struct __device_builtin__float2 float2;
typedef struct __device_builtin__float3 float3;
typedef struct __device_builtin__float4 float4;

typedef struct __device_builtin__longlong1 longlong1;
typedef struct __device_builtin__ulonglong1 ulonglong1;
typedef struct __device_builtin__longlong2 longlong2;
typedef struct __device_builtin__ulonglong2 ulonglong2;
typedef struct __device_builtin__longlong3 longlong3;
typedef struct __device_builtin__ulonglong3 ulonglong3;
typedef struct __device_builtin__longlong4 longlong4;
typedef struct __device_builtin__ulonglong4 ulonglong4;
typedef struct __device_builtin__double1 double1;
typedef struct __device_builtin__double2 double2;
typedef struct __device_builtin__double3 double3;
typedef struct __device_builtin__double4 double4;

/*******************************************************************************
 *                                                                              *
 *                                                                              *
 *                                                                              *
 *******************************************************************************/

inline uint4 make_uint4(unsigned int x, unsigned int y, unsigned int z,
		unsigned int w) {

	uint4 val;
	val.x = x;
	val.y = y;
	val.z = z;
	val.w = w;
	return val;
}

inline int4 make_int4(int x, int y, int z, int w) {

	int4 val;
	val.x = x;
	val.y = y;
	val.z = z;
	val.w = w;
	return val;
}
inline uint3 make_uint3(unsigned int x, unsigned int y, unsigned int z) {

	uint3 val;
	val.x = x;
	val.y = y;
	val.z = z;
	return val;
}
inline short2 make_short2(short x, short y) {
	short2 val;
	val.x = x;
	val.y = y;
	return val;
}

inline int3 make_int3(int x, int y, int z) {

	int3 val;
	val.x = x;
	val.y = y;
	val.z = z;
	return val;
}
inline float4 make_float4(float x, float y, float z, float w) {

	float4 val;
	val.x = x;
	val.y = y;
	val.z = z;
	val.w = w;
	return val;
}
inline float3 make_float3(float x, float y, float z) {

	float3 val;
	val.x = x;
	val.y = y;
	val.z = z;
	return val;
}
inline float2 make_float2(float x, float y) {

	float2 val;
	val.x = x;
	val.y = y;
	return val;
}
inline int2 make_int2(int x, int y) {
	int2 val;
	val.x = x;
	val.y = y;
	return val;
}
inline uint2 make_uint2(unsigned int x, unsigned int y) {
	uint2 val;
	val.x = x;
	val.y = y;
	return val;
}
inline uchar3 make_uchar3(unsigned char x, unsigned char y, unsigned char z) {
	uchar3 val;
	val.x = x;
	val.y = y;
	val.z = z;
	return val;
}
inline uchar4 make_uchar4(unsigned char x, unsigned char y, unsigned char z,
		unsigned char w) {
	uchar4 val;
	val.x = x;
	val.y = y;
	val.z = z;
	val.w = w;
	return val;
}

inline short4 make_short4(short x, short y, short z, short w){
    short4 val;
    val.x = x;
    val.y = y;
    val.z = z;
    val.w = w;
    return val;
    }

class dim3 {
public:
	unsigned int x, y, z;
#if defined(__cplusplus)
	dim3(unsigned int vx = 1, unsigned int vy = 1, unsigned int vz = 1) :
			x(vx), y(vy), z(vz) {
	}

	dim3(uint3 v) :
			x(v.x), y(v.y), z(v.z) {
	}
	operator uint3(void) {
		uint3 t;
		t.x = x;
		t.y = y;
		t.z = z;
		return t;
	}
#endif /* __cplusplus */
};

#endif /* !__VECTOR_TYPES_H__ */
