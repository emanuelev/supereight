/*

 Copyright (c) 2011-2013 Gerhard Reitmayr, TU Graz

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 Copyright 2016 Emanuele Vespa, Imperial College London 

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

#ifndef _COMMONS_
#define _COMMONS_

#if defined(__GNUC__)
// circumvent packaging problems in gcc 4.7.0
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

// need c headers for __int128 and uint16_t
#include <limits.h>
#endif
#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <iterator>
#include <set>

// Internal dependencies
#include <math_utils.h>
#include <se/constant_parameters.h>

//External dependencies
#undef isnan
#undef isfinite
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/GR_SVD.h>
////////////////////////// MATh STUFF //////////////////////

#define INVALID -2
// DATA TYPE
//

inline
bool is_file(std::string path) {
	struct stat buf;
	stat(path.c_str(), &buf);
	return S_ISREG(buf.st_mode);
}

template<typename T>
std::string NumberToString(T Number, int width = 6) {
	std::ostringstream ss;
	ss << std::setfill('0') << std::setw(width) << Number;
	return ss.str();
}

template<typename T>
void read_input(std::string inputfile, T * in) {
	size_t isize;
	std::ifstream file(inputfile.c_str(),
			std::ios::in | std::ios::binary | std::ios::ate);
	if (file.is_open()) {
		isize = file.tellg();
		file.seekg(0, std::ios::beg);
		file.read((char*) in, isize);
		file.close();
	} else {
		std::cout << "File opening failed : " << inputfile << std::endl;
		exit(1);
	}
}


inline uchar4 gs2rgb(double h) {
	uchar4 rgb;
	double v = 0;
	double r = 0, g = 0, b = 0;
	v = 0.75;
	if (v > 0) {
		double m;
		double sv;
		int sextant;
		double fract, vsf, mid1, mid2;
		m = 0.25;
		sv = 0.6667;
		h *= 6.0;
		sextant = (int) h;
		fract = h - sextant;
		vsf = v * sv * fract;
		mid1 = m + vsf;
		mid2 = v - vsf;
		switch (sextant) {
		case 0:
			r = v;
			g = mid1;
			b = m;
			break;
		case 1:
			r = mid2;
			g = v;
			b = m;
			break;
		case 2:
			r = m;
			g = v;
			b = mid1;
			break;
		case 3:
			r = m;
			g = mid2;
			b = v;
			break;
		case 4:
			r = mid1;
			g = m;
			b = v;
			break;
		case 5:
			r = v;
			g = m;
			b = mid2;
			break;
		default:
			r = 0;
			g = 0;
			b = 0;
			break;
		}
	}
	rgb.x = r * 255;
	rgb.y = g * 255;
	rgb.z = b * 255;
	rgb.w = 0; // Only for padding purposes 
	return rgb;
}

typedef struct Triangle {
  Eigen::Vector3f vertexes[3];
  Eigen::Vector3f vnormals[3];
  Eigen::Vector3f normal;
  float color;
  float surface_area;
  
  Triangle(){ 
    vertexes[0] = Eigen::Vector3f::Constant(0);
    vertexes[1] = Eigen::Vector3f::Constant(0);
    vertexes[2] = Eigen::Vector3f::Constant(0);
    normal = Eigen::Vector3f::Constant(0);
    surface_area = -1.f;
  }
  
  inline bool iszero(const Eigen::Vector3f& v){ 
    return !(v.array() == 0).all();
  }

  inline bool valid(){
    return !(iszero(vertexes[0]) && iszero(vertexes[1]) && iszero(vertexes[2]));
  }

  inline void compute_normal(){
    normal = (vertexes[1] - vertexes[0]).cross(vertexes[2] - vertexes[1]);
  }

  inline void compute_boundingbox(Eigen::Vector3f& minV, Eigen::Vector3f& maxV) const {
    minV = vertexes[0];
    maxV = vertexes[0];
    minV = minV.cwiseMin(vertexes[0]);
    minV = minV.cwiseMin(vertexes[1]);
    minV = minV.cwiseMin(vertexes[2]);
    maxV = maxV.cwiseMax(vertexes[0]);
    maxV = maxV.cwiseMax(vertexes[1]);
    maxV = maxV.cwiseMax(vertexes[2]);
  }

  inline float area() {
    // Use the cached value if available
    if(surface_area > 0) return surface_area;
    Eigen::Vector3f a = vertexes[1] - vertexes[0];
    Eigen::Vector3f b = vertexes[2] - vertexes[1];
    Eigen::Vector3f v = a.cross(b);
    surface_area = v.norm()/2;
    return surface_area; 
  }

  Eigen::Vector3f * uniform_sample(int num){

    Eigen::Vector3f * points = new Eigen::Vector3f[num];
    for(int i = 0; i < num; ++i){
      float u = ((float)rand())/(float)RAND_MAX; 
      float v = ((float)rand())/(float)RAND_MAX;
      if(u + v > 1){
        u = 1 - u;
        v = 1 - v;
      }
      float w = 1 - (u + v);
      points[i] = u*vertexes[0] + v*vertexes[1] + w*vertexes[2];
    } 

    return points;
  }

  Eigen::Vector3f * uniform_sample(int num, unsigned int& seed) const {

    Eigen::Vector3f * points = new Eigen::Vector3f[num];
    for(int i = 0; i < num; ++i){
      float u = ((float)rand_r(&seed))/(float)RAND_MAX; 
      float v = ((float)rand_r(&seed))/(float)RAND_MAX;
      if(u + v > 1){
        u = 1 - u;
        v = 1 - v;
      }
      float w = 1 - (u + v);
      points[i] = u*vertexes[0] + v*vertexes[1] + w*vertexes[2];
    } 
    return points;
  }

} Triangle;

struct RGBVolume {
    uint3 size;
    float3 dim;
    short4 * data;

    RGBVolume() { size = make_uint3(0); dim = make_float3(1); data = NULL; }

    __device__ float4 operator[]( const uint3 & pos ) const {
        const short4 d = data[pos.x + pos.y * size.x + pos.z * size.x * size.y];
        return make_float4(d.x * 0.00003051944088f, d.y * 0.00003051944088f, d.z * 0.00003051944088f, d.w); //  / 32766.0f
    }

    __device__ float3 v(const uint3 & pos) const {
        const float4 val = operator[](pos);
        return make_float3(val.x,val.y,val.z);
    }

    __device__ float3 vs(const uint3 & pos) const {
        const short4 val = data[pos.x + pos.y * size.x + pos.z * size.x * size.y];
        return make_float3(val.x,val.y,val.z);
    }

    __device__ void set(const uint3 & pos, const float4 & d ){
        data[pos.x + pos.y * size.x + pos.z * size.x * size.y] = make_short4(d.x * 32766.0f, d.y * 32766.0f, d.z * 32766.0f, d.w);
    }

    __device__ float3 pos( const uint3 & p ) const {
        return make_float3((p.x + 0.5f) * dim.x / size.x, (p.y + 0.5f) * dim.y / size.y, (p.z + 0.5f) * dim.z / size.z);
    }

    __device__ float3 interp( const float3 & pos ) const {
#if 0   // only for testing without linear interpolation
        const float3 scaled_pos = make_float3((pos.x * size.x / dim.x) , (pos.y * size.y / dim.y) , (pos.z * size.z / dim.z) );
        return v(make_uint3(clamp(make_int3(scaled_pos), make_int3(0), make_int3(size) - make_int3(1))));

#else
        const float3 scaled_pos = make_float3((pos.x * size.x / dim.x) - 0.5f, (pos.y * size.y / dim.y) - 0.5f, (pos.z * size.z / dim.z) - 0.5f);
        const int3 base = make_int3(floorf(scaled_pos));
        const float3 factor = fracf(scaled_pos);
        const int3 lower = max(base, make_int3(0));
        const int3 upper = min(base + make_int3(1), make_int3(size) - make_int3(1));
        return (
              ((vs(make_uint3(lower.x, lower.y, lower.z)) * (1-factor.x) + vs(make_uint3(upper.x, lower.y, lower.z)) * factor.x) * (1-factor.y)
             + (vs(make_uint3(lower.x, upper.y, lower.z)) * (1-factor.x) + vs(make_uint3(upper.x, upper.y, lower.z)) * factor.x) * factor.y) * (1-factor.z)
            + ((vs(make_uint3(lower.x, lower.y, upper.z)) * (1-factor.x) + vs(make_uint3(upper.x, lower.y, upper.z)) * factor.x) * (1-factor.y)
             + (vs(make_uint3(lower.x, upper.y, upper.z)) * (1-factor.x) + vs(make_uint3(upper.x, upper.y, upper.z)) * factor.x) * factor.y) * factor.z
            ) * 0.00003051944088f;
#endif
    }

    void init(uint3 s, float3 d){
        size = s;
        dim = d;
	 #ifdef CUDABASED
            cudaMalloc( &data,  size.x * size.y * size.z * sizeof(short4) ); 
         #else
	    std::cout << sizeof(short4) << std::endl;
	    data = (short4 *) malloc(size.x * size.y * size.z * sizeof(short4)); 
	    assert(data != (0));
         #endif
        //cudaMalloc(&data, size.x * size.y * size.z * sizeof(short4));
    }

    void release(){
      //cudaFree(data);
      //data = NULL;
           #ifdef ZEROCOPY_OPENCL
    	    clEnqueueUnmapMemObject(commandQueue, oclbuffer, data, 0, NULL, NULL);
    		clReleaseMemObject(oclbuffer);
		#else
        	free(data);
        	data = NULL;
		#endif // if def ZEROCOPY_OPENCL
    }
};


struct TrackData {
	int result;
	float error;
	float J[6];
};

inline Eigen::Matrix4f getCameraMatrix(const Eigen::Vector4f& k) {
  Eigen::Matrix4f K;
	K << k.x(), 0, k.z(), 0,
       0, k.y(), k.w(), 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
	return K;
}

inline Eigen::Matrix4f getInverseCameraMatrix(const Eigen::Vector4f& k) {
  Eigen::Matrix4f invK;
  invK << 1.0f / k.x(), 0, -k.z() / k.x(), 0,
       0, 1.0f / k.y(), -k.w() / k.y(), 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
	return invK;
}


//std::ostream& operator<<(std::ostream& os, const uint3 val) {
//    os << val.x << ", " << val.y << ", " << val.z;
//    return os;
//}

template<typename P, typename A>
TooN::Matrix<6> makeJTJ(const TooN::Vector<21, P, A> & v) {
	TooN::Matrix<6> C = TooN::Zeros;
	C[0] = v.template slice<0, 6>();
	C[1].template slice<1, 5>() = v.template slice<6, 5>();
	C[2].template slice<2, 4>() = v.template slice<11, 4>();
	C[3].template slice<3, 3>() = v.template slice<15, 3>();
	C[4].template slice<4, 2>() = v.template slice<18, 2>();
	C[5][5] = v[20];

	for (int r = 1; r < 6; ++r)
		for (int c = 0; c < r; ++c)
			C[r][c] = C[c][r];

	return C;
}

template<typename T, typename A>
TooN::Vector<6> solve(const TooN::Vector<27, T, A> & vals) {
	const TooN::Vector<6> b = vals.template slice<0, 6>();
	const TooN::Matrix<6> C = makeJTJ(vals.template slice<6, 21>());

	TooN::GR_SVD<6, 6> svd(C);
	return svd.backsub(b, 1e6);
}

template<typename P>
inline Eigen::Matrix4f toMatrix4f(const TooN::SE3<P> & p) {
	const TooN::Matrix<4, 4, float> I = TooN::Identity;
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor>  R;
	TooN::wrapMatrix<4, 4>(R.data()) = p * I;
	return R;
}

static const float epsilon = 0.0000001;

inline void compareTrackData(std::string str, TrackData* l, TrackData * r,
		uint size) {
	for (unsigned int i = 0; i < size; i++) {
		if (std::abs(l[i].error - r[i].error) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.error =  " << l[i].error << std::endl;
			std::cout << "r.error =  " << r[i].error << std::endl;
		}

		if (std::abs(l[i].result - r[i].result) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.result =  " << l[i].result << std::endl;
			std::cout << "r.result =  " << r[i].result << std::endl;
		}

	}
}

inline void compareFloat(std::string str, float* l, float * r, uint size) {
	for (unsigned int i = 0; i < size; i++) {
		if (std::abs(l[i] - r[i]) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l =  " << l[i] << std::endl;
			std::cout << "r =  " << r[i] << std::endl;
		}
	}
}
inline void compareFloat3(std::string str, float3* l, float3 * r, uint size) {
	for (unsigned int i = 0; i < size; i++) {
		if (std::abs(l[i].x - r[i].x) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.x =  " << l[i].x << std::endl;
			std::cout << "r.x =  " << r[i].x << std::endl;
		}
		if (std::abs(l[i].y - r[i].y) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.y =  " << l[i].y << std::endl;
			std::cout << "r.y =  " << r[i].y << std::endl;
		}
		if (std::abs(l[i].z - r[i].z) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.z =  " << l[i].z << std::endl;
			std::cout << "r.z =  " << r[i].z << std::endl;
		}
	}
}

inline void compareFloat4(std::string str, float4* l, float4 * r, uint size) {
	for (unsigned int i = 0; i < size; i++) {
		if (std::abs(l[i].x - r[i].x) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.x =  " << l[i].x << std::endl;
			std::cout << "r.x =  " << r[i].x << std::endl;
		}
		if (std::abs(l[i].y - r[i].y) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.y =  " << l[i].y << std::endl;
			std::cout << "r.y =  " << r[i].y << std::endl;
		}
		if (std::abs(l[i].z - r[i].z) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.z =  " << l[i].z << std::endl;
			std::cout << "r.z =  " << r[i].z << std::endl;
		}
		if (std::abs(l[i].w - r[i].w) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.w =  " << l[i].w << std::endl;
			std::cout << "r.w =  " << r[i].w << std::endl;
		}
	}
}


inline bool compareFloat4(float4* l, float4 * r, uint size) {
	for (unsigned int i = 0; i < size; i++) {
		if ((std::abs(l[i].x - r[i].x) > epsilon) ||
		    (std::abs(l[i].y - r[i].y) > epsilon) ||
		    (std::abs(l[i].z - r[i].z) > epsilon) ||
		    (std::abs(l[i].w - r[i].w) > epsilon))
     return false; 
	}
  return true;
}

inline void compareNormal(std::string str, float3* l, float3 * r, uint size) {
	for (unsigned int i = 0; i < size; i++) {
		if (std::abs(l[i].x - r[i].x) > epsilon) {
			std::cout << "Error into " << str << " at " << i << std::endl;
			std::cout << "l.x =  " << l[i].x << std::endl;
			std::cout << "r.x =  " << r[i].x << std::endl;
		} else if (r[i].x != INVALID) {
			if (std::abs(l[i].y - r[i].y) > epsilon) {
				std::cout << "Error into " << str << " at " << i << std::endl;
				std::cout << "l.y =  " << l[i].y << std::endl;
				std::cout << "r.y =  " << r[i].y << std::endl;
			}
			if (std::abs(l[i].z - r[i].z) > epsilon) {
				std::cout << "Error into " << str << " at " << i << std::endl;
				std::cout << "l.z =  " << l[i].z << std::endl;
				std::cout << "r.z =  " << r[i].z << std::endl;
			}
		}
	}
}

template<typename T>
void writefile(std::string prefix, int idx, T * data, uint size) {

	std::string filename = prefix + NumberToString(idx);
	FILE* pFile = fopen(filename.c_str(), "wb");

	if (!pFile) {
		std::cout << "File opening failed : " << filename << std::endl;
		exit(1);
	}

	size_t write_cnt = fwrite(data, sizeof(T), size, pFile);

	std::cout << "File " << filename << " of size " << write_cnt << std::endl;

	fclose(pFile);
}

template<typename T>
void writefile(std::string prefix, int idx, T * data, uint2 size) {
	writefile(prefix, idx, data, size.x * size.y);
}

// void writeVolume(std::string filename, Volume v) {
// 
// 	std::ofstream fDumpFile;
// 	fDumpFile.open(filename.c_str(), std::ios::out | std::ios::binary);
// 
// 	if (fDumpFile.fail()) {
// 		std::cout << "Error opening file: " << filename << std::endl;
// 		exit(1);
// 	}
// 
// 	// Retrieve the volumetric representation data
// 	short2 *hostData = (short2 *) v.data;
// 
// 	// Dump on file without the y component of the short2 variable
// 	for (unsigned int i = 0; i < v.size.x * v.size.y * v.size.z; i++) {
// 		fDumpFile.write((char *) (hostData + i), sizeof(short));
// 	}
// 
// 	fDumpFile.close();
// }

inline void writeVtkMesh(const char * filename, 
                         const std::vector<Triangle>& mesh,
                         const float * point_data = NULL,
                         const float * cell_data = NULL){
  std::stringstream points;
  std::stringstream polygons;
  std::stringstream pointdata;
  std::stringstream celldata;
  int point_count = 0;
  int triangle_count = 0;
  bool hasPointData = point_data != NULL;
  bool hasCellData = cell_data != NULL;

  for(unsigned int i = 0; i < mesh.size(); ++i ){
    const Triangle& t = mesh[i];

    points << t.vertexes[0](0) << " " << t.vertexes[0](1) << " " 
      << t.vertexes[0](2) << std::endl; 
    points << t.vertexes[1](0) << " " << t.vertexes[1](1) << " " 
      << t.vertexes[1](2) << std::endl; 
    points << t.vertexes[2](0) << " " << t.vertexes[2](1) << " " 
      << t.vertexes[2](2) << std::endl; 

    polygons << "3 " << point_count << " " << point_count+1 << 
      " " << point_count+2 << std::endl;

    if(hasPointData){
      pointdata << point_data[i*3] << std::endl;
      pointdata << point_data[i*3 + 1] << std::endl;
      pointdata << point_data[i*3 + 2] << std::endl;
    }

    if(hasCellData){
      celldata << cell_data[i] << std::endl;
    }

    point_count +=3;
    triangle_count++;
  }   

  std::ofstream f;
  f.open(filename);
  f << "# vtk DataFile Version 1.0" << std::endl;
  f << "vtk mesh generated from KFusion" << std::endl;
  f << "ASCII" << std::endl;
  f << "DATASET POLYDATA" << std::endl;

  f << "POINTS " << point_count << " FLOAT" << std::endl;
  f << points.str();

  f << "POLYGONS " << triangle_count << " " << triangle_count * 4 << std::endl;
  f << polygons.str() << std::endl;
  if(hasPointData){
    f << "POINT_DATA " << point_count << std::endl; 
    f << "SCALARS vertex_scalars float 1" << std::endl;
    f << "LOOKUP_TABLE default" << std::endl;
    f << pointdata.str();
  }

  if(hasCellData){
    f << "CELL_DATA " << triangle_count << std::endl; 
    f << "SCALARS cell_scalars float 1" << std::endl;
    f << "LOOKUP_TABLE default" << std::endl;
    f << celldata.str();
  }
  f.close();
}

inline void writeObjMesh(const char * filename,
                         const std::vector<Triangle>& mesh){
  std::stringstream points;
  std::stringstream faces;
  int point_count = 0;
  int face_count = 0;

  for(unsigned int i = 0; i < mesh.size(); i++){
    const Triangle& t = mesh[i];  
    points << "v " << t.vertexes[0](0) << " " << t.vertexes[0](1)
           << " "  << t.vertexes[0](2) << std::endl;
    points << "v " << t.vertexes[1](0) << " " << t.vertexes[1](1) 
           << " "  << t.vertexes[1](2) << std::endl;
    points << "v " << t.vertexes[2](0) << " " << t.vertexes[2](1) 
           << " "  << t.vertexes[2](2) << std::endl;

    faces  << "f " << (face_count*3)+1 << " " << (face_count*3)+2 
           << " " << (face_count*3)+3 << std::endl;

    point_count +=3;
    face_count += 1;
  }

  std::ofstream f(filename); 
  f << "# OBJ file format with ext .obj" << std::endl;
  f << "# vertex count = " << point_count << std::endl;
  f << "# face count = " << face_count << std::endl;
  f << points.str();
  f << faces.str();
  f.close();
  std::cout << "Written " << face_count << " faces and " << point_count 
            << " points" << std::endl;
}
#endif
