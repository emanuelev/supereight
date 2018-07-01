/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <iterator>
#include<stdint.h>
#include<vector>
#include <sstream>
#include <string>
#include <cstring>
#include <cmath>

#include "lodepng.h"

#include <sstream>
#include <iomanip>

typedef unsigned short int ushort;

static const float SceneK[3][3] = { 481.20, 0.00, 319.50, 0.00, -480.00, 239.50,
		0.00, 0.00, 1.00 };

static const int _scenewidth = 640;
static const int _sceneheight = 480;
static const float _u0 = SceneK[0][2];
static const float _v0 = SceneK[1][2];
static const float _focal_x = SceneK[0][0];
static const float _focal_y = SceneK[1][1];

struct uchar3 {
	unsigned char x, y, z;
};
struct uchar4 {
	unsigned char x, y, z, w;
};

struct uint2 {
	unsigned int x, y;
};

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

inline uint2 make_uint2(unsigned int x, unsigned int y) {
	uint2 val;
	val.x = x;
	val.y = y;
	return val;
}

int readDepthFile(ushort * depthMap, const char * filename) {

	std::ifstream source;
	source.open(filename, std::ios_base::in);
	double * scene_depth_float_buffer = (double*) malloc(
			sizeof(double) * _scenewidth * _sceneheight);

	if (!source) {
		std::cerr << "Can't open Data from " << filename << " !\n";
		return -1;
	}
	int index = 0;
	while (source.good()) {
		double d;
		source >> d;
		if (640 * 480 <= index)
			continue; // FIXME : not sure why it read one more float
		scene_depth_float_buffer[index] = d * 1000;
		index++;
	}

	for (int v = 0; v < _sceneheight; v++) {
		for (int u = 0; u < _scenewidth; u++) {
			double u_u0_by_fx = (u - _u0) / _focal_x;
			double v_v0_by_fy = (v - _v0) / _focal_y;

			depthMap[u + v * _scenewidth] = scene_depth_float_buffer[u
					+ v * _scenewidth]
					/ std::sqrt(
							u_u0_by_fx * u_u0_by_fx + v_v0_by_fy * v_v0_by_fy
									+ 1);

		}
	}
	free(scene_depth_float_buffer);
	//std::cout << "End of file reading (" << index <<  ")." << std::endl;
	return index;
}

int main(int argc, char ** argv) {

	uint2 inputSize = make_uint2(640, 480);
	static const uint2 imageSize = { 320, 240 };

	if (argc != 3) {
		std::cout
				<< "Bad entries... I just need the scene directory and the output file"
				<< std::endl;
		exit(1);
	}

	// Input
	// wait for something like this : scene/ ( scene_00_0000.depth )
	std::string dir = argv[1];
	FILE* pFile = fopen(argv[2], "wb");
	if (!pFile) {
		std::cout << "File opening failed : " << argv[2] << std::endl;
		exit(1);
	}

	uchar3 * rgbImage = (uchar3*) malloc(
			sizeof(uchar3) * inputSize.x * inputSize.y);

	ushort * inputFile = (ushort*) malloc(
			sizeof(ushort) * inputSize.x * inputSize.y);

	for (int i = 0; true; i++) {

		unsigned error;
		uchar4* image;
		unsigned width, height;

		std::ostringstream filename;
		std::ostringstream rgbfilename;

		filename << dir << "/scene_00_" << std::setfill('0') << std::setw(4)
				<< i << ".depth";
		rgbfilename << dir << "/scene_00_" << std::setfill('0') << std::setw(4)
				<< i << ".png";

		if (readDepthFile(inputFile, filename.str().c_str()) == -1) {
			break;
		}

		error = lodepng_decode32_file((unsigned char**) &image, &width, &height,
				rgbfilename.str().c_str());
		if (error) {
			printf("error %u: %s\n", error, lodepng_error_text(error));
		} else {
			for (unsigned int i = 0; i < inputSize.y * inputSize.x; i++) {
				rgbImage[i] = make_uchar3(image[i].x, image[i].y, image[i].z);
			}
		}

		int total = 0;
		total += fwrite(&(inputSize), sizeof(imageSize), 1, pFile);
		total += fwrite(inputFile, sizeof(uint16_t), inputSize.x * inputSize.y,
				pFile);
		total += fwrite(&(inputSize), sizeof(imageSize), 1, pFile);
		total += fwrite(rgbImage, sizeof(uchar3), inputSize.x * inputSize.y,
				pFile);

		std::cout << "\rRead frame " << std::setw(10) << i << " ";

		if (i % 2) {
			fflush(stdout);
		}
		free(image);
	}
	std::cout << std::endl;
	fclose(pFile);
}
