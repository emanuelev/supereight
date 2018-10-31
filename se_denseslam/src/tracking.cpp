/*

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
#include <math_utils.h>
#include "timings.h"

#include <se/commons.h>
#include <se/image/image.hpp>

void new_reduce(int blockIndex, float * out, TrackData* J, const uint2 Jsize,
		const uint2 size) {
	float *sums = out + blockIndex * 32;

	for (uint i = 0; i < 32; ++i)
		sums[i] = 0;
	float sums0, sums1, sums2, sums3, sums4, sums5, sums6, sums7, sums8, sums9,
			sums10, sums11, sums12, sums13, sums14, sums15, sums16, sums17,
			sums18, sums19, sums20, sums21, sums22, sums23, sums24, sums25,
			sums26, sums27, sums28, sums29, sums30, sums31;
	sums0 = 0.0f;
	sums1 = 0.0f;
	sums2 = 0.0f;
	sums3 = 0.0f;
	sums4 = 0.0f;
	sums5 = 0.0f;
	sums6 = 0.0f;
	sums7 = 0.0f;
	sums8 = 0.0f;
	sums9 = 0.0f;
	sums10 = 0.0f;
	sums11 = 0.0f;
	sums12 = 0.0f;
	sums13 = 0.0f;
	sums14 = 0.0f;
	sums15 = 0.0f;
	sums16 = 0.0f;
	sums17 = 0.0f;
	sums18 = 0.0f;
	sums19 = 0.0f;
	sums20 = 0.0f;
	sums21 = 0.0f;
	sums22 = 0.0f;
	sums23 = 0.0f;
	sums24 = 0.0f;
	sums25 = 0.0f;
	sums26 = 0.0f;
	sums27 = 0.0f;
	sums28 = 0.0f;
	sums29 = 0.0f;
	sums30 = 0.0f;
	sums31 = 0.0f;
// comment me out to try coarse grain parallelism 
#pragma omp parallel for reduction(+:sums0,sums1,sums2,sums3,sums4,sums5,sums6,sums7,sums8,sums9,sums10,sums11,sums12,sums13,sums14,sums15,sums16,sums17,sums18,sums19,sums20,sums21,sums22,sums23,sums24,sums25,sums26,sums27,sums28,sums29,sums30,sums31)
	for (uint y = blockIndex; y < size.y; y += 8) {
		for (uint x = 0; x < size.x; x++) {

			const TrackData & row = J[(x + y * Jsize.x)]; // ...
			if (row.result < 1) {
				// accesses sums[28..31]
				/*(sums+28)[1]*/sums29 += row.result == -4 ? 1 : 0;
				/*(sums+28)[2]*/sums30 += row.result == -5 ? 1 : 0;
				/*(sums+28)[3]*/sums31 += row.result > -4 ? 1 : 0;

				continue;
			}
			// Error part
			/*sums[0]*/sums0 += row.error * row.error;

			// JTe part
			/*for(int i = 0; i < 6; ++i)
			 sums[i+1] += row.error * row.J[i];*/
			sums1 += row.error * row.J[0];
			sums2 += row.error * row.J[1];
			sums3 += row.error * row.J[2];
			sums4 += row.error * row.J[3];
			sums5 += row.error * row.J[4];
			sums6 += row.error * row.J[5];

			// JTJ part, unfortunatly the double loop is not unrolled well...
			/*(sums+7)[0]*/sums7 += row.J[0] * row.J[0];
			/*(sums+7)[1]*/sums8 += row.J[0] * row.J[1];
			/*(sums+7)[2]*/sums9 += row.J[0] * row.J[2];
			/*(sums+7)[3]*/sums10 += row.J[0] * row.J[3];

			/*(sums+7)[4]*/sums11 += row.J[0] * row.J[4];
			/*(sums+7)[5]*/sums12 += row.J[0] * row.J[5];

			/*(sums+7)[6]*/sums13 += row.J[1] * row.J[1];
			/*(sums+7)[7]*/sums14 += row.J[1] * row.J[2];
			/*(sums+7)[8]*/sums15 += row.J[1] * row.J[3];
			/*(sums+7)[9]*/sums16 += row.J[1] * row.J[4];

			/*(sums+7)[10]*/sums17 += row.J[1] * row.J[5];

			/*(sums+7)[11]*/sums18 += row.J[2] * row.J[2];
			/*(sums+7)[12]*/sums19 += row.J[2] * row.J[3];
			/*(sums+7)[13]*/sums20 += row.J[2] * row.J[4];
			/*(sums+7)[14]*/sums21 += row.J[2] * row.J[5];

			/*(sums+7)[15]*/sums22 += row.J[3] * row.J[3];
			/*(sums+7)[16]*/sums23 += row.J[3] * row.J[4];
			/*(sums+7)[17]*/sums24 += row.J[3] * row.J[5];

			/*(sums+7)[18]*/sums25 += row.J[4] * row.J[4];
			/*(sums+7)[19]*/sums26 += row.J[4] * row.J[5];

			/*(sums+7)[20]*/sums27 += row.J[5] * row.J[5];

			// extra info here
			/*(sums+28)[0]*/sums28 += 1;

		}
	}
	sums[0] = sums0;
	sums[1] = sums1;
	sums[2] = sums2;
	sums[3] = sums3;
	sums[4] = sums4;
	sums[5] = sums5;
	sums[6] = sums6;
	sums[7] = sums7;
	sums[8] = sums8;
	sums[9] = sums9;
	sums[10] = sums10;
	sums[11] = sums11;
	sums[12] = sums12;
	sums[13] = sums13;
	sums[14] = sums14;
	sums[15] = sums15;
	sums[16] = sums16;
	sums[17] = sums17;
	sums[18] = sums18;
	sums[19] = sums19;
	sums[20] = sums20;
	sums[21] = sums21;
	sums[22] = sums22;
	sums[23] = sums23;
	sums[24] = sums24;
	sums[25] = sums25;
	sums[26] = sums26;
	sums[27] = sums27;
	sums[28] = sums28;
	sums[29] = sums29;
	sums[30] = sums30;
	sums[31] = sums31;

}
void reduceKernel(float * out, TrackData* J, const uint2 Jsize,
		const uint2 size) {
	TICK();
	int blockIndex;
#ifdef OLDREDUCE
#pragma omp parallel for private (blockIndex)
#endif
	for (blockIndex = 0; blockIndex < 8; blockIndex++) {

#ifdef OLDREDUCE
		float S[112][32]; // this is for the final accumulation
		// we have 112 threads in a blockdim
		// and 8 blocks in a gridDim?
		// ie it was launched as <<<8,112>>>
		uint sline;// threadIndex.x
		float sums[32];

		for(int threadIndex = 0; threadIndex < 112; threadIndex++) {
			sline = threadIndex;
			float * jtj = sums+7;
			float * info = sums+28;
			for(uint i = 0; i < 32; ++i) sums[i] = 0;

			for(uint y = blockIndex; y < size.y; y += 8 /*gridDim.x*/) {
				for(uint x = sline; x < size.x; x += 112 /*blockDim.x*/) {
					const TrackData & row = J[(x + y * Jsize.x)]; // ...

					if(row.result < 1) {
						// accesses S[threadIndex][28..31]
						info[1] += row.result == -4 ? 1 : 0;
						info[2] += row.result == -5 ? 1 : 0;
						info[3] += row.result > -4 ? 1 : 0;
						continue;
					}
					// Error part
					sums[0] += row.error * row.error;

					// JTe part
					for(int i = 0; i < 6; ++i)
					sums[i+1] += row.error * row.J[i];

					// JTJ part, unfortunatly the double loop is not unrolled well...
					jtj[0] += row.J[0] * row.J[0];
					jtj[1] += row.J[0] * row.J[1];
					jtj[2] += row.J[0] * row.J[2];
					jtj[3] += row.J[0] * row.J[3];

					jtj[4] += row.J[0] * row.J[4];
					jtj[5] += row.J[0] * row.J[5];

					jtj[6] += row.J[1] * row.J[1];
					jtj[7] += row.J[1] * row.J[2];
					jtj[8] += row.J[1] * row.J[3];
					jtj[9] += row.J[1] * row.J[4];

					jtj[10] += row.J[1] * row.J[5];

					jtj[11] += row.J[2] * row.J[2];
					jtj[12] += row.J[2] * row.J[3];
					jtj[13] += row.J[2] * row.J[4];
					jtj[14] += row.J[2] * row.J[5];

					jtj[15] += row.J[3] * row.J[3];
					jtj[16] += row.J[3] * row.J[4];
					jtj[17] += row.J[3] * row.J[5];

					jtj[18] += row.J[4] * row.J[4];
					jtj[19] += row.J[4] * row.J[5];

					jtj[20] += row.J[5] * row.J[5];

					// extra info here
					info[0] += 1;
				}
			}

			for(int i = 0; i < 32; ++i) { // copy over to shared memory
				S[sline][i] = sums[i];
			}
			// WE NO LONGER NEED TO DO THIS AS the threads execute sequentially inside a for loop

		} // threads now execute as a for loop.
		  //so the __syncthreads() is irrelevant

		for(int ssline = 0; ssline < 32; ssline++) { // sum up columns and copy to global memory in the final 32 threads
			for(unsigned i = 1; i < 112 /*blockDim.x*/; ++i) {
				S[0][ssline] += S[i][ssline];
			}
			out[ssline+blockIndex*32] = S[0][ssline];
		}
#else 
		new_reduce(blockIndex, out, J, Jsize, size);
#endif

	}

	TooN::Matrix<8, 32, float, TooN::Reference::RowMajor> values(out);
	for (int j = 1; j < 8; ++j) {
		values[0] += values[j];
		//std::cerr << "REDUCE ";for(int ii = 0; ii < 32;ii++)
		//std::cerr << values[0][ii] << " ";
		//std::cerr << "\n";
	}
	TOCK("reduceKernel", 512);
}

void trackKernel(TrackData* output, 
    const se::Image<Eigen::Vector3f>& inVertex,
		const se::Image<Eigen::Vector3f>& inNormal, 
    const se::Image<Eigen::Vector3f>&  refVertex,
		const se::Image<Eigen::Vector3f>& refNormal, 
    const Eigen::Matrix4f& Ttrack,
		const Eigen::Matrix4f& view, 
    const float dist_threshold,
		const float normal_threshold) {
	TICK();
	Eigen::Vector2i   pixel(0, 0);
  Eigen::Vector2i  inSize( inVertex.width(),  inVertex.height());
  Eigen::Vector2i refSize(refVertex.width(), refVertex.height());

	int pixely, pixelx;
#pragma omp parallel for \
	    shared(output), private(pixel,pixelx,pixely)
	for (pixely = 0; pixely < inSize.y(); pixely++) {
		for (pixelx = 0; pixelx < inSize.x(); pixelx++) {
			pixel.x() = pixelx;
			pixel.y() = pixely;

			TrackData & row = output[pixel.x() + pixel.y() * refSize.x()];

			if (inNormal[pixel.x() + pixel.y() * inSize.x()].x() == INVALID) {
				row.result = -1;
				continue;
			}

			const Eigen::Vector3f projectedVertex = (Ttrack * 
          inVertex[pixel.x() + pixel.y() * inSize.x()].homogeneous()).head<3>();
			const Eigen::Vector3f projectedPos = view * projectedVertex;
			const Eigen::Vector2f projPixel(
					projectedPos.x() / projectedPos.z() + 0.5f,
					projectedPos.y() / projectedPos.z() + 0.5f);
			if (projPixel.x() < 0 || projPixel.x() > refSize.x() - 1
					|| projPixel.y() < 0 || projPixel.y() > refSize.y() - 1) {
				row.result = -2;
				continue;
			}

			const Eigen::Vector2i refPixel = projPixel.cast<int>();
			const Eigen::Vector3f referenceNormal = refNormal[refPixel.x()
					+ refPixel.y() * refSize.x()];

			if (referenceNormal.x() == INVALID) {
				row.result = -3;
				continue;
			}

			const Eigen::Vector3f diff = refVertex[refPixel.x() + refPixel.y() * refSize.x()]
					- projectedVertex;
			const Eigen::Vector3f projectedNormal = Ttrack.topLeftCorner<3, 3>() * 
					inNormal[pixel.x() + pixel.y() * inSize.x()];

			if (diff.norm() > dist_threshold) {
				row.result = -4;
				continue;
			}
			if (projectedNormal.dot(referenceNormal) < normal_threshold) {
				row.result = -5;
				continue;
			}
			row.result = 1;
			row.error = referenceNormal.dot(diff);
			row.J[0] = referenceNormal.x();
			row.J[1] = referenceNormal.y();
			row.J[2] = referenceNormal.z();

      Eigen::Vector3f crossRes = projectedVertex.cross(referenceNormal);
			row.J[3] = crossRes.x();
			row.J[4] = crossRes.y();
			row.J[5] = crossRes.z();
		}
	}
	TOCK("trackKernel", inSize.x * inSize.y);
}

bool updatePoseKernel(Matrix4 & pose, const float * output,
		float icp_threshold) {
	bool res = false;
	TICK();
	// Update the pose regarding the tracking result
	TooN::Matrix<8, 32, const float, TooN::Reference::RowMajor> values(output);
	TooN::Vector<6> x = solve(values[0].slice<1, 27>());
	TooN::SE3<> delta(x);
	pose = toMatrix4(delta) * pose;

	// Return validity test result of the tracking
	if (norm(x) < icp_threshold)
		res = true;

	TOCK("updatePoseKernel", 1);
	return res;
}

bool checkPoseKernel(Matrix4 & pose, Matrix4 oldPose, const float * output,
		uint2 imageSize, float track_threshold) {

	// Check the tracking result, and go back to the previous camera position if necessary

	TooN::Matrix<8, 32, const float, TooN::Reference::RowMajor> values(output);

	if ((std::sqrt(values(0, 0) / values(0, 28)) > 2e-2)
			|| (values(0, 28) / (imageSize.x * imageSize.y) < track_threshold)) {
		pose = oldPose;
		return false;
	} else {
		return true;
	}

}


