/*
 Copyright (c) 2014 University of Edinburgh, Imperial College, University of
 Manchester. Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

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

#include <supereight/shared/timings.h>
#include <supereight/utils/math_utils.h>

#include <supereight/denseslam/commons.h>
#include <supereight/image/image.hpp>

static inline Eigen::Matrix<float, 6, 6> makeJTJ(
    const Eigen::Matrix<float, 1, 21>& v) {
    Eigen::Matrix<float, 6, 6> C = Eigen::Matrix<float, 6, 6>::Zero();
    C.row(0)                     = v.segment(0, 6);
    C.row(1).segment(1, 5)       = v.segment(6, 5);
    C.row(2).segment(2, 4)       = v.segment(11, 4);
    C.row(3).segment(3, 3)       = v.segment(15, 3);
    C.row(4).segment(4, 2)       = v.segment(18, 2);
    C(5, 5)                      = v(20);

    for (int r = 1; r < 6; ++r)
        for (int c = 0; c < r; ++c) C(r, c) = C(c, r);
    return C;
}

static inline Eigen::Matrix<float, 6, 1> solve(
    const Eigen::Matrix<float, 1, 27>& vals) {
    const Eigen::Matrix<float, 6, 1> b = vals.segment(0, 6);
    const Eigen::Matrix<float, 6, 6> C = makeJTJ(vals.segment(6, 21));
    Eigen::LLT<Eigen::Matrix<float, 6, 6>> llt;
    llt.compute(C);
    Eigen::Matrix<float, 6, 1> res = llt.solve(b);
    return llt.info() == Eigen::Success
        ? res
        : Eigen::Matrix<float, 6, 1>::Constant(0.f);
}

void new_reduce(int blockIndex, float* out, TrackData* J,
    const Eigen::Vector2i& Jsize, const Eigen::Vector2i& size) {
    float* sums = out + blockIndex * 32;

    for (unsigned int i = 0; i < 32; ++i) sums[i] = 0;

    float sums0  = 0.0f;
    float sums1  = 0.0f;
    float sums2  = 0.0f;
    float sums3  = 0.0f;
    float sums4  = 0.0f;
    float sums5  = 0.0f;
    float sums6  = 0.0f;
    float sums7  = 0.0f;
    float sums8  = 0.0f;
    float sums9  = 0.0f;
    float sums10 = 0.0f;
    float sums11 = 0.0f;
    float sums12 = 0.0f;
    float sums13 = 0.0f;
    float sums14 = 0.0f;
    float sums15 = 0.0f;
    float sums16 = 0.0f;
    float sums17 = 0.0f;
    float sums18 = 0.0f;
    float sums19 = 0.0f;
    float sums20 = 0.0f;
    float sums21 = 0.0f;
    float sums22 = 0.0f;
    float sums23 = 0.0f;
    float sums24 = 0.0f;
    float sums25 = 0.0f;
    float sums26 = 0.0f;
    float sums27 = 0.0f;
    float sums28 = 0.0f;
    float sums29 = 0.0f;
    float sums30 = 0.0f;
    float sums31 = 0.0f;

// comment me out to try coarse grain parallelism
#pragma omp parallel for reduction(+:sums0,sums1,sums2,sums3,sums4,sums5,      \
        sums6,sums7,sums8,sums9,sums10,sums11,sums12,sums13,sums14,sums15,     \
        sums16,sums17,sums18,sums19,sums20,sums21,sums22,sums23,sums24,sums25, \
        sums26,sums27,sums28,sums29,sums30,sums31)

    for (int y = blockIndex; y < size.y(); y += 8) {
        for (int x = 0; x < size.x(); x++) {
            const TrackData& row = J[(x + y * Jsize.x())]; // ...
            if (row.result < 1) {
                // accesses sums[28..31]
                /*(sums+28)[1]*/ sums29 += row.result == -4 ? 1 : 0;
                /*(sums+28)[2]*/ sums30 += row.result == -5 ? 1 : 0;
                /*(sums+28)[3]*/ sums31 += row.result > -4 ? 1 : 0;

                continue;
            }
            // Error part
            /*sums[0]*/ sums0 += row.error * row.error;

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
            /*(sums+7)[0]*/ sums7 += row.J[0] * row.J[0];
            /*(sums+7)[1]*/ sums8 += row.J[0] * row.J[1];
            /*(sums+7)[2]*/ sums9 += row.J[0] * row.J[2];
            /*(sums+7)[3]*/ sums10 += row.J[0] * row.J[3];

            /*(sums+7)[4]*/ sums11 += row.J[0] * row.J[4];
            /*(sums+7)[5]*/ sums12 += row.J[0] * row.J[5];

            /*(sums+7)[6]*/ sums13 += row.J[1] * row.J[1];
            /*(sums+7)[7]*/ sums14 += row.J[1] * row.J[2];
            /*(sums+7)[8]*/ sums15 += row.J[1] * row.J[3];
            /*(sums+7)[9]*/ sums16 += row.J[1] * row.J[4];

            /*(sums+7)[10]*/ sums17 += row.J[1] * row.J[5];

            /*(sums+7)[11]*/ sums18 += row.J[2] * row.J[2];
            /*(sums+7)[12]*/ sums19 += row.J[2] * row.J[3];
            /*(sums+7)[13]*/ sums20 += row.J[2] * row.J[4];
            /*(sums+7)[14]*/ sums21 += row.J[2] * row.J[5];

            /*(sums+7)[15]*/ sums22 += row.J[3] * row.J[3];
            /*(sums+7)[16]*/ sums23 += row.J[3] * row.J[4];
            /*(sums+7)[17]*/ sums24 += row.J[3] * row.J[5];

            /*(sums+7)[18]*/ sums25 += row.J[4] * row.J[4];
            /*(sums+7)[19]*/ sums26 += row.J[4] * row.J[5];

            /*(sums+7)[20]*/ sums27 += row.J[5] * row.J[5];

            // extra info here
            /*(sums+28)[0]*/ sums28 += 1;
        }
    }

    sums[0]  = sums0;
    sums[1]  = sums1;
    sums[2]  = sums2;
    sums[3]  = sums3;
    sums[4]  = sums4;
    sums[5]  = sums5;
    sums[6]  = sums6;
    sums[7]  = sums7;
    sums[8]  = sums8;
    sums[9]  = sums9;
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

void reduceKernel(float* out, TrackData* J, const Eigen::Vector2i& Jsize,
    const Eigen::Vector2i& size) {
    TICK();

    for (int blockIndex = 0; blockIndex < 8; blockIndex++) {
        new_reduce(blockIndex, out, J, Jsize, size);
    }

    Eigen::Map<Eigen::Matrix<float, 8, 32, Eigen::RowMajor>> values(out);
    for (int j = 1; j < 8; ++j) { values.row(0) += values.row(j); }

    TOCK("reduceKernel", 512);
}

void trackKernel(TrackData* output, const se::Image<Eigen::Vector3f>& inVertex,
    const se::Image<Eigen::Vector3f>& inNormal,
    const se::Image<Eigen::Vector3f>& refVertex,
    const se::Image<Eigen::Vector3f>& refNormal, const Eigen::Matrix4f& Ttrack,
    const Eigen::Matrix4f& view, const float dist_threshold,
    const float normal_threshold) {
    TICK();

    Eigen::Vector2i pixel(0, 0);
    Eigen::Vector2i inSize(inVertex.width(), inVertex.height());
    Eigen::Vector2i refSize(refVertex.width(), refVertex.height());

    int pixely, pixelx;

#pragma omp parallel for shared(output), private(pixel, pixelx, pixely)
    for (pixely = 0; pixely < inSize.y(); pixely++) {
        for (pixelx = 0; pixelx < inSize.x(); pixelx++) {
            pixel.x() = pixelx;
            pixel.y() = pixely;

            TrackData& row = output[pixel.x() + pixel.y() * refSize.x()];

            if (inNormal[pixel.x() + pixel.y() * inSize.x()].x() == INVALID) {
                row.result = -1;
                continue;
            }

            const Eigen::Vector3f projectedVertex = (Ttrack *
                inVertex[pixel.x() + pixel.y() * inSize.x()].homogeneous())
                                                        .head<3>();
            const Eigen::Vector3f projectedPos =
                (view * projectedVertex.homogeneous()).head<3>();

            const Eigen::Vector2f projPixel(
                projectedPos.x() / projectedPos.z() + 0.5f,
                projectedPos.y() / projectedPos.z() + 0.5f);

            if (projPixel.x() < 0 || projPixel.x() > refSize.x() - 1 ||
                projPixel.y() < 0 || projPixel.y() > refSize.y() - 1) {
                row.result = -2;
                continue;
            }

            const Eigen::Vector2i refPixel = projPixel.cast<int>();
            const Eigen::Vector3f referenceNormal =
                refNormal[refPixel.x() + refPixel.y() * refSize.x()];

            if (referenceNormal.x() == INVALID) {
                row.result = -3;
                continue;
            }

            const Eigen::Vector3f diff =
                refVertex[refPixel.x() + refPixel.y() * refSize.x()] -
                projectedVertex;

            const Eigen::Vector3f projectedNormal =
                Ttrack.topLeftCorner<3, 3>() *
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
            row.error  = referenceNormal.dot(diff);

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

bool updatePoseKernel(
    Eigen::Matrix4f& pose, const float* output, float icp_threshold) {
    TICK();

    bool res = false;

    Eigen::Map<const Eigen::Matrix<float, 8, 32, Eigen::RowMajor>> values(
        output);

    Eigen::Matrix<float, 6, 1> x = solve(values.row(0).segment(1, 27));
    Eigen::Matrix4f delta        = Sophus::SE3<float>::exp(x).matrix();
    pose                         = delta * pose;

    if (x.norm() < icp_threshold) res = true;

    TOCK("updatePoseKernel", 1);

    return res;
}

bool checkPoseKernel(Eigen::Matrix4f& pose, Eigen::Matrix4f& oldPose,
    const float* output, const Eigen::Vector2i& imageSize,
    float track_threshold) {
    // Check the tracking result, and go back to the previous camera position if
    // necessary

    const Eigen::Matrix<float, 8, 32, Eigen::RowMajor> values(output);

    if ((std::sqrt(values(0, 0) / values(0, 28)) > 2e-2) ||
        (values(0, 28) / (imageSize.x() * imageSize.y()) < track_threshold)) {
        pose = oldPose;
        return false;
    }

    return true;
}
