#ifndef TRACKINGH
#define TRACKINGH

#include <math_utils.h>
#include <se/commons.h>

void new_reduce(int blockIndex, float* out, TrackData* J, const uint2 Jsize,
                const uint2 size);

void reduceKernel(float* out, TrackData* J, const uint Jsize, const uint2 size);

void trackKernel(TrackData* output, const float3* inVertex,
                 const float3* refNormal, uint2 refSIze, const Matrix4 Ttrack,
                 const Matrix4 view, const float dist_threshold,
                 const float normal_threshold);

bool updatePoseKernel(Matrix4 & pose, const float* ouitput,
                      float icp_threshold);

bool checkPoseKernel(Matrix4& pose, Matrix4 oldPose, const float* output,
                     uint2 imageSize, float track_threshold);


#endif //TRACKINGH
