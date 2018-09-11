#ifndef PREPROCESSINGH
#define PREPROCESSINGH

#include <math_utils.h>

void bilateralFilterKernel(float* out, const float* in, uint2 size,
                           const float* gauusian, float e_d, int r);

void depth2vertexKernel(float3* vertex, const float* depth, uint2 imageSize,
                        const Matrix4 invK);

template <typename FieldType, bool NegY>
void vertex2normalKernel(float3* out, const float3* in, uint2 imageSize);

void mm2metersKernel(float* out, uint2 outSize, const ushort* in, uint2 inSize);

void halfSampleRobustImageKernel(float* out, const float* in, uint2 inSize,
                                 const float e_d, const int r);

#endif //PREPROCESSINGH
