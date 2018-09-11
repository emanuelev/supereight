#ifndef RENDERINGH
#define RENDERINGH

#include <math_utils.h>
#include <se/continuous/volume_template.hpp>
#include <se/commons.h>

template<typename T>
void raycastKernel(const Volume<T>& volume, float3* vertex, float3* normal,
                   uint2 inputSize, const Matrix4 view, const float nearPlane,
                   const float farPlane, const float mu, const float step,
                   const float largestep);

void renderNormalKernel(uchar3* out, const float3* normal, uint normalSize);

void renderDepthKernel(uchar4* out, float* depth, uint2 depthSIze,
                       const float nearPlane, const float farPlane);

void renderTrackKernel(uchar4* out, const TrackData* data, uint2 outSize);

template <typename T>
void renderVolumeKernel(const Volume<T>& volume, uchar4* out,
                        const uint2 depthSize, const Matrix4 view,
                        const float newaPlane, const float farPlane,
                        const float mu, const float step, const float largestep,
                        const float3 light, const float3 ambient, bool render,
                        const float3* vertex, const float3* normal);

#endif //RENDERINGH
