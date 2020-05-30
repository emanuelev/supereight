#pragma once

#include <supereight/shared/commons.h>
#include <supereight/voxel_traits.hpp>

#include <Eigen/Dense>

#ifdef SE_CUDA_VF
#include <cuda_runtime.h>
#endif

namespace se {

struct SDF {};

struct sdf_update {
    const float* depth;

    Eigen::Vector2i depthSize;

    float mu;
    int maxweight = 100;

    SE_DEVICE_FUNC
    sdf_update(const float* d, const Eigen::Vector2i& framesize, float m, float,
        float);

    template<typename DataHandlerT>
    SE_DEVICE_FUNC void operator()(DataHandlerT& handler,
        const Eigen::Vector3i&, const Eigen::Vector3f& pos,
        const Eigen::Vector2f& pixel);
};

template<>
struct voxel_traits<SDF> {
#ifdef SE_CUDA_VF
    using value_type = float2;
#else
    using value_type = struct {
        float x;
        float y;
    };
#endif

    using update_func_type = sdf_update;

    static constexpr bool invert_normals = true;

    SE_DEVICE_FUNC
    static value_type empty() { return {1.f, -1.f}; }

    SE_DEVICE_FUNC
    static value_type initValue() { return {1.f, 0.f}; }

    template<typename OctreeT, typename HashType, typename IncF>
    SE_DEVICE_FUNC static void buildAllocationList(HashType* allocation_list,
        int reserved, IncF get_idx, const OctreeT& octree,
        const Eigen::Vector3f& world_vertex, const Eigen::Vector3f& direction,
        const Eigen::Vector3f&, float, float mu);

    template<typename OctreeT>
    SE_DEVICE_FUNC static Eigen::Vector4f raycast(const OctreeT& octree,
        const Eigen::Vector3f& origin, const Eigen::Vector3f& direction,
        const float tnear, const float tfar, const float mu, const float step,
        const float largestep);
};

} // namespace se
