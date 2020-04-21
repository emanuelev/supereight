#pragma once

#include <supereight/voxel_traits.hpp>

#include <Eigen/Dense>

namespace se {

struct SDF {
    float x;
    float y;
};

struct sdf_update {
    const float* depth;

    Eigen::Vector2i depthSize;

    float mu;
    int maxweight = 100;

    sdf_update(const float* d, const Eigen::Vector2i& framesize, float m, float,
        float);

    template<typename DataHandlerT>
    void operator()(DataHandlerT& handler, const Eigen::Vector3i&,
        const Eigen::Vector3f& pos, const Eigen::Vector2f& pixel);
};

template<>
struct voxel_traits<SDF> {
    using value_type       = SDF;
    using update_func_type = sdf_update;

    static constexpr bool invert_normals = true;

    static value_type empty() { return {1.f, -1.f}; }
    static value_type initValue() { return {1.f, 0.f}; }

    template<typename OctreeT, typename HashType>
    static int buildAllocationList(HashType* allocation_list, size_t reserved,
        const OctreeT& octree, const Eigen::Matrix4f& pose,
        const Eigen::Matrix4f& K, const float* depth_map,
        const Eigen::Vector2i& image_size, float mu);

    template<typename OctreeT>
    static Eigen::Vector4f raycast(const OctreeT& octree,
        const Eigen::Vector3f& origin, const Eigen::Vector3f& direction,
        const float tnear, const float tfar, const float mu, const float step,
        const float largestep);
};

} // namespace se
