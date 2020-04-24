#pragma once

#include <supereight/voxel_traits.hpp>

#include <Eigen/Dense>

namespace se {

struct OFusion {
    float x;
    double y;
};

struct bfusion_update {
    const float* depth;

    Eigen::Vector2i depthSize;

    float noiseFactor;
    float timestamp;
    float voxelsize;

    bfusion_update(const float* d, const Eigen::Vector2i& framesize, float n,
        float t, float vs);

    template<typename DataHandlerT>
    void operator()(DataHandlerT& handler, const Eigen::Vector3i&,
        const Eigen::Vector3f& pos, const Eigen::Vector2f& pixel);
};

template<>
struct voxel_traits<OFusion> {
    using value_type       = OFusion;
    using update_func_type = bfusion_update;

    static constexpr bool invert_normals = false;

    static inline value_type empty() { return {0.f, 0.f}; }
    static inline value_type initValue() { return {0.f, 0.f}; }

    template<typename OctreeT, typename HashType>
    static void buildAllocationList(HashType* allocation_list, int reserved,
        std::atomic<int>& voxel_count, const OctreeT& octree,
        const Eigen::Vector3f& world_vertex, const Eigen::Vector3f& direction,
        const Eigen::Vector3f& camera_pos, float depth_sample, int max_depth,
        int block_depth, float voxel_size, float inverse_voxel_size,
        float noise_factor);

    template<typename OctreeT>
    static Eigen::Vector4f raycast(const OctreeT& octree,
        const Eigen::Vector3f& origin, const Eigen::Vector3f& direction,
        const float tnear, const float tfar, const float, const float step,
        const float);
};

} // namespace se
