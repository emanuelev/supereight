#include "../common/field_impls.hpp"
#include "allocate.hpp"

#include <supereight/backend/cuda_util.hpp>

#include <cuda_runtime.h>

namespace se {

template<typename OctreeT>
__global__ static void buildAllocationListKernel(
    BufferAccessorCUDA<se::key_t> allocation_list, OctreeT octree,
    int* voxel_count, Eigen::Matrix4f pose, Eigen::Matrix4f K,
    BufferAccessorCUDA<float> depth, Eigen::Vector2i frame_size, float mu) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= frame_size.x() || y >= frame_size.y()) return;

    const Eigen::Matrix4f inv_P = pose * K.inverse();

    const int max_depth = log2(octree.size());
    const int block_depth =
        log2(octree.size()) - se::math::log2_const(OctreeT::blockSide);

    const float voxel_size         = octree.dim() / octree.size();
    const float inverse_voxel_size = 1.f / voxel_size;

    const float depth_sample = *depth[x + y * frame_size.x()];
    if (depth_sample == 0) return;

    const Eigen::Vector3f camera_pos = pose.topRightCorner<3, 1>();
    Eigen::Vector3f world_vertex     = (inv_P *
        Eigen::Vector3f(
            (x + 0.5f) * depth_sample, (y + 0.5f) * depth_sample, depth_sample)
            .homogeneous())
                                       .head<3>();
    Eigen::Vector3f direction = (camera_pos - world_vertex).normalized();

    auto get_idx = [=]() { return atomicAdd(voxel_count, 1); };
    voxel_traits<typename OctreeT::value_type>::buildAllocationList(
        allocation_list.data(), allocation_list.size(), get_idx, octree,
        world_vertex, direction, camera_pos, depth_sample, max_depth,
        block_depth, voxel_size, inverse_voxel_size, mu);
}

int buildAllocationList(BufferAccessorCUDA<se::key_t> allocation_list,
    const Octree<FieldType, MemoryPoolCUDA>& octree, int* voxel_count,
    const Eigen::Matrix4f& pose, const Eigen::Matrix4f& K,
    BufferAccessorCUDA<float> depth, const Eigen::Vector2i& frame_size,
    float mu) {
    constexpr int thread_dim = 16;

    dim3 threads(thread_dim, thread_dim);
    dim3 blocks((frame_size.x() + thread_dim - 1) / thread_dim,
        (frame_size.y() + thread_dim - 1) / thread_dim);

    cudaMemset(voxel_count, 0, sizeof(int));

    buildAllocationListKernel<<<blocks, threads>>>(
        allocation_list, octree, voxel_count, pose, K, depth, frame_size, mu);
    safeCall(cudaPeekAtLastError());
    safeCall(cudaDeviceSynchronize());

    int final_count = *voxel_count;
    int reserved    = allocation_list.size();
    return final_count >= reserved ? reserved : final_count;
}

} // namespace se
