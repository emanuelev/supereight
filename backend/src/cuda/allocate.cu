#include "../common/field_impls.hpp"
#include "allocate.hpp"

#include <supereight/algorithms/unique.hpp>
#include <supereight/utils/cuda_util.hpp>

#include <cub/cub.cuh>
#include <cuda_runtime.h>

#include <chrono>

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

    const float depth_sample = depth[x + y * frame_size.x()];
    if (depth_sample == 0) return;

    const Eigen::Vector3f camera_pos = pose.topRightCorner<3, 1>();
    Eigen::Vector3f world_vertex     = (inv_P *
        Eigen::Vector3f(
            (x + 0.5f) * depth_sample, (y + 0.5f) * depth_sample, depth_sample)
            .homogeneous())
                                       .head<3>();
    Eigen::Vector3f direction = (camera_pos - world_vertex).normalized();

    auto get_idx = [=]() { return atomicAdd(voxel_count, 1); };
    OctreeT::traits_type::buildAllocationList(allocation_list.data(),
        allocation_list.size(), get_idx, octree, world_vertex, direction,
        camera_pos, depth_sample, mu);
}

__global__ static void keysToLevelKernel(BufferAccessorCUDA<se::key_t> out,
    BufferAccessorCUDA<se::key_t> in, int num_elem, int level, int max_level) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_elem) return;

    se::key_t key = in[idx];
    int key_level = keyops::level(key);

    unsigned shift = MAX_BITS - max_level - 1;
    se::key_t mask = MASK[level + shift];

    out[idx] = (key & mask & ~SCALE_MASK) | min(key_level, level);
}

template<typename OctreeT>
__global__ static void allocateLevelKernel(OctreeT octree,
    BufferAccessorCUDA<se::key_t> keys_at_level, int num_elem, int level) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_elem) return;

    se::key_t key = keys_at_level[idx];
    int key_level = keyops::level(key);

    if (key_level < level) return;

    octree.insert_one(keys_at_level[idx], level);
}

template<typename OctreeT>
__global__ static void allocateSequentialKernel(OctreeT octree,
    BufferAccessorCUDA<se::key_t> allocation_list, int num_elem) {
    for (int i = 0; i < num_elem; ++i) {
        se::key_t key = allocation_list[i];

        int level           = keyops::level(key);
        Eigen::Vector3i pos = keyops::decode(key);

        octree.insert(pos.x(), pos.y(), pos.z(), level);
    }
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

    int final_count;
    cudaMemcpy(&final_count, voxel_count, sizeof(int), cudaMemcpyDeviceToHost);

    int reserved = allocation_list.size();
    return final_count >= reserved ? reserved : final_count;
}

int filterAllocationList(BufferAccessorCUDA<se::key_t> allocation_list,
    int allocation_list_used, int* num_unique_device,
    BufferCUDA<std::uint8_t>& temp_storage) {
    std::size_t temp_storage_bytes = 0;

    // Calculate temp storage requirements
    cub::DeviceRadixSort::SortKeys(nullptr, temp_storage_bytes,
        allocation_list.data(), allocation_list.data(), allocation_list_used);
    safeCall(cudaPeekAtLastError());

    if (temp_storage_bytes > temp_storage.size())
        temp_storage.resize(temp_storage_bytes);

    // Sort
    cub::DeviceRadixSort::SortKeys(
        static_cast<void*>(temp_storage.accessor().data()), temp_storage_bytes,
        allocation_list.data(), allocation_list.data(), allocation_list_used);
    safeCall(cudaPeekAtLastError());

    // Calculate temp storage requirements
    cub::DeviceSelect::Unique(nullptr, temp_storage_bytes,
        allocation_list.data(), allocation_list.data(), num_unique_device,
        allocation_list_used);
    safeCall(cudaPeekAtLastError());

    if (temp_storage_bytes > temp_storage.size())
        temp_storage.resize(temp_storage_bytes);

    // Unique
    cub::DeviceSelect::Unique(temp_storage.accessor().data(),
        temp_storage_bytes, allocation_list.data(), allocation_list.data(),
        num_unique_device, allocation_list_used);
    safeCall(cudaPeekAtLastError());

    int num_unique;
    safeCall(cudaMemcpy(
        &num_unique, num_unique_device, sizeof(int), cudaMemcpyDeviceToHost));

    return num_unique;
}

void allocateParallel(Octree<FieldType, MemoryPoolCUDA>& octree,
    BufferAccessorCUDA<se::key_t> allocation_list, int num_unique,
    BufferCUDA<se::key_t>& keys_at_level, int* keys_at_level_used,
    BufferCUDA<std::uint8_t>& temp_storage) {
    auto& node_buffer  = octree.getNodesBuffer();
    auto& block_buffer = octree.getBlockBuffer();

    std::size_t node_buffer_used  = node_buffer.used();
    std::size_t block_buffer_used = block_buffer.used();

    if (static_cast<std::size_t>(num_unique) > keys_at_level.size())
        keys_at_level.resize(num_unique);

    std::size_t temp_storage_bytes = 0;
    for (int level = 1; level <= octree.blockDepth(); ++level) {
        constexpr int thread_dim_ktl = 256;

        dim3 threads_ktl(thread_dim_ktl);
        dim3 blocks_ktl((num_unique + thread_dim_ktl - 1) / thread_dim_ktl);

        keysToLevelKernel<<<blocks_ktl, threads_ktl>>>(keys_at_level.accessor(),
            allocation_list, num_unique, level, octree.maxDepth());
        safeCall(cudaPeekAtLastError());

        // Calculate temp storage requirements
        cub::DeviceSelect::Unique(nullptr, temp_storage_bytes,
            keys_at_level.accessor().data(), keys_at_level.accessor().data(),
            keys_at_level_used, num_unique);
        safeCall(cudaPeekAtLastError());

        if (temp_storage_bytes > temp_storage.size())
            temp_storage.resize(temp_storage_bytes);

        // Unique keys_at_level
        cub::DeviceSelect::Unique(temp_storage.accessor().data(),
            temp_storage_bytes, keys_at_level.accessor().data(),
            keys_at_level.accessor().data(), keys_at_level_used, num_unique);
        safeCall(cudaPeekAtLastError());

        int num_unique_at_level;
        safeCall(cudaMemcpy(&num_unique_at_level, keys_at_level_used,
            sizeof(int), cudaMemcpyDeviceToHost));
        if (num_unique_at_level == 0) continue;

        if (level < octree.blockDepth()) {
            node_buffer_used += num_unique_at_level;
            node_buffer.reserve(node_buffer_used);
        } else {
            block_buffer.reserve(block_buffer_used + num_unique_at_level);
        }

        constexpr int thread_dim_al = 256;

        dim3 threads_al(thread_dim_al);
        dim3 blocks_al(
            (num_unique_at_level + thread_dim_al - 1) / thread_dim_al);

        allocateLevelKernel<<<blocks_al, threads_al>>>(
            octree, keys_at_level.accessor(), num_unique_at_level, level);
        safeCall(cudaPeekAtLastError());
    }
}

void allocate(Octree<FieldType, MemoryPoolCUDA>& octree,
    BufferAccessorCUDA<se::key_t> allocation_list, int allocation_list_used,
    BufferCUDA<se::key_t>& keys_at_level, int* keys_at_level_used,
    BufferCUDA<std::uint8_t>& temp_storage) {
    if (allocation_list_used == 0) return;

    // Parameters found using linear regression on GTX 1080 Ti test platform
    constexpr int filter_threshold   = 25;
    constexpr int parallel_threshold = 56;

    int num_elem;
    if (allocation_list_used < filter_threshold) {
        num_elem = allocation_list_used;
    } else {
        num_elem = filterAllocationList(allocation_list, allocation_list_used,
            keys_at_level_used, temp_storage);
    }

    if (num_elem < parallel_threshold) {
        auto& node_buffer  = octree.getNodesBuffer();
        auto& block_buffer = octree.getBlockBuffer();

        std::size_t node_buffer_used  = node_buffer.used();
        std::size_t block_buffer_used = block_buffer.used();

        node_buffer.reserve(node_buffer_used + (num_elem * octree.maxDepth()));
        block_buffer.reserve(block_buffer_used + num_elem);

        allocateSequentialKernel<<<1, 1>>>(octree, allocation_list, num_elem);
        safeCall(cudaPeekAtLastError());
    } else {
        allocateParallel(octree, allocation_list, num_elem, keys_at_level,
            keys_at_level_used, temp_storage);
    }
}

} // namespace se
