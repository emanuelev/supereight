#include "projective_update.hpp"

#include "../common/field_impls.hpp"

#include <supereight/algorithms/filter.hpp>
#include <supereight/functors/data_handler.hpp>
#include <supereight/functors/data_handler_cuda.hpp>
#include <supereight/utils/cuda_util.hpp>

#include <cub/cub.cuh>
#include <cuda_runtime.h>

namespace se {

template<typename OctreeT>
__global__ static void updateBlockActiveKernel(OctreeT octree, Sophus::SE3f Tcw,
    Eigen::Matrix4f K, Eigen::Vector2i frame_size, int max_idx) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= max_idx) return;

    auto* block = octree.getBlockBuffer()[idx];
    if (!block->active()) {
        block->active(algorithms::in_frustum<OctreeT::block_type>(
            block, octree.voxelSize(), K * Tcw.matrix(), frame_size));
    }
}

template<typename OctreeT, typename UpdateFuncT>
__global__ static void // __launch_bounds__(64, 16)
    updateBlocksKernel(OctreeT octree, UpdateFuncT func, const Sophus::SE3f Tcw,
        const Eigen::Matrix4f K, const Eigen::Vector3f pos_delta,
        const Eigen::Vector3f camera_delta, const Eigen::Vector2i frame_size) {
    auto* block = octree.getBlockBuffer()[blockIdx.x];

    if (!block->active() /* &&
        !algorithms::in_frustum<OctreeT::block_type>(
            block, octree.voxelSize(), K * Tcw.matrix(), frame_size)*/)
        return;

    typedef cub::BlockReduce<int, 64> BlockReduce;
    __shared__ typename BlockReduce::TempStorage temp_storage;

    const Eigen::Vector3i block_coord = block->coordinates();

    const int x = threadIdx.x + block_coord(0);
    const int y = threadIdx.y + block_coord(1);

    Eigen::Vector3i pix = Eigen::Vector3i(x, y, block_coord(2));
    Eigen::Vector3f pos = Tcw * (octree.voxelSize() * pix.cast<float>());
    Eigen::Vector3f camera_voxel = K.topLeftCorner<3, 3>() * pos;

    int num_visible = 0;

    for (; pix(2) < block_coord(2) + BLOCK_SIDE; ++pix(2)) {
        if (pos(2) >= 0.0001f) {
            const float inverse_depth = 1.f / camera_voxel(2);
            const Eigen::Vector2f pixel =
                Eigen::Vector2f(camera_voxel(0) * inverse_depth + 0.5f,
                    camera_voxel(1) * inverse_depth + 0.5f);

            if (pixel(0) >= 0.5f && pixel(0) <= frame_size(0) - 1.5f &&
                pixel(1) >= 0.5f && pixel(1) <= frame_size(1) - 1.5f) {
                num_visible++;

                VoxelBlockHandlerCUDA<FieldType> handler = {block, pix};
                func(handler, pix, pos, pixel);
            }
        }

        pos += pos_delta;
        camera_voxel += camera_delta;
    }

    int num_active = BlockReduce(temp_storage).Sum(num_visible, 64);
    if (threadIdx.x == 0 && threadIdx.y == 0) { block->active(num_active > 0); }
}

template<typename OctreeT, typename UpdateFuncT>
__global__ static void updateNodesKernel(OctreeT octree, UpdateFuncT func,
    Sophus::SE3f Tcw, Eigen::Matrix4f K, Eigen::Vector2i frame_size,
    int maxIdx) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= maxIdx) return;

    auto node_buffer = octree.getNodesBuffer();
    auto* node       = node_buffer[idx];

    const Eigen::Vector3i voxel = Eigen::Vector3i(unpack_morton(node->code_));
    const Eigen::Vector3f delta = Tcw.rotationMatrix() *
        Eigen::Vector3f::Constant(0.5f * octree.voxelSize() * node->side_);

    const Eigen::Vector3f delta_c = K.topLeftCorner<3, 3>() * delta;

    Eigen::Vector3f base_cam = Tcw * (octree.voxelSize() * voxel.cast<float>());
    Eigen::Vector3f basepix_hom = K.topLeftCorner<3, 3>() * base_cam;

    for (int i = 0; i < 8; ++i) {
        const Eigen::Vector3i dir =
            Eigen::Vector3i((i & 1) > 0, (i & 2) > 0, (i & 4) > 0);
        const Eigen::Vector3f vox_cam =
            base_cam + dir.cast<float>().cwiseProduct(delta);
        const Eigen::Vector3f pix_hom =
            basepix_hom + dir.cast<float>().cwiseProduct(delta_c);

        if (vox_cam(2) < 0.0001f) continue;
        const float inverse_depth = 1.f / pix_hom(2);
        const Eigen::Vector2f pixel =
            Eigen::Vector2f(pix_hom(0) * inverse_depth + 0.5f,
                pix_hom(1) * inverse_depth + 0.5f);
        if (pixel(0) < 0.5f || pixel(0) > frame_size(0) - 1.5f ||
            pixel(1) < 0.5f || pixel(1) > frame_size(1) - 1.5f)
            continue;

        NodeHandler<FieldType> handler = {node, i};
        func(handler, voxel + dir, vox_cam, pixel);
    }
}

static void updateBlocks(Octree<FieldType, MemoryPoolCUDA>& octree,
    voxel_traits<FieldType>::update_func_type& func, Sophus::SE3f Tcw,
    Eigen::Matrix4f K, Eigen::Vector2i frame_size) {
    auto& block_buffer = octree.getBlockBuffer();
    int num_elem       = block_buffer.used();
    if (num_elem < 1) return;

    dim3 threads(BLOCK_SIDE, BLOCK_SIDE);
    dim3 blocks(num_elem);

    updateBlockActiveKernel<<<(num_elem + 255) / 256, 256>>>(
        octree, Tcw, K, frame_size, num_elem);
    safeCall(cudaPeekAtLastError());

    const Eigen::Vector3f pos_delta =
        Tcw.rotationMatrix() * Eigen::Vector3f(0, 0, octree.voxelSize());
    const Eigen::Vector3f camera_delta = K.topLeftCorner<3, 3>() * pos_delta;

    updateBlocksKernel<<<blocks, threads>>>(
        octree, func, Tcw, K, pos_delta, camera_delta, frame_size);
    safeCall(cudaPeekAtLastError());
}

static void updateNodes(Octree<FieldType, MemoryPoolCUDA>& octree,
    voxel_traits<FieldType>::update_func_type& func, Sophus::SE3f Tcw,
    Eigen::Matrix4f K, Eigen::Vector2i frame_size) {
    auto& node_buffer = octree.getNodesBuffer();
    int num_elem      = node_buffer.used();

    if (num_elem < 1) return;

    updateNodesKernel<<<(num_elem + 255) / 256, 256>>>(
        octree, func, Tcw, K, frame_size, num_elem);
    safeCall(cudaPeekAtLastError());
}

void projectiveUpdate(Octree<FieldType, MemoryPoolCUDA>& octree,
    voxel_traits<FieldType>::update_func_type& func, Sophus::SE3f Tcw,
    Eigen::Matrix4f K, Eigen::Vector2i frame_size) {
    updateBlocks(octree, func, Tcw, K, frame_size);
    updateNodes(octree, func, Tcw, K, frame_size);
}

} // namespace se
