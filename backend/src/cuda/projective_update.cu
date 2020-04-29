#include "../common/field_impls.hpp"
#include "projective_update.hpp"

#include <supereight/algorithms/filter.hpp>
#include <supereight/backend/cuda_util.hpp>
#include <supereight/functors/data_handler.hpp>

#include <cuda_runtime.h>

namespace se {

template<typename OctreeT, typename UpdateFuncT>
__global__ static void updateBlocksKernel(OctreeT octree, UpdateFuncT func,
    Sophus::SE3f Tcw, Eigen::Matrix4f K, Eigen::Vector2i frame_size,
    int maxIdx) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= maxIdx) return;

    auto block_buffer = octree.getBlockBuffer();
    auto* block       = block_buffer[idx];

    float voxel_size = octree.dim() / octree.size();

    if (!block->active() &&
        !algorithms::in_frustum<OctreeT::block_type>(
            block, voxel_size, K * Tcw.matrix(), frame_size))
        return;

    const Eigen::Vector3i blockCoord = block->coordinates();
    const Eigen::Vector3f delta =
        Tcw.rotationMatrix() * Eigen::Vector3f(voxel_size, 0, 0);
    const Eigen::Vector3f cameraDelta = K.topLeftCorner<3, 3>() * delta;
    bool is_visible                   = false;

    unsigned int y, z;
    unsigned int ylast = blockCoord(1) + BLOCK_SIDE;
    unsigned int zlast = blockCoord(2) + BLOCK_SIDE;

    for (z = blockCoord(2); z < zlast; ++z) {
        for (y = blockCoord(1); y < ylast; ++y) {
            Eigen::Vector3i pix   = Eigen::Vector3i(blockCoord(0), y, z);
            Eigen::Vector3f start = Tcw *
                Eigen::Vector3f((pix(0)) * voxel_size, (pix(1)) * voxel_size,
                    (pix(2)) * voxel_size);
            Eigen::Vector3f camerastart = K.topLeftCorner<3, 3>() * start;
            for (unsigned int x = 0; x < BLOCK_SIDE; ++x) {
                pix(0) = x + blockCoord(0);
                const Eigen::Vector3f camera_voxel =
                    camerastart + (x * cameraDelta);
                const Eigen::Vector3f pos = start + (x * delta);
                if (pos(2) < 0.0001f) continue;

                const float inverse_depth = 1.f / camera_voxel(2);
                const Eigen::Vector2f pixel =
                    Eigen::Vector2f(camera_voxel(0) * inverse_depth + 0.5f,
                        camera_voxel(1) * inverse_depth + 0.5f);
                if (pixel(0) < 0.5f || pixel(0) > frame_size(0) - 1.5f ||
                    pixel(1) < 0.5f || pixel(1) > frame_size(1) - 1.5f)
                    continue;
                is_visible = true;

                VoxelBlockHandler<typename OctreeT::value_type> handler = {
                    block, pix};
                func(handler, pix, pos, pixel);
            }
        }
    }

    block->active(is_visible);
}

template<typename OctreeT, typename UpdateFuncT>
__global__ static void updateNodesKernel(OctreeT octree, UpdateFuncT func,
    Sophus::SE3f Tcw, Eigen::Matrix4f K, Eigen::Vector2i frame_size,
    int maxIdx) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= maxIdx) return;

    auto node_buffer = octree.getNodesBuffer();
    auto* node       = node_buffer[idx];

    float voxel_size = octree.dim() / octree.size();

    const Eigen::Vector3i voxel = Eigen::Vector3i(unpack_morton(node->code_));
    const Eigen::Vector3f delta = Tcw.rotationMatrix() *
        Eigen::Vector3f::Constant(0.5f * voxel_size * node->side_);

    const Eigen::Vector3f delta_c = K.topLeftCorner<3, 3>() * delta;

    Eigen::Vector3f base_cam    = Tcw * (voxel_size * voxel.cast<float>());
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

    updateBlocksKernel<<<(num_elem + 255) / 256, 256>>>(
        octree, func, Tcw, K, frame_size, num_elem);
    safeCall(cudaPeekAtLastError());
}

static void updateNodes(Octree<FieldType, MemoryPoolCUDA>& octree,
    voxel_traits<FieldType>::update_func_type& func, Sophus::SE3f Tcw,
    Eigen::Matrix4f K, Eigen::Vector2i frame_size) {
    auto& node_buffer = octree.getNodesBuffer();
    int num_elem      = node_buffer.used();

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
