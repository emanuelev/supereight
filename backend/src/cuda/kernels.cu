#include "kernels.hpp"

namespace se {

template<typename OctreeT, typename UpdateFuncT>
void updateBlocks(OctreeT octree, UpdateFuncT func, Sophus::SE3f Tcw,
    Eigen::Matrix4f K, Eigen::Vector2i frame_size) {
    auto& block_buffer = octree.getBlockBuffer();
    int num_elem       = block_buffer.used();
    updateBlocks<<<(num_elem + 255) / 256, 256>>>(
        octree, func, Tcw, K, frame_size, num_elem);
}

template<typename OctreeT, typename UpdateFuncT>
__global__ void updateBlocksKernel(OctreeT octree, UpdateFuncT func,
    Sophus::SE3f Tcw, Eigen::Matrix4f K, Eigen::Vector2i frame_size,
    int maxIdx) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= maxIdx) return;

    auto block_buffer = octree.getBlockBuffer();
    auto* block       = block_buffer[idx];

    float voxel_size = octree.dim() / octree.size();

    if (!b->active()) return;
    if (!algorithms::in_frustum<OctreeT::block_type>(
            block, voxel_size, K * Tcw, frame_size))
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

                VoxelBlockHandler<FieldType> handler = {block, pix};
                func(handler, pix, pos, pixel);
            }
        }
    }

    block->active(is_visible);
}

} // namespace se
