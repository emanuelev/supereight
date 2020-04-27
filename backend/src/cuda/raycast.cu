#include "../common/field_impls.hpp"
#include "raycast.hpp"
#include "util.hpp"

#include <supereight/ray_iterator.hpp>

#include <cuda_runtime.h>

namespace se {

template<typename OctreeT>
__global__ static void raycastKernel(OctreeT octree, Eigen::Vector3f* vertex,
    Eigen::Vector3f* normal, Eigen::Vector2i frame_size, Eigen::Matrix4f view,
    Eigen::Vector2f planes, float mu, float step) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= frame_size.x() || y >= frame_size.y()) return;

    const Eigen::Vector3f dir =
        (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
    const Eigen::Vector3f transl = view.topRightCorner<3, 1>();

    se::ray_iterator<FieldType, MemoryPoolCUDA> ray(
        octree, transl, dir, planes(0), planes(1));
    ray.next();

    const float t_min         = ray.tcmin();
    const Eigen::Vector4f hit = t_min > 0.f
        ? voxel_traits<FieldType>::raycast(octree, transl, dir, t_min,
              ray.tmax(), mu, step, step * BLOCK_SIDE)
        : Eigen::Vector4f::Constant(0.f);
/*
    if (x == 160 && y == 120) {
        printf("t_min = %f; hit = %f, %f, %f, %f\n",
                t_min, hit.x(), hit.y(), hit.z(), hit.w());
    }
*/

    if (hit.w() > 0.0) {
        vertex[x + y * frame_size.x()] = hit.head<3>();

        const float inverseVoxelSize = octree.size() / octree.dim();
        Eigen::Vector3f surfNorm = octree.grad(inverseVoxelSize * hit.head<3>(),
            [](const auto& val) { return val.x; });

        if (surfNorm.norm() == 0) {
            normal[x + y * frame_size.x()] = Eigen::Vector3f(INVALID, 0, 0);
        } else {
            // Invert normals if SDF
            if (voxel_traits<FieldType>::invert_normals) surfNorm *= -1.f;
            normal[x + y * frame_size.x()] = surfNorm.normalized();
        }
    } else {
        vertex[x + y * frame_size.x()] = Eigen::Vector3f::Constant(0);
        normal[x + y * frame_size.x()] = Eigen::Vector3f(INVALID, 0, 0);
    }
}

void raycast(const Octree<FieldType, MemoryPoolCUDA>& octree,
    Eigen::Vector3f* vertex, Eigen::Vector3f* normal,
    Eigen::Vector2i frame_size, Eigen::Matrix4f view, Eigen::Vector2f planes,
    float mu, float step) {
    constexpr int thread_dim = 16;

    dim3 threads(thread_dim, thread_dim);
    dim3 blocks((frame_size.x() + thread_dim - 1) / thread_dim,
        (frame_size.y() + thread_dim - 1) / thread_dim);

    raycastKernel<<<blocks, threads>>>(
        octree, vertex, normal, frame_size, view, planes, mu, step);
    safeCall(cudaPeekAtLastError());

    safeCall(cudaDeviceSynchronize());
}

} // namespace se
