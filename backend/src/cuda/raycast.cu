#include "raycast.hpp"

#include "../common/field_impls.hpp"

#include <supereight/ray_iterator.hpp>
#include <supereight/utils/cuda_util.hpp>

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

    float t_min = ray.tcmin();

    const Eigen::Vector4f hit = t_min > 0.f
        ? voxel_traits<FieldType>::raycast(octree, transl, dir, t_min,
              ray.tmax(), mu, step, step * BLOCK_SIDE)
        : Eigen::Vector4f::Constant(0.f);

    if (hit.w() <= 0.0) {
        vertex[x + y * frame_size.x()] = Eigen::Vector3f::Constant(0);
        normal[x + y * frame_size.x()] = Eigen::Vector3f(INVALID, 0, 0);
        return;
    }

    vertex[x + y * frame_size.x()] = hit.head<3>();

    const float inverseVoxelSize = octree.size() / octree.dim();
    Eigen::Vector3f surfNorm     = octree.grad(inverseVoxelSize * hit.head<3>(),
        [](const auto& val) { return val.x; });

    if (surfNorm.norm() == 0) {
        normal[x + y * frame_size.x()] = Eigen::Vector3f(INVALID, 0, 0);
        return;
    }

    // Invert normals if SDF
    if (voxel_traits<FieldType>::invert_normals) surfNorm *= -1.f;
    normal[x + y * frame_size.x()] = surfNorm.normalized();
}

__global__ static void renderKernel(Eigen::Vector<unsigned char, 4>* out,
    Eigen::Vector2i output_size, Eigen::Vector3f* vertex,
    Eigen::Vector3f* normal, Eigen::Vector3f light, Eigen::Vector3f ambient) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= output_size.x() || y >= output_size.y()) return;

    int idx = x + output_size.x() * y;

    auto point     = vertex[idx];
    auto surf_norm = normal[idx];

    auto col = Eigen::Vector3f(255, 0, 0);
    if (surf_norm.x() != INVALID && surf_norm.norm() > 0) {
        const Eigen::Vector3f diff = (point - light).normalized();
        const Eigen::Vector3f dir  = Eigen::Vector3f::Constant(
            fmaxf(surf_norm.normalized().dot(diff), 0.f));

        col = dir + ambient;
        se::math::clamp(col, Eigen::Vector3f::Constant(0.f),
            Eigen::Vector3f::Constant(1.f));

        col *= 255.f;
    }

    out[idx].head<3>() = col.cast<unsigned char>();
    out[idx].w()       = 0;
}

void raycast(const Octree<FieldType, MemoryPoolCUDA>& octree,
    Eigen::Vector3f* vertex, Eigen::Vector3f* normal,
    Eigen::Vector2i frame_size, Eigen::Matrix4f view, Eigen::Vector2f planes,
    float mu, float step) {
    dim3 threads(16, 16);
    dim3 blocks((frame_size.x() + threads.x - 1) / threads.x,
        (frame_size.y() + threads.y - 1) / threads.y);

    raycastKernel<<<blocks, threads>>>(
        octree, vertex, normal, frame_size, view, planes, mu, step);
    safeCall(cudaPeekAtLastError());
}

void render(Eigen::Vector<unsigned char, 4>* out, Eigen::Vector2i output_size,
    Eigen::Vector3f* vertex, Eigen::Vector3f* normal, Eigen::Vector3f light,
    Eigen::Vector3f ambient) {
    constexpr int thread_dim = 16;

    dim3 threads(thread_dim, thread_dim);
    dim3 blocks((output_size.x() + thread_dim - 1) / thread_dim,
        (output_size.y() + thread_dim - 1) / thread_dim);

    renderKernel<<<blocks, threads>>>(
        out, output_size, vertex, normal, light, ambient);
    safeCall(cudaPeekAtLastError());
}

} // namespace se
