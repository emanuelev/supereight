#include "../common/field_impls.hpp"

#include "kernels.hpp"
#include "projective_update.hpp"
#include "raycast.hpp"
#include "util.hpp"

#include <supereight/backend/backend.hpp>

#include <cuda_runtime.h>

namespace se {

void Backend::allocate_(const Image<float>& depth, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
    float mu) {
    float voxel_size    = octree_.dim() / octree_.size();
    int num_vox_per_pix = octree_.dim() / (BLOCK_SIDE * voxel_size);
    size_t total =
        num_vox_per_pix * computation_size.x() * computation_size.y();

    if (depth_ == nullptr || depth_size_ != computation_size) {
        depth_size_ = computation_size;

        if (depth_ != nullptr) safeCall(cudaFree(depth_));
        safeCall(cudaMalloc(&depth_, sizeof(float) * depth_size_.prod()));
    }

    safeCall(cudaMemcpy(depth_, depth.data(),
        sizeof(float) * depth_size_.prod(), cudaMemcpyHostToDevice));

    allocation_list_.reserve(total);

    int allocated;
    buildAllocationListKernel(allocation_list_.data(),
        allocation_list_.capacity(), allocated, octree_, pose,
        getCameraMatrix(k), depth.data(), computation_size, mu);

    octree_.allocate(allocation_list_.data(), allocated);
}

void Backend::update_(const Image<float>&, const Sophus::SE3f& Tcw,
    const Eigen::Vector4f& k, const Eigen::Vector2i& computation_size, float mu,
    int frame) {
    float voxel_size = octree_.dim() / octree_.size();
    float timestamp  = (1.f / 30.f) * frame;

    voxel_traits<FieldType>::update_func_type func(
        depth_, computation_size, mu, timestamp, voxel_size);

    se::projectiveUpdate(
        octree_, func, Tcw, getCameraMatrix(k), computation_size);
}

void Backend::raycast_(Image<Eigen::Vector3f>& vertex,
    Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, float mu) {
    if (normal.dim() != vertex.dim()) return;
    int size = sizeof(Eigen::Vector3f) * normal.dim().prod();

    if (vertex_ == nullptr || normal_ == nullptr ||
        frame_size_ != normal.dim()) {
        frame_size_ = normal.dim();

        if (vertex_ != nullptr) safeCall(cudaFree(vertex_));
        if (normal_ != nullptr) safeCall(cudaFree(normal_));

        safeCall(cudaMalloc(&vertex_, size));
        safeCall(cudaMalloc(&normal_, size));
    }

    float step = octree_.dim() / octree_.size();
    se::raycast(octree_, vertex_, normal_, frame_size_,
        pose * getInverseCameraMatrix(k), Eigen::Vector2f(nearPlane, farPlane),
        mu, step);

    safeCall(cudaMemcpy(vertex.data(), vertex_, size, cudaMemcpyDeviceToHost));
    safeCall(cudaMemcpy(normal.data(), normal_, size, cudaMemcpyDeviceToHost));
}

void Backend::render_(unsigned char* out, const Eigen::Vector2i& output_size,
    const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float large_step,
    float mu, const Image<Eigen::Vector3f>& vertex,
    const Image<Eigen::Vector3f>& normal, const Eigen::Matrix4f& raycast_pose) {
    float step = octree_.dim() / octree_.size();

    renderVolumeKernel(octree_, out, output_size,
        pose * getInverseCameraMatrix(k), nearPlane, farPlane * 2.0f, mu, step,
        large_step, pose.topRightCorner<3, 1>(), ambient,
        !(pose.isApprox(raycast_pose)), vertex, normal);
}

} // namespace se
