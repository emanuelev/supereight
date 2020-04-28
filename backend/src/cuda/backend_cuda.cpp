#include "../common/field_impls.hpp"

#include "kernels.hpp"

#include "allocate.hpp"
#include "projective_update.hpp"
#include "raycast.hpp"

#include <supereight/backend/backend.hpp>
#include <supereight/backend/cuda_util.hpp>

#include <cuda_runtime.h>

namespace se {

void Backend::allocate_(const Image<float>& depth, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
    float mu) {
    float voxel_size    = octree_.dim() / octree_.size();
    int num_vox_per_pix = octree_.dim() / (BLOCK_SIDE * voxel_size);
    size_t total =
        num_vox_per_pix * computation_size.x() * computation_size.y();

    depth_.resize(computation_size.prod());

    safeCall(cudaMemcpy(depth_.accessor().data(), depth.data(),
        sizeof(float) * computation_size.prod(), cudaMemcpyHostToDevice));

    if (allocation_list_used_ == nullptr)
        safeCall(cudaMallocManaged(&allocation_list_used_, sizeof(int)));

    allocation_list_.resize(total);

    int allocated = buildAllocationList(allocation_list_.accessor(), octree_,
        allocation_list_used_, pose, getCameraMatrix(k), depth_.accessor(),
        computation_size, mu);

    octree_.allocate(allocation_list_.accessor().data(), allocated);
}

void Backend::update_(const Image<float>&, const Sophus::SE3f& Tcw,
    const Eigen::Vector4f& k, const Eigen::Vector2i& computation_size, float mu,
    int frame) {
    float voxel_size = octree_.dim() / octree_.size();
    float timestamp  = (1.f / 30.f) * frame;

    voxel_traits<FieldType>::update_func_type func(
        depth_.accessor().data(), computation_size, mu, timestamp, voxel_size);

    se::projectiveUpdate(
        octree_, func, Tcw, getCameraMatrix(k), computation_size);
}

void Backend::raycast_(Image<Eigen::Vector3f>& vertex,
    Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, float mu) {
    if (normal.dim() != vertex.dim()) return;
    int size = sizeof(Eigen::Vector3f) * normal.dim().prod();

    vertex_.resize(normal.dim().prod());
    normal_.resize(normal.dim().prod());

    float step = octree_.dim() / octree_.size();
    se::raycast(octree_, vertex_.accessor().data(), normal_.accessor().data(),
        normal.dim(), pose * getInverseCameraMatrix(k),
        Eigen::Vector2f(nearPlane, farPlane), mu, step);

    safeCall(cudaMemcpy(vertex.data(), vertex_.accessor().data(), size,
        cudaMemcpyDeviceToHost));
    safeCall(cudaMemcpy(normal.data(), normal_.accessor().data(), size,
        cudaMemcpyDeviceToHost));
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
