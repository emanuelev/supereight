#include "../common/field_impls.hpp"

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

    int allocated = se::buildAllocationList(allocation_list_.accessor(),
        octree_, allocation_list_used_, pose, getCameraMatrix(k),
        depth_.accessor(), computation_size, mu);

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

    raycast_dim_  = normal.dim();
    raycast_view_ = pose * getInverseCameraMatrix(k);

    vertex_.resize(raycast_dim_.prod());
    normal_.resize(raycast_dim_.prod());

    float step = octree_.dim() / octree_.size();
    se::raycast(octree_, vertex_.accessor().data(), normal_.accessor().data(),
        raycast_dim_, raycast_view_, Eigen::Vector2f(nearPlane, farPlane), mu,
        step);

    int size = sizeof(Eigen::Vector3f) * raycast_dim_.prod();
    safeCall(cudaMemcpy(vertex.data(), vertex_.accessor().data(), size,
        cudaMemcpyDeviceToHost));
    safeCall(cudaMemcpy(normal.data(), normal_.accessor().data(), size,
        cudaMemcpyDeviceToHost));
}

void Backend::render_(unsigned char* out, const Eigen::Vector2i& output_size,
    const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float, float mu,
    const Image<Eigen::Vector3f>&, const Image<Eigen::Vector3f>&,
    const Eigen::Matrix4f&) {
    float step = octree_.dim() / octree_.size();

    Eigen::Matrix4f render_view = pose * getInverseCameraMatrix(k);
    if (!render_view.isApprox(raycast_view_) ||
        !(output_size == raycast_dim_)) {
        vertex_.resize(output_size.prod());
        normal_.resize(output_size.prod());

        se::raycast(octree_, vertex_.accessor().data(),
            normal_.accessor().data(), output_size, render_view,
            Eigen::Vector2f(nearPlane, farPlane), mu, step);
    }

    render_out_.resize(output_size.prod());
    se::render(render_out_.accessor().data(), output_size,
        vertex_.accessor().data(), normal_.accessor().data(),
        pose.topRightCorner<3, 1>(), ambient);

    safeCall(cudaMemcpy(out, render_out_.accessor().data(),
        output_size.prod() * 4, cudaMemcpyDeviceToHost));
}

} // namespace se
