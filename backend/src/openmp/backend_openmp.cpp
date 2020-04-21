#include "../common/field_impls.hpp"
#include "rendering.hpp"

#include <supereight/backend/backend_openmp.hpp>
#include <supereight/functors/projective_functor.hpp>

namespace se {

void Backend::allocate_(const Image<float>& depth, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
    float mu) {
    float voxel_size    = octree_.dim() / octree_.size();
    int num_vox_per_pix = octree_.dim() / (BLOCK_SIDE * voxel_size);
    size_t total =
        num_vox_per_pix * computation_size.x() * computation_size.y();

    allocation_list_.reserve(total);

    int allocated = voxel_traits<FieldType>::buildAllocationList(
        allocation_list_.data(), allocation_list_.capacity(), octree_, pose,
        getCameraMatrix(k), depth.data(), computation_size, mu);

    octree_.allocate(allocation_list_.data(), allocated);
}

void Backend::update_(const Image<float>& depth, const Sophus::SE3f& Tcw,
    const Eigen::Vector4f& k, const Eigen::Vector2i& computation_size, float mu,
    int frame) {
    float voxel_size = octree_.dim() / octree_.size();
    float timestamp  = (1.f / 30.f) * frame;

    voxel_traits<FieldType>::update_func_type funct(
        depth.data(), computation_size, mu, timestamp, voxel_size);

    se::functor::projective_map(
        octree_, Tcw, getCameraMatrix(k), computation_size, funct);
}

void Backend::raycast_(Image<Eigen::Vector3f>& vertex,
    Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, float mu) {
    float step = octree_.dim() / octree_.size();

    raycastKernel(octree_, vertex, normal, pose * getInverseCameraMatrix(k),
        nearPlane, farPlane, mu, step, step * BLOCK_SIDE);
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
