#pragma once

namespace se {

template<typename Derived,
    template<template<typename> typename, typename, typename> class OctreeT,
    template<typename> class BufferT, typename FieldT, unsigned BlockSide = 8>
class Backend {
    Backend(int size, float dim);

    void integrate(const se::Image<float>& depth, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
        float mu);

    void raycast(se::Image<Eigen::Vector3f>& vertex,
        se::Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, float mu);

    void render(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        float large_step);

private:
    Derived* self = static_cast<Derived*>(this);

    std::vector<se::key_t> allocation_list_;
    OctreeT<BufferT, FieldT, BlockSide> octree_;

    virtual int buildAllocationList_(const se::Image<float>& depth,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        const Eigen::Vector2i& computation_size, float mu) = 0;

    virtual bool allocate_(int n) = 0;

    virtual void projectiveUpdate_(const se::Image<float>& depth,
        const Sophus::SE3f& Tcw, const Eigen::Vector4f& k,
        const Eigen::Vector2i& computation_size) = 0;

    virtual void raycast_(se::Image<Eigen::Vector3f>& vertex,
        se::Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, float mu) = 0;

    virtual void render_(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        float large_step) = 0;
}

Backend::Backend(int size, float dim)
    : octree_{size_, dim_} {};

void Backend::integrate(const se::Image<float>& depth, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
    float mu) {
    float voxel_size   = octree_.dim() / octree_.size();
    int voxels_per_pix = octree_.dim() / (BlockSide * voxel_size);
    size_t total =
        voxels_per_pix * computation_size_.x() * computation_size_.y();

    allocation_list_.reserve(total);

    int allocated =
        self->buildAllocationList_(depth, k, pose, computation_size, mu);
    self->allocate_(allocated);
    self->projectiveUpdate_(
        depth, Sophus::SE3f(tracker_.getPose()).inverse(), k, computation_size);
}

void Backend::raycast(se::Image<Eigen::Vector3f>& vertex,
    se::Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, float mu) {
    self->raycast_(vertex, normal, k, pose, mu);
}

void Backend::render(unsigned char* out, const Eigen::Vector2i& output_size,
    const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float large_step) {
    self->render(out, output_size, k, pose, large_step);
}

} // namespace se
