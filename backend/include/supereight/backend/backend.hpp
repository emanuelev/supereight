#pragma once

#include <supereight/octree.hpp>

namespace se {

using FieldType = SE_FIELD_TYPE;

template<typename Derived, template<typename> class BufferT>
class BackendBase {
    BackendBase(int size, float dim);

    void integrate(const Image<float>& depth, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
        float mu);

    void raycast(Image<Eigen::Vector3f>& vertex, Image<Eigen::Vector3f>& normal,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float mu);

    void render(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        float large_step);

private:
    Derived* self = static_cast<Derived*>(this);

    std::vector<key_t> allocation_list_;
    Octree<FieldType, BufferT> octree_;

    virtual int buildAllocationList_(const Image<float>& depth,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        const Eigen::Vector2i& computation_size, float mu) = 0;

    virtual bool allocate_(int n) = 0;

    virtual void projectiveUpdate_(const Image<float>& depth,
        const Sophus::SE3f& Tcw, const Eigen::Vector4f& k,
        const Eigen::Vector2i& computation_size) = 0;

    virtual void raycast_(Image<Eigen::Vector3f>& vertex,
        Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, float mu) = 0;

    virtual void render_(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        float large_step) = 0;
};

template<typename Derived, template<typename> class BufferT>
BackendBase<Derived, BufferT>::BackendBase(int size, float dim)
    : octree_{size, dim} {}

template<typename Derived, template<typename> class BufferT>
void BackendBase<Derived, BufferT>::integrate(const Image<float>& depth,
    const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
    const Eigen::Vector2i& computation_size, float mu) {
    float voxel_size   = octree_.dim() / octree_.size();
    int voxels_per_pix = octree_.dim() / (BLOCK_SIDE * voxel_size);
    size_t total = voxels_per_pix * computation_size.x() * computation_size.y();

    allocation_list_.reserve(total);

    int allocated =
        self->buildAllocationList_(depth, k, pose, computation_size, mu);
    self->allocate_(allocated);
    self->projectiveUpdate_(
        depth, Sophus::SE3f(pose).inverse(), k, computation_size);
}

template<typename Derived, template<typename> class BufferT>
void BackendBase<Derived, BufferT>::raycast(Image<Eigen::Vector3f>& vertex,
    Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, float mu) {
    self->raycast_(vertex, normal, k, pose, mu);
}

template<typename Derived, template<typename> class BufferT>
void BackendBase<Derived, BufferT>::render(unsigned char* out,
    const Eigen::Vector2i& output_size, const Eigen::Vector4f& k,
    const Eigen::Matrix4f& pose, float large_step) {
    self->render(out, output_size, k, pose, large_step);
}

} // namespace se

#define backend_header__(b) #b
#define backend_header_(b) backend_header__(backend_##b.hpp)
#define backend_header(b) backend_header_(b)

#include backend_header(SE_BACKEND)
