#pragma once

#include <supereight/backend/fields.hpp>
#include <supereight/octree.hpp>

namespace se {

template<typename Derived, template<typename> class BufferT>
class BackendBase {
public:
    BackendBase(int size, float dim);

    void integrate(const Image<float>& depth, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
        float mu, int frame);

    void raycast(Image<Eigen::Vector3f>& vertex, Image<Eigen::Vector3f>& normal,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float mu);

    void render(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float large_step,
        float mu, const Image<Eigen::Vector3f>& vertex,
        const Image<Eigen::Vector3f>& normal,
        const Eigen::Matrix4f& raycast_pose);

private:
    Derived* self = static_cast<Derived*>(this);

protected:
    Octree<FieldType, BufferT> octree_;

    virtual void allocate_(const Image<float>& depth, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
        float mu) = 0;

    virtual void update_(const Image<float>& depth, const Sophus::SE3f& Tcw,
        const Eigen::Vector4f& k, const Eigen::Vector2i& computation_size,
        float mu, int frame) = 0;

    virtual void raycast_(Image<Eigen::Vector3f>& vertex,
        Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, float mu) = 0;

    virtual void render_(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float large_step,
        float mu, const Image<Eigen::Vector3f>& vertex,
        const Image<Eigen::Vector3f>& normal,
        const Eigen::Matrix4f& raycast_pose) = 0;
};

template<typename Derived, template<typename> class BufferT>
BackendBase<Derived, BufferT>::BackendBase(int size, float dim) {
    octree_.init(size, dim);
}

template<typename Derived, template<typename> class BufferT>
void BackendBase<Derived, BufferT>::integrate(const Image<float>& depth,
    const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
    const Eigen::Vector2i& computation_size, float mu, int frame) {
    self->allocate_(depth, k, pose, computation_size, mu);

    self->update_(
        depth, Sophus::SE3f(pose).inverse(), k, computation_size, mu, frame);
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
    const Eigen::Matrix4f& pose, float large_step, float mu,
    const Image<Eigen::Vector3f>& vertex, const Image<Eigen::Vector3f>& normal,
    const Eigen::Matrix4f& raycast_pose) {
    self->render_(out, output_size, k, pose, large_step, mu, vertex, normal,
        raycast_pose);
}

} // namespace se

#define backend_header__(b) #b
#define backend_header_(b) backend_header__(backend_##b.hpp)
#define backend_header(b) backend_header_(b)

#include backend_header(SE_BACKEND)
