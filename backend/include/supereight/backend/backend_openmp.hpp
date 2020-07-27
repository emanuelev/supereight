#pragma once

#include <supereight/backend/backend.hpp>
#include <supereight/memory/memory_pool.hpp>

#include <supereight/octree.hpp>

namespace se {

class Backend final : public BackendBase<Backend, MemoryPool> {
public:
    Backend(int size, float dim) : BackendBase(size, dim) {}

private:
    std::vector<se::key_t> allocation_list_;

    void allocate_(const Image<float>& depth, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, const Eigen::Vector2i& computation_size,
        float mu);

    void update_(const Image<float>& depth, const Sophus::SE3f& Tcw,
        const Eigen::Vector4f& k, const Eigen::Vector2i& computation_size,
        float mu, int frame);

    void raycast_(Image<Eigen::Vector3f>& vertex,
        Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, float mu);

    void render_(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose, float large_step,
        float mu, const Image<Eigen::Vector3f>& vertex,
        const Image<Eigen::Vector3f>& normal,
        const Eigen::Matrix4f& raycast_pose);

    friend class BackendBase<Backend, MemoryPool>;
};

} // namespace se
