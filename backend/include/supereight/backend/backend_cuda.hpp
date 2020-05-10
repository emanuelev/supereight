#pragma once

#include <supereight/backend/backend.hpp>
#include <supereight/backend/buffer_cuda.hpp>
#include <supereight/backend/memory_pool_cuda.hpp>

#include <supereight/octree.hpp>

namespace se {

class Backend final : public BackendBase<Backend, MemoryPoolCUDA> {
public:
    Backend(int size, float dim) : BackendBase(size, dim) {}

private:
    BufferCUDA<se::key_t> allocation_list_;
    int* allocation_list_used_ = nullptr;

    BufferCUDA<se::key_t> keys_at_level_;
    int* keys_at_level_used_ = nullptr;

    BufferCUDA<std::uint8_t> allocation_temp_storage_;

    int* node_buffer_used_  = nullptr;
    int* block_buffer_used_ = nullptr;

    BufferCUDA<float> depth_;

    BufferCUDA<Eigen::Vector3f> vertex_;
    BufferCUDA<Eigen::Vector3f> normal_;

    BufferCUDA<Eigen::Vector<unsigned char, 4>> render_out_;

    Eigen::Vector2i raycast_dim_;
    Eigen::Matrix4f raycast_view_;

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

    friend class BackendBase<Backend, MemoryPoolCUDA>;
};

} // namespace se
