#pragma once

#include <supereight/backends/backend_base.hpp>
#include <supereight/backends/buffer_cpu.hpp>
#include <supereight/octree.hpp>

namespace se {

class BackendOpenMP final
    : public Backend<BackendOpenMP, Octree, BufferCPU, FIELD_TYPE> {
private:
    int buildAllocationList_(const se::Image<float>& depth,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        const Eigen::Vector2i& computation_size, float mu) override;

    bool allocate_(int n) override;

    void projectiveUpdate_(const se::Image<float>& depth,
        const Sophus::SE3f& Tcw, const Eigen::Vector4f& k,
        const Eigen::Vector2i& computation_size) override;

    void raycast_(se::Image<Eigen::Vector3f>& vertex,
        se::Image<Eigen::Vector3f>& normal, const Eigen::Vector4f& k,
        const Eigen::Matrix4f& pose, float mu) override;

    void render_(unsigned char* out, const Eigen::Vector2i& output_size,
        const Eigen::Vector4f& k, const Eigen::Matrix4f& pose,
        float large_step) override;
}

} // namespace se
