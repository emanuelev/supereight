#pragma once

#include <supereight/memory/image.hpp>
#include <supereight/tracking/tracker.hpp>

namespace se {
namespace tracking {

template<typename T>
using buffer_type = Image<T>;

class Tracker final : public TrackerBase<Tracker, buffer_type> {
public:
    Tracker(const std::vector<int>& pyramid, Eigen::Vector2i computation_size,
        Eigen::Matrix4f init_pose)
        : TrackerBase(pyramid, computation_size, init_pose) {}

private:
    void copyInput_(
        buffer_type<float>::accessor_type output, const Image<float>& input);

    void halfSample_(buffer_type<float>::accessor_type output,
        const buffer_type<float>::accessor_type& input, const float e_delta,
        const int radius);

    void depthToVertex_(buffer_type<Eigen::Vector3f>::accessor_type vertex,
        const buffer_type<float>::accessor_type& depth,
        const Eigen::Matrix4f& invK);

    template<bool NegY>
    void vertexToNormal_(buffer_type<Eigen::Vector3f>::accessor_type normal,
        const buffer_type<Eigen::Vector3f>::accessor_type& vertex);

    void track_(const buffer_type<Eigen::Vector3f>::accessor_type& vertex,
        const buffer_type<Eigen::Vector3f>::accessor_type& normal,
        const float dist_threshold, const float normal_threshold);

    void reduce_(const Eigen::Vector2i& local_size);

    bool updatePose_(const float icp_threshold);

    bool checkPose_(
        const Eigen::Matrix4f& old_pose, const float track_threshold);

    void renderTrack_(unsigned char* out, const Eigen::Vector2i& output_size);

    friend class TrackerBase<Tracker, buffer_type>;
};

} // namespace tracking
} // namespace se
