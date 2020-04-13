#include "tracking_kernels.hpp"
#include <supereight/tracking/tracker.hpp>

#include <tuple>

namespace se {

Tracker::Tracker(const std::vector<int>& pyramid,
    Eigen::Vector2i computation_size, Eigen::Matrix4f init_pose)
    : pyramid_{pyramid}, computation_size_{computation_size}, pose_{init_pose},
      init_pose_{init_pose} {
    int downsample = 1;
    for (auto iteration : pyramid_) {
        std::ignore = iteration;

        scaled_depth_.push_back(
            se::Image<float>(computation_size_.x() / downsample,
                computation_size_.y() / downsample));

        vertex_.push_back(
            se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
                computation_size_.y() / downsample));

        normal_.push_back(
            se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
                computation_size_.y() / downsample));

        downsample *= 2;
    }

    reduction_output_.resize(8 * 32);
    tracking_result_.resize(computation_size_.x() * computation_size_.y());
}

bool Tracker::track(const Eigen::Vector4f& k, se::Image<float>& input_depth,
    float icp_threshold, const Eigen::Matrix4f& render_pose,
    const se::Image<Eigen::Vector3f>& rendered_vertex,
    const se::Image<Eigen::Vector3f>& rendered_normal) {
    std::memcpy(scaled_depth_[0].data(), input_depth.data(),
        sizeof(float) * computation_size_.x() * computation_size_.y());

    // Half sample input depth maps into the pyramid levels
    for (std::size_t i = 1; i < pyramid_.size(); ++i) {
        halfSampleRobustImageKernel(
            scaled_depth_[i], scaled_depth_[i - 1], e_delta * 3, 1);
    }

    for (std::size_t i = 0; i < pyramid_.size(); ++i) {
        Eigen::Matrix4f invK = getInverseCameraMatrix(k / float(1 << i));
        depth2vertexKernel(vertex_[i], scaled_depth_[i], invK);

        if (k.y() < 0)
            vertex2normalKernel<true>(normal_[i], vertex_[i]);
        else
            vertex2normalKernel<false>(normal_[i], vertex_[i]);
    }

    Eigen::Matrix4f old_pose = pose_;
    const Eigen::Matrix4f projectReference =
        getCameraMatrix(k) * render_pose.inverse();

    for (int level = pyramid_.size() - 1; level >= 0; --level) {
        Eigen::Vector2i local_size(computation_size_.x() / (int) pow(2, level),
            computation_size_.y() / (int) pow(2, level));

        for (int i = 0; i < pyramid_[level]; ++i) {
            trackKernel(tracking_result_.data(), vertex_[level], normal_[level],
                rendered_vertex, rendered_normal, pose_, projectReference,
                dist_threshold, normal_threshold);

            reduceKernel(reduction_output_.data(), tracking_result_.data(),
                computation_size_, local_size);

            if (updatePoseKernel(
                    pose_, reduction_output_.data(), icp_threshold))
                break;
        }
    }

    return checkPoseKernel(pose_, old_pose, reduction_output_.data(),
        computation_size_, track_threshold);
}

void Tracker::renderTrack(
    unsigned char* out, const Eigen::Vector2i& output_size) {
    renderTrackKernel(out, tracking_result_.data(), output_size);
}

} // namespace se
