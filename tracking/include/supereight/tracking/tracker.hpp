#pragma once

#include <supereight/image/image.hpp>
#include <supereight/tracking/track_data.hpp>

#include <Eigen/Dense>
#include <vector>

namespace se {

class Tracker {
private:
    std::vector<int> pyramid_;
    Eigen::Vector2i computation_size_;

    Eigen::Matrix4f pose_;
    Eigen::Matrix4f init_pose_;

    std::vector<se::Image<float>> scaled_depth_;
    std::vector<se::Image<Eigen::Vector3f>> vertex_;
    std::vector<se::Image<Eigen::Vector3f>> normal_;

    std::vector<float> reduction_output_;
    std::vector<se::TrackData> tracking_result_;

public:
    Tracker(const std::vector<int>& pyramid, Eigen::Vector2i computation_size,
        Eigen::Matrix4f init_pose);

    Eigen::Matrix4f getPose() { return pose_; };
    Eigen::Matrix4f getInitPose() { return init_pose_; };

    void setPose(const Eigen::Matrix4f& pose) {
        pose_ = pose;
        pose_.block<3, 1>(0, 3) += init_pose_.block<3, 1>(0, 3);
    }

    bool track(const Eigen::Vector4f& k, se::Image<float>& input_depth,
        float icp_threshold, const Eigen::Matrix4f& render_pose,
        const se::Image<Eigen::Vector3f>& rendered_vertex,
        const se::Image<Eigen::Vector3f>& rendered_normal);

    void renderTrack(unsigned char* out, const Eigen::Vector2i& output_size);
};

} // namespace se
