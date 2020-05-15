#pragma once

#include <supereight/tracking/track_data.hpp>

#include <Eigen/Dense>
#include <vector>

namespace se {
namespace tracking {

template<typename Derived, template<typename> class BufferT>
class TrackerBase {
public:
    TrackerBase(const std::vector<int>& pyramid,
        Eigen::Vector2i computation_size, Eigen::Matrix4f init_pose);

    Eigen::Matrix4f getPose() const { return pose_; };
    Eigen::Matrix4f getInitPose() const { return init_pose_; };

    void setPose(const Eigen::Matrix4f& pose) {
        pose_ = pose;
        pose_.block<3, 1>(0, 3) += init_pose_.block<3, 1>(0, 3);
    }

    bool track(const Eigen::Vector4f& k, se::Image<float>& input_depth,
        float icp_threshold, const Eigen::Matrix4f& render_pose,
        const se::Image<Eigen::Vector3f>& rendered_vertex,
        const se::Image<Eigen::Vector3f>& rendered_normal);

    void renderTrack(
        unsigned char* out, const Eigen::Vector2i& output_size) const;

private:
    Derived* self = static_cast<Derived*>(this);

protected:
    std::vector<int> pyramid_;
    Eigen::Vector2i computation_size_;

    Eigen::Matrix4f pose_;
    Eigen::Matrix4f init_pose_;

    std::vector<BufferT<float>> scaled_depth_;
    std::vector<BufferT<Eigen::Vector3f>> vertex_;
    std::vector<BufferT<Eigen::Vector3f>> normal_;

    BufferT<float> reduction_output_;
    BufferT<TrackData> tracking_result_;
};

template<typename Derived, template<typename> class BufferT>
TrackerBase<Derived, BufferT>::TrackerBase(const std::vector<int>& pyramid,
    Eigen::Vector2i computation_size, Eigen::Matrix4f init_pose)
    : pyramid_{pyramid}, computation_size_{computation_size}, pose_{init_pose},
      init_pose_{init_pose}, reduction_output_{8 * 32, 1},
      tracking_result_{computation_size} {
    int downsample = 1;
    for (auto iteration : pyramid_) {
        std::ignore                      = iteration;
        Eigen::Vector2i downsampled_size = computation_size_ / downsample;

        scaled_depth_.push_back(BufferT<float>(downsampled_size));

        vertex_.push_back(BufferT<Eigen::Vector3f>(downsampled_size));
        normal_.push_back(BufferT<Eigen::Vector3f>(downsampled_size));

        downsample *= 2;
    }
}

template<typename Derived, template<typename> class BufferT>
bool TrackerBase<Derived, BufferT>::track(const Eigen::Vector4f& k,
    se::Image<float>& input_depth, float icp_threshold,
    const Eigen::Matrix4f& render_pose,
    const se::Image<Eigen::Vector3f>& rendered_vertex,
    const se::Image<Eigen::Vector3f>& rendered_normal) {
    self->copyInput_(scaled_depth_[0].accessor(), input_depth);

    // Half sample input depth maps into the pyramid levels
    for (std::size_t i = 1; i < pyramid_.size(); ++i) {
        self->halfSample_(scaled_depth_[i].accessor(),
            scaled_depth_[i - 1].accessor(), e_delta * 3, 1);
    }

    for (std::size_t i = 0; i < pyramid_.size(); ++i) {
        Eigen::Matrix4f invK = getInverseCameraMatrix(k / float(1 << i));
        self->depthToVertex_(
            vertex_[i].accessor(), scaled_depth_[i].accessor(), invK);

        if (k.y() < 0)
            self->template vertexToNormal_<true>(
                normal_[i].accessor(), vertex_[i].accessor());
        else
            self->template vertexToNormal_<false>(
                normal_[i].accessor(), vertex_[i].accessor());
    }

    Eigen::Matrix4f old_pose = pose_;
    const Eigen::Matrix4f render_view =
        getCameraMatrix(k) * render_pose.inverse();

    self->setReference_(rendered_vertex, rendered_normal, render_view);

    for (int level = pyramid_.size() - 1; level >= 0; --level) {
        Eigen::Vector2i local_size(computation_size_.x() / (int) pow(2, level),
            computation_size_.y() / (int) pow(2, level));

        for (int i = 0; i < pyramid_[level]; ++i) {
            self->track_(vertex_[level].accessor(), normal_[level].accessor(),
                dist_threshold, normal_threshold);

            self->reduce_(local_size);

            if (self->updatePose_(icp_threshold)) break;
        }
    }

    return self->checkPose_(old_pose, track_threshold);
}

template<typename Derived, template<typename> class BufferT>
void TrackerBase<Derived, BufferT>::renderTrack(
    unsigned char* out, const Eigen::Vector2i& output_size) const {
    self->renderTrack_(out, output_size);
}

} // namespace tracking
} // namespace se

#define tracker_header__(b) #b
#define tracker_header_(b) tracker_header__(tracker_##b.hpp)
#define tracker_header(b) tracker_header_(b)

#include tracker_header(SE_TRACKER)
