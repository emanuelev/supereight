#include "tracking_kernels.hpp"

#include <supereight/tracking/tracker_openmp.hpp>

namespace se {
namespace tracking {

void Tracker::copyInput_(
    buffer_type<float>::accessor_type output, const Image<float>& input) {
    std::memcpy(
        output.data(), input.accessor().data(), output.size() * sizeof(float));
}

void Tracker::halfSample_(buffer_type<float>::accessor_type output,
    const buffer_type<float>::accessor_type& input, const float e_delta,
    const int radius) {
    kernels::halfSampleRobustImage(output, input, e_delta, radius);
}

void Tracker::depthToVertex_(buffer_type<Eigen::Vector3f>::accessor_type vertex,
    const buffer_type<float>::accessor_type& depth,
    const Eigen::Matrix4f& invK) {
    kernels::depth2vertex(vertex, depth, invK);
}

template<bool NegY>
void Tracker::vertexToNormal_(
    buffer_type<Eigen::Vector3f>::accessor_type normal,
    const buffer_type<Eigen::Vector3f>::accessor_type& vertex) {
    kernels::vertex2normal<NegY>(normal, vertex);
}

template void Tracker::vertexToNormal_<true>(
    buffer_type<Eigen::Vector3f>::accessor_type normal,
    const buffer_type<Eigen::Vector3f>::accessor_type& vertex);

template void Tracker::vertexToNormal_<false>(
    buffer_type<Eigen::Vector3f>::accessor_type normal,
    const buffer_type<Eigen::Vector3f>::accessor_type& vertex);

void Tracker::setReference_(const Image<Eigen::Vector3f>& reference_vertex,
    const Image<Eigen::Vector3f>& reference_normal,
    const Eigen::Matrix4f& reference_view) {
    reference_vertex_ = &reference_vertex;
    reference_normal_ = &reference_normal;

    reference_view_ = reference_view;
}

void Tracker::track_(const buffer_type<Eigen::Vector3f>::accessor_type& vertex,
    const buffer_type<Eigen::Vector3f>::accessor_type& normal,
    const float dist_threshold, const float normal_threshold) {
    kernels::track(tracking_result_.accessor(), vertex, normal,
        reference_vertex_->accessor(), reference_normal_->accessor(), pose_,
        reference_view_, dist_threshold, normal_threshold);
}

void Tracker::reduce_(const Eigen::Vector2i& local_size) {
    kernels::reduce(reduction_output_.accessor(), tracking_result_.accessor(),
        computation_size_, local_size);
}

bool Tracker::updatePose_(const float icp_threshold) {
    return kernels::updatePose(
        pose_, reduction_output_.accessor(), icp_threshold);
}

bool Tracker::checkPose_(
    const Eigen::Matrix4f& old_pose, const float track_threshold) {
    return kernels::checkPose(pose_, old_pose, reduction_output_.accessor(),
        computation_size_, track_threshold);
}

void Tracker::renderTrack_(
    unsigned char* out, const Eigen::Vector2i& output_size) {
    kernels::renderTrack(out, tracking_result_.accessor(), output_size);
}

} // namespace tracking
} // namespace se
