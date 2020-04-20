#pragma once

#include <supereight/voxel_traits.hpp>

struct OFusion {
    float x;
    double y;
};

template<>
struct voxel_traits<OFusion> {
    using value_type = OFusion;

    static value_type empty() { return {0.f, 0.f}; }
    static value_type initValue() { return {0.f, 0.f}; }
};
