#pragma once

#include <supereight/voxel_traits.hpp>

struct SDF {
    float x;
    float y;
};

template<>
struct voxel_traits<SDF> {
    using value_type = SDF;

    static value_type empty() { return {1.f, -1.f}; }
    static value_type initValue() { return {1.f, 0.f}; }
};

