//
// Created by anna on 09/07/19.
//

#ifndef SUPEREIGHT_EIGEN_ALIGNED_ALLOCATION_H
#define SUPEREIGHT_EIGEN_ALIGNED_ALLOCATION_H

#include <set>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > vec3i;

typedef std::map<int,
                 vec3i,
                 std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, vec3i> > > mapvec3i;

typedef std::map<uint64_t,
                 Eigen::Vector3i,
                 std::less<uint64_t>,
                 Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector3i> > > map3i;

static Eigen::IOFormat
    InLine(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " ", ";");
#endif //SUPEREIGHT_EIGEN_ALIGNED_ALLOCATION_H
