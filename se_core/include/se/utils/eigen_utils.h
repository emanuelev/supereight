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

template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type> > ;

typedef AlignedVector<Eigen::Vector3i> VectorVec3i;
typedef AlignedVector<std::pair<Eigen::Vector3i, double>> VectorPair3iDouble;



typedef std::map<key_t ,
                 Eigen::Vector3i,
                 std::less<key_t>,
                 Eigen::aligned_allocator<std::pair<const key_t, Eigen::Vector3i> > > map3i;
typedef std::map<key_t,
                 VectorVec3i,
                 std::less<key_t>,
                 Eigen::aligned_allocator<std::pair<const key_t, VectorVec3i> > > mapvec3i;

static Eigen::IOFormat
    InLine(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " ", ";");
#endif //SUPEREIGHT_EIGEN_ALIGNED_ALLOCATION_H
