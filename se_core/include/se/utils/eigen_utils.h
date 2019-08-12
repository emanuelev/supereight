//
// Created by anna on 09/07/19.
//

#ifndef SUPEREIGHT_EIGEN_ALIGNED_ALLOCATION_H
#define SUPEREIGHT_EIGEN_ALIGNED_ALLOCATION_H

#include <set>
#include <map>
#include <vector>
#include <memory>
#include <utility>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <queue>
#include <deque>
#include "se/image/image.hpp"

#include "se/path_planning/exploration_utils.hpp"


template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type> > ;

template <typename Type>
using AlignedDeque = std::deque<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedQueue = std::queue<Type, AlignedDeque<Type>>;

typedef std::set<uint64_t > set3i;
typedef AlignedVector<Eigen::Vector3i> VecVec3i;
typedef AlignedVector<Eigen::Vector3f> VecVec3f;
typedef AlignedVector<std::pair<Eigen::Vector3i, float>> VectorPair3iFloat;


typedef AlignedVector<se::exploration::pose3D>  VecPose;
typedef AlignedVector<VecPose>  VecVecPose;
typedef AlignedVector<std::pair<se::exploration::pose3D, float>> VecPairPoseFloat;


typedef AlignedVector<se::Image<Eigen::Vector3f> > AlignedImage3f;
typedef AlignedVector<se::Image<float> > AlignedImagef;

// std::tuple doesn't need alignment, generalized version of pair
typedef AlignedQueue<std::tuple<Eigen::Vector3i, Eigen::Vector3i, int, int>> AlignedQueueTupleVec3i;
typedef AlignedQueue<std::tuple<Eigen::Vector3f, Eigen::Vector3f, int, int>> AlignedQueueTupleVec3f;

typedef std::map<key_t ,
                 Eigen::Vector3i,
                 std::less<key_t>,
                 Eigen::aligned_allocator<std::pair<const key_t, Eigen::Vector3i> > > map3i;
typedef std::map<key_t,
                 VecVec3i,
                 std::less<key_t>,
                 Eigen::aligned_allocator<std::pair<const key_t, VecVec3i> > > mapvec3i;



template <typename Type, typename... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  return std::allocate_shared<Type>(Eigen::aligned_allocator<TypeNonConst>(),
                                    std::forward<Arguments>(arguments)...);
}

static Eigen::IOFormat
    InLine(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " ", ";");
#endif //SUPEREIGHT_EIGEN_ALIGNED_ALLOCATION_H
