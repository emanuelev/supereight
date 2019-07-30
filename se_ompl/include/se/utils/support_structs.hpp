
/**
 * Originally from Motion Planning, Header.
 *
 * Copyright (C) 2017 Imperial College London.
 * Copyright (C) 2017 ETH Zürich.

 * @file support_structs.hpp
 *
 * @ingroup common
 *
 * @author Marius Grimm (marius.grimm93@web.de)
 * @date May 19, 2017
 */

#ifndef SUPEREIGHT_SUPPORT_STRUCTS_HPP
#define SUPEREIGHT_SUPPORT_STRUCTS_HPP

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "se/utils/eigen_utils.h"

namespace se {
namespace exploration {
static constexpr size_t kDim = 3;      /// Dimension for straight line planning
static constexpr size_t kDimTraj = 3;  /// Dimension for trajectory optimization


typedef Eigen::Matrix<float, 3 , 1> Point;
typedef Eigen::Matrix<int , 3, 1> AnyIndex;

typedef AnyIndex VoxelCoord;


template<int TSize>
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Matrix<double, TSize, 1> segment_end;
  double segment_radius = -1;
};




/** Struct defining a Header with a timestamp and frame id. */
struct Header {
  std::chrono::nanoseconds time_nsec;  ///> Timestamp in nanoseconds
  std::string frame_id;                ///> Frame id
};

/** Struct defining a Path with a Header and a vector of states. */
template<int TSize>
struct Path {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  /** Type definition for smart pointer. */
  typedef std::shared_ptr<Path<TSize> > Ptr;
  typedef AlignedVector < State <TSize> > StateVector;
  Header header;                     ///> Header holding timestamp and frame_id
  StateVector states;  ///> States of the path
};

/** Struct defining a Segment with a Header and a vector of states. */
template<int TSize>
struct Segment {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef AlignedVector < State <TSize> > StateVector;
  Header header;                     ///> Header holding timestamp and frame_id
  unsigned int num_points;           ///> Number of points in the segment
  StateVector states;  ///> States of the segment
  std::chrono::nanoseconds duration; ///> Duration of the segment
};

/** Struct defining a Trajectory with a Header and a vector of segments. */
template<int TSize>
struct Trajectory {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  /** Type definition for smart pointer. */
  typedef std::shared_ptr<Trajectory<TSize>> Ptr;

  typedef AlignedVector < Segment <TSize> > SegmentVector;
  Header header; ///> Header holding timestamp and frame_id
  SegmentVector segments; ///> Segments of the trajectory
};

}  // namespace exploration

}
#endif  // SUPEREIGHT_SUPPORT_STRUCTS_HPP
