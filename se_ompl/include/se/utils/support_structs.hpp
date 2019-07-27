
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

namespace se {
namespace exploration {
static constexpr size_t kDim = 3;      /// Dimension for straight line planning
static constexpr size_t kDimTraj = 3;  /// Dimension for trajectory optimization
template<int TSize>
struct State {
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
  /** Type definition for smart pointer. */
  typedef std::shared_ptr<Path<TSize>> Ptr;

  Header header;                     ///> Header holding timestamp and frame_id
  std::vector<State<TSize>> states;  ///> States of the path
};

/** Struct defining a Segment with a Header and a vector of states. */
template<int TSize>
struct Segment {
  Header header;                     ///> Header holding timestamp and frame_id
  unsigned int num_points;           ///> Number of points in the segment
  std::vector<State<TSize>> states;  ///> States of the segment
  std::chrono::nanoseconds duration; ///> Duration of the segment
};

/** Struct defining a Trajectory with a Header and a vector of segments. */
template<int TSize>
struct Trajectory {
  /** Type definition for smart pointer. */
  typedef std::shared_ptr<Trajectory<TSize>> Ptr;

  Header header; ///> Header holding timestamp and frame_id
  std::vector<Segment<TSize>> segments; ///> Segments of the trajectory
};

}  // namespace exploration

}
#endif  // SUPEREIGHT_SUPPORT_STRUCTS_HPP
