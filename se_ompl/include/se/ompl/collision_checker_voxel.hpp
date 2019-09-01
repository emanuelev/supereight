
/**
 * Information-theoretic exploration
 *
 * Copyright (C) 2019 Imperial College London.
 * Copyright (C) 2019 ETH Zürich.
 *
 * @file collision_checker_voxel.hpp
 * @author Nils Funk
 * @author Anna Dai
 * @date August 22, 2019
 */


#ifndef SUPEREIGHT_COLLISION_CHECKER_VOXEL_HPP
#define SUPEREIGHT_COLLISION_CHECKER_VOXEL_HPP
#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>
#include <math.h>
#include <vector>

#include "se/utils/eigen_utils.h"
#include "se/utils/support_structs.hpp"

#include "se/continuous/volume_template.hpp"
#include "se/octree.hpp"
#include "se/planner_config.h"

namespace se {
namespace exploration {


// TODO state or log prob for collision check



template<typename FieldType>
class CollisionCheckerV {
 public:
  typedef std::shared_ptr<CollisionCheckerV<FieldType> > Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionCheckerV(const std::shared_ptr<Octree < FieldType>
  > octree_ptr,
  const Planning_Configuration &planning_config
  );

  bool isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3i &start,
                                           const Eigen::Vector3i &end,
                                           const int r_min,
                                           const int r_max) const;

  bool isVoxelFree(const Eigen::Vector3i &point_v) const;

  bool isNodeFree(const key_t morton_code, const int node_level) const;

  bool isSphereSkeletonFreeCand(const Eigen::Vector3i &position_v, const int radius_v) const;

  bool isSphereSkeletonFree(const Eigen::Vector3i &position_v, const int radius_v) const;

  bool isLineFree(const Eigen::Vector3i &start,
                  const Eigen::Vector3i &connection,
                  const int num_subpos) const; // check segment fligh corridor skeleton

  float getVoxelDim() const { return voxel_dim_; }
  int getNodeLevel(const int object_size_v);
  set3i getCollisionNodeList( const VecVec3i& point_list, const int node_level) const;

 private:

  std::shared_ptr<Octree < FieldType> >
  octree_ptr_ = nullptr;

  Planning_Configuration planning_params_;

  float voxel_dim_;
  int node_level_;
};

template<typename FieldType>
CollisionCheckerV<FieldType>::CollisionCheckerV(const std::shared_ptr<Octree < FieldType>
> octree_ptr,
const Planning_Configuration &planning_config
)
:
octree_ptr_ (octree_ptr), planning_params_(planning_config) {
  voxel_dim_ = octree_ptr->voxelDim();
  node_level_ = getNodeLevel(std::ceil(planning_params_.robot_safety_radius/voxel_dim_)*2);

  DLOG(INFO) << "Collision Checker V setup";
  LOG(INFO)<< "node level "<< node_level_;
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSphereSkeletonFreeCand(const Eigen::Vector3i &position_v,
                                                            const int radius_v) const {

  // LOG(INFO) << "voxel center "<< position_v.format(InLine) << " radius " << radius_v;
  if (octree_ptr_->get(position_v).x > 0.f)
    return false;

  VecVec3i shell_main_pos;
  shell_main_pos.push_back(position_v + Eigen::Vector3i(1, 0, 0) * radius_v);
  shell_main_pos.push_back(position_v - Eigen::Vector3i(1, 0, 0) * radius_v);
  shell_main_pos.push_back(position_v + Eigen::Vector3i(0, 1, 0) * radius_v);
  shell_main_pos.push_back(position_v - Eigen::Vector3i(0, 1, 0) * radius_v);
  shell_main_pos.push_back(position_v + Eigen::Vector3i(0, 0, 1) * radius_v);
  shell_main_pos.push_back(position_v - Eigen::Vector3i(0, 0, 1) * radius_v);
  for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); it++) {
    if (octree_ptr_->get(*it).x > 0.f)

      return false;
  }

  Eigen::Vector3i
      vec1_u = Eigen::Vector3i(1, 0, 0) + Eigen::Vector3i(0, 1, 0) + Eigen::Vector3i(0, 0, 1);
  vec1_u.normalize();
  Eigen::Vector3i
      vec2_u = Eigen::Vector3i(-1, 0, 0) + Eigen::Vector3i(0, 1, 0) + Eigen::Vector3i(0, 0, 1);
  vec2_u.normalize();
  Eigen::Vector3i
      vec3_u = Eigen::Vector3i(1, 0, 0) + Eigen::Vector3i(0, -1, 0) + Eigen::Vector3i(0, 0, 1);
  vec3_u.normalize();
  Eigen::Vector3i
      vec4_u = Eigen::Vector3i(-1, 0, 0) + Eigen::Vector3i(0, -1, 0) + Eigen::Vector3i(0, 0, 1);
  vec4_u.normalize();

  VecVec3i shell_sub_pos;
  shell_sub_pos.push_back(position_v + vec1_u * radius_v);
  shell_sub_pos.push_back( position_v - vec1_u * radius_v);
  shell_sub_pos.push_back(position_v + vec2_u * radius_v);
  shell_sub_pos.push_back(position_v - vec2_u * radius_v);
  shell_sub_pos.push_back(position_v + vec3_u * radius_v);
  shell_sub_pos.push_back( position_v - vec3_u * radius_v);
  shell_sub_pos.push_back(position_v + vec4_u * radius_v);
  shell_sub_pos.push_back(position_v - vec4_u * radius_v);
  for (VecVec3i::iterator it = shell_sub_pos.begin(); it != shell_sub_pos.end(); ++it) {
    if (octree_ptr_->get(*it).x > 0.f)
      return false;
  }

  return true;
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSphereSkeletonFree(const Eigen::Vector3i &position_v,
                                                        const int radius_v) const {

  // LOG(INFO) << "voxel center "<< position_v.format(InLine) << " radius " << radius_v;
  if (!isVoxelFree(position_v))
    return false;

  Eigen::Vector3i
      vec1_u = Eigen::Vector3i(1, 0, 0) + Eigen::Vector3i(0, 1, 0) + Eigen::Vector3i(0, 0, 1);
  vec1_u.normalize();
  Eigen::Vector3i
      vec2_u = Eigen::Vector3i(-1, 0, 0) + Eigen::Vector3i(0, 1, 0) + Eigen::Vector3i(0, 0, 1);
  vec2_u.normalize();
  Eigen::Vector3i
      vec3_u = Eigen::Vector3i(1, 0, 0) + Eigen::Vector3i(0, -1, 0) + Eigen::Vector3i(0, 0, 1);
  vec3_u.normalize();
  Eigen::Vector3i
      vec4_u = Eigen::Vector3i(-1, 0, 0) + Eigen::Vector3i(0, -1, 0) + Eigen::Vector3i(0, 0, 1);
  vec4_u.normalize();

  VecVec3i shell_main_pos;
  shell_main_pos = {position_v + Eigen::Vector3i(1, 0, 0) * radius_v,
                    position_v - Eigen::Vector3i(1, 0, 0) * radius_v,
                    position_v + Eigen::Vector3i(0, 1, 0) * radius_v,
                    position_v - Eigen::Vector3i(0, 1, 0) * radius_v,
                    position_v + Eigen::Vector3i(0, 0, 1) * radius_v,
                    position_v - Eigen::Vector3i(0, 0, 1) * radius_v,
                    position_v + vec1_u * radius_v, position_v - vec1_u * radius_v,
                    position_v + vec2_u * radius_v, position_v - vec2_u * radius_v,
                    position_v + vec3_u * radius_v, position_v - vec3_u * radius_v,
                    position_v + vec4_u * radius_v, position_v - vec4_u * radius_v};

  set3i affected_nodes_list = getCollisionNodeList(shell_main_pos, node_level_);

  LOG(INFO)<< "affected node list size " << affected_nodes_list.size();

  for (set3i::iterator it_morton_code = affected_nodes_list.begin(); it_morton_code != affected_nodes_list.end(); it_morton_code++) {
    if (!isNodeFree(*it_morton_code, node_level_)){
      for (VecVec3i::iterator it_point = shell_main_pos.begin(); it_point != shell_main_pos.end(); it_point++) {
        set3i free_nodes;
        if(se::keyops::encode(it_point->x(), it_point->y(), it_point->z(), node_level_, octree_ptr_->max_level()) ==*it_morton_code){
          LOG(INFO)<< "morton code "<< *it_morton_code << " point "<< (*it_point).format(InLine);

          if(node_level_ != octree_ptr_->leaf_level()){
            key_t morton_tmp = se::keyops::encode(it_point->x(), it_point->y(), it_point->z(), node_level_+1, octree_ptr_->max_level());
            if(!isNodeFree(morton_tmp, node_level_+1)){
              if (!isVoxelFree(*it_point))
                return false;
            }else{
              free_nodes.insert(morton_tmp);
            }

          }

        }
      }
    }
  }

  return true;
  LOG(INFO) << "should display this";

  for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); it++) {
    if (!isVoxelFree(*it))

      return false;
  }



  VecVec3i shell_sub_pos;
  shell_sub_pos = {position_v + vec1_u * radius_v, position_v - vec1_u * radius_v,
                   position_v + vec2_u * radius_v, position_v - vec2_u * radius_v,
                   position_v + vec3_u * radius_v, position_v - vec3_u * radius_v,
                   position_v + vec4_u * radius_v, position_v - vec4_u * radius_v};
  for (VecVec3i::iterator it = shell_sub_pos.begin(); it != shell_sub_pos.end(); ++it) {
    if (!isVoxelFree(*it))
      return false;
  }

  return true;
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isVoxelFree(const Eigen::Vector3i &point_v) const {

  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;
  bool is_voxel_block;
  if(point_v.x()<0|| point_v.y() <0 || point_v.z()<0 ||
    point_v.x() > octree_ptr_->size() || point_v.y() > octree_ptr_->size() || point_v.z() > octree_ptr_->size()){
    return false;
  }
  octree_ptr_->fetch_octant(point_v.x(), point_v.y(), point_v.z(), node, is_voxel_block);
  if (is_voxel_block) {
    block = static_cast<se::VoxelBlock<FieldType> *> (node);
    if (block->data(point_v).x < SURF_BOUNDARY) {
      DLOG(INFO) << "free at "
      << (point_v.cast<float>() * voxel_dim_).format(InLine) << " state "
      << block->data(point_v).st << " prob "<< block->data(point_v).x ;

      return true;
    } else {
      DLOG(INFO)  << "collision at " << point_v.format(InLine) << " state "<<  block->data(point_v).st <<
      " prob " << block->data(point_v).x ;
      return false;
    }

  } else {
    // TODO without up propagation, ignore unknown nodes
    if (octree_ptr_->get(se::keyops::decode(node->code_)).x > 0.f) {
      // const Eigen::Vector3i pos = se::keyops::decode(node->code_);
      // LOG(INFO) << "collision at node "
      // << (pos.cast<float>() * voxel_dim_).format(InLine) << std::endl;
      return false;
    } else
      return true;
  }

}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isNodeFree(const key_t morton_code, const int node_level) const {
  se::Node<FieldType> *node = nullptr;
  se::VoxelBlock<FieldType> *block = nullptr;

  if(node_level == octree_ptr_->leaf_level()){
    block = octree_ptr_->fetch(morton_code);
    LOG(INFO) << "VB morton "<< morton_code<< " prob " << block->data(VoxelBlock<OFusion>::buff_size - 1).x <<
    " state " << block->data(VoxelBlock<OFusion>::buff_size - 1).st;
    if(block->data(VoxelBlock<OFusion>::buff_size - 1).x < SURF_BOUNDARY){
      return true;
    } else {
      return false;
    }

  } else{
    Eigen::Vector3i coord = se::keyops::decode(morton_code);
    node = octree_ptr_->fetch_octant(coord.x(), coord.y(), coord.z(), node_level_);
    const unsigned int id = se::child_id(morton_code, node_level_, octree_ptr_->max_level());
    auto& data = node->parent()->value_[id];
    LOG(INFO) << "Node morton " << morton_code << " level "<< node_level<<" prob "<< data.x << " state " << data.st;
    if(data.x < SURF_BOUNDARY){
      return true;
    }else{
      return false;
    }
  }

}


template<typename FieldType>
int CollisionCheckerV<FieldType>::getNodeLevel(const int object_size_v){

  int node_level= 0;
  int side = 1 << (octree_ptr_->max_level()  - node_level);

  for(int i  = 1; i <= octree_ptr_->leaf_level() ; ++i){
    int next_side = 1 << (octree_ptr_->max_level() - i);

    if ( next_side > object_size_v ){
      side = next_side;
      node_level = i;
    }
  }
  return node_level;
}

template<typename FieldType>
set3i CollisionCheckerV<FieldType>::getCollisionNodeList(const VecVec3i& point_list, const int node_level) const {
  set3i morton_code_list;

  for(const auto& point : point_list){
    Node<FieldType> * node = octree_ptr_->fetch_octant(point.x(), point.y(), point.z(), node_level);
    if(node == nullptr){
      DLOG(INFO) << "node null ptr";
      continue;
    }
    key_t code = node->code_;
    morton_code_list.insert(code);
    DLOG(INFO) << "code "<< node->code_ << " coord " << se::keyops::decode(node->code_).format(InLine) <<
    " point " << point.format(InLine);
    const unsigned int id = se::child_id(node->code_, se::keyops::level(node->code_), octree_ptr_->max_level());
    if(node->parent()){
       auto data = node->parent()->value_[id];
       DLOG(INFO) << "data " << data.x;
    }else if (node->parent() ==nullptr){
      DLOG(INFO) << "null parent";
    }
  }
  return morton_code_list;
}

//   !!! took code snippets from DubinsSateSpace MotionValidator
template<typename FieldType>
bool CollisionCheckerV<FieldType>::isSegmentFlightCorridorSkeletonFree(const Eigen::Vector3i &start,
                                                                       const Eigen::Vector3i &end,
                                                                       const int r_min_v,
                                                                       const int r_max_v) const {
  // std::cout<< "VOXELTEST S " << start.format(InLine) << " " << end.format(InLine)<< std::endl;
  if (start == end)
    return true;
  /**
   * Set up the line-of-sight connection
   */
  const int seg_prec_v = std::ceil(planning_params_.skeleton_sample_precision / voxel_dim_); //
  // Precision at which
  // to sample the connection [m/voxel]

  VecVec3i segment_flight_corridor;
  const Eigen::Vector3i corridor_axis_u = (end - start) / (end - start).norm();
  const Eigen::Vector3i start_corridor_v = start - r_max_v * corridor_axis_u;
  const Eigen::Vector3i end_corridor_v = end + r_max_v * corridor_axis_u;
// std::cout << "start_corridor_v" << start_corridor_v.format(InLine)<<
  // " end_corridor_v " << end_corridor_v.format(InLine) << std::endl;
  const Eigen::Vector3i vec_seg_connection_v =
      end_corridor_v - start_corridor_v; // The vector in [voxel] connecting the start and end
  // position
  // TODO same as corridor axis u
  const Eigen::Vector3i vec_seg_direction_u = vec_seg_connection_v
      / vec_seg_connection_v.norm(); // The vector in [voxel] connecting the start and end position
  const int num_axial_subpos =
      vec_seg_connection_v.norm() / seg_prec_v; // Number of sub points along the line to be checked
  // LOG(INFO)<< "num disc to check " << num_axial_subpos;
  if (!isSphereSkeletonFree(end, r_max_v)) {
    // DLOG(INFO)<< "SPHERE F" ;
    return false;
  }
  // TODO implement checkLine
  // check the central line from start to end
  // std::cout << "Check central line"<< std::endl;
  if (!isLineFree(start_corridor_v,
                  vec_seg_connection_v,
                  num_axial_subpos)) { // Check if the line-of-sight connection is collision free
    // DLOG(INFO) << "line not free at " << (start_corridor_v).format(InLine);
    return false;
  }
  /**
   * Get cylinder extrema in horizontal and vertical direction
   */
  Eigen::Vector3i vec_z_u = Eigen::Vector3i(0, 0, 1);
  Eigen::Vector3i vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  Eigen::Vector3i vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3i(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3i(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3i(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }

// check 4 lines along cylinder surface from start to end
  VecVec3i shell_main_pos;
  shell_main_pos =
      {start_corridor_v + vec_horizontal_u * r_max_v, start_corridor_v - vec_horizontal_u * r_max_v,
       start_corridor_v + vec_vertical_u * r_max_v, start_corridor_v - vec_vertical_u * r_max_v};
  // std::cout << "Check 4 lines along cylinder"<< std::endl;
  for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); ++it) {

    if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
      // DLOG(INFO) << "line not free at " << (*it).format(InLine);
      return false;
    }
  }

  // number of points in a quater of the circumfence
  // std::cout << "Check 4 lines along circumfence"<< std::endl;
  const int num_circ_shell_subpos = static_cast<float>(r_max_v) * M_PI / (2.f * seg_prec_v);

  if (num_circ_shell_subpos > 1) {

    AlignedQueueTupleVec3i circ_shell_pos; // Not sure yet
    circ_shell_pos.push(std::make_tuple(vec_vertical_u,
                                        vec_horizontal_u,
                                        1,
                                        num_circ_shell_subpos - 1));

    while (!circ_shell_pos.empty()) {

      std::tuple<Eigen::Vector3i, Eigen::Vector3i, int, int> x = circ_shell_pos.front();
      const int mid = std::round(static_cast<float>(std::get<2>(x) + std::get<3>(x)) / 2.f);
      Eigen::Vector3f vec1_u = (std::get<0>(x).cast<float>() + std::get<1>(x).cast<float>()) / 2.f;
      vec1_u.normalize();
      Eigen::Vector3f vec2_u = (std::get<0>(x).cast<float>() - std::get<1>(x).cast<float>()) / 2.f;
      vec2_u.normalize();
      VecVec3i circ_shell_sub_pos;
      circ_shell_sub_pos = {start_corridor_v + (vec1_u * r_max_v).cast<int>(),
                            start_corridor_v - (vec1_u * r_max_v).cast<int>(),
                            start_corridor_v + (vec2_u * r_max_v).cast<int>(),
                            start_corridor_v - (vec2_u * r_max_v).cast<int>()};
      for (VecVec3i::iterator it = circ_shell_sub_pos.begin(); it != circ_shell_sub_pos.end();
           ++it) {
        if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
          return false;
        }
      }

      circ_shell_pos.pop();

      if (std::get<2>(x) < mid)
        circ_shell_pos.push(std::make_tuple(std::get<0>(x),
                                            vec1_u.cast<int>(),
                                            std::get<2>(x),
                                            mid - 1));
      if (std::get<3>(x) > mid)
        circ_shell_pos.push(std::make_tuple(vec1_u.cast<int>(),
                                            std::get<1>(x),
                                            mid + 1,
                                            std::get<3>(x)));
    }
  }

  // num  lines in cylinder . cross and along circumfence
  const int num_radial_subpos = r_max_v / seg_prec_v;
  if (num_radial_subpos > 1) {

    std::queue<std::pair<int, int>> radial_pos; // Not sure yet
    radial_pos.push(std::make_pair(1, num_radial_subpos - 1));

    while (!radial_pos.empty()) {
      std::pair<int, int> y = radial_pos.front();
      const float mid = static_cast<float>(y.first + y.second) / 2.f;
      const int r_v = std::round(mid / num_radial_subpos * r_max_v);
      VecVec3i circ_main_pos;
      circ_main_pos =
          {start_corridor_v + vec_horizontal_u * r_v, start_corridor_v - vec_horizontal_u * r_v,
           start_corridor_v + vec_vertical_u * r_v, start_corridor_v - vec_vertical_u * r_v};
      // std::cout << "Check lines IN cylinder"<< std::endl;
      for (VecVec3i::iterator it = circ_main_pos.begin(); it != circ_main_pos.end(); ++it) {

        if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
          return false;
        }
      }

      const int num_circ_subpos = static_cast<float>(r_v) * M_PI / (2.f * seg_prec_v);

      if (num_circ_subpos > 1) {

        AlignedQueueTupleVec3i circ_sub_pos; // Not sure yet
        circ_sub_pos.push(std::make_tuple(vec_vertical_u,
                                          vec_horizontal_u,
                                          1,
                                          num_circ_subpos - 1));

        while (!circ_sub_pos.empty()) {

          std::tuple<Eigen::Vector3i, Eigen::Vector3i, int, int> x = circ_sub_pos.front();
          int mid = std::round(static_cast<float>(std::get<2>(x) + std::get<3>(x)) / 2.f);
          Eigen::Vector3f
              vec1_u = (std::get<0>(x).cast<float>() + std::get<1>(x).cast<float>()) / 2.f;
          vec1_u.normalize();
          Eigen::Vector3f
              vec2_u = (std::get<0>(x).cast<float>() - std::get<1>(x).cast<float>()) / 2.f;
          vec2_u.normalize();
          VecVec3i sub_starts;
          sub_starts = {start_corridor_v + (vec1_u * r_v).cast<int>(),
                        start_corridor_v - (vec1_u * r_v).cast<int>(),
                        start_corridor_v + (vec2_u * r_v).cast<int>(),
                        start_corridor_v - (vec2_u * r_v).cast<int>()};
          for (VecVec3i::iterator it = sub_starts.begin(); it != sub_starts.end(); ++it) {

            if (!isLineFree(*it, vec_seg_connection_v, num_axial_subpos)) {
              // DLOG(INFO) << "line not free at " << (*it).format(InLine);
              return false;
            }
          }

          circ_sub_pos.pop();

          if (std::get<2>(x) < mid)
            circ_sub_pos.push(std::make_tuple(std::get<0>(x),
                                              vec1_u.cast<int>(),
                                              std::get<2>(x),
                                              mid - 1));
          if (std::get<3>(x) > mid)
            circ_sub_pos.push(std::make_tuple(vec1_u.cast<int>(),
                                              std::get<1>(x),
                                              mid + 1,
                                              std::get<3>(x)));
        }
      }

      radial_pos.pop();

      if (y.first < mid)
        radial_pos.push(std::make_pair(y.first, mid - 1));
      if (y.second > mid)
        radial_pos.push(std::make_pair(mid + 1, y.second));
    }
  }

  return true;
}

template<typename FieldType>
bool CollisionCheckerV<FieldType>::isLineFree(const Eigen::Vector3i &start_v,
                                              const Eigen::Vector3i &connection_v,
                                              const int num_subpos) const {

  std::queue<std::pair<int, int>> line_pos;
  line_pos.push(std::make_pair(1, num_subpos - 1));
  // Repeatedly subdivide the path segment in the middle (and check the middle)
  while (!line_pos.empty()) {
    std::pair<int, int> x = line_pos.front();

    float mid = (x.first + x.second) / 2.f;
    Eigen::Vector3f shift = mid / static_cast<float>(num_subpos) * connection_v.cast<float>();


    // Compute midpoint
    // Eigen::Vector3i position_v = Eigen::Vector3i(std::round(shift.x()), std::round(shift.y()), std::round(shift.z()))+ start_v;
    Eigen::Vector3i position_v = shift.cast<int>() + start_v;

    if (!isVoxelFree(position_v)) {
      // std::cout <<"W: "<< position_v.format(InLine)<< " 1"<< std::endl;
      return false;
    }
    // std::cout <<"E: "<< position_v.format(InLine)<< " 1"<< std::endl;
    line_pos.pop();

    if (x.first < mid)
      line_pos.push(std::make_pair(x.first, mid - 1));
    if (x.second > mid)
      line_pos.push(std::make_pair(mid + 1, x.second));
  }

  return true;
}

}
}
#endif //SUPEREIGHT_COLLISION_CHECKER_VOXEL_HPP
