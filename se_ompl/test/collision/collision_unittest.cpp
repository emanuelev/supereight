#include <fstream>
#include <string>
#include <random>
#include "se/octree.hpp"
#include "se/node_iterator.hpp"
#include "se/node.hpp"
#include "se/utils/math_utils.h"
#include "se/utils/morton_utils.hpp"
#include "se/utils/eigen_utils.h"
#include "se/config.h"
#include "se/planner_config.h"
#include "se/path_planning/exploration_utils.hpp"
// #include "se/path_planning/planning_history.hpp"
#include "se/utils/memory_pool.hpp"

#include "se/path_planner_ompl.hpp"
#include "se/ompl/collision_checker_voxel.hpp"
#include "se/ompl/motion_validator_occupancy_skeleton.hpp"

#include "gtest/gtest.h"

class CollisionUnitTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_ = std::make_shared<se::Octree<OFusion> >();
    std::string filename =
        "/home/anna/exploration_ws/src/supereight/se_ompl/test/collision/w_box_multires.bin";
    tree_->load(filename);
    std::cout << "file loaded" << std::endl;
    max_depth_ = log2(tree_->size());
    leaves_level_ = tree_->leaf_level();
    dim_ = tree_->voxelDim();
    planner_config_ = getDefaultPlanningConfig();

  }

  set3i createMap3i(se::MemoryPool<se::VoxelBlock<OFusion> > &block_buffer) {
    set3i morton_set;
    for (int i = 0; i < block_buffer.size(); i++) {
      // const Eigen::Vector3i block_coord = block_buffer[i]->coordinates();
      const key_t morton_code = block_buffer[i]->code_;
      morton_set.emplace(morton_code);
    }
    return morton_set;
  }

  VecVec3i getSphereSkeletonPoints(const Eigen::Vector3i &position_v, const int radius_v) const {

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
    return shell_main_pos;
  }

  std::shared_ptr<se::Octree<OFusion> > tree_;
  int max_depth_;
  int leaves_level_;
  float dim_;
  Planning_Configuration planner_config_;

};

TEST_F(CollisionUnitTest, CorridorCheckPass) {
  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  Eigen::Vector3i start = {90, 75, 72};
  Eigen::Vector3i end = {75, 75, 72};
  // WHEN: checking for collision of the segment corridor in free space
  bool is_collision_free = collision_checker->isSegmentFlightCorridorSkeletonFree(start, end, 3);

  // THEN: the corridor is collision free
  EXPECT_TRUE(is_collision_free);

}

TEST_F(CollisionUnitTest, CorridorCheckFail) {

  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  Eigen::Vector3i start = {60, 50, 72};
  Eigen::Vector3i end = {80, 50, 72};
// WHEN: checking for collision of the segment corridor with a wall between the two points
  bool is_collision_free = collision_checker->isSegmentFlightCorridorSkeletonFree(start, end, 4);

  // THEN: the collision check should fail,
  EXPECT_FALSE(is_collision_free);

}

TEST_F(CollisionUnitTest, PathPlanningPass) {
  //GIVEN a octree map , a path planner, a collision checker for the map, start and end point of a path are free
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  planner_config_.robot_safety_radius_max = 0.5;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  auto path_planner_ompl_ptr = aligned_shared<se::exploration::PathPlannerOmpl<OFusion> >(tree_,
                                                                                          collision_checker,
                                                                                          planner_config_,
                                                                                          12.1f);
  set3i free_map = createMap3i(block_buffer_base);
  Eigen::Vector3i lower_bound;
  Eigen::Vector3i upper_bound;
  getFreeMapBounds(tree_, free_map, lower_bound, upper_bound);
  bool setup_planner = path_planner_ompl_ptr->setupPlanner(lower_bound, upper_bound);
  Eigen::Vector3i start = {89, 75, 72};
  Eigen::Vector3i end = {95, 75, 72};
  // WHEN planning a path in free space
  int solution_status = path_planner_ompl_ptr->planPath(start, end);

  // THEN there is an exact solution
  EXPECT_EQ(solution_status, 1);

}

TEST_F(CollisionUnitTest, PathPlanningAroundWall) {
// GIVEN a octree map , a path planner, a collision checker for the map, start and end point of a path are free
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  auto path_planner_ompl_ptr = aligned_shared<se::exploration::PathPlannerOmpl<OFusion> >(tree_,
                                                                                          collision_checker,
                                                                                          planner_config_,
                                                                                          12.1f);
  set3i free_map = createMap3i(block_buffer_base);
  Eigen::Vector3i lower_bound;
  Eigen::Vector3i upper_bound;
  getFreeMapBounds(tree_, free_map, lower_bound, upper_bound);
  bool setup_planner = path_planner_ompl_ptr->setupPlanner(lower_bound, upper_bound);
  Eigen::Vector3i start = {80, 80, 72};
  Eigen::Vector3i end = {90, 50, 72};
  // WHEN planning a path around a wall
  int solution_status = path_planner_ompl_ptr->planPath(start, end);

  // THEN there is an exact solution
  EXPECT_EQ(solution_status, 1);

}

TEST_F(CollisionUnitTest, ApproximateSolution) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // start is free and end is occupied
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  auto path_planner_ompl_ptr = aligned_shared<se::exploration::PathPlannerOmpl<OFusion> >(tree_,
                                                                                          collision_checker,
                                                                                          planner_config_,
                                                                                          12.1f);
  set3i free_map = createMap3i(block_buffer_base);
  Eigen::Vector3i lower_bound;
  Eigen::Vector3i upper_bound;
  getFreeMapBounds(tree_, free_map, lower_bound, upper_bound);
  bool setup_planner = path_planner_ompl_ptr->setupPlanner(lower_bound, upper_bound);
  Eigen::Vector3i start = {80, 80, 72};
  Eigen::Vector3i end = {90, 60, 72};

  // WHEN planning a path around a wall
  int solution_status = path_planner_ompl_ptr->planPath(start, end);

  // THEN there is only an approxiamted solution
  EXPECT_EQ(solution_status, 2);

}

TEST_F(CollisionUnitTest, SideLengthCheck) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // start is free and end is occupied
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level());

  object_size_v = 10;
  node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level() - 1);

  object_size_v = 55;
  node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level() - 3);

}

TEST_F(CollisionUnitTest, GetMortonCode) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // start is free and end is occupied
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  VecVec3i point_list;
  point_list = {{80, 80, 72}, {90, 60, 72}};
  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level());
  set3i morton_set_leaf = collision_checker->getCollisionNodeList(point_list, node_level);

  object_size_v = 10;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node = collision_checker->getCollisionNodeList(point_list, node_level);
  EXPECT_EQ(node_level, tree_->leaf_level() - 1);

  object_size_v = 27;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node2 = collision_checker->getCollisionNodeList(point_list, node_level);
  EXPECT_EQ(node_level, tree_->leaf_level() - 2);

  object_size_v = 55;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node3 = collision_checker->getCollisionNodeList(point_list, node_level);
  EXPECT_EQ(node_level, tree_->leaf_level() - 3);


  // check that parent node has the highest occupancy child probability
  float prob[8];
  for (int i = 0; i < 8; i++) {
    prob[i] = tree_->root()->child(5)->value_[i].x;
  }
  float *parent_prob;
  parent_prob = std::max_element(prob, prob + 8);
  EXPECT_FLOAT_EQ(tree_->root()->value_[5].x, *parent_prob);

  for (int i = 0; i < 8; i++) {
    prob[i] = tree_->root()->child(5)->child(2)->value_[i].x;
  }
  parent_prob;
  parent_prob = std::max_element(prob, prob + 8);
  EXPECT_FLOAT_EQ(tree_->root()->child(5)->value_[2].x, *parent_prob);

  for (int i = 0; i < 8; i++) {
    prob[i] = tree_->root()->child(5)->child(2)->child(1)->value_[i].x;
  }
  parent_prob;
  parent_prob = std::max_element(prob, prob + 8);
  EXPECT_FLOAT_EQ(tree_->root()->child(5)->child(2)->value_[1].x, *parent_prob);

  // DLOG(INFO) << "Level 1: " <<tree_->root()->value_[5].x;
  // DLOG(INFO) << tree_->root()->child(5)->value_[0].x << ", " <<
  // tree_->root()->child(5)->value_[1].x << ",  " <<
  // tree_->root()->child(5)->value_[2].x << ",  " <<
  // tree_->root()->child(5)->value_[3].x << ",  " <<
  // tree_->root()->child(5)->value_[4].x << ",  " <<
  // tree_->root()->child(5)->value_[5].x << ",  " <<
  // tree_->root()->child(5)->value_[6].x <<  ", "  <<
  // tree_->root()->child(5)->value_[7].x ;

  // DLOG(INFO) <<"Level 2: " << tree_->root()->child(5)->value_[2].x;
  // DLOG(INFO) << tree_->root()->child(5)->child(2)->value_[0].x << ", " <<
  // tree_->root()->child(5)->child(2)->value_[1].x << ",  " <<
  // tree_->root()->child(5)->child(2)->value_[2].x << ",  " <<
  // tree_->root()->child(5)->child(2)->value_[3].x << ",  " <<
  // tree_->root()->child(5)->child(2)->value_[4].x << ",  " <<
  // tree_->root()->child(5)->child(2)->value_[5].x << ",  " <<
  // tree_->root()->child(5)->child(2)->value_[6].x << ", "<<
  // tree_->root()->child(5)->child(2)->value_[7].x ;


  // DLOG(INFO) <<"Level 3: "<<  tree_->root()->child(5)->child(2)->value_[1].x;
  // DLOG(INFO) <<  tree_->root()->child(5)->child(2)->child(1) ->value_[0].x<< ", " <<
  // tree_->root()->child(5)->child(2)->child(1) ->value_[1].x<< ", " <<
  // tree_->root()->child(5)->child(2)->child(1) ->value_[2].x<< ", " <<
  // tree_->root()->child(5)->child(2)->child(1) ->value_[3].x<< ", " <<
  // tree_->root()->child(5)->child(2)->child(1) ->value_[4].x<< ", " <<
  // tree_->root()->child(5)->child(2)->child(1) ->value_[5].x<< ", " <<
  // tree_->root()->child(5)->child(2)->child(1) ->value_[6].x<< ", " <<
  // tree_->root()->child(5)->child(2)->child(1) ->value_[7].x;
}

TEST_F(CollisionUnitTest, CollisionCheckSpherePass) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // sphere center and radius
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_max = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_max / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level() - 1);

  Eigen::Vector3i center = {80, 80, 72};
  key_t morton = se::keyops::encode(80, 80, 72, 3, tree_->max_level());
  int level = se::keyops::level(morton);
  LOG(INFO) << "level " << level;
  bool is_collision_free = collision_checker->isSphereSkeletonFree(center,
                                                                   planner_config_.robot_safety_radius_max
                                                                       / tree_->voxelDim());

  EXPECT_TRUE(is_collision_free);

}
TEST_F(CollisionUnitTest, CollisionCheckSphereOldPass) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // sphere center and radius
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_max = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_max / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level() - 1);

  Eigen::Vector3i center = {80, 80, 72};
  bool is_collision_free = collision_checker->isSphereSkeletonFreeCand(center,
                                                                       planner_config_.robot_safety_radius_max
                                                                           / tree_->voxelDim());

  EXPECT_TRUE(is_collision_free);

}

TEST_F(CollisionUnitTest, CollisionCheckSphereFail) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  //sphere center and radius
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_max = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_max / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level() - 1);

  Eigen::Vector3i center = {95, 80, 72};
  bool is_collision_free = collision_checker->isSphereSkeletonFree(center,
                                                                   planner_config_.robot_safety_radius_max
                                                                       / tree_->voxelDim());

  EXPECT_FALSE(is_collision_free);

}

TEST_F(CollisionUnitTest, CollisionCheckSphereOldFail) {
// GIVEN a octree map , a path planner, a collision checker for the map, sphere center and radius
  auto &block_buffer_base = tree_->getBlockBuffer();

  planner_config_.robot_safety_radius_max = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_max / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level() - 1);

  Eigen::Vector3i center = {95, 80, 72};
  bool is_collision_free = collision_checker->isSphereSkeletonFreeCand(center,
                                                                       planner_config_.robot_safety_radius_max
                                                                           / tree_->voxelDim());

  EXPECT_FALSE(is_collision_free);

}

TEST_F(CollisionUnitTest, CollisionCheckSphereFailVB) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // sphere center and radius
  auto &block_buffer_base = tree_->getBlockBuffer();
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level());

  Eigen::Vector3i center = {98, 60, 72};
  bool is_collision_free = collision_checker->isSphereSkeletonFree(center,
                                                                   planner_config_.robot_safety_radius_max
                                                                       / tree_->voxelDim());

  EXPECT_FALSE(is_collision_free);

}

TEST_F(CollisionUnitTest, CollisionCheckSphereOldFailVB) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // sphere center and radius
  auto &block_buffer_base = tree_->getBlockBuffer();
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level());

  Eigen::Vector3i center = {98, 60, 72};
  bool is_collision_free = collision_checker->isSphereSkeletonFreeCand(center,
                                                                       planner_config_.robot_safety_radius_max
                                                                           / tree_->voxelDim());

  EXPECT_FALSE(is_collision_free);

}

TEST_F(CollisionUnitTest, GetLinePoints) {
  //GIVEN a octree map , a collision checker for the map,
  auto &block_buffer_base = tree_->getBlockBuffer();
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {89, 75, 72};
  const Eigen::Vector3i end = {80, 75, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;
  // WHEN: getting the line
  VecVec3i points = collision_checker->getLinePoints(start, vec_seg_connection_v, num_axial_subpos);
  const int size = points.size();
  // for(auto point : points){
  //   LOG(INFO)<< point.format(InLine);
  // }

  // THEN: the corridor line length to check is the num axial points +2
  EXPECT_EQ(size, num_axial_subpos + 2);

}

TEST_F(CollisionUnitTest, CheckSphereLeafLevel) {

  bool collision_free = true;

  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const Eigen::Vector3i center = {89, 75, 72};

  // WHEN: checking for collision of the sphere in free space
  VecVec3i points = getSphereSkeletonPoints(center,
                                            std::ceil(planner_config_.robot_safety_radius_min
                                                          / tree_->voxelDim()));
  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, true);

}
TEST_F(CollisionUnitTest, CheckSphereLeafLevelFail) {

  bool collision_free = true;

  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i center = {95, 80, 72};

  // WHEN: checking for collision of the segment corridor in free space
  // VecVec3i points = collision_checker->getLinePoints(start, vec_seg_connection_v, num_axial_subpos);
  VecVec3i points = getSphereSkeletonPoints(center,
                                            std::ceil(planner_config_.robot_safety_radius_min
                                                          / tree_->voxelDim()));
  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, false);

}
TEST_F(CollisionUnitTest, CheckSphereNodeLevel) {
  //GIVEN a octree map , a collision checker for the map, center and the radius of a sphere
  bool collision_free = true;
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_min = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const Eigen::Vector3i start = {89, 75, 72};
  // WHEN: checking for collision of the segment corridor in free space

  VecVec3i points = getSphereSkeletonPoints(start,
                                            std::ceil(planner_config_.robot_safety_radius_min
                                                          / tree_->voxelDim()));
  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, true);

}
TEST_F(CollisionUnitTest, CheckSphereNodeLevelFail) {
  //GIVEN a octree map , a collision checker for the map, center and the radius of a sphere
  bool collision_free = true;
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_min = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const Eigen::Vector3i center = {95, 80, 72};

  // WHEN: checking for collision of the segment corridor in free space

  VecVec3i points = getSphereSkeletonPoints(center,
                                            std::ceil(planner_config_.robot_safety_radius_min
                                                          / tree_->voxelDim()));
  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, false);

}
TEST_F(CollisionUnitTest, CheckLineNodeLevel) {
  //GIVEN a octree map , a collision checker for the map, center and the radius of a sphere
  bool collision_free = true;
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_min = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {90, 75, 72};
  const Eigen::Vector3i end = {75, 75, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;
  // WHEN: checking for collision of the segment corridor in free space
  VecVec3i points = collision_checker->getLinePoints(start, vec_seg_connection_v, num_axial_subpos);
  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, true);

}

TEST_F(CollisionUnitTest, CheckCorridorVBLevel) {
  //GIVEN a octree map , a collision checker for the map, center and the radius of a sphere
  bool collision_free = true;
  auto &block_buffer_base = tree_->getBlockBuffer();

  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {90, 75, 72};
  const Eigen::Vector3i end = {75, 75, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const Eigen::Vector3i vec_seg_direction_u = vec_seg_connection_v / vec_seg_connection_v.norm();
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;

  Eigen::Vector3i vec_z_u = Eigen::Vector3i(0, 0, 1);
  Eigen::Vector3i vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  LOG(INFO) << "vec " << vec_vertical_u.format(InLine);
  Eigen::Vector3i vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3i(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3i(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3i(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }
  int r_max_v = 3;
// check 4 lines along cylinder surface from start to end
  VecVec3i points;
  VecVec3i shell_main_pos;
  shell_main_pos = {start + vec_horizontal_u * r_max_v, start - vec_horizontal_u * r_max_v,
                    start + vec_vertical_u * r_max_v, start - vec_vertical_u * r_max_v};
  // std::cout << "Check 4 lines along cylinder"<< std::endl;
  for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); ++it) {
    VecVec3i
        points_new = collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
    points.insert(points.end(), points_new.begin(), points_new.end());
    LOG(INFO) << "points size " << points.size();
  }

  // WHEN: checking for collision of the segment corridor in free space

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, true);

}

TEST_F(CollisionUnitTest, CheckFullCorridorVBLevel) {
  //GIVEN a octree map , a collision checker for the map, center and the radius of a sphere
  bool collision_free = true;
  auto &block_buffer_base = tree_->getBlockBuffer();

  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {90, 75, 72};
  const Eigen::Vector3i end = {75, 75, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const Eigen::Vector3i vec_seg_direction_u = vec_seg_connection_v / vec_seg_connection_v.norm();
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;

  const int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  const int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  Eigen::Vector3i vec_z_u = Eigen::Vector3i(0, 0, 1);
  Eigen::Vector3i vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  // LOG(INFO) << "vec " << vec_vertical_u.format(InLine);
  Eigen::Vector3i vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3i(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3i(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3i(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }
  int r_max_v = 3;
  collision_free = collision_checker->isSphereSkeletonFree(end, r_max_v);

// check 4 lines along cylinder surface from start to end
  VecVec3i points;
  if (collision_free) {
    points = collision_checker->getLinePoints(start, vec_seg_connection_v, num_axial_subpos);
    collision_free = collision_checker->isCollisionFree(points);

    VecVec3i shell_main_pos;
    shell_main_pos = {start + vec_horizontal_u * r_max_v, start - vec_horizontal_u * r_max_v,
                      start + vec_vertical_u * r_max_v, start - vec_vertical_u * r_max_v};
    // std::cout << "Check 4 lines along cylinder"<< std::endl;
    for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); ++it) {
      VecVec3i points_new =
          collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
      points.insert(points.end(), points_new.begin(), points_new.end());
      // LOG(INFO)<< "points size "<< points.size();
    }
    // for(int i = 0 ; i <= tree_->leaf_level()- node_level ; i ++){
    collision_free = collision_checker->isCollisionFree(points);
    LOG(INFO) << "main shell free " << collision_free;
  }
  const int num_circ_shell_subpos = static_cast<float>(r_max_v) * M_PI / (2.f * seg_prec_v);

  if (num_circ_shell_subpos > 1 && collision_free) {
    points.clear();
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
      circ_shell_sub_pos =
          {start + (vec1_u * r_max_v).cast<int>(), start - (vec1_u * r_max_v).cast<int>(),
           start + (vec2_u * r_max_v).cast<int>(), start - (vec2_u * r_max_v).cast<int>()};
      for (VecVec3i::iterator it = circ_shell_sub_pos.begin(); it != circ_shell_sub_pos.end();
           ++it) {
        VecVec3i points_new =
            collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
        points.insert(points.end(), points_new.begin(), points_new.end());
        // LOG(INFO)<< "points size "<< points.size();
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
    collision_free = collision_checker->isCollisionFree(points);
  }
  LOG(INFO) << "circ shell free  " << collision_free;
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
      circ_main_pos = {start + vec_horizontal_u * r_v, start - vec_horizontal_u * r_v,
                       start + vec_vertical_u * r_v, start - vec_vertical_u * r_v};
      // std::cout << "Check lines IN cylinder"<< std::endl;
      for (VecVec3i::iterator it = circ_main_pos.begin(); it != circ_main_pos.end(); ++it) {

        VecVec3i points_new =
            collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
        points.insert(points.end(), points_new.begin(), points_new.end());
        // LOG(INFO)<< "points size "<< points.size();
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
          sub_starts = {start + (vec1_u * r_v).cast<int>(), start - (vec1_u * r_v).cast<int>(),
                        start + (vec2_u * r_v).cast<int>(), start - (vec2_u * r_v).cast<int>()};
          for (VecVec3i::iterator it = sub_starts.begin(); it != sub_starts.end(); ++it) {
            VecVec3i points_new =
                collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
            points.insert(points.end(), points_new.begin(), points_new.end());
            // LOG(INFO)<< "points size "<< points.size();
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
    collision_free = collision_checker->isCollisionFree(points);
  }

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, true);

}

TEST_F(CollisionUnitTest, CheckCorridorNodeLevelFail) {
  //GIVEN a octree map , a collision checker for the map, center and the radius of a sphere
  bool collision_free = true;
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_min = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {60, 50, 72};
  const Eigen::Vector3i end = {80, 50, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const Eigen::Vector3i vec_seg_direction_u = vec_seg_connection_v / vec_seg_connection_v.norm();
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;

  Eigen::Vector3i vec_z_u = Eigen::Vector3i(0, 0, 1);
  Eigen::Vector3i vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  LOG(INFO) << "vec " << vec_vertical_u.format(InLine);
  Eigen::Vector3i vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3i(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3i(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3i(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }
  int r_max_v = 3;
// check 4 lines along cylinder surface from start to end
  VecVec3i points;
  VecVec3i shell_main_pos;
  shell_main_pos = {start + vec_horizontal_u * r_max_v, start - vec_horizontal_u * r_max_v,
                    start + vec_vertical_u * r_max_v, start - vec_vertical_u * r_max_v};
  // std::cout << "Check 4 lines along cylinder"<< std::endl;
  for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); ++it) {
    VecVec3i
        points_new = collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
    points.insert(points.end(), points_new.begin(), points_new.end());
    LOG(INFO) << "points size " << points.size();
  }

  // WHEN: checking for collision of the segment corridor in free space

  int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;

  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, false);

}

TEST_F(CollisionUnitTest, CheckFullCorridorNodeLevelFail) {
  //GIVEN a octree map , a collision checker for the map, center and the radius of a sphere
  bool collision_free = true;
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius_min = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {60, 50, 72};
  const Eigen::Vector3i end = {80, 50, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const Eigen::Vector3i vec_seg_direction_u = vec_seg_connection_v / vec_seg_connection_v.norm();
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;
  const int object_size_v = std::ceil(planner_config_.robot_safety_radius_min / tree_->voxelDim()) * 2;
  const int node_level = collision_checker->getNodeLevel(object_size_v);
  mapvec3i affected_nodes_list;

  Eigen::Vector3i vec_z_u = Eigen::Vector3i(0, 0, 1);
  Eigen::Vector3i vec_vertical_u = vec_seg_direction_u.cross(vec_z_u);
  // LOG(INFO) << "vec " << vec_vertical_u.format(InLine);
  Eigen::Vector3i vec_horizontal_u;
  if (vec_vertical_u == Eigen::Vector3i(0, 0, 0)) {
    vec_vertical_u = Eigen::Vector3i(1, 0, 0);
    vec_horizontal_u = Eigen::Vector3i(0, 1, 0);
  } else {
    vec_vertical_u.normalize();
    vec_horizontal_u = vec_seg_direction_u.cross(vec_vertical_u);
    vec_horizontal_u.normalize();
  }
  int r_max_v = 3;
// check 4 lines along cylinder surface from start to end
  VecVec3i points;
  VecVec3i shell_main_pos;
  shell_main_pos = {start + vec_horizontal_u * r_max_v, start - vec_horizontal_u * r_max_v,
                    start + vec_vertical_u * r_max_v, start - vec_vertical_u * r_max_v};
  // std::cout << "Check 4 lines along cylinder"<< std::endl;
  for (VecVec3i::iterator it = shell_main_pos.begin(); it != shell_main_pos.end(); ++it) {
    VecVec3i
        points_new = collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
    points.insert(points.end(), points_new.begin(), points_new.end());
    LOG(INFO) << "points size " << points.size();
  }

  for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
    if (!points.empty()) {
      affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
      points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
      LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                << " level " << node_level + i;
    }
  }
  points = collision_checker->checkPointsAtVoxelLevel(points);
  if (points.size() != 0)
    collision_free = false;
  LOG(INFO) << "main shell free " << collision_free;

  const int num_circ_shell_subpos = static_cast<float>(r_max_v) * M_PI / (2.f * seg_prec_v);

  if (num_circ_shell_subpos > 1 && collision_free) {
    points.clear();
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
      circ_shell_sub_pos =
          {start + (vec1_u * r_max_v).cast<int>(), start - (vec1_u * r_max_v).cast<int>(),
           start + (vec2_u * r_max_v).cast<int>(), start - (vec2_u * r_max_v).cast<int>()};
      for (VecVec3i::iterator it = circ_shell_sub_pos.begin(); it != circ_shell_sub_pos.end();
           ++it) {
        VecVec3i points_new =
            collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
        points.insert(points.end(), points_new.begin(), points_new.end());
        LOG(INFO) << "points size " << points.size();
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

    for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
      if (!points.empty()) {
        affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
        points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
        LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                  << " level " << node_level + i;
      }
    }
    points = collision_checker->checkPointsAtVoxelLevel(points);
    if (points.size() != 0)
      collision_free = false;
  }

  LOG(INFO) << "circ shell free " << collision_free;
  // num  lines in cylinder . cross and along circumfence
  const int num_radial_subpos = r_max_v / seg_prec_v;
  if (num_radial_subpos > 1 && collision_free) {

    points.clear();
    std::queue<std::pair<int, int>> radial_pos; // Not sure yet
    radial_pos.push(std::make_pair(1, num_radial_subpos - 1));

    while (!radial_pos.empty()) {
      std::pair<int, int> y = radial_pos.front();
      const float mid = static_cast<float>(y.first + y.second) / 2.f;
      const int r_v = std::round(mid / num_radial_subpos * r_max_v);
      VecVec3i circ_main_pos;
      circ_main_pos = {start + vec_horizontal_u * r_v, start - vec_horizontal_u * r_v,
                       start + vec_vertical_u * r_v, start - vec_vertical_u * r_v};
      // std::cout << "Check lines IN cylinder"<< std::endl;
      for (VecVec3i::iterator it = circ_main_pos.begin(); it != circ_main_pos.end(); ++it) {

        VecVec3i points_new =
            collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
        points.insert(points.end(), points_new.begin(), points_new.end());
        // LOG(INFO)<< "points size "<< points.size();
      }

      const int num_circ_subpos = static_cast<float>(r_v) * M_PI / (2.f * seg_prec_v);

      if (num_circ_subpos > 1 && collision_free) {

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
          sub_starts = {start + (vec1_u * r_v).cast<int>(), start - (vec1_u * r_v).cast<int>(),
                        start + (vec2_u * r_v).cast<int>(), start - (vec2_u * r_v).cast<int>()};
          for (VecVec3i::iterator it = sub_starts.begin(); it != sub_starts.end(); ++it) {
            VecVec3i points_new =
                collision_checker->getLinePoints(*it, vec_seg_connection_v, num_axial_subpos);
            points.insert(points.end(), points_new.begin(), points_new.end());
            // LOG(INFO)<< "points size "<< points.size();
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
    for (int i = 0; i <= tree_->leaf_level() - node_level; i++) {
      if (!points.empty()) {
        affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level + i);
        points = collision_checker->checkPointsAtNodeLevel(affected_nodes_list, node_level + i);
        LOG(INFO) << "size " << affected_nodes_list.size() << " point size " << points.size()
                  << " level " << node_level + i;
      }
    }
    points = collision_checker->checkPointsAtVoxelLevel(points);
    if (points.size() != 0)
      collision_free = false;
  }





  // WHEN: checking for collision of the segment corridor in free space




  // THEN: the sphere is collision free
  EXPECT_EQ(collision_free, false);

}