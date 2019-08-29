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
#include "se/path_planning/planning_history.hpp"
#include "se/utils/memory_pool.hpp"

#include "se/path_planner_ompl.hpp"
#include "se/ompl/collision_checker_voxel.hpp"
#include "se/ompl/motion_validator_occupancy_skeleton.hpp"

#include "gtest/gtest.h"

class CollisionUnitTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_ = std::make_shared<se::Octree<OFusion> >();
    std::string
        filename = "/home/anna/exploration_ws/src/supereight/se_ompl/test/collision/w_box.bin";
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

  std::shared_ptr<se::Octree<OFusion> > tree_;
  int max_depth_;
  int leaves_level_;
  int dim_;
  Planning_Configuration planner_config_;

};

TEST_F(CollisionUnitTest, CorridorCheckPass) {
  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  Eigen::Vector3i start = {89, 75, 72};
  Eigen::Vector3i end = {95, 75, 72};
  // WHEN: checking for collision of the segment corridor in free space
  bool is_collision_free = collision_checker->isSegmentFlightCorridorSkeletonFree(start, end, 0, 3);

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
  bool is_collision_free = collision_checker->isSegmentFlightCorridorSkeletonFree(start, end, 0, 4);

  // THEN: the collision check should fail,
  EXPECT_FALSE(is_collision_free);

}

TEST_F(CollisionUnitTest, PathPlanningPass) {
 //GIVEN a octree map , a path planner, a collision checker for the map, start and end point of a path are free
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  auto path_planner_ompl_ptr = aligned_shared<se::exploration::PathPlannerOmpl<OFusion> >(tree_,
                                                                                          collision_checker,
                                                                                          planner_config_,
                                                                                          12.1f);
  set3i free_map = createMap3i(block_buffer_base);
  Eigen::Vector3i lower_bound ;
  Eigen::Vector3i upper_bound ;
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
  Eigen::Vector3i lower_bound ;
  Eigen::Vector3i upper_bound ;
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
  Eigen::Vector3i lower_bound ;
  Eigen::Vector3i upper_bound ;
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


  int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
  int node_level = collision_checker->getNodeLevel( object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level());

  object_size_v = 10;
  node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level()-1);

  object_size_v = 55;
  node_level = collision_checker->getNodeLevel(object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level()-3);


}



TEST_F(CollisionUnitTest, GetMortonCode) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // start is free and end is occupied
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  VecVec3i point_list;
  point_list = {{80,80,72}, {90,60,72}};
  int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
  int node_level = collision_checker->getNodeLevel( object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level());
  set3i morton_set_leaf = collision_checker->getCollisionNodeList(node_level,point_list );


  object_size_v = 10;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node = collision_checker->getCollisionNodeList(node_level,point_list );
  EXPECT_EQ(node_level, tree_->leaf_level()-1);


  object_size_v = 55;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node2 = collision_checker->getCollisionNodeList(node_level,point_list );
  EXPECT_EQ(node_level, tree_->leaf_level()-3);


}

