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


#include "se/path_planner_ompl.hpp"
#include "se/ompl/collision_checker_voxel.hpp"
#include "se/ompl/motion_validator_occupancy_skeleton.hpp"

#include "gtest/gtest.h"



class CollisionUnitTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
  	tree_= std::make_shared<se::Octree<OFusion> >();
  std::string filename = "/home/anna/exploration_ws/src/supereight/se_ompl/test/collision/w_box.bin";
  tree_->load(filename);
  std::cout<< "file loaded" << std::endl;
  max_depth_ = log2(tree_->size());
  leaves_level_ = tree_->leaf_level();
  dim_ = tree_->voxelDim();
  planner_config_ = getDefaultPlanningConfig();

  }

  std::shared_ptr<se::Octree<OFusion> >  tree_;
  int max_depth_;
  int leaves_level_;
  int dim_;
  Planning_Configuration planner_config_;

};

TEST_F(CollisionUnitTest, CollisionFree) {

  auto& block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker = aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  std::cout <<"buffer size "<< block_buffer_base.size() << std::endl;
  Eigen::Vector3i start = {89, 75, 72};
  Eigen::Vector3i end = {80, 75, 72};
  std::cout << "prob " << tree_->get(start).x << " end prob " << tree_->get(end).x << std::endl;
  bool is_collision_free= collision_checker->isSegmentFlightCorridorSkeletonFree(start, end, 0, 3);
 EXPECT_TRUE(is_collision_free);

}

TEST_F(CollisionUnitTest, CollisionFail) {

  auto& block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker = aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  std::cout <<"buffer size "<< block_buffer_base.size() << std::endl;
  Eigen::Vector3i start = {50, 55, 72};
  Eigen::Vector3i end = {90, 41, 72};
  std::cout << "prob " << tree_->get(start).x << " end prob " << tree_->get(end).x << std::endl;
  bool is_collision_free= collision_checker->isSegmentFlightCorridorSkeletonFree(start, end, 0, 4);
 EXPECT_TRUE(is_collision_free);

}