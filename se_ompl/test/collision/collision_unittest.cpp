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
        filename = "/home/anna/exploration_ws/src/supereight/se_ompl/test/collision/w_box_multires.bin";
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
  float dim_;
  Planning_Configuration planner_config_;

};

TEST_F(CollisionUnitTest, CorridorCheckPass) {
  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);

  Eigen::Vector3i start = {89, 75, 72};
  Eigen::Vector3i end = {80, 75, 72};
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

// TEST_F(CollisionUnitTest, PathPlanningPass) {
//  //GIVEN a octree map , a path planner, a collision checker for the map, start and end point of a path are free
//   auto &block_buffer_base = tree_->getBlockBuffer();
//   // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
//   auto collision_checker =
//       aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
//   auto path_planner_ompl_ptr = aligned_shared<se::exploration::PathPlannerOmpl<OFusion> >(tree_,
//                                                                                           collision_checker,
//                                                                                           planner_config_,
//                                                                                           12.1f);
//   set3i free_map = createMap3i(block_buffer_base);
//   Eigen::Vector3i lower_bound ;
//   Eigen::Vector3i upper_bound ;
//   getFreeMapBounds(tree_, free_map, lower_bound, upper_bound);
//   bool setup_planner = path_planner_ompl_ptr->setupPlanner(lower_bound, upper_bound);
//   Eigen::Vector3i start = {89, 75, 72};
//   Eigen::Vector3i end = {95, 75, 72};
//   // WHEN planning a path in free space
//   int solution_status = path_planner_ompl_ptr->planPath(start, end);

//   // THEN there is an exact solution
//   EXPECT_EQ(solution_status, 1);

// }

// TEST_F(CollisionUnitTest, PathPlanningAroundWall) {
// // GIVEN a octree map , a path planner, a collision checker for the map, start and end point of a path are free
//   auto &block_buffer_base = tree_->getBlockBuffer();
//   // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
//   auto collision_checker =
//       aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
//   auto path_planner_ompl_ptr = aligned_shared<se::exploration::PathPlannerOmpl<OFusion> >(tree_,
//                                                                                           collision_checker,
//                                                                                           planner_config_,
//                                                                                           12.1f);
//   set3i free_map = createMap3i(block_buffer_base);
//   Eigen::Vector3i lower_bound ;
//   Eigen::Vector3i upper_bound ;
//   getFreeMapBounds(tree_, free_map, lower_bound, upper_bound);
//   bool setup_planner = path_planner_ompl_ptr->setupPlanner(lower_bound, upper_bound);
//   Eigen::Vector3i start = {80, 80, 72};
//   Eigen::Vector3i end = {90, 50, 72};
//    // WHEN planning a path around a wall
//   int solution_status = path_planner_ompl_ptr->planPath(start, end);

//   // THEN there is an exact solution
//   EXPECT_EQ(solution_status, 1);

// }

// TEST_F(CollisionUnitTest, ApproximateSolution) {
// // GIVEN a octree map , a path planner, a collision checker for the map,
//   // start is free and end is occupied
//   auto &block_buffer_base = tree_->getBlockBuffer();
//   // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
//   auto collision_checker =
//       aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
//   auto path_planner_ompl_ptr = aligned_shared<se::exploration::PathPlannerOmpl<OFusion> >(tree_,
//                                                                                           collision_checker,
//                                                                                           planner_config_,
//                                                                                           12.1f);
//   set3i free_map = createMap3i(block_buffer_base);
//   Eigen::Vector3i lower_bound ;
//   Eigen::Vector3i upper_bound ;
//   getFreeMapBounds(tree_, free_map, lower_bound, upper_bound);
//   bool setup_planner = path_planner_ompl_ptr->setupPlanner(lower_bound, upper_bound);
//   Eigen::Vector3i start = {80, 80, 72};
//   Eigen::Vector3i end = {90, 60, 72};

//  // WHEN planning a path around a wall
//   int solution_status = path_planner_ompl_ptr->planPath(start, end);

//   // THEN there is only an approxiamted solution
//   EXPECT_EQ(solution_status, 2);

// }



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
  set3i morton_set_leaf = collision_checker->getCollisionNodeList(point_list, node_level);


  object_size_v = 10;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node = collision_checker->getCollisionNodeList(point_list, node_level );
  EXPECT_EQ(node_level, tree_->leaf_level()-1);

  object_size_v = 27;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node2 = collision_checker->getCollisionNodeList(point_list, node_level );
  EXPECT_EQ(node_level, tree_->leaf_level()-2);

  object_size_v = 55;
  node_level = collision_checker->getNodeLevel(object_size_v);
  set3i morton_set_node3 = collision_checker->getCollisionNodeList(point_list, node_level );
  EXPECT_EQ(node_level, tree_->leaf_level()-3);


  // check that parent node has the highest occupancy child probability
  float prob[8];
  for(int i = 0; i <8 ; i++){
    prob[i] = tree_->root()->child(5)->value_[i].x;
  }
  float* parent_prob;
  parent_prob = std::max_element(prob, prob + 8);
  EXPECT_FLOAT_EQ( tree_->root()->value_[5].x, *parent_prob);

  for(int i = 0; i <8 ; i++){
    prob[i] = tree_->root()->child(5)->child(2)->value_[i].x;
  }
  parent_prob;
  parent_prob = std::max_element(prob, prob + 8);
  EXPECT_FLOAT_EQ( tree_->root()->child(5)->value_[2].x, *parent_prob);

  for(int i = 0; i <8 ; i++){
    prob[i] = tree_->root()->child(5)->child(2)->child(1)->value_[i].x;
  }
  parent_prob;
  parent_prob = std::max_element(prob, prob + 8);
  EXPECT_FLOAT_EQ( tree_->root()->child(5)->child(2)->value_[1].x, *parent_prob);

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
  // start is free and end is occupied
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  planner_config_.robot_safety_radius = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);


  int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
  int node_level = collision_checker->getNodeLevel( object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level()-1);


  Eigen::Vector3i center = {80, 80, 72};
  bool is_collision_free = collision_checker->isSphereSkeletonFree(center, planner_config_.robot_safety_radius /tree_->voxelDim());

  EXPECT_TRUE(is_collision_free);

}
TEST_F(CollisionUnitTest, CollisionCheckSphereOldPass) {
// GIVEN a octree map , a path planner, a collision checker for the map,
  // start is free and end is occupied
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  planner_config_.robot_safety_radius = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);


  int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
  int node_level = collision_checker->getNodeLevel( object_size_v);
  EXPECT_EQ(node_level, tree_->leaf_level()-1);


  Eigen::Vector3i center = {80, 80, 72};
  bool is_collision_free = collision_checker->isSphereSkeletonFreeCand(center, planner_config_.robot_safety_radius /tree_->voxelDim());

  EXPECT_TRUE(is_collision_free);

}

// TEST_F(CollisionUnitTest, CollisionCheckSphereFail) {
// // GIVEN a octree map , a path planner, a collision checker for the map,
//   // start is free and end is occupied
//   auto &block_buffer_base = tree_->getBlockBuffer();
//   // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
//   planner_config_.robot_safety_radius = 0.8;
//   auto collision_checker =
//       aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);


//   int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
//   int node_level = collision_checker->getNodeLevel( object_size_v);
//   EXPECT_EQ(node_level, tree_->leaf_level()-1);


//   Eigen::Vector3i center = {89, 80, 72};
//   bool is_collision_free = collision_checker->isSphereSkeletonFree(center, planner_config_.robot_safety_radius /tree_->voxelDim());

//   EXPECT_TRUE(is_collision_free);

// }
// TEST_F(CollisionUnitTest, CollisionCheckSphereOldFail) {
// // GIVEN a octree map , a path planner, a collision checker for the map,
//   // start is free and end is occupied
//   auto &block_buffer_base = tree_->getBlockBuffer();
//   // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
//   planner_config_.robot_safety_radius = 0.8;
//   auto collision_checker =
//       aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);


//   int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
//   int node_level = collision_checker->getNodeLevel( object_size_v);
//   EXPECT_EQ(node_level, tree_->leaf_level()-1);


//   Eigen::Vector3i center = {89, 80, 72};
//   bool is_collision_free = collision_checker->isSphereSkeletonFreeCand(center, planner_config_.robot_safety_radius /tree_->voxelDim());

//   EXPECT_TRUE(is_collision_free);

// }
// TEST_F(CollisionUnitTest, CollisionCheckSphereFailVB) {
// // GIVEN a octree map , a path planner, a collision checker for the map,
//   // start is free and end is occupied
//   auto &block_buffer_base = tree_->getBlockBuffer();
//   // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);

//   auto collision_checker =
//       aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);


//   int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
//   int node_level = collision_checker->getNodeLevel( object_size_v);
//   EXPECT_EQ(node_level, tree_->leaf_level());


//   Eigen::Vector3i center = {89, 80, 72};
//   bool is_collision_free = collision_checker->isSphereSkeletonFree(center, planner_config_.robot_safety_radius /tree_->voxelDim());

//   EXPECT_TRUE(is_collision_free);

// }
// TEST_F(CollisionUnitTest, CollisionCheckSphereOldFailVB) {
// // GIVEN a octree map , a path planner, a collision checker for the map,
//   // start is free and end is occupied
//   auto &block_buffer_base = tree_->getBlockBuffer();
//   // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);

//   auto collision_checker =
//       aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);


//   int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
//   int node_level = collision_checker->getNodeLevel( object_size_v);
//   EXPECT_EQ(node_level, tree_->leaf_level());


//   Eigen::Vector3i center = {89, 80, 72};
//   bool is_collision_free = collision_checker->isSphereSkeletonFreeCand(center, planner_config_.robot_safety_radius /tree_->voxelDim());

//   EXPECT_TRUE(is_collision_free);

// }

TEST_F(CollisionUnitTest, GetLinePoints) {
  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {89, 75, 72};
  const Eigen::Vector3i end = {80, 75, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;
  // WHEN: checking for collision of the segment corridor in free space
  VecVec3i points = collision_checker->getLinePoints(start, vec_seg_connection_v, num_axial_subpos);
  const int size =  points.size();
  for(auto point : points){
    LOG(INFO)<< point.format(InLine);
  }

  // THEN: the corridor is collision free
  EXPECT_EQ(size, num_axial_subpos+2);

}
TEST_F(CollisionUnitTest, CheckLinePass) {

  bool collision_free = true;

  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  // std::shared_ptr<se::Octree<OFusion> > octree_ptr(&tree_);
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {89, 75, 72};
  const Eigen::Vector3i end = {80, 75, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;
  // WHEN: checking for collision of the segment corridor in free space
  // VecVec3i points = collision_checker->getLinePoints(start, vec_seg_connection_v, num_axial_subpos);
  VecVec3i points = collision_checker->getSphereSkeletonPoints(start,std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim()) );
  int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
  int node_level = collision_checker->getNodeLevel( object_size_v);
  mapvec3i affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level);
  LOG(INFO)<< "size " << affected_nodes_list.size();
  bool check_lower_node_level= false;
  VecVec3i points_new;
  for (mapvec3i::iterator it_morton_code = affected_nodes_list.begin(); it_morton_code != affected_nodes_list.end(); ) {
    if (collision_checker->isNodeFree(it_morton_code->first, node_level)){
      it_morton_code = affected_nodes_list.erase(it_morton_code);
      LOG(INFO)<< "erase ";
    }else{
      for( auto p : affected_nodes_list[it_morton_code->first]){
        points_new.push_back(p);
      }
      it_morton_code++;
    }
  }
  if(points_new.size()!= 0){
    for (VecVec3i::iterator it = points_new.begin(); it != points_new.end(); ++it) {
      if (tree_->get(*it).x > 0.f)
        collision_free = false;
    }
  }
  LOG(INFO)<< "point size " << points.size() << " new point size() " << points_new.size();

  LOG(INFO) << "node list size "<< affected_nodes_list.size();


  // THEN: the corridor is collision free
  EXPECT_EQ(collision_free, true);

}

TEST_F(CollisionUnitTest, CheckLinePass3) {

  bool collision_free = true;

  //GIVEN a octree map , a collision checker for the map, start and end point of a path segment
  auto &block_buffer_base = tree_->getBlockBuffer();
  planner_config_.robot_safety_radius = 0.8;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  const int seg_prec_v = std::ceil(planner_config_.skeleton_sample_precision / dim_);
  const Eigen::Vector3i start = {89, 75, 72};
  const Eigen::Vector3i end = {80, 75, 72};
  const Eigen::Vector3i vec_seg_connection_v = end - start;
  const int num_axial_subpos = vec_seg_connection_v.norm() / seg_prec_v;
  // WHEN: checking for collision of the segment corridor in free space
  // VecVec3i points = collision_checker->getLinePoints(start, vec_seg_connection_v, num_axial_subpos);
  VecVec3i points = collision_checker->getSphereSkeletonPoints(start,std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim()) );
  int object_size_v = std::ceil(planner_config_.robot_safety_radius /tree_->voxelDim())*2;
  int node_level = collision_checker->getNodeLevel( object_size_v);
  mapvec3i affected_nodes_list = collision_checker->getCollisionNodePointList(points, node_level);
  LOG(INFO)<< "size " << affected_nodes_list.size();
  bool check_lower_node_level= false;
  VecVec3i points_new = collision_checker->areNodesFree(affected_nodes_list,node_level);

  if(points_new.size()!= 0){
    affected_nodes_list = collision_checker->getCollisionNodePointList(points_new, node_level+1);
    points_new  =collision_checker->areNodesFree(affected_nodes_list,node_level+1);
  }
  if(points_new.size()!= 0 && node_level+1 == tree_->leaf_level()){
    for (VecVec3i::iterator it = points_new.begin(); it != points_new.end(); ) {
      LOG(INFO)<< tree_->get(*it).x ;
      if (tree_->get(*it).x > 0.f)
        it++;
      else
        it = points_new.erase(it);
    }
  }
  if(points_new.size()!=0)
    collision_free = false;
  LOG(INFO)<< "point size " << points.size() << " new point size() " << points_new.size();

  LOG(INFO) << "node list size "<< affected_nodes_list.size();


  // THEN: the corridor is collision free
  EXPECT_EQ(collision_free, true);

}