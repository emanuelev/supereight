/*
  Copyright 2016 Emanuele Vespa, Imperial College London
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors
  may be used to endorse or promote products derived from this software without
  specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <random>

#include "se/octree.hpp"
#include "se/node_iterator.hpp"
#include "se/node.hpp"
#include "se/utils/math_utils.h"
#include "se/utils/morton_utils.hpp"
#include "se/utils/eigen_utils.h"

#include "se/functors/axis_aligned_functor.hpp"
#include "se/config.h"
#include "se/planner_config.h"
#include "gtest/gtest.h"

// #include "se/ompl/collision_checker_voxel.hpp"
#include "se/path_planning/candidate_view.hpp"
#include "se/path_planning/init_new_position.hpp"

// typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > vec3i;

template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
typedef struct {
  float x;
} testT;

template<>
struct voxel_traits<testT> {
  typedef testT value_type;
  static inline value_type empty() { return {0.f}; }
  static inline value_type initValue() { return {0.f}; }
};

class CandidateViewTest : public ::testing::Test {
 protected:
  virtual void SetUp() {

    unsigned size = 128;
    float dim = 12.8f;
    oct_.init(size, dim); // 10 meters
    planning_config = getDefaultPlanningConfig();

    const unsigned center = 6.4f;

    float voxelsize = oct_.dim() / oct_.size(); // 0.1m
    const float inverse_voxelsize = 1.f / voxelsize;
    const int band = 1 * inverse_voxelsize; // 10
    const Eigen::Vector3i offset = Eigen::Vector3i::Constant(oct_.size() / 2 - band / 2);
    unsigned leaf_level = log2(size) - log2(se::Octree<testT>::blockSide);
    for (int z = 0; z < band; ++z) {
      for (int y = 0; y < band; ++y) {
        for (int x = 0; x < band; ++x) {
          const Eigen::Vector3i vox = Eigen::Vector3i(x + offset(0), y + offset(1), z + offset(2));
          alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2), leaf_level));
        }
      }
    }
    oct_.allocate(alloc_list.data(), alloc_list.size());

  }

  Volume<testT> volume_;
  typedef se::Octree<testT> OctreeF;
  OctreeF oct_;
  std::vector<se::key_t> alloc_list;
  Planning_Configuration planning_config;
  Configuration config;
  float voxelsize;

};

TEST_F(CandidateViewTest, WeightFunction) {
  //GIVEN: a cand view object
  Eigen::Matrix4f pose;
  pose << 1.f, 0.f, 0.f, 3.f, 0.f, 1.f, 0.f, 3.f, 0.f, 0.f, 1.f, 3.f, 0.f, 0.f, 0.f, 1.f;
  std::shared_ptr<se::Octree<testT> > tree = aligned_shared<se::Octree<testT> >();
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<testT> >(tree, planning_config);
  se::exploration::CandidateView<testT>
      cand_view(volume_, planning_config,collision_checker, voxelsize, config, pose, 0.1f, 12.1f);
  bool hit_unknown[3] = {false, false, false};
  float t_hit[3]={0.f, 0.f , 0.f};
  float prob_log[3] = {0.f, 3.f, 0.f};
  float ray_length[3] = {0.f, 1.f, 2.f};
  float ray_max = 4.f;
  float ratio = 1.f;
  float weight[3];
  //WHEN: check the tanh function , that when prob_log = 0 , unknown voxel is hit

  testing::internal::CaptureStdout();



  weight[0]= cand_view.getIGWeight_tanh(ray_max, ratio, ray_length[0], prob_log[0], t_hit[0],
      hit_unknown[0]);

  weight[1]= cand_view.getIGWeight_tanh(ray_max, ratio, ray_length[1], prob_log[1], t_hit[1],
                                        hit_unknown[1]);
 // when a ray hits a unknown voxel at 2m . the unknown voxel should have the weight of tanh(4)
  weight[2]= cand_view.getIGWeight_tanh(ray_max, ratio, ray_length[2], prob_log[2], t_hit[2],
                                        hit_unknown[2]);

  //THEN: the tanh function should be applied
  EXPECT_FLOAT_EQ(tanh(4), weight[0]);
  EXPECT_TRUE(hit_unknown[0]);
  EXPECT_FLOAT_EQ(0.f, t_hit[0]); //


  EXPECT_FLOAT_EQ(1.f, weight[1]);
  EXPECT_FALSE(hit_unknown[1]);
  EXPECT_FLOAT_EQ(0.f, t_hit[1]);

  EXPECT_FLOAT_EQ(tanh(4), weight[2]);
  EXPECT_TRUE(hit_unknown[2]);
  EXPECT_FLOAT_EQ(2.f, t_hit[2]);

  std::string output = testing::internal::GetCapturedStdout();
//  EXPECT_TRUE(false) << output;
}


class CandViewUnitTest : public ::testing::Test {
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  virtual void SetUp() {
    tree_ = std::make_shared<se::Octree<OFusion> >();
    std::string
        filename = "/home/anna/exploration_ws/src/supereight/se_shared/test/path_planning/frontier_multires.bin";
    tree_->load(filename);
    std::cout << "file loaded" << std::endl;
    max_depth_ = log2(tree_->size());
    leaves_level_ = tree_->leaf_level();
    dim_ = tree_->voxelDim();
    planner_config_ = getDefaultPlanningConfig();
    auto &block_buffer_base = tree_->getBlockBuffer();
    morton_code_ = createMap3i(block_buffer_base);
    volume_ = Volume<OFusion>(128, 24, tree_.get());

  }

  set3i createMap3i(se::MemoryPool<se::VoxelBlock<OFusion> > &block_buffer) {
    set3i morton_map;
    for (int i = 0; i < block_buffer.size(); i++) {

      const key_t morton_code = block_buffer[i]->code_;
      morton_map.emplace(morton_code);
    }
    return morton_map;
  }
  Volume<OFusion> volume_;
  std::shared_ptr<se::Octree<OFusion> > tree_;
  set3i morton_code_;
  int max_depth_;
  int leaves_level_;
  float dim_;
  Planning_Configuration planner_config_;
  Configuration config_;

};

TEST_F(CandViewUnitTest, ClassSetup){
  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  se::exploration::CandidateView<OFusion>
      cand_view(volume_, planner_config_, collision_checker, dim_, config_, curr_pose, 0.1f, 12.1f);


  EXPECT_FLOAT_EQ(cand_view.curr_pose_.pose.p.x(), 15.f/dim_);
  EXPECT_FLOAT_EQ(cand_view.curr_pose_.pose.p.y(), 12.f/dim_);
  EXPECT_FLOAT_EQ(cand_view.curr_pose_.pose.p.z(), 13.5f/dim_);

  EXPECT_EQ(cand_view.candidates_.size(), planner_config_.num_cand_views);
}



TEST_F(CandViewUnitTest, GetViewInformationGain4Times){

  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  se::exploration::CandidateView<OFusion>
      cand_view(volume_, planner_config_, collision_checker, dim_, config_, curr_pose, 0.1f, 12.1f);
  cand_view.candidates_[0].pose.p = Eigen::Vector3f(50, 100, 75);
  cand_view.candidates_[1].pose.p = Eigen::Vector3f(51, 115, 72);

  float ig_1 = cand_view.getViewInformationGain(cand_view.candidates_[0].pose);
  float yaw_1 =  se::exploration::toEulerAngles(cand_view.candidates_[0].pose.q).yaw ;

 float ig_2 =  cand_view.getViewInformationGain(cand_view.candidates_[0].pose);

  float yaw_2 =  se::exploration::toEulerAngles(cand_view.candidates_[0].pose.q).yaw ;

  EXPECT_FLOAT_EQ(yaw_1, yaw_2);
  EXPECT_FLOAT_EQ(ig_1, ig_2);

   ig_1 =  cand_view.getViewInformationGain(cand_view.candidates_[1].pose);
   yaw_1 =  se::exploration::toEulerAngles(cand_view.candidates_[1].pose.q).yaw ;

  ig_2 = cand_view.getViewInformationGain(cand_view.candidates_[1].pose);

   yaw_2 =  se::exploration::toEulerAngles(cand_view.candidates_[1].pose.q).yaw ;
  EXPECT_FLOAT_EQ(yaw_1, yaw_2);
  EXPECT_FLOAT_EQ(ig_1, ig_2);
}

TEST_F(CandViewUnitTest, IGSparsityTest){

  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  auto collision_checker =
  aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  int dphi[4] = {1, 5 , 10, 15};
  int dtheta[4] = {1, 10, 20, 30};
    std::chrono::time_point<std::chrono::steady_clock> timings[2];
  for (int i = 0; i < 4 ; i++ ){

    planner_config_.dtheta = dtheta[i];
    planner_config_.dphi = dphi[i];
    se::exploration::CandidateView<OFusion>
    cand_view(volume_, planner_config_, collision_checker, dim_, config_, curr_pose, 0.1f, 12.1f);
    cand_view.candidates_[0].pose.p = Eigen::Vector3f(50, 100, 75);
    timings[0] = std::chrono::steady_clock::now();
    float ig_1 = cand_view.getViewInformationGain(cand_view.candidates_[0].pose);

    timings[1] = std::chrono::steady_clock::now();
    double progress_time = std::chrono::duration_cast<std::chrono::duration<double> >
    (timings[1] - timings[0]).count();
    float yaw_1 =  se::exploration::toEulerAngles(cand_view.candidates_[0].pose.q).yaw ;

    float ig_2 = cand_view.getViewInformationGain(cand_view.candidates_[0].pose);
    float yaw_2 =  se::exploration::toEulerAngles(cand_view.candidates_[0].pose.q).yaw ;
    LOG(INFO) <<"theta " << dtheta[i] << " phi " << dphi[i]<< " "<< ig_1 << " time " << progress_time ;
    int n_col = planner_config_.fov_hor * 0.75 / dphi[i];
    int n_row = 360 / dtheta[i];
    LOG(INFO)<< "time per ray " << progress_time/ (n_col*n_row) << " num ray " << n_row*n_col;

    EXPECT_FLOAT_EQ(yaw_1, yaw_2);
    EXPECT_FLOAT_EQ(ig_1, ig_2);
  }

}


TEST_F(CandViewUnitTest, GetRandCand){
  Eigen::Matrix4f curr_pose;
 curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
    std::chrono::time_point<std::chrono::steady_clock> timings[2];

 int num_sample[4] = {10, 20, 30, 40};
 for(int i = 0 ; i < 4 ; i ++){
  int frontier_cluster_size = planner_config_.frontier_cluster_size;
  planner_config_.num_cand_views = num_sample[i];
  se::exploration::CandidateView<OFusion>
      cand_view(volume_, planner_config_, collision_checker, dim_, config_, curr_pose, 0.1f, 12.1f);
    timings[0] = std::chrono::steady_clock::now();
  while(cand_view.getNumValidCandidates()<5){

    cand_view.getCandidateViews(morton_code_, frontier_cluster_size);
    if(frontier_cluster_size>8){
      frontier_cluster_size/=2;
    }
  }
    timings[1] = std::chrono::steady_clock::now();
    double progress_time = std::chrono::duration_cast<std::chrono::duration<double> >
    (timings[1] - timings[0]).count();

  int num_cand = cand_view.getNumValidCandidates();
  LOG(INFO) << "num_cand " << num_cand << " cluster size " << frontier_cluster_size <<
  " progress time " << progress_time;

  EXPECT_TRUE(num_cand>0);
}
}

TEST_F(CandViewUnitTest, AddSegments){
  Eigen::Matrix4f curr_pose;
 curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  auto collision_checker =
      aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
    std::chrono::time_point<std::chrono::steady_clock> timings[2];
 se::exploration::pose3D start;
 start = {{50, 100, 75}, {1.f, 0.f, 0.f, 0.f}};
 se::exploration::pose3D end;
 end = {{60, 120, 75}, {1.f, 0.f, 0.f, 0.f}};

 float sampling_dist[3] = {0.4, 0.8, 1.2};
 int num_seg[3] = {13, 7, 5};
 for(int i = 0 ; i < 3 ; i ++){
    planner_config_.max_rrt_edge_length = sampling_dist[i];
    se::exploration::CandidateView<OFusion>
      cand_view(volume_, planner_config_, collision_checker, dim_, config_, curr_pose, 0.1f, 12.1f);
    timings[0] = std::chrono::steady_clock::now();
    VecPose path = cand_view.addPathSegments( start, end );
    timings[1] = std::chrono::steady_clock::now();
    double progress_time = std::chrono::duration_cast<std::chrono::duration<double> >
    (timings[1] - timings[0]).count();


  LOG(INFO) << "samling dist " << planner_config_.max_rrt_edge_length<< " path size "<< path.size() << " progress time " << progress_time;
  EXPECT_EQ(path.size(), num_seg[i] );
}
}



TEST_F(CandViewUnitTest, quaternion){
  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  const float max_yaw_rate =planner_config_.max_yaw_rate;
  std::chrono::time_point<std::chrono::steady_clock> timings[2];
  se::exploration::pose3D start;
  start = {{50, 100, 75}, {0.1736f, 0.f, 0.f, 0.984807f}};
  LOG(INFO)<< "yaw " << se::exploration::toEulerAngles(start.q).yaw*180/M_PI ;
  se::exploration::pose3D end;
  end = {{60, 120, 75}, {1.f, 0.f, 0.f, 0.f}};
  float yaw_diff = se::exploration::toEulerAngles(end.q).yaw - se::exploration::toEulerAngles(start.q).yaw;
  LOG(INFO) << "yaw diff " << yaw_diff *180/M_PI;
  se::exploration::wrapYawRad(yaw_diff);
  se::exploration::pose3D pose_tmp;
  LOG(INFO)<< std::abs(yaw_diff)/max_yaw_rate << " " << max_yaw_rate *180 /M_PI;
  for(int i = 0; i < std::abs(yaw_diff)/max_yaw_rate ;i++){
    pose_tmp = end;
    pose_tmp.q = start.q.slerp(i*max_yaw_rate/std::abs(yaw_diff), end.q);
    LOG(INFO)<< "intermediate yaw "<< se::exploration::toEulerAngles(pose_tmp.q).yaw *180/M_PI;
  }
  EXPECT_EQ(int(yaw_diff*180/M_PI), -160);
}
TEST_F(CandViewUnitTest, quaternionMinus){
  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  const float max_yaw_rate =planner_config_.max_yaw_rate;
  std::chrono::time_point<std::chrono::steady_clock> timings[2];
  se::exploration::pose3D start;
  start = {{50, 100, 75}, {0.1736f, 0.f, 0.f, -0.984807f}};
  LOG(INFO)<< "yaw " << se::exploration::toEulerAngles(start.q).yaw*180/M_PI ;
  se::exploration::pose3D end;
  end = {{60, 120, 75}, {1.f, 0.f, 0.f, 0.f}};
  float yaw_diff = se::exploration::toEulerAngles(end.q).yaw - se::exploration::toEulerAngles(start.q).yaw;
  LOG(INFO) << "yaw diff " << yaw_diff *180/M_PI;
  se::exploration::wrapYawRad(yaw_diff);
  se::exploration::pose3D pose_tmp;
  LOG(INFO)<< std::abs(yaw_diff)/max_yaw_rate << " " << max_yaw_rate *180 /M_PI;
  for(int i = 0; i < std::abs(yaw_diff)/max_yaw_rate ;i++){
    pose_tmp = end;
    pose_tmp.q = start.q.slerp(i*max_yaw_rate/std::abs(yaw_diff), end.q);
    LOG(INFO)<< "intermediate yaw "<< se::exploration::toEulerAngles(pose_tmp.q).yaw *180/M_PI;
  }
  EXPECT_EQ(int(yaw_diff*180/M_PI), 160);
}
TEST_F(CandViewUnitTest, quaternionold){
  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  const float max_yaw_rate =planner_config_.max_yaw_rate;
  std::chrono::time_point<std::chrono::steady_clock> timings[2];
  se::exploration::pose3D start;
  start = {{50, 100, 75}, {0.1736f, 0.f, 0.f, -0.984807f}};
  LOG(INFO)<< "yaw " << se::exploration::toEulerAngles(start.q).yaw*180/M_PI ;
  se::exploration::pose3D end;
  end = {{60, 120, 75}, {1.f, 0.f, 0.f, 0.f}};
  float yaw_diff = se::exploration::toEulerAngles(end.q).yaw - se::exploration::toEulerAngles(start.q).yaw;
  LOG(INFO) << "yaw diff " << yaw_diff *180/M_PI;
  se::exploration::wrapYawRad(yaw_diff);
  se::exploration::pose3D pose_tmp;
  if (yaw_diff >= 0) {

    for (float dyaw = 0.0f; dyaw < yaw_diff; dyaw += max_yaw_rate) {
      pose_tmp = end;
      pose_tmp.q = se::exploration::toQuaternion(se::exploration::toEulerAngles(start.q).yaw + dyaw, 0.f, 0.f);
      pose_tmp.q.normalize();
      LOG(INFO)<< "intermediate yaw "<< se::exploration::toEulerAngles(pose_tmp.q).yaw *180/M_PI;

    }
  } else {

    for (float dyaw = 0.0f; dyaw > yaw_diff; dyaw -= max_yaw_rate) {
      pose_tmp = end;
      pose_tmp.q = se::exploration::toQuaternion(se::exploration::toEulerAngles(start.q).yaw + dyaw, 0.f, 0.f);
      pose_tmp.q.normalize();
      LOG(INFO)<< "intermediate yaw "<< se::exploration::toEulerAngles(pose_tmp.q).yaw *180/M_PI;
    }
  }
}

TEST_F(CandViewUnitTest, quaternion2){
  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  const float max_yaw_rate =planner_config_.max_yaw_rate;
  std::chrono::time_point<std::chrono::steady_clock> timings[2];
  se::exploration::pose3D start;
  start = {{50, 100, 75}, {-0.707168f, 0.f, 0.f, 0.707168f}};
  LOG(INFO)<< "yaw " << se::exploration::toEulerAngles(start.q).yaw*180/M_PI ;
  se::exploration::pose3D end;
  end = {{60, 120, 75}, {1.f, 0.f, 0.f, 0.f}};
  float yaw_diff = se::exploration::toEulerAngles(end.q).yaw - se::exploration::toEulerAngles(start.q).yaw;
  LOG(INFO) << "yaw diff " << yaw_diff *180/M_PI;
  se::exploration::wrapYawRad(yaw_diff);
  se::exploration::pose3D pose_tmp;
  LOG(INFO)<< std::abs(yaw_diff)/max_yaw_rate << " " << max_yaw_rate *180 /M_PI;
  for(int i = 1; i < std::abs(yaw_diff)/max_yaw_rate ;i++){
    pose_tmp = end;
    pose_tmp.q = start.q.slerp(i*max_yaw_rate/std::abs(yaw_diff), end.q);
    LOG(INFO)<< "intermediate yaw "<< se::exploration::toEulerAngles(pose_tmp.q).yaw *180/M_PI;
  }
 EXPECT_EQ(int(yaw_diff*180/M_PI), 90);
}

TEST_F(CandViewUnitTest, quaternion2Minus){
  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  const float max_yaw_rate =planner_config_.max_yaw_rate;
  std::chrono::time_point<std::chrono::steady_clock> timings[2];
  se::exploration::pose3D start;
  start = {{50, 100, 75}, {-0.707168f, 0.f, 0.f, -0.707168f}}; // -270
  LOG(INFO)<< "yaw " << se::exploration::toEulerAngles(start.q).yaw*180/M_PI ;
  se::exploration::pose3D end;
  end = {{60, 120, 75}, {1.f, 0.f, 0.f, 0.f}};
  float yaw_diff = se::exploration::toEulerAngles(end.q).yaw - se::exploration::toEulerAngles(start.q).yaw;
  LOG(INFO) << "yaw diff " << yaw_diff *180/M_PI;
  se::exploration::wrapYawRad(yaw_diff);
  se::exploration::pose3D pose_tmp;
  LOG(INFO)<< std::abs(yaw_diff)/max_yaw_rate << " " << max_yaw_rate *180 /M_PI;
  for(int i = 1; i < std::abs(yaw_diff)/max_yaw_rate ;i++){
    pose_tmp = end;
    pose_tmp.q = start.q.slerp(i*max_yaw_rate/std::abs(yaw_diff), end.q);
    LOG(INFO)<< "intermediate yaw "<< se::exploration::toEulerAngles(pose_tmp.q).yaw *180/M_PI;
  }
  EXPECT_EQ(int(yaw_diff*180/M_PI), -90);
}


TEST_F(CandViewUnitTest, fusePath){
  Eigen::Matrix4f curr_pose;
  curr_pose << 1 , 0,0, 15.f ,0,1,0,12.f, 0,0,1,13.5f, 0,0,0,1;
  const float max_yaw_rate =planner_config_.max_yaw_rate;
  std::chrono::time_point<std::chrono::steady_clock> timings[2];
  auto collision_checker =
    aligned_shared<se::exploration::CollisionCheckerV<OFusion> >(tree_, planner_config_);
  se::exploration::CandidateView<OFusion>
    cand_view(volume_, planner_config_, collision_checker, dim_, config_, curr_pose, 0.1f, 12.1f);


  se::exploration::pose3D start;
  start = {{50, 100, 75}, {-0.707168f, 0.f, 0.f, -0.707168f}}; // -270
  LOG(INFO)<< "yaw " << se::exploration::toEulerAngles(start.q).yaw*180/M_PI ;
  se::exploration::pose3D end;
  end = {{60, 120, 75}, {1.f, 0.f, 0.f, 0.f}};

  float yaw_diff = se::exploration::toEulerAngles(end.q).yaw - se::exploration::toEulerAngles(start.q).yaw;
  LOG(INFO) << "yaw diff " << yaw_diff *180/M_PI;
  se::exploration::wrapYawRad(yaw_diff);
  se::exploration::pose3D pose_tmp;

  VecPose path_tmp = cand_view.addPathSegments(start, end);
  LOG(INFO)<< "path size " << path_tmp.size();
  VecPose yaw_path = cand_view.getYawPath(start, end);
  LOG(INFO)<< "yaw path size  "<< yaw_path.size();

  VecPose fused = cand_view.fusePath(path_tmp, yaw_path);

  for(auto path : fused){
    LOG(INFO) << path.p.format(InLine) << " " <<  se::exploration::toEulerAngles(path.q).yaw *180/M_PI;
  }
  EXPECT_EQ(int(yaw_diff*180/M_PI), -90);
}
