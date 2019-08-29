#include <se/octree.hpp>
#include <gtest/gtest.h>
#include "../../include/se/volume_traits.hpp"
#include "../../src/multires_bfusion/mapping_impl.hpp"

class MultiresOFusionNodeUpPropagation : public ::testing::Test {
protected:
  virtual void SetUp() {
    size_ = 64;                               // 512 x 512 x 512 voxel^3
    max_level_ = se::math::log2_const(size_);
    voxel_size_ = 0.005;                      // 5 mm/voxel
    dim_ = size_ * voxel_size_;               // [m^3]
    oct_.init(size_, dim_);

    side_ = se::VoxelBlock<OFusion>::side;
    for(int z = 0; z < size_; z += side_) {
      for(int y = 0; y < size_; y += side_) {
        for(int x = 0; x < size_; x += side_) {
          const Eigen::Vector3i vox(x, y, z);
          alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2)));
        }
      }
    }
    oct_.allocate(alloc_list.data(), alloc_list.size());
  }

  typedef se::Octree<OFusion> OctreeT;
  OctreeT oct_;
  int size_;
  int side_;
  int max_level_;
  float voxel_size_;
  float dim_;
  
private:
  std::vector<se::key_t> alloc_list;
};

TEST_F(MultiresOFusionNodeUpPropagation, Simple) {
  std::vector<se::VoxelBlock<OFusion>*> active_list;
  std::deque<se::Node<OFusion>*> prop_list;

  se::VoxelBlock<OFusion>* vb = NULL;

  // Update some voxels.
  constexpr size_t num_voxels = 4;
  const Eigen::Vector3i voxels[num_voxels] =
      {{0, 0, 0}, {8, 8, 0}, {48, 48, 0}, {56, 56, 0}};

  for (size_t i = 0; i < num_voxels; ++i) {
    oct_.set(voxels[i].x(), voxels[i].y(), voxels[i].z(), {1.f, 1.f, voxel_state::kOccupied}); // Set occupied, frame 1
    vb = oct_.fetch(voxels[i].x(), voxels[i].y(), voxels[i].z());
    active_list.push_back(vb);
  }


  const Eigen::Vector3i free_block(0,0,8);
  vb = oct_.fetch(free_block.x(), free_block.y(), free_block.z());
  active_list.push_back(vb);
  for (int x = 0; x < side_; x++)
    for (int y = 0; y < side_; y++)
      for (int z = 0; z < side_; z++)
    oct_.set(free_block.x() + x, free_block.y() + y, free_block.z() + z, {-(x+1), 1.f, voxel_state::kFree}); // Set free, frame 1


  const Eigen::Vector3i free_frontier_block(8,0,8);
  vb = oct_.fetch(free_frontier_block.x(), free_frontier_block.y(), free_frontier_block.z());
  active_list.push_back(vb);
  for (int x = 0; x < side_; x++)
    for (int y = 0; y < side_; y++)
      for (int z = 0; z < side_; z++)
        oct_.set(free_frontier_block.x() + x, free_frontier_block.y() + y, free_frontier_block.z() + z, {-(x+1), 1.f, voxel_state::kFree}); // Set free, frame 1
  oct_.set(free_frontier_block.x()+5, free_frontier_block.y(), free_frontier_block.z(), {-0.5, 1.f, voxel_state::kFrontier}); // Set frontier, frame 1


  const Eigen::Vector3i free_unknown_block(0,8,8);
  vb = oct_.fetch(free_unknown_block.x(), free_unknown_block.y(), free_unknown_block.z());
  active_list.push_back(vb);
  for (int x = 0; x < side_; x++)
    for (int y = 0; y < side_; y++)
      for (int z = 0; z < side_; z++)
        oct_.set(free_unknown_block.x() + x, free_unknown_block.y() + y, free_unknown_block.z() + z, {-(x+1), 1.f, voxel_state::kFree}); // Set free, frame 1
  oct_.set(free_unknown_block.x()+5, free_unknown_block.y(), free_unknown_block.z(), {0, 1.f, voxel_state::kUnknown}); // Set unknown, frame 1


  const Eigen::Vector3i free_occupied_block(8,8,8);
  vb = oct_.fetch(free_occupied_block.x(), free_occupied_block.y(), free_occupied_block.z());
  active_list.push_back(vb);
  for (int x = 0; x < side_; x++)
    for (int y = 0; y < side_; y++)
      for (int z = 0; z < side_; z++)
        oct_.set(free_occupied_block.x() + x, free_occupied_block.y() + y, free_occupied_block.z() + z, {-(x+1), 1.f, voxel_state::kFree}); // Set free, frame 1
  oct_.set(free_occupied_block.x()+5, free_occupied_block.y(), free_occupied_block.z(), {1, 1.f, voxel_state::kOccupied}); // Set occupied, frame 1

  const Eigen::Vector3i frontier_unknown_block(16,16,16);
  vb = oct_.fetch(frontier_unknown_block.x(), frontier_unknown_block.y(), frontier_unknown_block.z());
  active_list.push_back(vb);
  for (int x = 0; x < side_; x++)
    for (int y = 0; y < side_; y++)
      for (int z = 0; z < side_; z++)
        oct_.set(frontier_unknown_block.x() + x, frontier_unknown_block.y() + y, frontier_unknown_block.z() + z, {-0.5, 1.f, voxel_state::kFrontier}); // Set free, frame 1
  oct_.set(frontier_unknown_block.x()+5, frontier_unknown_block.y(), frontier_unknown_block.z(), {0, 1.f, voxel_state::kUnknown}); // Set occupied, frame 1

  const Eigen::Vector3i frontier_occupied_block(24,16,16);
  vb = oct_.fetch(frontier_occupied_block.x(), frontier_occupied_block.y(), frontier_occupied_block.z());
  active_list.push_back(vb);
  for (int x = 0; x < side_; x++)
    for (int y = 0; y < side_; y++)
      for (int z = 0; z < side_; z++)
        oct_.set(frontier_occupied_block.x() + x, frontier_occupied_block.y() + y, frontier_occupied_block.z() + z, {-0.5, 1.f, voxel_state::kFrontier}); // Set free, frame 1
  oct_.set(frontier_occupied_block.x()+5, frontier_occupied_block.y(), frontier_occupied_block.z(), {1, 1.f, voxel_state::kOccupied}); // Set occupied, frame 1

  const Eigen::Vector3i unknown_occupied_block(16,24,16);
  vb = oct_.fetch(unknown_occupied_block.x(), unknown_occupied_block.y(), unknown_occupied_block.z());
  active_list.push_back(vb);
  for (int x = 0; x < side_; x++)
    for (int y = 0; y < side_; y++)
      for (int z = 0; z < side_; z++)
        oct_.set(unknown_occupied_block.x() + x, unknown_occupied_block.y() + y, unknown_occupied_block.z() + z, {0, 1.f, voxel_state::kUnknown}); // Set free, frame 1
  oct_.set(unknown_occupied_block.x()+5, unknown_occupied_block.y(), unknown_occupied_block.z(), {2, 1.f, voxel_state::kOccupied}); // Set occupied, frame 1


  for (const auto& b : active_list) {
    se::multires::ofusion::propagate_up(b, 0);
  }

  int frame = 1;

  for(const auto& b : active_list) {
    if(b->parent()) {
      prop_list.push_back(b->parent());
      b->timestamp_ = frame;
      const unsigned int id = se::child_id(b->code_,
                                           se::keyops::level(b->code_),  max_level_);
      auto data = b->data(b->coordinates(), se::math::log2_const(se::VoxelBlock<OFusion>::side));
      auto& parent_data = b->parent()->value_[id];
      parent_data = data;
    }
  }


  while(!prop_list.empty()) {
    se::Node<OFusion>* n = prop_list.front();
    prop_list.pop_front();
    if(n->timestamp() == frame) continue;
    se::multires::ofusion::propagate_up(n, max_level_, frame);
    if(n->parent()) prop_list.push_back(n->parent());
  }

  se::Node<OFusion>* n = oct_.root();
  ASSERT_EQ(n->value_[0].x, 2);
  ASSERT_EQ(n->value_[1].x, 0);
  ASSERT_EQ(n->value_[2].x, 0);
  ASSERT_EQ(n->value_[3].x, 1);
  ASSERT_EQ(n->value_[4].x, 0);
  ASSERT_EQ(n->value_[5].x, 0);
  ASSERT_EQ(n->value_[6].x, 0);
  ASSERT_EQ(n->value_[7].x, 0);

  ASSERT_EQ(n->value_[0].st, voxel_state::kOccupied);
  ASSERT_EQ(n->value_[1].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[2].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[3].st, voxel_state::kOccupied);
  ASSERT_EQ(n->value_[4].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[5].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[6].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[7].st, voxel_state::kUnknown);

  n = oct_.root()->child(0);
  ASSERT_EQ(n->value_[0].x, 1);
  ASSERT_EQ(n->value_[1].x, 0);
  ASSERT_EQ(n->value_[2].x, 0);
  ASSERT_EQ(n->value_[3].x, 0);
  ASSERT_EQ(n->value_[4].x, 0);
  ASSERT_EQ(n->value_[5].x, 0);
  ASSERT_EQ(n->value_[6].x, 0);
  ASSERT_EQ(n->value_[7].x, 2);

  ASSERT_EQ(n->value_[0].st, voxel_state::kOccupied);
  ASSERT_EQ(n->value_[1].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[2].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[3].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[4].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[5].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[6].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[7].st, voxel_state::kOccupied);

  n = oct_.root()->child(3);
  ASSERT_EQ(n->value_[0].x, 0);
  ASSERT_EQ(n->value_[1].x, 0);
  ASSERT_EQ(n->value_[2].x, 0);
  ASSERT_EQ(n->value_[3].x, 1);
  ASSERT_EQ(n->value_[4].x, 0);
  ASSERT_EQ(n->value_[5].x, 0);
  ASSERT_EQ(n->value_[6].x, 0);
  ASSERT_EQ(n->value_[7].x, 0);

  ASSERT_EQ(n->value_[0].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[1].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[2].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[3].st, voxel_state::kOccupied);
  ASSERT_EQ(n->value_[4].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[5].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[6].st, voxel_state::kUnknown);
  ASSERT_EQ(n->value_[7].st, voxel_state::kUnknown);

  n = oct_.root()->child(0)->child(0);
  ASSERT_EQ(n->value_[4].x, -1);
  ASSERT_EQ(n->value_[4].st, voxel_state::kFree);

  n = oct_.root()->child(0)->child(0);
  ASSERT_EQ(n->value_[5].x, -0.5);
  ASSERT_EQ(n->value_[5].st, voxel_state::kFrontier);

  n = oct_.root()->child(0)->child(0);
  ASSERT_EQ(n->value_[6].x, 0);
  ASSERT_EQ(n->value_[6].st, voxel_state::kUnknown);

  n = oct_.root()->child(0)->child(0);
  ASSERT_EQ(n->value_[7].x, 1);
  ASSERT_EQ(n->value_[7].st, voxel_state::kOccupied);

  n = oct_.root()->child(0)->child(7);
  ASSERT_EQ(n->value_[0].x, 0);
  ASSERT_EQ(n->value_[0].st, voxel_state::kUnknown);

  n = oct_.root()->child(0)->child(7);
  ASSERT_EQ(n->value_[1].x, 1);
  ASSERT_EQ(n->value_[1].st, voxel_state::kOccupied);

  n = oct_.root()->child(0)->child(7);
  ASSERT_EQ(n->value_[2].x, 2);
  ASSERT_EQ(n->value_[2].st, voxel_state::kOccupied);

}