/**
 * Probabilistic Trajectory Planning, Map interface for supereight library of
 * Emanuele Vespa.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @file OccupancyWorld.cpp
 *
 *
 * @author Nils Funk
 * @date July 5, 2018
 */

#include <se/occupancy_world.hpp>
#include <glog/logging.h>

namespace se {
namespace exploration {
OccupancyWorld::OccupancyWorld() {};

void OccupancyWorld::UpdateMapBounds() {
  map_bounds_min_ = Eigen::Vector3d(0, 0, 0);
  map_bounds_max_ = Eigen::Vector3d(0, 0, 0);

  se::Node<OFusion> *nodeStack[se::Octree<OFusion>::max_depth * 8 + 1];
  size_t stack_idx = 0;

  std::vector<Eigen::Vector3i> min_leaf_nodes;
  for (int i = 0; i < 3; i++) {
    min_leaf_nodes.push_back(Eigen::Vector3i(octree_->size(), octree_->size(), octree_->size()));
  }

  std::vector<Eigen::Vector3i> max_leaf_nodes;
  for (int i = 0; i < 3; i++) {
    max_leaf_nodes.push_back(Eigen::Vector3i(0, 0, 0));
  }

  se::Node<OFusion> *node = octree_->root();

  Eigen::Vector3d map_bounds_min(octree_->size(), octree_->size(), octree_->size());
  Eigen::Vector3d map_bounds_max(0, 0, 0);

  if (node) {
    se::Node<OFusion> *current;
    current = node;
    nodeStack[stack_idx++] = current;
    int block_side;
    block_side = (int) se::VoxelBlock<OFusion>::side; // 8

    while (stack_idx != 0) {
      node = current;// ? why

      if (node->isLeaf()) {
        se::VoxelBlock<OFusion> *block = static_cast<se::VoxelBlock<OFusion> *>(node);
        const Eigen::Vector3i block_coord = block->coordinates();

        for (int i = 0; i < 3; i++) {
          if ((block_coord(i) <= (min_leaf_nodes[i])(i)) && (block_coord(i) < map_bounds_min(i))) {
            min_leaf_nodes[i] = block_coord;
            se::VoxelBlock<OFusion> *block =
                octree_->fetch(min_leaf_nodes[i](0), min_leaf_nodes[i](1), min_leaf_nodes[i](2));
            const Eigen::Vector3i block_coord = block->coordinates();
            int x, y, z;
            int xlast = block_coord(0) + block_side;
            int ylast = block_coord(1) + block_side;
            int zlast = block_coord(2) + block_side;
            for (z = block_coord(2); z < zlast; ++z) {
              for (y = block_coord(1); y < ylast; ++y) {
                for (x = block_coord(0); x < xlast; ++x) {
                  typename se::VoxelBlock<OFusion>::value_type value;
                  const Eigen::Vector3i vox{x, y, z};
                  value = block->data(Eigen::Vector3i(x, y, z));
                  if (value.x != 0) {
                    if (vox(i) < map_bounds_min(i)) map_bounds_min(i) = vox(i);
                  }
                }
              }
            }
          }
        }

        for (int i = 0; i < 3; i++) {
          if ((block_coord(i) >= (max_leaf_nodes[i])(i))
              && ((block_coord(i) + block_side - 1) > map_bounds_min(i))) {
            max_leaf_nodes[i] = block_coord;
            se::VoxelBlock<OFusion> *block =
                octree_->fetch(max_leaf_nodes[i](0), max_leaf_nodes[i](1), max_leaf_nodes[i](2));
            const Eigen::Vector3i block_coord = block->coordinates();
            int x, y, z, block_side;
            block_side = (int) se::VoxelBlock<OFusion>::side;
            int xlast = block_coord(0) + block_side;
            int ylast = block_coord(1) + block_side;
            int zlast = block_coord(2) + block_side;
            for (z = block_coord(2); z < zlast; ++z) {
              for (y = block_coord(1); y < ylast; ++y) {
                for (x = block_coord(0); x < xlast; ++x) {
                  typename se::VoxelBlock<OFusion>::value_type value;
                  const Eigen::Vector3i vox{x, y, z};
                  value = block->data(Eigen::Vector3i(x, y, z));
                  if (value.x != 0) {
                    if (vox(i) > map_bounds_max(i)) map_bounds_max(i) = vox(i);
                  }
                }
              }
            }
          }
        }
      }

      if (node->children_mask_ == 0) {
        current = nodeStack[--stack_idx];
        continue;
      }

      for (int i = 0; i < 8; ++i) {
        se::Node<OFusion> *child = node->child(i);
        if (child != NULL) {
          nodeStack[stack_idx++] = child;
        }
      }
      current = nodeStack[--stack_idx];
    }
  }

  std::cout << "Min node" << "\n" << map_bounds_min << "\n" << std::endl;
  std::cout << "Max node" << "\n" << map_bounds_max << "\n" << std::endl;

  map_bounds_min_ = map_bounds_min;
  map_bounds_max_ = map_bounds_max;
}

void OccupancyWorld::readSupereight(const std::string &filename) {
  octree_ = std::shared_ptr<se::Octree<OFusion>>(new se::Octree<OFusion>());
  octree_->load(filename);
  UpdateMapBounds();
  res_ = double(octree_->dim()) / double(octree_->size());
}

void OccupancyWorld::ReadSupereightMultilevel(const std::string &filename) {
  octree_ = std::shared_ptr<se::Octree<OFusion>>(new se::Octree<OFusion>());
  octree_->loadMultilevel(filename);
  UpdateMapBounds();
  res_ = double(octree_->dim()) / double(octree_->size());
}

void OccupancyWorld::saveMapAsSupereight(const std::string &filename) {
  octree_->save(filename);
}

void OccupancyWorld::saveMapAsSupereightMultilevel(const std::string &filename) {
  octree_->saveMultilevel(filename);
}

void OccupancyWorld::setOctree(std::shared_ptr<se::Octree<OFusion>> octree) {
  octree_ = octree;
  UpdateMapBounds();
  res_ = double(octree_->dim()) / double(octree_->size());
}

bool OccupancyWorld::GetMapBounds(Eigen::Vector3d &map_bounds_min_v,
                                  Eigen::Vector3d &map_bounds_max_v) {
  if (map_bounds_min_ != Eigen::Vector3d(-1, -1, -1)
      && map_bounds_max_ != Eigen::Vector3d(-1, -1, -1)) {
    map_bounds_min_v = map_bounds_min_;
    map_bounds_max_v = map_bounds_max_;
    return true;
  }
  return false;
};

bool OccupancyWorld::GetMapBoundsMeter(Eigen::Vector3d &map_bounds_min_m,
                                       Eigen::Vector3d &map_bounds_max_m) {
  if (map_bounds_min_ != Eigen::Vector3d(-1, -1, -1)
      && map_bounds_max_ != Eigen::Vector3d(-1, -1, -1)) {
    map_bounds_min_m = map_bounds_min_ * res_;
    map_bounds_max_m = map_bounds_max_ * res_;
    return true;
  }
  return false;
}

bool OccupancyWorld::inMapBounds(Eigen::Vector3d position_v) {
  return ((map_bounds_min_.x() <= position_v.x() <= map_bounds_max_.x())
      && (map_bounds_min_.y() <= position_v.y() <= map_bounds_max_.y())
      && (map_bounds_min_.z() <= position_v.z()) <= map_bounds_max_.z());
}

bool OccupancyWorld::InMapBoundsMeter(Eigen::Vector3d position_m) {
  return ((map_bounds_min_.x() * res_ <= position_m.x() <= map_bounds_max_.x() * res_)
      && (map_bounds_min_.y() * res_ <= position_m.y() <= map_bounds_max_.y() * res_)
      && (map_bounds_min_.z() * res_ <= position_m.z()) <= map_bounds_max_.z() * res_);
}

double OccupancyWorld::GetMapResolution() { return res_; }

double OccupancyWorld::getNodeSize(bool single_voxel) {
  if (single_voxel) {
    return res_;
  } else {
    return se::Octree<OFusion>::blockSide * res_;
  }
};

bool OccupancyWorld::isVoxelBlockAllocated(Eigen::Vector3d position_v) {
  if (octree_->fetch(position_v.x(), position_v.y(), position_v.z()) == NULL)
    return false;
  return true;
}

bool OccupancyWorld::isVoxelBlockAllocatedMeter(Eigen::Vector3d position_m) {
  Eigen::Vector3d position_v = position_m / res_;
  if (octree_->fetch(position_v.x(), position_v.y(), position_v.z()) == NULL)
    return false;
  return true;
}

std::vector<std::pair<Eigen::Vector3d,
                      std::pair<int,
                                double>>> OccupancyWorld::getTreeLeafCentersDepthAndOccupancies() {
  se::Node<OFusion> *node = octree_->root();
  se::Node<OFusion> *nodeStack[se::Octree<OFusion>::max_depth * 8 + 1];
  size_t stack_idx = 0;

  std::vector<std::pair<Eigen::Vector3d, std::pair<int, double>>> leaves_information;
  if (node) {
    se::Node<OFusion> *current;
    current = node;
    nodeStack[stack_idx++] = current;
    int block_side;
    block_side = (int) se::VoxelBlock<OFusion>::side;

    while (stack_idx != 0) {
      node = current;

      if (node->isLeaf()) {
        se::VoxelBlock<OFusion> *block = static_cast<se::VoxelBlock<OFusion> *>(node);
        const Eigen::Vector3i block_coord = block->coordinates();
        if (block->value_[0].x < 0) {
          Eigen::Vector3d leaf_center_meter = (block_coord.cast<double>()
              + Eigen::Vector3d(block_side / 2, block_side / 2, block_side / 2)) * res_;
          std::pair<int, double> leaf_content(0, -1);
          std::pair<Eigen::Vector3d, std::pair<int, double>>
              leaf_information(leaf_center_meter, leaf_content);
          leaves_information.push_back(leaf_information);
        } else {
          int x, y, z;
          int xlast = block_coord(0) + block_side;
          int ylast = block_coord(1) + block_side;
          int zlast = block_coord(2) + block_side;
          for (z = block_coord(2); z < zlast; ++z) {
            for (y = block_coord(1); y < ylast; ++y) {
              for (x = block_coord(0); x < xlast; ++x) {
                typename se::VoxelBlock<OFusion>::value_type value;
                const Eigen::Vector3i vox{x, y, z};
                value = block->data(Eigen::Vector3i(x, y, z));
                if (value.x > 0) {
                  Eigen::Vector3d leaf_center_meter =
                      (vox.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * res_;
                  std::pair<int, double> leaf_content(1, 1);
                  std::pair<Eigen::Vector3d, std::pair<int, double>>
                      leaf_information(leaf_center_meter, leaf_content);
                  leaves_information.push_back(leaf_information);
                } else if (value.x < 0) {
                  Eigen::Vector3d leaf_center_meter =
                      (vox.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * res_;
                  std::pair<int, double> leaf_content(1, -1);
                  std::pair<Eigen::Vector3d, std::pair<int, double>>
                      leaf_information(leaf_center_meter, leaf_content);
                  leaves_information.push_back(leaf_information);
                }
              }
            }
          }
        }
      }

      if (node->children_mask_ == 0) {
        current = nodeStack[--stack_idx];
        continue;
      }

      for (int i = 0; i < 8; ++i) {
        se::Node<OFusion> *child = node->child(i);
        if (child != NULL) {
          nodeStack[stack_idx++] = child;
        }
      }
      current = nodeStack[--stack_idx];
    }
  }
  return leaves_information;
}

//std::vector<Eigen::Vector3i> OccupancyWorld::GetUnknownVoxels() {
//  se::node_iterator<OFusion> node_it(*octree_);
//  return node_it.getUnknownVoxels();
//}
//
//  std::vector<Eigen::Vector3d>
//  OccupancyWorld::getPathLeafCenters(Path<kDim>::Ptr path) {
//    int octomap_shift_x;
//    int octomap_shift_y;
//    int octomap_shift_z;
//
//    bool save_octomap_at_origin_ = true;
//
//    if (save_octomap_at_origin_) {
//      octomap_shift_x = 0;
//      octomap_shift_y = 0;
//      octomap_shift_z = 0;
//
//      if (map_bounds_max_(0) >= 32768)
//        octomap_shift_x = map_bounds_max_(0) - 32768;
//      if (map_bounds_max_(1) >= 32768)
//        octomap_shift_y = map_bounds_max_(0) - 32768;
//      if (map_bounds_max_(2) >= 32768)
//        octomap_shift_z = map_bounds_max_(0) - 32768;
//    } else {
//      octomap_shift_x = (map_bounds_max_(0) + map_bounds_min_(0)) / 2;
//      octomap_shift_y = (map_bounds_max_(1) + map_bounds_min_(1)) / 2;
//      octomap_shift_z = (map_bounds_max_(2) + map_bounds_min_(2)) / 2;
//    }
//
//    std::vector<Eigen::Vector3d> flight_corridors_meter;
//
//    ptp::VolumeShifter::Ptr vs = std::make_unique<ptp::VolumeShifter>(res_,
//    2);
//    Eigen::Vector3d start = Eigen::Vector3d(-1,-1,-1);
//    Eigen::Vector3d end = Eigen::Vector3d(-1,-1,-1);
//    for (std::vector<State<kDim>>::iterator it_i = path->states.begin(); it_i
//    != path->states.end(); it_i++) {
//      start = end;
//      end = (*it_i).segment_end;
//      if(start != Eigen::Vector3d(-1,-1,-1) && (*it_i).segment_radius != -1) {
//        std::vector<Eigen::Vector3i> flight_corridor;
//        vs->generateSegmentFlightCorridor(flight_corridor, start, end, 0,
//        (*it_i).segment_radius);
//        for (std::vector<Eigen::Vector3i>::iterator it_j =
//        flight_corridor.begin() ; it_j != flight_corridor.end(); ++it_j) {
//          //std::cout << "Tunnel voxel: " << *it_j << std::endl;
//          Eigen::Vector3d flight_corridor_point = Eigen::Vector3d((*it_j)[0] -
//          octomap_shift_x + 0.5,
//                                                (*it_j)[1] - octomap_shift_y +
//                                                0.5,
//                                                (*it_j)[2] - octomap_shift_z +
//                                                0.5) * res_;
//          flight_corridors_meter.push_back(flight_corridor_point);
//        }
//      }
//    }
//    return flight_corridors_meter;
//  }

}  // namespace exploration
}// se