/**
 * Probabilistic Trajectory Planning, Map interface for supereight library of
 * Emanuele Vespa.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @file OccupancyWorld.hpp
 *
 *
 * @author Nils Funk
 * @date July 5, 2018
 */

#ifndef OCCUPANCYWORLD_HPP
#define OCCUPANCYWORLD_HPP

#include <cstring>
#include <bitset>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>

#include <Eigen/Dense>

#include "se/node.hpp"
#include "se/node_iterator.hpp"
#include "se/octree.hpp"
#include <se/volume_traits.hpp>

#include <boost/foreach.hpp>
#include <boost/range/combine.hpp>

#include "se/utils/support_structs.hpp"
#include "se/utils/eigen_utils.h"
namespace se {
namespace exploration {
class OccupancyWorld {
 public:
  typedef std::shared_ptr<OccupancyWorld> Ptr;

  OccupancyWorld();
  ~OccupancyWorld() {};

  class Color {
   public:
    Color() : r(255), g(255), b(255) {};
    Color(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {};
    inline bool operator==(const Color &other) const {
      return (r == other.r && g == other.g && b == other.b);
    };
    inline bool operator!=(const Color &other) const {
      return (r != other.r || g != other.g || b != other.b);
    };
    uint8_t r, g, b;
  };

  /*
   * Supereight I/O:
   * - loading and saving maps with or without multilevel resolution
   * - set and get octree
   * - get octree root
   * - get voxel occupancy
   */
  void readSupereight(const std::string &filename);
  void ReadSupereightMultilevel(const std::string &filename);
  void saveMapAsSupereight(const std::string &filename);
  void saveMapAsSupereightMultilevel(const std::string &filename);
  // void setOctree(se::Octree<OFusion>*  octree);
  void setOctree(std::shared_ptr<se::Octree<OFusion>> octree);
  // se::Octree<OFusion>* getMap(){return octree_;} //TODO: Change to
  // getOctree()
  std::shared_ptr<se::Octree<OFusion>> getMap() {
    return octree_;
  };  // TODO: Change to getOctree()
  se::Node<OFusion> *getRoot() { return octree_->root(); };
  float get(Eigen::Vector3i position_v) {
    return (octree_->get(position_v.x(), position_v.y(), position_v.z())).x;
  };
  bool isVoxelBlockAllocated(Eigen::Vector3d position_v);
  bool isVoxelBlockAllocatedMeter(Eigen::Vector3d position_m);

  //
  double getNodeSize(bool single_voxel);
  std::vector<std::pair<Eigen::Vector3d,
                        std::pair<int, double>>> getTreeLeafCentersDepthAndOccupancies();

  std::vector<Eigen::Vector3i> GetUnknownVoxels();
  //    std::vector<Eigen::Vector3d> getPathLeafCenters(Path<kDim>::Ptr path);

  //    /*
  //     * OccupancyWold I/O
  //     */
  bool GetMapBounds(Eigen::Vector3d &map_bounds_min_v, Eigen::Vector3d &map_bounds_max_v);
  bool GetMapBoundsMeter(Eigen::Vector3d &map_bounds_min_m, Eigen::Vector3d &map_bounds_max_m);
  bool inMapBounds(Eigen::Vector3d position_v);
  bool InMapBoundsMeter(Eigen::Vector3d position_m);
  double GetMapResolution();

 private:
  //    bool removeValue(std::istream &s);
  //    bool readValue(std::istream &s);

  void UpdateMapBounds();

  const int max_map_size_ = 65536;
  ////    const int octomap_shift_ = 32768;
  //
  Eigen::Vector3d map_bounds_max_ = Eigen::Vector3d(-1, -1, -1);
  Eigen::Vector3d map_bounds_min_ = Eigen::Vector3d(-1, -1, -1);

  int allocated_nodes_;

  double res_;
  std::string id_;
  std::vector<Eigen::Vector3i> voxels_;
  std::vector<float> voxel_values_;
  std::vector<se::key_t> alloc_list_;
  // se::Octree<OFusion>*  octree_ = NULL;
  std::shared_ptr<se::Octree<OFusion>> octree_ = NULL;
  //    std::shared_ptr<octomap::OcTree> octomap_ = NULL;
};
}  // namespace exploration
} // se

#endif  // OCCUPANCYWORLD_HPP
