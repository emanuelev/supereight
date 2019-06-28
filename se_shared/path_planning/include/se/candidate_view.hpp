//
// Created by anna on 27/06/19.
//

#ifndef SUPEREIGHT_CANDIDATE_VIEW_HPP
#define SUPEREIGHT_CANDIDATE_VIEW_HPP

#include <set>
#include <map>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <iterator>

#include <Eigen/StdVector>
#include <se/continuous/volume_template.hpp>
#include <se/octree.hpp>
#include <se/node_iterator.hpp>

#include <se/config.h>
template<typename T> using Volume = VolumeTemplate<T, se::Octree>;
namespace se {
struct pose3D {
  Eigen::Vector3f p;
  Eigen::Quaternionf q;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  pose3D(Eigen::Vector3f point, Eigen::Quaternionf quat) : p(point), q(quat){}
};

static inline std::ostream &operator<<(std::ostream &os, const pose3D &pose) {
  return os << pose.p.x() << pose.p.y() << pose.p.z() << pose.q.x() << pose.q.y() << pose.q.z()
            << pose.q.z();
}
static inline std::istream& operator>>(std::istream& input, pose3D& pose) {
  input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >>
        pose.q.y() >> pose.q.z() >> pose.q.w();
  // Normalize the quaternion to account for precision loss due to
  // serialization.
  pose.q.normalize();
  return input;
}



static Eigen::IOFormat InLine(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " ",
    ";");
typedef std::vector<pose3D, Eigen::aligned_allocator<pose3D> > posevector;

namespace exploration {
typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > vec3i;

typedef std::map<int,
                 vec3i,
                 std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, vec3i> > > mapvec3i;

typedef std::map<uint64_t,
                 Eigen::Vector3i,
                 std::less<uint64_t>,
                 Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector3i> > > map3i;

template<typename T>
static void printFaceVoxels(const Volume<T> & volume, const Eigen::Vector3i &_voxel) {
  vec3i face_neighbour_voxel(6);
  face_neighbour_voxel[0] << _voxel.x() - 1, _voxel.y(), _voxel.z();
  face_neighbour_voxel[1] << _voxel.x() + 1, _voxel.y(), _voxel.z();
  face_neighbour_voxel[2] << _voxel.x(), _voxel.y() - 1, _voxel.z();
  face_neighbour_voxel[3] << _voxel.x(), _voxel.y() + 1, _voxel.z();
  face_neighbour_voxel[4] << _voxel.x(), _voxel.y(), _voxel.z() - 1;
  face_neighbour_voxel[5] << _voxel.x(), _voxel.y(), _voxel.z() + 1;
  std::cout<< "[se/cand view] face voxel states " ;
  for (const auto &face_voxel : face_neighbour_voxel) {
    std::cout << volume._map_index->get(face_voxel).st << " ";
  }
  std::cout << std::endl;
}

template<typename T>
static Eigen::Vector3i getOffsetCandidate(const Volume<T> &volume, const Eigen::Vector3i &cand_v,
                                          const vec3i &frontier_voxels,
                                          const double res) {
  Eigen::Vector3i offset_cand_v(0, 0, 0);
  float offset = 0.4; // [m]
  std::cout << "[se/cand view/getoffset] res " << res << std::endl;
  // curr res 24m/128 vox = 0.1875 m/vx
  int offset_v = static_cast<int>(offset / res); // 2 vox
  // fit plane through frontier voxels in the block
// source https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
// https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
  const float num_points = frontier_voxels.size();
  if (num_points < 3) return Eigen::Vector3i(0, 0, 0);
  Eigen::Vector3i sum(0, 0, 0);
  for (auto p : frontier_voxels) {
    sum += p;
  }
  Eigen::Vector3f centroid = sum.cast<float>() * (1.0f / num_points);
  // calculate full 3x3 covariance matrix, excluding symmetries
  // relative around the centroid
  float xx = 0.0f, xy = 0.0f, xz = 0.0f, yy = 0.0f, yz = 0.0f, zz = 0.0f;
  for (auto p : frontier_voxels) {
    Eigen::Vector3f dist = p.cast<float>() - centroid;
    xx += dist.x() * dist.x();
    xy += dist.x() * dist.y();
    xz += dist.x() * dist.z();
    yy += dist.y() * dist.y();
    yz += dist.y() * dist.z();
    zz += dist.z() * dist.z();
  }
  xx /= num_points;
  xy /= num_points;
  xz /= num_points;
  yy /= num_points;
  yz /= num_points;
  zz /= num_points;

  float det_x = yy * zz - yz * yz;
  float det_y = xx * zz - xz * xz;
  float det_z = xx * yy - xy * xy;

  float max_det = std::max({det_x, det_y, det_z});

  std::cout << "[se/cand view] max det " << max_det << std::endl;
  if (max_det <= 0.0f) {
    return Eigen::Vector3i(0, 0, 0);
  }
  // find normal
  Eigen::Vector3f normal;
  if (max_det == det_x) {
    normal.x() = det_x;
    normal.y() = xz * yz - xy * zz;
    normal.z() = xy * yz - xz * yy;
  } else if (max_det == det_y) {

    normal.x() = xz * yz - xy * zz;
    normal.y() = det_y;
    normal.z() = xy * yz - xz * xx;
  } else if (max_det == det_z) {

    normal.x() = xz * yz - xy * yy;
    normal.y() = xy * xz - yz * xx;
    normal.z() = det_z;
  }
  normal = normal.normalized();
  if (std::abs(normal.dot(cand_v.cast<float>())) > 0.01) {
    std::cout << "[se/cand view] cand point is " << normal.dot(cand_v.cast<float>())
              << " voxel dist away from plane" << std::endl;
  }
  // offset candidate
  offset_cand_v.x() = ceil(normal.x()*offset_v);
  offset_cand_v.y() = ceil(normal.y()*offset_v);
  offset_cand_v.z() = ceil(normal.z()*offset_v);
  offset_cand_v = cand_v + offset_cand_v.cast<int>();
  // TODO check which direction is into freespace
  std::cout << "[se/cand view] normal "<< normal.x() << " " <<normal.y() << " " << normal.z() <<
  std::endl;
  printFaceVoxels(volume, offset_cand_v);

  if(volume._map_index->get(offset_cand_v).st != voxel_state::kFree){

    offset_cand_v.x() = ceil(-normal.x()*offset_v);
    offset_cand_v.y() = ceil(-normal.y()*offset_v);
    offset_cand_v.z() = ceil(-normal.z()*offset_v);
    offset_cand_v = cand_v + offset_cand_v.cast<int>();
    printFaceVoxels(volume, offset_cand_v);
    if(volume._map_index->get(offset_cand_v).st == voxel_state::kFree) {
      normal = -normal;
    } else{
      std::cout << "[se/cand view] no free voxel after offset. next candidate" << std::endl;
//      return Eigen::Vector3i(0,0,0);
    }
  }
  std::cout << " [se/cand view] candidate " << cand_v.x() << " " << cand_v.y() << " " << cand_v.z()
            << " offset by " << offset_v << " results in " << offset_cand_v.x() << " "
            << offset_cand_v.y() << " " << offset_cand_v.z() << " voxel state " <<
            volume._map_index->get(offset_cand_v).st << std::endl;


  return offset_cand_v;

}
// function generate candidate views
// input mapvec3i< morton code, vector with frontier voxel in the voxel block>, pointer to octree
// return a vector with possible candidates [voxel coord]



template<typename T>
vec3i getCandidateViews(const Volume<T> &volume,
                        const map3i &frontier_blocks_map,
                        const double res,
                        const int  num_cand_views) {
  vec3i cand_views;
  mapvec3i frontier_voxels_map;
  node_iterator<T> node_it(*(volume._map_index));
  std::vector<int> max_num_frontier_voxel;
  uint64_t max_morton = 0;

  // get all frontier voxels inside a voxel block
  // TODO
  for (const auto &frontier_block : frontier_blocks_map) {
    vec3i frontier_voxels = node_it.getFrontierVoxels(frontier_block.first);
/*    if (frontier_voxels.size() > max_num_frontier_voxel) {
      max_num_frontier_voxel = frontier_voxels.size();
      max_morton = frontier_block.first;
    }*/
    frontier_voxels_map[frontier_block.first] = frontier_voxels;

  }
  std::cout << "[se/cand view] frontier voxels map size " << frontier_blocks_map.size()
            << std::endl;

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution_block(0, frontier_blocks_map.size() - 1);

  // generate cand views
  for (int i = 0 ; i <= num_cand_views ; i++) {
    auto it = frontier_voxels_map.begin();
    std::advance(it, distribution_block(generator));
    uint64_t rand_morton = it->first;
    std::uniform_int_distribution<int> distribution_voxel(0, frontier_voxels_map[rand_morton].size() -
    1);
    int rand_voxel = distribution_voxel(generator);
    Eigen::Vector3i candidate_frontier_voxel = frontier_voxels_map[rand_morton].at(rand_voxel);
    std::cout << "[se/cand view] rand morton " << rand_morton << " rand voxel " << rand_voxel
    << std::endl;
    std::cout << "[se/cand view] candidate frontier voxel " << candidate_frontier_voxel.x() << " "
              << candidate_frontier_voxel.y() << " " << candidate_frontier_voxel.z() << std::endl;
    vec3i frontier_voxels_vec = frontier_voxels_map[rand_morton];
//  random sample N views
// from morton code with max frontier voxels, generate a cand view
    Eigen::Vector3i cand_view_v =
        getOffsetCandidate(volume, candidate_frontier_voxel, frontier_voxels_vec, res);
    std::cout << "[se/cand view] cand view v " << cand_view_v.format(InLine) << std::endl;
    if (cand_view_v != Eigen::Vector3i(0, 0, 0)) {
      cand_views.push_back(cand_view_v);
    }
  }
  std::cout << "[se/cand view] getCandView size " << cand_views.size() << std::endl;
  return cand_views;
//https://eigen.tuxfamily.org/dox/classEigen_1_1Hyperplane.html#a23f225bb36b10ce116ca97d2ca7aa345
// find the surface normal of the candidate view
// offset view along the normal
// check for collision

}
// function: check for collision for the offset candidate view
//
// maybe use what nils has implemented?

// function:

// function planning called from denseSLAM pipeline
// returns vector with pose => path
// calls frunction from above
//similar structure to projective functor


template<typename T>
void getExplorationPath(const Volume<T> &volume,
                        const map3i &frontier_map,
                        const double &res,
                        const Planning_Configuration &config,
                        posevector &path,
                        vec3i &cand_views) {
//  std::cout << " [se/candidate_view] getExplorationPath "  << std::endl;
  cand_views= getCandidateViews(volume, frontier_map, res, config.num_cand_views);
  std::cout << "[se/cand view] cand view length " << cand_views.size() << std::endl;
  pose3D tmp_pose({0.f, 0.f, 0.f},{0.f, 0.f, 0.f, 1.f});
  path.push_back(tmp_pose);



}

} // namespace exploration
} // namespace se


#endif //SUPEREIGHT_CANDIDATE_VIEW_HPP
