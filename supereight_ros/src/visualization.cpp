 /**
 * Motion Planning, Visualization of octomap, path, trajectory, ... .
 *
 * Copyright (C) 2017 Imperial College London.
 * Copyright (C) 2017 ETH Zürich.
 *
 * @todo LICENSE
 *
 * @file visualization.cpp
 *
 * @author Marius Grimm
 * @date May 26, 2017
 */

//#include "ptp_ros/eigen_helpers_conversions.hpp"
#include "supereight_ros/visualization.hpp"

namespace se {

Visualization::Visualization(const VisualizationBag& parameter) :
    parameter_(parameter) {}

/**
 *
 * Modified from https://github.com/ethz-asl/volumetric_mapping 
 */
void Visualization::generateMarkerArray(const OccupancyWorld::Ptr&       ow,
                                        const std::string&               frame_id,
                                        visualization_msgs::MarkerArray* occupied_nodes,
                                        visualization_msgs::MarkerArray* free_nodes) {
  ROS_ERROR_COND(occupied_nodes == nullptr,
                 "%soccupied_nodes == nullptr", kOutputPrefixVis.c_str());
  ROS_ERROR_COND(free_nodes == nullptr,
                 "%sfree_nodes == nullptr", kOutputPrefixVis.c_str());

  /* In the marker array, assign each node to its respective depth level, since
   * all markers in a CUBE_LIST must have the same scale. */
  occupied_nodes->markers.resize(2);
  free_nodes->markers.resize(2);

  /* Metric min and max z of the map: */
  Eigen::Vector3d min_bound, max_bound;
  ow->getMapBoundsMeter(min_bound, max_bound);

  /* Update values from params if necessary */
  // if (parameter_.visualize_min_z && (parameter_.min_z > min_bound[2])) {
  //   min_bound[2] = parameter_.min_z;
  // }
  // if (parameter_.visualize_max_z && (parameter_.max_z < max_bound[2])) {
  //   max_bound[2] = parameter_.max_z;
  // }

  for (int i = 0; i < 2; ++i) {
    const double size = ow->getNodeSize(i);
    occupied_nodes->markers[i].header.frame_id = frame_id;
    occupied_nodes->markers[i].ns = "map";
    occupied_nodes->markers[i].id = i;
    occupied_nodes->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupied_nodes->markers[i].scale.x = size;
    occupied_nodes->markers[i].scale.y = size;
    occupied_nodes->markers[i].scale.z = size;

    free_nodes->markers[i] = occupied_nodes->markers[i];
  }

  std::vector<std::pair<Eigen::Vector3d, std::pair<int, double> >> leaf_centers_and_depth =
          ow->getTreeLeafCentersDepthAndOccupancies();

  for (const auto& leaf : leaf_centers_and_depth) {
    geometry_msgs::Point cube_center;
    cube_center.x = leaf.first[0];
    cube_center.y = leaf.first[1];
    cube_center.z = leaf.first[2];

    if (cube_center.z > max_bound[2] || cube_center.z < min_bound[2]) {
      continue;
    }

    const std::pair<int, double> leaf_content = leaf.second;

    if (leaf_content.second > 0.0) {
      occupied_nodes->markers[leaf_content.first].points.push_back(cube_center);
      occupied_nodes->markers[leaf_content.first].colors.push_back(
              percentToColor(colorizeMapByHeight(
                      leaf.first[2], min_bound[2], max_bound[2])));
    } else {
      free_nodes->markers[leaf_content.first].points.push_back(cube_center);
      free_nodes->markers[leaf_content.first].colors.push_back(
              percentToColor(colorizeMapByHeight(
                      leaf.first[2], min_bound[2], max_bound[2])));
    }
  }

  for (int i = 0; i < 2; ++i) {
    if (occupied_nodes->markers[i].points.size() > 0) {
      occupied_nodes->markers[i].action = visualization_msgs::Marker::ADD;
    } else {
      occupied_nodes->markers[i].action = visualization_msgs::Marker::DELETE;
    }

    if (free_nodes->markers[i].points.size() > 0) {
      free_nodes->markers[i].action = visualization_msgs::Marker::ADD;
    } else {
      free_nodes->markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
}

void Visualization::generateMapMarkerArray(const OccupancyWorld::Ptr&       ow,
                                           const std::string&               frame_id,
                                           visualization_msgs::Marker* occupied_voxels) {
  ROS_ERROR_COND(occupied_voxels == nullptr,
                 "%soccupied_voxels == nullptr", kOutputPrefixVis.c_str());

  /* In the marker array, assign each node to its respective depth level, since
   * all markers in a CUBE_LIST must have the same scale. */
  double res = ow->getMapResolution();
  // std::cout << "RES = " << res << std::endl;

  occupied_voxels->header.frame_id = frame_id;
  occupied_voxels->ns = "map";
  occupied_voxels->id = 0;
  occupied_voxels->type = visualization_msgs::Marker::CUBE_LIST;
  occupied_voxels->scale.x = res;
  occupied_voxels->scale.y = res;
  occupied_voxels->scale.z = res;

  std::vector<Eigen::Vector3i,Eigen::aligned_allocator<Eigen::Vector3i>> leaf_centers = ow->getOccupiedVoxel();

  int max(0);
  int min(3000);
  for (const auto& leaf : leaf_centers) {
    geometry_msgs::Point cube_center;

    if (leaf[2] < min && leaf[0] > 8)
      min = leaf[2];
    if (leaf[2] > max)
      max = leaf[2];

    // std::cout << "Voxel Coord = " << leaf[0] << ", " << leaf[1] << ", " << leaf[2] << std::endl;
    cube_center.x = ((double)leaf[0] + 0.5) * res;
    cube_center.y = ((double)leaf[1] + 0.5) * res;
    cube_center.z = ((double)leaf[2] + 0.5) * res;
    // std::cout << "Cube Coord  = " << cube_center.x << ", " << cube_center.y << ", " << cube_center.z << std::endl;

    occupied_voxels->points.push_back(cube_center);
    occupied_voxels->colors.push_back(percentToColor(0.5));
  }

  // std::cout << "MAX Z  [vox] = " << max << std::endl;
  // std::cout << "MIN Z  [vox] = " << min << std::endl;
  // std::cout << "DIFF Z [vox] = " << max - min << std::endl;

  if (occupied_voxels->points.size() > 0) {
    occupied_voxels->action = visualization_msgs::Marker::ADD;
  } else {
    occupied_voxels->action = visualization_msgs::Marker::DELETE;
  }
}

//void Visualization::generateVoxelTubeMarkerArray(const OccupancyWorld::Ptr&       ow,
//                                                 const std::string&               frame_id,
//                                                 const Path<kDim>::Ptr            path,
//                                                 visualization_msgs::MarkerArray* tube_nodes) {
//  ROS_ERROR_COND(tube_nodes == nullptr,
//                 "%stube_nodes == nullptr", kOutputPrefixVis.c_str());
//
//  /* In the marker array, assign each node to its respective depth level, since
//   * all markers in a CUBE_LIST must have the same scale. */
//  tube_nodes->markers.resize(1);
//
//  /* Metric min and max z of the map: */
//  Eigen::Vector3d min_bound, max_bound;
//  ow->getMapBoundsMeter(min_bound, max_bound);
//
//  const double size = ow->getNodeSize(1);
//  tube_nodes->markers[0].header.frame_id = frame_id;
//  tube_nodes->markers[0].ns = "map";
//  tube_nodes->markers[0].id = 0;
//  tube_nodes->markers[0].type = visualization_msgs::Marker::CUBE_LIST;
//  tube_nodes->markers[0].scale.x = size;
//  tube_nodes->markers[0].scale.y = size;
//  tube_nodes->markers[0].scale.z = size;
//
//  std::vector<Eigen::Vector3d> leaf_centers_and_depth = ow->getPathLeafCenters(path);
//
//  for (const auto& leaf : leaf_centers_and_depth) {
//    geometry_msgs::Point cube_center;
//    cube_center.x = leaf[0];
//    cube_center.y = leaf[1];
//    cube_center.z = leaf[2];
//
//    if (cube_center.z > max_bound[2] || cube_center.z < min_bound[2]) {
//      continue;
//    }
//
//    tube_nodes->markers[0].points.push_back(cube_center);
//    tube_nodes->markers[0].colors.push_back(
//            percentToColor(colorizeMapByHeight(
//                    leaf[2], min_bound[2], max_bound[2])));
//
//  }
//
//  if (tube_nodes->markers[0].points.size() > 0) {
//    tube_nodes->markers[0].action = visualization_msgs::Marker::ADD;
//  } else {
//    tube_nodes->markers[0].action = visualization_msgs::Marker::DELETE;
//  }
//
//}
//
//void Visualization::generateControlPointTubeMarkerArray(const OccupancyWorld::Ptr&        ow,
//                                                        const std::string&                frame_id,
//                                                        const Path<kDim>::Ptr             path,
//                                                        visualization_msgs::MarkerArray*  tube_nodes) {
//  ROS_ERROR_COND(tube_nodes == nullptr,
//                 "%stube_nodes == nullptr", kOutputPrefixVis.c_str());
//
//  /* In the marker array, assign each node to its respective depth level, since
//   * all markers in a CUBE_LIST must have the same scale. */
//  int n_vertices = path->states.size();
//  tube_nodes->markers.resize(2 * n_vertices - 1);
//
//  /* Metric min and max z of the map: */
//  Eigen::Vector3d min_bound, max_bound;
//  ow->getMapBoundsMeter(min_bound, max_bound);
//
//  std::vector<std::pair<double, double>> segment_radii;
//  for (std::vector<ptp::State<ptp::kDim>>::iterator it_i = path->states.begin(); it_i != path->states.end(); ++it_i) {
//    if (std::next(it_i) != path->states.end()) {
//      if (it_i != path->states.begin()) {
//        std::pair<double, double> radius((*it_i).segment_radius,
//                                         std::min((*std::next(it_i)).segment_radius, (*it_i).segment_radius));
//        segment_radii.push_back(radius);;
//      }
//    } else {
//      std::pair<double, double> radius((*it_i).segment_radius, (*it_i).segment_radius);
//      segment_radii.push_back(radius);
//    }
//  }
//
//  for (int sphere_idx = 0; sphere_idx < n_vertices; sphere_idx++) {
//    Eigen::Vector3d sphere_position = path->states[sphere_idx].segment_end;
//    double sphere_radius;
//    if (sphere_idx == 0) {
//      sphere_radius = segment_radii[0].first;
//    } else {
//      sphere_radius = segment_radii[sphere_idx-1].second;
//    }
//    tube_nodes->markers[sphere_idx].header.frame_id = frame_id;
//    tube_nodes->markers[sphere_idx].ns = "map";
//    tube_nodes->markers[sphere_idx].id = sphere_idx;
//    tube_nodes->markers[sphere_idx].type = visualization_msgs::Marker::SPHERE;
//    tube_nodes->markers[sphere_idx].scale.x = sphere_radius * 2;
//    tube_nodes->markers[sphere_idx].scale.y = sphere_radius * 2;
//    tube_nodes->markers[sphere_idx].scale.z = sphere_radius * 2;
//    tube_nodes->markers[sphere_idx].pose.position.x = sphere_position[0];
//    tube_nodes->markers[sphere_idx].pose.position.y = sphere_position[1];
//    tube_nodes->markers[sphere_idx].pose.position.z = sphere_position[2];
//    tube_nodes->markers[sphere_idx].color.a = 0.7; /* Don't forget to set the alpha! */
//    tube_nodes->markers[sphere_idx].color.r = 1.0;
//    tube_nodes->markers[sphere_idx].color.g = 0.0;
//    tube_nodes->markers[sphere_idx].color.b = 1.0;
//  }
//
//  for (int tube_idx = 0; tube_idx < n_vertices - 1; tube_idx++) {
//    Eigen::Vector3d start_m = path->states[tube_idx].segment_end;
//    Eigen::Vector3d end_m = path->states[tube_idx + 1].segment_end;
//    Eigen::Vector3d path_seg_axis = (end_m - start_m);
//    path_seg_axis.normalize();
//
//    Eigen::Vector3d start_cylinder_m;
//    Eigen::Vector3d end_cylinder_m;
//    if (tube_idx == 0) {
//      start_cylinder_m = start_m - path_seg_axis * segment_radii[0].first;
//    } else {
//      start_cylinder_m = start_m - path_seg_axis * segment_radii[tube_idx - 1].second;
//    }
//    end_cylinder_m = end_m + path_seg_axis * segment_radii[tube_idx].second;
//
//
//    Eigen::Vector3d cylinder_position = (end_cylinder_m + start_cylinder_m) / 2;
//    Eigen::Vector3d path_seg_length_vec = end_cylinder_m - start_cylinder_m;
//    double path_seg_length = path_seg_length_vec.norm();
//
//    /* Compute rotation and translation and transform points */
//    Eigen::Vector3d z_axis(0,0,1);
//    double rot_angle = acos(z_axis.dot(path_seg_axis));
//    Eigen::Vector3d rot_axis = z_axis.cross(path_seg_axis) / (sin(rot_angle));
//    Eigen::Quaternion<double> q;  q = Eigen::AngleAxis<double>(rot_angle, rot_axis);
//    double sphere_radius = segment_radii[tube_idx].first;
//    tube_nodes->markers[n_vertices + tube_idx].header.frame_id = frame_id;
//    tube_nodes->markers[n_vertices + tube_idx].ns = "map";
//    tube_nodes->markers[n_vertices + tube_idx].id = n_vertices + tube_idx;
//    tube_nodes->markers[n_vertices + tube_idx].type = visualization_msgs::Marker::CYLINDER;
//    tube_nodes->markers[n_vertices + tube_idx].scale.x = sphere_radius * 2;
//    tube_nodes->markers[n_vertices + tube_idx].scale.y = sphere_radius * 2;
//    tube_nodes->markers[n_vertices + tube_idx].scale.z = path_seg_length;
//    tube_nodes->markers[n_vertices + tube_idx].pose.position.x = cylinder_position[0];
//    tube_nodes->markers[n_vertices + tube_idx].pose.position.y = cylinder_position[1];
//    tube_nodes->markers[n_vertices + tube_idx].pose.position.z = cylinder_position[2];
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.x = q.x();
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.y = q.y();
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.z = q.z();
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.w = q.w();
//    tube_nodes->markers[n_vertices + tube_idx].color.a = 0.5; /* Don't forget to set the alpha! */
//    tube_nodes->markers[n_vertices + tube_idx].color.r = 1.0;
//    tube_nodes->markers[n_vertices + tube_idx].color.g = 0.4;
//    tube_nodes->markers[n_vertices + tube_idx].color.b = 0.4;
//  }
//}
//
//void Visualization::generateSafetyTubeMarkerArray(const OccupancyWorld::Ptr&        ow,
//                                                  const std::string&                frame_id,
//                                                  const Path<kDim>::Ptr             path,
//                                                  PlanningParameter                 pp,
//                                                  visualization_msgs::MarkerArray*  tube_nodes) {
//  ROS_ERROR_COND(tube_nodes == nullptr,
//                 "%stube_nodes == nullptr", kOutputPrefixVis.c_str());
//
//  /* In the marker array, assign each node to its respective depth level, since
//     all markers in a CUBE_LIST must have the same scale. */
//  int n_vertices = path->states.size();
//  tube_nodes->markers.resize(2 * n_vertices - 1);
//
//  /* Metric min and max z of the map: */
//  Eigen::Vector3d min_bound, max_bound;
//  ow->getMapBoundsMeter(min_bound, max_bound);
//
//  double min_flight_corridor_radius = pp.robot_radius + pp.min_control_point_radius;;
//
//  for (int sphere_idx = 0; sphere_idx < n_vertices; sphere_idx++) {
//    Eigen::Vector3d sphere_position = path->states[sphere_idx].segment_end;
//    tube_nodes->markers[sphere_idx].header.frame_id = frame_id;
//    tube_nodes->markers[sphere_idx].ns = "map";
//    tube_nodes->markers[sphere_idx].id = sphere_idx;
//    tube_nodes->markers[sphere_idx].type = visualization_msgs::Marker::SPHERE;
//    tube_nodes->markers[sphere_idx].scale.x = min_flight_corridor_radius * 2;
//    tube_nodes->markers[sphere_idx].scale.y = min_flight_corridor_radius * 2;
//    tube_nodes->markers[sphere_idx].scale.z = min_flight_corridor_radius * 2;
//    tube_nodes->markers[sphere_idx].pose.position.x = sphere_position[0];
//    tube_nodes->markers[sphere_idx].pose.position.y = sphere_position[1];
//    tube_nodes->markers[sphere_idx].pose.position.z = sphere_position[2];
//    tube_nodes->markers[sphere_idx].color.a = 1.0; /* Don't forget to set the alpha! */
//    tube_nodes->markers[sphere_idx].color.r = 0.0;
//    tube_nodes->markers[sphere_idx].color.g = 1.0;
//    tube_nodes->markers[sphere_idx].color.b = 0.0;
//  }
//
//  for (int tube_idx = 0; tube_idx < n_vertices - 1; tube_idx++) {
//    Eigen::Vector3d start_m = path->states[tube_idx].segment_end;
//    Eigen::Vector3d end_m = path->states[tube_idx + 1].segment_end;
//    Eigen::Vector3d path_seg_axis = (end_m - start_m);
//    path_seg_axis.normalize();
//
//    Eigen::Vector3d start_cylinder_m = start_m - path_seg_axis * min_flight_corridor_radius;
//    Eigen::Vector3d end_cylinder_m = end_m + path_seg_axis * min_flight_corridor_radius;
//    Eigen::Vector3d cylinder_position = (end_cylinder_m + start_cylinder_m) / 2;
//    Eigen::Vector3d path_seg_length_vec = end_cylinder_m - start_cylinder_m;
//    double path_seg_length = path_seg_length_vec.norm();
//
//    /* Compute rotation and translation and transform points */
//    Eigen::Vector3d z_axis(0,0,1);
//    double rot_angle = acos(z_axis.dot(path_seg_axis));
//    Eigen::Vector3d rot_axis = z_axis.cross(path_seg_axis) / (sin(rot_angle));
//    Eigen::Quaternion<double> q;  q = Eigen::AngleAxis<double>(rot_angle, rot_axis);
//    tube_nodes->markers[n_vertices + tube_idx].header.frame_id = frame_id;
//    tube_nodes->markers[n_vertices + tube_idx].ns = "map";
//    tube_nodes->markers[n_vertices + tube_idx].id = n_vertices + tube_idx;
//    tube_nodes->markers[n_vertices + tube_idx].type = visualization_msgs::Marker::CYLINDER;
//    tube_nodes->markers[n_vertices + tube_idx].scale.x = min_flight_corridor_radius * 2;
//    tube_nodes->markers[n_vertices + tube_idx].scale.y = min_flight_corridor_radius * 2;
//    tube_nodes->markers[n_vertices + tube_idx].scale.z = path_seg_length;
//    tube_nodes->markers[n_vertices + tube_idx].pose.position.x = cylinder_position[0];
//    tube_nodes->markers[n_vertices + tube_idx].pose.position.y = cylinder_position[1];
//    tube_nodes->markers[n_vertices + tube_idx].pose.position.z = cylinder_position[2];
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.x = q.x();
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.y = q.y();
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.z = q.z();
//    tube_nodes->markers[n_vertices + tube_idx].pose.orientation.w = q.w();
//    tube_nodes->markers[n_vertices + tube_idx].color.a = 1.0; /* Don't forget to set the alpha! */
//    tube_nodes->markers[n_vertices + tube_idx].color.r = 0.0;
//    tube_nodes->markers[n_vertices + tube_idx].color.g = 1.0;
//    tube_nodes->markers[n_vertices + tube_idx].color.b = 0.0;
//  }
//}


/* Taken from https://github.com/ethz-asl/volumetric_mapping */
double Visualization::colorizeMapByHeight(double z, 
                                          double min_z,
                                          double max_z) const {
  return (1.0 - std::min(std::max((z - min_z) / (max_z - min_z), 0.0), 1.0));
}

/* Taken from https://github.com/ethz-asl/volumetric_mapping */
std_msgs::ColorRGBA Visualization::percentToColor(double h) const {
  /* Helen's note: direct copy from OctomapProvider. */
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  /* blend over HSV-values (more colors) */

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  /* if i is even */
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}


} // namespace se
