# Future tasks for supereight

## General
- Proper transformations [PRIORITY]
	- (world - map - body - camera)
- Efficient neighbor queries at voxel level [PRIORITY]
	- Generic function to get arbitrary neighbors and specialized (and faster)
	  function for common neighbors (face, face+edge, face+edge+corner etc.)
	- `get_face_neighbors(neighbor_list&) // Return pointers to neighbors`
	- `get_neighbors(coordinate_offsets, neighbor_list&) // Return pointers to
	  neighbors defined by relative offsets`
- Dense allocation [Almost done - Nils]
- Install script [Done, will merge - Sot]
- Return updated voxels. Return voxel indices/addresses and delta (changes in
  their stored values). Can then be filtered by a user-defined function.
- Proper camera model class instead of K matrix. Allow changing near/far plane
  during runtime. [Partially done - Sot]
- Read depth as float (meters) instead of uint16 (millimeters). [Almost done -
  Sot]
- Read RGB as float in the pipeline. [Almost done - Sot]



## Multires (Nils, backlog for Sot-Anna)
- OFusion
- Up/down propagation at the node level
- ESDF
- RGB input/fusion [Depends on reading RGB from General tasks]
- Multires marching cubes



## Integration/Wrappers
- ROS wrapper inputs
	- RGB
	- Depth
	- Ground truth pose
	- Constant transformations (camera to body etc.)
- ROS wrapper outputs
	- Tracked pose
	- Transformations
	- rviz
- Separate repo for ROS wrapper. For now have a basic visualization example,
  use as basis for more specialized code. For the time being, all
planning/exploration is developed within the ROS wrapper. In the future a
wrapper with more features should be created allowing to move the
planning/exploration in separate ROS nodes.



## Backlog
- Pangolin visualization



# Completed tasks

