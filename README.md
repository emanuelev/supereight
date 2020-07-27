# supereight: a fast octree library for Dense SLAM
welcome to *supereight*: a high performance template octree library and a dense
volumetric SLAM pipeline implementation.

# Related publications
This software implements the octree library and dense SLAM system presented in
our paper
[Efficient Octree-Based Volumetric SLAM Supporting Signed-Distance and
Occupancy
Mapping.](https://spiral.imperial.ac.uk/bitstream/10044/1/55715/2/EVespaRAL_final.pdf)
If you publish work that relates to this software,
please cite our paper as:

`@ARTICLE{VespaRAL18, 
author={E. Vespa and N. Nikolov and M. Grimm and L. Nardi and P. H. J. Kelly
and S. Leutenegger}, 
journal={IEEE Robotics and Automation Letters}, 
title={Efficient Octree-Based Volumetric SLAM Supporting Signed-Distance and
Occupancy Mapping}, year={2018}, volume={3}, number={2}, pages={1144-1151}, 
doi={10.1109/LRA.2018.2792537}, ISSN={}, month={April}}`

# Licence
The core library is released under the BSD 3-clause Licence. There are part of
the this software that are released under MIT licence, see individual headers
for which licence applies.

# Project structure
supereight is made of three main different components:

* `core`: the main header only template octree library
* `backend`: the implementations of allocation, integration and
  raycasting for the octree. This is built as a static library for every
  combination of backend (currently OpenMP and CUDA) and field type. The field
  types are defined in `backend/CMakeLists.txt`
* `tracking`: the ICP based tracking implementation
* `denseslam`: the volumetric SLAM pipelines presented in [1], which can be
  compiled in a library and used in external projects. Notice that the pipeline
  API exposes the discrete octree map via a shared_ptr. This is built as a
  static library for every version of `backend` that was built
* `apps`: front-end applications which run the se-denseslam pipelines on
  given inputs or live camera.

Additionally, `tools` includes the dataset generation tool and some libraries
required by `denseslam` and `apps`.

# Dependencies
The following packages are required to build the `denseslam` library:
* CMake >= 3.10
* Eigen3 (use latest from https://gitlab.com/libeigen/eigen)
* Sophus (use latest from https://github.com/strasdat/Sophus)
* CUDA >= 10.1 (optional, for GPU backend)
* CUDA CUB >= 1.8.0 (optional, for GPU backend)
* OpenMP (optional)
* GTest

The benchmarking and GUI apps additionally require:
* GLut
* OpenGL
* OpenNI2
* PkgConfig/Qt5
* Python/Numpy for evaluation scripts

# Build
From the project root:
`make`
This will create a build/ folder from which `cmake ..` is invoked.

# Usage example
To run one of the apps in apps you need to first produce an input file. We
use SLAMBench 1.0 file format (https://github.com/pamela-project/slambench).
The tool scene2raw can be used to produce an input sequence from the ICL-NUIM
dataset:
```
mkdir living_room_traj2_loop
cd living_room_traj2_loop
wget http://www.doc.ic.ac.uk/~ahanda/living_room_traj2_loop.tgz
tar xzf living_room_traj2_loop.tgz
cd ..
build/tools/scene2raw living_room_traj2_loop living_room_traj2_loop/scene.raw
```
Then it can be used as input to one of the apps

```
./build/apps/supereight-denseslam-cuda-ofusion-main -i living_room_traj2_loop/scene.raw -s 4.8 -p 0.34,0.5,0.24 -z 4 -c 2 -r 1 -k 481.2,-480,320,240  > benchmark.log
```
