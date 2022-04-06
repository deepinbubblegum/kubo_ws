# Mesh Navigation

The *Mesh Navigation* bundle provides software to perform efficient robot navigation on 2D-manifolds in 3D represented 
as triangular meshes. It allows to safely navigate in various complex outdoor environments by using a modular extendable
layerd mesh map. Layers can be loaded as plugins and represent certain geometric or semantic metrics of the terrain.
The layered mesh map is integrated with Move Base Flex (MBF) which provides a universal ROS action interface for path
planning and motion control, as well as for recovery behaviours. Thus, additional planner and controller plugins running
on the layered mesh map are provided.

Maintainer: [Sebastian Pütz](mailto:spuetz@uos.de)  
Author: [Sebastian Pütz](mailto:spuetz@uos.de)

* [Publications](#publications)
* [Installation](#installation)
* [Software Stack](#software-stack)
* [Mesh Map](#mesh-map)
* [Planners](#planners)
* [Controllers](#controllers)
* [Simulation](#simulation)
* [Demos](#demos)
* [Build Status](#build-status)

<img title="Demo Gif" src="docs/images/demo.gif?raw=true" alt="Demo Gif" width="600">

## Publications
Please reference the following papers when using the navigation stack in your scientific work.

#### Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots
```  
@inproceedings{puetz21cvp,
    author = {Pütz, Sebastian and Wiemann, Thomas and Kleine Piening, Malte and Hertzberg, Joachim},
    title = {Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots},
    booktitle = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
    year = 2021,
    url = {https://github.com/uos/mesh_navigation},
    note = {Software available at \url{https://github.com/uos/mesh_navigation}}
}
```
#### Move Base Flex: A Highly Flexible Navigation Framework for Mobile Robots
```  
@inproceedings{puetz18mbf,
    author = {Sebastian Pütz and Jorge Santos Simón and Joachim Hertzberg},
    title = {{Move Base Flex}: A Highly Flexible Navigation Framework for Mobile Robots},
    booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    year = 2018,
    month = {October},
    url = {https://github.com/magazino/move_base_flex},
    note = {Software available at \url{https://github.com/magazino/move_base_flex}}
}
```

## Installation

Please use the official released ros package or install more recent versions from source.  

`sudo apt install ros-melodic-mesh-navigation`

**Installation from source**  
All dependencies can be installed using rosdep  
`rosdep install mesh_navigation`

As explicit dependencies we refer to the following ROS packages, which are also developed by us:

* **[lvr2](https://github.com/uos/lvr2)**
* **[mesh_tools](https://github.com/uos/mesh_tools/)**
* **[move_base_flex](https://github.com/magazino/move_base_flex/)**

Use the **[pluto_robot](https://github.com/uos/pluto_robot)** package for example HDF5 map datasets, Gazebo simulations, and example configurations.

## Software Stack

This **[mesh_navigation](https://github.com/uos/mesh_navigation)** stack provides a navigation server for 
**[Move Base Flex (MBF)](https://github.com/magazino/move_base_flex)**. It provides a couple of configuration files and launch 
files to start the navigation server with the configured layer plugins for the layered mesh map, and the configured
planners and controller to perform path planning and motion control in 3D (or more specifically on 2D-manifold). 

The package structure is as follows:

- `mesh_navigation` The corresponding ROS meta package.

- `mbf_mesh_core` contains the plugin interfaces derived from the abstract MBF plugin interfaces to initialize 
  planner and controller plugins with one `mesh_map` instance. It provides the following three interfaces:
  
  - MeshPlanner - `mbf_mesh_core/mesh_planner.h`
  - MeshController - `mbf_mesh_core/mesh_controller.h`
  - MeshRecovery - `mbf_mesh_core/mesh_recovery.h`

- `mbf_mesh_nav` contains the mesh navigation server which build on top of the abstract MBF navigation server.
  It uses the plugin interfaces in `mbf_mesh_core` to load and initialize plugins of the types described above.

- `mesh_map` contains an implementation of a mesh map representation building on top of the mesh data structures
  in **[lvr2](https://github.com/uos/lvr2)**. This package provides a layered mesh map implementation. Layers can be 
  loaded as plugins to allow a highly configurable 3D navigation stack for robots traversing on the ground in outdoor
  and rough terrain.

- `mesh_layers` The package provides a couple of mesh layers to compute the trafficability of the terrain. 
  Furthermore, these plugins have access to the HDF5 map file and can load and store layer information. 
  The mesh layers can be configured for the robots abilities and needs. Currently we provide the following layer plugins:
  
  - HeightDiffLayer - `mesh_layers/HeightDiffLayer`
  - RoughnessLayer - `mesh_layers/RoughnessLayer`
  - SteepnessLayer - `mesh_layers/SteepnessLayer`
  - RidgeLayer - `mesh_layer/RidgeLayer`
  - InflationLayer - `mesh_layers/InflationLayer`

- `dijkstra_mesh_planner` contains a mesh planner plugin providing a path planning method based on Dijkstra's algorithm.
  It plans by using the edges of the mesh map. The propagation start a the goal pose, thus a path from every accessed 
  vertex to the goal pose can be computed. This leads in a sub-optimal potential field, which highly depends on the mesh 
  structure.

- `cvp_mesh_planner` contains a Fast Marching Method (FMM) wave front path planner to take the 2D-manifold into account.
  This planner is able to plan over the surface, due to that it results in shorter paths than the `dijkstra_mesh_planner`,
  since it is not restricted to the edges or topology of the mesh. A comparison is shown below. Please refer to the paper
  `Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots` which is stated above.

- `mesh_client` Is an experimental package to additionally load navigation meshes from a server.

### Path Planning and Motion Control

Use the `MeshGoal` tool to select a goal pose on the shown mesh in RViz. 

## Mesh Map

### Mesh Layers

The following table gives an overview of all currently implemented layer plugins available in the stack and the 
corresponding types tp specify for usage in the mesh map configuration. An example mesh map configuration is shown
below.

#### Overview of all layers

| Layer               | Plugin Type Specifier         | Description of Cost Computation          | Example Image                                                                           |
| ------------------- | ----------------------------- | ---------------------------------------- | --------------------------------------------------------------------------------------- |
| **HeightDiffLayer** | `mesh_layers/HeightDiffLayer` | local radius based height differences    | ![HeightDiffLayer](docs/images/costlayers/height_diff.jpg?raw=true "Height Diff Layer") |
| **RoughnessLayer**  | `mesh_layers/RoughnessLayer`  | local radius based normal fluctuation    | ![RoughnessLayer](docs/images/costlayers/roughness.jpg?raw=true "Roughness Layer")      |
| **SteepnessLayer**  | `mesh_layers/SteepnessLayer`  | arccos of the normal's z coordinate      | ![SteepnessLayer](docs/images/costlayers/steepness.jpg?raw=true "Steepness Layer")      |
| **RidgeLayer**      | `mesh_layer/RidgeLayer`       | local radius based distance along normal | ![RidgeLayer](docs/images/costlayers/ridge.jpg?raw=true "RidgeLayer")                   |
| **InflationLayer**  | `mesh_layers/InflationLayer`  | by distance to a lethal vertex           | ![InflationLayer](docs/images/costlayers/inflation.jpg?raw=true "Inflation Layer")      |

## Planners

### Usage with Move Base Flex

Currently the following planners are available:

#### Dijkstra Mesh Planner

```
  - name: 'dijkstra_mesh_planner'
    type: 'dijkstra_mesh_planner/DijkstraMeshPlanner'
```

#### Continuous Vector Field Planner

```
  - name: 'cvp_mesh_planner'
    type: 'cvp_mesh_planner/CVPMeshPlanner'
```

#### MMP Planner

```
  - name: 'mmp_planner'
    type: 'mmp_planner/MMPPlanner'
```

The planners are compared to each other.

| Vector Field Planner                                                                        | Dijkstra Mesh Planner                                                                              | ROS Global Planner on 2.5D DEM                                                     |
| ------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| ![VectorFieldPlanner](docs/images/stone_quarry/fmm_pot.jpg?raw=true "Vector Field Planner") | ![DijkstraMeshPlanner](docs/images/stone_quarry/dijkstra_pot.jpg?raw=true "Dijkstra Mesh Planner") | ![2D-DEM-Planner](docs/images/stone_quarry/dem_side.jpg?raw=true "2D DEM Planner") |

## Controllers

## Simulation

If you want to test the mesh navigation stack with Pluto please use the simulation setup and the corresponding launch
files below for the respective outdoor or rough terrain environment. The mesh tools have to be installed.
We developed the **[Mesh Tools](https://github.com/uos/mesh_tools)** as a package consisting of message definitions, RViz plugins and tools, as well as a
persistence layer to store such maps. These tools make the benefits of annotated triangle maps available in ROS and
allow to publish, edit and inspect such maps within the existing ROS software stack.

## Demos

In the following demo videos we used the developed VFP, i.e., the wavefront_propagatn_planner.
It will be renamed soon to vector_field_planner.

| Dataset and Description                  | Demo Video                                                                                                               |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| Botanical Garden of Osnabrück University | [![Mesh Navigation with Pluto](http://img.youtube.com/vi/qAUWTiqdBM4/0.jpg)](http://www.youtube.com/watch?v=qAUWTiqdBM4) |
| Stone Quarry in the Forest Brockum       | [![Mesh Navigation with acron19](http://img.youtube.com/vi/DFmv3wnIxug/0.jpg)](https://youtu.be/DFmv3wnIxug)             |

### Stone Quarry in the Forest in Brockum

| Colored Point Cloud                                                                              | Height Diff Layer                                                                                      | RGB Vertex Colors                                                                                       |
| ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------- |
| ![StoneQuarryPointCLoud](docs/images/stone_quarry/cloud.png?raw=true "Stone Quarry Point Cloud") | ![StoneQuarryHeightDiff](docs/images/stone_quarry/height_diff.jpg?raw=true "Stone Quarry Height Diff") | ![StoneQuarryVertexColors](docs/images/stone_quarry/mesh_rgb.jpg?raw=true "Stone Quarry Vertex Colors") |


## Build Status

| ROS Distro  | GitHub CI | Develop | Documentation | Source Deb | Binary Deb |
|-------------|-----------|---------|---------------|------------|------------|
| **Melodic** | [![Melodic CI](https://github.com/uos/mesh_navigation/workflows/Melodic%20CI/badge.svg)](https://github.com/uos/mesh_navigation/actions?query=workflow%3A%22Melodic+CI%22) | [![Build Dev Status](http://build.ros.org/buildStatus/icon?job=Mdev__mesh_navigation__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__mesh_navigation__ubuntu_bionic_amd64) | [![Build Doc Status](http://build.ros.org/buildStatus/icon?job=Mdoc__mesh_navigation__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdoc__mesh_navigation__ubuntu_bionic_amd64) | [![Build Src Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mesh_navigation__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mesh_navigation__ubuntu_bionic__source) | [![Build Bin Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mesh_navigation__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mesh_navigation__ubuntu_bionic_amd64__binary) |
| **Noetic**  | [![Noetic CI](https://github.com/uos/mesh_navigation/workflows/Noetic%20CI/badge.svg)](https://github.com/uos/mesh_navigation/actions?query=workflow%3A%22Noetic+CI%22) | [![Build Dev Status](http://build.ros.org/buildStatus/icon?job=Ndev__mesh_navigation__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__mesh_navigation__ubuntu_focal_amd64) | [![Build Doc Status](http://build.ros.org/buildStatus/icon?job=Ndoc__mesh_navigation__ubuntu_focal_amd64)](http://build.ros.org/job/Ndoc__mesh_navigation__ubuntu_focal_amd64) | [![Build Src Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mesh_navigation__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mesh_navigation__ubuntu_focal__source) | [![Build Bin Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mesh_navigation__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mesh_navigation__ubuntu_focal_amd64__binary) | 
