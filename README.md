# ROS Wrapper for GPD

* [Author's website](http://www.ccs.neu.edu/home/atp/)
* [License](https://github.com/atenpas/gpd2_ros/blob/master/LICENSE.md)
* [GPD library](https://github.com/atenpas/gpd2)

## Overview

A ROS wrapper around the [GPD](https://github.com/atenpas/gpd2) package for detecting 6-DOF grasp poses for a
2-finger robot hand (e.g., a parallel jaw gripper) in 3D point clouds.

## 1) Installation

The following instructions have been tested on **Ubuntu 16.04**. Similar
instructions should work for other Linux distributions.

1. Install GPD. You can follow [these instructions](https://github.com/atenpas/gpd2#install).

2. Clone this repository into the `src` folder of your catkin workspace:

   ```
   cd <location_of_your_workspace>/src
   git clone https://github.com/atenpas/gpd2_ros
   ```

3. Build your catkin workspace:

   ```
   cd <location_of_your_workspace>
   catkin_make
   ```

## 2) Generate Grasps for a Point Cloud on a ROS Topic

As C++ has issues with relative file paths, we first need to modify the config
file in your `gpd` folder, e.g., `<location_of_gpd>/cfg/ros_eigen_params.cfg`.
Search for parameters that have absolute file paths and change them according
to your system.

Now, we can run GPD as a ROS node. The following command will launch a ROS node
that waits for point clouds on the ROS topic `/cloud_stitched`. Once a point
cloud is received, the node will search the cloud for grasps.

```
roslaunch gpd_ros ur5.launch
```

## 3) Troubleshooting

If `catkin_make` cannot find `libinference_engine.so`, make sure that `LD_LIBRARY_PATH` contains the path to that library:

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:path_to_dldt/inference-engine/bin/intel64/Release/lib/
```
