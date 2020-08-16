# ROS Wrapper for GPD

* [Author's website](http://www.ccs.neu.edu/home/atp/)
* [License](https://github.com/atenpas/gpd_ros/blob/master/LICENSE.md)
* [GPD library](https://github.com/atenpas/gpd)

## Overview

A ROS wrapper around the [GPD](https://github.com/atenpas/gpd) package for detecting 6-DOF grasp poses for a
2-finger robot hand (e.g., a parallel jaw gripper) in 3D point clouds.

## 1) Installation

The following instructions have been tested on **Ubuntu 16.04**. Similar
instructions should work for other Linux distributions.

1. Install GPD. You can follow [these instructions](https://github.com/atenpas/gpd#install). Make sure to run `make install` to install GPD as a library.

2. Clone this repository into the `src` folder of your catkin workspace:

   ```
   cd <location_of_your_workspace>/src
   git clone https://github.com/atenpas/gpd_ros
   ```

3. Build your catkin workspace:

   ```
   cd <location_of_your_workspace>
   catkin_make
   ```

## 2) Generate Grasps for a Point Cloud on a ROS Topic

First, you need to modify the config file in your `gpd` folder, e.g., 
`<path_to_gpd>/cfg/ros_eigen_params.cfg`. Search for parameters that have 
absolute file paths and change them to actual paths on your system.

Next, you need to modify the path in the ROS launch file that points to the 
config file that you changed in the previous step, e.g., 
[this line](https://github.com/atenpas/gpd_ros/blob/master/launch/ur5.launch#L18).

Now, you can run GPD as a ROS node. The following command will launch a ROS node
that waits for point clouds on the ROS topic `/cloud_stitched`. Once a point
cloud is received, the node will search the cloud for grasps.

```
roslaunch gpd_ros ur5.launch
```

## 3) Using Advanced Messages

If you want to speed up GPD or look for grasps on a specific object, you should 
use one of these messages: [CloudSamples](https://github.com/atenpas/gpd_ros/blob/master/msg/CloudSamples.msg), [CloudIndexed](https://github.com/atenpas/gpd_ros/blob/master/msg/CloudIndexed.msg). Both of these messages build up on the [CloudSources](https://github.com/atenpas/gpd_ros/blob/master/msg/CloudSources.msg) message that can be used to represent a point cloud whose points were seen by multiple cameras or from multiple viewpoints.

As a typical use case for the `CloudSamples` message, consider a table with a single object on top of it, observed by one camera. The complete point cloud should be put in the message so that GPD can check grasp poses against collisions with the table. Samples in the message should correspond to points on the object so that GPD can search for grasps on the object (and avoids searching for grasps on the table).

## 4) Troubleshooting

If `catkin_make` cannot find `libinference_engine.so`, required by OpenVino, make 
sure that `LD_LIBRARY_PATH` contains the path to that library:

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:path_to_dldt/inference-engine/bin/intel64/Release/lib/
```
