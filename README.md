# pplanner
This is a portable planner for ground robots.
## Dependencies
These instructions assume that ROS desktop-full of the appropriate ROS distro is installed.

Install necessary libraries:

- [grid_map](https://github.com/ANYbotics/grid_map) (grid map library for mobile robots)
```bash
sudo apt-get install ros-noetic-grid-map-core ros-noetic-grid-map-msgs
```
- [elevation_mapping_cupy](https://github.com/YinghaoJia98/elevation_mapping_cupy.git) The test branch is used in my experiment and the original [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy.git) also can be used.
- [pplanner_msgs](https://github.com/YinghaoJia98/pplanner_msgs.git) The msg and srv files which would be in cluded into this pack in the future.
- [pplanner_ui](https://github.com/YinghaoJia98/pplanner_ui.git) The ui files which would be in cluded into this pack in the future.

## Building
```bash
mkdir -p pplanner_ws/src/
git clone git@github.com:YinghaoJia98/pplanner.git
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Using for Jueying X20

Make sure the time has been synchronized. The way could refer to [blog1](https://blog.csdn.net/weixin_43885544/article/details/117223880) and [blog2](https://blog.csdn.net/weixin_35804181/article/details/125778648). Later I will summarize a brief instruction.

The individual experience is as [blog](https://zhuanlan.zhihu.com/p/612879934).

In pico for perception task, the drivers of lidar and imu and the transform cmd to pico for locomotion task and other nodes should be launched by pplanner [pplanner_jueying1.launch](/launch/pplanner_jueying1.launch) file.
```bash
cd pplanner_ws
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch pplanner pplanner_jueying1.launch
```

In jetson Xavier nx, the elevation_mapping_cupy should be launched
```bash
cd ele_ws
catkin build elevation_mapping_cupy -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch elevation_mapping_cupy ele_jueying.launch
```

Make sure the elevation mapping has been published and launch the pplanner, the logocial requirement can be fixed in future and it is just a little problem.

### Parameters
The config file is [pplanner_settings_ceshi.yaml](/config/pplanner_settings_ceshi.yaml). 

The subscribed topics for mapping:
* grid_map_topic
* elevation_layer
* traversability_layer
* traversability_supplementary_layer
* world_frame
* track_frame

The subscribed topics for tracker:
* World_Frame
* Robot_Frame

The published topics:
* cmd_topic_

## Contact us
* [Yinghao Jia](mailto:yinghaojia@163.com)
* Wei Tang
