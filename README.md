# pplanner
This is a portable planner for ground robots.

The latest updated date is October 30th, 2024.

Notably, the codes of this work is cumbersome and no longer maintained as our attention is paied to improve the efficiency by other methods, which would be coming soon.
## Dependencies
These instructions assume that ROS desktop-full of the appropriate ROS distro is installed.
<!-- [UFEP](https://github.com/YinghaoJia98/UFEP-Released.git) -->
Install necessary libraries:

- [grid_map](https://github.com/ANYbotics/grid_map) (grid map library for mobile robots)
```bash
sudo apt-get install ros-noetic-grid-map-core ros-noetic-grid-map-msgs
```
- [elevation_mapping_cupy](https://github.com/YinghaoJia98/elevation_mapping_cupy.git) The test_carved_stair and ReviseInverseDensity branches could be used in my experiment and the original [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy.git) also can be used.
- [pplanner_msgs](https://github.com/YinghaoJia98/pplanner_msgs.git) The msg and srv files which would be in cluded into this pack in the future.
- [pplanner_ui](https://github.com/YinghaoJia98/pplanner_ui.git) The ui files which would be in cluded into this pack in the future.

## Build
```bash
mkdir -p pplanner_ws/src/
git clone git@github.com:YinghaoJia98/pplanner.git
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Run

### Simulation
Intstall the simulator
- [pplanner_simulator](https://github.com/YinghaoJia98/pplanner_simulator.git), UnitreeA1MPCImproveTest might work with changing the parameters of sensors.


Run the demo with wheeled robot [SuperMegaBot](https://github.com/ntnu-arl/smb_simulator.git).
```bash
cd pplanner_ws
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch pplanner_simulator PplannerSmbCeshi2ForPplanner.launch
roslaunch pplanner pplanner_simulator.launch
```

Run the demo with legged robot UnitreeA1, which is controlled by [MPC](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller.git) method.
```bash
roslaunch pplanner_simulator PplannerUnitreeA1.launch
roslaunch pplanner pplanner_UnitreeA1Simulator_nogdb.launch
```

### Parameters
The config file is [pplanner_settings_simulator.yaml](/config/pplanner_settings_simulator.yaml). 

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


## Experiments Videos
[![pplanner_video](img/front.jpg)](https://youtu.be/Ml_Qq0PLGvM)

## Reference
If you utilize this codebase in your research, we kindly request you to reference our work. You can cite us as follows:

 - Y. Jia, W. Tang, H. Sun, J. Yang, B. Liu and C. Wang, "Portable Planner for Enhancing Ground Robots Exploration Performance in Unstructured Environments," in IEEE Robotics and Automation Letters, vol. 9, no. 11, pp. 9295-9302, Nov. 2024, doi: 10.1109/LRA.2024.3440094.

## Thanks
The robots used in simulator is reorganized from [GBP](https://github.com/ntnu-arl/gbplanner_ros.git).
Benifiting from the robotics community and we choose to make our work public.

## Contact us
* [Yinghao Jia](mailto:yinghaojia@163.com)
* Wei Tang
