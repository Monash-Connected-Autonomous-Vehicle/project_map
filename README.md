<!-- Add an appropriate emoji infornt of the project name, project name numt be Heading 1 (#) -->
# 🗺️ Project Mapping and Localisation

<!-- A very short blurb about the project -->
> The localisation part of this project takes LiDAR point clouds and IMU data and publishes a pose based off an Iterative Closest Point (ICP) 
algorithm. The mapping portion of this project takes the same inputs as localisation but creates a stitched together PointCloud map.

<!-- badges, must include contributors. Tests, and others can be added if you want. -->
[![All Contributors](https://img.shields.io/badge/all_contributors-1-orange.svg?style=flat-square)](#contributors)

<!-- A screenshot or Gif of the working project -->
![](images/ex.png)

<!-- All requirements of the project should be links to where we can install them -->
<!-- pip dependencies must be added into a requirements.txt file -->
## Requirements
- [Ubuntu 20.04](https://ubuntu.com/download/desktop)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
- ```pip install -r requirements.txt``` 

### Other Requirements
- ros_pcl
- pcl_conversions

<!-- Very simple installation procedure. If you can't install within 5 single line bullet points, you need to refactor your code to be  \
able to do so -->
## Installation
- Go to the `src` directory: `cd ~/colcon_ws/src`
- Clone the source code: `git clone `
- Go to the root of the workspace: `cd ~/colcon_ws`
- Install ROS dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Build: `colcon build --symlink-install`

<!-- Directory structure gives a brief on what folders contain what. -->
This should result in a directory structure similar to the following:
```
ws/                                                     
├── build                                                                                                               
└── src
    └── project_map
        ├── icp_cpp_localiser           # ICP localiser functionality
        ├── icp_visualisation           # Visualisation of ICP published pose over time
        ├── perception_pcl              # Dependency with ros_pcl and pcl_conversions
        └── README.md                   # Read me file
```

## Usage
<!-- 
 Usage instructions must be concise. Any export statements must be added to .bashrc (add steps in either requirements of installation).

It should follow the structure mentioned below:
Terminal # (What are we doing):
- `shell code`
-->

### CARLA example 
Terminal 1 (Launch CARLA server):
- `/opt/carla-simulator/CarlaUE4.sh`

Terminal 2 (Launch ROS Bridge):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py`

Terminal 3 (Launch Carla Spawn Vehicles):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py`

Terminal 4 (Launch Carla Manual Control):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch carla_manual_control carla_manual_control.launch.py`

Terminal 5 (Launch ICP CPP Matching):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch icp_cpp_localiser icp_localiser.launch.py`

Terminal 6 (Launch ICP Matching Visualisation):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 run icp_visualisation icp_local`


### StreetDrone example
...

## Tests

Located under `tests`. Enter `pytest` in the terminal to run all tests.

## ROS Parameters and Topics
Please see the [`ROSINFO.md`](https://github.com/Monash-Connected-Autonomous-Vehicle/mcav-GitHub-documentation-standard/blob/main/ROSINFO.md) file for more info.



## Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore -->
<table>
    <tr>
        <td align="center"><a href="https://github.com/ldalwood"><img src="https://avatars.githubusercontent.com/u/69286161?v=4" width="100px;" alt="Liam Dalwood"/><br /><sub><b>Liam Dalwood</b></sub></a><br /><a title="Code">💻</a></td>
        <td align="center"><a href="https://github.com/lakshjaisinghani"><img src="https://avatars3.githubusercontent.com/u/45281017?v=4" width="100px;" alt="Laksh Jaisinghani"/><br /><sub><b>Laksh Jaisinghani</b></sub></a><br /><a title="Mentoring">🧑‍🏫 </a></td>
        <td align="center"><a href="https://github.com/owenbrooks"><img src="https://avatars.githubusercontent.com/u/7232997?v=4" width="100px;" alt="Owen Brooks"/><br /><sub><b>Owen Brooks</b></sub></a><br /><a title="Review">👀 </a></td>
    </tr>
</table>

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
# Creating a map

## Recording Data
Connect the LiDAR sensor to the main computer. 

Set up the CAN bus connection: 
`SD-VehicleInterface/can_setup.sh`

```
docker/run.sh
src
ros2 launch data_recording record.launch.xml rec_lidar:=true lidar_driver:=true launch_vi:=true
```
When done, press Ctrl-c. This should create a rosbag folder with a name something like `rosbag2_2023_03_08-06_07_25_0`.

## Mapping
https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/creating-maps-for-autoware/

Get the autoware prebuilt docker: https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation-prebuilt/

```
cd /autoware

source install/setup.bash

ros2 launch tier4_localization_launch localization.launch.xml
```
But that needs more parameters
