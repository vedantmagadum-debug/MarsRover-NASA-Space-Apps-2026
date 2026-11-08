# mars_rover (ROS 2 Humble + Gazebo Fortress)

Full folder explanation and every command: see COMMANDS.md

## Install once
    sudo apt install ignition-fortress ros-humble-ros-gz ros-humble-slam-toolbox \
      ros-humble-robot-localization ros-humble-rviz2 ros-humble-robot-state-publisher \
      ros-humble-teleop-twist-keyboard ros-humble-rqt-image-view ros-humble-nav2-map-server
    # if ros-humble-ros-gz is not found use ros-humble-ros-ign

## Build (inside the workspace folder, never in ~)
    cd ~/mars_rover_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## Run, everything at once
    ros2 launch mars_rover mars_rover.launch.py            # autonomous
    ros2 launch mars_rover mars_rover.launch.py mode:=manual

## Run, separate terminals (source install/setup.bash in each)
    1) ros2 launch mars_rover sim.launch.py
    2) ros2 launch mars_rover fusion_slam.launch.py
    3) ros2 launch mars_rover control.launch.py
    4) ros2 launch mars_rover rviz.launch.py
    Dashboard + browser control: http://localhost:8080 (opens automatically; keys W A S D, Space = stop, 1 = auto, 2 = manual)

## Switch mode any time
    ros2 run mars_rover set_mode auto | manual | stop

## Troubleshooting
* No /scan or camera: change <render_engine>ogre2</render_engine> to ogre in worlds/mars_world.sdf.
* Scan frame_id starts with mars_rover/: add a static_transform_publisher from lidar_link to that name.
* Rover will not turn: lower mu2 in the URDF.
* A 2D lidar cannot see rocks lower than about 0.29 m.

## Personal web page
Open personal_dashboard.html in any browser (or host it on your own site). Set the rover server address in the box at
the top. Details and network/security notes: COMMANDS.md, section 6b.
