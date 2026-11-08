Mars Rover Simulation

A six-wheel rocker-bogie Mars rover simulated in Gazebo Fortress with ROS 2 Humble. The rover drives autonomously across a Jezero-crater style Mars world (3.721 m/s² gravity, boulders, ridges, slabs and walls). It maps with lidar SLAM, fuses wheel odometry with an IMU, avoids obstacles using lidar and camera, and streams everything to a live web dashboard.
Features

    Rover model (URDF): six driven wheels with grouser cleats, rocker-bogie suspension, a solar panel, a camera mast, a high-gain dish, a whip antenna and an RTG-style power unit
    Suspension: rocker and bogie joints are held level by PD position controllers, giving a compliant rocker-bogie
    Mars world (SDF): dunes, scattered rocks, boulders, slabs, a ridge, a wall and a small crater rim
    Sensors: 360° lidar, IMU and 640x480 camera
    Sensor fusion: robot_localization EKF combining wheel odometry (forward speed) with IMU gyro (yaw rate)
    Mapping: slam_toolbox builds a live /map from the lidar
    Autonomy: an obstacle avoider using lidar, camera, IMU tilt and odometry
    Drive modes: auto, manual or stop, switchable at any time
    Web dashboard: camera, map, lidar radar, IMU, odometry and drive buttons, plus a standalone personal_dashboard.html
    RViz layout: map, scan, camera and robot model

Requirements

    Ubuntu 22.04
    ROS 2 Humble
    Gazebo Fortress (Ignition)
