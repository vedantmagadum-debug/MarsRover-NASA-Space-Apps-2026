# mars_rover: folders, nodes and commands

## 1. What every folder / file does
```
mars_rover_ws/                      <- the ROS 2 workspace. Run colcon here.
  src/                              <- your source code (only this is edited by you)
    mars_rover/                     <- the ROS 2 package
      package.xml                   package name + dependencies (ROS reads this)
      setup.py / setup.cfg          how the package is installed; lists the runnable nodes (entry_points)
      resource/mars_rover           empty marker file so ROS can find the package
      README.md, COMMANDS.md        documentation
      personal_dashboard.html       stand-alone web page for your own site (shows all data + controls)
      urdf/mars_rover.urdf          ROBOT BODY: links, wheels, joints, lidar/IMU/camera + Gazebo plugins
      worlds/mars_world.sdf         SIMULATED WORLD: Mars gravity, ground, rocks, craters, dunes, canyon walls
      launch/                       start-up scripts (see section 2)
      config/ekf.yaml               sensor-fusion settings (wheel odometry + IMU gyro)
      config/slam_toolbox.yaml      lidar mapping settings
      rviz/mars_rover.rviz          RViz window layout (map, scan, camera, robot)
      mars_rover/                   PYTHON NODES (the programs)
        obstacle_avoider.py         autonomous driving brain (lidar + IMU + camera + odometry)
        cmd_vel_mux.py              chooses who drives: auto / manual / stop
        data_server.py              web dashboard + JSON API + CSV logging + web drive buttons
        set_mode.py                 tiny command to switch the drive mode
        generate_world.py           makes a new random Mars world file
  build/  install/  log/            created by colcon build. Never edit. Safe to delete and rebuild.
```

## 2. Launch files (one per terminal, or everything with one command)
| Command | What it starts |
|---|---|
| `ros2 launch mars_rover sim.launch.py` | Gazebo + Mars world + rover spawn + ROS bridge + robot_state_publisher |
| `ros2 launch mars_rover fusion_slam.launch.py` | EKF sensor fusion + slam_toolbox (builds /map) |
| `ros2 launch mars_rover control.launch.py` | cmd_vel_mux + obstacle_avoider + data_server (all start instantly) |
| `ros2 launch mars_rover rviz.launch.py` | RViz |
| `ros2 launch mars_rover mars_rover.launch.py` | all four above at once |

Options: `mode:=auto` (default) / `mode:=manual` / `mode:=stop`, `rviz:=false`, `data_server:=false`, `port:=8080`,
`open_browser:=false`, `host:=0.0.0.0`, `allow_control:=false`.

## 3. Single nodes (run by hand)
| Command | What it does |
|---|---|
| `ros2 run mars_rover obstacle_avoider` | autonomous driving; publishes /cmd_vel_auto and /obstacle_status |
| `ros2 run mars_rover cmd_vel_mux` | picks /cmd_vel_auto or /cmd_vel_manual and sends it to /cmd_vel |
| `ros2 run mars_rover data_server` | web dashboard at http://localhost:8080 |
| `ros2 run mars_rover set_mode manual` | switch mode: auto, manual or stop |
| `ros2 run mars_rover generate_world --out src/mars_rover/worlds/mars_world.sdf --seed 11 --rocks 40` | new random terrain (rebuild or use --symlink-install) |

## 4. Driving
| Want | Command |
|---|---|
| Autonomous | `ros2 run mars_rover set_mode auto` |
| Manual (keyboard) | `ros2 run mars_rover set_mode manual` then in another terminal `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_manual` |
| Manual (browser) | open http://localhost:8080 , press MANUAL, hold the arrow buttons |
| Emergency stop | `ros2 run mars_rover set_mode stop` |
| Send one manual command | `ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"` |
| Change autonomous goal distance | `ros2 param set /obstacle_avoider goal_x 60.0` |
| Change cruise speed | `ros2 param set /obstacle_avoider cruise_speed 0.25` |
| Which mode now? | `ros2 topic echo /drive_mode_status --once` |

Keyboard keys: i forward, k stop, j/l turn, u/o arcs, , back, q/z faster/slower.

## 5. Sensors: readings and commands
| Sensor / node | Topic | Reading | Command |
|---|---|---|---|
| Lidar | /scan (LaserScan, 10 Hz) | 360 distances, 0.15 to 12 m, angle 0 = forward | `ros2 topic echo /scan --once` |
| | | scan frame | `ros2 topic echo /scan --once --field header.frame_id` |
| | | rate | `ros2 topic hz /scan` |
| IMU | /imu/data (Imu, 100 Hz) | orientation quaternion | `ros2 topic echo /imu/data --field orientation` |
| | | gyro (rad/s) | `ros2 topic echo /imu/data --field angular_velocity` |
| | | accelerometer (m/s2, z about 3.7 on Mars) | `ros2 topic echo /imu/data --field linear_acceleration` |
| Camera | /camera/image_raw (Image, 15 Hz) | 640x480 RGB | `ros2 run rqt_image_view rqt_image_view` |
| | | size / rate | `ros2 topic echo /camera/image_raw --once --field width` ; `ros2 topic hz /camera/image_raw` |
| Wheel odometry | /odom (Odometry, 50 Hz) | x, y, speed | `ros2 topic echo /odom --field pose.pose.position` ; `ros2 topic echo /odom --field twist.twist` |
| EKF fusion | /odometry/filtered | fused x, y, yaw, speed | `ros2 topic echo /odometry/filtered --field pose.pose` |
| Wheels | /joint_states | wheel angles/speeds | `ros2 topic echo /joint_states --once` |
| SLAM | /map (OccupancyGrid) | -1 unknown, 0 free, 100 occupied | `ros2 topic echo /map --once --field info` |
| Transforms | /tf, /tf_static | map->odom->base_link->sensors | `ros2 run tf2_ros tf2_echo map base_link` ; `ros2 run tf2_tools view_frames` |
| Obstacle avoider | /obstacle_status (JSON text, 2 Hz) | state, front/left/right distance, camera rock fraction, tilt | `ros2 topic echo /obstacle_status` |
| Drive command | /cmd_vel, /cmd_vel_auto, /cmd_vel_manual | v and w | `ros2 topic echo /cmd_vel` |

Avoider states: WAITING (no sensor data yet), CRUISE, CAUTION (camera+lidar see rock), AVOID (turning), TILT_STOP, STALE_LIDAR, GOAL_REACHED.

Gazebo side (before the bridge): `ign topic -l` lists topics, `ign topic -e -t /scan` prints lidar, `ign topic -e -t /imu/data` prints IMU.

## 6. Data server and web control panel (http://localhost:8080)
The browser opens by itself when control.launch.py starts (turn off with `open_browser:=false`).
If it does not, open http://localhost:8080 yourself. The terminal prints: DATA SERVER RUNNING -> open ...

Control from the browser (no terminal needed):
| Input | Action |
|---|---|
| AUTO / MANUAL / STOP buttons | switch mode (keys 1 / 2 / Space) |
| W A S D or arrow keys (hold) | drive; releasing the key stops the rover |
| on-screen arrow pad (hold) | drive, including diagonals |
| Speed and Turn sliders | set manual speed (0.1 to 0.6 m/s) and turn rate |
| Space or red STOP button | stop immediately |
Pressing a drive key or button in AUTO switches to MANUAL for you.
The page also shows camera, SLAM map, a live lidar radar, avoider state, IMU, odometry and topic rates.

| URL | Gives |
|---|---|
| / | live dashboard: camera, map, all readings, AUTO/MANUAL/STOP and drive buttons |
| /api/all | everything in one JSON |
| /api/scan , /api/scan?full=1 | closest obstacle, front/left/right/rear distance, optional all 360 ranges |
| /api/imu | roll, pitch, yaw (deg), gyro, accelerometer |
| /api/odom , /api/fused_odom | raw and fused position, speed |
| /api/camera , /api/map | metadata (map: size, explored area m2) |
| /api/obstacle , /api/cmd_vel , /api/drive_mode | avoider status, last command, mode |
| /api/rates | measured Hz and age of every topic (health check) |
| /camera.png , /map.png | pictures |
| /api/mode?set=manual | switch mode |
| /api/drive?lin=0.3&ang=0 | manual command (manual mode only) |

Examples: `curl -s localhost:8080/api/scan`, `curl -s localhost:8080/api/imu`, `curl -s localhost:8080/api/rates`.
Logs: a CSV is written to ~/mars_rover_logs/ twice per second (position, speed, tilt, distances, mode, explored area).
Allow other computers: `ros2 run mars_rover data_server --ros-args -p host:=0.0.0.0` (only on a trusted network, the page can drive the rover).


## 6b. Show the data on your own web page
The data server already allows other web pages to read it (CORS is on). Three ways to use it:

1. **Ready-made page:** `personal_dashboard.html` (in this folder). Double-click it to open in a browser while the
   rover runs. It has a "server" box at the top (default http://localhost:8080) and shows everything: camera, map,
   lidar radar, IMU, odometry, avoider state, rates, plus the drive/mode controls. Put the same file on your own
   website and it works there too. You can also pass the server in the link: `personal_dashboard.html?server=http://192.168.1.50:8080`
2. **Embed in your page:** `<iframe src="http://localhost:8080" width="100%" height="900" style="border:0"></iframe>`
3. **Your own code:** read the JSON yourself, for example
   `fetch('http://localhost:8080/api/all').then(r => r.json()).then(d => console.log(d.imu, d.fused_odom, d.obstacle))`

Other devices (phone, another laptop on the same Wi-Fi):
    hostname -I                                                   # find this PC's IP, e.g. 192.168.1.50
    ros2 launch mars_rover control.launch.py host:=0.0.0.0        # (or add host:=0.0.0.0 to mars_rover.launch.py)
then open http://192.168.1.50:8080 (or enter that address in the server box of personal_dashboard.html).

Read-only for a public page: add `allow_control:=false`. The data stays visible but the drive/mode buttons are refused
(HTTP 403). Do NOT expose port 8080 to the internet with control enabled, anyone could drive the rover.
A page served over https may block http addresses other than localhost (mixed content); use a tunnel that gives
you an https address (for example cloudflared or ngrok) and always with `allow_control:=false`.

## 7. Health checks
```
ros2 node list                  # expect: cmd_vel_mux obstacle_avoider data_server ekf_filter_node slam_toolbox ...
ros2 topic list
ros2 node info /obstacle_avoider
ros2 param list /obstacle_avoider
ros2 run rqt_graph rqt_graph    # picture of who talks to whom
ros2 bag record /scan /imu/data /odom /odometry/filtered /cmd_vel -o rover_run   # record a run
ros2 run nav2_map_server map_saver_cli -f ~/mars_map                              # save the map
```
