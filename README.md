# mavros_control
A ROS2 package for controlling ArduPilot-based robots.

This package includes the ```waypoint_path_follower``` node, which has methods to arm/disarm, take off/land, set the home location, and visit a given set of waypoints.

## Package setup
  ```
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://github.com/itskalvik/mavros_control.git
  cd mavros_control
  python3 -m pip install -r requirements.txt
  cd ~/ros2_ws
  rosdep install --from-paths src --ignore-src -y
  colcon build --symlink-install
  source ~/ros2_ws/install/setup.bash
  ```

## Demo
The package includes a demo launch file that starts the ```waypoint_path_follower``` node. The node arms the vehicle, visits two waypoints at 5-meter offsets from the starting location, and then disarms:
```
ros2 launch mavros_control demo.launch.py
```