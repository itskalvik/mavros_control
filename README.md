# mavros_control
A ROS2 package for controlling ArduPilot-based robots.

This package includes the ```controller``` node, which has methods to arm/disarm, take off/land, set the home location, visit a given set of waypoints if a GPS is available, and RC-control to move the vehicle.

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
The package includes a demo launch file that starts the ```controller``` node. The node arms the vehicle, moves it, and then disarms it:
```
ros2 launch mavros_control demo.launch.py
```

### Parameters
The launch file has the following paramerets:
- ```xy_tolerance (default: 0.7)```: 

  XY-axis distance (meters) tolerance used to determine if a waypoint is reached
- ```z_tolerance (default: 0.3)```: 

  Z-axis distance (meters) tolerance used to determine if a waypoint is reached, used only when ```use_altitude``` is ```True``` 
- ```use_altitude (default: False)```:

  If ```True``` 3D waypoints are used for the path
- ```navigation_type (default: 0)```: 

  If ```0```: Use global position based navigation (assumes access to GPS)
  
  If ```1```: Use raw rc controls for navigation