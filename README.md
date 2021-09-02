# Simple RQT plugin for custom extruder

### Dependencies
- ROS2 Foxy: https://docs.ros.org/en/foxy/Installation.html
- am_extruder_tool (connects with this): https://github.com/mltmyr/am_extruder_tool

### Build
`cd [your_ros2_workspace]`

`source /opt/ros/foxy/setup.bash`

`git clone git@github.com:mltmyr/am_extruder_simple_ui.git`

`colcon build`

`source install/setup.bash`

### Run GUI
Three ways to use/run:
1. `rqt`. Under Plugins, choose am_extruder/Simple extruder GUI.
2. `rqt --standalone am_extruder_simple_ui`
3. `ros2 run am_extruder_simple_ui am_extruder_simple_ui`

Then, enter target values and observe the current values returned from the extruder.
If am_extruder_tool is running simultaneously, then the gui can be used to set target values for the nozzle heat control and filament speed control.

### Details
Nozzle temperature target must be between 0 and 215 celsius and is sent over topic `/filament_heater_controller/commands`. Message type: `std_msgs/msg/Float64MultiArray`.
Use `ros2 topic echo /filament_heater_controller/commands` to inspect what is sent.

Filament speed target must be between -100 and 100 mm/s and is sent over topic `/filament_mover_controller/commands`. Message type: `std_msgs/msg/Float64MultiArray`.
Use `ros2 topic echo /filament_mover_controller/commands` to inspect what is sent.

Current nozzle temperature and filament speed is received over topic `/joint_states`. Message type: `sensor_msgs/msg/JointState`.
To send a test message on the `/joint_states` topic, use: `ros2 topic pub --once /joint_states sensor_msgs/msg/JointState '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'JointState'}, name: {"filament_mover", "filament_heater"}, position: {25.0, 0.0}, velocity: {0.0, 5.45}, effort: {0.0, 0.0}}'`.
