# tf2_pkg
ROS2 package for static and dynamic transforms in both rclpy and rclcpp

Available for:
- tbd

## Usage
1. Clone the repository in your workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/alexMarFar/tf2_pkg.git
```
2. Update the topic_name in the launch file.

3. Build the package
```bash
cd ~/ros2_ws/
colcon build --packages-up-to tf2_pkg
source install/setup.bash
```
4. Launch the nodes
```bash
ros2 launch tf2_pkg listener.launch.py
```

