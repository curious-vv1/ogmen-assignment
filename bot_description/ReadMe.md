# bot_description

This package is designed to simulate and visualize a robot in ROS 2. It includes configurations for RViz and Gazebo, allowing you to interact with the robot in a simulation environment. Below are the steps to set up and use this package.

## Prerequisites

Ensure that you have the following installed:
- ROS 2 (Humble or compatible distribution)
- RViz 2
- Gazebo
- `teleop_twist_keyboard` package for keyboard-based control

## Setup Instructions

1. **Source your ROS 2 workspace**
   ```bash
   source /opt/ros/<your_ros2_distro>/setup.bash
   ```
   Replace `<your_ros2_distro>` with your ROS 2 distribution (e.g., `humble`, `foxy`).

2. **Clone this package into your workspace**
   Navigate to your ROS 2 workspace `src` directory and clone this repository:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

3. **Build the workspace**
   Navigate to the root of your workspace and build it:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. **Source the workspace**
   After building, source the workspace to make the package available:
   ```bash
   source install/setup.bash
   ```

## Usage Instructions

### Visualize in RViz
To visualize the robot model in RViz:
1. Launch the `display.launch.py` file:
   ```bash
   ros2 launch bot_description display.launch.py
   ```

### Visualize in Gazebo
To simulate the robot in Gazebo:
1. Launch the `spawn.launch.py` file:
   ```bash
   ros2 launch bot_description spawn.launch.py
   ```

### Control the Robot
To control the robot using the keyboard in the Gazebo simulation:
1. Install the `teleop_twist_keyboard` package if not already installed:
   ```bash
   sudo apt install ros-<your_ros2_distro>-teleop-twist-keyboard
   ```
2. Run the teleoperation node:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

Use the keyboard commands to move the robot around in the simulation environment.

## File Structure
- **`launch/`**: Contains launch files for RViz (`display.launch.py`) and Gazebo (`spawn.launch.py`).
- **`urdf/`**: Stores the robot's URDF description.
- **`rviz/`**: Configuration for RViz visualization.
- **`package.xml`**: ROS 2 package manifest.
- **`CMakeLists.txt`**: Build instructions for the package.

## License
This package is licensed under Apache-2.0.


