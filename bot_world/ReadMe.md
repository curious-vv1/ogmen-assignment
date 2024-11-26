# Bot World ROS Package

This package provides a simulation environment for a bot. Follow the steps below to integrate it into your ROS2 workspace and launch the simulation.

## Prerequisites

1. ROS2 installed (ensure compatibility with the package version).
2. The `bot_description` package must be available and functional in your workspace.

## Installation

1. Clone or copy the `bot_world` package into your ROS2 workspace's `src` directory:
   ```bash
   cd ~/ros2_ws/src
   cp -r /path/to/bot_world .
   ```

2. Navigate to your workspace and build the package:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Source the setup script to include the package in your environment:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Launching the Simulation

1. Ensure that the `bot_description` package is correctly set up and sourced:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Launch the `bot_world` simulation:
   ```bash
   ros2 launch bot_world bot_world.launch.py
   ```

## Package Structure

- **`launch/bot_world.launch.py`**: The primary launch file for the simulation.
- **`src/`**: Source code for the package.
- **`include/`**: Header files.
- **`world/`**: Simulation world files.
- **`CMakeLists.txt`**: Build configuration file.
- **`package.xml`**: Package manifest.

## License

This package is licensed under the terms specified in the `LICENSE` file.

## Troubleshooting

- Ensure all dependencies are sourced before launching the simulation.
- Check for any missing dependencies in the `package.xml` file and install them if needed.
- Verify that the `bot_description` package is correctly installed and functional.

For additional support, please refer to the documentation or contact me.
