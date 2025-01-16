# RobotOne Joystick Package

## Overview
The `robotone_joystick` package provides joystick control for the RobotOne system. It enables users to interact with the RobotOne platform using an Xbox controller for intuitive operation and testing.

Key features include:
- Seamless integration with ROS2 nodes
- Configurable joystick mapping for flexible control

## Prerequisites
Ensure the following dependencies are installed:
- ROS2 (e.g., Humble, Foxy, or Rolling)
- `joy` and `teleop_twist_joy` ROS2 packages
- SDL2 library (for Linux systems)

## Installation

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/robotone_joystick.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. Verify the installation by listing available ROS2 packages:
   ```bash
   ros2 pkg list | grep robotone_joystick
   ```

## Configuration
The `robotone_joystick` package uses a configuration file to map joystick inputs to specific actions. The default configuration file is located at `config/xbox_config.yaml`.

You can customize the mappings by editing this file:
```yaml
robotone_joystick_node:
  ros__parameters:
    topic_name: /robotone_joystick
    dev: /dev/input/js0
    debug: true
    autorepeat_rate: 0   #Hz
    coalesce_interval: 1 #ms
    deadzone: 0.05

```

## Usage

1. Launch the joystick node:
   ```bash
   ros2 launch robotone_joystick joystick_control.launch.py
   ```

2. Connect your Xbox controller and ensure it is recognized by the system. For Linux, verify using:
   ```bash
   jstest /dev/input/js0
   ```

3. Control the RobotOne system using the joystick. By default:
   - Left joystick controls linear motion.
   - Right joystick controls angular motion.

## Directory Structure
```plaintext
robotone_joystick/
├── config/                             # Configuration files
│   └── xbox_config.yaml
├── include/                            # header files
│   └── robotone_joystick/
│       └── robotone_joystick.hpp/
├── launch/                             # Launch files
│   └── robotone_joystick_launch.py
├── src/                                #Source code
│   └── robotone_joystick_node.hpp
│       └── robotone_joystick,cpp           
├── CMakeLists.txt  
├── package.xml                         # Project documentation
└── README.md                           # ROS2 package manifest
```

## Testing
To test the joystick functionality:
1. Run the joystick node in simulation mode:
   ```bash
   ros2 run robotone_joystick test_joystick
   ```

2. Use the controller to verify that inputs are correctly mapped to the expected outputs.

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a detailed description of your changes.

## License
This project is licensed under the [GNU GPL v3 License](../../LICENSE).

## Contact
For support or questions, feel free to reach out:
- Email: alonsofra1alv@gmail.com.com
- GitHub: [Fra1alv](https://github.com/fra1alv)
