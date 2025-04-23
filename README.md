# RobotOne Joystick Package

- [RobotOne Joystick Package](#robotone-joystick-package)
  - [Overview](#overview)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Configuration](#configuration)
  - [Usage](#usage)
  - [Directory Structure](#directory-structure)
  - [Contributing](#contributing)
  - [License](#license)
  - [Contact](#contact)


## Overview
The `robotone_joystick` package provides joystick control for the RobotOne system. It enables users to interact with the RobotOne platform using an Xbox controller for operation and testing. This package runs only on Linux system.

Key features include:
- Crete Joystick node
- Publish Joystick message (sensor_msgs/msg/joy)
- Configurable joystick interface

## Prerequisites
Ensure the following dependencies are installed:
- ROS2 (Humble or above)
- Python 3


## Installation

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/fra1alv/robotone_joystick.git
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
The `robotone_joystick` package uses a configuration file to setup an interface with a joystick and process its input, then publishes it joy meassage. The default configuration file is located at `config/xbox_config.yaml`.

You can customize the parameters by editing this file:
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
   ros2 launch robotone_joystick joystick_control_launch.py
   ```
   Note: xBox controller is default in future we might have different controllers.
2. Verify the node list
   ```bash
   ros2 node list
   ```
   Node: you must to find the node robotone_joystick_node
3. List topics published by a node:
    ```bash
    ros2 node info /robotone_joystick_node
4. Listen to the topic
   ```bash
   ros2 topic echo /robotone_joystick
   ```
   Note: Topic name can be set on the config file

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
├── package.xml                         # ROS2 package manifest
└── README.md                           
```
## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a detailed description of your changes.

## License
This project is licensed under the [GPLv3 License](../../LICENSE).

## Contact
For support or questions, feel free to reach out:
- Email: alonsofra1alv@gmail.com.com
- GitHub: [Fra1alv](https://github.com/fra1alv)
