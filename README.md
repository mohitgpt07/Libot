# LiBot - Autonomous Mobile Robot

LiBot is an autonomous mobile robot designed for real-time mapping, localization, and dynamic navigation in unknown environments. Using Raspberry Pi for high-level control, Arduino for motor interfacing, and RPLidar for environmental scanning, LiBot is capable of avoiding obstacles, creating maps, and navigating in 2D plane.

Check out the ***photos and videos*** of LiBot running in real-world environments in the [media](https://github.com/mohitgpt07/libot/tree/main/media) folder for a demonstration of its capabilities.

## Robot Overview
### Key Components
**Raspberry Pi 4:** Serves as the central processing unit for LiBot, running the ROS 2 framework to manage the robot’s functionalities. Its key responsibilities include: Sensor Data Processing, Path Planning, Communication, Control and Coordination.    
**Arduino Uno:** Handles motor control, receiving commands from the Raspberry Pi and controlling motor drivers.  
**RPLidar A1M8:** Provides 2D laser scans for mapping and obstacle detection.  
**L298N Motor Drivers:** Controls the robot's differential drive motors, receiving PWM signals from the Arduino.  

### Software Components
[diffdrive_arduino](https://github.com/mohitgpt07/diffdrive_arduino): Implements a ros2_control hardware interface, allowing ROS 2 to communicate with the Arduino for controlling LiBot's motors. It uses serial communication and works with the diff_drive_controller for velocity feedback and control.

[ros_arduino_bridge](https://github.com/mohitgpt07/ros_arduino_bridge): Provides a simple serial communication bridge between the Raspberry Pi (running ROS 2) and the Arduino. It sends velocity commands from ROS 2 to the Arduino, which generates the necessary PWM signals for the motor driver.

[serial](https://github.com/mohitgpt07/serial): A cross-platform library used to establish reliable and efficient serial communication between the Raspberry Pi and Arduino, managing data flow for real-time motor control and feedback.

## Prerequisites
Ensure the following are installed on the Raspberry Pi for LiBot’s functionality:  

*ROS 2 Humble*  
*colcon*  
*libserial*  
*rplidar_ros*  
*ros2 xacro package*  
*ros2_control*  
*SLAM Toolbox*  
*Nav2 stack*

## Setting up the Workspace
**Create the workspace:**
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
```

**Clone the repositories:**
```
git clone <libot-repo-link>
git clone <diffdrive_arduino-repo-link>
git clone <ros_arduino_bridge-repo-link>
git clone <serial-repo-link>
```

## Building the Workspace
**1. Source ROS 2 environment:**
```
source /opt/ros/<your-ros2-distro>/setup.bash
```

**2. Build the workspace:**
```
cd ~/robot_ws
colcon build
```

**3. Source the workspace after the build completes:**
```
source install/setup.bash
```

## Flashing the Arduino
1. Connect the Arduino Uno to your system.
2. Open the Arduino IDE and upload the code from ros_arduino_bridge to enable motor control via ROS.
3. Ensure the correct board and port are selected before uploading.

## Running the Robot
### Simulation in Gazebo
For testing LiBot in a simulated environment, use Gazebo along with SLAM and navigation.
Run each component in separate terminals to simulate LiBot’s capabilities:

***Terminal 1: Launch Gazebo simulation:***
```
ros2 launch libot launch_sim.launch.py
```

***Terminal 2: Launch SLAM:***
```
ros2 launch libot online_async_launch.py params_file:=./robot_ws/src/libot/config/mapper_params_online_async.yaml use_sim_time:=true
```

***Terminal 3: Launch Navigation:***
```
ros2 launch libot navigation_launch.py use_sim_time:=true
```

***Terminal 4: Use RViz2 for visualization and setting navigation goals:***
```
rviz2 -d libot_ws/src/libot/RViz/libot_rviz_config.rviz
```
This will allow you to visualize the map being created in real-time and set navigation goals to see LiBot navigating in the simulated environment.

### Physical Robot
For running the physical robot, you will need to launch the RPLidar, SLAM, and navigation separately in different terminals as well.

***Terminal 1: Launch Real Robot:***
```
ros2 launch libot launch_sim.launch.py
```

***Terminal 2: Launch RPLidar:***
```
ros2 launch libot rplidar.launch.py
```

***Terminal 3: Launch SLAM:***
```
ros2 launch libot online_async_launch.py params_file:=./robot_ws/src/libot/config/mapper_params_online_async.yaml use_sim_time:=false
```

***Terminal 4: Launch Navigation:***
```
ros2 launch libot navigation_launch.py use_sim_time:=false
```

***Terminal 5: Use RViz2 for visualization and setting navigation goals:***
```
rviz2 -d libot_ws/src/libot/RViz/libot_rviz_config.rviz
```
With RViz2, you can observe LiBot's live map, visualize the robot's surroundings, and manually set navigation goals for demonstration.

## Directory Structure
```
robot_ws/
│
├── src/
│   ├── libot/                       # Main package for LiBot, containing configuration, URDF, and launch files
│   │   ├── config/                  # Configuration files for sensors, controllers, SLAM, nav2 etc.
│   │   ├── description/             # URDF and Xacro files for LiBot's robot model
│   │   ├── launch/                  # Launch files for both simulation and physical robot runs
│   │   ├── rviz/                    # RViz2 configuration for visualizing LiBot
│   ├── diffdrive_arduino/           # ROS2_control hardware interface for motor control using Arduino
│   ├── ros_arduino_bridge/          # Serial communication bridge between Raspberry Pi and Arduino, generates PWM signals
│   └── serial/                      # Cross-platform C++ library for handling RS-232-like serial communication
│
├── install/                         # Installation files after building the workspace
├── log/                             # Log files generated by ROS 2 during runtime
└── build/                           # Build files generated by colcon
```

## Issues and Contribution
Feel free to open an issue or contribute by submitting pull requests. Reach out if you need further guidance or clarification.

