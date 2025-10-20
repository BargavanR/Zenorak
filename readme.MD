ZENORAK_ROS2_CONTROL PACKAGE:

### Package Structure

zenorak_ros2_control/
├── config/
│ ├── controller.yaml # Controller configuration
├── description/
│ └── zenorak.xacro # Robot URDF description
├── launch/
│ ├── launch_robot.launch.py # Main robot launch file
│ └── rsp.launch.py # Robot state publisher launch
├── meshes/
│ ├── collision/ # Collision meshes
│ └── visual/ # Visual meshes
└── rviz/
└── default.rviz # RViz visualization config

## Key Components

### 1. Robot Description (`zenorak.xacro`) - Defines the physical structure of the rover:

- **Base Link**: Main chassis with mass of \_\_ kg
- **Four Wheels**: Each wheel with individual inertial properties
  - Front Left Wheel
  - Front Right Wheel
  - Rear Left Wheel
  - Rear Right Wheel
- **Joint Configuration**: Continuous joints for wheel rotation
- **Hardware Interface**: Configured for Arduino-based differential drive control

### 2. Robot State Publisher (`rsp.launch.py`) - Processes the Xacro file and publishes robot state information:

- Converts Xacro to URDF format
- Publishes TF (transform) tree for all robot links
- Provides `/robot_description` topic for visualization
- Essential for RViz and navigation stack integration

**Key Functions:**

- Reads `zenorak.xacro` from the description folder
- Processes macros into complete URDF
- Launches `robot_state_publisher` node with robot description

### 3. Main Launch File (`launch_robot.launch.py`) - Orchestrates the complete robot system startup with coordinated timing:

**Launch Sequence:**

1. **Robot State Publisher** (Immediate) - Publishes robot URDF and transforms
2. **Controller Manager** (3-second delay)

   - Manages hardware interface to Arduino
   - Loads controller configurations
   - Provides real-time control loop

3. **Differential Drive Controller** (After Controller Manager)

   - Accepts `/cmd_vel` commands
   - Controls front left and right wheels
   - Publishes odometry data

4. **Joint State Broadcaster** (After Controller Manager)
   - Publishes joint positions and velocities
   - Enables state monitoring and visualization

### 4. RViz Configuration (`default.rviz`)

Pre-configured visualization setup:

- Robot model display with collision/visual meshes
- Grid reference frame
- TF tree visualization
- Interactive tools for pose setting and measurements
- Fixed frame: `base_link`
- Orbit camera view for optimal robot observation

### Launching the Robot

# Launch complete robot system

ros2 launch zenorak_ros2_control launch_robot.launch.py

This command will:

1. Start robot state publisher
2. Initialize hardware interface (after 3-second delay)
3. Spawn differential drive controller
4. Spawn joint state broadcaster

### Controlling the Robot - Send velocity commands to the robot:

# Move forward

ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Rotate in place

ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

### Visualizing in RViz

# Launch RViz with default configuration

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix zenorak_ros2_control)/share/zenorak_ros2_control/rviz/default.rviz

### Monitoring Robot State

# Check joint states

ros2 topic echo /joint_states

# Check odometry

ros2 topic echo /diff_cont/odom

# List all active controllers

ros2 control list_controllers

## Dependencies

- ROS 2 (Humble or later recommended)
- `ros2_control` and `ros2_controllers`
- `robot_state_publisher`
- `xacro`
- `rviz2`
- `diffdrive_arduino` hardware plugin
- `controller_manager`

ZENORAK_TELEOP PACKAGE:

## Features

- **Keyboard Control**: Use W, A, S, D keys for intuitive robot movement
- **Incremental Commands**: Sends progressive command values (0-120) at 1Hz
- **Non-blocking Input**: Reads keyboard in a separate thread without blocking ROS operations
- **Clean Termination**: Graceful shutdown with proper terminal restoration

## Package Structure

zenorak_teleop/
├── scripts/
│ └── zenorak_teleop.py # Main teleoperation node
├── launch/
│ └── teleop.launch.py # Launch file
├── CMakeLists.txt # Build configuration
└── package.xml # Package metadata

### Command Protocol - The node publishes to the `zenorak_teleop_cmd` topic with the following format:

| Key   | Command Prefix | Direction | Example Messages       |
| ----- | -------------- | --------- | ---------------------- |
| W     | `f`            | Forward   | `f0`, `f1`, ... `f120` |
| A     | `l`            | Left      | `l0`, `l1`, ... `l120` |
| S     | `b`            | Backward  | `b0`, `b1`, ... `b120` |
| D     | `r`            | Right     | `r0`, `r1`, ... `r120` |
| Enter | `s`            | Stop      | `s0`                   |

### Operation Flow

1. **Key Press Detection**: Continuously reads keyboard input in raw mode (no Enter needed)
2. **Counter Increment**: When a key (W/A/S/D) is held, counter increments from 0 to 120
3. **Message Publishing**: Publishes command at 1Hz (every second) while key is active
4. **Stop Command**: Pressing Enter sends `s0` and resets the system

### Dependencies

# ROS 2 packages

sudo apt install ros-<distro>-rclpy ros-<distro>-std-msgs

### Building

cd ~/ros2_ws
colcon build --packages-select zenorak_teleop
source install/setup.bash

### Method 1: Using ros2 run

ros2 run zenorak_teleop zenorak_teleop.py

### Method 2: Using Launch File

ros2 launch zenorak_teleop teleop.launch.py

### Methods Explained

**`_read_keys()`**

- Runs in separate thread
- Reads single characters from stdin
- Updates `active_key` when W/A/S/D pressed
- Sends stop command on Enter

**`_tick()`**

- Called every 1 second by ROS timer
- Publishes command if key is active
- Increments counter (0 → 120)
- Stops incrementing at 120

**`destroy_node()`**

- Stops keyboard thread
- Restores terminal settings
- Cleans up resources

ZENORAK_SERIAL PACKAGE:

## Package Structure

zenorak_serial/
├── scripts/
│ └── zenorak_serial.py # Serial bridge node
├── CMakeLists.txt # Build configuration
└── package.xml # Package metadata

### Message Processing

1. **Subscribe**: Listens to `/zenorak_teleop_cmd` topic
2. **Extract**: Gets command string from ROS message (e.g., "f25")
3. **Format**: Adds newline character for Arduino's `readStringUntil('\n')`
4. **Transmit**: Sends via serial to `/dev/ttyACM0` at 115200 baud
5. **Log**: Records successful transmission for debugging

### Serial Protocol

**Examples:**

- `f25\n` → Forward command, intensity 25
- `l10\n` → Left turn, intensity 10
- `s0\n` → Stop command

## Installation & Setup

### Dependencies

# ROS 2 packages

sudo apt install ros-<distro>-rclpy ros-<distro>-std-msgs

# Python serial library

sudo apt install python3-serial

# OR

pip3 install pyserial

### Starting the Bridge

ros2 run zenorak_serial zenorak_serial.py

### Testing Communication

**Terminal 1: Start the bridge**

```bash
ros2 run zenorak_serial zenorak_serial.py
```

**Terminal 2: Send test command**

```bash
ros2 topic pub /zenorak_teleop_cmd std_msgs/msg/String "{data: 'f50'}"
```

## Full Workflow

### 1. Hardware Setup

```bash
# Connect Arduino via USB
# Upload control firmware to Arduino
# Verify connection: ls /dev/ttyACM0
```

### 2. Launch Main Control System

```bash
# Terminal 1: Launch robot control
ros2 launch zenorak_ros2_control launch_robot.launch.py
```

### 3. Start Serial Bridge

```bash
# Terminal 2: Start Arduino bridge
ros2 run zenorak_serial zenorak_serial.py
```

### 4. Start Teleoperation

```bash
# Terminal 3: Start keyboard control
ros2 run zenorak_teleop zenorak_teleop.py
```

### 5. Control the Robot

- Use W/A/S/D keys to drive
- Press Enter to stop
- Watch RViz for visualization
