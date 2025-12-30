# zenorak_ros2_control — ROS 2 Control (Quick Reference)

This document is a concise reference for the `zenorak_ros2_control` package. It explains what the bundle of launch files starts, which controllers are spawned, the most important topics you will use to send commands and read feedback, useful example commands (trajectory and teleop), and some troubleshooting tips.

Keep this README handy when running demos or handing off the code to other team members.

## Quick summary

- Primary launch file: `launch/launch_robot.launch.py`
- Includes: `launch/rsp.launch.py` (robot_state_publisher + URDF)
- Controller manager: `ros2_control_node` (started via `controller_manager` package)
- Controllers spawned automatically by the launch: `diff_cont`, `joint_broad`, `arm_trajectory_controller`,'joint_state_broadcaster'

## What runs when you run the main launch

Command to start everything (from the workspace root after sourcing the workspace):

```bash
source install/setup.bash
ros2 launch zenorak_ros2_control launch_robot.launch.py
```

What this does (high level):

- Starts `robot_state_publisher` (from `rsp.launch.py`) which publishes the robot TFs and the `robot_description` parameter (URDF).
- Starts `ros2_control_node` (controller_manager) with the robot description and controller configuration YAMLs.
- Spawns controllers using `controller_manager` spawner tools after the controller manager is up:
  - `diff_cont` — differential-drive velocity controller (subscribes to cmd_vel-like topic)
  - `joint_broad` — joint state broadcaster (publishes `/joint_states`)
  - `arm_trajectory_controller` — trajectory-based controller for the manipulator arm
  - 'joint_state_broadcaster' - Joint State broadcaster for the overall feedback for actuator

The launch script delays the controller manager start briefly so the `robot_description` is available and registers event handlers so controllers are spawned only after the controller manager is running.

## Important topics (command & feedback)

Control topics (what you send to):

- `/arm_trajectory_controller/joint_trajectory` [trajectory_msgs/msg/JointTrajectory]
  - Send joint trajectories to the arm controller.
- `/diff_cont/cmd_vel_unstamped` [geometry_msgs/msg/Twist]
  - Velocity commands for the differential drive controller (diff_cont).

Feedback topics (what you read from):

- `/arm_trajectory_controller/state` [control_msgs/msg/JointTrajectoryControllerState]
  - State & feedback from the arm trajectory controller.
- `/diff_cont/odom` [nav_msgs/msg/Odometry]
  - Odometry from the differential drive controller / hardware interface.
- `/joint_states` [sensor_msgs/msg/JointState]
  - Consolidated joint positions/velocities/efforts (published by `joint_broad`).
- `/robot_description` [std_msgs/msg/String]
  - The URDF/XML published by `robot_state_publisher` (useful for diagnostics or confirming the loaded robot model).

You can list topics and types with:

```bash
ros2 topic list
ros2 topic list -t
```

And echo a topic to inspect messages:

```bash
ros2 topic echo /joint_states
ros2 topic echo /diff_cont/odom
ros2 topic echo /arm_trajectory_controller/state
```

## Example commands

1. Publish a one-shot joint trajectory to move the arm (example you already used):

```bash
ros2 topic pub --once /arm_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names:
- linkk_1
- link_2
- link_3
points:
- positions: [0.0, 80.0, 0.0]
  time_from_start:
    sec: 0
"
```

Notes:

- The `joint_names` must match the joints used by the controller (check `/joint_states` for naming).
- `time_from_start` = 0 makes this a position setpoint; for smooth motion provide a non-zero time.

2. Run teleop for driving the base (remap cmd_vel to the controller topic):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

This remaps the `teleop_twist_keyboard` default `/cmd_vel` to the controller's `/diff_cont/cmd_vel_unstamped` topic.

3. Spawn or stop a controller manually (if you need to):

- Spawn (alternative to automatic spawner):

```bash
ros2 run controller_manager spawner diff_cont
ros2 run controller_manager spawner arm_trajectory_controller
```

- To stop/unload a controller use the spawner with the `--stopped` flag (or use the controller manager services):

```bash
ros2 run controller_manager spawner --stopped diff_cont
```

If you prefer services, the controller manager exposes service calls such as `/controller_manager/list_controllers` and `/controller_manager/spawn_controller` (useful for scripts).

## How to verify controllers are running

- Check controller topics and feedback:

```bash
ros2 topic list -t
ros2 topic echo /arm_trajectory_controller/state
ros2 topic echo /diff_cont/odom
ros2 topic echo /joint_states
```

- Watch controller manager logs (in the terminal where the launch was run). The spawner prints messages when controllers are started.
- Use `ros2 service list` and check `/controller_manager/*` services for lifecycle & management operations.

## Troubleshooting

- Error: "Unable to parse the value of parameter robot_description as yaml"
  - Cause: `robot_description` contains URDF/XML which the ROS 2 launch parameter loader may try to parse as YAML. XML has characters that confuse the YAML parser.
  - Solution: ensure launch files pass the URDF as a raw string. Example fix we applied in `launch/rsp.launch.py` and `launch/launch_robot.launch.py`:

```python
from launch_ros.parameter_descriptions import ParameterValue
robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
params = {'robot_description': ParameterValue(robot_description, value_type=str)}
```

- If you see that error, check the launch file(s) that set `robot_description` and wrap the value in `ParameterValue(..., value_type=str)`.

- If controllers fail to spawn:
  - Confirm `ros2_control_node` is running and has loaded the controller YAMLs.
  - Check log output from the spawner node; it will indicate why a controller failed to start (missing hardware interface, parameter errors, missing joints, etc.).

## Helpful quick commands

- Source workspace:

```bash
source install/setup.bash
```

- Launch the robot + controllers:

```bash
ros2 launch zenorak_ros2_control launch_robot.launch.py
```

- Echo a topic:

```bash
ros2 topic echo /joint_states
```

- Spawn a controller manually:

```bash
ros2 run controller_manager spawner arm_trajectory_controller
```

# Programmatic examples for zenorak_ros2_control

This file contains short Python examples showing how to call key behaviors from code.

## 1) Publish a one-shot JointTrajectory to `/arm_trajectory_controller/joint_trajectory` using rclpy

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def send_one_shot_trajectory():
    rclpy.init()
    node = Node('traj_pub')
    pub = node.create_publisher(JointTrajectory, '/arm_trajectory_controller/joint_trajectory', 10)

    msg = JointTrajectory()
    msg.joint_names = ['linkk_1', 'link_2', 'link_3']
    point = JointTrajectoryPoint()
    point.positions = [0.0, 80.0, 0.0]
    # time_from_start=0 -> immediate setpoint; use non-zero for smooth motion
    point.time_from_start.sec = 0
    msg.points = [point]

    # publish once
    pub.publish(msg)
    node.get_logger().info('Published one-shot JointTrajectory')

    # give the middleware a short time to send
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    send_one_shot_trajectory()
```

## 2) Publish a Twist to drive the base programmatically

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def send_twist(linear_x=0.2, angular_z=0.0):
    rclpy.init()
    node = Node('twist_pub')
    pub = node.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    t = Twist()
    t.linear.x = float(linear_x)
    t.angular.z = float(angular_z)

    pub.publish(t)
    node.get_logger().info(f'Published Twist linear.x={linear_x} angular.z={angular_z}')
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    send_twist(0.2, 0.1)
```

## 3) Spawn a controller from Python (calls the `spawner` executable)

```python
#!/usr/bin/env python3
import subprocess


def spawn_controller(controller_name: str):
    # This blocks until the spawner process finishes; it prints logs to stdout/stderr.
    subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', controller_name], check=True)


if __name__ == '__main__':
    spawn_controller('diff_cont')
```

Notes:

- The rclpy examples assume your ROS_DOMAIN_ID and environment are set appropriately and you have sourced your workspace where nodes are running.
- For programmatic controller management using services, you can call controller manager services (`/controller_manager/list_controllers`, `/controller_manager/switch_controller`, etc.) via rclpy — that requires the controller_manager_msgs service types and slightly more boilerplate, so the spawner subprocess is a quick option.
