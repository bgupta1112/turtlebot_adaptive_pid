# Turtlebot Adaptive PID Controller

A ROS 2 package implementing an adaptive PID controller for the turtlesim simulator.

## Description

This package demonstrates:
- PID control implementation in ROS 2
- Adaptive gain tuning
- Publisher/Subscriber pattern
- Launch file configuration

## Author

Bibek - PhD Student, Northeastern University  
Silicon Synapse Lab - Guidance, Navigation, and Control for Autonomous Aerial Systems

## Prerequisites

- ROS 2 Humble
- Python 3.10+
- turtlesim package

## Installation
```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/turtlebot_adaptive_pid.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select turtlebot_adaptive_pid

# Source the workspace
source install/setup.bash
```

## Usage
source install/setup.bash
### Launch the controller with turtlesim:
```bash
ros2 launch turtlebot_adaptive_pid turtlebot_pid_launch.py
```

### Run nodes separately (for debugging):

Terminal 1:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:
```bash
ros2 run turtlebot_adaptive_pid pid
```

## Configuration

Edit the goal position in `turtlebot_adaptive_pid/pid.py`:
```python
self.goal_x = 9.0  # Target X coordinate
self.goal_y = 9.0  # Target Y coordinate
```

## Package Structure
```
turtlebot_adaptive_pid/
├── turtlebot_adaptive_pid/
│   ├── __init__.py
│   └── pid.py              # Main PID controller node
├── launch/
│   └── turtlebot_pid_launch.py  # Launch file
├── resource/
├── test/
├── package.xml             # Package dependencies
├── setup.py               # Python package configuration
├── setup.cfg
└── README.md
```

## Topics

**Subscribed:**
- `/turtle1/pose` (turtlesim/msg/Pose) - Turtle position and orientation

**Published:**
- `/turtle1/cmd_vel` (geometry_msgs/msg/Twist) - Velocity commands

## Parameters

- `Kp_linear`: Proportional gain for linear velocity (default: 1.0)
- `Ki_linear`: Integral gain for linear velocity (default: 0.0)
- `Kd_linear`: Derivative gain for linear velocity (default: 0.1)
- `Kp_angular`: Proportional gain for angular velocity (default: 4.0)
- `Ki_angular`: Integral gain for angular velocity (default: 0.0)
- `Kd_angular`: Derivative gain for angular velocity (default: 0.5)

## Future Work

- [ ] Implement adaptive gain adjustment
- [ ] Add parameter server support
- [ ] Implement multiple waypoint navigation
- [ ] Add data logging and visualization
- [ ] Implement anti-windup for integral term

## License

Apache 2.0

## Acknowledgments

Built as part of learning ROS 2 fundamentals at Northeastern University.
