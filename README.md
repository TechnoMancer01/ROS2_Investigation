# ROS2 Mill Relining Robot (MRM) Investigation

A comprehensive ROS2 implementation for mill relining robot simulation and kinematics analysis, featuring a 7-DOF manipulator system modeled after the Russel 7 mill relining machine.

## Project Overview

This project presents a technical analysis of ROS2 implementation for autonomous mill relining operations through the development of a Russel 7-like manipulator system. The investigation focuses on kinematic modeling, inverse kinematics calculations, motion planning, and visualization capabilities within the ROS2 ecosystem.

### Key Features

- **7-DOF Kinematic Model**: Complete URDF/Xacro representation of a mill relining robot
- **Inverse Kinematics**: MoveIt2-based IK solver with KDL plugin
- **Interactive Tools**: Command-line scripts for easy IK calculations and joint control
- **Motion Planning**: Integration with MoveIt2 for trajectory generation and optimization
- **3D Visualization**: RViz2 integration for real-time robot state visualization
- **Modular Architecture**: Scalable design allowing easy configuration updates

## System Architecture

The system leverages ROS2's distributed node architecture with the following key components:

### Kinematic Configuration (7-DOF)
1. **Prismatic beam joint** - Horizontal positioning across mill width
2. **Revolute turntable** - 360° rotational coverage inside mill
3. **Revolute pitch mechanism** - Vertical positioning of boom
4. **Prismatic boom extension** - Horizontal reach adjustment
5. **Revolute slewing joint** - Grapple assembly rotation
6. **Revolute roll joint** - Grapple orientation adjustment
7. **Revolute pitch joint** - End effector tilting

### Core Technologies
- **ROS2 Kilted Kaiju** - Core framework and middleware
- **MoveIt2** - Motion planning and kinematics
- **RViz2** - 3D visualization and debugging
- **KDL Kinematics Plugin** - Inverse kinematics calculations
- **URDF/Xacro** - Robot description and modeling

## Getting Started

### Prerequisites

- **Ubuntu 24.04.2 LTS** (recommended)
- **ROS2 Kilted Kaiju** (latest)
- **MoveIt2**
- **Colcon build tools**

### Installation

1. **Install ROS2 Kilted Kaiju**:
   ```bash
   # Follow official ROS2 installation guide
   # https://docs.ros.org/en/kilted/Installation.html
   ```

2. **Source ROS2 environment**:
   ```bash
   echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Clone the repository**:
   ```bash
   git clone https://github.com/TechnoMancer01/ROS2_Investigation.git
   cd ROS2_Investigation
   ```

4. **Install dependencies**:
   ```bash
   sudo apt update
   sudo apt install ros-kilted-moveit ros-kilted-joint-state-publisher-gui
   ```

5. **Build the workspace**:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Usage

### Quick Start - Full System Launch

The easiest way to start the complete system:

```bash
./launch_MRM.sh
```

This script will:
- Clean previous builds
- Build the workspace
- Launch MoveIt2 with robot visualization
- Start RViz2 with motion planning interface
- Initialize joint state publisher GUI

### Manual Launch

For more control over individual components:

```bash
# Build and source workspace
colcon build
source install/setup.bash

# Launch robot description and MoveIt2
ros2 launch moveit_config demo.launch.py
```

### Interactive Inverse Kinematics

Use the interactive IK solver script:

```bash
./ik_solver.sh
```

This script provides:
- Interactive prompts for target position (X, Y, Z)
- Quaternion orientation input
- Automatic IK calculation via MoveIt2
- Joint position extraction and visualization
- Real-time robot state updates

#### Example IK Usage:
```bash
$ ./ik_solver.sh
Interactive Inverse Kinematics Solver
==========================================

Enter target position coordinates:
X position (meters): 10.0
Y position (meters): -4.0
Z position (meters): 1.0

Enter grapple orientation (quaternion):
Note: For no rotation, use x=0, y=0, z=0, w=1
Orientation X: 0.0
Orientation Y: 0.0
Orientation Z: 0.0
Orientation W: 1.0

Computing inverse kinematics...
Position: (10.0, -4.0, 1.0)
Orientation: (0.0, 0.0, 0.0, 1.0)

IK solution found! Extracting joint positions...
Joint positions: [1.427, 0.935, 0.644, 1.533, 1.213, -0.612, -1.139]

Publishing joint states to robot...
✓ Robot updated successfully!
```

### Manual Service Calls

For advanced users, direct ROS2 service calls:

```bash
# Compute inverse kinematics
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'relining_arm',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 10.0, y: -4.0, z: 1.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    timeout: {sec: 5},
    avoid_collisions: false
  }
}"

# Publish joint states
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['beam_joint', 'turn_table', 'boom_pitch', 'boom_inner_joint', 'grapple_slew', 'grapple_roll', 'grapple_pitch'],
  position: [1.427, 0.935, 0.644, 1.533, 1.213, -0.612, -1.139],
  velocity: [],
  effort: []
}" --once
```

## Project Structure

```
ROS2_MRM/
├── src/
│   ├── ROS2_MRM/              # Main robot package
│   │   ├── launch/            # Launch files
│   │   ├── urdf/              # Robot description files
│   │   ├── rviz/              # RViz configurations
│   │   └── package.xml        # Package dependencies
│   └── moveit_config/         # MoveIt2 configuration
│       ├── config/            # MoveIt2 config files
│       │   ├── kinematics.yaml
│       │   ├── joint_limits.yaml
│       │   ├── ompl_planning.yaml
│       │   └── ros2_controllers.yaml
│       └── launch/            # MoveIt2 launch files
├── launch_MRM.sh              # Main launch script
├── ik_solver.sh               # Interactive IK solver
├── ROS2_Investigation.tex     # Technical documentation
└── README.md                  # This file
```

## Configuration

### Joint Limits

Joint limits are configured in `src/moveit_config/config/joint_limits.yaml`:

```yaml
joint_limits:
  beam_joint:
    max_position: 5.5
    min_position: 0.0
    max_velocity: 0.5
  turn_table:
    max_position: 6.28318  # 2π radians
    min_position: -6.28318
    max_velocity: 1.0
  # ... additional joints
```

### Kinematics Solver

The system uses KDL kinematics plugin configured in `src/moveit_config/config/kinematics.yaml`:

```yaml
relining_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 5.0
```

## Technical Highlights

### Inverse Kinematics Implementation
- **Algorithm**: KDL (Kinematics and Dynamics Library) Jacobian-based solver
- **Timeout**: 5-second calculation limit
- **Search Resolution**: 0.005 for optimal precision/speed balance
- **Group Name**: `relining_arm` (all 7 joints)

### Joint Configuration
- **Prismatic Joints**: `beam_joint`, `boom_inner_joint`
- **Revolute Joints**: `turn_table`, `boom_pitch`, `grapple_slew`, `grapple_roll`, `grapple_pitch`
- **Coordinate Frame**: `base_link` reference frame
- **Units**: Meters for prismatic, radians for revolute

### Motion Planning
- **Framework**: MoveIt2 with OMPL planning library
- **Algorithms**: RRT-Connect for efficient path generation
- **Collision Detection**: Self-collision avoidance enabled
- **Real-time Updates**: Live joint state visualization

## Troubleshooting

### Common Issues

1. **Build Failures**:
   ```bash
   # Clean build and retry
   rm -rf build/ install/ log/
   colcon build
   ```

2. **Missing Dependencies**:
   ```bash
   # Install missing MoveIt2 components
   sudo apt install ros-kilted-moveit-*
   ```

3. **Service Not Available**:
   ```bash
   # Check if MoveIt2 is running
   ros2 service list | grep compute_ik
   
   # Restart if needed
   ros2 launch moveit_config demo.launch.py
   ```

4. **Joint State Publisher Issues**:
   ```bash
   # Verify joint state publisher is active
   ros2 topic list | grep joint_states
   ros2 topic echo /joint_states
   ```

### Environment Setup

Ensure proper environment sourcing:
```bash
# Source ROS2
source /opt/ros/kilted/setup.bash

# Source workspace (after every build)
source install/setup.bash
```

## Documentation

- **Technical Report**: `ROS2_Investigation.tex` - Comprehensive technical analysis and implementation details
- **ROS2 Official Docs**: [docs.ros.org](https://docs.ros.org/en/kilted/)
- **MoveIt2 Documentation**: [moveit.picknik.ai](https://moveit.picknik.ai/kilted/index.html)

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-capability`)
3. Commit your changes (`git commit -am 'Add new capability'`)
4. Push to the branch (`git push origin feature/new-capability`)
5. Create a Pull Request

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Author

**Alexander Armstrong**  

## Future Development

Potential areas for enhancement:
- **Sensor Integration**: Implementation of ROS2 Control for real hardware
- **Collision Avoidance**: Advanced obstacle detection and path planning
- **Multi-Robot Collaboration**: Fleet management with Robotics Middleware Framework (RMF)
- **Hardware Integration**: Real-time control of physical mill relining robots

## Acknowledgments

- ROS2 development team for the robust framework
- MoveIt2 community for motion planning capabilities
- Open Motion Planning Library (OMPL) contributors
- Russel Mineral Equipment for the industrial application context

---

*This project demonstrates the feasibility and potential of ROS2 for complex industrial automation applications in the mining and mineral processing industry.*
