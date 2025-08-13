#!/bin/bash

# Interactive IK Solver Script
# Prompts user for position and orientation, then calls MoveIt IK service

echo "Interactive Inverse Kinematics Solver"
echo "=========================================="
echo ""

# Change to workspace directory and source setup
cd /home/aarmstrong/ROS2_MRM
source install/setup.bash

# Prompt for position coordinates
echo "Enter target position coordinates:"
read -p "X position (meters): " pos_x
read -p "Y position (meters): " pos_y
read -p "Z position (meters): " pos_z

echo ""

# Prompt for orientation
echo "Enter grapple orientation (quaternion):"
echo "Note: For no rotation, use x=0, y=0, z=0, w=1"
read -p "Orientation X: " orient_x
read -p "Orientation Y: " orient_y
read -p "Orientation Z: " orient_z
read -p "Orientation W: " orient_w

echo ""
echo "Computing inverse kinematics..."
echo "Position: ($pos_x, $pos_y, $pos_z)"
echo "Orientation: ($orient_x, $orient_y, $orient_z, $orient_w)"
echo ""

# Call the IK service with user-provided values and capture response
ik_response=$(ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'relining_arm',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: $pos_x, y: $pos_y, z: $pos_z},
        orientation: {x: $orient_x, y: $orient_y, z: $orient_z, w: $orient_w}
      }
    },
    timeout: {sec: 5},
    avoid_collisions: false
  }
}")

#echo "IK Service Response:"
#echo "$ik_response"
#echo ""

# Check if IK solution was found
if echo "$ik_response" | grep -q "error_code.*val=1"; then
    echo "IK solution found! Extracting joint positions..."
    
    # Extract joint positions from the response
    # The response contains: position=[val1, val2, val3, val4, val5, val6, val7] within the solution section
    joint_positions=$(echo "$ik_response" | grep -o "solution=.*position=\[[^]]*\]" | grep -o "position=\[[^]]*\]" | sed 's/position=\[\(.*\)\]/\1/')

    if [ -n "$joint_positions" ]; then
        echo "Joint positions: [$joint_positions]"
        echo ""
        echo "Publishing joint states to robot..."
        
        # Publish the joint states
        ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
        header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
        name: ['beam_joint', 'turn_table', 'boom_pitch', 'boom_inner_joint', 'grapple_slew', 'grapple_roll', 'grapple_pitch'],
        position: [$joint_positions],
        velocity: [],
        effort: []
        }" --once > /dev/null 2>&1
        
        echo "Joint states published successfully!"
        echo "Robot should now move to the target position"
    else
        echo "Failed to extract joint positions from response"
    fi
else
    echo "IK solution not found. The target pose may be unreachable."
    echo "Try adjusting the position or orientation values."
fi
