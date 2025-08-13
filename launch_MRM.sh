#!/bin/bash

# Synchronized launch script for robot visualization
cd /home/aarmstrong/ROS2_MRM

# Delete the the previous builds "install/" "build/" and "log/" folders/files
rm -r install/ build/ log/ 

# Source ROS2
source /opt/ros/kilted/setup.bash

# Build the package first
echo "Building package..."
colcon build
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Source the workspace
source install/setup.bash

# Kill any existing ROS processes
echo "Cleaning up existing ROS processes..."
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f rviz2
sleep 2

# Step 1: Start robot state publisher first and wait for it to be ready
echo "Starting robot state publisher..."
ros2 launch moveit_config demo.launch.py &
LAUNCH_PID=$!

# Wait for robot description to be published
echo "Waiting for robot description to be published..."
timeout=30
counter=0
while [ $counter -lt $timeout ]; do
    if ros2 topic list | grep -q "/robot_description"; then
        echo "Robot description topic found!"
        break
    fi
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo "Timeout waiting for robot description!"
    kill $LAUNCH_PID
    exit 1
fi

# Step 2: Wait a bit more for robot state publisher to fully initialize
echo "Waiting for robot state publisher to initialize..."
sleep 3

# Step 3: Verify joint states are being published
echo "Verifying joint states are being published..."
timeout=10
counter=0
while [ $counter -lt $timeout ]; do
    if ros2 topic list | grep -q "/joint_states"; then
        echo "Joint states topic found!"
        break
    fi
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo "Warning: Joint states topic not found, but continuing..."
fi

echo "System is ready! Joint state publisher GUI should have working sliders."
echo "Press Ctrl+C to stop all processes."

# Wait for user to stop
wait $LAUNCH_PID
