# filepath: /home/aarmstrong/mill_relining_robot_ws/src/mill_relining_robot/launch/robot_description.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package path
    pkg_share = get_package_share_directory('ROS2_MRM')
    
    # Path to the Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'mill_relining_robot.urdf.xacro')
    
    # Process the xacro file to generate URDF
    robot_description_content = Command(['xacro ', xacro_file])
    
    # Robot State Publisher - starts first
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher GUI - delayed start to ensure robot description is available
    joint_state_publisher_gui = TimerAction(
        period=3.0,  # Wait 3 seconds for robot state publisher to fully initialize
        actions=[
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }]
            )
        ]
    )
    
    # RViz2 - delayed start to ensure everything is ready
    rviz2 = TimerAction(
        period=4.0,  # Wait 4 seconds
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_config.rviz')] if os.path.exists(os.path.join(pkg_share, 'rviz', 'robot_config.rviz')) else []
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        #robot_state_publisher,
        #joint_state_publisher_gui,
        #rviz2
    ])