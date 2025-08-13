import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    moveit_config_pkg = get_package_share_directory("moveit_config")
    ros2_mrm_pkg = get_package_share_directory("ROS2_MRM")
    
    # Robot description from xacro
    urdf_file = os.path.join(ros2_mrm_pkg, "urdf", "mill_relining_robot.urdf.xacro")
    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )
    
    # Robot description semantic (SRDF)
    srdf_file = os.path.join(moveit_config_pkg, "config", "mill_relining_robot.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic_content = f.read()
    
    # Load configuration files
    kinematics_yaml = os.path.join(moveit_config_pkg, "config", "kinematics.yaml")
    with open(kinematics_yaml, "r") as f:
        kinematics_config = yaml.safe_load(f)
    
    joint_limits_yaml = os.path.join(moveit_config_pkg, "config", "joint_limits.yaml")
    with open(joint_limits_yaml, "r") as f:
        joint_limits_config = yaml.safe_load(f)
        
    planning_pipeline_yaml = os.path.join(moveit_config_pkg, "config", "planning_pipeline.yaml")
    with open(planning_pipeline_yaml, "r") as f:
        planning_pipeline_config = yaml.safe_load(f)
        
    ompl_planning_yaml = os.path.join(moveit_config_pkg, "config", "ompl_planning.yaml")
    with open(ompl_planning_yaml, "r") as f:
        ompl_planning_config = yaml.safe_load(f)
    
    # Basic MoveIt parameters
    moveit_config_dict = {
        "robot_description": robot_description_content,
        "robot_description_semantic": robot_description_semantic_content,
        "robot_description_kinematics": kinematics_config,
        "robot_description_planning": joint_limits_config,
    }
    
    # Add planning pipeline config
    moveit_config_dict.update(planning_pipeline_config)
    moveit_config_dict.update(ompl_planning_config)
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )
    
    # Joint State Publisher (without GUI) - allows external control
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_gui": False},
            {"publish_default_positions": False}  # Changed to False - allows manual control
        ],
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )
    
    # RViz with MoveIt config (delayed to ensure move_group is ready)
    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config_dict],
    )
    
    # Delay RViz startup to ensure move_group is ready
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        robot_state_publisher,
        
        # enable or disable one of the following to perform manual joint control with the ik_solver script or approximation with the GUI
        #joint_state_publisher_gui,
        joint_state_publisher,
        
        move_group_node,
        delayed_rviz,
    ])
