import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_name = "prarob_manipulator"
    package_name = "ros2_prarob"
    vision_package_name = "prarob_vision"

    # Get package directories
    package_dir = get_package_share_directory(package_name)
    vision_package_dir = get_package_share_directory(vision_package_name)

    rviz_config = os.path.join(package_dir, "launch", "prarob_manipulator.rviz")
    robot_description = os.path.join(package_dir, "urdf", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(package_dir, "controllers", "controllers.yaml")

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        # Add move_robot node from prarob_vision package
        Node(
            package=vision_package_name,
            executable='move_robot.py',
            name='move_robot_node',
            output='screen',
            cwd=os.path.join(vision_package_dir, 'scripts')  # If scripts are in scripts folder
        ),

        # Add pathfinder node from prarob_vision package
        Node(
            package=vision_package_name,
            executable='pathfinder_a_star.py',
            name='pathfinder_node',
            output='screen',
            cwd=os.path.join(vision_package_dir, 'scripts')  # If scripts are in scripts folder
        )
    ])