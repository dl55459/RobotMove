from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

camera_config_path = os.path.join(
        get_package_share_directory('eyes_v1'),
        'config',
        'camera_params.yaml'
        )
        
def generate_launch_description():

	# Turn on camera
    usb_cam_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name="camera",
            parameters=[camera_config_path],
            output='screen',
    )

	# Turn on yolo
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch',
                'yolov12.launch.py'
            )
        )
    )
	# Turn on robot eyes
    robot_eyes_node = Node(
        package='eyes_v1',
        executable='robot_eyes_v1',
        name='robot_eyes',
        output='screen',
    )

    return LaunchDescription([
    	usb_cam_node,
        yolo_launch,
        robot_eyes_node
    ])

