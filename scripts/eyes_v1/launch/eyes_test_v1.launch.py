from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('usb_cam').find('usb_cam'),
                'launch',
                'home_camera.launch.py'
            )
        )
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('yolo_bringup').find('yolo_bringup'),
                'launch',
                'yolov12.launch.py'  # adjust as needed
            )
        )
    )

    return LaunchDescription([
        usb_cam_launch,
        yolo_launch
    ])

