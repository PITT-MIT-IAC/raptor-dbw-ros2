import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    should_launch_can_arg = DeclareLaunchArgument(
        "should_launch_can",
        default_value="true",
        description="Whether or not to launch CAN drivers",
    )

    dbc_file_path = get_share_file(
        package_name="raptor_dbw_can", file_name="config/CAN1-INDY-V9.dbc"
    )

    socketcan_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_share_file(
                    package_name="ros2_socketcan",
                    file_name="launch/socket_can_receiver.launch.py",
                )
            ]
        ),
        launch_arguments={"interface": "can0", "use_bus_time": "True"}.items(),
        condition=IfCondition(LaunchConfiguration("should_launch_can")),
    )

    socketcan_sender_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_share_file(
                    package_name="ros2_socketcan",
                    file_name="launch/socket_can_sender.launch.py",
                )
            ]
        ),
        launch_arguments={"interface": "can0"}.items(),
        condition=IfCondition(LaunchConfiguration("should_launch_can")),
    )

    raptor_node = Node(
        package="raptor_dbw_can",
        executable="raptor_dbw_can_node",
        output="screen",
        namespace="raptor_dbw_interface",
        parameters=[{"dbw_dbc_file": dbc_file_path}],
        remappings=[
            ("/raptor_dbw_interface/can_rx", "/from_can_bus"),
            ("/raptor_dbw_interface/can_tx", "/to_can_bus"),
        ],
    )

    return LaunchDescription(
        [
            should_launch_can_arg,
            socketcan_receiver_launch,
            socketcan_sender_launch,
            raptor_node,
        ]
    )
