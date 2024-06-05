
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from pathlib import Path
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    # URDF loading
    package_path = get_package_share_path("diablo_simulation")
    default_model_path = package_path / "urdf/diablo_description.xacro"

    robot_urdf = LaunchConfiguration("urdf_path")

    robot_urdf_arg = DeclareLaunchArgument(
        name="urdf_path",
        default_value=str(default_model_path),
        description="Absolute path to robot URDF file",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                robot_urdf,
                #" namespace:=",
                #namespace,
                #" enable_simulation:=",
                #enable_simulation,
                #" enable_cameras:=",
                #enable_cameras,
                #" simulation_tf_base_frame:=",
                #simulation_tf_base_frame,
            ]
        ),
        value_type=str,
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # namespace=namespace,
        parameters=[
            {"robot_description": robot_description}
        ],
        output="both",
        # arguments=["--ros-args", "--log-level", "warn"],
    )

    return LaunchDescription([
	
        # Velodyne Convert launch with a delay
        TimerAction(
           period=1.0,
           actions=[
               ExecuteProcess(
                   cmd=['ros2', 'launch', 'velodyne_driver', 'velodyne_driver_node-VLP16-launch.py'],
                   output='screen'
               )
           ]
        ),

        # Velodyne Driver launch with a delay
        TimerAction(
           period=1.0,
           actions=[
               ExecuteProcess(
                   cmd=['ros2', 'launch', 'velodyne_pointcloud', 'velodyne_convert_node-VLP16-launch.py'],
                   output='screen'
               )
           ]
        ),

	# Velodyne LaserScan launch with a delay
        #TimerAction(
        #  period=2.0,
        #  actions=[
        #      ExecuteProcess(
        #          cmd=['ros2', 'launch', 'velodyne_laserscan', 'velodyne_laserscan_node-launch.py'],
        #          output='screen'
        #      )
        #  ]

        #),
        
        # RealSense Camera with a delay
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                  # cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 'enable_gyro:=true', 'enable_accel:=true', 'unite_imu_method:=2','tf_publish_rate:=5.0', 'publish_tf:=true', 'align_depth.enable:=true', 'initial_reset:=true', 'rgb_camera.profile:="848,480,15"','depth_module.profile:="848,480,15"', 'enable_sync:=true'],
                    cmd=[
                        'ros2',
                        'launch',
                        'realsense2_camera',
                        'rs_launch.py',
                        'enable_gyro:=true',
                        'enable_accel:=true',
                        'accel_fps:=250',
                        'gyro_fps:=400',
                        'unite_imu_method:=2',
                        'publish_tf:=true',
                        'initial_reset:=true',
                        'rgb_camera.profile:="848,480,15"',
                        'depth_module.profile:="848,480,15"',
                        'enable_sync:=true',
                        'hold_back_imu_for_frames:=true',
                    ],
                    output='screen'
                )
            ]
        ),
        
        # RTAB-Map with a delay
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.expanduser('~/diablo_ws/misc/'), 'realsense_d435i_color.launch.py'])
               )
            ]
        ),
        robot_urdf_arg,
        robot_state_publisher_node,
    ])
