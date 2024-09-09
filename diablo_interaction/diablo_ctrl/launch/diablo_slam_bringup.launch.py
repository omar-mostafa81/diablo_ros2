from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from pathlib import Path
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, SetLaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import ament_index_python.packages
import launch_ros.actions
import yaml
import os


def generate_launch_description():

    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

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
            ]
        ),
        value_type=str,
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description}
        ],
        output="both",
    )
    
    #RGBD odometry from realsense d435i camera
    parameters_rgbd=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync': False,
          'wait_for_transform': 0.2,
          'use_sim_time': LaunchConfiguration('use_sim_time'),
          'publish_tf': False,
          'wait_imu_to_init':True,
          'Odom/Strategy':'1', #0: F2M, 1: F2F
          'Vis/CorType':'1', #0: Feuters Matching, 1: Optical Flow (more matches but less robust)
          'GFTT/MinDistance':'15', #Default: 5
          #'Odom/ResetCountdown': '5',
          }]

    remappings_rgbd=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/realigned_depth_to_color/image_raw'),
          ('/odom_rgbd_image','/rgbd_odom_rgbd_image'),
          ('/odom_last_frame','/rgbd_odom_last_frame'),
          ('/odom_local_map','/rgbd_odom_local_map'),
          ('/odom_info_lite','/rgbd_odom_info_lite'),
          ('/odom','/rgbd_odom'),
          ('/odom_info','/rgbd_odom_info'),
          ('/odom_local_scan_map','/rgbd_odom_local_scan_map')
    ]
    
    rtabmap_rgbd_odom = Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters_rgbd,
            remappings=remappings_rgbd,
            arguments=["--ros-args", "--log-level", "warn"])
          
    #Icp odometry from LIDAR 
    parameters_icp=[{
          'frame_id':'base_link',
          'wait_for_transform': 0.2,
          'deskewing': False,
          'use_sim_time': LaunchConfiguration('use_sim_time'),
          'scan_cloud_max_points': 0,
          'scan_voxel_size': 0.1,
          'wait_imu_to_init': True,
          'publish_tf': True,
          'Odom/Holonomic': 'false',
          'Icp/PointToPlaneK': '15',
          #'Odom/ResetCountdown': '10',
          'Odom/ImageBufferSize': '10',
          'Icp/VoxelSize': '0.1',
          'Icp/MaxTranslation': '0.7',
          'Icp/Iterations': '15',
          }]

    remappings_icp=[
          ('scan_cloud', '/velodyne_points'),
          ('imu', '/imu/data'),
          ('/odom_rgbd_image','/icp_odom_rgbd_image'),
          ('/odom_last_frame','/icp_odom_last_frame'),
          ('/odom_local_map','/icp_odom_local_map'),
          ('/odom_info_lite','/icp_odom_info_lite'),
          ('/odom','/icp_odom'),
          ('/odom_info','/icp_odom_info'),
          ('/odom_local_scan_map','/icp_odom_local_scan_map'),
          ('/odom_filtered_input_scan','/icp_odom_filtered_input_scan')
    ]
    
    rtabmap_icp_odom = Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=parameters_icp,
            remappings=remappings_icp,
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", "warn"])
            
    # Include realsense camera launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch',
            'rs_launch.py'
        )),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'accel_fps': '250',
            'gyro_fps': '400',
            'unite_imu_method': '2',
            'publish_tf': 'true',
            'tf_publish_rate':'5.0',
            'initial_reset': 'true',
            'rgb_camera.profile': '848,480,15',
            'depth_module.profile': '848,480,15',
            'enable_sync': 'true',
            'hold_back_imu_for_frames': 'true',
            'enable_rgb': 'true',
            'enable_depth': 'true',
            'enable_pointcloud': 'true',
            'align_depth.enable':'true',
        }.items()
    )
    
    # Because of this issue: https://github.com/IntelRealSense/realsense-ros/issues/2564
    # Generate point cloud from not aligned depth            
    depth_to_pcd = Node(
            package='rtabmap_util', executable='point_cloud_xyz', name="depth_to_pcd_node", output='screen',
            parameters=[{'approx_sync':False}],
            remappings=[('depth/image',       '/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud',             '/camera/pcd')])
    
    # Generate aligned depth to color camera from the point cloud above       
    pcd_to_depthimage = Node(
            package='rtabmap_util', executable='pointcloud_to_depthimage', name="pcd_to_depth_node", output='screen',
            parameters=[{ 'decimation':2,
                          'fixed_frame_id':'camera_link',
                          'fill_holes_size':1}],
            remappings=[('camera_info', '/camera/color/camera_info'),
                        ('cloud',       '/camera/pcd'),
                        ('image_raw',   '/camera/realigned_depth_to_color/image_raw')])
    
    # Compute quaternion of the IMU                    
    imu_madgwick = Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'publish_tf': False}],
            remappings=[('/imu/data_raw', '/camera/imu')],
            arguments=["--ros-args", "--log-level", "warn"])  
   
    # The IMU frame is missing in TF tree, add it:
    imu_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']) 
            
    #Include Velodyne launch files

    # velodyne_driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('velodyne_driver'),
    #         'launch',
    #         'velodyne_driver_node-VLP16-launch.py'
    #     ))
    # )
    
    # velodyne_pointcloud_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('velodyne_pointcloud'),
    #         'launch',
    #         'velodyne_transform_node-VLP16-launch.py'
    #     ))
    # )

    velodyne_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('velodyne'),
            'launch',
            'velodyne-all-nodes-VLP16-launch.py'
        ))
    )
    
    #Msg converter node
    msgs_converter_node = Node(
        package='diablo_ctrl', 
        executable='msgs_converter_node',
        output='screen',
    )
    
    # Define the absolute path to ekf.yaml file
    ekf_params_file = '/home/risc-diablo1/diablo_ws/misc/ekf.yaml'
    
    #Include robot_localization
    # robot_localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('robot_localization'),
    #             'launch',
    #             'ekf.launch.py'
    #         )
    #     )
    # )

    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters= [{
		        'frame_id': 'base_link',
          	    'approx_sync': True, #For external odom (odometry filtered) it should be true
                'publish_tf':True,
                'subscribe_odom_info':False, #external odom used
          	    'wait_imu_to_init': True,
          	    'wait_for_transform': 0.2,
          	    'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_scan_cloud': True,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'RGBD/MarkerDetection': 'False',
                'Marker/CornerRefinementMethod': '1', #1: Subpixel
                'Marker/Dictionary': '3', #DICT_ARUCO_4X4_1000=3
                'Marker/Length': '0.2',
                'Marker/MaxDepthError': '0.01',
                'Marker/MaxRange': '10',
                'Marker/MinRange': '0',
                'Optimizer/Strategy': '1',
            	'Optimizer/Robust':'True',
	        	'Optimizer/Iterations':'10',
          	    'RGBD/OptimizeMaxError':'0',
          	    #'RGBD/OptimizeFromGraphEnd':'true',
                #'Rtabmap/StartNewMapOnLoopClosure':'true'
          	    'Rtabmap/DetectionRate':'1',
                'cloud_noise_filtering_radius':'0.1',
                'cloud_noise_filtering_min_neighbors':'10',
                #For a planer map (apartment)
                'Reg/Force3DoF': "true",
                #Parameters for ground/obstacles detection
                'Grid/GroundIsObstacle': "false",
                'Grid/NormalsSegmentation': "false",
		#'Grid/MinGroundHeight':"-0.25",
                #'Grid/MaxGroundAngle': "20", #Only if NormalSegmentation = True
		'Grid/MaxGroundHeight':"0.20", #should be set if normalSegmentation = False
		'Grid/MaxObstacleHeight':"0.8",
		    }],
            remappings=[
                        ('imu', '/imu/data'),
                        ('rgb/image', '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth/image', '/camera/realigned_depth_to_color/image_raw'),
                        ('scan_cloud', '/velodyne_points'),
                        #to use robot_localization
                        #('/odom','/odometry/filtered'),
                        #to use icp:
                        ('/odom','/icp_odom'),
            ],
            arguments=['-d', "--ros-args", "--log-level", "info"])
            
    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            #parameters=parameters_rgbd,
            #remappings=remappings_rgbd
            parameters=parameters_icp,
            remappings=[
                        ('scan_cloud', '/velodyne_points'),
                        ('imu', '/imu/data'),
                        ('rgb/image', '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth/image', '/camera/realigned_depth_to_color/image_raw'),
                        #('/odom','/odometry/filtered'),
                        #to use icp:
                        ('/odom','/icp_odom'),
                        ('/odom_info','/icp_odom_info'),
                        ('/odom_local_scan_map','/icp_odom_local_scan_map'),
                        ('/odom_filtered_input_scan','/icp_odom_filtered_input_scan'),
                        ('/odom_rgbd_image','/icp_odom_rgbd_image'),
                        ('/odom_last_frame','/icp_odom_last_frame'),
                        ('/odom_local_map','/icp_odom_local_map'),
                        ('/odom_info_lite','/icp_odom_info_lite')
            ],
            )
    
    #Enable only if using recorded data   
    depth_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_depth_frame'])
    
    depth_optical_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'camera_depth_frame', 'camera_depth_optical_frame']) 
            
    color_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.0', '0.015', '0', '-0.006', '0.000006', '0.002', 'camera_link', 'camera_color_frame'])
    
    color_optical_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0.015', '0', '-1.5707963', '0', '-1.5707963', 'camera_color_frame', 'camera_color_optical_frame']) 
            
    accel_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.012', '-0.006', '0.005', '0', '0', '0', 'camera_link', 'camera_accel_frame'])
    
    accel_optical_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.012', '-0.006', '0.005', '-1.5707963', '0', '-1.5707963', 'camera_accel_frame', 'camera_accel_optical_frame'])             

    gyro_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.012', '-0.006', '0.005', '0', '0', '0', 'camera_link', 'camera_gyro_frame'])
    
    gyro_optical_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.012', '-0.006', '0.005', '-1.5707963', '0', '-1.5707963', 'camera_gyro_frame', 'camera_gyro_optical_frame']) 

    # The IMU frame is missing in TF tree, add it:
    imu_static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']) 
    
    return LaunchDescription([

        use_sim_time_arg,
        # Velodyne Convert launch with a delay
        #   TimerAction(period=1.0, actions=[velodyne_driver_launch]),

        # Velodyne Driver launch with a delay
        #TimerAction(period=0.0, actions=[velodyne_pointcloud_launch]),
        velodyne_pointcloud_launch,

        # ICP Odometry with a delay
        TimerAction(period=5.0, actions=[rtabmap_icp_odom]),

        # Launch realsense camera
        TimerAction(period=1.0, actions=[realsense_launch]),
       
        # Launch other rgbd_odom requirements
        TimerAction(
            period=5.0,
            actions=[
                depth_to_pcd,
                pcd_to_depthimage,
                imu_madgwick,
                imu_static_tf,
            ]
        ),

        # RTAB-Map RGBD odom with a delay
        #TimerAction(period=10.0, actions=[rtabmap_rgbd_odom]),

        #TimerAction for 'msgs_converter_node'
        #TimerAction(period=5.0, actions=[msgs_converter_node]),

        #TimerAction for the 'robot_localization' ekf.launch.py
        #TimerAction(period=12.0, actions=[robot_localization_launch]),

        #rtabmap_slam,
        #rtabmap_viz,

        #Enable only if live data 
        robot_urdf_arg,
        robot_state_publisher_node,

        #Enable the following only if using recorded data
        #depth_static_tf,
        #depth_optical_static_tf,
        #color_static_tf,
        #color_optical_static_tf,
        #accel_static_tf,
        #accel_optical_static_tf,
        #gyro_static_tf,
        #gyro_optical_static_tf,      
    ])
