import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

	
    
	rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
            	get_package_share_directory('urdf_tutorial_r2d2'),
                'config',
                'lidar.rviz')]
        )
	lidar = Node(
            package='urdf_tutorial_r2d2',  # Replace with the name of your package
            executable='lidar_publisher_slam',  # Name of the static transform broadcaster script
            name='lidar_publisher_slam',
            output='screen'
        )
	slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'odom_frame': 'reference_point',
            'map_frame': 'map',
        }],
    )
	
	
	l_d = LaunchDescription([rviz, lidar,slam_toolbox])
	
	return l_d
