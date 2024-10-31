import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

	urdf_file = os.path.join(
		get_package_share_directory('urdf_tutorial_r2d2'),
        	'urdf',
        	'robot.urdf'  # Replace with the name of your URDF file
    	)
	
	
	robot_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        )
	joint_pub = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    
	rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
            	get_package_share_directory('urdf_tutorial_r2d2'),
                'config',
                'urdf_robot.rviz')]
        )
	static = Node(
            package='urdf_tutorial_r2d2',  # Replace with the name of your package
            executable='static',  # Name of the static transform broadcaster script
            name='static',
            output='screen'
        )
	
	dynamic = Node(
	    package='urdf_tutorial_r2d2',  # Replace with the name of your package
            executable='dynamic',  # Name of the static transform broadcaster script
            name='dynamic',
            output='screen'
	    )
	
	l_d = LaunchDescription([robot_pub, joint_pub, rviz, static, dynamic])
	
	return l_d
