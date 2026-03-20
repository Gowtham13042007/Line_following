import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name='line_following_robot'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file=os.path.join(pkg_share,'urdf','one_robot.xacro')
    world_path=os.path.join(pkg_share,'worlds','line_track_world.sdf')

    robot_description_raw=xacro.process_file(xacro_file).toxml()
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
            launch_arguments={'world': world_path}.items()
        )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )


    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'one_robot',
        '-x', '0.0', 
        '-y', '0.0',  # Placed just above the bottom line
        '-z', '0.0', 
        '-Y', '-1.57'],
        output='screen'
    )

    line_follower_node = Node(
        package=pkg_name,
        executable='line_follower', 
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        line_follower_node
    ])