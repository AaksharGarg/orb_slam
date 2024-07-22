#! /usr/bin/env python3
import xacro
from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # with_bridge = LaunchConfiguration('with_bridge')
    # argument to specify if bridge needs to be launched
    # arg_with_bridge = DeclareLaunchArgument('with_bridge', default_value='true',
                                            # description="Set true to bridge ROS 2 & Gz topics")
    #path to xacro file
    xacro_file=get_package_share_directory('robot_description')+'/urdf/robot.xacro'
    bridge_config=get_package_share_directory('robot_description')+'/config/bridge.yaml'
    world_path= get_package_share_directory("robot_description")+'/worlds/world.sdf'
    rviz_config= get_package_share_directory("robot_description")+'/config/robot.rviz'
    # Include the gazebo.launch.py file

    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']),launch_arguments={
                    'gz_args' : world_path + " -v 4"
                }.items())
    
    #publishing robot_state into topic robot_description
    robot_state=Node(package = 'robot_state_publisher',
                            executable = 'robot_state_publisher',
                            name='robot_state_publisher',
                            parameters = [{'robot_description': ParameterValue(Command( \
                                        ['xacro ', xacro_file,
                                        # ' kinect_enabled:=', "true",
                                        # ' lidar_enabled:=', "true",
                                        # ' camera_enabled:=', camera_enabled,
                                        ]), value_type=str)}]
                            )
    
    #spawn amr using the topic "/amr_description"
    robot_spawn=Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                    '-name', 'robot',
                    '-topic', '/robot_description',
                    "-allow_renaming", "true",
                    '-x', '0.0',  # Set the initial X position
                    '-y', '0.0',  # Set the initial Y position
                    '-z', '0.1' ,  # Set the initial Z position
                    '-Y', '0.0', # Set the Yaw
                    '-P', '0.0', #set the Pitch
                    '-R', '0.0' # Set the Roll
    ]
    )

    # parameter bridge node to brie different gz and tos 2 topics
    ros_gz_bridge = Node(package="ros_gz_bridge", 
                executable="parameter_bridge",
                parameters = [
                    {'config_file': bridge_config}],
                # condition=IfCondition(with_bridge)
                )
    
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time',
											default_value='true',
											description="Enable sim time from /clock")
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    

    
    return LaunchDescription([
        arg_use_sim_time,
        gazebo,
        robot_state,
        robot_spawn,
        ros_gz_bridge,
        rviz_node,
    ])