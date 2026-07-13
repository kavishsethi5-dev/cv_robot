#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    pkg_name = 'cv_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to your URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'cv_robot.urdf')
    #adding materials path
    materials_path = os.path.join(pkg_share, 'materials')
    #adding camera path
    camera_sdf_file = os.path.join(pkg_share,'overhead_cam.sdf')
    #world file
    world_file = os.path.join(pkg_share, 'worlds', 'cv_world.sdf')

    # Read the URDF XML data
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # 1. Node: robot_state_publisher (Broadcasts /robot_description)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 2. Node: RViz2 (Visualization)
    #Removed

    # 3. Include: Modern Gazebo Harmonic Simulator Starter
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        # Opens the simulator running an empty world layout immediately (-r) ,-p for paused state
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 4. Node: Spawner (Reads from the /robot_description topic and builds it in Gazebo)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-z', '0.1', # (Spawn ~ 10cm above ground to prevent clipping floor physics)
            '-x', '0',  
            '-y', '0'   
        ],
        output='screen'
    )
    # 5. Node: Spawner for the camera (Reads from the camera URDF file and builds it in Gazebo)
    spawn_camera_node = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'overhead_camera',   # different name, not 'my_robot'
        '-file', camera_sdf_file, 
        '-x', '0',
        '-y', '0',
        '-z', '20',
        '-R', '0',
        '-P', '1.5708',
        '-Y', '0',                     # your URD
    ],
    output='screen'
    )
   
    Bridge_node=Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': os.path.join(pkg_share, 'config', 'gz_bridge.yaml')}],
    #I didnt really understand the config and parameters
    output='screen'
    
    )
    #ros2 run ros2_aruco aruco_node --ros-args -p marker_size:=1.0 -p aruco_di:=/camera/camera_info
    #check with ros2 param list
    Aruco_Node=Node(
    package= 'ros2_aruco',
    executable= 'aruco_node',
    parameters=[{'marker_size' : 1.0,
                 'aruco_dictionary_id' : 'DICT_5X5_250',
                 'image_topic':'/camera/image',
                 'camera_info_topic' : '/camera/camera_info'}],
    output='screen'
    )


    # Return the entire system execution stack
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[materials_path + '/scripts:' + materials_path + '/textures']
        ),
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
        spawn_camera_node,
        Bridge_node,
        Aruco_Node
    ])

