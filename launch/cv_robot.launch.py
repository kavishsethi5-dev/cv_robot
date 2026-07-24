#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():

    pkg_name = 'cv_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to your URDF file(old way of doing it, now we are using xacro)
    #urdf_file = os.path.join(pkg_share, 'urdf', 'cv_robot.urdf')
    # Path to your assembled xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'system.urdf.xacro')
    #adding materials path
    materials_path = os.path.join(pkg_share, 'materials')
    #adding camera path
    camera_sdf_file = os.path.join(pkg_share,'overhead_cam.sdf')
    #world file
    world_file = os.path.join(pkg_share, 'worlds', 'cv_world.sdf')
    box_file = os.path.join(pkg_share, 'urdf', 'box.urdf')

    ## Read the URDF XML data
    #with open(urdf_file, 'r') as f:
        #robot_description = f.read()

    # Process xacro -> expanded URDF string
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
   
    # 1. 

    use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use Gazebo simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    #Node: robot_state_publisher (Broadcasts /robot_description)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
        {'use_sim_time': use_sim_time}], # tells the node to use the simulation clock(self get clock now, this has to be  bridged also in the gz_bridge.yaml file)
        output='screen'
    )

    # 2. 
    
    

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
        '-z', '10',
        '-R', '0',
        '-P', '1.5708',
        '-Y', '0',                     # your URD
    ],
    output='screen'
    )
    spawn_box_node = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'box',   # different name, not 'my_robot'
        '-file', box_file, 
        '-x', '3',
        '-y', '3',
        '-z', '0.1',
        '-R', '0',
        '-P', '0',
        '-Y', '0', 
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
    #check with ros2 param list, 
    #this is for the robot
    Aruco_Node=Node(
    package= 'ros2_aruco_switchable', #normally ros2_aruco but I made an edited clone repo
    executable= 'low_res_aruco_node',
    parameters=[{'marker_size' : 1.0,
                 'aruco_dictionary_id' : 'DICT_5X5_250',
                 'image_topic':'/camera/image', #renamed from /camera/image_raw to /camera/image in the camera sdf file
                 'camera_info_topic' : '/camera/camera_info',
                 'use_sim_time': use_sim_time}],
    output='screen'
    )
    #for the box
    Aruco_Node_box=Node(
    package='ros2_aruco_switchable', #normally ros2_aruco but I made an edited clone repo
    executable='aruco_node',
    name='aruco_node_box',
    remappings=[
        ('/aruco_markers', '/aruco_markers_box'),
        ('/aruco_poses', '/aruco_poses_box'),
    ], #apparentely ros2_aruco doesn't auto-namespace outputs based on node name
    parameters=[{'marker_size': 0.1,
                 'aruco_dictionary_id': 'DICT_4X4_50',
                 'image_topic': '/camera/image',
                 'camera_info_topic': '/camera/camera_info',
                 'use_sim_time': use_sim_time}],
    output='screen' 
)
    #output screen is helpful for debugging
      
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # spawn arm_controller AFTER jsb exits successfully
    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # spawn fin_controller AFTER jsb exits successfully
    spawn_fin= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fin_controller'],
        output='screen'
    )
    # delay the spawning after jsb, we only need to add this in the launch description
    delay_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_arm, spawn_fin],
        )
    )


    # Return the entire system execution stack
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[materials_path + '/scripts:' + materials_path + '/textures']
        ),
        use_sim_time_arg, #DeclareLaunchArgument needs to be first (or at least earlier than any action that resolves the corresponding LaunchConfiguration) 
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
        spawn_camera_node,
        spawn_box_node,
        spawn_jsb,
        delay_arm_after_jsb,
        Bridge_node,
        Aruco_Node,
        Aruco_Node_box
    ])

