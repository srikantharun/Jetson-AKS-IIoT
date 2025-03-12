#!/usr/bin/env python3
"""
Launch file for SIWVS Gazebo simulation with ROS 2 control
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Path to package directories
    siwvs_bringup_dir = get_package_share_directory('siwvs_bringup')
    siwvs_control_dir = get_package_share_directory('siwvs_control')
    
    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_model = LaunchConfiguration('robot_model', default='inventory_arm')
    world_file = LaunchConfiguration('world_file', default='shelf_inventory.world')
    
    # Get URDF file
    urdf_file = os.path.join(
        siwvs_control_dir,
        'urdf',
        robot_model.perform(None) + '.urdf.xacro'
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(siwvs_bringup_dir, 'worlds', world_file.perform(None)),
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot node
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_model.perform(None),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
   
   # Add camera node
   camera_node = Node(
        package='siwvs_vision',
        executable='camera_node.py',
        name='camera_node',
        parameters=[{
           'use_sim_time': use_sim_time,
           'camera_topic': '/camera/image_raw',
           'camera_info_topic': '/camera/camera_info',
           'frame_rate': 15.0
        }],
        output='screen'
    )

    # Add product detector node
    product_detector_node = Node(
        package='siwvs_vision',
           executable='product_detector.py',
           name='product_detector',
           parameters=[{
               'use_sim_time': use_sim_time,
               'model_path': os.path.join(get_package_share_directory('siwvs_vision'), 
                                 'models/product_detection/model.pb'),
               'label_path': os.path.join(get_package_share_directory('siwvs_vision'),
                                 'models/product_detection/labels.txt'),
               'confidence_threshold': 0.7
        }],
        output='screen'
    )
 
    # Add inventory manager node
    inventory_manager_node = Node(
        package='siwvs_inventory',
        executable='inventory_manager.py',
        name='inventory_manager',
        parameters=[{
           'use_sim_time': use_sim_time,
           'database_path': '/data/inventory.db',
           'update_rate': 1.0
        }],
        output='screen'
    )
 
    # Controller spawner
    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ]
    )
    
    # Arm controller spawner
    arm_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager'
        ]
    )
    
    # Weight sensor nodes for each sensor in the shelf
    weight_sensor_nodes = []
    
    # Create 3x3 grid of weight sensors
    # Create 3x3 grid of weight sensors (3 rows, 3 columns)
    for row in range(1, 4):
        for col in range(1, 4):
            sensor_id = f'sensor_{row}_{col}'
            weight_sensor_nodes.append(
                Node(
                    package='siwvs_control',
                    executable='weight_sensor_node.py',
                    name=f'weight_sensor_{row}_{col}',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'sensor_id': sensor_id,
                        'shelf_id': 'shelf_01',
                        'row': row,
                        'column': col,
                        'config_file': '/config/sensor_config.yaml'
                    }],
                    output='screen'
                )
            )
    
    # RViz
    rviz_config_path = os.path.join(siwvs_bringup_dir, 'rviz', 'siwvs.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # tf_plugin for publishing transforms in Gazebo
    tf_plugin_node = Node(
        package='gazebo_tf_publisher',
        executable='gazebo_tf_publisher_node',
        name='gazebo_tf_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ROS Bridge server for web interface
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 9090
        }],
        output='screen'
    )
    
    # Create LaunchDescription
    ld = LaunchDescription()
    
    # Add all nodes and processes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robot_node)
    ld.add_action(camera_node)
    ld.add_action(product_detector_node)
    ld.add_action(inventory_manager_node)
 
    # Add controller spawners
    event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[controller_spawner_node]
        )
    )
    ld.add_action(event_handler)
    
    event_handler_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner_node,
            on_exit=[arm_controller_spawner_node]
        )
    )
    ld.add_action(event_handler_arm)
    
    # Add weight sensor nodes
    for node in weight_sensor_nodes:
        ld.add_action(node)
    
    # Add tf plugin
    ld.add_action(tf_plugin_node)
    
    # Add RViz
    ld.add_action(rviz_node)
    
    # Add ROS Bridge
    ld.add_action(rosbridge_node)
    
    return ld
