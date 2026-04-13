from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Definiamo i percorsi
    workspace_dir = '/data/itina99/Progetti/spotSDK-autonomousMission'
    odom_to_tf_script = os.path.join(workspace_dir, 'odom_to_tf.py')
    wait_for_map_script = os.path.join(workspace_dir, 'wait_for_map.py')
    rviz_config_file = os.path.join(workspace_dir, 'RvizConfig', 'spotConfig.rviz')

    # Nodo per SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': 'map',
            'odom_frame': 'odom_spot',
            'base_frame': 'base_link'
        }],
        remappings=[
            ('scan', '/spot/lidar/scan'),
            ('/scan', '/spot/lidar/scan')
        ]
    )

    # Bridge odometria -> TF per pubblicare odom_spot -> base_link
    odom_to_tf_node = ExecuteProcess(
        cmd=[
            'python3', odom_to_tf_script,
            '--ros-args',
            '-p', 'use_sim_time:=true'
        ],
        cwd=workspace_dir,
        output='screen'
    )

    # Nodo RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Processo di gating: continua solo quando /map e' realmente disponibile
    wait_for_map_process = ExecuteProcess(
        cmd=[
            'python3', wait_for_map_script,
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'map_topic:=/map',
            '-p', 'timeout_sec:=0.0'
        ],
        cwd=workspace_dir,
        output='screen'
    )

    # Avvio in sequenza: odom_to_tf -> slam_toolbox -> rviz2
    start_slam_after_odom = RegisterEventHandler(
        OnProcessStart(
            target_action=odom_to_tf_node,
            on_start=[slam_toolbox_node]
        )
    )

    start_rviz_after_slam = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[wait_for_map_process]
        )
    )

    start_rviz_after_map = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_process,
            on_exit=[rviz2_node]
        )
    )

    return LaunchDescription([
        odom_to_tf_node,
        start_slam_after_odom,
        start_rviz_after_slam,
        start_rviz_after_map
    ])
