from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
import os


def generate_launch_description():

    # =========================
    # 📁 PATH
    # =========================
    workspace_dir = '/data/itina99/Progetti/spotSDK-autonomousMission'
    odom_to_tf_script = os.path.join(workspace_dir, 'odom_to_tf.py')
    wait_for_map_script = os.path.join(workspace_dir, 'wait_for_map.py')
    rviz_config_file = os.path.join(workspace_dir, 'RvizConfig', 'spotConfig.rviz')

    # Bridge side fisheye cameras from Gazebo topics to ROS topics.
    side_camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/spot/camera/left_fisheye_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/left_fisheye_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/right_fisheye_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/right_fisheye_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r', '/model/spot/camera/left_fisheye_image:=/spot/camera/left_fisheye/image_raw',
            '-r', '/model/spot/camera/left_fisheye_image/camera_info:=/spot/camera/left_fisheye/camera_info',
            '-r', '/model/spot/camera/right_fisheye_image:=/spot/camera/right_fisheye/image_raw',
            '-r', '/model/spot/camera/right_fisheye_image/camera_info:=/spot/camera/right_fisheye/camera_info',
        ],
        output='screen'
    )

    # =========================
    # 🔵 TF STATICI (FIX + DELAY)
    # =========================

    tf_camera_frontleft = TimerAction(
        period=2.0,
        actions=[Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_camera_frontleft',
            arguments=['0','0','0','0','0','0',
                       'camera_frontleft',
                       'spot/camera_frontleft/frontleft_fisheye_image']
        )]
    )

    tf_camera_frontright = TimerAction(
        period=2.0,
        actions=[Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_camera_frontright',
            arguments=['0','0','0','0','0','0',
                       'camera_frontright',
                       'spot/camera_frontright/frontright_fisheye_image']
        )]
    )

    tf_camera_back = TimerAction(
        period=2.0,
        actions=[Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_camera_back',
            arguments=['0','0','0','0','0','0',
                       'camera_back',
                       'spot/camera_back/back_fisheye_image']
        )]
    )

    # 🔵 LIDAR (usa solo se necessario)
    tf_lidar = TimerAction(
        period=2.0,
        actions=[Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar',
            arguments=['0','0','0','0','0','0',
                       'lidar_link',
                       'spot/lidar']
        )]
    )

    # =========================
    # 🔵 SLAM TOOLBOX
    # =========================

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

    # =========================
    # 🔵 ODOM → TF
    # =========================

    odom_to_tf_node = ExecuteProcess(
        cmd=[
            'python3', odom_to_tf_script,
            '--ros-args',
            '-p', 'use_sim_time:=true'
        ],
        cwd=workspace_dir,
        output='screen'
    )

    # =========================
    # 🔵 RVIZ
    # =========================

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # =========================
    # 🔵 WAIT FOR MAP
    # =========================

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

    # =========================
    # 🔵 EVENT FLOW
    # =========================

    start_slam_after_odom = RegisterEventHandler(
        OnProcessStart(
            target_action=odom_to_tf_node,
            on_start=[slam_toolbox_node]
        )
    )

    start_wait_for_map_after_slam = RegisterEventHandler(
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

    # =========================
    # 🚀 LAUNCH COMPLETO
    # =========================

    return LaunchDescription([

        # side camera bridge
        side_camera_bridge,

        # TF FIX (ritardati per evitare race condition)
        tf_camera_frontleft,
        tf_camera_frontright,
        tf_camera_back,
        tf_lidar,

        # pipeline principale
        odom_to_tf_node,
        start_slam_after_odom,
        start_wait_for_map_after_slam,
        start_rviz_after_map
    ])