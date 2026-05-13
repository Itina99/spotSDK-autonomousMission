from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
import os


def generate_launch_description():

    # =========================
    # 📁 PATH
    # =========================
    # Ottieni la directory del file di lancio (portatile su qualsiasi PC)
    workspace_dir = os.path.dirname(os.path.abspath(__file__))
    odom_to_tf_script = os.path.join(workspace_dir, 'odom_to_tf.py')
    wait_for_map_script = os.path.join(workspace_dir, 'wait_for_map.py')
    rviz_config_file = os.path.join(workspace_dir, 'RvizConfig', 'spotConfig.rviz')

    # =========================
    # 🎥 CAMERA + DEPTH BRIDGE
    # =========================
    camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',

            # RGB
            '/model/spot/camera/frontleft_fisheye_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/frontright_fisheye_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/left_fisheye_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/right_fisheye_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/back_fisheye_image@sensor_msgs/msg/Image@gz.msgs.Image',

            # RGB CAMERA INFO
            '/model/spot/camera/frontleft_fisheye_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/frontright_fisheye_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/left_fisheye_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/right_fisheye_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/back_fisheye_image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

            # DEPTH
            '/model/spot/camera/frontleft_depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/frontright_depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/left_depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/right_depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/camera/back_depth@sensor_msgs/msg/Image@gz.msgs.Image',

            # DEPTH CAMERA INFO
            '/model/spot/camera/frontleft_depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/frontright_depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/left_depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/right_depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/spot/camera/back_depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

            # THERMAL
            '/model/spot/thermal_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/spot/thermal_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

            '--ros-args',

            # REMAP RGB
            '-r', '/model/spot/camera/frontleft_fisheye_image:=/spot/camera/frontleft/image_raw',
            '-r', '/model/spot/camera/frontright_fisheye_image:=/spot/camera/frontright/image_raw',
            '-r', '/model/spot/camera/left_fisheye_image:=/spot/camera/left/image_raw',
            '-r', '/model/spot/camera/right_fisheye_image:=/spot/camera/right/image_raw',
            '-r', '/model/spot/camera/back_fisheye_image:=/spot/camera/back/image_raw',

            # REMAP RGB CAMERA INFO
            '-r', '/model/spot/camera/frontleft_fisheye_image/camera_info:=/spot/camera/frontleft/camera_info',
            '-r', '/model/spot/camera/frontright_fisheye_image/camera_info:=/spot/camera/frontright/camera_info',
            '-r', '/model/spot/camera/left_fisheye_image/camera_info:=/spot/camera/left/camera_info',
            '-r', '/model/spot/camera/right_fisheye_image/camera_info:=/spot/camera/right/camera_info',
            '-r', '/model/spot/camera/back_fisheye_image/camera_info:=/spot/camera/back/camera_info',

            # REMAP DEPTH
            '-r', '/model/spot/camera/frontleft_depth:=/spot/camera/frontleft/depth',
            '-r', '/model/spot/camera/frontright_depth:=/spot/camera/frontright/depth',
            '-r', '/model/spot/camera/left_depth:=/spot/camera/left/depth',
            '-r', '/model/spot/camera/right_depth:=/spot/camera/right/depth',
            '-r', '/model/spot/camera/back_depth:=/spot/camera/back/depth',

            # REMAP DEPTH CAMERA INFO
            '-r', '/model/spot/camera/frontleft_depth/camera_info:=/spot/camera/frontleft/depth/camera_info',
            '-r', '/model/spot/camera/frontright_depth/camera_info:=/spot/camera/frontright/depth/camera_info',
            '-r', '/model/spot/camera/left_depth/camera_info:=/spot/camera/left/depth/camera_info',
            '-r', '/model/spot/camera/right_depth/camera_info:=/spot/camera/right/depth/camera_info',
            '-r', '/model/spot/camera/back_depth/camera_info:=/spot/camera/back/depth/camera_info',

            # REMAP THERMAL
            '-r', '/model/spot/thermal_camera:=/spot/camera/thermal/image_raw',
            '-r', '/model/spot/thermal_camera/camera_info:=/spot/camera/thermal/camera_info',
        ],
        output='screen'
    )

    # =========================
    # 📡 LIDAR BRIDGE
    # =========================
    lidar_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/spot/lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '--ros-args',
            '-r', '/model/spot/lidar:=/spot/lidar/points'
        ],
        output='screen'
    )

    # =========================
    # 🔄 POINTCLOUD → LASERSCAN
    # =========================
    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_scan',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 0.2,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 10.0
        }],
        remappings=[
            ('cloud_in', '/spot/lidar/points'),
            ('scan', '/spot/lidar/scan')
        ],
        output='screen'
    )

    # =========================
    # 🧭 IMU BRIDGE
    # =========================
    imu_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/spot/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '--ros-args',
            '-r', '/model/spot/imu:=/spot/imu'
        ],
        output='screen'
    )

    # =========================
    # 🔵 TF STATICI
    # =========================
    tf_lidar = TimerAction(
        period=2.0,
        actions=[Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar',
            arguments=['0','0','0','0','0','0',
                       'base_link',
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
            ('scan', '/spot/lidar/scan')
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
    # 🚀 LAUNCH
    # =========================
    return LaunchDescription([

        camera_bridge,
        lidar_bridge,
        imu_bridge,
        pointcloud_to_scan,

        tf_lidar,

        odom_to_tf_node,
        start_slam_after_odom,
        start_wait_for_map_after_slam,
        start_rviz_after_map
    ])