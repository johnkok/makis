"""
bringup.launch.py

Starts hardware drivers:
  - robot_state_publisher  (URDF → TF)
  - arduino_bridge_node    (serial + odometry)
  - imu_node               (LSM303DLHC + L3GD20)
  - hls_lfcd_lds_driver    (LiDAR)
  - v4l2_camera            (Pi Camera)
  - robot_localization EKF (fuse odom + IMU)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('tank_bringup')
    desc_dir    = get_package_share_directory('tank_description')
    urdf_path   = os.path.join(desc_dir, 'urdf', 'tank.urdf.xacro')

    # ── Args ─────────────────────────────────────────────────
    arduino_port = LaunchConfiguration('arduino_port')
    lidar_port   = LaunchConfiguration('lidar_port')
    i2c_bus      = LaunchConfiguration('i2c_bus')

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port',   default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('i2c_bus',      default_value='1'),

        # ── URDF / TF ──────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': False,
            }],
        ),

        # ── Arduino bridge (motors + encoders) ─────────────
        Node(
            package='tank_hardware',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{
                'port':           arduino_port,
                'baud':           115200,
                'wheel_sep_x':    0.09,
                'wheel_sep_y':    0.125,
                'wheel_radius':   0.0325,
                'ticks_per_rev':  1800,
                'publish_tf':     True,
            }],
            output='screen',
        ),

        # ── IMU ────────────────────────────────────────────
        Node(
            package='tank_imu',
            executable='imu_node',
            name='imu_node',
            parameters=[{
                'i2c_bus':        i2c_bus,
                'accel_addr':     0x19,
                'mag_addr':       0x1E,
                'gyro_addr':      0x6B,
                'publish_rate_hz': 50.0,
                'frame_id':       'imu_link',
            }],
            output='screen',
        ),

        # ── LiDAR ──────────────────────────────────────────
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='lidar',
            parameters=[{
                'port':       lidar_port,
                'frame_id':   'laser',
            }],
            output='screen',
        ),

        # ── Pi Camera ──────────────────────────────────────
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size':   [640, 480],
                'camera_frame_id': 'camera_optical_frame',
            }],
            output='screen',
        ),

        # ── EKF sensor fusion ──────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            parameters=[
                os.path.join(bringup_dir, 'config', 'ekf.yaml'),
            ],
            remappings=[('odometry/filtered', '/odometry/filtered')],
            output='screen',
        ),
    ])
