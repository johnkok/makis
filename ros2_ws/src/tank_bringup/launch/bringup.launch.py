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
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('tank_bringup')
    desc_dir    = get_package_share_directory('tank_description')
    urdf_path   = os.path.join(desc_dir, 'urdf', 'tank.urdf.xacro')
    default_arduino_port = os.environ.get('ARDUINO_PORT', '/dev/ttyUSB0')
    default_lidar_port = os.environ.get('LIDAR_PORT', '/dev/ttyAMA0')
    default_i2c_bus = os.environ.get('I2C_BUS', '1')
    default_accel_addr = int(os.environ.get('IMU_ACCEL_ADDR', '0x19'), 0)
    default_mag_addr = int(os.environ.get('IMU_MAG_ADDR', '0x1E'), 0)
    default_gyro_addr = int(os.environ.get('IMU_GYRO_ADDR', '0x69'), 0)
    default_use_gyro = os.environ.get('IMU_USE_GYRO', 'true').lower() in ('1', 'true', 'yes', 'on')
    i2c_device = f'/dev/i2c-{default_i2c_bus}'

    # ── Args ─────────────────────────────────────────────────
    arduino_port = LaunchConfiguration('arduino_port')
    lidar_port   = LaunchConfiguration('lidar_port')
    i2c_bus      = LaunchConfiguration('i2c_bus')

    actions = [
        DeclareLaunchArgument('arduino_port', default_value=default_arduino_port),
        DeclareLaunchArgument('lidar_port',   default_value=default_lidar_port),
        DeclareLaunchArgument('i2c_bus',      default_value=default_i2c_bus),

        # ── URDF / TF ──────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro', ' ', urdf_path]),
                    value_type=str,
                ),
                'use_sim_time': False,
            }],
        ),
    ]

    if os.path.exists(default_arduino_port):
        actions.append(
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
            )
        )
    else:
        actions.append(LogInfo(msg=f'Skipping Arduino bridge; device not found: {default_arduino_port}'))

    if os.path.exists(i2c_device):
        actions.append(
            Node(
                package='tank_imu',
                executable='imu_node',
                name='imu_node',
                parameters=[{
                    'i2c_bus':        i2c_bus,
                    'accel_addr':     default_accel_addr,
                    'mag_addr':       default_mag_addr,
                    'gyro_addr':      default_gyro_addr,
                    'use_gyro':       default_use_gyro,
                    'publish_rate_hz': 50.0,
                    'frame_id':       'imu_link',
                }],
                output='screen',
            )
        )
    else:
        actions.append(LogInfo(msg=f'Skipping IMU node; device not found: {i2c_device}'))

    if os.path.exists(default_lidar_port):
        actions.append(
            Node(
                package='hls_lfcd_lds_driver',
                executable='hlds_laser_publisher',
                name='lidar',
                parameters=[{
                    'port':       lidar_port,
                    'frame_id':   'laser',
                }],
                output='screen',
            )
        )
    else:
        actions.append(LogInfo(msg=f'Skipping LiDAR node; device not found: {default_lidar_port}'))

    if os.path.exists('/dev/video0'):
        actions.append(
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
            )
        )
    else:
        actions.append(LogInfo(msg='Skipping camera node; device not found: /dev/video0'))

    actions.append(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            parameters=[
                os.path.join(bringup_dir, 'config', 'ekf.yaml'),
            ],
            remappings=[('odometry/filtered', '/odometry/filtered')],
            output='screen',
        )
    )

    return LaunchDescription(actions)
