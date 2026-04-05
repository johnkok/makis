#!/usr/bin/env python3
"""
imu_node.py

Reads LSM303DLHC (accel + magnetometer) and L3GD20 (gyroscope) via I²C and
publishes:
  /imu/data  (sensor_msgs/Imu)                — accel + gyro + orientation estimate
  /imu/mag   (sensor_msgs/MagneticField)       — raw magnetometer

I²C addresses (default):
  LSM303DLHC accel : 0x19  (SA0 high)
  LSM303DLHC mag   : 0x1E
  L3GD20 gyro      : 0x6B  (SDO high)

Parameters
----------
i2c_bus          : int   default 1
accel_addr       : int   default 0x19
mag_addr         : int   default 0x1E
gyro_addr        : int   default 0x6B
publish_rate_hz  : float default 50.0
frame_id         : str   default 'imu_link'
"""

import struct
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

try:
    import smbus2
    _SMBUS_OK = True
except ImportError:
    _SMBUS_OK = False

# ── Register maps ─────────────────────────────────────────────

# LSM303DLHC Accelerometer
ACCEL_CTRL1  = 0x20   # ODR=100 Hz, all axes on : 0x57
ACCEL_OUT_XL = 0x28   # auto-increment when bit7=1 → 0xA8
ACCEL_SCALE  = 2.0    # ±2g  →  1 mg/LSB in high-res mode

# LSM303DLHC Magnetometer
MAG_CRA      = 0x00   # 0x14 → 30 Hz
MAG_CRB      = 0x01   # 0x20 → ±1.3 Gauss  (scale 1100 LSB/Gauss)
MAG_MR       = 0x02   # 0x00 → continuous
MAG_OUT_XH   = 0x03
MAG_XY_SCALE = 1.0 / 1100.0   # Gauss per LSB (XY)
MAG_Z_SCALE  = 1.0 / 980.0    # Gauss per LSB (Z)

# L3GD20 Gyroscope
GYRO_CTRL1   = 0x20   # 0x0F → 95 Hz, all axes
GYRO_OUT_XL  = 0x28   # auto-increment: 0xA8
GYRO_SCALE   = 8.75e-3 * (math.pi / 180.0)  # dps/LSB → rad/s  (250 dps mode)


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus',         1)
        self.declare_parameter('accel_addr',       0x19)
        self.declare_parameter('mag_addr',         0x1E)
        self.declare_parameter('gyro_addr',        0x6B)
        self.declare_parameter('publish_rate_hz',  50.0)
        self.declare_parameter('frame_id',         'imu_link')

        bus_n      = self.get_parameter('i2c_bus').value
        self._acc  = self.get_parameter('accel_addr').value
        self._mag  = self.get_parameter('mag_addr').value
        self._gyro = self.get_parameter('gyro_addr').value
        rate       = self.get_parameter('publish_rate_hz').value
        self._fid  = self.get_parameter('frame_id').value

        if not _SMBUS_OK:
            self.get_logger().error('smbus2 not installed; IMU disabled')
            return

        self._bus = smbus2.SMBus(bus_n)
        self._init_sensors()

        self._imu_pub = self.create_publisher(Imu,           '/imu/data', 10)
        self._mag_pub = self.create_publisher(MagneticField, '/imu/mag',  10)

        # Simple Mahony filter state
        self._q = [1.0, 0.0, 0.0, 0.0]  # w,x,y,z
        self._Ki = 0.01
        self._Kp = 2.0
        self._ex_int = self._ey_int = self._ez_int = 0.0

        self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info('IMU node started')

    # ── Sensor init ───────────────────────────────────────────
    def _init_sensors(self):
        # Accel: 100 Hz, ±2g, all axes
        self._bus.write_byte_data(self._acc, ACCEL_CTRL1, 0x57)
        # Accel high-res mode
        self._bus.write_byte_data(self._acc, 0x23, 0x08)  # CTRL_REG4: HR

        # Mag: 30 Hz, ±1.3 Gauss, continuous
        self._bus.write_byte_data(self._mag, MAG_CRA, 0x14)
        self._bus.write_byte_data(self._mag, MAG_CRB, 0x20)
        self._bus.write_byte_data(self._mag, MAG_MR,  0x00)

        # Gyro: 95 Hz, 250 dps, all axes
        self._bus.write_byte_data(self._gyro, GYRO_CTRL1, 0x0F)

    # ── I²C read helpers ──────────────────────────────────────
    def _read_accel(self):
        data = self._bus.read_i2c_block_data(self._acc, 0xA8, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        # High-res: 12-bit left-aligned in 16-bit register → shift 4
        g = 9.80665
        scale = (ACCEL_SCALE / 32768.0) * g
        return x * scale, y * scale, z * scale

    def _read_mag(self):
        data = self._bus.read_i2c_block_data(self._mag, MAG_OUT_XH, 6)
        # Order in LSM303DLHC: X_H, X_L, Z_H, Z_L, Y_H, Y_L
        xh, xl, zh, zl, yh, yl = data
        rx = struct.unpack('>h', bytes([xh, xl]))[0]
        rz = struct.unpack('>h', bytes([zh, zl]))[0]
        ry = struct.unpack('>h', bytes([yh, yl]))[0]
        return (rx * MAG_XY_SCALE * 1e-4,   # Gauss → Tesla
                ry * MAG_XY_SCALE * 1e-4,
                rz * MAG_Z_SCALE  * 1e-4)

    def _read_gyro(self):
        data = self._bus.read_i2c_block_data(self._gyro, 0xA8, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        return x * GYRO_SCALE, y * GYRO_SCALE, z * GYRO_SCALE

    # ── Mahony AHRS (simplified, no magnetometer fusion) ──────
    def _mahony_update(self, ax, ay, az, gx, gy, gz, dt):
        q0, q1, q2, q3 = self._q

        # Normalise accel
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            return
        ax /= norm; ay /= norm; az /= norm

        # Estimated gravity from quaternion
        vx = 2*(q1*q3 - q0*q2)
        vy = 2*(q0*q1 + q2*q3)
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3

        # Error (cross product)
        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # Integral feedback
        self._ex_int += ex * self._Ki * dt
        self._ey_int += ey * self._Ki * dt
        self._ez_int += ez * self._Ki * dt

        gx += self._Kp * ex + self._ex_int
        gy += self._Kp * ey + self._ey_int
        gz += self._Kp * ez + self._ez_int

        # Integrate quaternion
        gx *= 0.5 * dt; gy *= 0.5 * dt; gz *= 0.5 * dt
        q0 += -q1*gx - q2*gy - q3*gz
        q1 +=  q0*gx + q2*gz - q3*gy
        q2 +=  q0*gy - q1*gz + q3*gx
        q3 +=  q0*gz + q1*gy - q2*gx

        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self._q = [q0/norm, q1/norm, q2/norm, q3/norm]

    # ── Timer callback ────────────────────────────────────────
    _last_time = None

    def _timer_cb(self):
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        try:
            ax, ay, az = self._read_accel()
            mx, my, mz = self._read_mag()
            gx, gy, gz = self._read_gyro()
        except OSError as e:
            self.get_logger().warn(f'I2C read error: {e}', throttle_duration_sec=5.0)
            return

        self._mahony_update(ax, ay, az, gx, gy, gz, dt)
        q0, q1, q2, q3 = self._q

        stamp = now.to_msg()

        # ── Imu message ───────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = self._fid
        imu_msg.orientation.w = q0
        imu_msg.orientation.x = q1
        imu_msg.orientation.y = q2
        imu_msg.orientation.z = q3
        imu_msg.orientation_covariance    = [0.01]*9
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.angular_velocity_covariance    = [0.001]*9
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.linear_acceleration_covariance = [0.1]*9
        self._imu_pub.publish(imu_msg)

        # ── MagneticField message ─────────────────────────────
        mag_msg = MagneticField()
        mag_msg.header.stamp    = stamp
        mag_msg.header.frame_id = self._fid
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
        mag_msg.magnetic_field_covariance = [1e-6]*9
        self._mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
