#!/usr/bin/env python3
"""
arduino_bridge_node.py

Bridges the Arduino Uno ↔ ROS 2.

Subscriptions
-------------
/cmd_vel  (geometry_msgs/Twist)  → mecanum IK → Arduino CMD

Publications
------------
/odom             (nav_msgs/Odometry)
/tf               (base_footprint → odom)
/ps2/joy          (sensor_msgs/Joy)   raw PS2 joystick data
/hardware_status  (std_msgs/String)   INFO messages from Arduino

Parameters
----------
port        : serial port   default /dev/ttyACM0
baud        : serial baud   default 115200
wheel_sep_x : half wheelbase (front-back)  default 0.09  m
wheel_sep_y : half track width             default 0.125 m
wheel_radius: wheel radius                 default 0.0325 m
ticks_per_rev: encoder ticks per wheel rev default 1800
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import tf2_ros


class ArduinoBridgeNode(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('port',          '/dev/ttyACM0')
        self.declare_parameter('baud',           115200)
        self.declare_parameter('wheel_sep_x',    0.09)    # m (front-back half)
        self.declare_parameter('wheel_sep_y',    0.125)   # m (left-right half)
        self.declare_parameter('wheel_radius',   0.0325)  # m
        self.declare_parameter('ticks_per_rev',  1800)    # enc ticks / wheel turn
        self.declare_parameter('publish_tf',     True)

        port  = self.get_parameter('port').value
        baud  = self.get_parameter('baud').value
        self._r    = self.get_parameter('wheel_radius').value
        self._lx   = self.get_parameter('wheel_sep_x').value
        self._ly   = self.get_parameter('wheel_sep_y').value
        self._tpr  = self.get_parameter('ticks_per_rev').value
        self._pub_tf = self.get_parameter('publish_tf').value

        # metres per encoder tick
        self._m_per_tick = (2 * math.pi * self._r) / self._tpr

        # ── Serial ────────────────────────────────────────────
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)  # wait for Arduino reset
            self._ser.reset_input_buffer()
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial: {e}')
            raise

        # ── Odometry state ────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        # ── Publishers ────────────────────────────────────────
        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._odom_pub   = self.create_publisher(Odometry, '/odom', 10)
        self._joy_pub    = self.create_publisher(Joy, '/ps2/joy', qos)
        self._status_pub = self.create_publisher(String, '/hardware_status', 10)

        if self._pub_tf:
            self._tf_br = tf2_ros.TransformBroadcaster(self)

        # ── Subscriber ────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ── Serial reader thread ───────────────────────────────
        self._running = True
        self._read_thread = threading.Thread(target=self._serial_reader,
                                             daemon=True)
        self._read_thread.start()

    # ── cmd_vel callback → mecanum IK ─────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        L = self._lx + self._ly
        # Inverse mecanum kinematics  (rad/s → normalize to -255..255)
        # Max wheel speed at full command assumed = 1.0 m/s
        scale = 255.0 / (abs(vx) + abs(vy) + abs(omega) * L + 1e-9)
        scale = min(scale, 255.0)

        fl = int((vx - vy - omega * L) * scale)
        fr = int((vx + vy + omega * L) * scale)
        rl = int((vx + vy - omega * L) * scale)
        rr = int((vx - vy + omega * L) * scale)

        cmd = f'CMD:{fl}:{fr}:{rl}:{rr}\n'
        try:
            self._ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial reader loop ────────────────────────────────────
    def _serial_reader(self):
        while self._running:
            try:
                line = self._ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException:
                time.sleep(0.1)
                continue
            if not line:
                continue

            if line.startswith('ENC:'):
                self._handle_enc(line[4:])
            elif line.startswith('PS2:'):
                self._handle_ps2(line[4:])
            elif line.startswith('INFO:'):
                msg = String()
                msg.data = line[5:]
                self._status_pub.publish(msg)

    # ── ENC handler → odometry ────────────────────────────────
    def _handle_enc(self, payload: str):
        try:
            parts = payload.split(':')
            d_fl  = int(parts[0])
            d_fr  = int(parts[1])
            d_rl  = int(parts[2])
            d_rr  = int(parts[3])
            dt_ms = int(parts[4])
        except (ValueError, IndexError):
            return

        dt = dt_ms / 1000.0
        if dt <= 0.0:
            return

        mpt = self._m_per_tick
        L   = self._lx + self._ly

        # Wheel linear displacements (m)
        dfl = d_fl * mpt
        dfr = d_fr * mpt
        drl = d_rl * mpt
        drr = d_rr * mpt

        # Mecanum forward kinematics (robot body displacement)
        dx     = (dfl + dfr + drl + drr) / 4.0
        dy     = (-dfl + dfr + drl - drr) / 4.0
        dtheta = (-dfl + dfr - drl + drr) / (4.0 * L)

        # Integrate pose
        self._x     += dx * math.cos(self._theta) - dy * math.sin(self._theta)
        self._y     += dx * math.sin(self._theta) + dy * math.cos(self._theta)
        self._theta += dtheta

        vx    = dx / dt
        vy    = dy / dt
        omega = dtheta / dt

        now = self.get_clock().now().to_msg()
        q   = _euler_to_quat(0, 0, self._theta)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = omega

        # Diagonal covariances (tunable)
        pc = [0.01, 0, 0, 0, 0, 0,
              0, 0.01, 0, 0, 0, 0,
              0, 0, 1e6, 0, 0, 0,
              0, 0, 0, 1e6, 0, 0,
              0, 0, 0, 0, 1e6, 0,
              0, 0, 0, 0, 0, 0.03]
        odom.pose.covariance  = pc
        odom.twist.covariance = pc
        self._odom_pub.publish(odom)

        if self._pub_tf:
            tf = TransformStamped()
            tf.header.stamp    = now
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_footprint'
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self._tf_br.sendTransform(tf)

    # ── PS2 handler ───────────────────────────────────────────
    def _handle_ps2(self, payload: str):
        try:
            parts = payload.split(':')
            lx   = int(parts[0])
            ly   = int(parts[1])
            rx   = int(parts[2])
            ry   = int(parts[3])
            btns = int(parts[4])
        except (ValueError, IndexError):
            return

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        # Normalise sticks to -1..1
        joy.axes = [
            (lx - 128) / 128.0,
            (ly - 128) / 128.0,
            (rx - 128) / 128.0,
            (ry - 128) / 128.0,
        ]
        # Expand 16-bit button bitmask to individual float (1.0 = pressed)
        joy.buttons = [(btns >> i) & 1 for i in range(16)]
        self._joy_pub.publish(joy)

    def destroy_node(self):
        self._running = False
        if self._ser.is_open:
            self._ser.close()
        super().destroy_node()


# ── Quaternion helper ─────────────────────────────────────────
def _euler_to_quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5);  sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
