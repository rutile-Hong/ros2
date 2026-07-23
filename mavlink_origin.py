#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from pymavlink import mavutil


class ExternalPositionPublisher(Node):

    def __init__(self):
        super().__init__('external_position_publisher')

        # ==========================================================
        # DDS publisher: computer -> ArduPilot
        # ==========================================================
        self.tf_publisher = self.create_publisher(
            TFMessage,
            '/ap/tf',
            10
        )

        # ==========================================================
        # Local position in ROS ENU coordinates
        #
        # x: positive forward/east, depending on your odom definition
        # y: positive left/north
        # z: positive upward
        #
        # This local position begins at (0, 0, 0).
        # ==========================================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # ==========================================================
        # Geographic EKF origin corresponding to local (0, 0, 0)
        #
        # Use the approximate latitude, longitude, and MSL altitude
        # of your test location.
        # ==========================================================
        self.origin_latitude_deg = 30.6187000
        self.origin_longitude_deg = -96.3365000
        self.origin_altitude_m = 100.0

        # ==========================================================
        # MAVLink connection
        #
        # IMPORTANT:
        # This port must be different from the port used by the
        # Micro XRCE-DDS agent.
        # ==========================================================
        self.mavlink_port = '/dev/ttyUSB0'
        self.mavlink_baud = 115200

        self.mavlink = mavutil.mavlink_connection(
            self.mavlink_port,
            baud=self.mavlink_baud,
            source_system=255,
            source_component=190
        )

        self.connect_mavlink()
        self.set_ekf_origin()
        self.verify_ekf_origin()

        # Publish TF at 20 Hz.
        self.publish_rate_hz = 20.0
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_position
        )

        self.publish_counter = 0

        self.get_logger().info(
            'External-position publisher started at 20 Hz'
        )

    def connect_mavlink(self):
        """Wait for a MAVLink heartbeat from ArduPilot."""

        self.get_logger().info(
            f'Waiting for MAVLink heartbeat on '
            f'{self.mavlink_port}...'
        )

        heartbeat = self.mavlink.wait_heartbeat(timeout=15)

        if heartbeat is None:
            raise RuntimeError(
                f'No MAVLink heartbeat received on '
                f'{self.mavlink_port}'
            )

        self.get_logger().info(
            'MAVLink heartbeat received: '
            f'system={self.mavlink.target_system}, '
            f'component={self.mavlink.target_component}'
        )

    def set_ekf_origin(self):
        """Set the geographic coordinate corresponding to local 0,0,0."""

        latitude_int = round(
            self.origin_latitude_deg * 1.0e7
        )

        longitude_int = round(
            self.origin_longitude_deg * 1.0e7
        )

        altitude_mm = round(
            self.origin_altitude_m * 1000.0
        )

        # time_usec = 0 is acceptable here.
        self.mavlink.mav.set_gps_global_origin_send(
            self.mavlink.target_system,
            latitude_int,
            longitude_int,
            altitude_mm,
            0
        )

        self.get_logger().info(
            'SET_GPS_GLOBAL_ORIGIN sent: '
            f'latitude={self.origin_latitude_deg:.7f}, '
            f'longitude={self.origin_longitude_deg:.7f}, '
            f'altitude={self.origin_altitude_m:.2f} m MSL'
        )

    def verify_ekf_origin(self):
        """Wait briefly for ArduPilot to report its EKF origin."""

        self.get_logger().info(
            'Waiting for GPS_GLOBAL_ORIGIN confirmation...'
        )

        timeout_time = time.monotonic() + 5.0

        while time.monotonic() < timeout_time:

            message = self.mavlink.recv_match(
                type='GPS_GLOBAL_ORIGIN',
                blocking=True,
                timeout=1.0
            )

            if message is None:
                continue

            latitude_deg = message.latitude / 1.0e7
            longitude_deg = message.longitude / 1.0e7
            altitude_m = message.altitude / 1000.0

            self.get_logger().info(
                'EKF origin confirmed: '
                f'latitude={latitude_deg:.7f}, '
                f'longitude={longitude_deg:.7f}, '
                f'altitude={altitude_m:.2f} m MSL'
            )

            return

        self.get_logger().warning(
            'No GPS_GLOBAL_ORIGIN confirmation was received. '
            'The origin may already have been set before this script started.'
        )

    def publish_position(self):
        """Publish local external-navigation position to ArduPilot."""

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # These frame names must remain exactly as shown.
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = float(self.current_x)
        transform.transform.translation.y = float(self.current_y)
        transform.transform.translation.z = float(self.current_z)

        # Identity quaternion:
        # roll = 0, pitch = 0, yaw = 0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        tf_message = TFMessage()
        tf_message.transforms = [transform]

        self.tf_publisher.publish(tf_message)

        self.publish_counter += 1

        # Log approximately once per second.
        if self.publish_counter % int(self.publish_rate_hz) == 0:
            self.get_logger().info(
                'External position sent: '
                f'x={self.current_x:.3f} m, '
                f'y={self.current_y:.3f} m, '
                f'z={self.current_z:.3f} m'
            )


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = ExternalPositionPublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\nExternal-position publisher stopped.')

    except Exception as error:
        print(f'External-position publisher failed: {error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()