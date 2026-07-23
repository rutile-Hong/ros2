#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node

from pymavlink import mavutil


class MAVLinkModeMonitor(Node):

    def __init__(self):
        super().__init__("mavlink_mode_monitor")

        self.serial_port = "/dev/ttyUSB0"
        self.baud_rate = 115200

        self.mavlink_connection = None
        self.previous_mode = None

        self.get_logger().info(
            f"Connecting to MAVLink on "
            f"{self.serial_port} at {self.baud_rate} baud..."
        )

        try:
            self.mavlink_connection = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baud_rate,
                autoreconnect=True,
            )

        except Exception as error:
            self.get_logger().error(
                f"Failed to open MAVLink connection: {error}"
            )
            raise

        self.listener_thread = threading.Thread(
            target=self.mavlink_listener,
            daemon=True,
        )
        self.listener_thread.start()

    def mavlink_listener(self):

        self.get_logger().info("Waiting for Cube Orange heartbeat...")

        while rclpy.ok():

            try:
                msg = self.mavlink_connection.recv_match(
                    type="HEARTBEAT",
                    blocking=True,
                    timeout=1.0,
                )

                if msg is None:
                    continue

                # Ignore heartbeats from non-flight-controller components,
                # such as the telemetry radio or ground-control software.
                if msg.get_type() == "BAD_DATA":
                    continue

                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue

                mode = mavutil.mode_string_v10(msg)

                if mode != self.previous_mode:

                    if self.previous_mode is None:
                        self.get_logger().info(
                            f"Current flight mode: {mode}"
                        )
                    else:
                        self.get_logger().warn(
                            f"Flight mode changed: "
                            f"{self.previous_mode} -> {mode}"
                        )

                    self.previous_mode = mode

            except Exception as error:
                self.get_logger().error(
                    f"MAVLink receive error: {error}"
                )


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = MAVLinkModeMonitor()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nMode monitor stopped by keyboard.")

    except Exception as error:
        print(f"Mode monitor error: {error}")

    finally:
        if node is not None:

            if node.mavlink_connection is not None:
                node.mavlink_connection.close()

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()