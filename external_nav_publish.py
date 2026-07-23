#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class ExternalNavPublisher(Node):

    def __init__(self):
        super().__init__("external_nav_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Make sure the callback uses this exact name:
        # self.tf_publisher
        self.tf_publisher = self.create_publisher(
            TFMessage,
            "/ap/tf",
            qos,
        )

        self.start_time = self.get_clock().now()

        # Position ramp rate, meters per second.
        self.ramp_speed = 0.1

        # 20 Hz publication timer.
        # Make sure this callback name exactly matches the function below.
        self.timer = self.create_timer(
            0.05,
            self.publish_external_nav,
        )

        self.message_count = 0

        self.get_logger().info(
            "External navigation publisher started."
        )
        self.get_logger().info(
            "Publishing /ap/tf at 20 Hz."
        )
        self.get_logger().info(
            "Position will ramp from (0,0,0) to (10,10,10)."
        )

    def publish_external_nav(self):
        now = self.get_clock().now()

        elapsed = (
            now - self.start_time
        ).nanoseconds * 1.0e-9

        # Increase from 0 to 10 meters.
        position = min(
            10.0,
            self.ramp_speed * elapsed,
        )

        x = position
        y = position
        z = position

        yaw = 0.0

        transform = TransformStamped()

        transform.header.stamp = now.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # Valid yaw-only quaternion.
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)

        message = TFMessage()
        message.transforms = [transform]

        self.tf_publisher.publish(message)

        self.message_count += 1

        # Print once every 20 messages, approximately once per second.
        if self.message_count % 20 == 0:
            self.get_logger().info(
                f"Published #{self.message_count}: "
                f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = ExternalNavPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "External navigation publisher stopped."
        )
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()