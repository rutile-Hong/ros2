# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy

# class DDSMotorSpin(Node):
#     def __init__(self):
#         super().__init__('dds_motor_spin')
#         self.pub = self.create_publisher(Joy, '/ap/joy', 10)
#         self.timer = self.create_timer(0.1, self.send_joy)

#     def send_joy(self):
#         msg = Joy()

#         # RC-style joystick axes:
#         # roll, pitch, throttle, yaw
#         # throttle: -1.0 low, 0.0 mid, 1.0 high
#         msg.axes = [
#             0.2,   # roll
#             0.0,   # pitch
#             -0.2,  # throttle: very low spin test
#             0.0    # yaw
#         ]

#         msg.buttons = []
#         self.pub.publish(msg)

# def main():
#     rclpy.init()
#     node = DDSMotorSpin()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import threading

class DDSMotorSpin(Node):
    def __init__(self):
        super().__init__('dds_motor_spin')

        self.pub = self.create_publisher(Joy, '/ap/joy', 10)

        self.roll = -0.2
        self.pitch = 0.0
        self.throttle = -0.5
        self.yaw = 0.0

        self.motor_enabled = True

        self.timer = self.create_timer(0.1, self.send_joy)

        threading.Thread(target=self.keyboard_listener, daemon=True).start()

        self.get_logger().info("Press 'q' then Enter to stop motors")

    def keyboard_listener(self):
        while rclpy.ok():
            key = input().strip().lower()

            if key == 'q':
                self.motor_enabled = False
                self.throttle = -1.0

                self.get_logger().warn("EMERGENCY MOTOR STOP")

                msg = Joy()
                msg.axes = [
                    self.roll,
                    self.pitch,
                    self.throttle,
                    self.yaw
                ]
                msg.buttons = []

                for _ in range(5):
                    self.pub.publish(msg)

                self.destroy_timer(self.timer)

    def send_joy(self):
        if not self.motor_enabled:
            return

        msg = Joy()

        msg.axes = [
            self.roll,      # roll
            self.pitch,     # pitch
            self.throttle,  # throttle
            self.yaw        # yaw
        ]

        msg.buttons = []

        self.pub.publish(msg)

def main():
    rclpy.init()

    node = DDSMotorSpin()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
