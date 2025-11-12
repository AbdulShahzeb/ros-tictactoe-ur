#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from helper.msg import GridPose
import re
import threading
from random import randint

POSES = [
    Pose2D(x=0.541, y=0.183, theta=88.1),
    Pose2D(x=0.540, y=0.245, theta=88.1),
    Pose2D(x=0.539, y=0.307, theta=88.1),
    Pose2D(x=0.603, y=0.184, theta=88.1),
    Pose2D(x=0.602, y=0.246, theta=88.1),
    Pose2D(x=0.601, y=0.308, theta=88.1),
    Pose2D(x=0.665, y=0.185, theta=88.1),
    Pose2D(x=0.664, y=0.247, theta=88.1),
    Pose2D(x=0.663, y=0.310, theta=88.1),
]


class GridPosePublisher(Node):
    def __init__(self):
        super().__init__("grid_pose_publisher")
        self.pub = self.create_publisher(GridPose, "/perception/cell_poses", 10)
        self.colors = [0] * 9
        self.noise_mode = False
        self.pub_timer = self.create_timer(1 / 6, self.publish)
        self.get_logger().info(
            "GridPosePublisher initialised. Commands: n=toggle noise, p=<9 ints>, q=quit"
        )

    def handle_input(self):
        while rclpy.ok():
            try:
                user_input = input("Enter cmd: ")
                if user_input.lower() == "n":
                    self.noise_mode = not self.noise_mode
                    self.colors = [0] * 9   # Reset colors when toggling noise
                    self.get_logger().info(
                        f"Noise mode {'enabled' if self.noise_mode else 'disabled'}."
                    )
                elif user_input.startswith("p "):
                    parts = re.findall(r"-?\d+", user_input)
                    if len(parts) != 9:
                        self.get_logger().error("Please enter 9 integers for colors.")
                        continue
                    self.colors = [int(c) for c in parts]
                    self.get_logger().info(f"Set colors: {self.colors}")
                elif user_input.lower() == "q":
                    self.get_logger().info("Exiting GridPosePublisher.")
                    rclpy.shutdown()
                    break
                else:
                    self.get_logger().error(
                        "Unknown command. Use 'n', 'p <9 ints>', or 'q'."
                    )
            except EOFError:
                rclpy.shutdown()
                break

    def publish(self):
        msg = GridPose()
        msg.header = Header()
        msg.header.frame_id = "camera_depth_optical_frame"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = POSES
        msg.colors = (
            [randint(-1, 1) for _ in range(9)] if self.noise_mode else self.colors
        )
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GridPosePublisher()

    input_thread = threading.Thread(target=node.handle_input, daemon=True)
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
