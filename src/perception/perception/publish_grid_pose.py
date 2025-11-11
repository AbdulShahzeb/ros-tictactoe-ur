#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from helper.msg import GridPose


def main(args=None):
    rclpy.init(args=args)
    node = Node('grid_pose_publisher')
    pub = node.create_publisher(GridPose, '/perception/cell_poses', 10)

    msg = GridPose()
    msg.header = Header()
    msg.header.frame_id = 'camera_depth_optical_frame'
    msg.header.stamp = node.get_clock().now().to_msg()

    msg.poses = [
        Pose2D(x=0.541, y=0.183, theta=88.1),
        Pose2D(x=0.540, y=0.245, theta=88.1),
        Pose2D(x=0.539, y=0.307, theta=88.1),
        Pose2D(x=0.603, y=0.184, theta=88.1),
        Pose2D(x=0.602, y=0.246, theta=88.1),
        Pose2D(x=0.601, y=0.308, theta=88.1),
        Pose2D(x=0.665, y=0.185, theta=88.1),
        Pose2D(x=0.664, y=0.247, theta=88.1),
        Pose2D(x=0.663, y=0.310, theta=88.1)
    ]
    msg.colors = [0, 0, 0, 0, 1, 0, 0, 0, 0]

    node.get_logger().info('Waiting for subscriber on /perception/cell_poses...')

    timeout_sec = 5.0
    start_time = time.time()
    while rclpy.ok():
        if pub.get_subscription_count() > 0:
            node.get_logger().info(f'Found {pub.get_subscription_count()} subscriber(s). Publishing...')
            break
        if time.time() - start_time > timeout_sec:
            node.get_logger().warn('Timeout: No subscriber found. Publishing anyway (use latched QoS for reliability).')
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    pub.publish(msg)
    node.get_logger().info('Message published.')

    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()