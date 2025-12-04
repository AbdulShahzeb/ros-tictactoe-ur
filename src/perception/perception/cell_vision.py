#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Header, Int32
from geometry_msgs.msg import Pose2D, PoseStamped
from helper.msg import GridPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray

class CellVision(Node):
    def __init__(self):
        super().__init__("cell_vision")
        
        # --- CV ---
        self.bridge = CvBridge()
        self.cell_size_mm = 70.0
        self.grid_rows = 3
        self.grid_cols = 3

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publishers & Subscribers ---
        self.image_sub = self.create_subscription(Image, "/perception/warped_grid", self.image_callback, 10)
        self.set_min_blue_sat_sub = self.create_subscription(Int32, "/kb/set_min_blue_sat", self.set_min_blue_sat_callback, 10)
        self.min_blue_sat = 130
        self.shutdown_sub = self.create_subscription(
            Bool, "/kb/shutdown", self.shutdown_callback, 10
        )
        self.shutdown_requested = False

        self.cell_poses_pub = self.create_publisher(GridPose, "/perception/cell_poses", 10)
        self.cell_marker_pub = self.create_publisher(MarkerArray, "/perception/cell_markers", 10)

        # Meshes
        self.x_mesh_path = "package://perception/meshes/x.STL"
        self.o_mesh_path = "package://perception/meshes/o.STL"

    # ===============================================================
    #   Helper Functions
    # ===============================================================

    def pixel_to_grid_coords(self, x_px: float, y_px: float, img_w: int, img_h: int):
        """Convert pixel coordinates to grid coordinates in meters."""
        pixel_to_metres = 0.001
        x_grid = (x_px - img_w / 2.0) * pixel_to_metres
        y_grid = -(y_px - img_h / 2.0) * pixel_to_metres
        return x_grid, y_grid
    
    def grid_to_world_pose(self, x_grid, y_grid, yaw_grid=0.0):
        """Return PoseStamped in world frame from grid coordinates + yaw."""
        try:
            pose_grid = PoseStamped()
            pose_grid.header.stamp = rclpy.time.Time(seconds=0).to_msg()  # Use time=0 for latest
            pose_grid.header.frame_id = "grid_frame"
            pose_grid.pose.position.x = x_grid
            pose_grid.pose.position.y = y_grid
            pose_grid.pose.position.z = 0.0
            # Convert yaw to quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, yaw_grid)
            pose_grid.pose.orientation.x = q[0]
            pose_grid.pose.orientation.y = q[1]
            pose_grid.pose.orientation.z = q[2]
            pose_grid.pose.orientation.w = q[3]

            # Transform to world using latest available transform
            pose_world = self.tf_buffer.transform(
                pose_grid,
                "world",
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return pose_world

        except Exception as e:
            if not hasattr(self, '_last_tf_warn_time'):
                self._last_tf_warn_time = 0
            
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_tf_warn_time > 5.0:
                self.get_logger().warn(f"TF transform failed (grid_frame may not be detected yet): {e}")
                self._last_tf_warn_time = now
            return None


    def decide_color_at(self, img, x, y):
        """
        Detect color at given pixel coordinates.
        Returns: color code (-1=RED, 0=NONE, 1=BLUE)
        """

        # Color detection parameters
        RADIUS_START = 35
        RADIUS_MIN = 25
        RADIUS_STEP = 4
        MIN_AREA_FRAC = 0.02
        MIN_PIXELS_FLOOR = 10

        # HSV thresholds for blue and red
        BLUE_LOWER = (90, self.min_blue_sat, 60)
        BLUE_UPPER = (150, 255, 255)
        RED1_LOWER = (0, 80, 60)
        RED1_UPPER = (10, 255, 255)
        RED2_LOWER = (120, 25, 60)
        RED2_UPPER = (180, 255, 255)

        h, w = img.shape[:2]

        # Convert to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Create color masks
        mask_blue = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
        mask_red = cv2.inRange(hsv, RED1_LOWER, RED1_UPPER) | cv2.inRange(
            hsv, RED2_LOWER, RED2_UPPER
        )

        # Morphological opening to reduce noise
        kernel = np.ones((3, 3), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=1)

        # Try decreasing radii until we find a single color
        for r in range(RADIUS_START, RADIUS_MIN - 1, -RADIUS_STEP):
            circle_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(circle_mask, (int(round(x)), int(round(y))), r, 255, -1)

            blue_cnt = cv2.countNonZero(
                cv2.bitwise_and(mask_blue, mask_blue, mask=circle_mask)
            )
            red_cnt = cv2.countNonZero(
                cv2.bitwise_and(mask_red, mask_red, mask=circle_mask)
            )

            area = math.pi * (r**2)
            threshold = max(int(area * MIN_AREA_FRAC), MIN_PIXELS_FLOOR)

            blue_hit = blue_cnt >= threshold
            red_hit = red_cnt >= threshold

            # If both colors detected, radius too large - continue shrinking
            if blue_hit and red_hit:
                continue

            if blue_hit:
                return 1  # Blue
            if red_hit:
                return -1  # Red

            if r == RADIUS_MIN:
                return 0

        return 0

    # ===============================================================
    #   Callbacks
    # ===============================================================
    def set_min_blue_sat_callback(self, msg: Int32):
        """Set minimum blue saturation threshold."""
        self.min_blue_sat = msg.data
        self.get_logger().info(f"Minimum blue saturation threshold set to {self.min_blue_sat}")

    def image_callback(self, msg: Image):
        """Process warped grid image to find cell poses."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_h, img_w = cv_image.shape[:2]
        time = msg.header.stamp

        cell_poses = []
        color_codes = []
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                # Calculate center pixel of each cell
                x_px = (col + 0.5) * (img_w / self.grid_cols)
                y_px = (row + 0.5) * (img_h / self.grid_rows)

                # Convert to grid coordinates
                x_grid, y_grid = self.pixel_to_grid_coords(x_px, y_px, img_w, img_h)

                # Convert to world pose
                pose_world = self.grid_to_world_pose(x_grid, y_grid)
                if pose_world:
                    cell_poses.append(Pose2D(
                        x=pose_world.pose.position.x,
                        y=pose_world.pose.position.y,
                        theta=2.0 * math.atan2(pose_world.pose.orientation.z, pose_world.pose.orientation.w) * (180.0 / math.pi)
                    ))
                    color_codes.append(self.decide_color_at(cv_image, int(x_px), int(y_px)))

        # Publish cell poses
        grid_pose_msg = GridPose()
        grid_pose_msg.header = Header()
        grid_pose_msg.header.frame_id = "world"
        grid_pose_msg.header.stamp = time
        grid_pose_msg.poses = cell_poses
        grid_pose_msg.colors = color_codes
        self.cell_poses_pub.publish(grid_pose_msg)
        self.publish_cell_markers(color_codes, time)


    def shutdown_callback(self, msg: Bool):
        """Shuts down the Cell Vision Node"""
        self.get_logger().info("Received shutdown signal. Exiting...")
        self.shutdown_requested = True


    # ===============================================================
    #   RViz
    # ===============================================================
    def publish_cell_markers(self, color_codes, time):
        """Publish RViz markers for cell states in grid frame."""
        # Get center positions of each cell in grid frame
        if len(color_codes) != self.grid_rows * self.grid_cols:
            self.get_logger().warn("Color codes length does not match grid size.")
            return

        img_h = img_w = 210
        cell_poses = []
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                x_px = (col + 0.5) * (img_w / self.grid_cols)
                y_px = (row + 0.5) * (img_h / self.grid_rows)
                x_grid, y_grid = self.pixel_to_grid_coords(x_px, y_px, img_w, img_h)
                cell_poses.append((x_grid, y_grid))

        marker_array = MarkerArray()
        for i, (x, y) in enumerate(cell_poses):
            marker = Marker()
            marker.header.frame_id = "grid_frame"
            marker.header.stamp = time
            marker.ns = "cell_markers"
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            
            if color_codes[i] == 0:
                # Delete marker if cell is empty
                marker.action = Marker.DELETE
            else:
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.mesh_resource = self.x_mesh_path if color_codes[i] == 1 else self.o_mesh_path
                marker.mesh_use_embedded_materials = False

                # Set color based on detected color code
                if color_codes[i] == 1:  # Blue
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0
                elif color_codes[i] == -1:  # Red
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.cell_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = CellVision()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
