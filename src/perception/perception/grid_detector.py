#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from helper.msg import CellPose, CellPoseArray


class GridDetector(Node):
    def __init__(self):
        super().__init__("grid_detector_node")

        self.GRID_COLS = 8
        self.GRID_ROWS = 5
        self.CELL_SIZE_M = 0.070
        self.CAMERA_OFFSET_X = 0.5
        self.CAMERA_OFFSET_Y = 0.0
        self.CAMERA_OFFSET_Z = 0.8
        self.BOARD_HEIGHT_Z = 0.002
        self.MIN_LINE_LENGTH = 400
        self.MAX_LINE_GAP = 10
        self.CANNY_LOW = 100
        self.CANNY_HIGH = 150

        self.bridge = CvBridge()

        # Subscriber
        self.rgb_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        # Publisher
        self.cells_pub = self.create_publisher(CellPoseArray, "cell_poses", 10)

        self.get_logger().info("Grid detector node initialized")
        self.get_logger().info(
            f"Grid: {self.GRID_COLS}x{self.GRID_ROWS}, Cell size: {self.CELL_SIZE_M}m"
        )

    def image_callback(self, msg):
        """Process incoming RGB image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            grid_data = self.detect_grid_lines(cv_image)

            if grid_data is not None:
                grid_corners_px, grid_angle, pixels_per_meter = grid_data

                cell_centers_px = self.compute_cell_centers(grid_corners_px)
                cell_poses_base = self.transform_to_base_frame(
                    cell_centers_px, grid_angle, pixels_per_meter
                )

                self.publish_cell_poses(cell_poses_base, msg.header.stamp)
                self.visualize_detection(cv_image, grid_corners_px, cell_centers_px)
            else:
                self.get_logger().warn("Could not detect grid")

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def detect_grid_lines(self, image):
        """
        Detect the grid using line detection
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, self.CANNY_LOW, self.CANNY_HIGH, apertureSize=3)
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=self.MIN_LINE_LENGTH,
            maxLineGap=self.MAX_LINE_GAP,
        )

        if lines is None or len(lines) < 4:
            return None

        h_lines, v_lines = self.classify_lines(lines)

        if len(h_lines) < 2 or len(v_lines) < 2:
            return None

        grid_corners, angle = self.find_grid_bounds(h_lines, v_lines)

        if grid_corners is None:
            return None

        grid_width_px = np.linalg.norm(grid_corners[1] - grid_corners[0])
        grid_height_px = np.linalg.norm(grid_corners[3] - grid_corners[0])

        expected_width_m = self.GRID_COLS * self.CELL_SIZE_M
        expected_height_m = self.GRID_ROWS * self.CELL_SIZE_M

        pixels_per_meter = (
            grid_width_px / expected_width_m + grid_height_px / expected_height_m
        ) / 2

        return grid_corners, angle, pixels_per_meter

    def classify_lines(self, lines):
        """Separate lines into horizontal and vertical"""
        h_lines = []
        v_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.abs(np.arctan2(y2 - y1, x2 - x1))

            if angle < np.pi / 4 or angle > 3 * np.pi / 4:
                h_lines.append(line[0])
            else:
                v_lines.append(line[0])

        return h_lines, v_lines

    def find_grid_bounds(self, h_lines, v_lines):
        """
        Find the outer bounds of the grid from horizontal and vertical lines
        """
        # Get extreme positions
        h_y_coords = []
        v_x_coords = []

        for x1, y1, x2, y2 in h_lines:
            h_y_coords.extend([y1, y2])

        for x1, y1, x2, y2 in v_lines:
            v_x_coords.extend([x1, x2])

        if not h_y_coords or not v_x_coords:
            return None, None

        # Find bounds
        top_y = min(h_y_coords)
        bottom_y = max(h_y_coords)
        left_x = min(v_x_coords)
        right_x = max(v_x_coords)

        # Create corner points
        corners = np.array(
            [
                [left_x, top_y],  # top-left
                [right_x, top_y],  # top-right
                [right_x, bottom_y],  # bottom-right
                [left_x, bottom_y],  # bottom-left
            ],
            dtype=np.float32,
        )

        # Calculate grid angle from top edge
        angle = np.arctan2(corners[1][1] - corners[0][1], corners[1][0] - corners[0][0])

        return corners, angle

    def compute_cell_centers(self, grid_corners):
        """
        Compute pixel coordinates for all cell centers
        Returns: Nx2 array of (x, y) pixel coordinates
        """
        cell_centers = []

        for row in range(self.GRID_ROWS):
            for col in range(self.GRID_COLS):
                u = (col + 0.5) / self.GRID_COLS
                v = (row + 0.5) / self.GRID_ROWS

                top = (1 - u) * grid_corners[0] + u * grid_corners[1]
                bottom = (1 - u) * grid_corners[3] + u * grid_corners[2]
                center = (1 - v) * top + v * bottom

                cell_centers.append(center)

        return np.array(cell_centers)

    def transform_to_base_frame(self, points_px, grid_angle, pixels_per_meter):
        """
        Transform pixel coordinates to robot base frame
        Assumes camera is mounted above looking down
        """
        cell_poses = []

        img_center_x = points_px[:, 0].mean()
        img_center_y = points_px[:, 1].mean()

        for point_px in points_px:
            # Convert pixel offset from image center to meters
            dx_px = point_px[0] - img_center_x
            dy_px = point_px[1] - img_center_y

            dx_m = dx_px / pixels_per_meter
            dy_m = dy_px / pixels_per_meter

            # Transform to base frame
            x_base = self.CAMERA_OFFSET_X + dy_m
            y_base = self.CAMERA_OFFSET_Y + dx_m
            z_base = self.BOARD_HEIGHT_Z

            cell_poses.append(
                {
                    "x": float(x_base),
                    "y": float(y_base),
                    "z": float(z_base),
                    "angle": float(grid_angle),
                }
            )

        return cell_poses

    def publish_cell_poses(self, cell_poses, timestamp):
        """Publish array of cell poses"""
        msg = CellPoseArray()
        msg.header.stamp = timestamp
        msg.header.frame_id = "base_link"

        for pose_dict in cell_poses:
            cell_pose = CellPose()
            cell_pose.x = pose_dict["x"]
            cell_pose.y = pose_dict["y"]
            cell_pose.angle = pose_dict["angle"]
            msg.poses.append(cell_pose)

        self.cells_pub.publish(msg)
        self.get_logger().info(
            f"Published {len(cell_poses)} cell poses, "
            f"sample: x={cell_poses[0]['x']:.3f}, y={cell_poses[0]['y']:.3f}"
        )

    def visualize_detection(self, image, grid_corners, cell_centers):
        """Draw detection results on image for debugging"""
        vis = image.copy()

        for corner in grid_corners:
            cv2.circle(vis, tuple(corner.astype(int)), 8, (0, 255, 0), -1)

        cv2.polylines(vis, [grid_corners.astype(int)], True, (0, 255, 0), 3)

        for idx, center in enumerate(cell_centers):
            row = idx // self.GRID_COLS
            col = idx % self.GRID_COLS
            cv2.circle(vis, tuple(center.astype(int)), 4, (255, 0, 0), -1)
            cv2.putText(
                vis,
                f"{row},{col}",
                tuple((center + 5).astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (0, 0, 255),
                1,
            )

        cv2.imshow("Grid Detection", vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = GridDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
