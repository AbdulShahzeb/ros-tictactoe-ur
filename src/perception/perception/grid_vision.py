#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Pose2D
from helper.msg import GridPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict


@dataclass
class MarkerPose:
    """Structure to store marker position and orientation"""

    x: float
    y: float
    angle: float

    def __str__(self):
        return f"({self.x:.1f}, {self.y:.1f}, {self.angle:.1f}°)"


class MovingAverageFilter:
    """Moving average filter for marker poses"""

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.x_values = deque(maxlen=window_size)
        self.y_values = deque(maxlen=window_size)
        self.angle_values = deque(maxlen=window_size)

    def add_sample(self, pose: MarkerPose):
        """Add a new pose sample to the filter"""
        self.x_values.append(pose.x)
        self.y_values.append(pose.y)
        self.angle_values.append(pose.angle)

    def get_filtered_pose(self) -> Optional[MarkerPose]:
        """Get the filtered (averaged) pose"""
        if (
            len(self.x_values) == 0
            or len(self.y_values) == 0
            or len(self.angle_values) == 0
        ):
            return None

        avg_x = sum(self.x_values) / len(self.x_values)
        avg_y = sum(self.y_values) / len(self.y_values)

        # Handle angle averaging (circular mean for angles)
        angles_rad = [math.radians(a) for a in self.angle_values]
        sin_sum = sum(math.sin(a) for a in angles_rad)
        cos_sum = sum(math.cos(a) for a in angles_rad)
        avg_angle_rad = math.atan2(sin_sum, cos_sum)
        avg_angle = math.degrees(avg_angle_rad)

        return MarkerPose(avg_x, avg_y, avg_angle)

    def is_ready(self) -> bool:
        """Check if filter has enough samples"""
        return len(self.x_values) >= self.window_size


class GridVisionNode(Node):
    def __init__(self):
        super().__init__("grid_vision_node")

        # --- CV2 Setup ---
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector_params = cv2.aruco.DetectorParameters()

        # --- Declare and get ROS parameters ---
        self.declare_parameter("exposure", 90)
        self.declare_parameter("corner0_x", 0.8315)  # Top left X (metres)
        self.declare_parameter("corner0_y", -0.0909)  # Top left Y (metres)
        self.declare_parameter("corner1_x", 0.8315)  # Bottom left X (metres)
        self.declare_parameter("corner1_y", 0.5000)  # Bottom left Y (metres)
        self.declare_parameter("corner2_x", 0.2625)  # Top right X (metres)
        self.declare_parameter("corner2_y", -0.0912)  # Top right Y (metres)
        self.declare_parameter("corner3_x", 0.2625)  # Bottom right X (metres)
        self.declare_parameter("corner3_y", 0.5000)  # Bottom right Y (metres)

        self.exposure = (
            self.get_parameter("exposure").get_parameter_value().integer_value
        )

        # Store corner positions (in metres)
        self.corner_positions_m = {
            0: [
                self.get_parameter("corner0_x").get_parameter_value().double_value,
                self.get_parameter("corner0_y").get_parameter_value().double_value,
            ],
            1: [
                self.get_parameter("corner1_x").get_parameter_value().double_value,
                self.get_parameter("corner1_y").get_parameter_value().double_value,
            ],
            2: [
                self.get_parameter("corner2_x").get_parameter_value().double_value,
                self.get_parameter("corner2_y").get_parameter_value().double_value,
            ],
            3: [
                self.get_parameter("corner3_x").get_parameter_value().double_value,
                self.get_parameter("corner3_y").get_parameter_value().double_value,
            ],
        }

        # --- MAFs for outer markers (ID: 0-3) ---
        self.corner_filters: Dict[int, MovingAverageFilter] = {
            0: MovingAverageFilter(5),  # Top left
            1: MovingAverageFilter(5),  # Bottom left
            2: MovingAverageFilter(5),  # Top right
            3: MovingAverageFilter(5),  # Bottom right
        }

        # --- MAFs for grid markers (ID: 4-7) in pixel coordinates ---
        self.grid_corner_filters: Dict[int, MovingAverageFilter] = {
            4: MovingAverageFilter(5),  # Grid top left
            5: MovingAverageFilter(5),  # Grid bottom left
            6: MovingAverageFilter(5),  # Grid top right
            7: MovingAverageFilter(5),  # Grid bottom right
        }

        # --- Image Warp Parameters ---
        self.filtered_poses: Dict[int, MarkerPose] = {}
        self.grid_filtered_poses: Dict[int, MarkerPose] = {}

        # Grid parameters
        self.cell_size_mm = 70.0  # mm
        self.grid_rows = 5
        self.grid_cols = 5

        # Warped grid size in pixels (using cell size as reference)
        pixels_per_mm = 2.0  # Adjust for desired resolution
        self.grid_warped_width = int(self.grid_cols * self.cell_size_mm * pixels_per_mm)
        self.grid_warped_height = int(
            self.grid_rows * self.cell_size_mm * pixels_per_mm
        )

        self.grid_transform_matrix = None
        self.grid_to_robot_transform = None

        # Publishers and Subscribers
        self.camera_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        self.toggle_log_sub = self.create_subscription(
            Bool, "/kb/toggle_log", self.log_callback, 10
        )
        self.toggle_log = False

        self.shutdown_sub = self.create_subscription(
            Bool, "/kb/shutdown", self.shutdown_callback, 10
        )
        self.shutdown_requested = False

        self.cell_poses_pub = self.create_publisher(
            GridPose, "/perception/cell_poses", 10
        )

        # --- Set Camera Exposure ---
        self.exposure_param_timer = self.create_timer(1.0, self.setup_camera_parameters)
        self.get_logger().info("Grid Vision node started")

    def setup_camera_parameters(self):
        """Set up camera parameters"""
        node = Node("exposure_setter_node")
        exposure_client = node.create_client(
            SetParameters, "/camera/camera/set_parameters"
        )
        for _ in range(5):
            if exposure_client.wait_for_service(timeout_sec=1.0):
                break
            node.get_logger().info(
                "Service /camera/camera/set_parameters not available, retrying..."
            )
        else:
            node.get_logger().error(
                "Service /camera/camera/set_parameters not available, giving up."
            )
            if self.exposure_param_timer:
                self.exposure_param_timer.cancel()
                self.exposure_param_timer = None
            node.destroy_node()
            return

        request = SetParameters.Request()
        parameter = Parameter(
            name="rgb_camera.exposure", value=self.exposure
        ).to_parameter_msg()
        request.parameters = [parameter]
        future = exposure_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            if future.result().results[0].successful:
                node.get_logger().info(
                    f"Successfully set parameter rgb_camera.exposure to {self.exposure} ms"
                )
            else:
                node.get_logger().error(
                    f"Failed to set parameter: {future.result().results[0].reason}"
                )
        else:
            node.get_logger().error("Service call failed %r" % (future.exception(),))

        if self.exposure_param_timer:
            self.exposure_param_timer.cancel()
            self.exposure_param_timer = None

        node.destroy_node()

    def get_marker_corner_position(self, marker_id: int, corners) -> tuple:
        """Extract the appropriate corner position from marker corners"""
        marker_corners = corners[0]

        # Marker corners are: [0]=TL, [1]=TR, [2]=BR, [3]=BL
        corner_map = {
            0: 0,  # Top left -> use top-left corner
            1: 3,  # Bottom left -> use bottom-left corner
            2: 1,  # Top right -> use top-right corner
            3: 2,  # Bottom right -> use bottom-right corner
            4: 0,  # Grid top left -> use top-left corner
            5: 3,  # Grid bottom left -> use bottom-left corner
            6: 1,  # Grid top right -> use top-right corner
            7: 2,  # Grid bottom right -> use bottom-right corner
        }

        corner_idx = corner_map[marker_id]
        return float(marker_corners[corner_idx][0]), float(
            marker_corners[corner_idx][1]
        )

    def transform_pixel_to_robot(
        self, pixel_x: float, pixel_y: float
    ) -> Optional[tuple]:
        """Transform pixel coordinates to robot coordinates (in mm) using outer markers 0-3"""
        # Need all 4 outer corner markers
        required_markers = [0, 1, 2, 3]
        if not all(marker_id in self.filtered_poses for marker_id in required_markers):
            return None

        # Source points: detected pixel positions of outer markers
        src_points = np.float32(
            [
                [self.filtered_poses[0].x, self.filtered_poses[0].y],  # Top left
                [self.filtered_poses[1].x, self.filtered_poses[1].y],  # Bottom left
                [self.filtered_poses[2].x, self.filtered_poses[2].y],  # Top right
                [self.filtered_poses[3].x, self.filtered_poses[3].y],  # Bottom right
            ]
        )

        # Destination points: known real-world positions in mm
        corner_mm = {
            k: [v[0] * 1000, v[1] * 1000] for k, v in self.corner_positions_m.items()
        }
        dst_points = np.float32(
            [
                [corner_mm[0][0], corner_mm[0][1]],  # Top left
                [corner_mm[1][0], corner_mm[1][1]],  # Bottom left
                [corner_mm[2][0], corner_mm[2][1]],  # Top right
                [corner_mm[3][0], corner_mm[3][1]],  # Bottom right
            ]
        )

        # Create pixel-to-robot transform
        pixel_to_robot_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        # Transform the pixel point
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        robot_point = cv2.perspectiveTransform(pixel_point, pixel_to_robot_matrix)

        return (robot_point[0][0][0], robot_point[0][0][1])

    def detect_marker_pose(self, marker_id, corners) -> MarkerPose:
        """Extract pose (position and orientation) from marker corners"""
        marker_corners = corners[0]

        # Get the appropriate corner position
        x, y = self.get_marker_corner_position(marker_id, corners)

        # Calculate orientation from bottom edge
        bottom_left = marker_corners[3]
        bottom_right = marker_corners[2]

        dx = bottom_right[0] - bottom_left[0]
        dy = bottom_right[1] - bottom_left[1]
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)

        return MarkerPose(x, y, angle_deg)

    def update_grid_transform(self) -> bool:
        """Update perspective transform matrix for grid based on grid markers (4-7)"""
        required_markers = [4, 5, 6, 7]
        if not all(
            marker_id in self.grid_filtered_poses for marker_id in required_markers
        ):
            return False

        # Source points from camera image (corner positions of markers 4-7)
        src_points = np.float32(
            [
                [
                    self.grid_filtered_poses[4].x,
                    self.grid_filtered_poses[4].y,
                ],  # Top left
                [
                    self.grid_filtered_poses[5].x,
                    self.grid_filtered_poses[5].y,
                ],  # Bottom left
                [
                    self.grid_filtered_poses[6].x,
                    self.grid_filtered_poses[6].y,
                ],  # Top right
                [
                    self.grid_filtered_poses[7].x,
                    self.grid_filtered_poses[7].y,
                ],  # Bottom right
            ]
        )

        # Destination points for warped grid image (corners of the warped rectangle)
        dst_points = np.float32(
            [
                [0, 0],  # Top left
                [0, self.grid_warped_height],  # Bottom left
                [self.grid_warped_width, 0],  # Top right
                [self.grid_warped_width, self.grid_warped_height],  # Bottom right
            ]
        )

        # Calculate perspective transform matrix
        self.grid_transform_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        # Set up transform from warped grid to robot coordinates
        self.setup_grid_to_robot_transform()
        return True

    def setup_grid_to_robot_transform(self):
        """Set up transformation from warped grid coordinates to robot coordinates"""
        # Warped grid corners in pixel coordinates
        warped_points = np.float32(
            [
                [0, 0],
                [0, self.grid_warped_height],
                [self.grid_warped_width, 0],
                [self.grid_warped_width, self.grid_warped_height],
            ]
        )

        # Grid physical corners in robot frame (in mm)
        # Need to transform the detected pixel positions of markers 4-7 to robot coordinates
        # This requires first having a camera-to-robot transform based on markers 0-3

        robot_points_mm = []
        for marker_id in [4, 5, 6, 7]:
            if marker_id in self.grid_filtered_poses:
                # Get pixel position
                pixel_pose = self.grid_filtered_poses[marker_id]

                # Transform pixel to robot coordinates using the outer marker reference
                # You need a transform_pixel_to_robot() function here
                robot_pos = self.transform_pixel_to_robot(pixel_pose.x, pixel_pose.y)
                robot_points_mm.append([robot_pos[0], robot_pos[1]])

        if len(robot_points_mm) != 4:
            return

        robot_points = np.float32(robot_points_mm)

        self.grid_to_robot_transform = cv2.getPerspectiveTransform(
            warped_points, robot_points
        )

    def calculate_grid_cell_poses(self) -> Optional[list]:
        """Calculate poses for all grid cells based on warped grid"""
        if self.grid_to_robot_transform is None:
            return None

        cell_poses = []

        # Calculate cell size in warped image pixels
        cell_width_px = self.grid_warped_width / self.grid_cols
        cell_height_px = self.grid_warped_height / self.grid_rows

        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                # Calculate center of cell in warped image coordinates
                center_x_warped = (col + 0.5) * cell_width_px
                center_y_warped = (row + 0.5) * cell_height_px

                # Transform to robot coordinates (in mm)
                warped_point = np.array(
                    [[[center_x_warped, center_y_warped]]], dtype=np.float32
                )
                robot_point = cv2.perspectiveTransform(
                    warped_point, self.grid_to_robot_transform
                )
                robot_x_mm = robot_point[0][0][0]
                robot_y_mm = robot_point[0][0][1]

                # Calculate angle (assume grid is aligned, so use average angle from markers)
                if len(self.grid_filtered_poses) >= 4:
                    angles = [pose.angle for pose in self.grid_filtered_poses.values()]
                    angles_rad = [math.radians(a) for a in angles]
                    sin_sum = sum(math.sin(a) for a in angles_rad)
                    cos_sum = sum(math.cos(a) for a in angles_rad)
                    angle = math.degrees(math.atan2(sin_sum, cos_sum)) - 90.0
                else:
                    angle = 0.0

                # Convert from mm to metres
                cell_poses.append(
                    MarkerPose(robot_x_mm / 1000.0, robot_y_mm / 1000.0, angle)
                )

        return cell_poses

    def draw_grid_labels(self, warped_grid):
        """Draw cell labels on warped grid image"""
        if self.grid_to_robot_transform is None:
            return

        cell_width_px = self.grid_warped_width / self.grid_cols
        cell_height_px = self.grid_warped_height / self.grid_rows

        # Get average angle
        if len(self.grid_filtered_poses) >= 4:
            angles = [pose.angle for pose in self.grid_filtered_poses.values()]
            angles_rad = [math.radians(a) for a in angles]
            sin_sum = sum(math.sin(a) for a in angles_rad)
            cos_sum = sum(math.cos(a) for a in angles_rad)
            avg_angle = math.degrees(math.atan2(sin_sum, cos_sum))
        else:
            avg_angle = 0.0

        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                # Calculate center of cell
                center_x_warped = (col + 0.5) * cell_width_px
                center_y_warped = (row + 0.5) * cell_height_px

                # Transform to robot coordinates
                warped_point = np.array(
                    [[[center_x_warped, center_y_warped]]], dtype=np.float32
                )
                robot_point = cv2.perspectiveTransform(
                    warped_point, self.grid_to_robot_transform
                )
                robot_x_m = robot_point[0][0][0] / 1000.0
                robot_y_m = robot_point[0][0][1] / 1000.0

                # Draw cell boundary
                cell_x = int(col * cell_width_px)
                cell_y = int(row * cell_height_px)
                cv2.rectangle(
                    warped_grid,
                    (cell_x, cell_y),
                    (int((col + 1) * cell_width_px), int((row + 1) * cell_height_px)),
                    (0, 255, 0),
                    1,
                )

                # Draw center point
                cv2.circle(
                    warped_grid,
                    (int(center_x_warped), int(center_y_warped)),
                    3,
                    (255, 0, 0),
                    -1,
                )

                # Draw label with coordinates
                label = f"({robot_x_m:.3f}, {robot_y_m:.3f})"
                angle_label = f"{avg_angle:.1f}"

                # Position text
                text_x = int(center_x_warped) - 40
                text_y = int(center_y_warped) - 5

                cv2.putText(
                    warped_grid,
                    label,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (255, 255, 255),
                    1,
                )
                cv2.putText(
                    warped_grid,
                    angle_label,
                    (text_x + 10, text_y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (255, 255, 255),
                    1,
                )

    def decide_color_at(self, img, x, y):
        """
        Detect color at given pixel coordinates.
        Returns: color code (-1=RED, 0=NONE, 1=BLUE)
        """

        # Color detection parameters
        RADIUS_START = 40
        RADIUS_MIN = 30
        RADIUS_STEP = 4
        MIN_AREA_FRAC = 0.02
        MIN_PIXELS_FLOOR = 10

        # HSV thresholds for blue and red
        BLUE_LOWER = (100, 80, 50)
        BLUE_UPPER = (140, 255, 255)
        RED1_LOWER = (0, 80, 60)
        RED1_UPPER = (10, 255, 255)
        RED2_LOWER = (170, 80, 60)
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

            # If we've reached minimum radius, return NONE
            if r == RADIUS_MIN:
                return 0

        return 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

            if ids is not None:
                # Process all markers (0-7)
                for i, marker_id in enumerate(ids.flatten()):
                    pose = self.detect_marker_pose(marker_id, corners[i])

                    if marker_id in [0, 1, 2, 3]:
                        # Outer corner markers - not needed for grid warp but keep for reference
                        self.corner_filters[marker_id].add_sample(pose)
                        if self.corner_filters[marker_id].is_ready():
                            filtered_pose = self.corner_filters[
                                marker_id
                            ].get_filtered_pose()
                            if filtered_pose:
                                self.filtered_poses[marker_id] = filtered_pose

                    elif marker_id in [4, 5, 6, 7]:
                        # Grid corner markers
                        self.grid_corner_filters[marker_id].add_sample(pose)
                        if self.grid_corner_filters[marker_id].is_ready():
                            filtered_pose = self.grid_corner_filters[
                                marker_id
                            ].get_filtered_pose()
                            if filtered_pose:
                                self.grid_filtered_poses[marker_id] = filtered_pose

                # Update grid transform and create warped grid image
                if self.update_grid_transform():
                    warped_grid = cv2.warpPerspective(
                        cv_image,
                        self.grid_transform_matrix,
                        (self.grid_warped_width, self.grid_warped_height),
                    )

                    # Draw cell labels
                    self.draw_grid_labels(warped_grid)

                    # Calculate and publish cell poses
                    cell_poses = self.calculate_grid_cell_poses()
                    if cell_poses is not None:
                        grid_pose = GridPose()
                        grid_pose.header = Header()
                        grid_pose.header.stamp = self.get_clock().now().to_msg()
                        grid_pose.header.frame_id = "robot_base"

                        # Publish center 3x3 grid
                        row_center = self.grid_rows // 2
                        col_center = self.grid_cols // 2
                        cell_width = self.grid_warped_width / self.grid_cols
                        cell_height = self.grid_warped_height / self.grid_rows
                        for r in range(row_center - 1, row_center + 2):
                            for c in range(col_center - 1, col_center + 2):
                                i = r * self.grid_cols + c
                                if 0 <= i < len(cell_poses):
                                    cell_pose = cell_poses[i]

                                    cx_warped = (c + 0.5) * cell_width
                                    cy_warped = (r + 0.5) * cell_height
                                    color_code = self.decide_color_at(
                                        warped_grid, cx_warped, cy_warped
                                    )

                                    cell_msg = Pose2D()
                                    cell_msg.x = cell_pose.x
                                    cell_msg.y = cell_pose.y
                                    cell_msg.theta = cell_pose.angle
                                    grid_pose.poses.append(cell_msg)
                                    grid_pose.colors.append(color_code)

                        self.cell_poses_pub.publish(grid_pose)

                        if self.toggle_log:
                            self.get_logger().info(
                                f"Published {len(cell_poses)} cell poses"
                            )

                    cv2.imshow("Warped Grid", warped_grid)

                # Draw detected markers on original image
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            cv2.imshow("RealSense Image", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def log_callback(self, msg):
        self.toggle_log = not self.toggle_log

    def shutdown_callback(self, msg):
        """Shuts down the Grid Vision Node"""
        self.get_logger().info("Received shutdown signal. Exiting...")
        self.shutdown_requested = True


def main(args=None):
    rclpy.init(args=args)
    node = GridVisionNode()

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
