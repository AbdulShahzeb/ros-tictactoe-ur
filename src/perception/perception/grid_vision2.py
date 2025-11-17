#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Pose2D, TransformStamped
from helper.msg import GridPose
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict
import tf2_ros
from tf_transformations import quaternion_from_euler


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
        self.exposure = (
            self.get_parameter("exposure").get_parameter_value().integer_value
        )

        # --- Camera intrinsics (will be set from CameraInfo) ---
        self.fx = self.fy = self.cx = self.cy = None

        # --- Camera to base_link transform (adjust if needed) ---
        # If the board appears rotated/flipped, modify translation or RPY values
        self.cam_to_base_translation = (0.850, 0.231, 0.912)  # (x, y, z) in meters
        self.cam_to_base_rpy = (math.pi, 0, math.pi)  # (roll, pitch, yaw)

        # --- MAFs for grid markers (ID: 4-7) in pixel coordinates ---
        self.grid_corner_filters: Dict[int, MovingAverageFilter] = {
            4: MovingAverageFilter(5),  # Grid top left
            5: MovingAverageFilter(5),  # Grid bottom left
            6: MovingAverageFilter(5),  # Grid top right
            7: MovingAverageFilter(5),  # Grid bottom right
        }

        self.grid_filtered_poses: Dict[int, MarkerPose] = {}

        # Grid parameters (centres of ArUco markers define the grid)
        # Physical board: 350x350mm total, markers at 35+70+70+70+35 = 280mm apart
        # But the actual 5x5 grid is 350x350mm (half cells on edges)
        self.cell_size_mm = 70.0  # mm per cell
        self.grid_rows = 5  # Full 5x5 grid
        self.grid_cols = 5
        self.board_size_mm = 280.0  # Distance from top-left to bottom-right marker centres
        self.full_grid_size_mm = 350.0  # Full grid including half cells
        self.edge_offset_mm = 35.0  # Offset from marker to edge (half cell)

        # Warped grid size in pixels (full 5x5 grid)
        pixels_per_mm = 2.0  # Adjust for desired resolution
        self.grid_warped_width = int(self.full_grid_size_mm * pixels_per_mm)
        self.grid_warped_height = int(self.full_grid_size_mm * pixels_per_mm)

        self.grid_transform_matrix = None

        # --- TF Broadcaster ---
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_camera_transform()

        # Publishers and Subscribers
        self.camera_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10,
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

        self.marker_pub = self.create_publisher(
            MarkerArray, "/perception/grid_markers", 10
        )

        # --- Set Camera Exposure ---
        self.exposure_param_timer = self.create_timer(1.0, self.setup_camera_parameters)
        self.get_logger().info("Grid Vision node started")

    def broadcast_camera_transform(self):
        """Broadcast static transform from base_link to camera_depth_optical_frame"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_depth_optical_frame"

        t.transform.translation.x = self.cam_to_base_translation[0]
        t.transform.translation.y = self.cam_to_base_translation[1]
        t.transform.translation.z = self.cam_to_base_translation[2]

        q = quaternion_from_euler(*self.cam_to_base_rpy)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics from CameraInfo message"""
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

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

    def get_marker_center_position(self, corners) -> tuple:
        """Get center position of marker from corners"""
        marker_corners = corners[0]
        center = marker_corners.mean(axis=0)
        return float(center[0]), float(center[1])

    def pixel_to_camera_coords(self, pixel_x: float, pixel_y: float, z_depth: float) -> tuple:
        """
        Convert pixel coordinates to 3D camera coordinates.
        z_depth: assumed depth in meters (distance from camera to board)
        If positions are incorrect, check z_depth value and camera intrinsics.
        """
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return None

        x_cam = ((pixel_x - self.cx) * z_depth) / self.fx
        y_cam = ((pixel_y - self.cy) * z_depth) / self.fy
        z_cam = z_depth

        return (x_cam, y_cam, z_cam)

    def camera_to_base_coords(self, x_cam: float, y_cam: float, z_cam: float) -> tuple:
        """
        Transform camera coordinates to base_link coordinates.
        Applies the camera transform (translation + rotation).
        If board appears flipped or rotated, check cam_to_base_rpy values.
        """
        # Apply rotation (RPY = pi, 0, pi)
        # This is equivalent to rotating 180° around X, then 180° around Z
        # Result: X -> -X, Y -> -Y, Z -> Z
        x_rotated = -x_cam
        y_rotated = -y_cam
        z_rotated = z_cam

        # Apply translation
        x_base = x_rotated + self.cam_to_base_translation[0]
        y_base = y_rotated + self.cam_to_base_translation[1]
        z_base = z_rotated + self.cam_to_base_translation[2]

        return (x_base, y_base, z_base)

    def pixel_to_base_coords(self, pixel_x: float, pixel_y: float, z_depth: float) -> Optional[tuple]:
        """Convert pixel coordinates directly to base_link coordinates"""
        cam_coords = self.pixel_to_camera_coords(pixel_x, pixel_y, z_depth)
        if cam_coords is None:
            return None
        return self.camera_to_base_coords(*cam_coords)

    def pixel_to_base_coords_fixed_z(self, pixel_x: float, pixel_y: float) -> Optional[tuple]:
        """
        Convert pixel coordinates to base_link coordinates with fixed Z=0.002m.
        The board is always 2mm above world zero.
        """
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return None

        # Target Z in base_link frame
        z_base_target = 0.002  # 2mm above world zero

        # We need to find the z_cam that, after transformation, gives us z_base_target
        # Given transform: z_base = z_cam + cam_to_base_translation[2]
        # So: z_cam = z_base_target - cam_to_base_translation[2]
        z_cam = z_base_target - self.cam_to_base_translation[2]

        # Now use this z_cam for pixel-to-camera conversion
        x_cam = ((pixel_x - self.cx) * z_cam) / self.fx
        y_cam = ((pixel_y - self.cy) * z_cam) / self.fy

        # Transform to base_link
        x_rotated = x_cam
        y_rotated = -y_cam
        z_rotated = z_cam

        x_base = x_rotated + self.cam_to_base_translation[0]
        y_base = y_rotated + self.cam_to_base_translation[1]
        z_base = z_rotated + self.cam_to_base_translation[2]

        return (x_base, y_base, z_base)

    def detect_marker_pose(self, corners) -> MarkerPose:
        """Extract pose (position and orientation) from marker corners"""
        marker_corners = corners[0]

        # Get center position
        x, y = self.get_marker_center_position(corners)

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

        # Source points from camera image (center positions of markers 4-7)
        src_points = np.float32(
            [
                [self.grid_filtered_poses[4].x, self.grid_filtered_poses[4].y],  # Top left
                [self.grid_filtered_poses[5].x, self.grid_filtered_poses[5].y],  # Bottom left
                [self.grid_filtered_poses[6].x, self.grid_filtered_poses[6].y],  # Top right
                [self.grid_filtered_poses[7].x, self.grid_filtered_poses[7].y],  # Bottom right
            ]
        )

        # Destination points for warped grid image
        # Markers are offset 35mm from edges in the full 350x350mm grid
        offset_px = self.edge_offset_mm * 2.0  # pixels_per_mm = 2.0
        dst_points = np.float32(
            [
                [offset_px, offset_px],  # Top left marker
                [offset_px, self.grid_warped_height - offset_px],  # Bottom left marker
                [self.grid_warped_width - offset_px, offset_px],  # Top right marker
                [self.grid_warped_width - offset_px, self.grid_warped_height - offset_px],  # Bottom right marker
            ]
        )

        # Calculate perspective transform matrix
        self.grid_transform_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        return True

    def calculate_grid_cell_poses(self) -> Optional[list]:
        """
        Calculate poses for the center 3x3 grid cells in base_link coordinates.
        The board is fixed at Z=0.002m (2mm above world zero).
        """
        if self.grid_transform_matrix is None:
            return None

        cell_poses = []

        # Cell size in warped image pixels
        cell_width_px = self.grid_warped_width / self.grid_cols  # Divide by 5
        cell_height_px = self.grid_warped_height / self.grid_rows  # Divide by 5

        # Calculate average angle from grid markers
        if len(self.grid_filtered_poses) >= 4:
            angles = [pose.angle for pose in self.grid_filtered_poses.values()]
            angles_rad = [math.radians(a) for a in angles]
            sin_sum = sum(math.sin(a) for a in angles_rad)
            cos_sum = sum(math.cos(a) for a in angles_rad)
            # Adjust the angle offset here if orientation is wrong: try 0, 90, 180, -90
            avg_angle = math.degrees(math.atan2(sin_sum, cos_sum)) + 0.0
        else:
            avg_angle = 0.0

        # Only calculate for center 3x3 cells (skip first and last row/col)
        for row in range(1, 4):  # Rows 1, 2, 3 (skip 0 and 4)
            for col in range(1, 4):  # Cols 1, 2, 3 (skip 0 and 4)
                # Calculate center of cell in warped image coordinates
                center_x_warped = (col + 0.5) * cell_width_px
                center_y_warped = (row + 0.5) * cell_height_px

                # Transform back to pixel coordinates
                warped_point = np.array([[[center_x_warped, center_y_warped]]], dtype=np.float32)
                pixel_point = cv2.perspectiveTransform(
                    warped_point, np.linalg.inv(self.grid_transform_matrix)
                )
                pixel_x = pixel_point[0][0][0]
                pixel_y = pixel_point[0][0][1]

                # Convert pixel to base_link coordinates with fixed Z
                base_coords = self.pixel_to_base_coords_fixed_z(pixel_x, pixel_y)
                if base_coords is None:
                    continue

                cell_poses.append(
                    MarkerPose(base_coords[0], base_coords[1], avg_angle)
                )

        return cell_poses

    def draw_grid_labels(self, warped_grid, cell_poses):
        """Draw cell labels on warped grid image for center 3x3 cells"""
        if cell_poses is None or len(cell_poses) != 9:
            return

        cell_width_px = self.grid_warped_width / self.grid_cols  # Divide by 5
        cell_height_px = self.grid_warped_height / self.grid_rows  # Divide by 5

        # Draw all cell boundaries (5x5)
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                cell_x = int(col * cell_width_px)
                cell_y = int(row * cell_height_px)
                cv2.rectangle(
                    warped_grid,
                    (cell_x, cell_y),
                    (int((col + 1) * cell_width_px), int((row + 1) * cell_height_px)),
                    (0, 255, 0),
                    1,
                )

        # Draw labels only for center 3x3 cells
        pose_idx = 0
        for row in range(1, 4):  # Rows 1, 2, 3
            for col in range(1, 4):  # Cols 1, 2, 3
                cell_pose = cell_poses[pose_idx]
                pose_idx += 1

                # Calculate center of cell in warped coordinates
                center_x_warped = (col + 0.5) * cell_width_px
                center_y_warped = (row + 0.5) * cell_height_px

                # Draw center point
                cv2.circle(
                    warped_grid,
                    (int(center_x_warped), int(center_y_warped)),
                    3,
                    (255, 0, 0),
                    -1,
                )

                # Draw label with coordinates
                label = f"({cell_pose.x:.3f}, {cell_pose.y:.3f})"
                angle_label = f"{cell_pose.angle:.1f}°"

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
        RADIUS_START = 40
        RADIUS_MIN = 30
        RADIUS_STEP = 4
        MIN_AREA_FRAC = 0.02
        MIN_PIXELS_FLOOR = 10

        # HSV thresholds for blue and red
        BLUE_LOWER = (100, 180, 60)
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

            # If we've reached minimum radius, return NONE
            if r == RADIUS_MIN:
                return 0

        return 0

    def publish_visualization_markers(self, cell_poses, colors, stamp):
        """Publish RViz markers for ArUco centers and colored objects"""
        marker_array = MarkerArray()
        marker_id = 0

        # Publish ArUco marker centers as black cubes (fixed Z=0.002m)
        for marker_idx, pose in self.grid_filtered_poses.items():
            base_coords = self.pixel_to_base_coords_fixed_z(pose.x, pose.y)
            if base_coords is None:
                continue

            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = stamp
            marker.ns = "aruco_centers"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = base_coords[0]
            marker.pose.position.y = base_coords[1]
            marker.pose.position.z = base_coords[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.01
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # Publish colored objects in cells
        if cell_poses is not None and len(cell_poses) == 9:
            for i, (cell_pose, color) in enumerate(zip(cell_poses, colors)):
                if color == 0:  # No color
                    continue

                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = stamp
                marker.ns = "cell_objects"
                marker.id = marker_id
                marker_id += 1

                if color == -1:  # Red -> Cylinder
                    marker.type = Marker.CYLINDER
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif color == 1:  # Blue -> Cube
                    marker.type = Marker.CUBE
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0

                marker.action = Marker.ADD
                marker.pose.position.x = cell_pose.x
                marker.pose.position.y = cell_pose.y
                marker.pose.position.z = 0.01
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.04
                marker.scale.y = 0.04
                marker.scale.z = 0.04
                marker.color.a = 0.8
                marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def image_callback(self, msg):
        """Main image processing callback"""
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return  # Wait for camera info

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

            if ids is not None:
                # Process grid corner markers (4-7)
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in [4, 5, 6, 7]:
                        pose = self.detect_marker_pose(corners[i])
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

                    # Calculate cell poses (fixed Z=0.002m)
                    cell_poses = self.calculate_grid_cell_poses()
                    
                    if cell_poses is not None and len(cell_poses) == 9:
                        # Draw labels on warped grid
                        self.draw_grid_labels(warped_grid, cell_poses)

                        # Publish cell poses and colors
                        grid_pose = GridPose()
                        grid_pose.header = Header()
                        grid_pose.header.stamp = self.get_clock().now().to_msg()
                        grid_pose.header.frame_id = "base_link"

                        cell_width = self.grid_warped_width / self.grid_cols  # Divide by 5
                        cell_height = self.grid_warped_height / self.grid_rows  # Divide by 5
                        
                        colors = []
                        # Only process center 3x3 cells
                        for row in range(1, 4):
                            for col in range(1, 4):
                                cx_warped = (col + 0.5) * cell_width
                                cy_warped = (row + 0.5) * cell_height
                                color_code = self.decide_color_at(
                                    warped_grid, cx_warped, cy_warped
                                )
                                colors.append(color_code)

                                idx = (row - 1) * 3 + (col - 1)  # Map to 0-8 for 3x3 grid
                                cell_msg = Pose2D()
                                cell_msg.x = cell_poses[idx].x
                                cell_msg.y = cell_poses[idx].y
                                cell_msg.theta = cell_poses[idx].angle
                                grid_pose.poses.append(cell_msg)
                                grid_pose.colors.append(color_code)

                        self.cell_poses_pub.publish(grid_pose)

                        # Publish visualization markers
                        self.publish_visualization_markers(
                            cell_poses, colors, grid_pose.header.stamp
                        )

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
        """Toggle logging"""
        self.toggle_log = not self.toggle_log
        status = "enabled" if self.toggle_log else "disabled"
        self.get_logger().info(f"Logging {status} via keyboard node")

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