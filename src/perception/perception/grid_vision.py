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
        if len(self.x_values) == 0:
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
        self.declare_parameter('exposure', 90)
        self.declare_parameter('corner0_x', -0.650)  # Top left X (metres)
        self.declare_parameter('corner0_y', -0.078)  # Top left Y (metres)
        self.declare_parameter('corner1_x', -0.625)  # Bottom left X (metres)
        self.declare_parameter('corner1_y', -0.697)  # Bottom left Y (metres)
        self.declare_parameter('corner2_x', -0.210)  # Top right X (metres)
        self.declare_parameter('corner2_y', -0.060)  # Top right Y (metres)
        self.declare_parameter('corner3_x', -0.197)  # Bottom right X (metres)
        self.declare_parameter('corner3_y', -0.686)  # Bottom right Y (metres)

        self.exposure = self.get_parameter('exposure').get_parameter_value().integer_value
        
        # Store corner positions
        self.corner_positions_m = {
            0: [self.get_parameter('corner0_x').get_parameter_value().double_value,
                self.get_parameter('corner0_y').get_parameter_value().double_value],
            1: [self.get_parameter('corner1_x').get_parameter_value().double_value,
                self.get_parameter('corner1_y').get_parameter_value().double_value],
            2: [self.get_parameter('corner2_x').get_parameter_value().double_value,
                self.get_parameter('corner2_y').get_parameter_value().double_value],
            3: [self.get_parameter('corner3_x').get_parameter_value().double_value,
                self.get_parameter('corner3_y').get_parameter_value().double_value],
        }

        # --- MAFs for outer markers (ID: 0-3) ---
        self.corner_filters: Dict[int, MovingAverageFilter] = {
            0: MovingAverageFilter(5),  # Top left
            1: MovingAverageFilter(5),  # Bottom left
            2: MovingAverageFilter(5),  # Top right
            3: MovingAverageFilter(5),  # Bottom right
        }

        # --- MAFs for grid markers (ID: 4-7) ---
        self.grid_corner_filters: Dict[int, MovingAverageFilter] = {
            4: MovingAverageFilter(5),  # Grid top left
            5: MovingAverageFilter(5),  # Grid bottom left
            6: MovingAverageFilter(5),  # Grid top right
            7: MovingAverageFilter(5),  # Grid bottom right
        }

        # --- Image Warp Parameters ---
        self.filtered_poses: Dict[int, MarkerPose] = {}
        self.grid_filtered_poses: Dict[int, MarkerPose] = {}

        self.transform_matrix = None
        self.warped_size = (438, 627)  # Size of warped outer image (measure manually)
        self.zoom_mm = 35

        self.warped_to_robot_transform = None
        
        # Grid parameters
        self.cell_size_mm = 70.0  # mm
        self.grid_rows = 5
        self.grid_cols = 8
        self.grid_height_mm = 2.0  # mm

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

        self.cell_poses_pub = self.create_publisher(GridPose, "/cell_poses", 10)

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
            return

        request = SetParameters.Request()
        parameter = Parameter(name="rgb_camera.exposure", value=self.exposure).to_parameter_msg()
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

    def detect_marker_pose(self, marker_id, corners) -> MarkerPose:
        """Extract pose (center and orientation) from marker corners"""
        marker_corners = corners[0]

        # Calculate center
        center_x = float(np.mean(marker_corners[:, 0]))
        center_y = float(np.mean(marker_corners[:, 1]))

        # Calculate orientation
        top_left = marker_corners[0]
        top_right = marker_corners[1]
        bottom_right = marker_corners[2]
        bottom_left = marker_corners[3]

        dx = bottom_right[0] - bottom_left[0]
        dy = bottom_right[1] - bottom_left[1]
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)

        # For outer corner markers (0-3), use corner positions
        # For grid markers (4-7), use center positions
        if marker_id in [0, 1, 2, 3]:
            match marker_id:
                case 0:
                    return MarkerPose(top_left[0], top_left[1], angle_deg)
                case 1:
                    return MarkerPose(bottom_left[0], bottom_left[1], angle_deg)
                case 2:
                    return MarkerPose(top_right[0], top_right[1], angle_deg)
                case 3:
                    return MarkerPose(bottom_right[0], bottom_right[1], angle_deg)
        else:
            return MarkerPose(center_x, center_y, angle_deg)

    def update_perspective_transform(self):
        """Update perspective transform matrix based on filtered corner markers"""
        required_markers = [0, 1, 2, 3]
        if not all(marker_id in self.filtered_poses for marker_id in required_markers):
            return False

        src_points = np.float32(
            [
                [self.filtered_poses[0].x, self.filtered_poses[0].y],  # Top left
                [self.filtered_poses[1].x, self.filtered_poses[1].y],  # Bottom left
                [self.filtered_poses[2].x, self.filtered_poses[2].y],  # Top right
                [self.filtered_poses[3].x, self.filtered_poses[3].y],  # Bottom right
            ]
        )

        # Convert corner positions to mm for calculations
        corner0_mm = [self.corner_positions_m[0][0] * 1000, self.corner_positions_m[0][1] * 1000]
        corner2_mm = [self.corner_positions_m[2][0] * 1000, self.corner_positions_m[2][1] * 1000]
        corner1_mm = [self.corner_positions_m[1][0] * 1000, self.corner_positions_m[1][1] * 1000]

        field_width_mm = abs(corner0_mm[0] - corner2_mm[0])
        field_height_mm = abs(corner0_mm[1] - corner1_mm[1])

        # Calculate expansion in pixels
        expansion_x_pixels = (self.zoom_mm / field_width_mm) * self.warped_size[0]
        expansion_y_pixels = (self.zoom_mm / field_height_mm) * self.warped_size[1]

        # Destination points for warped image
        dst_points = np.float32(
            [
                [expansion_x_pixels, expansion_y_pixels],
                [expansion_x_pixels, self.warped_size[1] - expansion_y_pixels],
                [self.warped_size[0] - expansion_x_pixels, expansion_y_pixels],
                [self.warped_size[0] - expansion_x_pixels, self.warped_size[1] - expansion_y_pixels],
            ]
        )

        # Calculate perspective transform matrix
        self.transform_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        # Set up transform from warped image CF to robot CF
        self.setup_warped_to_robot_transform()
        return True

    def setup_warped_to_robot_transform(self):
        """Set up transformation from warped image CF to robot CF"""
        warped_points = np.float32(
            [
                [0, 0],
                [0, self.warped_size[1]],
                [self.warped_size[0], 0],
                [self.warped_size[0], self.warped_size[1]],
            ]
        )

        # Convert to mm for calculations
        corner_mm = {k: [v[0] * 1000, v[1] * 1000] for k, v in self.corner_positions_m.items()}

        # Corresponding robot CF positions (in mm)
        robot_points = np.float32(
            [
                [corner_mm[0][0] - self.zoom_mm, corner_mm[0][1] + self.zoom_mm],
                [corner_mm[1][0] - self.zoom_mm, corner_mm[1][1] - self.zoom_mm],
                [corner_mm[2][0] + self.zoom_mm, corner_mm[2][1] + self.zoom_mm],
                [corner_mm[3][0] + self.zoom_mm, corner_mm[3][1] - self.zoom_mm],
            ]
        )

        self.warped_to_robot_transform = cv2.getPerspectiveTransform(
            warped_points, robot_points
        )

    def transform_to_robot_coordinates(
        self, warped_x: float, warped_y: float, warped_angle: float
    ) -> MarkerPose:
        """Transform position from warped image coordinates to robot coordinates (in mm)"""
        if self.warped_to_robot_transform is None:
            return None

        # Transform position
        warped_point = np.array([[[warped_x, warped_y]]], dtype=np.float32)
        robot_point = cv2.perspectiveTransform(
            warped_point, self.warped_to_robot_transform
        )
        robot_x = robot_point[0][0][0]
        robot_y = robot_point[0][0][1]

        # For angle transformation
        unit_end_x = warped_x + math.cos(warped_angle)
        unit_end_y = warped_y + math.sin(warped_angle)

        warped_unit_end = np.array([[[unit_end_x, unit_end_y]]], dtype=np.float32)
        robot_unit_end = cv2.perspectiveTransform(
            warped_unit_end, self.warped_to_robot_transform
        )

        # Calculate angle in robot CF
        dx_robot = robot_unit_end[0][0][0] - robot_x
        dy_robot = robot_unit_end[0][0][1] - robot_y
        tmp_angle = math.degrees(math.atan2(dy_robot, dx_robot))

        if tmp_angle >= 0.0:
            robot_angle = tmp_angle - 180
        else:
            robot_angle = tmp_angle + 180

        return MarkerPose(robot_x, robot_y, robot_angle)

    def calculate_grid_cell_poses(self) -> Optional[list]:
        """Calculate poses for all 40 grid cells based on the 4 grid corner markers"""
        # Need all 4 grid corner markers
        required_grid_markers = [4, 5, 6, 7]
        if not all(marker_id in self.grid_filtered_poses for marker_id in required_grid_markers):
            return None

        # Get the 4 corner marker poses in robot coordinates (in mm)
        tl = self.grid_filtered_poses[4]  # Top left (row 0, col 0)
        bl = self.grid_filtered_poses[5]  # Bottom left (row 4, col 0)
        tr = self.grid_filtered_poses[6]  # Top right (row 0, col 7)
        br = self.grid_filtered_poses[7]  # Bottom right (row 4, col 7)

        cell_poses = []

        # Calculate poses for all cells using bilinear interpolation
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                # Normalized coordinates (0 to 1)
                u = col / (self.grid_cols - 1) if self.grid_cols > 1 else 0  # Left to right
                v = row / (self.grid_rows - 1) if self.grid_rows > 1 else 0  # Top to bottom

                # Bilinear interpolation for position
                x = (1-u)*(1-v)*tl.x + u*(1-v)*tr.x + (1-u)*v*bl.x + u*v*br.x
                y = (1-u)*(1-v)*tl.y + u*(1-v)*tr.y + (1-u)*v*bl.y + u*v*br.y

                # Bilinear interpolation for angle (using circular mean)
                angles_rad = [math.radians(tl.angle), math.radians(tr.angle), 
                              math.radians(bl.angle), math.radians(br.angle)]
                weights = [(1-u)*(1-v), u*(1-v), (1-u)*v, u*v]
                
                sin_sum = sum(w * math.sin(a) for w, a in zip(weights, angles_rad))
                cos_sum = sum(w * math.cos(a) for w, a in zip(weights, angles_rad))
                angle = math.degrees(math.atan2(sin_sum, cos_sum))

                cell_poses.append(MarkerPose(x, y, angle))

        return cell_poses

    def draw_markers_and_region(self, cv_image, corners, ids):
        """Draw detected markers and the region between corner markers"""
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Draw filtered outer corner positions
        corner_points = []
        for marker_id in [0, 1, 2, 3]:
            if marker_id in self.filtered_poses:
                pose = self.filtered_poses[marker_id]
                center = (int(pose.x), int(pose.y))
                cv2.circle(cv_image, center, 8, (0, 255, 0), -1)

                line_length = 30
                end_x = int(pose.x + line_length * math.cos(math.radians(pose.angle)))
                end_y = int(pose.y + line_length * math.sin(math.radians(pose.angle)))
                cv2.arrowedLine(cv_image, center, (end_x, end_y), (255, 0, 0), 2)

                text = f"ID{marker_id}"
                cv2.putText(
                    cv_image, text, (center[0] - 30, center[1] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                )
                corner_points.append([pose.x, pose.y])

        # Draw the region between corner markers
        if len(corner_points) == 4:
            points = np.array(
                [corner_points[0], corner_points[2], corner_points[3], corner_points[1]],
                dtype=np.int32,
            )
            cv2.polylines(cv_image, [points], True, (0, 255, 255), 2)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

            if ids is not None:
                # Process outer corner markers (0-3)
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in [0, 1, 2, 3]:
                        pose = self.detect_marker_pose(marker_id, corners[i])
                        self.corner_filters[marker_id].add_sample(pose)

                        if self.corner_filters[marker_id].is_ready():
                            filtered_pose = self.corner_filters[marker_id].get_filtered_pose()
                            if filtered_pose:
                                self.filtered_poses[marker_id] = filtered_pose

                # Update perspective transform
                if self.update_perspective_transform():
                    warped_image = cv2.warpPerspective(
                        cv_image, self.transform_matrix, self.warped_size
                    )

                    # Detect grid corner markers in warped image
                    warped_gray = cv2.cvtColor(warped_image, cv2.COLOR_BGR2GRAY)
                    warped_corners, warped_ids, _ = cv2.aruco.detectMarkers(
                        warped_gray, self.aruco_dict, parameters=self.detector_params
                    )

                    if warped_ids is not None:
                        # Process grid corner markers (4-7)
                        for i, marker_id in enumerate(warped_ids.flatten()):
                            if marker_id in [4, 5, 6, 7]:
                                warped_pose = self.detect_marker_pose(
                                    marker_id, warped_corners[i]
                                )

                                # Transform to robot coordinates
                                robot_pose = self.transform_to_robot_coordinates(
                                    warped_pose.x,
                                    warped_pose.y,
                                    math.radians(warped_pose.angle),
                                )

                                if robot_pose is not None:
                                    self.grid_corner_filters[marker_id].add_sample(robot_pose)

                                    if self.grid_corner_filters[marker_id].is_ready():
                                        filtered_pose = self.grid_corner_filters[marker_id].get_filtered_pose()
                                        if filtered_pose:
                                            self.grid_filtered_poses[marker_id] = filtered_pose

                        # Draw grid markers on warped image
                        cv2.aruco.drawDetectedMarkers(warped_image, warped_corners, warped_ids)
                        
                        for marker_id in [4, 5, 6, 7]:
                            if marker_id in self.grid_filtered_poses:
                                pose = self.grid_filtered_poses[marker_id]
                                text = f"Grid{marker_id}: ({pose.x:.0f}, {pose.y:.0f})mm"
                                cv2.putText(
                                    warped_image, text, (10, 30 + (marker_id-4)*25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                                )

                    # Calculate and publish cell poses
                    cell_poses = self.calculate_grid_cell_poses()
                    if cell_poses is not None:
                        grid_pose = GridPose()
                        grid_pose.header = Header()
                        grid_pose.header.stamp = self.get_clock().now().to_msg()
                        grid_pose.header.frame_id = "robot_base"

                        for cell_pose in cell_poses:
                            cell_msg = Pose2D()
                            # Convert from mm to metres
                            cell_msg.x = cell_pose.x / 1000.0
                            cell_msg.y = cell_pose.y / 1000.0
                            cell_msg.theta = cell_pose.angle
                            grid_pose.poses.append(cell_msg)

                        self.cell_poses_pub.publish(grid_pose)

                        if self.toggle_log:
                            self.get_logger().info(
                                f"Published {len(cell_poses)} cell poses"
                            )

                    cv2.imshow("Warped Region", warped_image)

                # Draw markers and region on original image
                self.draw_markers_and_region(cv_image, corners, ids)

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