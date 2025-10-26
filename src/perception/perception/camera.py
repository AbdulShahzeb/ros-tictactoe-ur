#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
from helper.msg import CellPose, CellPoseArray


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Parameters
        self.GRID_WIDTH = 0.38  # meters - INNER playable area width
        self.GRID_HEIGHT = 0.24  # meters - INNER playable area height
        self.BORDER_WIDTH = 0.01  # meters - thickness of black border
        self.GRID_COLS = 8
        self.GRID_ROWS = 5
        self.BLACK_THRESH = 120

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # TF2 for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )

        self.pc_sub = self.create_subscription(
            PointCloud2, "/camera/depth/color/points", self.pointcloud_callback, 10
        )

        # Publisher for cell poses
        self.cells_pub = self.create_publisher(CellPoseArray, "cell_poses", 10)

        # Store latest point cloud for depth lookup
        self.latest_pointcloud = None

        self.get_logger().info("Camera node initialized")

    def pointcloud_callback(self, msg):
        """Store latest point cloud for depth lookup"""
        self.latest_pointcloud = msg

    def image_callback(self, msg):
        """Process incoming RGB image"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Detect grid
            grid_corners_px, grid_angle_image = self.detect_grid(cv_image)

            if grid_corners_px is not None:
                # Detect cell centers in image
                cell_centers_px = self.detect_cell_centers(cv_image, grid_corners_px)

                # Transform to robot base frame using point cloud depth
                cell_poses_base = self.transform_to_robot_base(
                    cell_centers_px, grid_angle_image, msg.header.stamp
                )

                if cell_poses_base is not None:
                    # Publish results
                    self.publish_cell_poses(cell_poses_base, msg.header.stamp)

                # Visualize
                self.visualize_detection(cv_image, grid_corners_px, cell_centers_px)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def detect_grid(self, image):
        """
        Detect the 8x5 grid with thick black border
        Returns: (corners_px, angle) or (None, None)
        corners_px: 4x2 array of [x,y] in pixel coordinates (top-left, top-right, bottom-right, bottom-left)
        angle: angle of top edge in image space (radians)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Threshold to find black border
        _, thresh = cv2.threshold(gray, self.BLACK_THRESH, 255, cv2.THRESH_BINARY_INV)

        # Find contours
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, None

        # Find largest rectangular contour (the grid border)
        largest_contour = max(contours, key=cv2.contourArea)

        # Approximate to polygon
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        if len(approx) != 4:
            self.get_logger().warn("Could not find 4-corner grid")
            return None, None

        # Order corners: top-left, top-right, bottom-right, bottom-left
        corners = self.order_corners(approx.reshape(4, 2))

        # Calculate grid angle (angle of top edge in image)
        dx = corners[1][0] - corners[0][0]
        dy = corners[1][1] - corners[0][1]
        angle = np.arctan2(dy, dx)

        return corners, angle

    def order_corners(self, pts):
        """Order points as: top-left, top-right, bottom-right, bottom-left"""
        rect = np.zeros((4, 2), dtype=np.float32)

        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        return rect

    def detect_cell_centers(self, image, grid_corners):
        """
        Detect centers of each cell in the grid
        grid_corners: outer corners of the border (detected rectangle)
        Returns: Nx2 array of (x, y) pixel coordinates for each cell center
        """
        # Inset the corners by border width to get inner playable area
        inner_corners = self.inset_corners(grid_corners, self.BORDER_WIDTH)

        cell_centers = []

        # Subdivide inner grid area to find cell centers
        for row in range(self.GRID_ROWS):
            for col in range(self.GRID_COLS):
                # Cell center position (0.5 offset for center)
                u = (col + 0.5) / self.GRID_COLS
                v = (row + 0.5) / self.GRID_ROWS

                # Bilinear interpolation on INNER corners
                top = (1 - u) * inner_corners[0] + u * inner_corners[1]
                bottom = (1 - u) * inner_corners[3] + u * inner_corners[2]
                center = (1 - v) * top + v * bottom

                cell_centers.append(center)

        return np.array(cell_centers)

    def inset_corners(self, corners, border_width_m):
        """
        Inset corner points by border width to get inner playable area
        corners: 4x2 array [top-left, top-right, bottom-right, bottom-left]
        border_width_m: border width in meters
        Returns: 4x2 array of inset corners
        """
        # Calculate the scale factor to convert pixels to approximate meters
        # Use the detected width and known real-world width
        detected_width_px = np.linalg.norm(corners[1] - corners[0])
        detected_height_px = np.linalg.norm(corners[3] - corners[0])

        # Total dimensions including border
        total_width_m = self.GRID_WIDTH + 2 * border_width_m
        total_height_m = self.GRID_HEIGHT + 2 * border_width_m

        # Pixels per meter
        px_per_m_width = detected_width_px / total_width_m
        px_per_m_height = detected_height_px / total_height_m

        # Border width in pixels
        border_width_px_h = border_width_m * px_per_m_width
        border_width_px_v = border_width_m * px_per_m_height

        # Calculate inset corners
        # Top edge: move down
        # Bottom edge: move up
        # Left edge: move right
        # Right edge: move left

        # Get edge vectors
        top_vec = corners[1] - corners[0]
        right_vec = corners[2] - corners[1]
        bottom_vec = corners[2] - corners[3]
        left_vec = corners[3] - corners[0]

        # Normalize and scale by border width
        top_inset = (top_vec / np.linalg.norm(top_vec)) * border_width_px_h
        right_inset = (right_vec / np.linalg.norm(right_vec)) * border_width_px_v
        bottom_inset = (bottom_vec / np.linalg.norm(bottom_vec)) * border_width_px_h
        left_inset = (left_vec / np.linalg.norm(left_vec)) * border_width_px_v

        # Perpendicular insets (moving inward)
        top_perp = (
            np.array([-top_vec[1], top_vec[0]])
            / np.linalg.norm(top_vec)
            * border_width_px_v
        )
        right_perp = (
            np.array([-right_vec[1], right_vec[0]])
            / np.linalg.norm(right_vec)
            * border_width_px_h
        )

        # Calculate inner corners
        inner_corners = np.zeros_like(corners)
        inner_corners[0] = corners[0] + left_inset + top_perp  # top-left
        inner_corners[1] = corners[1] - right_inset + top_perp  # top-right
        inner_corners[2] = corners[2] - right_inset - top_perp  # bottom-right
        inner_corners[3] = corners[3] + left_inset - top_perp  # bottom-left

        return inner_corners

    def get_3d_point_from_pointcloud(self, x_px, y_px):
        """
        Get 3D point in camera frame from pixel coordinates using point cloud
        Returns: (x, y, z) in camera frame or None if no valid depth
        """
        if self.latest_pointcloud is None:
            return None

        # Convert point cloud to list of points
        pc_data = pc2.read_points(
            self.latest_pointcloud,
            field_names=("x", "y", "z"),
            skip_nans=True,
            uvs=[(int(x_px), int(y_px))],
        )

        points = list(pc_data)
        if len(points) == 0:
            return None

        point = points[0]
        return np.array([point[0], point[1], point[2]])

    def transform_to_robot_base(self, points_px, grid_angle_image, timestamp):
        """
        Transform pixel coordinates to robot base frame using point cloud
        points_px: Nx2 array of pixel coordinates
        grid_angle_image: angle of grid in image space
        Returns: List of (x, y, angle) in robot base frame
        """
        if self.latest_pointcloud is None:
            self.get_logger().warn("No point cloud available")
            return None

        try:
            # Lookup transform from camera to robot base
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            cell_poses = []

            # Get 3D positions for all cells
            points_3d_camera = []
            for point_px in points_px:
                point_3d = self.get_3d_point_from_pointcloud(point_px[0], point_px[1])
                if point_3d is not None:
                    points_3d_camera.append(point_3d)
                else:
                    self.get_logger().warn(f"No depth at pixel {point_px}")
                    return None

            points_3d_camera = np.array(points_3d_camera)

            # Transform points to base frame
            t = transform.transform.translation
            r = transform.transform.rotation

            # Convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(r)
            translation = np.array([t.x, t.y, t.z])

            # Transform all points
            points_3d_base = (rotation_matrix @ points_3d_camera.T).T + translation

            # Calculate grid angle in base frame
            # Use two corner points to determine orientation
            grid_corners_3d_camera = []
            for i in [0, 1]:  # Top-left and top-right corners
                row, col = 0, i * (self.GRID_COLS - 1)
                idx = row * self.GRID_COLS + col
                grid_corners_3d_camera.append(points_3d_camera[idx])

            grid_corners_3d_camera = np.array(grid_corners_3d_camera)
            grid_corners_3d_base = (
                rotation_matrix @ grid_corners_3d_camera.T
            ).T + translation

            # Calculate angle in base frame (yaw)
            dx = grid_corners_3d_base[1][0] - grid_corners_3d_base[0][0]
            dy = grid_corners_3d_base[1][1] - grid_corners_3d_base[0][1]
            grid_angle_base = np.arctan2(dy, dx)

            # Package results
            for point_base in points_3d_base:
                cell_poses.append(
                    {"x": point_base[0], "y": point_base[1], "angle": grid_angle_base}
                )

            return cell_poses

        except Exception as e:
            self.get_logger().error(f"Transform failed: {str(e)}")
            return None

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to 3x3 rotation matrix"""
        x, y, z, w = q.x, q.y, q.z, q.w

        return np.array(
            [
                [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
            ]
        )

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
        self.get_logger().info(f"Published {len(cell_poses)} cell poses")

    def visualize_detection(self, image, grid_corners, cell_centers):
        """Draw detection results on image for debugging"""
        vis = image.copy()

        # Draw grid corners
        for corner in grid_corners:
            cv2.circle(vis, tuple(corner.astype(int)), 5, (0, 255, 0), -1)

        # Draw grid border
        cv2.polylines(vis, [grid_corners.astype(int)], True, (0, 255, 0), 2)

        # Draw cell centers
        for center in cell_centers:
            cv2.circle(vis, tuple(center.astype(int)), 3, (255, 0, 0), -1)

        cv2.imshow("Grid Detection", vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

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
