#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Pose2D, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
from tf_transformations import quaternion_from_euler


class ArucoVisionNode(Node):
    # ===============================================================
    #   Setup
    # ===============================================================
    def __init__(self):
        super().__init__("aruco_vision_node")

        # --- CV2 Setup ---
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()

        # --- Declare and get ROS parameters ---
        self.declare_parameter("exposure", 180)
        self.exposure = (
            self.get_parameter("exposure").get_parameter_value().integer_value
        )

        # --- Camera intrinsics ---
        self.fx = 911.52001953125
        self.fy = 646.7098388671875
        self.cx = 911.445068359375
        self.cy = 373.8798522949219

        # --- Camera to base_link transform
        self.cam_to_base_translation = (0.850, 0.231, 0.912)  # (x, y, z) in meters
        self.cam_to_base_rpy = (math.pi, 0, math.pi)  # (roll, pitch, yaw)

        # Grid parameters
        self.cell_size_mm = 70.0
        self.grid_size_mm = 210.0

        # --- TF Broadcaster ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.camera_broadcaster = self.create_timer(3.0, self.broadcast_camera_transform)

        # --- Publishers and Subscribers ---
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

        self.enable_prelim_cv_sub = self.create_subscription(
            Bool, "/kb/enable_prelim_cv", self.enable_prelim_cv_callback, 10
        )
        self.prelim_cv_enabled = False

        self.shutdown_sub = self.create_subscription(
            Bool, "/kb/shutdown", self.shutdown_callback, 10
        )
        self.shutdown_requested = False

        self.warped_pub = self.create_publisher(Image, "/perception/warped_grid", 10)

        self.aruco_pub = self.create_publisher(
            MarkerArray, "/perception/aruco_markers", 10
        )

        # --- Set Camera Exposure ---
        self.exposure_param_timer = self.create_timer(1.0, self.setup_camera_parameters)
        self.get_logger().info("Aruco Vision node started")

    # ===============================================================
    #   Helper Functions
    # ===============================================================
    def broadcast_camera_transform(self):
        """Broadcast static transform from camera to world frame"""
        child_frame_ids = ["camera_rgb_optical_frame", "camera_color_optical_frame", "camera_color_frame", "camera_link"]
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        t.header.frame_id = "world"

        t.transform.translation.x = self.cam_to_base_translation[0]
        t.transform.translation.y = self.cam_to_base_translation[1]
        t.transform.translation.z = self.cam_to_base_translation[2]

        q = quaternion_from_euler(*self.cam_to_base_rpy)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        for child_frame_id in child_frame_ids:
            t.child_frame_id = child_frame_id
            self.static_tf_broadcaster.sendTransform(t)

    def pixel_to_grid_coords(
        self, x_px: float, y_px: float, img_w: int, img_h: int
    ):
        """Convert pixel coordinates in warped grid to grid frame coordinates"""
        pixel_to_metres = 0.001
        x_grid = (x_px - img_w / 2.0) * pixel_to_metres
        y_grid = -(y_px - img_h / 2.0) * pixel_to_metres
        return x_grid, y_grid

    # ===============================================================
    #   Callbacks
    # ===============================================================
    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics from CameraInfo message"""
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def setup_camera_parameters(self):
        exposure_client = self.create_client(
            SetParameters, "/camera/camera/set_parameters"
        )
        
        if not exposure_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "Service /camera/camera/set_parameters not available"
            )
            if self.exposure_param_timer:
                self.exposure_param_timer.cancel()
                self.exposure_param_timer = None
            return

        request = SetParameters.Request()
        parameter = Parameter(
            name="rgb_camera.exposure", value=self.exposure
        ).to_parameter_msg()
        request.parameters = [parameter]
        future = exposure_client.call_async(request)
        future.add_done_callback(self._on_exposure_set)


    def _on_exposure_set(self, future):
        """Callback when exposure parameter is set"""
        try:
            result = future.result()
            if result.results[0].successful:
                self.get_logger().info(
                    f"Successfully set parameter rgb_camera.exposure to {self.exposure} ms"
                )
            else:
                self.get_logger().error(
                    f"Failed to set parameter: {result.results[0].reason}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally:
            if self.exposure_param_timer:
                self.exposure_param_timer.cancel()
                self.exposure_param_timer = None

    def enable_prelim_cv_callback(self, msg):
        """Enable preliminary CV window display"""
        self.prelim_cv_enabled = not self.prelim_cv_enabled
        if not self.prelim_cv_enabled:
            cv2.destroyWindow("Aruco Detection")

    # ===============================================================
    #   ArUco detection
    # ===============================================================

    def image_callback(self, msg):
        """Main image processing callback"""
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return

        time = self.get_clock().now().to_msg()
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params
        )

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # Draw IDs
            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                cx = tuple(c.mean(axis=0).astype(int))
                cv2.putText(
                    frame,
                    str(marker_id),
                    cx,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            # Detect grid
            grid_pts = self.get_grid_corners(ids, corners)
            if len(grid_pts) == 4:
                self.broadcast_grid_frame(grid_pts, time)
                H = self.publish_warped_grid(frame, grid_pts, time)
                if H is not None:
                    self.publish_aruco_markers(grid_pts, H, time)
                else:
                    self.get_logger().warn("Homography matrix is None")

        if self.prelim_cv_enabled:
            cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

    def get_grid_corners(self, ids, corners):
        """Get 2D corners of grid"""
        grid_ids = [4, 5, 6, 7]  # Top-left, Bottom-left, Top-right, Bottom-right
        grid_pts = {}
        for i, marker_id in enumerate(ids):
            if marker_id in grid_ids:
                c = corners[i][0]
                grid_pts[marker_id] = np.mean(c, axis=0)
        if len(grid_pts) == 4:
            return [grid_pts[id] for id in grid_ids]
        return []

    def broadcast_grid_frame(self, grid_pts, time):
        """Compute and broadcast grid frame based on detected corners"""
        if len(grid_pts) != 4:
            self.get_logger().warn("Not enough grid corners detected")
            return

        pts = np.array(grid_pts, dtype=float)
        TL, BL, TR, BR = pts

        center = np.mean(pts, axis=0)
        u, v = center
        z = self.cam_to_base_translation[2]
        x_m = ((u - self.cx) * z) / self.fx
        y_m = ((v - self.cy) * z) / self.fy

        # Orientation (use all 4 vectors)
        v_top = TR - TL
        v_bottom = BR - BL
        v_left = BL - TL
        v_right = BR - TR

        avg_vec_x = (v_top + v_bottom) / 2
        avg_vec_y = (v_left + v_right) / 2
        avg_vec_x /= np.linalg.norm(avg_vec_x)
        avg_vec_y /= np.linalg.norm(avg_vec_y)

        cross = np.cross(avg_vec_x, avg_vec_y)
        if cross < 0:
            avg_vec_x = -avg_vec_x

        yaw = math.atan2(avg_vec_x[1], avg_vec_x[0])
        q = quaternion_from_euler(math.pi, 0, yaw)

        # Build transform
        t = TransformStamped()
        # Use latest time for transform
        t.header.stamp = time
        t.header.frame_id = "camera_rgb_optical_frame"
        t.child_frame_id = "grid_frame"
        t.transform.translation.x = float(x_m)
        t.transform.translation.y = float(y_m)
        t.transform.translation.z = float(self.cam_to_base_translation[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_warped_grid(self, frame, grid_pts, time):
        """Warp grid to top-down view and publish"""
        try:
            if len(grid_pts) != 4:
                self.get_logger().warn("Not enough grid corners for warping")
                return None

            tl, bl, tr, br = [np.array(p, dtype=float) for p in grid_pts]

            # Source points in image
            src = np.array([tl, bl, tr, br], dtype=float)

            # Destination points (top-down square)
            crop_mm = -self.cell_size_mm / 2.0
            dst = np.array(
                [
                    [crop_mm, crop_mm],
                    [crop_mm, self.grid_size_mm - 1 - crop_mm],
                    [self.grid_size_mm - 1 - crop_mm, crop_mm],
                    [self.grid_size_mm - 1 - crop_mm, self.grid_size_mm - 1 - crop_mm],
                ],
                dtype=float,
            )

            H, _ = cv2.findHomography(src, dst)
            warped = cv2.warpPerspective(frame, H, (int(self.grid_size_mm), int(self.grid_size_mm)))
            warped_msg = self.bridge.cv2_to_imgmsg(warped, "bgr8")
            warped_msg.header.stamp = time
            self.warped_pub.publish(warped_msg)
            cv2.imshow("Warped Grid", warped)
            return H
        except Exception as e:
            self.get_logger().warn(f"Error warping grid: {e}")

    def publish_aruco_markers(self, grid_pts, H, time):
        """Publish ArUco marker positions as RViz markers"""
        if len(grid_pts) != 4:
            self.get_logger().warn("Not enough grid corners for marker publishing")
            return

        # Convert to shape (N,1,2) for cv2.perspectiveTransform
        pts = np.array(grid_pts, dtype=float).reshape(-1, 1, 2)
        warped_pts = cv2.perspectiveTransform(pts, H).reshape(-1, 2)

        marker_array = MarkerArray()
        img_h, img_w = self.grid_size_mm, self.grid_size_mm
        labels = ["TL", "BL", "TR", "BR"]

        for i, (label, pt) in enumerate(zip(labels, warped_pts)):
            x_m, y_m = self.pixel_to_grid_coords(
                x_px=float(pt[0]),
                y_px=float(pt[1]),
                img_w=img_w,
                img_h=img_h,
            )

            cube = Marker()
            cube.header.frame_id = "grid_frame"
            cube.header.stamp = time
            cube.ns = "aruco_markers"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(x_m)
            cube.pose.position.y = float(y_m)
            cube.pose.position.z = float(0.002)
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = self.cell_size_mm / 1000.0
            cube.scale.z = 0.01
            cube.color.r = 0.0
            cube.color.g = 0.0
            cube.color.b = 0.0
            cube.color.a = 1.0
            marker_array.markers.append(cube)

            text = Marker()
            text.header.frame_id = "grid_frame"
            text.header.stamp = time
            text.ns = "aruco_labels"
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x_m)
            text.pose.position.y = float(y_m)
            text.pose.position.z = float(0.002 + 0.05)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.06
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = label
            marker_array.markers.append(text)

        self.aruco_pub.publish(marker_array)

    # ===============================================================
    #   Other Callbacks
    # ===============================================================
    def log_callback(self, msg):
        self.toggle_log = not self.toggle_log
        status = "enabled" if self.toggle_log else "disabled"
        self.get_logger().info(f"Logging {status} via keyboard node")

    def shutdown_callback(self, msg):
        """Shuts down the Grid Vision Node"""
        self.get_logger().info("Received shutdown signal. Exiting...")
        self.shutdown_requested = True


def main(args=None):
    rclpy.init(args=args)
    node = ArucoVisionNode()

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
