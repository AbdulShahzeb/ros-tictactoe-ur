#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from custom_interface.msg import Player, Players
from geometry_msgs.msg import TransformStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf_transformations import quaternion_from_euler
import math

# Reusable helper (defines static camera→world TF and constant height)
from perception_cv import broadcast_camera_to_world, pixel_to_board_coords, CAM_HEIGHT_M, BOARD_H_MM, BOARD_W_MM, BOARD_W_MM

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.process_every_n = 15   # process 1 out of every 5 frames
        self.frame_count = 0

        # --- Parameters ---
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('show_image', True)
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value

        # --- ArUco Setup ---
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # --- Subscribers ---
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.color_sub = self.create_subscription(Image, color_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo,
                                                        '/camera/camera/aligned_depth_to_color/camera_info',
                                                        self.camera_info_callback, 10)

        # --- Publishers ---
        self.warped_pub = self.create_publisher(Image, 'board/warped_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers', 10)

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        broadcast_camera_to_world(self)

        # --- Intrinsics ---
        self.fx = self.fy = self.cx = self.cy = None

        self.get_logger().info("✅ ArUco detector node started (no depth mode).")

    # ----------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    # ----------------------------------------------------------
    def image_callback(self, msg: Image):
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return
        
        # # --- Throttle processing ---
        # self.frame_count += 1
        # if self.frame_count % self.process_every_n != 0:
        #     return  # skip this frame

        # timestamp 
        stamp_now = self.get_clock().now().to_msg()

        # --- ArUco detection ---
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)
        players_msg = Players()

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # Draw IDs on image
            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                center_px = tuple(c.mean(axis=0).astype(int))
                cv2.putText(frame, str(marker_id), center_px,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # Detect board (markers 0–3)
            board_pts_2d = self.get_board_corners_2d(ids, corners)
            if len(board_pts_2d) == 4:
                self.broadcast_board_frame(board_pts_2d)
                H = self.publish_warped_board(frame, board_pts_2d, stamp_now)

                if H is not None:
                    self.publish_aruco_markers(board_pts_2d, H, stamp_now)


        if self.show_image:
            cv2.imshow('ArUco Detection', frame)
            cv2.waitKey(1)

    # ----------------------------------------------------------
    def get_board_corners_2d(self, ids, corners):
        """Return ordered 2D corners for markers 0–3."""
        board_pts = {}
        for i, marker_id in enumerate(ids):
            if marker_id in [0, 1, 2, 3]:
                c = corners[i][0]
                board_pts[marker_id] = np.mean(c, axis=0)
        if len(board_pts) != 4:
            return []
        # Order: TL, TR, BR, BL
        return [board_pts[i] for i in [0, 1, 2, 3]]

    # ----------------------------------------------------------
    def broadcast_board_frame(self, board_points_2d):
        """Compute board_frame pose using all four corners (robust yaw) and constant height."""
        self.get_logger().debug("Broadcasting board_frame TF.")

        if len(board_points_2d) != 4:
            self.get_logger().warn("Insufficient board corners for TF broadcast.")
            return

        # Board corners: [TL, TR, BR, BL] in pixel coordinates
        pts = np.array(board_points_2d, dtype=float)
        TL, TR, BR, BL = pts

        # ===============================================================
        #   Translation: board center (in meters)
        # ===============================================================
        center_px = np.mean(pts, axis=0)
        u, v = center_px
        z = CAM_HEIGHT_M
        x_m = ((u - self.cx) * z) / self.fx 
        y_m = ((v - self.cy) * z) / self.fy 

        # ===============================================================
        #   Orientation: use all 4 sides for a robust yaw
        # ===============================================================
        # Edge vectors (in image/pixel space)
        v_top = TR - TL
        v_bottom = BR - BL
        v_left = BL - TL
        v_right = BR - TR

        # Average horizontal and vertical directions
        avg_vec_x = (v_top + v_bottom) / 2.0
        avg_vec_y = (v_left + v_right) / 2.0

        # Normalize both
        avg_vec_x /= np.linalg.norm(avg_vec_x)
        avg_vec_y /= np.linalg.norm(avg_vec_y)

        # Ensure a right-handed coordinate system (cross > 0)
        cross = np.cross(avg_vec_x, avg_vec_y)
        if cross < 0:
            avg_vec_x = -avg_vec_x

        # Compute yaw (camera y-down, so flip sign for world y-up)
        yaw_cam = np.arctan2(avg_vec_x[1], avg_vec_x[0])
        yaw_world = yaw_cam

        # ===============================================================
        #   Build and broadcast transform
        # ===============================================================
        q = quaternion_from_euler(math.pi, 0.0, yaw_world)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_frame'
        t.child_frame_id = 'board_frame'

        t.transform.translation.x = float(x_m)
        t.transform.translation.y = float(y_m)  # flip to match world-up
        t.transform.translation.z = float(CAM_HEIGHT_M)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


    # ----------------------------------------------------------
    def publish_warped_board(self, frame, board_points_2d, stamp):
        """Warp board to top-down view for debugging (adds 50 mm border for ArUco tags)."""
        try:
            # Physical board size (in mm)
            
            aruco_half_mm = 70.0  # 100 mm tags → 50 mm offset per side
            # NOTE: slightly cutting off corners for some reason. Not sure why.

            # Reason: We are using the vector lines of the edges to compute the offset,
            # this means we can assume 1mm in pixel space is the same as 1mm in real world space

            # Better approach: use the aruco top corner points instead of the centroids. 
            # Would need to edit get_board_corners_2d to return all 4 corners per marker.

            if len(board_points_2d) != 4:
                self.get_logger().warn("Not enough board corners for warp.")
                return

            # Extract detected marker centers
            tl, tr, br, bl = [np.array(p, dtype=np.float32) for p in board_points_2d]

            # --- Apply outward 50 mm offsets ---
            # Assume board coordinate order: TL(0), TR(1), BR(2), BL(3)
            # Offsets in pixel space proportional to line directions
            # Compute edge direction vectors
            vec_top = tr - tl
            vec_left = bl - tl
            norm_top = vec_top / np.linalg.norm(vec_top)
            norm_left = vec_left / np.linalg.norm(vec_left)

            # Per-corner outward offset = combination of normalised top/left vectors
            # For each corner: ±x (along top edge), ±y (along left edge)
            offsets = {
                'tl': -norm_top * aruco_half_mm - norm_left * aruco_half_mm,
                'tr':  norm_top * aruco_half_mm - norm_left * aruco_half_mm,
                'br':  norm_top * aruco_half_mm + norm_left * aruco_half_mm,
                'bl': -norm_top * aruco_half_mm + norm_left * aruco_half_mm,
            }

            src = np.array([
                tl + offsets['tl'],
                tr + offsets['tr'],
                br + offsets['br'],
                bl + offsets['bl'],
            ], dtype=np.float32)

            # --- Destination points (include same margin in warp space) ---
            dst = np.array([
                [0, 0],
                [BOARD_W_MM - 1, 0],
                [BOARD_W_MM - 1, BOARD_H_MM - 1],
                [0, BOARD_H_MM - 1]
            ], dtype=np.float32)

            # Compute homography and warp
            H, _ = cv2.findHomography(src, dst)
            warped = cv2.warpPerspective(frame, H, (int(BOARD_W_MM), int(BOARD_H_MM)))

            warped_msg = self.bridge.cv2_to_imgmsg(warped, encoding='bgr8')
            warped_msg.header.stamp = stamp
            self.warped_pub.publish(warped_msg)

            if self.show_image:
                cv2.imshow('Board Warped', warped)
            
            return H

        except Exception as e:
            self.get_logger().warn(f"Warp failed: {e}")


    # ----------------------------------------------------------
    def publish_aruco_markers(self, board_points_2d, H, stamp):
        """
        Publish ArUco detections (board corners) as RViz markers
        in the board_frame using warped coordinates.
        Expects ordered board_points_2d: [TL, TR, BR, BL].
        """
        if len(board_points_2d) != 4:
            self.get_logger().warn("Not enough board points to publish markers.")
            return

        # Convert to shape (N,1,2) for cv2.perspectiveTransform
        pts = np.array(board_points_2d, dtype=np.float32).reshape(-1, 1, 2)
        warped_pts = cv2.perspectiveTransform(pts, H).reshape(-1, 2)

        marker_array = MarkerArray()
        img_h, img_w = 390, 765  # warped board size in pixels (same as warp)
        label_order = ["TL", "TR", "BR", "BL"]

        for i, (label, pt) in enumerate(zip(label_order, warped_pts)):
            # self.get_logger().info(f"Marker {label} at pixel coords ({pt[0]:.1f}, {pt[1]:.1f})")
            x_m, y_m, z_m = pixel_to_board_coords(
                x_px=pt[0],
                y_px=pt[1],
                img_w=img_w,
                img_h=img_h,
                z_offset=0.0,
            )
            # self.get_logger().info(f"Marker {label} at board coords ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})")

            # Cube marker
            cube = Marker()
            cube.header.frame_id = "board_frame"
            cube.header.stamp = stamp
            cube.ns = "board_corners"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(x_m)
            cube.pose.position.y = float(y_m)
            cube.pose.position.z = float(z_m)
            cube.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            cube.scale.x = cube.scale.y = 0.1
            cube.scale.z = 0.01
            cube.color.r = 0.0
            cube.color.g = 0.0
            cube.color.b = 1.0
            cube.color.a = 1.0
            marker_array.markers.append(cube)

            # Text label
            text = Marker()
            text.header.frame_id = "board_frame"
            text.header.stamp = stamp
            text.ns = "corner_text"
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x_m)
            text.pose.position.y = float(y_m)
            text.pose.position.z = float(z_m + 0.05)
            text.scale.z = 0.06
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = label
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
