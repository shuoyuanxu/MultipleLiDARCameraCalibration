#!/usr/bin/env python3
"""Repair recorded frames/timestamps and build LiDAR/camera QA products."""

from __future__ import annotations

import copy
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField

from .calibration_io import load_sensor_extrinsics


OUTPUT_CLOUD_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)


@dataclass
class CloudArrays:
    xyz: np.ndarray
    intensity: np.ndarray
    receipt_ns: int
    source_stamp_ns: int


@dataclass
class CameraFrame:
    bgr: np.ndarray
    k: np.ndarray
    receipt_ns: int


def _point_field_offset(msg: PointCloud2, name: str) -> Optional[int]:
    for field in msg.fields:
        if field.name == name:
            if field.datatype != PointField.FLOAT32 or field.count != 1:
                raise ValueError(f"Point field {name!r} must be one FLOAT32 value")
            return int(field.offset)
    return None


def cloud_to_arrays(msg: PointCloud2, receipt_ns: int) -> CloudArrays:
    """Read xyz and optional intensity without assuming a packed PCL layout."""
    if msg.point_step <= 0:
        raise ValueError("PointCloud2 point_step is zero")
    endian = ">" if msg.is_bigendian else "<"
    x_offset = _point_field_offset(msg, "x")
    y_offset = _point_field_offset(msg, "y")
    z_offset = _point_field_offset(msg, "z")
    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError("PointCloud2 needs x, y, and z FLOAT32 fields")
    intensity_offset = _point_field_offset(msg, "intensity")

    names = ["x", "y", "z"]
    offsets = [x_offset, y_offset, z_offset]
    formats = [endian + "f4"] * 3
    if intensity_offset is not None:
        names.append("intensity")
        offsets.append(intensity_offset)
        formats.append(endian + "f4")

    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(msg.point_step),
        }
    )
    count = len(msg.data) // int(msg.point_step)
    raw = np.frombuffer(msg.data, dtype=dtype, count=count)
    xyz = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32, copy=False)
    if intensity_offset is None:
        intensity = np.zeros(count, dtype=np.float32)
    else:
        intensity = np.asarray(raw["intensity"], dtype=np.float32)

    finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
    return CloudArrays(
        xyz=np.ascontiguousarray(xyz[finite]),
        intensity=np.ascontiguousarray(intensity[finite]),
        receipt_ns=receipt_ns,
        source_stamp_ns=int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec),
    )


def make_xyz_intensity_rgb_cloud(
    xyz: np.ndarray,
    intensity: np.ndarray,
    bgr: np.ndarray,
    frame_id: str,
    stamp,
    sensor_id: Optional[np.ndarray] = None,
) -> PointCloud2:
    """Create an RViz/PCL-compatible xyz, intensity, packed-rgb cloud."""
    xyz = np.asarray(xyz, dtype=np.float32)
    intensity = np.asarray(intensity, dtype=np.float32)
    bgr = np.asarray(bgr, dtype=np.uint8)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    if intensity.shape != (xyz.shape[0],) or bgr.shape != (xyz.shape[0], 3):
        raise ValueError("intensity and bgr lengths must match xyz")

    include_sensor_id = sensor_id is not None
    dtype_fields = [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("rgb", "<f4"),
    ]
    if include_sensor_id:
        dtype_fields.append(("sensor_id", "<f4"))
    packed = np.empty(xyz.shape[0], dtype=np.dtype(dtype_fields))
    packed["x"], packed["y"], packed["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    packed["intensity"] = intensity
    rgb_u32 = (
        (bgr[:, 2].astype(np.uint32) << 16)
        | (bgr[:, 1].astype(np.uint32) << 8)
        | bgr[:, 0].astype(np.uint32)
    )
    packed["rgb"] = rgb_u32.view(np.float32)
    if include_sensor_id:
        packed["sensor_id"] = np.asarray(sensor_id, dtype=np.float32)

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    if include_sensor_id:
        msg.fields.append(
            PointField(name="sensor_id", offset=20, datatype=PointField.FLOAT32, count=1)
        )
    msg.is_bigendian = False
    msg.point_step = int(packed.dtype.itemsize)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = packed.tobytes()
    return msg


def relabel_cloud(msg: PointCloud2, frame_id: str, stamp) -> PointCloud2:
    """Clone cloud metadata/data while repairing its ambiguous frame and time."""
    out = PointCloud2()
    out.header.frame_id = frame_id
    out.header.stamp = stamp
    out.height = msg.height
    out.width = msg.width
    out.fields = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = msg.point_step
    out.row_step = msg.row_step
    out.data = msg.data
    out.is_dense = msg.is_dense
    return out


class SensorOverlayNode(Node):
    """Produce time/frame-corrected streams and visual calibration products."""

    def __init__(self) -> None:
        super().__init__("sensor_overlay")
        self.declare_parameter("lidar90_topic", "/lidar/lidar_90")
        self.declare_parameter("lidar91_topic", "/lidar/lidar_91")
        self.declare_parameter("image_topic", "/camera/color/image_raw/compressed")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter(
            "depth_image_topic", "/camera/depth/image_raw/compressedDepth"
        )
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter(
            "tag_detections_topic", "/overlay/apriltag/detections"
        )
        self.declare_parameter("lidar90_frame", "lidar_90")
        self.declare_parameter("lidar91_frame", "lidar_91")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_frame", "camera_depth_optical_frame")
        self.declare_parameter("max_image_age_sec", 0.080)
        self.declare_parameter("lidar_scan_midpoint_offset_sec", 0.050)
        self.declare_parameter("max_lidar_pair_age_sec", 0.030)
        self.declare_parameter("min_camera_depth_m", 0.20)
        self.declare_parameter("max_camera_depth_m", 80.0)
        self.declare_parameter("depth_stride", 2)
        self.declare_parameter("min_depth_m", 0.20)
        self.declare_parameter("max_depth_m", 20.0)
        self.declare_parameter("publish_projection_image", True)
        self.declare_parameter("lidar_calibration_path", "")
        self.declare_parameter("camera_calibration_path", "")

        lidar_calibration_path = str(
            self.get_parameter("lidar_calibration_path").value
        )
        camera_calibration_path = str(
            self.get_parameter("camera_calibration_path").value
        )
        if not lidar_calibration_path:
            raise ValueError("lidar_calibration_path is required")
        if not camera_calibration_path:
            raise ValueError("camera_calibration_path is required")
        extrinsics = load_sensor_extrinsics(
            lidar_calibration_path,
            camera_calibration_path,
        )
        self.r_l90_l91 = extrinsics.lidar90_from_lidar91.rotation
        self.t_l90_l91 = extrinsics.lidar90_from_lidar91.translation
        self.r_l91_camera = extrinsics.lidar91_from_camera.rotation
        self.t_l91_camera = extrinsics.lidar91_from_camera.translation

        self.lidar90_frame = str(self.get_parameter("lidar90_frame").value)
        self.lidar91_frame = str(self.get_parameter("lidar91_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.depth_frame = str(self.get_parameter("depth_frame").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.depth_info_topic = str(self.get_parameter("depth_info_topic").value)
        self.max_image_age_ns = int(
            float(self.get_parameter("max_image_age_sec").value) * 1e9
        )
        self.lidar_scan_midpoint_offset_ns = int(
            float(self.get_parameter("lidar_scan_midpoint_offset_sec").value) * 1e9
        )
        self.max_lidar_pair_age_ns = int(
            float(self.get_parameter("max_lidar_pair_age_sec").value) * 1e9
        )
        self.min_camera_depth = float(self.get_parameter("min_camera_depth_m").value)
        self.max_camera_depth = float(self.get_parameter("max_camera_depth_m").value)
        self.depth_stride = max(1, int(self.get_parameter("depth_stride").value))
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        if self.min_depth < 0.0 or self.max_depth <= self.min_depth:
            raise ValueError("depth range must satisfy 0 <= min_depth_m < max_depth_m")
        self.publish_projection_image = bool(
            self.get_parameter("publish_projection_image").value
        )

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.depth_camera_info: Optional[CameraInfo] = None
        self.depth_camera_info_logged = False
        self.rectify_key: Optional[Tuple[float, ...]] = None
        self.rectify_maps: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.rectified_k: Optional[np.ndarray] = None
        self.camera_frames = deque(maxlen=8)
        self.cloud90: Optional[CloudArrays] = None
        self.cloud91: Optional[CloudArrays] = None
        self.last_merged_pair: Optional[Tuple[int, int]] = None
        self.last_status_log_ns = 0
        self.last_depth_log_ns = 0
        self.last_warning_log_ns = 0

        self.pub_lidar90 = self.create_publisher(
            PointCloud2, "/overlay/lidar90/points", OUTPUT_CLOUD_QOS
        )
        self.pub_lidar91 = self.create_publisher(
            PointCloud2, "/overlay/lidar91/points", OUTPUT_CLOUD_QOS
        )
        self.pub_merged = self.create_publisher(
            PointCloud2, "/overlay/lidars/merged", OUTPUT_CLOUD_QOS
        )
        self.pub_colored = self.create_publisher(
            PointCloud2, "/overlay/lidar91/points_colored", OUTPUT_CLOUD_QOS
        )
        self.pub_image_rect = self.create_publisher(
            Image, "/overlay/camera/image_rect", qos_profile_sensor_data
        )
        self.pub_camera_info = self.create_publisher(
            CameraInfo, "/overlay/camera/camera_info", qos_profile_sensor_data
        )
        self.pub_projection = self.create_publisher(
            Image, "/overlay/camera/lidar_projection", qos_profile_sensor_data
        )
        self.pub_tag_overlay = self.create_publisher(
            Image, "/overlay/camera/apriltag_overlay", qos_profile_sensor_data
        )
        self.pub_depth_points = self.create_publisher(
            PointCloud2, "/overlay/camera/depth_points", OUTPUT_CLOUD_QOS
        )

        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.sub_image = self.create_subscription(
            CompressedImage,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )
        self.sub_depth_info = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.on_depth_camera_info,
            qos_profile_sensor_data,
        )
        self.sub_depth_image = self.create_subscription(
            CompressedImage,
            str(self.get_parameter("depth_image_topic").value),
            self.on_depth_image,
            qos_profile_sensor_data,
        )
        self.sub_tag_detections = self.create_subscription(
            AprilTagDetectionArray,
            str(self.get_parameter("tag_detections_topic").value),
            self.on_tag_detections,
            qos_profile_sensor_data,
        )
        self.sub_lidar90 = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lidar90_topic").value),
            self.on_lidar90,
            qos_profile_sensor_data,
        )
        self.sub_lidar91 = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lidar91_topic").value),
            self.on_lidar91,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"LiDAR calibration loaded from {extrinsics.lidar_calibration_path}"
        )
        self.get_logger().info(
            f"Camera-LiDAR calibration loaded from {extrinsics.camera_calibration_path} "
            f"(results.{extrinsics.camera_transform_key})"
        )
        self.get_logger().info(
            f"Color intrinsics source: ROS topic {self.camera_info_topic} "
            "(calib.json supplies the extrinsic only)"
        )
        self.get_logger().info(
            f"Depth intrinsics source: ROS topic {self.depth_info_topic}"
        )
        self.get_logger().info(
            "Overlay node ready: repairing duplicate LiDAR frames and camera clock offset"
        )

    def receipt_time(self) -> Time:
        return self.get_clock().now()

    def warn_throttled(self, text: str, now_ns: int) -> None:
        if now_ns - self.last_warning_log_ns >= int(5e9):
            self.get_logger().warning(text)
            self.last_warning_log_ns = now_ns

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def on_depth_camera_info(self, msg: CameraInfo) -> None:
        self.depth_camera_info = msg
        if not self.depth_camera_info_logged:
            k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f"Depth CameraInfo active from {self.depth_info_topic}: "
                f"{msg.width}x{msg.height}, fx={k[0,0]:.3f}, fy={k[1,1]:.3f}, "
                f"cx={k[0,2]:.3f}, cy={k[1,2]:.3f}"
            )
            self.depth_camera_info_logged = True

    def on_depth_image(self, msg: CompressedImage) -> None:
        """Decode ROS compressedDepth 16UC1 data and publish a native depth cloud."""
        now = self.receipt_time()
        now_ns = now.nanoseconds
        info = self.depth_camera_info
        if info is None:
            self.warn_throttled(
                f"Waiting for {self.depth_info_topic}", now_ns
            )
            return
        if "16UC1" not in msg.format or "compressedDepth" not in msg.format:
            self.warn_throttled(
                f"Unsupported depth format {msg.format!r}; expected 16UC1 compressedDepth",
                now_ns,
            )
            return

        payload = bytes(msg.data)
        png_signature = b"\x89PNG\r\n\x1a\n"
        png_offset = payload.find(png_signature)
        if png_offset < 0:
            self.warn_throttled("Depth message has no PNG payload", now_ns)
            return
        depth_mm = cv2.imdecode(
            np.frombuffer(payload[png_offset:], dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        if depth_mm is None or depth_mm.dtype != np.uint16 or depth_mm.ndim != 2:
            self.warn_throttled("Could not decode depth as a 16UC1 image", now_ns)
            return
        if depth_mm.shape != (int(info.height), int(info.width)):
            self.warn_throttled(
                f"Depth image {depth_mm.shape[::-1]} does not match CameraInfo "
                f"{(int(info.width), int(info.height))}",
                now_ns,
            )
            return

        sampled = depth_mm[:: self.depth_stride, :: self.depth_stride]
        v_grid, u_grid = np.indices(sampled.shape, dtype=np.float64)
        u = u_grid * self.depth_stride
        v = v_grid * self.depth_stride
        z = sampled.astype(np.float64) * 0.001
        valid = (
            (sampled != 0)
            & (sampled != np.iinfo(np.uint16).max)
            & np.isfinite(z)
            & (z >= self.min_depth)
            & (z <= self.max_depth)
        )
        if not np.any(valid):
            self.warn_throttled("Depth image contains no points in the requested range", now_ns)
            return

        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
        if fx <= 0.0 or fy <= 0.0:
            self.warn_throttled("Depth CameraInfo has invalid focal lengths", now_ns)
            return

        z_valid = z[valid]
        xyz = np.column_stack(
            (
                (u[valid] - cx) * z_valid / fx,
                (v[valid] - cy) * z_valid / fy,
                z_valid,
            )
        ).astype(np.float32)
        depth_fraction = np.clip(
            (z_valid - self.min_depth) / (self.max_depth - self.min_depth),
            0.0,
            1.0,
        )
        depth_u8 = np.rint((1.0 - depth_fraction) * 255.0).astype(np.uint8)
        colors_bgr = cv2.applyColorMap(
            depth_u8.reshape(-1, 1), cv2.COLORMAP_TURBO
        )[:, 0, :]
        self.pub_depth_points.publish(
            make_xyz_intensity_rgb_cloud(
                xyz,
                z_valid.astype(np.float32),
                colors_bgr,
                self.depth_frame,
                now.to_msg(),
            )
        )
        if now_ns - self.last_depth_log_ns >= int(5e9):
            self.get_logger().info(
                f"Depth cloud healthy: {xyz.shape[0]} points in {self.depth_frame}; "
                f"stride={self.depth_stride}"
            )
            self.last_depth_log_ns = now_ns

    def on_tag_detections(self, msg: AprilTagDetectionArray) -> None:
        """Draw detector pixel corners and IDs on the matching rectified frame."""
        if not self.camera_frames:
            return
        detection_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        camera_frame = min(
            self.camera_frames,
            key=lambda frame: abs(frame.receipt_ns - detection_stamp_ns),
        )
        image_age_ns = abs(camera_frame.receipt_ns - detection_stamp_ns)
        if image_age_ns > self.max_image_age_ns:
            self.warn_throttled(
                f"Nearest camera image is {image_age_ns / 1e6:.1f} ms from tag detections",
                self.receipt_time().nanoseconds,
            )
            return

        annotated = camera_frame.bgr.copy()
        height, width = annotated.shape[:2]
        for detection in msg.detections:
            corners = np.rint(
                [[corner.x, corner.y] for corner in detection.corners]
            ).astype(np.int32)
            cv2.polylines(
                annotated,
                [corners.reshape(-1, 1, 2)],
                True,
                (0, 255, 0),
                3,
                cv2.LINE_AA,
            )
            centre = tuple(
                np.rint([detection.centre.x, detection.centre.y]).astype(np.int32)
            )
            cv2.circle(annotated, centre, 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(
                annotated,
                tuple(corners[0]),
                6,
                (255, 80, 20),
                -1,
                cv2.LINE_AA,
            )
            text_x = int(np.clip(np.min(corners[:, 0]), 0, max(0, width - 1)))
            text_y = int(np.clip(np.min(corners[:, 1]) - 10, 24, max(24, height - 1)))
            label = f"{detection.family}:{detection.id}"
            cv2.putText(
                annotated,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                5,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        overlay_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        overlay_msg.header = copy.deepcopy(msg.header)
        overlay_msg.header.frame_id = self.camera_frame
        self.pub_tag_overlay.publish(overlay_msg)

    def _prepare_rectification(self, info: CameraInfo, width: int, height: int) -> None:
        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(info.d, dtype=np.float64)
        key = tuple(k.ravel()) + tuple(d.ravel()) + (float(width), float(height))
        if key == self.rectify_key:
            return
        self.rectify_maps = cv2.initUndistortRectifyMap(
            k,
            d,
            np.eye(3, dtype=np.float64),
            k,
            (width, height),
            cv2.CV_32FC1,
        )
        self.rectified_k = k
        self.rectify_key = key
        self.get_logger().info(
            f"Color CameraInfo active from {self.camera_info_topic}: "
            f"{width}x{height}, fx={k[0,0]:.3f}, fy={k[1,1]:.3f}, "
            f"cx={k[0,2]:.3f}, cy={k[1,2]:.3f}"
        )

    def on_image(self, msg: CompressedImage) -> None:
        now = self.receipt_time()
        now_ns = now.nanoseconds
        if self.camera_info is None:
            self.warn_throttled(f"Waiting for {self.camera_info_topic}", now_ns)
            return
        compressed = np.frombuffer(msg.data, dtype=np.uint8)
        image_bgr = cv2.imdecode(compressed, cv2.IMREAD_COLOR)
        if image_bgr is None:
            self.warn_throttled("OpenCV could not decode the compressed color image", now_ns)
            return

        height, width = image_bgr.shape[:2]
        self._prepare_rectification(self.camera_info, width, height)
        assert self.rectify_maps is not None
        rectified = cv2.remap(
            image_bgr,
            self.rectify_maps[0],
            self.rectify_maps[1],
            interpolation=cv2.INTER_LINEAR,
        )
        stamp = now.to_msg()
        image_out = self.bridge.cv2_to_imgmsg(rectified, encoding="bgr8")
        image_out.header.frame_id = self.camera_frame
        image_out.header.stamp = stamp

        info_out = copy.deepcopy(self.camera_info)
        info_out.header.frame_id = self.camera_frame
        info_out.header.stamp = stamp
        info_out.d = [0.0] * len(info_out.d)
        info_out.distortion_model = "plumb_bob"

        assert self.rectified_k is not None
        self.camera_frames.append(
            CameraFrame(bgr=rectified, k=self.rectified_k.copy(), receipt_ns=now_ns)
        )
        self.pub_camera_info.publish(info_out)
        self.pub_image_rect.publish(image_out)

    def on_lidar90(self, msg: PointCloud2) -> None:
        now = self.receipt_time()
        stamp = now.to_msg()
        self.pub_lidar90.publish(relabel_cloud(msg, self.lidar90_frame, stamp))
        try:
            self.cloud90 = cloud_to_arrays(msg, now.nanoseconds)
        except ValueError as exc:
            self.warn_throttled(f"Cannot parse lidar_90 cloud: {exc}", now.nanoseconds)
            return
        self.try_publish_merged(stamp)

    def on_lidar91(self, msg: PointCloud2) -> None:
        now = self.receipt_time()
        stamp = now.to_msg()
        self.pub_lidar91.publish(relabel_cloud(msg, self.lidar91_frame, stamp))
        try:
            self.cloud91 = cloud_to_arrays(msg, now.nanoseconds)
        except ValueError as exc:
            self.warn_throttled(f"Cannot parse lidar_91 cloud: {exc}", now.nanoseconds)
            return
        self.try_publish_merged(stamp)
        self.publish_colored_lidar91(self.cloud91, stamp)

    def try_publish_merged(self, stamp) -> None:
        if self.cloud90 is None or self.cloud91 is None:
            return
        # The two LiDAR header clocks are mutually consistent (unlike the
        # camera clock), so use their native scan-start stamps for pairing.
        pair = (self.cloud90.source_stamp_ns, self.cloud91.source_stamp_ns)
        if pair == self.last_merged_pair:
            return
        delta_ns = abs(pair[0] - pair[1])
        if delta_ns > self.max_lidar_pair_age_ns:
            return

        xyz90 = self.cloud90.xyz
        xyz91_in_90 = (
            self.cloud91.xyz.astype(np.float64) @ self.r_l90_l91.T
            + self.t_l90_l91
        ).astype(np.float32)
        xyz = np.vstack((xyz90, xyz91_in_90))
        intensity = np.concatenate((self.cloud90.intensity, self.cloud91.intensity))
        sensor_id = np.concatenate(
            (
                np.zeros(xyz90.shape[0], dtype=np.float32),
                np.ones(xyz91_in_90.shape[0], dtype=np.float32),
            )
        )
        colors = np.empty((xyz.shape[0], 3), dtype=np.uint8)
        colors[: xyz90.shape[0]] = (255, 210, 40)  # BGR: cyan-blue for lidar 90
        colors[xyz90.shape[0] :] = (30, 120, 255)  # BGR: orange for lidar 91
        self.pub_merged.publish(
            make_xyz_intensity_rgb_cloud(
                xyz, intensity, colors, self.lidar90_frame, stamp, sensor_id=sensor_id
            )
        )
        self.last_merged_pair = pair

    def publish_colored_lidar91(self, cloud: CloudArrays, stamp) -> None:
        now_ns = cloud.receipt_ns
        if not self.camera_frames:
            self.warn_throttled("Waiting for a rectified camera image before colorizing", now_ns)
            return

        # Bag receipt is just after a 100 ms Livox scan. Pair against the scan
        # midpoint rather than its end, using the nearest buffered camera frame.
        target_image_ns = now_ns - self.lidar_scan_midpoint_offset_ns
        camera_frame = min(
            self.camera_frames, key=lambda frame: abs(frame.receipt_ns - target_image_ns)
        )
        image_age_ns = abs(target_image_ns - camera_frame.receipt_ns)
        if image_age_ns > self.max_image_age_ns:
            self.warn_throttled(
                f"Nearest camera image is {image_age_ns / 1e3:.0f} us from lidar_91; skipping colorization",
                now_ns,
            )
            return

        # Invert the camera->lidar91 transform loaded from calib.json.
        xyz_camera = (
            (cloud.xyz.astype(np.float64) - self.t_l91_camera)
            @ self.r_l91_camera
        )
        z = xyz_camera[:, 2]
        valid = (
            np.isfinite(xyz_camera).all(axis=1)
            & (z > self.min_camera_depth)
            & (z < self.max_camera_depth)
        )
        valid_indices = np.flatnonzero(valid)
        if valid_indices.size == 0:
            return

        camera_points = xyz_camera[valid_indices]
        k = camera_frame.k
        u = np.rint(k[0, 0] * camera_points[:, 0] / camera_points[:, 2] + k[0, 2]).astype(np.int32)
        v = np.rint(k[1, 1] * camera_points[:, 1] / camera_points[:, 2] + k[1, 2]).astype(np.int32)
        height, width = camera_frame.bgr.shape[:2]
        inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        valid_indices = valid_indices[inside]
        u, v = u[inside], v[inside]
        if valid_indices.size == 0:
            return

        colors_bgr = camera_frame.bgr[v, u]
        self.pub_colored.publish(
            make_xyz_intensity_rgb_cloud(
                cloud.xyz[valid_indices],
                cloud.intensity[valid_indices],
                colors_bgr,
                self.lidar91_frame,
                stamp,
            )
        )

        if self.publish_projection_image:
            overlay = camera_frame.bgr.copy()
            visible_depth = camera_points[inside, 2]
            depth_scale = np.clip((visible_depth - 0.5) / 30.0, 0.0, 1.0)
            depth_u8 = np.rint((1.0 - depth_scale) * 255.0).astype(np.uint8)
            depth_colors = cv2.applyColorMap(depth_u8.reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
            for du, dv in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
                uu = np.clip(u + du, 0, width - 1)
                vv = np.clip(v + dv, 0, height - 1)
                overlay[vv, uu] = depth_colors
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header.frame_id = self.camera_frame
            overlay_msg.header.stamp = stamp
            self.pub_projection.publish(overlay_msg)

        if now_ns - self.last_status_log_ns >= int(5e9):
            self.get_logger().info(
                f"Overlay healthy: colored {valid_indices.size}/{cloud.xyz.shape[0]} lidar_91 points; "
                f"camera-to-scan-midpoint delta={image_age_ns / 1e6:.1f} ms"
            )
            self.last_status_log_ns = now_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorOverlayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
