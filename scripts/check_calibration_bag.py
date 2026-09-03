#!/usr/bin/env python3
"""Read-only preflight checks for a direct_visual_lidar_calibration ROS 2 bag.

Run after sourcing the same ROS 2 installation used to record the bag.  The
script intentionally reads only the first selected message of each required
topic, so it is safe to use on large bags and never changes their contents.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path
from statistics import median
from typing import Any, Iterable

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import PointField
except ImportError as exc:  # pragma: no cover - environment dependent
    print(
        "error: ROS 2 Python modules are unavailable. Source ROS first, e.g.\n"
        "  source /opt/ros/humble/setup.bash",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


IMAGE_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
}
CAMERA_INFO_TYPE = "sensor_msgs/msg/CameraInfo"
POINTS_TYPE = "sensor_msgs/msg/PointCloud2"
SUPPORTED_CAMERA_MODELS = {
    "plumb_bob",
    "fisheye",
    "equidistant",
    "omnidir",
    "equirectangular",
}
TIME_FIELDS = ("t", "time", "time_stamp", "timestamp")


class Report:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        print(f"INFO  {message}")

    def warn(self, message: str) -> None:
        self.warnings.append(message)
        print(f"WARN  {message}")

    def fail(self, message: str) -> None:
        self.errors.append(message)
        print(f"FAIL  {message}")


def seconds(nanoseconds: int) -> str:
    return f"{nanoseconds / 1e9:.3f}s"


def topic_candidates(topic_types: dict[str, str], allowed: Iterable[str]) -> list[str]:
    allowed_set = set(allowed)
    return sorted(name for name, msg_type in topic_types.items() if msg_type in allowed_set)


def select_topic(
    report: Report,
    label: str,
    requested: str | None,
    candidates: list[str],
    topic_types: dict[str, str],
    allowed: Iterable[str],
) -> str | None:
    if requested:
        actual = topic_types.get(requested)
        if actual is None:
            report.fail(f"selected {label} topic does not exist: {requested}")
            return None
        if actual not in set(allowed):
            report.fail(f"selected {label} topic has type {actual}, not a supported type")
            return None
        return requested

    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        report.fail(f"no {label} topic was found")
    else:
        report.fail(
            f"multiple {label} topics found ({', '.join(candidates)}); "
            f"pass --{label.replace('_', '-')}-topic explicitly"
        )
    return None


def open_reader(bag: Path) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def first_messages(
    bag: Path,
    wanted: set[str],
    topic_types: dict[str, str],
) -> dict[str, tuple[Any, int]]:
    reader = open_reader(bag)
    result: dict[str, tuple[Any, int]] = {}
    while reader.has_next() and len(result) < len(wanted):
        topic, serialized, receive_time_ns = reader.read_next()
        if topic not in wanted or topic in result:
            continue
        message_type = get_message(topic_types[topic])
        result[topic] = (deserialize_message(serialized, message_type), receive_time_ns)
    return result


def header_time_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def check_header_clock(report: Report, label: str, message: Any, receive_time_ns: int) -> None:
    delta_s = (header_time_ns(message) - receive_time_ns) / 1e9
    report.info(f"{label} header minus bag-receive timestamp: {delta_s:+.6f}s")
    if abs(delta_s) > 0.2:
        report.warn(
            f"{label} header clock differs from bag receive time by {delta_s:+.3f}s; "
            "this calibrator does not synchronize image/cloud messages, so use only a fully stationary bag"
        )


def decode_compressed_size(message: Any) -> tuple[int, int] | None:
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None
    image = cv2.imdecode(np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        return None
    return int(image.shape[1]), int(image.shape[0])


def check_image(report: Report, topic: str, message: Any, receive_time_ns: int) -> tuple[int, int] | None:
    check_header_clock(report, "image", message, receive_time_ns)
    if hasattr(message, "width"):
        width, height = int(message.width), int(message.height)
        report.info(f"image {topic}: {width}x{height}, encoding={message.encoding}, frame={message.header.frame_id!r}")
        return width, height

    size = decode_compressed_size(message)
    if size is None:
        report.fail(f"cannot decode compressed image from {topic}")
        return None
    report.info(
        f"image {topic}: {size[0]}x{size[1]}, format={message.format!r}, "
        f"frame={message.header.frame_id!r}"
    )
    return size


def check_camera_info(
    report: Report,
    topic: str,
    message: Any,
    receive_time_ns: int,
    image_size: tuple[int, int] | None,
) -> None:
    check_header_clock(report, "CameraInfo", message, receive_time_ns)
    width, height = int(message.width), int(message.height)
    fx, fy, cx, cy = float(message.k[0]), float(message.k[4]), float(message.k[2]), float(message.k[5])
    report.info(
        f"CameraInfo {topic}: {width}x{height}, model={message.distortion_model!r}, "
        f"K=[{fx:.4f}, {fy:.4f}, {cx:.4f}, {cy:.4f}], D={len(message.d)} values, "
        f"frame={message.header.frame_id!r}"
    )
    if width <= 0 or height <= 0 or fx <= 0 or fy <= 0:
        report.fail("CameraInfo has invalid dimensions or focal lengths")
    if image_size and image_size != (width, height):
        report.fail(
            f"CameraInfo resolution {width}x{height} does not match image resolution "
            f"{image_size[0]}x{image_size[1]}"
        )
    if message.distortion_model not in SUPPORTED_CAMERA_MODELS:
        report.warn(
            f"camera model {message.distortion_model!r} is not one of the documented models "
            f"({', '.join(sorted(SUPPORTED_CAMERA_MODELS))}); specify a supported model/intrinsics manually if preprocessing fails"
        )


_STRUCT_FORMATS = {
    PointField.UINT8: "B",
    PointField.INT8: "b",
    PointField.UINT16: "H",
    PointField.INT16: "h",
    PointField.UINT32: "I",
    PointField.INT32: "i",
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    index = round((len(values) - 1) * fraction)
    return values[max(0, min(len(values) - 1, index))]


def sample_field(message: Any, field: Any, maximum: int = 50_000) -> list[float]:
    fmt = _STRUCT_FORMATS.get(field.datatype)
    if fmt is None or message.is_bigendian:
        return []
    point_count = int(message.width) * int(message.height)
    stride = max(1, point_count // maximum)
    unpack = struct.Struct("<" + fmt).unpack_from
    data = memoryview(message.data)
    values: list[float] = []
    for index in range(0, point_count, stride):
        value = float(unpack(data, index * int(message.point_step) + int(field.offset))[0])
        if math.isfinite(value):
            values.append(value)
    values.sort()
    return values


def check_points(report: Report, topic: str, message: Any, receive_time_ns: int) -> None:
    check_header_clock(report, "PointCloud2", message, receive_time_ns)
    fields = {field.name: field for field in message.fields}
    point_count = int(message.width) * int(message.height)
    description = ", ".join(f"{field.name}:{field.datatype}@{field.offset}" for field in message.fields)
    report.info(
        f"PointCloud2 {topic}: {point_count} points, step={message.point_step}, "
        f"frame={message.header.frame_id!r}, fields=[{description}]"
    )

    for coordinate in ("x", "y", "z"):
        field = fields.get(coordinate)
        if field is None:
            report.fail(f"PointCloud2 has no {coordinate!r} field")
        elif field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            report.fail(f"PointCloud2 field {coordinate!r} must be FLOAT32 or FLOAT64")

    intensity = fields.get("reflectivity") or fields.get("intensity")
    if intensity is None:
        report.fail("PointCloud2 has neither 'intensity' nor 'reflectivity'; this method needs LiDAR intensity texture")
    elif intensity.datatype not in _STRUCT_FORMATS:
        report.fail(f"unsupported {intensity.name!r} field datatype {intensity.datatype}")
    else:
        values = sample_field(message, intensity)
        if not values:
            report.warn(f"could not sample {intensity.name!r} values (big-endian or unsupported layout)")
        else:
            report.info(
                f"recommended intensity channel: {intensity.name}; sampled {len(values)} values: "
                f"min={values[0]:.3g}, p01={percentile(values, .01):.3g}, "
                f"median={median(values):.3g}, p99={percentile(values, .99):.3g}, max={values[-1]:.3g}"
            )
            if values[0] == values[-1]:
                report.fail("LiDAR intensity is constant in the first cloud; direct visual registration cannot use it")

    found_times = [name for name in TIME_FIELDS if name in fields]
    if found_times:
        report.info(f"per-point time field(s): {', '.join(found_times)}")
    else:
        report.warn("no recognized per-point timestamp field; fine for a stationary MID-360 bag, unsafe for moving-data integration")
    if message.is_bigendian:
        report.warn("PointCloud2 is big-endian; this calibrator normally expects little-endian ROS point data")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory (the directory containing metadata.yaml)")
    parser.add_argument("--image-topic")
    parser.add_argument("--camera-info-topic")
    parser.add_argument("--points-topic")
    args = parser.parse_args()

    report = Report()
    bag = args.bag.expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        report.fail(f"not a finalized rosbag2 directory (metadata.yaml missing): {bag}")
        return 2

    try:
        reader = open_reader(bag)
        metadata = reader.get_metadata()
        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    except Exception as exc:  # pragma: no cover - rosbag errors vary by distro
        report.fail(f"cannot open bag: {exc}")
        return 2

    report.info(
        f"bag={bag}; duration={seconds(metadata.duration.nanoseconds)}; "
        f"messages={metadata.message_count}; size={metadata.bag_size / (1024 * 1024):.1f} MiB"
    )
    if not 10 <= metadata.duration.nanoseconds / 1e9 <= 20:
        report.warn("duration is outside the recommended 10-20 s stationary MID-360 capture window")

    image_candidates = topic_candidates(topic_types, IMAGE_TYPES)
    info_candidates = topic_candidates(topic_types, {CAMERA_INFO_TYPE})
    points_candidates = topic_candidates(topic_types, {POINTS_TYPE})
    report.info(f"image candidates: {', '.join(image_candidates) or '(none)'}")
    report.info(f"CameraInfo candidates: {', '.join(info_candidates) or '(none)'}")
    report.info(f"PointCloud2 candidates: {', '.join(points_candidates) or '(none)'}")

    image_topic = select_topic(report, "image", args.image_topic, image_candidates, topic_types, IMAGE_TYPES)
    info_topic = select_topic(report, "camera_info", args.camera_info_topic, info_candidates, topic_types, {CAMERA_INFO_TYPE})
    points_topic = select_topic(report, "points", args.points_topic, points_candidates, topic_types, {POINTS_TYPE})
    if report.errors:
        return 2

    assert image_topic and info_topic and points_topic
    selected = {image_topic, info_topic, points_topic}
    try:
        messages = first_messages(bag, selected, topic_types)
    except Exception as exc:  # pragma: no cover - serialization errors vary by distro
        report.fail(f"cannot deserialize selected messages: {exc}")
        return 2
    missing = selected - set(messages)
    if missing:
        report.fail(f"bag contains no message for selected topic(s): {', '.join(sorted(missing))}")
        return 2

    image, image_receive = messages[image_topic]
    info, info_receive = messages[info_topic]
    points, points_receive = messages[points_topic]
    image_size = check_image(report, image_topic, image, image_receive)
    check_camera_info(report, info_topic, info, info_receive, image_size)
    check_points(report, points_topic, points, points_receive)

    if report.errors:
        print(f"\nRESULT: FAIL ({len(report.errors)} hard issue(s), {len(report.warnings)} warning(s))")
        return 2
    print(f"\nRESULT: PASS ({len(report.warnings)} warning(s))")
    print("Reminder: PASS verifies message compatibility only. Confirm that the vehicle stayed completely still and that shared scene structure is visible in both sensors.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
