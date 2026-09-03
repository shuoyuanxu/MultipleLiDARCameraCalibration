"""Load the two authoritative U701 inter-sensor calibration files."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence, Tuple

import numpy as np
import yaml


LIDAR90_FRAME = "lidar_90"
LIDAR91_FRAME = "lidar_91"
CAMERA_FRAME = "camera_color_optical_frame"
CAMERA_TRANSFORM_KEYS = ("T_lidar_camera", "init_T_lidar_camera")


@dataclass(frozen=True)
class RigidTransform:
    """A parent-from-child rigid transform using an xyzw quaternion."""

    translation: np.ndarray
    quaternion_xyzw: np.ndarray

    @property
    def rotation(self) -> np.ndarray:
        return quaternion_xyzw_to_matrix(self.quaternion_xyzw)


@dataclass(frozen=True)
class SensorExtrinsics:
    """The calibrated lidar90<-lidar91<-camera transform chain."""

    lidar90_from_lidar91: RigidTransform
    lidar91_from_camera: RigidTransform
    lidar_calibration_path: Path
    camera_calibration_path: Path
    camera_transform_key: str


def _mapping(value: Any, path: Path, key: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{path}: {key} must be an object")
    return value


def _finite_float(value: Any, path: Path, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path}: {key} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{path}: {key} must be finite")
    return result


def _xyz_object(value: Any, path: Path, key: str) -> np.ndarray:
    obj = _mapping(value, path, key)
    try:
        return np.array(
            [_finite_float(obj[axis], path, f"{key}.{axis}") for axis in "xyz"],
            dtype=np.float64,
        )
    except KeyError as exc:
        raise ValueError(f"{path}: missing {key}.{exc.args[0]}") from exc


def _xyzw_object(value: Any, path: Path, key: str) -> np.ndarray:
    obj = _mapping(value, path, key)
    try:
        values = np.array(
            [_finite_float(obj[axis], path, f"{key}.{axis}") for axis in "xyzw"],
            dtype=np.float64,
        )
    except KeyError as exc:
        raise ValueError(f"{path}: missing {key}.{exc.args[0]}") from exc
    return normalize_quaternion(values, path, key)


def normalize_quaternion(
    quaternion_xyzw: Sequence[float], path: Path, key: str
) -> np.ndarray:
    values = np.asarray(quaternion_xyzw, dtype=np.float64)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError(f"{path}: {key} must contain four finite xyzw values")
    norm = float(np.linalg.norm(values))
    if norm <= 1.0e-12:
        raise ValueError(f"{path}: {key} is a zero-length quaternion")
    return values / norm


def quaternion_xyzw_to_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Return the active 3x3 rotation matrix for an xyzw quaternion."""
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64)
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def quaternion_xyzw_to_rpy(quaternion_xyzw: Sequence[float]) -> Tuple[float, float, float]:
    """Convert an xyzw quaternion to URDF fixed-axis roll, pitch, yaw."""
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _transform_matrix(transform: RigidTransform) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = transform.rotation
    matrix[:3, 3] = transform.translation
    return matrix


def _optional_matrix(
    document: Mapping[str, Any], key: str, path: Path
) -> np.ndarray | None:
    if key not in document:
        return None
    try:
        matrix = np.asarray(document[key], dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}: {key} must be a numeric 4x4 matrix") from exc
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{path}: {key} must be a finite 4x4 matrix")
    return matrix


def load_lidar_calibration(path_value: str | Path) -> Tuple[RigidTransform, Path]:
    """Load p_lidar90 = T_lidar90_lidar91 * p_lidar91 from YAML."""
    path = Path(path_value).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"LiDAR calibration file not found: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"{path}: invalid YAML: {exc}") from exc
    document = _mapping(document, path, "document")

    expected = {
        "parent_frame": LIDAR90_FRAME,
        "child_frame": LIDAR91_FRAME,
        "direction": "lidar_91_to_lidar_90",
    }
    for key, expected_value in expected.items():
        actual = document.get(key)
        if actual != expected_value:
            raise ValueError(
                f"{path}: {key} must be {expected_value!r}, got {actual!r}"
            )
    try:
        translation = _xyz_object(document["translation_m"], path, "translation_m")
        quaternion = _xyzw_object(
            document["quaternion_xyzw"], path, "quaternion_xyzw"
        )
    except KeyError as exc:
        raise ValueError(f"{path}: missing {exc.args[0]}") from exc

    transform = RigidTransform(translation, quaternion)
    direct_matrix = _optional_matrix(document, "matrix_T_lidar90_lidar91", path)
    if direct_matrix is not None and not np.allclose(
        direct_matrix, _transform_matrix(transform), atol=2.0e-9, rtol=0.0
    ):
        raise ValueError(
            f"{path}: matrix_T_lidar90_lidar91 disagrees with translation/quaternion"
        )
    inverse_matrix = _optional_matrix(document, "inverse_matrix_T_lidar91_lidar90", path)
    if inverse_matrix is not None and not np.allclose(
        inverse_matrix, np.linalg.inv(_transform_matrix(transform)), atol=2.0e-9, rtol=0.0
    ):
        raise ValueError(
            f"{path}: inverse_matrix_T_lidar91_lidar90 disagrees with translation/quaternion"
        )
    return transform, path


def load_camera_calibration(
    path_value: str | Path, transform_key: str | None = None
) -> Tuple[RigidTransform, Path, str]:
    """Load the accepted camera pose, preferring final over manual-only data."""
    path = Path(path_value).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Camera-LiDAR calibration file not found: {path}")
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid JSON: {exc}") from exc
    document = _mapping(document, path, "document")
    meta = _mapping(document.get("meta"), path, "meta")
    expected_topics = {
        "points_topic": "/lidar/lidar_91",
        "image_topic": "/camera/color/image_raw/compressed",
        "camera_info_topic": "/camera/color/camera_info",
    }
    for key, expected_topic in expected_topics.items():
        actual_topic = meta.get(key)
        if actual_topic != expected_topic:
            raise ValueError(
                f"{path}: meta.{key} must be {expected_topic!r}, "
                f"got {actual_topic!r}"
            )
    results = _mapping(document.get("results"), path, "results")
    if transform_key is None:
        transform_key = next(
            (key for key in CAMERA_TRANSFORM_KEYS if key in results), None
        )
        if transform_key is None:
            raise ValueError(
                f"{path}: results must contain T_lidar_camera or "
                "init_T_lidar_camera"
            )
    if transform_key not in CAMERA_TRANSFORM_KEYS:
        raise ValueError(
            f"camera transform key must be one of {CAMERA_TRANSFORM_KEYS}, "
            f"got {transform_key!r}"
        )
    values = results.get(transform_key)
    if not isinstance(values, (list, tuple)) or len(values) != 7:
        raise ValueError(
            f"{path}: results.{transform_key} must be "
            "[tx, ty, tz, qx, qy, qz, qw]"
        )
    numbers = np.array(
        [
            _finite_float(value, path, f"results.{transform_key}[{index}]")
            for index, value in enumerate(values)
        ],
        dtype=np.float64,
    )
    quaternion = normalize_quaternion(
        numbers[3:], path, f"results.{transform_key}[3:7]"
    )
    return RigidTransform(numbers[:3], quaternion), path, transform_key


def load_sensor_extrinsics(
    lidar_calibration_path: str | Path,
    camera_calibration_path: str | Path,
) -> SensorExtrinsics:
    lidar_transform, lidar_path = load_lidar_calibration(lidar_calibration_path)
    camera_transform, camera_path, camera_transform_key = load_camera_calibration(
        camera_calibration_path
    )
    return SensorExtrinsics(
        lidar90_from_lidar91=lidar_transform,
        lidar91_from_camera=camera_transform,
        lidar_calibration_path=lidar_path,
        camera_calibration_path=camera_path,
        camera_transform_key=camera_transform_key,
    )


def _format_vector(values: Sequence[float]) -> str:
    return " ".join(f"{float(value):.12f}" for value in values)


def render_standalone_urdf(extrinsics: SensorExtrinsics) -> str:
    lidar = extrinsics.lidar90_from_lidar91
    camera = extrinsics.lidar91_from_camera
    lidar_rpy = quaternion_xyzw_to_rpy(lidar.quaternion_xyzw)
    camera_rpy = quaternion_xyzw_to_rpy(camera.quaternion_xyzw)
    return f'''<?xml version="1.0"?>
<!-- AUTO-GENERATED by generate_three_sensor_urdf.py from the calibration YAML and JSON. -->
<robot name="u701_three_sensor_extrinsics">
  <link name="{LIDAR90_FRAME}"/>
  <link name="{LIDAR91_FRAME}"/>
  <joint name="lidar90_to_lidar91" type="fixed">
    <parent link="{LIDAR90_FRAME}"/>
    <child link="{LIDAR91_FRAME}"/>
    <origin xyz="{_format_vector(lidar.translation)}" rpy="{_format_vector(lidar_rpy)}"/>
  </joint>

  <link name="{CAMERA_FRAME}"/>
  <joint name="lidar91_to_camera_color_optical" type="fixed">
    <parent link="{LIDAR91_FRAME}"/>
    <child link="{CAMERA_FRAME}"/>
    <origin xyz="{_format_vector(camera.translation)}" rpy="{_format_vector(camera_rpy)}"/>
  </joint>
</robot>
'''


def render_deployment_xacro(extrinsics: SensorExtrinsics) -> str:
    lidar = extrinsics.lidar90_from_lidar91
    camera = extrinsics.lidar91_from_camera
    lidar_rpy = quaternion_xyzw_to_rpy(lidar.quaternion_xyzw)
    camera_rpy = quaternion_xyzw_to_rpy(camera.quaternion_xyzw)
    return f'''<?xml version="1.0"?>
<!-- AUTO-GENERATED by generate_three_sensor_urdf.py from the calibration YAML and JSON. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro
    name="u701_three_sensor_extrinsics"
    params="parent parent_to_lidar90_xyz parent_to_lidar90_rpy
            lidar90_link:='{LIDAR90_FRAME}'
            lidar91_link:='{LIDAR91_FRAME}'
            camera_optical_link:='{CAMERA_FRAME}'">

    <link name="${{lidar90_link}}"/>
    <joint name="robot_to_lidar90_calibrated" type="fixed">
      <parent link="${{parent}}"/>
      <child link="${{lidar90_link}}"/>
      <origin xyz="${{parent_to_lidar90_xyz}}" rpy="${{parent_to_lidar90_rpy}}"/>
    </joint>

    <link name="${{lidar91_link}}"/>
    <joint name="lidar90_to_lidar91_calibrated" type="fixed">
      <parent link="${{lidar90_link}}"/>
      <child link="${{lidar91_link}}"/>
      <origin xyz="{_format_vector(lidar.translation)}" rpy="{_format_vector(lidar_rpy)}"/>
    </joint>

    <link name="${{camera_optical_link}}"/>
    <joint name="lidar91_to_camera_color_optical_calibrated" type="fixed">
      <parent link="${{lidar91_link}}"/>
      <child link="${{camera_optical_link}}"/>
      <origin xyz="{_format_vector(camera.translation)}" rpy="{_format_vector(camera_rpy)}"/>
    </joint>
  </xacro:macro>
</robot>
'''
