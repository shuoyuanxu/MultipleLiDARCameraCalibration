#!/usr/bin/env python3
"""Generate the deployable U701 URDF/Xacro from the accepted calibration files."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "three_sensor_overlay"))

from three_sensor_overlay.calibration_io import (  # noqa: E402
    load_sensor_extrinsics,
    render_deployment_xacro,
    render_standalone_urdf,
)


DEFAULT_LIDAR_CALIBRATION = (
    ROOT / "calibration_results" / "extrinsic_lidar91_to_lidar90.yaml"
)
DEFAULT_CAMERA_CALIBRATION = ROOT / "calibration_vision_lidar91" / "calib.json"
DEFAULT_URDF_OUTPUT = (
    ROOT / "three_sensor_overlay" / "urdf" / "u701_three_sensor_extrinsics.urdf"
)
DEFAULT_XACRO_OUTPUT = DEFAULT_URDF_OUTPUT.with_suffix(".urdf.xacro")


def atomic_write(path: Path, contents: str) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent, text=True
    )
    temporary_path = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o644)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.replace(path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate U701 sensor URDF files directly from YAML/JSON calibration results."
    )
    parser.add_argument(
        "--lidar-calibration",
        type=Path,
        default=DEFAULT_LIDAR_CALIBRATION,
        help=f"LiDAR YAML (default: {DEFAULT_LIDAR_CALIBRATION})",
    )
    parser.add_argument(
        "--camera-calibration",
        type=Path,
        default=DEFAULT_CAMERA_CALIBRATION,
        help=f"Camera-LiDAR JSON (default: {DEFAULT_CAMERA_CALIBRATION})",
    )
    parser.add_argument("--urdf-output", type=Path, default=DEFAULT_URDF_OUTPUT)
    parser.add_argument("--xacro-output", type=Path, default=DEFAULT_XACRO_OUTPUT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    extrinsics = load_sensor_extrinsics(
        args.lidar_calibration,
        args.camera_calibration,
    )
    urdf_text = render_standalone_urdf(extrinsics)
    xacro_text = render_deployment_xacro(extrinsics)
    atomic_write(args.urdf_output, urdf_text)
    atomic_write(args.xacro_output, xacro_text)
    print(f"LiDAR calibration: {extrinsics.lidar_calibration_path}")
    print(f"Camera calibration: {extrinsics.camera_calibration_path}")
    print(f"Camera transform: results.{extrinsics.camera_transform_key}")
    print(f"Generated URDF: {args.urdf_output.expanduser().resolve()}")
    print(f"Generated Xacro: {args.xacro_output.expanduser().resolve()}")


if __name__ == "__main__":
    main()
