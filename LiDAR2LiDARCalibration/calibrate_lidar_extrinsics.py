#!/usr/bin/env python3
"""Calibrate LiDAR 91 to LiDAR 90 from trajectories, maps, and scans.

The reported transform is::

    T_lidar90_lidar91
    p_lidar90 = T_lidar90_lidar91 @ p_lidar91

The script deliberately uses only NumPy and SciPy.  It reads both ordinary
binary PCD maps and PCL binary_compressed keyframe PCDs without Open3D.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


ROOT = Path(__file__).resolve().parent


@dataclass
class PoseSeries:
    timestamps_ns: np.ndarray
    translations: np.ndarray
    rotations: np.ndarray


@dataclass
class TransformEstimate:
    name: str
    rotation: np.ndarray
    translation: np.ndarray
    diagnostics: Dict[str, object]


def transform_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def invert_transform(rotation: np.ndarray, translation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rotation_inv = rotation.T
    return rotation_inv, -(rotation_inv @ translation)


def rotation_mean(rotations: np.ndarray) -> np.ndarray:
    """Return the chordal quaternion mean for a tight cluster of rotations."""
    quaternions = Rotation.from_matrix(rotations).as_quat()
    accumulator = quaternions.T @ quaternions
    _, eigenvectors = np.linalg.eigh(accumulator)
    quaternion = eigenvectors[:, -1]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return Rotation.from_quat(quaternion).as_matrix()


def rotation_distance_deg(rotation_a: np.ndarray, rotation_b: np.ndarray) -> float:
    delta = Rotation.from_matrix(rotation_a).inv() * Rotation.from_matrix(rotation_b)
    return float(np.degrees(np.linalg.norm(delta.as_rotvec())))


def rigid_fit(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Least-squares rigid transform mapping source points to target points."""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    return rotation, target_center - rotation @ source_center


def _lzf_decompress(compressed: bytes, expected_size: int) -> bytes:
    """Decode the LZF stream used by PCL's binary_compressed PCD format."""
    input_index = 0
    output = bytearray()
    compressed_size = len(compressed)

    while input_index < compressed_size:
        control = compressed[input_index]
        input_index += 1
        if control < 32:
            literal_length = control + 1
            end = input_index + literal_length
            if end > compressed_size:
                raise ValueError("Truncated literal in LZF stream")
            output.extend(compressed[input_index:end])
            input_index = end
            continue

        copy_length = control >> 5
        reference = len(output) - ((control & 0x1F) << 8) - 1
        if copy_length == 7:
            if input_index >= compressed_size:
                raise ValueError("Truncated length in LZF stream")
            copy_length += compressed[input_index]
            input_index += 1
        if input_index >= compressed_size:
            raise ValueError("Truncated offset in LZF stream")
        reference -= compressed[input_index]
        input_index += 1
        copy_length += 2
        if reference < 0:
            raise ValueError("Invalid back-reference in LZF stream")
        for _ in range(copy_length):
            output.append(output[reference])
            reference += 1

    if len(output) != expected_size:
        raise ValueError(
            f"LZF size mismatch: decoded {len(output)} bytes, expected {expected_size}"
        )
    return bytes(output)


def _pcd_numpy_type(type_code: str, byte_size: int) -> np.dtype:
    types = {
        ("F", 4): np.dtype("<f4"),
        ("F", 8): np.dtype("<f8"),
        ("I", 1): np.dtype("i1"),
        ("I", 2): np.dtype("<i2"),
        ("I", 4): np.dtype("<i4"),
        ("I", 8): np.dtype("<i8"),
        ("U", 1): np.dtype("u1"),
        ("U", 2): np.dtype("<u2"),
        ("U", 4): np.dtype("<u4"),
        ("U", 8): np.dtype("<u8"),
    }
    try:
        return types[(type_code.upper(), byte_size)]
    except KeyError as exc:
        raise ValueError(f"Unsupported PCD scalar type {type_code}{byte_size}") from exc


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Read xyz from ASCII, binary, or PCL binary_compressed PCD."""
    header: Dict[str, List[str]] = {}
    with path.open("rb") as stream:
        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise ValueError(f"{path}: PCD header ended before DATA")
            line = raw_line.decode("ascii").strip()
            if line and not line.startswith("#"):
                parts = line.split()
                header[parts[0].upper()] = parts[1:]
            if line.upper().startswith("DATA "):
                break

        fields = header.get("FIELDS", header.get("FIELD"))
        if not fields or not all(axis in fields for axis in ("x", "y", "z")):
            raise ValueError(f"{path}: PCD does not contain x, y, z fields")
        sizes = [int(value) for value in header["SIZE"]]
        type_codes = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        point_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        data_mode = header["DATA"][0].lower()

        if not (len(fields) == len(sizes) == len(type_codes) == len(counts)):
            raise ValueError(f"{path}: inconsistent PCD field metadata")

        if data_mode == "ascii":
            values = np.loadtxt(stream, dtype=float, ndmin=2)
            scalar_columns: Dict[str, int] = {}
            column = 0
            for name, count in zip(fields, counts):
                scalar_columns[name] = column
                column += count
            points = np.column_stack([values[:, scalar_columns[axis]] for axis in "xyz"])

        elif data_mode == "binary":
            offsets = []
            formats = []
            offset = 0
            for size, type_code, count in zip(sizes, type_codes, counts):
                offsets.append(offset)
                base_type = _pcd_numpy_type(type_code, size)
                formats.append(base_type if count == 1 else (base_type, (count,)))
                offset += size * count
            dtype = np.dtype(
                {
                    "names": fields,
                    "formats": formats,
                    "offsets": offsets,
                    "itemsize": offset,
                }
            )
            raw = stream.read(point_count * dtype.itemsize)
            if len(raw) != point_count * dtype.itemsize:
                raise ValueError(f"{path}: truncated binary PCD payload")
            records = np.frombuffer(raw, dtype=dtype, count=point_count)
            points = np.column_stack([records[axis] for axis in "xyz"]).astype(float)

        elif data_mode == "binary_compressed":
            size_header = stream.read(8)
            if len(size_header) != 8:
                raise ValueError(f"{path}: missing binary_compressed size header")
            compressed_size, uncompressed_size = struct.unpack("<II", size_header)
            compressed = stream.read(compressed_size)
            if len(compressed) != compressed_size:
                raise ValueError(f"{path}: truncated binary_compressed payload")
            uncompressed = _lzf_decompress(compressed, uncompressed_size)

            arrays: Dict[str, np.ndarray] = {}
            offset = 0
            for name, size, type_code, count in zip(fields, sizes, type_codes, counts):
                field_bytes = point_count * size * count
                block = uncompressed[offset : offset + field_bytes]
                if len(block) != field_bytes:
                    raise ValueError(f"{path}: truncated decompressed field {name}")
                if name in ("x", "y", "z"):
                    values = np.frombuffer(
                        block, dtype=_pcd_numpy_type(type_code, size), count=point_count * count
                    ).reshape(point_count, count)
                    arrays[name] = values[:, 0]
                offset += field_bytes
            points = np.column_stack([arrays[axis] for axis in "xyz"]).astype(float)
        else:
            raise ValueError(f"{path}: unsupported PCD DATA mode {data_mode}")

    finite = np.isfinite(points).all(axis=1)
    return points[finite]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0:
        return points.copy()
    voxel_keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(voxel_keys, axis=0, return_inverse=True)
    downsampled = np.zeros((int(inverse.max()) + 1, 3), dtype=float)
    np.add.at(downsampled, inverse, points)
    downsampled /= np.bincount(inverse)[:, None]
    return downsampled


def estimate_normals(
    points: np.ndarray, tree: cKDTree | None = None, neighbors: int = 12
) -> Tuple[np.ndarray, np.ndarray]:
    if len(points) < neighbors:
        raise ValueError(f"Need at least {neighbors} points to estimate normals")
    if tree is None:
        tree = cKDTree(points)
    _, indices = tree.query(points, k=neighbors, workers=-1)
    neighborhoods = points[indices]
    centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
    covariances = np.einsum("nki,nkj->nij", centered, centered) / float(neighbors)
    eigenvalues, eigenvectors = np.linalg.eigh(covariances)
    normals = eigenvectors[:, :, 0]
    planarity = (eigenvalues[:, 1] - eigenvalues[:, 0]) / np.maximum(
        eigenvalues[:, 2], 1e-12
    )
    return normals, planarity


def synchronize_trajectories(
    trajectory90: PoseSeries,
    trajectory91: PoseSeries,
    tolerance_ms: float,
) -> Tuple[PoseSeries, PoseSeries, Dict[str, float]]:
    """Pair each LiDAR-90 pose with the closest unused LiDAR-91 pose."""
    if tolerance_ms <= 0.0:
        raise ValueError("trajectory pairing tolerance must be positive")

    tolerance_ns = int(round(tolerance_ms * 1e6))
    indices90: List[int] = []
    indices91: List[int] = []
    last91 = -1

    for index90, timestamp90 in enumerate(trajectory90.timestamps_ns):
        first_available = last91 + 1
        if first_available >= len(trajectory91.timestamps_ns):
            break
        insertion = first_available + int(
            np.searchsorted(
                trajectory91.timestamps_ns[first_available:], timestamp90, side="left"
            )
        )
        candidates = [
            index
            for index in (insertion - 1, insertion)
            if last91 < index < len(trajectory91.timestamps_ns)
        ]
        if not candidates:
            continue
        best91 = min(
            candidates,
            key=lambda index: abs(int(trajectory91.timestamps_ns[index]) - int(timestamp90)),
        )
        if abs(int(trajectory91.timestamps_ns[best91]) - int(timestamp90)) <= tolerance_ns:
            indices90.append(index90)
            indices91.append(best91)
            last91 = best91

    if len(indices90) < 100:
        raise ValueError(f"only {len(indices90)} synchronized trajectory pairs")

    trajectory90 = PoseSeries(
        trajectory90.timestamps_ns[indices90],
        trajectory90.translations[indices90],
        trajectory90.rotations[indices90],
    )
    trajectory91 = PoseSeries(
        trajectory91.timestamps_ns[indices91],
        trajectory91.translations[indices91],
        trajectory91.rotations[indices91],
    )
    delta_ms = (trajectory91.timestamps_ns - trajectory90.timestamps_ns) / 1e6
    timing = {
        "pair_count": int(len(indices90)),
        "median_delta_91_minus_90_ms": float(np.median(delta_ms)),
        "p95_absolute_delta_ms": float(np.percentile(np.abs(delta_ms), 95)),
        "maximum_absolute_delta_ms": float(np.max(np.abs(delta_ms))),
    }
    return trajectory90, trajectory91, timing


def build_relative_motions(
    trajectory90: PoseSeries, trajectory91: PoseSeries
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    count = len(trajectory90.translations)
    rotations_a: List[np.ndarray] = []
    translations_a: List[np.ndarray] = []
    rotations_b: List[np.ndarray] = []
    translations_b: List[np.ndarray] = []

    # At roughly 10 Hz these span 1--80 seconds.  Multiple baselines avoid
    # letting thousands of tiny, noisy scan-to-scan motions dominate.
    for lag in (10, 25, 50, 100, 200, 400, 800):
        if lag >= count:
            continue
        sampling_step = max(lag // 2, 10)
        for first in range(0, count - lag, sampling_step):
            second = first + lag
            rotation90_first_t = trajectory90.rotations[first].T
            rotation91_first_t = trajectory91.rotations[first].T
            rotation_a = rotation90_first_t @ trajectory90.rotations[second]
            translation_a = rotation90_first_t @ (
                trajectory90.translations[second] - trajectory90.translations[first]
            )
            rotation_b = rotation91_first_t @ trajectory91.rotations[second]
            translation_b = rotation91_first_t @ (
                trajectory91.translations[second] - trajectory91.translations[first]
            )
            rotation_amount = max(
                np.linalg.norm(Rotation.from_matrix(rotation_a).as_rotvec()),
                np.linalg.norm(Rotation.from_matrix(rotation_b).as_rotvec()),
            )
            translation_amount = max(np.linalg.norm(translation_a), np.linalg.norm(translation_b))
            if rotation_amount > math.radians(0.5) or translation_amount > 0.1:
                rotations_a.append(rotation_a)
                translations_a.append(translation_a)
                rotations_b.append(rotation_b)
                translations_b.append(translation_b)

    if len(rotations_a) < 50:
        raise ValueError("Too few informative relative motions for hand-eye calibration")
    return (
        np.asarray(rotations_a),
        np.asarray(translations_a),
        np.asarray(rotations_b),
        np.asarray(translations_b),
    )


def estimate_from_trajectories(
    trajectory90: PoseSeries, trajectory91: PoseSeries, timing: Dict[str, float]
) -> TransformEstimate:
    # Coarsely align the trajectory shapes only to initialize the two-sided
    # hand-eye problem.  This world-gauge alignment is not itself an extrinsic.
    map_rotation_seed, _ = rigid_fit(trajectory91.translations, trajectory90.translations)
    sample = slice(None, None, 20)
    rotation_candidates = np.einsum(
        "nij,jk,nkl->nil",
        trajectory90.rotations[sample].transpose(0, 2, 1),
        map_rotation_seed,
        trajectory91.rotations[sample],
    )
    rotation_seed = rotation_mean(rotation_candidates)

    rotation_a, translation_a, rotation_b, translation_b = build_relative_motions(
        trajectory90, trajectory91
    )
    translation_system = (rotation_a - np.eye(3)).reshape(-1, 3)
    translation_rhs = (
        np.einsum("ij,nj->ni", rotation_seed, translation_b) - translation_a
    ).reshape(-1)
    translation_seed = np.linalg.lstsq(translation_system, translation_rhs, rcond=None)[0]

    maximum_optimizer_motions = 3000
    if len(rotation_a) > maximum_optimizer_motions:
        selected = np.linspace(0, len(rotation_a) - 1, maximum_optimizer_motions).astype(int)
    else:
        selected = np.arange(len(rotation_a))
    opt_rotation_a = rotation_a[selected]
    opt_translation_a = translation_a[selected]
    opt_rotation_b = rotation_b[selected]
    opt_translation_b = translation_b[selected]

    rotation_scale = math.radians(0.3)
    translation_scale = 0.02

    def residual(parameters: np.ndarray) -> np.ndarray:
        rotation_x = Rotation.from_rotvec(parameters[:3]).as_matrix()
        translation_x = parameters[3:]
        left = np.einsum("nij,jk->nik", opt_rotation_a, rotation_x)
        right = np.einsum("ij,njk->nik", rotation_x, opt_rotation_b)
        rotation_error = np.einsum("nji,njk->nik", left, right)
        rotation_residual = Rotation.from_matrix(rotation_error).as_rotvec() / rotation_scale
        translation_residual = (
            np.einsum("nij,j->ni", opt_rotation_a, translation_x)
            + opt_translation_a
            - np.einsum("ij,nj->ni", rotation_x, opt_translation_b)
            - translation_x
        ) / translation_scale
        return np.column_stack((rotation_residual, translation_residual)).reshape(-1)

    initial = np.r_[Rotation.from_matrix(rotation_seed).as_rotvec(), translation_seed]
    solution = least_squares(
        residual,
        initial,
        loss="soft_l1",
        f_scale=2.0,
        max_nfev=100,
        xtol=1e-11,
        ftol=1e-11,
        gtol=1e-11,
    )
    rotation_x = Rotation.from_rotvec(solution.x[:3]).as_matrix()
    translation_x = solution.x[3:]

    left = np.einsum("nij,jk->nik", rotation_a, rotation_x)
    right = np.einsum("ij,njk->nik", rotation_x, rotation_b)
    rotation_errors = np.degrees(
        np.linalg.norm(
            Rotation.from_matrix(np.einsum("nji,njk->nik", left, right)).as_rotvec(), axis=1
        )
    )
    translation_errors = np.linalg.norm(
        np.einsum("nij,j->ni", rotation_a, translation_x)
        + translation_a
        - np.einsum("ij,nj->ni", rotation_x, translation_b)
        - translation_x,
        axis=1,
    )
    observability = np.linalg.svd(translation_system, compute_uv=False)
    diagnostics: Dict[str, object] = {
        **timing,
        "relative_motion_count": int(len(rotation_a)),
        "optimizer_motion_count": int(len(selected)),
        "solver_success": bool(solution.success),
        "solver_message": str(solution.message),
        "rotation_residual_median_deg": float(np.median(rotation_errors)),
        "rotation_residual_p95_deg": float(np.percentile(rotation_errors, 95)),
        "translation_residual_median_m": float(np.median(translation_errors)),
        "translation_residual_p95_m": float(np.percentile(translation_errors, 95)),
        "translation_observability_singular_values": observability.tolist(),
        "weakest_to_strongest_translation_observability_ratio": float(
            observability[-1] / observability[0]
        ),
    }
    return TransformEstimate("trajectory_to_trajectory", rotation_x, translation_x, diagnostics)


def load_pose_csv(path: Path) -> PoseSeries:
    rows = list(csv.DictReader(path.open(newline="")))
    if len(rows) < 2:
        raise ValueError(f"{path}: pose file has fewer than two rows")
    timestamps = np.array([int(row["timestamp_ns"]) for row in rows], dtype=np.int64)
    translations = np.array(
        [[float(row[f"lidar_t{axis}"]) for axis in "xyz"] for row in rows], dtype=float
    )
    quaternions = np.array(
        [[float(row[f"lidar_q{axis}"]) for axis in "xyzw"] for row in rows], dtype=float
    )
    return PoseSeries(timestamps, translations, Rotation.from_quat(quaternions).as_matrix())


def interpolate_two_pose_series(
    poses90: PoseSeries, poses91: PoseSeries, step_seconds: float = 1.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    epoch = min(int(poses90.timestamps_ns[0]), int(poses91.timestamps_ns[0]))
    time90 = (poses90.timestamps_ns - epoch) * 1e-9
    time91 = (poses91.timestamps_ns - epoch) * 1e-9
    start = max(float(time90[0]), float(time91[0]))
    stop = min(float(time90[-1]), float(time91[-1]))
    common_time = np.arange(start, stop, step_seconds)
    if len(common_time) < 10:
        raise ValueError("Optimized trajectories do not have enough common time")

    translation90 = np.column_stack(
        [np.interp(common_time, time90, poses90.translations[:, axis]) for axis in range(3)]
    )
    translation91 = np.column_stack(
        [np.interp(common_time, time91, poses91.translations[:, axis]) for axis in range(3)]
    )
    rotation90 = Slerp(time90, Rotation.from_matrix(poses90.rotations))(common_time).as_matrix()
    rotation91 = Slerp(time91, Rotation.from_matrix(poses91.rotations))(common_time).as_matrix()
    return rotation90, translation90, rotation91, translation91


def initial_map_alignment(
    extrinsic: TransformEstimate,
    optimized90: PoseSeries,
    optimized91: PoseSeries,
) -> Tuple[np.ndarray, np.ndarray]:
    rotation90, translation90, rotation91, translation91 = interpolate_two_pose_series(
        optimized90, optimized91, step_seconds=2.0
    )
    gauge_rotations = np.einsum(
        "nij,jk,nkl->nil",
        rotation90,
        extrinsic.rotation,
        rotation91.transpose(0, 2, 1),
    )
    gauge_rotation = rotation_mean(gauge_rotations)
    gauge_translations = (
        translation90
        + np.einsum("nij,j->ni", rotation90, extrinsic.translation)
        - np.einsum("ij,nj->ni", gauge_rotation, translation91)
    )
    return gauge_rotation, np.median(gauge_translations, axis=0)


def _clamp_icp_step(delta: np.ndarray, max_rotation_deg: float, max_translation_m: float) -> np.ndarray:
    delta = delta.copy()
    rotation_norm = np.linalg.norm(delta[:3])
    maximum_rotation = math.radians(max_rotation_deg)
    if rotation_norm > maximum_rotation:
        delta[:3] *= maximum_rotation / rotation_norm
    translation_norm = np.linalg.norm(delta[3:])
    if translation_norm > max_translation_m:
        delta[3:] *= max_translation_m / translation_norm
    return delta


def point_to_plane_map_icp(
    target: np.ndarray,
    source: np.ndarray,
    initial_rotation: np.ndarray,
    initial_translation: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object], Tuple[np.ndarray, np.ndarray]]:
    target_tree = cKDTree(target)
    target_normals, target_planarity = estimate_normals(target, target_tree, neighbors=15)
    source_normals, source_planarity = estimate_normals(source, neighbors=15)
    rotation = initial_rotation.copy()
    translation = initial_translation.copy()
    iterations = 0
    last_correspondence_count = 0

    for maximum_distance in (1.0, 0.5, 0.3, 0.15, 0.08):
        for _ in range(15):
            transformed = source @ rotation.T + translation
            distances, indices = target_tree.query(transformed, workers=-1)
            matched_normals = target_normals[indices]
            plane_residual = np.einsum(
                "ij,ij->i", matched_normals, transformed - target[indices]
            )
            transformed_source_normals = source_normals @ rotation.T
            normal_agreement = np.abs(
                np.einsum("ij,ij->i", transformed_source_normals, matched_normals)
            )
            mask = (
                (distances < maximum_distance)
                & (target_planarity[indices] > 0.05)
                & (source_planarity > 0.05)
                & (normal_agreement > math.cos(math.radians(60.0)))
            )
            if int(mask.sum()) < 100:
                raise RuntimeError("Map ICP found too few valid correspondences")
            robust_limit = np.percentile(np.abs(plane_residual[mask]), 85)
            mask &= np.abs(plane_residual) <= robust_limit
            design = np.column_stack(
                (
                    np.cross(transformed[mask], matched_normals[mask]),
                    matched_normals[mask],
                )
            )
            rhs = -plane_residual[mask]
            delta = np.linalg.lstsq(design, rhs, rcond=None)[0]
            delta = _clamp_icp_step(delta, max_rotation_deg=0.2, max_translation_m=0.02)
            delta_rotation = Rotation.from_rotvec(delta[:3]).as_matrix()
            rotation = delta_rotation @ rotation
            translation = delta_rotation @ translation + delta[3:]
            iterations += 1
            last_correspondence_count = int(mask.sum())
            if np.linalg.norm(delta[:3]) < 1e-6 and np.linalg.norm(delta[3:]) < 1e-6:
                break

    transformed = source @ rotation.T + translation
    forward_distance, forward_index = target_tree.query(transformed, workers=-1)
    reverse_distance, _ = cKDTree(transformed).query(target, workers=-1)
    final_plane_residual = np.abs(
        np.einsum(
            "ij,ij->i",
            target_normals[forward_index],
            transformed - target[forward_index],
        )
    )
    diagnostics: Dict[str, object] = {
        "icp_iterations": int(iterations),
        "last_inlier_correspondence_count": last_correspondence_count,
        "target_downsampled_points": int(len(target)),
        "source_downsampled_points": int(len(source)),
        "forward_nearest_median_m": float(np.median(forward_distance)),
        "forward_nearest_p95_m": float(np.percentile(forward_distance, 95)),
        "reverse_nearest_median_m": float(np.median(reverse_distance)),
        "reverse_nearest_p95_m": float(np.percentile(reverse_distance, 95)),
        "symmetric_overlap_fraction_at_0_30m": float(
            0.5 * (np.mean(forward_distance < 0.30) + np.mean(reverse_distance < 0.30))
        ),
        "point_to_plane_residual_median_m": float(np.median(final_plane_residual)),
        "point_to_plane_residual_p90_m": float(np.percentile(final_plane_residual, 90)),
    }
    return rotation, translation, diagnostics, (target_normals, target_planarity)


def robust_extrinsic_from_map_alignment(
    gauge_rotation: np.ndarray,
    gauge_translation: np.ndarray,
    optimized90: PoseSeries,
    optimized91: PoseSeries,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, float]]:
    rotation90, translation90, rotation91, translation91 = interpolate_two_pose_series(
        optimized90, optimized91, step_seconds=0.5
    )
    extrinsic_rotations = np.einsum(
        "nij,jk,nkl->nil",
        rotation90.transpose(0, 2, 1),
        gauge_rotation,
        rotation91,
    )
    rotation = rotation_mean(extrinsic_rotations)
    angle_error = np.degrees(
        np.linalg.norm(
            (
                Rotation.from_matrix(rotation).inv()
                * Rotation.from_matrix(extrinsic_rotations)
            ).as_rotvec(),
            axis=1,
        )
    )
    angle_median = np.median(angle_error)
    angle_mad = 1.4826 * np.median(np.abs(angle_error - angle_median))
    rotation_inliers = angle_error <= angle_median + 3.0 * max(angle_mad, 0.02)
    rotation = rotation_mean(extrinsic_rotations[rotation_inliers])

    extrinsic_translations = np.einsum(
        "nji,nj->ni",
        rotation90,
        gauge_translation
        + np.einsum("ij,nj->ni", gauge_rotation, translation91)
        - translation90,
    )
    translation = np.median(extrinsic_translations, axis=0)
    translation_error = np.linalg.norm(extrinsic_translations - translation, axis=1)
    translation_median = np.median(translation_error)
    translation_mad = 1.4826 * np.median(np.abs(translation_error - translation_median))
    translation_inliers = (
        translation_error <= translation_median + 3.0 * max(translation_mad, 0.002)
    )
    translation = np.median(extrinsic_translations[translation_inliers], axis=0)

    final_angle_error = np.degrees(
        np.linalg.norm(
            (
                Rotation.from_matrix(rotation).inv()
                * Rotation.from_matrix(extrinsic_rotations)
            ).as_rotvec(),
            axis=1,
        )
    )
    final_translation_error = np.linalg.norm(extrinsic_translations - translation, axis=1)
    diagnostics = {
        "interpolated_pose_samples": int(len(rotation90)),
        "pose_rotation_scatter_median_deg": float(np.median(final_angle_error)),
        "pose_rotation_scatter_p95_deg": float(np.percentile(final_angle_error, 95)),
        "pose_translation_scatter_median_m": float(np.median(final_translation_error)),
        "pose_translation_scatter_p95_m": float(np.percentile(final_translation_error, 95)),
    }
    return rotation, translation, diagnostics


def estimate_from_maps(
    lidar90_dir: Path,
    lidar91_dir: Path,
    trajectory_estimate: TransformEstimate,
    optimized90: PoseSeries,
    optimized91: PoseSeries,
    voxel_size: float,
) -> Tuple[TransformEstimate, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    map90 = read_pcd_xyz(lidar90_dir / "final_map.pcd")
    map91 = read_pcd_xyz(lidar91_dir / "final_map.pcd")
    map90_downsampled = voxel_downsample(map90, voxel_size)
    map91_downsampled = voxel_downsample(map91, voxel_size)

    gauge_rotation_seed, gauge_translation_seed = initial_map_alignment(
        trajectory_estimate, optimized90, optimized91
    )
    gauge_rotation, gauge_translation, icp_diagnostics, _ = point_to_plane_map_icp(
        map90_downsampled,
        map91_downsampled,
        gauge_rotation_seed,
        gauge_translation_seed,
    )
    rotation, translation, pose_diagnostics = robust_extrinsic_from_map_alignment(
        gauge_rotation, gauge_translation, optimized90, optimized91
    )
    diagnostics: Dict[str, object] = {
        "map_voxel_size_m": float(voxel_size),
        "raw_map90_points": int(len(map90)),
        "raw_map91_points": int(len(map91)),
        "gauge_transform_map91_to_map90": transform_matrix(
            gauge_rotation, gauge_translation
        ).tolist(),
        "gauge_seed_map91_to_map90": transform_matrix(
            gauge_rotation_seed, gauge_translation_seed
        ).tolist(),
        **icp_diagnostics,
        **pose_diagnostics,
    }
    estimate = TransformEstimate("map_to_map", rotation, translation, diagnostics)
    return estimate, map90_downsampled, map91_downsampled, gauge_rotation, gauge_translation


def load_keyframe_rows(path: Path) -> List[Dict[str, str]]:
    rows = list(csv.DictReader(path.open(newline="")))
    required = {"timestamp_ns", "lidar_cloud_file"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"{path}: expected timestamp_ns and lidar_cloud_file columns")
    return rows


def match_keyframes(
    rows90: Sequence[Dict[str, str]],
    rows91: Sequence[Dict[str, str]],
    tolerance_ms: float,
    maximum_pairs: int,
) -> List[Tuple[int, int, float]]:
    timestamps91 = np.array([int(row["timestamp_ns"]) for row in rows91], dtype=np.int64)
    candidates: List[Tuple[int, int, float]] = []
    for index90, row90 in enumerate(rows90):
        timestamp90 = int(row90["timestamp_ns"])
        insertion = int(np.searchsorted(timestamps91, timestamp90))
        possible = [index for index in (insertion - 1, insertion) if 0 <= index < len(rows91)]
        if not possible:
            continue
        index91 = min(possible, key=lambda index: abs(int(timestamps91[index]) - timestamp90))
        delta_ms = (int(timestamps91[index91]) - timestamp90) / 1e6
        if abs(delta_ms) <= tolerance_ms:
            candidates.append((index90, index91, float(delta_ms)))

    # Greedily enforce one-to-one matching, preferring the closest timestamps.
    used90 = set()
    used91 = set()
    unique: List[Tuple[int, int, float]] = []
    for match in sorted(candidates, key=lambda item: abs(item[2])):
        if match[0] not in used90 and match[1] not in used91:
            unique.append(match)
            used90.add(match[0])
            used91.add(match[1])
    unique.sort(key=lambda item: item[0])

    if maximum_pairs > 0 and len(unique) > maximum_pairs:
        selected = np.linspace(0, len(unique) - 1, maximum_pairs).astype(int)
        unique = [unique[index] for index in selected]
    return unique


def multi_scan_point_to_plane_icp(
    scan_pairs: Sequence[
        Tuple[np.ndarray, np.ndarray, cKDTree, np.ndarray, np.ndarray, np.ndarray, np.ndarray]
    ],
    initial_rotation: np.ndarray,
    initial_translation: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    rotation = initial_rotation.copy()
    translation = initial_translation.copy()
    iterations = 0
    last_count = 0

    for maximum_distance in (0.5, 0.3, 0.2, 0.12):
        for _ in range(15):
            designs: List[np.ndarray] = []
            right_sides: List[np.ndarray] = []
            for (
                target,
                source,
                target_tree,
                target_normals,
                target_planarity,
                source_normals,
                source_planarity,
            ) in scan_pairs:
                transformed = source @ rotation.T + translation
                distances, indices = target_tree.query(transformed)
                matched_normals = target_normals[indices]
                plane_residual = np.einsum(
                    "ij,ij->i", matched_normals, transformed - target[indices]
                )
                transformed_source_normals = source_normals @ rotation.T
                normal_agreement = np.abs(
                    np.einsum("ij,ij->i", transformed_source_normals, matched_normals)
                )
                mask = (
                    (distances < maximum_distance)
                    & (target_planarity[indices] > 0.05)
                    & (source_planarity > 0.05)
                    & (normal_agreement > math.cos(math.radians(45.0)))
                )
                if int(mask.sum()) < 30:
                    continue
                robust_limit = np.percentile(np.abs(plane_residual[mask]), 85)
                mask &= np.abs(plane_residual) <= robust_limit
                selected = np.flatnonzero(mask)
                if len(selected) > 500:
                    selected = selected[
                        np.linspace(0, len(selected) - 1, 500).astype(int)
                    ]
                designs.append(
                    np.column_stack(
                        (
                            np.cross(transformed[selected], matched_normals[selected]),
                            matched_normals[selected],
                        )
                    )
                )
                right_sides.append(-plane_residual[selected])

            if not designs:
                raise RuntimeError("Scan ICP found no usable correspondences")
            design = np.vstack(designs)
            rhs = np.hstack(right_sides)
            if len(rhs) < 500:
                raise RuntimeError("Scan ICP found too few usable correspondences")
            delta = np.linalg.lstsq(design, rhs, rcond=None)[0]
            delta = _clamp_icp_step(delta, max_rotation_deg=0.2, max_translation_m=0.02)
            delta_rotation = Rotation.from_rotvec(delta[:3]).as_matrix()
            rotation = delta_rotation @ rotation
            translation = delta_rotation @ translation + delta[3:]
            iterations += 1
            last_count = int(len(rhs))
            if np.linalg.norm(delta[:3]) < 1e-6 and np.linalg.norm(delta[3:]) < 1e-6:
                break

    nearest_distances: List[np.ndarray] = []
    plane_residuals: List[np.ndarray] = []
    inlier_plane_residuals: List[np.ndarray] = []
    for (
        target,
        source,
        target_tree,
        target_normals,
        _,
        _,
        _,
    ) in scan_pairs:
        transformed = source @ rotation.T + translation
        distances, indices = target_tree.query(transformed)
        residual = np.abs(
            np.einsum("ij,ij->i", target_normals[indices], transformed - target[indices])
        )
        nearest_distances.append(distances)
        plane_residuals.append(residual)
        inlier_plane_residuals.append(residual[distances < 0.20])

    all_distances = np.hstack(nearest_distances)
    all_plane_residuals = np.hstack(plane_residuals)
    all_inlier_plane_residuals = np.hstack(inlier_plane_residuals)
    diagnostics: Dict[str, object] = {
        "icp_iterations": int(iterations),
        "last_inlier_correspondence_count": last_count,
        "source_point_count": int(len(all_distances)),
        "nearest_distance_median_m": float(np.median(all_distances)),
        "nearest_distance_p90_m": float(np.percentile(all_distances, 90)),
        "overlap_fraction_at_0_20m": float(np.mean(all_distances < 0.20)),
        "all_point_to_plane_residual_median_m": float(np.median(all_plane_residuals)),
        "overlap_point_to_plane_residual_median_m": float(
            np.median(all_inlier_plane_residuals)
        ),
        "overlap_point_to_plane_residual_p90_m": float(
            np.percentile(all_inlier_plane_residuals, 90)
        ),
    }
    return rotation, translation, diagnostics


def estimate_from_scans(
    lidar90_dir: Path,
    lidar91_dir: Path,
    trajectory_estimate: TransformEstimate,
    tolerance_ms: float,
    maximum_pairs: int,
) -> TransformEstimate:
    rows90 = load_keyframe_rows(lidar90_dir / "keyframe_poses_lidar.csv")
    rows91 = load_keyframe_rows(lidar91_dir / "keyframe_poses_lidar.csv")
    matches = match_keyframes(rows90, rows91, tolerance_ms, maximum_pairs)
    if len(matches) < 10:
        raise RuntimeError(
            f"Only {len(matches)} keyframe pairs are within {tolerance_ms:g} ms; need at least 10"
        )

    scan_pairs = []
    for index90, index91, _ in matches:
        target = read_pcd_xyz(lidar90_dir / rows90[index90]["lidar_cloud_file"])
        source = read_pcd_xyz(lidar91_dir / rows91[index91]["lidar_cloud_file"])
        target_tree = cKDTree(target)
        target_normals, target_planarity = estimate_normals(target, target_tree, neighbors=10)
        source_normals, source_planarity = estimate_normals(source, neighbors=10)
        scan_pairs.append(
            (
                target,
                source,
                target_tree,
                target_normals,
                target_planarity,
                source_normals,
                source_planarity,
            )
        )

    rotation, translation, icp_diagnostics = multi_scan_point_to_plane_icp(
        scan_pairs, trajectory_estimate.rotation, trajectory_estimate.translation
    )
    delta_ms = np.array([match[2] for match in matches])
    diagnostics: Dict[str, object] = {
        "matched_scan_pairs": int(len(matches)),
        "scan_pair_tolerance_ms": float(tolerance_ms),
        "median_delta_91_minus_90_ms": float(np.median(delta_ms)),
        "maximum_absolute_delta_ms": float(np.max(np.abs(delta_ms))),
        "matched_keyframe_indices": [
            {"lidar90": int(index90), "lidar91": int(index91), "delta_ms": float(delta)}
            for index90, index91, delta in matches
        ],
        **icp_diagnostics,
    }
    return TransformEstimate("scan_to_scan", rotation, translation, diagnostics)


def validate_method_agreement(estimates: Sequence[TransformEstimate]) -> Dict[str, object]:
    """Compare methods without pretending their shared inputs are independent."""
    disagreements = []
    for first in range(len(estimates)):
        for second in range(first + 1, len(estimates)):
            disagreements.append(
                {
                    "methods": [estimates[first].name, estimates[second].name],
                    "translation_m": float(
                        np.linalg.norm(estimates[first].translation - estimates[second].translation)
                    ),
                    "rotation_deg": rotation_distance_deg(
                        estimates[first].rotation, estimates[second].rotation
                    ),
                }
            )
    maximum_translation = max(item["translation_m"] for item in disagreements)
    maximum_rotation = max(item["rotation_deg"] for item in disagreements)
    diagnostics: Dict[str, object] = {
        "pairwise_disagreement": disagreements,
        "maximum_pairwise_translation_disagreement_m": float(maximum_translation),
        "maximum_pairwise_rotation_disagreement_deg": float(maximum_rotation),
        "methods_consistent": bool(maximum_translation < 0.10 and maximum_rotation < 1.0),
        "consistency_threshold_translation_m": 0.10,
        "consistency_threshold_rotation_deg": 1.0,
        "interpretation": (
            "Agreement check only, not an independent statistical confidence interval. "
            "Map and scan ICP use the trajectory result as a local-registration seed, "
            "and the map estimate also uses optimized SLAM poses to remove map gauge."
        ),
    }
    return diagnostics


def estimate_to_dict(estimate: TransformEstimate) -> Dict[str, object]:
    quaternion = Rotation.from_matrix(estimate.rotation).as_quat()
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    inverse_rotation, inverse_translation = invert_transform(
        estimate.rotation, estimate.translation
    )
    return {
        "name": estimate.name,
        "direction": "lidar_91_to_lidar_90",
        "equation": "p_lidar90 = R * p_lidar91 + t",
        "translation_m": {
            axis: float(value) for axis, value in zip("xyz", estimate.translation)
        },
        "quaternion_xyzw": {
            axis: float(value) for axis, value in zip("xyzw", quaternion)
        },
        "rpy_xyz_deg": {
            axis: float(value)
            for axis, value in zip(
                "rpy",
                Rotation.from_matrix(estimate.rotation).as_euler("xyz", degrees=True),
            )
        },
        "matrix_T_lidar90_lidar91": transform_matrix(
            estimate.rotation, estimate.translation
        ).tolist(),
        "inverse_matrix_T_lidar91_lidar90": transform_matrix(
            inverse_rotation, inverse_translation
        ).tolist(),
        "diagnostics": estimate.diagnostics,
    }


def write_overlay_ply(
    path: Path,
    map90: np.ndarray,
    map91: np.ndarray,
    gauge_rotation: np.ndarray,
    gauge_translation: np.ndarray,
) -> None:
    transformed91 = map91 @ gauge_rotation.T + gauge_translation
    points = np.vstack((map90, transformed91)).astype("<f4")
    colors = np.empty((len(points), 3), dtype=np.uint8)
    colors[: len(map90)] = (40, 190, 255)
    colors[len(map90) :] = (255, 80, 170)
    records = np.empty(
        len(points),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    records["x"], records["y"], records["z"] = points.T
    records["red"], records["green"], records["blue"] = colors.T
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment cyan=map90 magenta=map91_aligned_to_map90\n"
        f"element vertex {len(records)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        records.tofile(stream)


def write_extrinsic_yaml(path: Path, estimate: TransformEstimate) -> None:
    quaternion = Rotation.from_matrix(estimate.rotation).as_quat()
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    matrix = transform_matrix(estimate.rotation, estimate.translation)
    inverse_rotation, inverse_translation = invert_transform(
        estimate.rotation, estimate.translation
    )
    inverse_matrix = transform_matrix(inverse_rotation, inverse_translation)

    lines = [
        "# Recommended scan-to-scan transform, cross-checked against trajectories and maps",
        "# Convention: p_lidar90 = R * p_lidar91 + t",
        f"method: {estimate.name}",
        "parent_frame: lidar_90",
        "child_frame: lidar_91",
        "direction: lidar_91_to_lidar_90",
        "translation_m:",
        f"  x: {estimate.translation[0]:.12f}",
        f"  y: {estimate.translation[1]:.12f}",
        f"  z: {estimate.translation[2]:.12f}",
        "quaternion_xyzw:",
        f"  x: {quaternion[0]:.12f}",
        f"  y: {quaternion[1]:.12f}",
        f"  z: {quaternion[2]:.12f}",
        f"  w: {quaternion[3]:.12f}",
        "matrix_T_lidar90_lidar91:",
    ]
    lines.extend("  - [" + ", ".join(f"{value:.12f}" for value in row) + "]" for row in matrix)
    lines.append("inverse_matrix_T_lidar91_lidar90:")
    lines.extend(
        "  - [" + ", ".join(f"{value:.12f}" for value in row) + "]"
        for row in inverse_matrix
    )
    path.write_text("\n".join(lines) + "\n")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Estimate T_lidar90_lidar91 using trajectory hand-eye, map-to-map, "
            "and synchronized scan-to-scan calibration."
        )
    )
    parser.add_argument(
        "--lidar90-dir",
        type=Path,
        default=ROOT / "lidar_90" / "attempt_001",
        help="LiDAR 90 session directory",
    )
    parser.add_argument(
        "--lidar91-dir",
        type=Path,
        default=ROOT / "lidar_91" / "attempt_001",
        help="LiDAR 91 session directory",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "calibration_results",
        help="Directory for the JSON/YAML results and map overlay",
    )
    parser.add_argument(
        "--trajectory-tolerance-ms",
        type=float,
        default=5.0,
        help="Maximum timestamp separation for trajectory pairs (default: 5 ms)",
    )
    parser.add_argument(
        "--map-voxel-size",
        type=float,
        default=0.25,
        help="Map ICP voxel size in metres (default: 0.25)",
    )
    parser.add_argument(
        "--scan-tolerance-ms",
        type=float,
        default=5.0,
        help="Maximum timestamp separation for scan pairs (default: 5 ms)",
    )
    parser.add_argument(
        "--max-scan-pairs",
        type=int,
        default=60,
        help="Maximum evenly distributed scan pairs; 0 keeps all (default: 60)",
    )
    parser.add_argument(
        "--no-overlay",
        action="store_true",
        help="Do not write the colored downsampled map overlay PLY",
    )
    return parser.parse_args()


def print_estimate(estimate: TransformEstimate) -> None:
    quaternion = Rotation.from_matrix(estimate.rotation).as_quat()
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    euler = Rotation.from_matrix(estimate.rotation).as_euler("xyz", degrees=True)
    print(f"\n{estimate.name}")
    print("  translation xyz [m]: " + " ".join(f"{value: .6f}" for value in estimate.translation))
    print("  RPY xyz [deg]:       " + " ".join(f"{value: .6f}" for value in euler))
    print("  quaternion xyzw:     " + " ".join(f"{value: .9f}" for value in quaternion))


def main() -> int:
    arguments = parse_arguments()
    for path in (arguments.lidar90_dir, arguments.lidar91_dir):
        if not path.is_dir():
            raise FileNotFoundError(path)
    if arguments.map_voxel_size <= 0.0:
        raise ValueError("--map-voxel-size must be positive")
    if arguments.scan_tolerance_ms <= 0.0:
        raise ValueError("--scan-tolerance-ms must be positive")

    print("[1/3] Trajectory-to-trajectory hand-eye calibration")
    trajectory90 = load_pose_csv(
        arguments.lidar90_dir / "trajectory_scan_poses_lidar.csv"
    )
    trajectory91 = load_pose_csv(
        arguments.lidar91_dir / "trajectory_scan_poses_lidar.csv"
    )
    trajectory90, trajectory91, timing = synchronize_trajectories(
        trajectory90, trajectory91, arguments.trajectory_tolerance_ms
    )
    trajectory_estimate = estimate_from_trajectories(trajectory90, trajectory91, timing)
    print_estimate(trajectory_estimate)

    optimized90 = load_pose_csv(arguments.lidar90_dir / "keyframe_poses_optimized_lidar.csv")
    optimized91 = load_pose_csv(arguments.lidar91_dir / "keyframe_poses_optimized_lidar.csv")

    print("\n[2/3] Map-to-map point-to-plane registration")
    (
        map_estimate,
        map90_downsampled,
        map91_downsampled,
        gauge_rotation,
        gauge_translation,
    ) = estimate_from_maps(
        arguments.lidar90_dir,
        arguments.lidar91_dir,
        trajectory_estimate,
        optimized90,
        optimized91,
        arguments.map_voxel_size,
    )
    print_estimate(map_estimate)

    print("\n[3/3] Joint scan-to-scan point-to-plane registration")
    scan_estimate = estimate_from_scans(
        arguments.lidar90_dir,
        arguments.lidar91_dir,
        trajectory_estimate,
        arguments.scan_tolerance_ms,
        arguments.max_scan_pairs,
    )
    print_estimate(scan_estimate)

    validation = validate_method_agreement(
        [trajectory_estimate, map_estimate, scan_estimate]
    )
    # The synchronized local clouds directly measure the desired LiDAR-to-LiDAR
    # geometry.  The other two methods validate it but are not independent data.
    recommended = scan_estimate
    print("\nRecommended final transform: scan_to_scan")

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "transform_convention": {
            "name": "T_lidar90_lidar91",
            "direction": "lidar_91_to_lidar_90",
            "equation": "p_lidar90 = T_lidar90_lidar91 @ p_lidar91",
            "note": (
                "Map and scan ICP are local refinements initialized by the trajectory "
                "estimate. Their agreement checks compatibility of the local solution; "
                "it is not an independent statistical validation."
            ),
        },
        "recommended": estimate_to_dict(recommended),
        "validation": validation,
        "methods": {
            estimate.name: estimate_to_dict(estimate)
            for estimate in (trajectory_estimate, map_estimate, scan_estimate)
        },
        "input_paths": {
            "lidar90_dir": str(arguments.lidar90_dir.resolve()),
            "lidar91_dir": str(arguments.lidar91_dir.resolve()),
            "lidar90_trajectory": str(
                (arguments.lidar90_dir / "trajectory_scan_poses_lidar.csv").resolve()
            ),
            "lidar91_trajectory": str(
                (arguments.lidar91_dir / "trajectory_scan_poses_lidar.csv").resolve()
            ),
        },
    }
    report_path = arguments.output_dir / "calibration_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    yaml_path = arguments.output_dir / "extrinsic_lidar91_to_lidar90.yaml"
    write_extrinsic_yaml(yaml_path, recommended)

    overlay_path = arguments.output_dir / "map_overlay_downsampled.ply"
    if not arguments.no_overlay:
        write_overlay_ply(
            overlay_path,
            map90_downsampled,
            map91_downsampled,
            gauge_rotation,
            gauge_translation,
        )

    consistent = bool(validation["methods_consistent"])
    print("\nValidation")
    print(
        "  max method disagreement: "
        f"{validation['maximum_pairwise_translation_disagreement_m']:.4f} m, "
        f"{validation['maximum_pairwise_rotation_disagreement_deg']:.4f} deg"
    )
    print(f"  cross-method local agreement: {'PASS' if consistent else 'REVIEW'}")
    print("  note: methods share data/initialization; PASS is not a statistical confidence bound")
    print(f"\nWrote {report_path}")
    print(f"Wrote {yaml_path}")
    if not arguments.no_overlay:
        print(f"Wrote {overlay_path}")
    return 0 if consistent else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
