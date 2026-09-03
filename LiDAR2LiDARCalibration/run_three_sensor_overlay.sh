#!/usr/bin/env bash
set -euo pipefail

OVERLAY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_BUILD_ROOT="${OVERLAY_ROOT}/.three_sensor_overlay_ws"
DEFAULT_BAG_PATH="$(realpath "${OVERLAY_ROOT}/../..")"
DEFAULT_LIDAR_CALIB_PATH="${OVERLAY_ROOT}/calibration_results/extrinsic_lidar91_to_lidar90.yaml"
DEFAULT_CAMERA_CALIB_PATH="${OVERLAY_ROOT}/calibration_vision_lidar91/calib.json"

ROS_SETUP="/opt/ros/humble/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 Humble was not found at ${ROS_SETUP}" >&2
  exit 1
fi

BAG_PATH="${BAG_PATH:-${DEFAULT_BAG_PATH}}"
LIDAR_CALIB_PATH="${LIDAR_CALIB_PATH:-${DEFAULT_LIDAR_CALIB_PATH}}"
CAMERA_CALIB_PATH="${CAMERA_CALIB_PATH:-${DEFAULT_CAMERA_CALIB_PATH}}"
TAG_SIZE_M="${TAG_SIZE_M:-0.0872}"
TAG_FAMILY="${TAG_FAMILY:-Standard41h12}"
APRILTAG="${APRILTAG:-true}"
PUBLISH_SENSOR_TF="${PUBLISH_SENSOR_TF:-true}"
# This wrapper always replays the historical bag, which contains depth images
# but no color-to-depth TF. Enable the explicit preview-only identity TF so the
# depth cloud is visible by default. Set false when a real camera TF is present.
ASSUME_DEPTH_ALIGNED="${ASSUME_DEPTH_ALIGNED:-true}"
PLAY_RATE="${PLAY_RATE:-1.0}"
START_OFFSET="${START_OFFSET:-0.0}"

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "No ROS 2 bag metadata found at ${BAG_PATH}/metadata.yaml" >&2
  echo "Set BAG_PATH=/absolute/path/to/bag and run again." >&2
  exit 1
fi
if [[ ! -f "${LIDAR_CALIB_PATH}" ]]; then
  echo "LiDAR calibration file not found: ${LIDAR_CALIB_PATH}" >&2
  exit 1
fi
if [[ ! -f "${CAMERA_CALIB_PATH}" ]]; then
  echo "Camera-LiDAR calibration file not found: ${CAMERA_CALIB_PATH}" >&2
  exit 1
fi
LIDAR_CALIB_PATH="$(realpath -- "${LIDAR_CALIB_PATH}")"
CAMERA_CALIB_PATH="$(realpath -- "${CAMERA_CALIB_PATH}")"

# shellcheck disable=SC1090
set +u
source "${ROS_SETUP}"
set -u

# VS Code is running from Snap on this host and exports GTK/GIO paths whose
# bundled glibc is incompatible with RViz. Keep those paths out of child nodes.
unset GIO_MODULE_DIR GTK_PATH GTK_EXE_PREFIX
unset GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE
unset GTK_IM_MODULE_FILE GSETTINGS_SCHEMA_DIR LOCPATH

colcon --log-base "${OVERLAY_BUILD_ROOT}/log" build \
  --base-paths "${OVERLAY_ROOT}/three_sensor_overlay" \
  --packages-select three_sensor_overlay \
  --build-base "${OVERLAY_BUILD_ROOT}/build" \
  --install-base "${OVERLAY_BUILD_ROOT}/install" \
  --event-handlers console_direct+

# shellcheck disable=SC1090
set +u
source "${OVERLAY_BUILD_ROOT}/install/setup.bash"
set -u

echo "Bag: ${BAG_PATH}"
echo "LiDAR calibration: ${LIDAR_CALIB_PATH}"
echo "Camera-LiDAR calibration: ${CAMERA_CALIB_PATH}"
echo "AprilTag: ${TAG_FAMILY}, black-square edge ${TAG_SIZE_M} m"
echo "Tag image: /overlay/camera/apriltag_overlay"
echo "Depth cloud: /overlay/camera/depth_points"
if [[ "${ASSUME_DEPTH_ALIGNED}" == "true" ]]; then
  echo "WARNING: using an identity color-to-depth TF for preview only; the bag did not record the factory TF." >&2
else
  echo "Depth identity preview disabled; a real camera_color_optical_frame -> camera_depth_optical_frame TF is required." >&2
fi
echo "Press Ctrl-C once to stop RViz, detector, overlay node, and bag playback."

exec ros2 launch three_sensor_overlay three_sensor_overlay.launch.py \
  play_bag:=true \
  bag_path:="${BAG_PATH}" \
  apriltag:="${APRILTAG}" \
  tag_family:="${TAG_FAMILY}" \
  tag_size_m:="${TAG_SIZE_M}" \
  lidar_calibration_path:="${LIDAR_CALIB_PATH}" \
  camera_calibration_path:="${CAMERA_CALIB_PATH}" \
  publish_sensor_tf:="${PUBLISH_SENSOR_TF}" \
  assume_depth_aligned:="${ASSUME_DEPTH_ALIGNED}" \
  play_rate:="${PLAY_RATE}" \
  start_offset:="${START_OFFSET}" \
  "$@"
