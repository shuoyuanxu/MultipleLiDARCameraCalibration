#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Record one stationary MID-360/camera calibration bag.

Usage:
  scripts/record_mid360_calibration.sh BAG_PARENT IMAGE_TOPIC CAMERA_INFO_TOPIC POINTS_TOPIC [POINTS_TOPIC ...]

Example (the topics already seen on this PC):
  scripts/record_mid360_calibration.sh "$HOME/vlcal_data/bags" \
    /camera/color/image_raw/compressed \
    /camera/color/camera_info \
    /lidar/lidar_90 /lidar/lidar_91

Environment:
  VLCAL_RECORD_SECONDS   Recording length; default 15
  ROS_SETUP              ROS setup file; default /opt/ros/humble/setup.bash
  SENSOR_WS_SETUP        Optional sensor-workspace setup file
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

[[ $# -ge 4 ]] || { usage; exit 1; }

bag_parent="$1"
image_topic="$2"
camera_info_topic="$3"
shift 3
points_topics=("$@")
duration="${VLCAL_RECORD_SECONDS:-15}"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
sensor_setup="${SENSOR_WS_SETUP:-}"

[[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "VLCAL_RECORD_SECONDS must be a positive number"
[[ -r "$ros_setup" ]] || die "cannot read ROS setup: $ros_setup"
# shellcheck disable=SC1090
source "$ros_setup"
if [[ -n "$sensor_setup" ]]; then
  [[ -r "$sensor_setup" ]] || die "cannot read sensor workspace setup: $sensor_setup"
  # shellcheck disable=SC1090
  source "$sensor_setup"
fi

command -v ros2 >/dev/null 2>&1 || die "ros2 is not available after sourcing $ros_setup"

check_topic() {
  local topic="$1"
  shift
  local actual
  actual="$(timeout 5 ros2 topic type "$topic" 2>/dev/null || true)"
  [[ -n "$actual" ]] || die "no live publisher/type found for $topic"

  local wanted
  for wanted in "$@"; do
    [[ "$actual" == "$wanted" ]] && return 0
  done
  die "$topic has type $actual; expected one of: $*"
}

check_topic "$image_topic" sensor_msgs/msg/Image sensor_msgs/msg/CompressedImage
check_topic "$camera_info_topic" sensor_msgs/msg/CameraInfo
for topic in "${points_topics[@]}"; do
  check_topic "$topic" sensor_msgs/msg/PointCloud2
done

mkdir -p "$bag_parent"
bag_parent="$(realpath "$bag_parent")"
bag_name="pose_$(date +%Y%m%d_%H%M%S)"
bag_path="$bag_parent/$bag_name"
topics=("$image_topic" "$camera_info_topic" "${points_topics[@]}")

cat <<EOF
Ready to record: $bag_path
Duration:        ${duration}s
Topics:          ${topics[*]}

The vehicle must already be parked, brakes set, suspension settled, and the
camera exposure stable. Do not move the vehicle or sensors during this run.
EOF

status=0
timeout --foreground --signal=INT --kill-after=10 "${duration}s" \
  ros2 bag record \
    --storage sqlite3 \
    --max-cache-size 536870912 \
    --output "$bag_path" \
    "${topics[@]}" || status=$?

# GNU timeout returns 124 when it ends the command at the requested duration.
if [[ "$status" -ne 0 && "$status" -ne 124 && "$status" -ne 130 ]]; then
  die "ros2 bag record failed with status $status"
fi

[[ -s "$bag_path/metadata.yaml" ]] || die "the bag did not finalize correctly: $bag_path"
printf '\nRecorded bag:\n'
ros2 bag info "$bag_path"
printf '\nNext: keep the rig rigid, drive to a new view, stop fully, and run this script again.\n'
