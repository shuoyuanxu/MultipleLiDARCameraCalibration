#!/usr/bin/env bash
set -Eeuo pipefail

readonly DEFAULT_IMAGE="koide3/direct_visual_lidar_calibration:humble"
IMAGE="${VLCAL_IMAGE:-$DEFAULT_IMAGE}"

usage() {
  cat <<'EOF'
Run direct_visual_lidar_calibration in Docker.

Usage:
  docker/vlcal.sh doctor
  docker/vlcal.sh pull
  docker/vlcal.sh bag-info BAG_DIRECTORY
  docker/vlcal.sh preprocess BAG_PARENT OUTPUT_DIR POINTS_TOPIC IMAGE_TOPIC CAMERA_INFO_TOPIC [extra preprocess options]
  docker/vlcal.sh preprocess-gui BAG_PARENT OUTPUT_DIR POINTS_TOPIC IMAGE_TOPIC CAMERA_INFO_TOPIC [extra preprocess options]
  docker/vlcal.sh manual OUTPUT_DIR
  docker/vlcal.sh calibrate OUTPUT_DIR [extra calibrate options]
  docker/vlcal.sh calibrate-headless OUTPUT_DIR [extra calibrate options]
  docker/vlcal.sh viewer OUTPUT_DIR

Notes:
  * BAG_PARENT must contain one or more rosbag2 directories as immediate children.
  * MID-360 data should normally use static integration, so this wrapper never adds -d.
  * Set VLCAL_OVERWRITE=1 only when intentionally replacing an existing calib.json.
  * Set VLCAL_SOFTWARE_GL=1 to force Mesa software rendering for GUI commands.
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

have_current_docker_access() {
  command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1
}

require_docker() {
  command -v docker >/dev/null 2>&1 || die "Docker is not installed"
  if ! docker info >/dev/null 2>&1; then
    if getent group docker 2>/dev/null | grep -Eq "(^|,)${USER}(,|$)"; then
      die "your docker-group membership is not active in this shell; run 'newgrp docker' (or log out and back in), then retry"
    fi
    die "cannot access the Docker daemon; add this user to the docker group or use rootless Docker"
  fi
}

require_image() {
  docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE is not present; run '$0 pull' first"
}

existing_dir() {
  local path="$1"
  [[ -d "$path" ]] || die "directory does not exist: $path"
  realpath "$path"
}

output_dir() {
  local path="$1"
  mkdir -p "$path"
  realpath "$path"
}

fix_output_ownership() {
  local path="$1"
  docker run --rm \
    --entrypoint /bin/chown \
    --mount "type=bind,src=$path,dst=/data/output" \
    "$IMAGE" -R "$(id -u):$(id -g)" /data/output >/dev/null
}

build_gui_args() {
  [[ -n "${DISPLAY:-}" ]] || die "DISPLAY is unset; run GUI commands from the logged-in desktop session"

  local auth_file="${XAUTHORITY:-}"
  if [[ -z "$auth_file" && -r "${HOME}/.Xauthority" ]]; then
    auth_file="${HOME}/.Xauthority"
  fi
  [[ -n "$auth_file" && -r "$auth_file" ]] || die "cannot read the Xauthority cookie; export XAUTHORITY to the active Xwayland/X11 authority file"

  GUI_ARGS=(
    --network host
    --env "DISPLAY=$DISPLAY"
    --env XAUTHORITY=/tmp/vlcal.xauth
    --env QT_X11_NO_MITSHM=1
    --mount "type=bind,src=$auth_file,dst=/tmp/vlcal.xauth,readonly"
  )

  if [[ -d /tmp/.X11-unix ]]; then
    GUI_ARGS+=(--mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix)
  fi
  if [[ -d /dev/dri ]]; then
    GUI_ARGS+=(--mount type=bind,src=/dev/dri,dst=/dev/dri)
  fi

  if [[ "${VLCAL_SOFTWARE_GL:-0}" == "1" ]]; then
    GUI_ARGS+=(--env LIBGL_ALWAYS_SOFTWARE=1)
  elif docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q 'nvidia'; then
    GUI_ARGS+=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
  else
    # Mesa's software renderer is slower, but sufficient for manual picking
    # and inspection when the NVIDIA Container Toolkit is unavailable.
    GUI_ARGS+=(--env LIBGL_ALWAYS_SOFTWARE=1)
  fi
}

run_with_output() {
  local output="$1"
  shift
  local status=0
  docker run "$@" || status=$?
  fix_output_ownership "$output"
  return "$status"
}

command_doctor() {
  printf 'Docker CLI: '
  if command -v docker >/dev/null 2>&1; then
    docker --version
  else
    printf 'missing\n'
  fi

  printf 'Docker daemon access: '
  if have_current_docker_access; then
    printf 'OK\n'
  else
    printf 'FAILED (run: newgrp docker)\n'
    return 1
  fi

  printf 'Calibration image: '
  if docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker image inspect --format '{{index .RepoTags 0}}  {{.Id}}' "$IMAGE"
  else
    printf 'missing (run: %s pull)\n' "$0"
  fi

  printf 'NVIDIA container runtime: '
  if docker info --format '{{json .Runtimes}}' | grep -q 'nvidia'; then
    printf 'available\n'
  else
    printf 'not installed; GUI will use software OpenGL\n'
  fi

  printf 'Display: %s\n' "${DISPLAY:-unset}"
  printf 'Xauthority: %s\n' "${XAUTHORITY:-${HOME}/.Xauthority}"
}

command_pull() {
  require_docker
  docker pull "$IMAGE"
}

command_bag_info() {
  [[ $# -eq 1 ]] || die "bag-info needs BAG_DIRECTORY"
  require_docker
  require_image
  local bag
  bag="$(existing_dir "$1")"
  docker run --rm \
    --mount "type=bind,src=$bag,dst=/data/bag,readonly" \
    "$IMAGE" ros2 bag info /data/bag
}

command_preprocess() {
  local gui="$1"
  shift
  [[ $# -ge 5 ]] || die "preprocess needs BAG_PARENT OUTPUT_DIR POINTS_TOPIC IMAGE_TOPIC CAMERA_INFO_TOPIC"
  require_docker
  require_image

  local input output points_topic image_topic camera_info_topic
  input="$(existing_dir "$1")"
  output="$(output_dir "$2")"
  points_topic="$3"
  image_topic="$4"
  camera_info_topic="$5"
  shift 5

  if [[ -f "$output/calib.json" && "${VLCAL_OVERWRITE:-0}" != "1" ]]; then
    die "$output/calib.json already exists; choose a new output or set VLCAL_OVERWRITE=1"
  fi

  local docker_args=(
    --rm
    --mount "type=bind,src=$input,dst=/data/input,readonly"
    --mount "type=bind,src=$output,dst=/data/output"
  )
  local program_args=(
    ros2 run direct_visual_lidar_calibration preprocess
    /data/input /data/output
    --points_topic "$points_topic"
    --image_topic "$image_topic"
    --camera_info_topic "$camera_info_topic"
    --intensity_channel "${VLCAL_INTENSITY_CHANNEL:-auto}"
  )

  if [[ "$gui" == "1" ]]; then
    build_gui_args
    docker_args+=("${GUI_ARGS[@]}")
    program_args+=(--visualize)
  fi

  run_with_output "$output" "${docker_args[@]}" "$IMAGE" "${program_args[@]}" "$@"

  [[ -s "$output/calib.json" ]] || die "preprocessing did not create calib.json; inspect the log above"
  compgen -G "$output/*.ply" >/dev/null || die "preprocessing did not create any PLY clouds"
  compgen -G "$output/*.png" >/dev/null || die "preprocessing did not create any PNG images"
  printf 'Preprocessed data: %s\n' "$output"
}

command_output_program() {
  local program="$1"
  local gui="$2"
  shift 2
  [[ $# -ge 1 ]] || die "$program needs OUTPUT_DIR"
  require_docker
  require_image

  local output
  output="$(existing_dir "$1")"
  shift
  [[ -s "$output/calib.json" ]] || die "missing $output/calib.json; run preprocess first"

  local docker_args=(--rm --mount "type=bind,src=$output,dst=/data/output")
  if [[ "$gui" == "1" ]]; then
    build_gui_args
    docker_args+=("${GUI_ARGS[@]}")
  fi

  run_with_output "$output" "${docker_args[@]}" "$IMAGE" \
    ros2 run direct_visual_lidar_calibration "$program" /data/output "$@"
}

main() {
  local command="${1:-}"
  [[ -n "$command" ]] || { usage; exit 1; }
  shift || true

  case "$command" in
    doctor) [[ $# -eq 0 ]] || die "doctor takes no arguments"; command_doctor ;;
    pull) [[ $# -eq 0 ]] || die "pull takes no arguments"; command_pull ;;
    bag-info) command_bag_info "$@" ;;
    preprocess) command_preprocess 0 "$@" ;;
    preprocess-gui) command_preprocess 1 "$@" ;;
    manual) command_output_program initial_guess_manual 1 "$@" ;;
    calibrate) command_output_program calibrate 1 "$@" ;;
    # The program's background mode hides the viewer but still initializes GLFW.
    calibrate-headless) command_output_program calibrate 1 "$@" --background --auto_quit ;;
    viewer) command_output_program viewer 1 "$@" ;;
    -h|--help|help) usage ;;
    *) usage; die "unknown command: $command" ;;
  esac
}

main "$@"
