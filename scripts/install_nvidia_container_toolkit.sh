#!/usr/bin/env bash
# Configure Docker to expose the host NVIDIA driver to containers.
# Run with: sudo ./scripts/install_nvidia_container_toolkit.sh
set -Eeuo pipefail

if [[ ${EUID} -ne 0 ]]; then
  printf 'Run this installer with sudo or pkexec.\n' >&2
  exit 1
fi

readonly KEYRING=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
readonly REPO_FILE=/etc/apt/sources.list.d/nvidia-container-toolkit.list
readonly DOCKER_CONFIG=/etc/docker/daemon.json
readonly BACKUP_SUFFIX=".pre-vlcal-$(date +%Y%m%d%H%M%S)"

backup_if_present() {
  local path="$1"
  if [[ -e "$path" ]]; then
    cp -a -- "$path" "${path}${BACKUP_SUFFIX}"
    printf 'Backed up %s to %s\n' "$path" "${path}${BACKUP_SUFFIX}"
  fi
}

key_tmp="$(mktemp)"
list_tmp="$(mktemp)"
trap 'rm -f -- "$key_tmp" "$list_tmp"' EXIT

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends ca-certificates curl gnupg2

backup_if_present "$KEYRING"
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | gpg --dearmor --yes --output "$key_tmp"
install -Dm644 -- "$key_tmp" "$KEYRING"

backup_if_present "$REPO_FILE"
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  > "$list_tmp"
install -Dm644 -- "$list_tmp" "$REPO_FILE"

apt-get update
apt-get install -y nvidia-container-toolkit

# nvidia-ctk merges the NVIDIA runtime configuration into daemon.json.
backup_if_present "$DOCKER_CONFIG"
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker
systemctl is-active --quiet docker

printf '\nNVIDIA Container Toolkit is installed and Docker was restarted.\n'
printf 'Next, run the GPU validation command shown by the calibration setup.\n'
