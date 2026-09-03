# Calibrate an existing ROS2 bag

This guide does **not** record sensors. You already have a ROS2 bag; this
project turns that bag into a camera–LiDAR calibration.

## 1. Start Docker

Run this once in a terminal:

```bash
newgrp docker
```

Then run:

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
./docker/vlcal.sh doctor
```

If you see `cannot access the Docker daemon`, run `newgrp docker` again, then
repeat the command in the new terminal shell. Do not use `sudo docker`.

## GUI / visualization through Docker

Run viewer and manual-alignment commands from the logged-in desktop terminal,
not through SSH. `docker/vlcal.sh` automatically passes the desktop display,
Xauthority cookie, X11 socket, and available GPU into the container.

If it says `DISPLAY is unset` or cannot read Xauthority, run this once in that
terminal, then retry the viewer command:

```bash
export DISPLAY=:0
export XAUTHORITY="$(find "/run/user/$(id -u)" -maxdepth 1 -type f -name '.mutter-Xwaylandauth.*' -print -quit)"
```

Check the GUI connection:

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
./docker/vlcal.sh doctor
```

## 2. Preprocess the supplied bag

This converts the bag into a point cloud, camera image, and `calib.json`.
Run it once for LiDAR 91:

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main

./docker/vlcal.sh preprocess \
  /home/shuoyuan/snap/code/258/.local/share/Trash/files/rosbag-stage.wQuQOZ \
  /home/shuoyuan/tools/direct_visual_lidar_calibration-main/calibration_trial_lidar91 \
  /lidar/lidar_91 \
  /camera/color/image_raw/compressed \
  /camera/color/camera_info
```

Do not run this again if `calibration_trial_lidar91` already exists. It has
already been run for the current bag.

To use LiDAR 90 instead, change both `lidar91` to `lidar90` and
`/lidar/lidar_91` to `/lidar/lidar_90`.

## 3. Visualize the current calibration

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
./docker/vlcal.sh viewer "$PWD/calibration_trial_lidar91"
```

In the viewer, open **data selection** -> **Transformation**. Choose:

- `INIT_GUESS (MANUAL)` for your manual alignment.
- `CALIBRATION_RESULT` for the current saved final calibration.

For the current trial, both are the manual alignment because it was chosen over
the refinement result.

## 4. Redo the manual alignment

Only do this when you want to change your alignment:

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
./docker/vlcal.sh manual "$PWD/calibration_trial_lidar91"
```

Pick matching cloud/image points, click **Add picked points** 6-10 times,
then click **Estimate** and **Save**.

## 5. Run refinement

Only run this when you want the optimizer to replace `T_lidar_camera` with its
own result:

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
./docker/vlcal.sh calibrate-headless "$PWD/calibration_trial_lidar91"
```

The calibration is saved here:

```text
/home/shuoyuan/tools/direct_visual_lidar_calibration-main/calibration_trial_lidar91/calib.json
```

## Complete first-time installation (Ubuntu 22.04 + NVIDIA)

Use this appendix only on a new PC. Skip a section that already works. These
commands install Docker, allow your user to run it, enable NVIDIA containers,
pull the calibration image, and verify that GUI windows can open through
Docker. This repository already contains `docker/vlcal.sh`; do not set
`VLCAL_REPOSITORY_URL` and do not build an image locally.

### A. Install Docker Engine

Run these commands once on a new **Ubuntu 22.04 amd64** PC:

```bash
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<'EOF'
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: jammy
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
```

Allow your normal user to run Docker, then open the new shell that `newgrp`
starts:

```bash
sudo usermod -aG docker "$USER"
newgrp docker
docker run --rm hello-world
```

The `docker` group is effectively administrator access to the machine. Use it
only for trusted users and images.

### B. Enable NVIDIA in Docker

First confirm that the host NVIDIA driver works:

```bash
nvidia-smi
```

If this fails, install a suitable NVIDIA driver and reboot before continuing.
Then run the installer included in this repository:

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
sudo ./scripts/install_nvidia_container_toolkit.sh
```

Verify that a container can see the GPU:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

### C. Pull and test the calibration image

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
./docker/vlcal.sh pull
./docker/vlcal.sh doctor
```

The final command must report `Docker daemon access: OK` and `NVIDIA container
runtime: available`.

### D. Enable visualization through Docker

Run GUI commands from the logged-in desktop terminal, not SSH. The wrapper
automatically mounts the X11/Xwayland display socket and authentication cookie
into the container, and passes the available GPU.

```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
echo "$DISPLAY"
echo "$XAUTHORITY"
./docker/vlcal.sh viewer "$PWD/calibration_trial_lidar91"
```

If the wrapper reports missing `DISPLAY` or Xauthority, run this in the same
desktop terminal and retry:

```bash
export DISPLAY=:0
export XAUTHORITY="$(find "/run/user/$(id -u)" -maxdepth 1 -type f -name '.mutter-Xwaylandauth.*' -print -quit)"
```

Use the same setup for `manual`, `viewer`, and GUI preprocessing. `calibrate-headless`
hides the window but still needs this desktop display setup on this build.

For the upstream installation details, see the [Docker Engine Ubuntu guide](https://docs.docker.com/engine/install/ubuntu/) and [NVIDIA Container Toolkit guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
