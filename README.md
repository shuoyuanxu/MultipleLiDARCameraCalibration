Ported from https://github.com/koide3/direct_visual_lidar_calibration

Edpendded to have all sensor calibrations for our Robot
<img width="2512" height="1407" alt="1" src="https://github.com/user-attachments/assets/e84bc8b9-a20a-4d8f-ab76-91c31e08d342" />

Extrinsics needed:

```text

camera_color_optical_frame --T_lidar91_camera--> lidar_91

lidar_91 --T_lidar90_lidar91--> lidar_90

```

Using column vectors, the conventions are:

```text<img width="2512" height="1407" alt="4" src="https://github.com/user-attachments/assets/12bd283f-2351-47bd-82f0-280ea848e5e2" />


p_lidar91 = T_lidar91_camera * p_camera

p_lidar90 = T_lidar90_lidar91 * p_lidar91

p_lidar90 = T_lidar90_lidar91 * T_lidar91_camera * p_camera

```
# Installation Guide
Tested with Ubuntu 22.04 and ROS 2 Humble.
### Dependencies:
```bash
sudo apt update
sudo apt install -y \
python3-colcon-common-extensions python3-numpy python3-opencv \
python3-scipy python3-yaml \
ros-humble-apriltag-ros ros-humble-cv-bridge \
ros-humble-robot-state-publisher ros-humble-rviz2 \
ros-humble-tf2-ros ros-humble-xacro
```
### Download and build camera-LiDAR calibrator
```bash
git clone --recursive \
https://github.com/shuoyuanxu/direct_visual_lidar_calibration-main.git \
cd ~/direct_visual_lidar_calibration-main
./docker/vlcal.sh pull
./docker/vlcal.sh doctor
```
### NVIDIA acceleration (must!)
If `nvidia-smi` works on the host, enable NVIDIA support for Docker with the helper included in the cloned repository:
```bash
sudo ./scripts/install_nvidia_container_toolkit.sh
./docker/vlcal.sh doctor
```

# Existing results   

- `calibration_results/extrinsic_lidar91_to_lidar90.yaml` — LiDAR 91 into LiDAR 90.

- `calibration_vision_lidar91/calib.json` — camera optical frame into LiDAR 91.  

# Part 1 — LiDAR-to-LiDAR result

<img width="2512" height="1407" alt="1" src="https://github.com/user-attachments/assets/93319e86-3c43-43ad-9b16-89a6223eec0c" />
<img width="2512" height="1407" alt="3" src="https://github.com/user-attachments/assets/ada5dfe1-45b8-4310-8b8c-a73a8c303d5a" />

## 1.  Data prep
Mid360 LiDAR topics:
```text
/lidar/lidar_90 /livox/imu_90
/lidar/lidar_91 /livox/imu_91
```

Start with a static scene for 2mins, and then run a FastLIO capable mapping run in a non-challenging enviroment.
## 2. FastLIO for map generation
Our modified FastLIO should generate the following result:
```text
final_map.pcd
keyframes_lidar/*.pcd
keyframe_poses_lidar.csv
keyframe_poses_optimized_lidar.csv
trajectory_scan_poses_lidar.csv
```
We are going to use all 3 (keyframe, trajectory, and map) sourcs to cross validate the calibration quality.
## 3. Run all three LiDAR calibration methods
The script needs Python 3, NumPy, and SciPy. Run:
```bash
python3 calibrate_lidar_extrinsics.py \
--lidar90-dir lidar_90/attempt_001 \
--lidar91-dir lidar_91/attempt_001 \
--trajectory-pairs calibration_ready/trajectory_pairs_lidar.csv \
--output-dir calibration_results \
--map-voxel-size 0.25 \
--scan-tolerance-ms 5 \
--max-scan-pairs 60
```
All arguments above are the defaults, so this is equivalent:
```bash
python3 calibrate_lidar_extrinsics.py
```
It does calibration in 3 ways: 
1. Trajectory-to-trajectory hand-eye calibration (`AX = XB`).
2. Accumulated-map point-to-plane ICP
3. Joint point-to-plane ICP over synchronized keyframe scans. 

**The synchronized scan result is the deployed result, everything else is for crossing checking**

Calibration result:
```text
calibration_results/extrinsic_lidar91_to_lidar90.yaml
calibration_results/calibration_report.json
calibration_results/map_overlay_downsampled.ply
``` 
# Part 2 — Camera-to-LiDAR

<img width="2512" height="1407" alt="4" src="https://github.com/user-attachments/assets/66824614-864e-4636-94af-ca5683ac7542" />
<img width="2512" height="1407" alt="1" src="https://github.com/user-attachments/assets/508bdbe1-4632-40aa-8a82-e4dfe73d0c0e" />

## 1. Data prep
At each pose: stop the vehicle, wait for vibration to settle, then record for 10-15 seconds. **Never drive during a bag, drive only between bags**. Ensure that the tags or corners are visble in lidar, our experience is put Tags on transparent glass. Required topics:
```bash
/camera/color/image_raw/compressed \
/camera/color/camera_info \
/lidar/lidar_90 \
/lidar/lidar_91
```
## 2. Preprocess - converting bag into required format
```bash
cd /home/shuoyuan/tools/direct_visual_lidar_calibration-main
source /opt/ros/humble/setup.bash
source /home/shuoyuan/ros2_anto_ws/install/setup.bash

./docker/vlcal.sh preprocess \
/home/shuoyuan/vlcal_data/bags \
/home/shuoyuan/vlcal_data/preprocessed_lidar91 \
/lidar/lidar_91 \
/camera/color/image_raw/compressed \
/camera/color/camera_info
```
## 3. Make and save the manual initial alignment
```bash
./docker/vlcal.sh manual /home/shuoyuan/vlcal_data/preprocessed_lidar91
```
In the GUI:

1. Pick the same feature in the point cloud and image.
2. Click **Add picked points**.
3. Repeat for 6–10 features spread over the field of view and depth range.
4. Click **Estimate**.
5. Inspect the blended image and then click **Save**.

Back up the good manual result before optional refinement, **sometimes optimisation makes it worse!**:
```bash
cp /home/shuoyuan/vlcal_data/preprocessed_lidar91/calib.json \
/home/shuoyuan/vlcal_data/preprocessed_lidar91/calib.manual.json
```
## 4. Refine, compare, and accept or reject
```bash
./docker/vlcal.sh calibrate-headless \
/home/shuoyuan/vlcal_data/preprocessed_lidar91
./docker/vlcal.sh viewer \
/home/shuoyuan/vlcal_data/preprocessed_lidar91
```
The JSON array order is:
```text
results.T_lidar_camera = [x, y, z, qx, qy, qz, qw]
```

```text
p_lidar91 = T_lidar91_camera * p_camera
```

**Accept only after visual checks:

```text
sudo ./docker/vlcal.sh calibrate-headless /home/shuoyuan/vlcal_data/preprocessed_lidar91
```

once accepted, copy it into LL calibration project folder: `calibration_vision_lidar91/calib.json` 
# Part 3 — Generating final URDF
Run:
```bash
./generate_three_sensor_urdf.py
```
It reads these source results directly:
```text
calibration_results/extrinsic_lidar91_to_lidar90.yaml
calibration_vision_lidar91/calib.json
```
and generates:
```text
three_sensor_overlay/urdf/u701_three_sensor_extrinsics.urdf
three_sensor_overlay/urdf/u701_three_sensor_extrinsics.urdf.xacro
```
# Part 4 — Replay overlay for double checking

```bash
BAG_PATH=/media/shuoyuan/CrucialX9/Antobot/bags/U701_0901_compressed_merged \
./run_three_sensor_overlay.sh
```
Override only when using a different target:
```bash
TAG_SIZE_M=0.0872 ./run_three_sensor_overlay.sh
```
The replay reads both calibration files directly not a URDF
Color and depth intrinsics come from `/camera/color/camera_info` and
`/camera/depth/camera_info` 
The bag does not contain the factory color-to-depth TF, so the script uses a identity TF. 
