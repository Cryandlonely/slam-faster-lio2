# FAST_LIO_RoboSense

ROS2 Humble version of FAST_LIO with support for RoboSense Airy LiDAR.

## Features

- ✅ Based on FAST_LIO ROS2 Humble version
- ✅ Support for RoboSense Airy LiDAR (lidar_type: 5)

## Installation

1. Initialize git submodule:

```bash
git clone https://github.com/MichaelSun1001/FAST_LIO_Airy_ROS2 --recursive
```

2. Build:

```bash
colcon build --parallel-workers 20 --symlink-install
```

## Usage

### Launch RoboSense Airy LiDAR Mapping

```bash
source install/setup.bash
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py
```

### Save Map

```bash
source install/setup.bash
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py map_file_path:=/home/sax/map.pcd

# In another terminal
source install/setup.bash
ros2 service call /map_save std_srvs/srv/Trigger
```

### Parameter Configuration

Configuration file is located at `config/robosenseAiry.yaml`. Main parameters:

- `preprocess.lidar_type: 5` - RoboSense Airy LiDAR type
- `preprocess.scan_line: 96` - Number of scan lines
- `common.lid_topic: "/rslidar_points"` - Point cloud topic
- `common.imu_topic: "/rslidar_imu_data"` - IMU topic

## Troubleshooting

### Mapping "flies away" at startup with some bags

If one bag works but another "flies away" immediately, first compare `/rslidar_points` fields.

- Expected (this repo): `x, y, z, intensity, ring, timestamp` (`point_step = 26`)
- Some bags use: `x, y, z, intensity, tag, ring, timestamp` (`point_step = 27`)

The extra `tag` changes field offsets and may break `ring/timestamp` parsing, causing undistortion errors and early divergence.

Also check that per-point `timestamp` units/semantics are the same across bags. If startup motion is large, try `ros2 bag play <bag> --start-offset 2.0`.
