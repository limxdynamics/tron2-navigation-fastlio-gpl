<p align="center">
  <img src="doc/banner.png" alt="A floor plan that FAST_LIO built from one LiDAR drive, with the robot's current position marked on it" width="100%">
</p>

<p align="center">
  <b>English</b> | <a href="README.zh-CN.md">简体中文</a>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-GPL--2.0--only-3d7eff" alt="License: GPL-2.0-only"></a>
  <img src="https://img.shields.io/badge/ROS%202-Humble-3d7eff" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu%2022.04-aarch64-3d7eff" alt="Ubuntu 22.04 aarch64">
  <a href="https://github.com/limxdynamics/tron2-navigation-fastlio-gpl/tags"><img src="https://img.shields.io/github/v/tag/limxdynamics/tron2-navigation-fastlio-gpl?color=3d7eff" alt="Latest tag"></a>
  <a href="https://github.com/hku-mars/FAST_LIO"><img src="https://img.shields.io/badge/upstream-FAST--LIO-5a6b85" alt="Upstream: FAST-LIO"></a>
</p>

# FAST-LIO: map a floor, then find yourself on it

`fast_lio` is a LiDAR-inertial odometry node. Drive a robot through a building
once and it accumulates every sweep into a point-cloud map, saved as a single
`.pcd` file. `fast_lio_localization` loads that file and, on every later
start-up, matches the live scan against it to report where the robot stands.

Neither node needs GPS, fiducial markers, or a pre-surveyed floor plan, and
neither needs the navigation stack to be running.

> This repository publishes the complete source used for the two GPL-2.0-only
> FAST-LIO runtime units required by the LimX ROS 2 navigation integration:
> `FAST_LIO` (ROS 2 mapping/LIO derivative) and `FAST_LIO_LOCALIZATION2` (ROS 2
> map-localization derivative). Both directories include their embedded
> `ikd-Tree` source, package build files, GPLv2 license text, upstream
> attribution, baseline commit, and local modification notice. No license in
> this repository is changed to Apache or BSD.

## Highlights

- **A map from one drive, a pose from one launch.** Each sweep is matched
  against the map built so far, so the accumulated point cloud *is* the map —
  there is no separate mapping pass to schedule.
- **100 m map, 15 m registration.** Full-range points stay in the exported map
  while scan-to-map matching only uses the near field, so a long corridor does
  not drag the EKF onto sparse far returns.
- **Both PCDs commit or neither does.** When a PCT source map is configured,
  `/map_save` writes both files to temporary paths and swaps them in together;
  a failure leaves the previous pair intact instead of pairing a new map with
  an old one.
- **Parameters fail closed.** Contradictory range settings stop the node during
  start-up with the reason, rather than silently mapping with the wrong filter.
- **No Livox driver required.** `livox_ros_driver2` is optional at configure
  time; without it the packages build PointCloud2-only LiDAR support, which is
  what the RoboSense profile uses. Upstream FAST-LIO requires the driver.

## What is in this repository

| Path | What it does | Language |
| --- | --- | --- |
| `FAST_LIO/` | Mapping/LIO package `fast_lio`, RoboSense profile, launch, RViz layout | C++, Python |
| `FAST_LIO/src/laserMapping.cpp` | EKF scan-to-map, publishes odometry and clouds, serves `/map_save` | C++ |
| `FAST_LIO/src/preprocess.cpp` | Undistortion and per-sensor point parsing | C++ |
| `FAST_LIO/launch/mapping.launch.py` | Mapping launch arguments below | Python |
| `FAST_LIO/config/rs_fairy.yaml` | The supplied RSFAIRY profile | YAML |
| `FAST_LIO_LOCALIZATION2/` | Localization package `fast_lio_localization` | C++, Python |
| `FAST_LIO_LOCALIZATION2/fast_lio_localization/global_localization.py` | Open3D ICP against the saved map, gated by a fitness threshold | Python |
| `FAST_LIO_LOCALIZATION2/fast_lio_localization/transform_fusion.py` | Publishes `map → camera_init` and `map → odom`, then the pose | Python |
| `FAST_LIO_LOCALIZATION2/fast_lio_localization/publish_initial_pose.py` | Publishes `/initialpose` from the command line | Python |
| `FAST_LIO*/include/ikd-Tree/` | Incremental kd-tree used by the EKF update | C++ |
| `SOURCE_INVENTORY.sha256`, `audit_source.sh` | Per-file hash lock and its checker | Bash |

## Requirements

ROS 2 Humble on Ubuntu 22.04, plus the dependencies named in both package
manifests. The Python ones are not pulled in by every image:

```bash
sudo apt install ros-humble-pcl-ros python3-transforms3d python3-open3d
```

`pcl_ros` provides the `pcd_to_pointcloud` node that republishes the saved map,
`open3d` does the registration, and `transforms3d` supplies the quaternion
conversions. Keep the Ubuntu NumPy ahead of anything installed under
`/usr/local`, so it stays ABI-compatible with the packaged Open3D.

A RoboSense RSFAIRY needs its own separately licensed driver built and sourced
before runtime. The profile expects the standard ROS 2 interfaces
`/rslidar_points` (`sensor_msgs/PointCloud2`) and `/rslidar_imu_data`
(`sensor_msgs/Imu`); no bridge node is involved.

The robot must stand still during IMU initialisation. `rs_fairy.yaml` sets
`mapping.gravity_align_world: true`, which keeps world Z vertical even if the
body starts slightly pitched, and that alignment is only correct if the IMU is
quiet while it happens.

## Build

On Ubuntu 22.04 aarch64, source ROS and build into an independent GPL install
prefix:

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --base-paths FAST_LIO FAST_LIO_LOCALIZATION2 \
  --packages-select fast_lio fast_lio_localization \
  --build-base build \
  --install-base install \
  --merge-install \
  --symlink-install \
  --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The navigation integration builds this repository in a separate prefix from its
own code; the two are joined at runtime over ROS 2 topics and TF, not at build
time.

## Run the mapping unit

```bash
ros2 launch fast_lio mapping.launch.py \
  map_file_path:=$HOME/maps/rs_fairy_map.pcd
```

Drive or push the robot through the area once, then save the map:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

`/map_save` writes `map_file_path`, and `pct_map_file_path` as well when one is
configured, then answers with the paths it wrote in `message`. Saving requires
`pcd_save.pcd_save_en: true` in the profile, which `rs_fairy.yaml` sets; with it
turned off the service answers `Map save disabled.`

### Mapping launch arguments

| Argument | Default | Role |
| --- | --- | --- |
| `config_file` | `rs_fairy.yaml` | Profile under `FAST_LIO/config/` |
| `config_path` | installed `config/` directory | Where to look for the profile |
| `map_file_path` | `rs_fairy_map.pcd` | Localization map written by `/map_save` |
| `pct_map_file_path` | empty | Optional near-field PCT source map |
| `pct_max_range` | `0.0` | Range kept in the PCT source map; `0` disables it |
| `registration_max_range` | `15.0` | Range used by scan-to-map registration |
| `rviz` | `true` | Start RViz 2 with `rviz/fastlio.rviz` |

The launch sets `map_file_path` explicitly, so the argument overrides the value
in the YAML profile rather than the other way round.

## Run the localization unit

Localization needs a map produced by the mapping unit, and it needs to be told
roughly where the robot starts:

```bash
ros2 launch fast_lio_localization localization.launch.py \
  map:=$HOME/maps/rs_fairy_map.pcd
```

Then give it the initial pose, either with RViz 2's **2D Pose Estimate** tool, or
from the command line (arguments are `x y z yaw pitch roll`, in radians):

```bash
ros2 run fast_lio_localization publish_initial_pose 0.0 0.0 0.0 0.0 0.0 0.0
```

`publish_initial_pose` publishes `/initialpose` three times and exits. It reports
an error if no subscriber appears within five seconds, which in practice means
localization is not running yet.

Until ICP accepts a match the node stays quiet: `publish_before_localization`
defaults to `false`, so `/pose_stamped` carries nothing and the corrected cloud
is not published. If something downstream needs output before the first match,
set it to `true` and accept an identity-aligned pose until then.

### Localization launch arguments

| Argument | Default | Role |
| --- | --- | --- |
| `map` | empty, required | Path to the `.pcd` map |
| `config_file` | `rs_fairy.yaml` | LIO profile used by the localization node |
| `pcd_map_topic` | `/map` | The saved map republished as `PointCloud2` |
| `frame_id` | `map` | Fixed navigation frame |
| `pose_topic` | `/pose_stamped` | Pose output consumed by the navigation stack |
| `corrected_cloud_topic` | `/corrected_current_pcd` | Live scan transformed into `map` |
| `base_yaw_offset_deg` | `60.0` | Fixed yaw from the LiDAR/IMU pose to the robot base |
| `z_offset` | `0.0` | Height offset applied to the published pose |
| `publish_before_localization` | `false` | Publish an identity-aligned pose before ICP succeeds |
| `rviz` | `true` | Start RViz 2 with `rviz/fastlio_localization.rviz` |

`base_yaw_offset_deg` is the one argument that depends on how the LiDAR is bolted
to the robot. The `60.0` default comes from the established TRON2 mounting; a
different mount needs its own value, or the pose will be rotated about Z by a
constant.

The ICP parameters themselves are set in the launch file rather than exposed as
launch arguments: `map_voxel_size` `0.4`, `scan_voxel_size` `0.1`,
`localization_threshold` `0.8` on the fitness score, and `freq_localization`
`0.5` Hz. Change them by editing `localization.launch.py`.

Both launch files start their own LIO node and subscribe to the same LiDAR
topics, so run one unit at a time: mapping to produce a map, localization to use
it.

## Topics, services and frames

| Direction | Name | Type | Notes |
| --- | --- | --- | --- |
| in | `/rslidar_points` | `sensor_msgs/PointCloud2` | `common.lid_topic`, from the RoboSense driver |
| in | `/rslidar_imu_data` | `sensor_msgs/Imu` | `common.imu_topic`, same driver |
| in | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Localization only |
| out | `/cloud_registered` | `sensor_msgs/PointCloud2` | Scan in the map frame |
| out | `/cloud_registered_body` | `sensor_msgs/PointCloud2` | Scan in the body frame |
| out | `/Laser_map` | `sensor_msgs/PointCloud2` | Accumulated map, mapping only |
| out | `/Odometry` | `nav_msgs/Odometry` | LIO odometry |
| out | `/path` | `nav_msgs/Path` | Trajectory |
| out | `/map_to_odom` | `nav_msgs/Odometry` | ICP correction, localization only |
| out | `/pose_stamped` | `geometry_msgs/PoseStamped` | Localization output |
| out | `/localization` | `nav_msgs/Odometry` | Localization output as odometry |
| out | `/corrected_current_pcd` | `sensor_msgs/PointCloud2` | Live scan placed in `map` |
| service | `/map_save` | `std_srvs/srv/Trigger` | Writes the map PCD files |

Frames: the mapping node publishes its trajectory and pose in `camera_init`; the
localization node publishes `map → camera_init` plus `map → odom` and reports
the robot base pose in `map`.

## v1.1.0 mapping contract

The mapping derivative can keep three range policies independent:

- `preprocess.max_range`: full points retained for map export;
- `mapping.registration_max_range`: near-field points used by EKF scan-to-map
  registration and the incremental ikd-tree;
- `pcd_save.pct_max_range`: near-field PCD exported for PCT tomogram building.

The supplied RoboSense profile keeps a 100 m full localization map while using
15 m for registration and PCT input. Tune these values for the actual sensor
and environment.

These are not free-form numbers: the node validates them during start-up and
refuses to run on an inconsistent set.

- `registration_max_range` must be finite and non-negative, must exceed
  `preprocess.blind` (`0.5`), and must not exceed `preprocess.max_range`.
- `pct_map_file_path` and `pcd_save.pct_max_range` must be set together.
- A PCT source map also requires `pcd_save.use_all_frames: true`,
  `pcd_save.interval <= 0`, and a path different from `map_file_path`.

When both `map_file_path` and `pct_map_file_path` are configured, `/map_save`
prepares both PCDs first and commits them as one recoverable file-set
transaction: the previous files are backed up, the new ones renamed into place,
and any failure restores the backup. A failure cannot silently leave one new map
paired with one old map. The companion integration repository writes and
verifies the mapping session manifest that binds these files to the
`camera_init` frame. Do not combine maps produced by different mapping sessions.

## Source integrity

`SOURCE_INVENTORY.sha256` locks every distributed file except itself. After
cloning, verify the tree:

```bash
./audit_source.sh
```

The script needs `file`, `git` and `sha256sum`. It checks that the inventory
matches the files on disk and that the file set has gained or lost nothing, that
both `package.xml` files declare `GPL-2.0-only`, that each `NOTICE` records its
baseline commit and modification scope, and that the tree carries no generated
objects, maps, recordings, private addresses, machine paths or files over
25 MiB. It prints `GPL_SOURCE_AUDIT=PASS` on success.

Because the inventory is byte-exact, adding a file means recomputing it in the
same commit; a documentation-only change touches a few lines and leaves the
rest byte-identical.

## Integration boundary

The full navigation system checks out this GPL repository separately from the
permissive integration repository and builds it into a separate prefix. Runtime
communication with RSLIDAR, PCT, and SCAN uses ROS 2/DDS messages and TF. This
engineering boundary does not alter GPL obligations or determine a legal
aggregation/derivative-work question.

Companion repositories:

- <https://github.com/limxdynamics/tron2-navigation-ros2>
- <https://github.com/limxdynamics/tron2-navigation-pct-gpl>

## This source snapshot

This is the complete source snapshot expected by the companion LimX ROS 2
navigation integration. The public upstream commits document its provenance,
but they are not drop-in replacements for these ROS 2 Humble and RoboSense
derivatives.

| Directory | Upstream | Baseline commit |
| --- | --- | --- |
| `FAST_LIO` | <https://github.com/Ericsii/FAST_LIO.git> | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| `FAST_LIO_LOCALIZATION2` | <https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2.git> | `f04974907c8da976dd0495b272d18ac4c534d41f` |

The local changes are the ROS 2 Humble/aarch64 and RoboSense RSFAIRY
adaptations: mapping and map save, gravity alignment, initial-pose
localization, transform fusion, base-yaw handling, shutdown safety, and the
v1.1.0 range split described above. See [RELEASE_INFO.md](RELEASE_INFO.md),
[FAST_LIO/NOTICE](FAST_LIO/NOTICE), and
[FAST_LIO_LOCALIZATION2/NOTICE](FAST_LIO_LOCALIZATION2/NOTICE) for exact
provenance and modification scope.

Site calibration, maps, robot credentials, endpoint values, generated binaries,
and install prefixes are intentionally absent: they are runtime data, not
corresponding source.

The `v1.1.0` tag marks the source release the navigation integration fetches;
`master` carries later documentation-only commits.

## License

The derivative programs are distributed under GNU GPL version 2 only. Read the
root [LICENSE](LICENSE), each component's `LICENSE`, and both `NOTICE` files.
The software is provided without warranty.
