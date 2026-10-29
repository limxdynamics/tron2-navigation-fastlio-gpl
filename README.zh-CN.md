<p align="center">
  <img src="doc/banner.png" alt="FAST_LIO 用一次激光雷达巡走建出的平面图，并标出机器人当前所在的点" width="100%">
</p>

<p align="center">
  <a href="README.md">English</a> | <b>简体中文</b>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-GPL--2.0--only-3d7eff" alt="License: GPL-2.0-only"></a>
  <img src="https://img.shields.io/badge/ROS%202-Humble-3d7eff" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu%2022.04-aarch64-3d7eff" alt="Ubuntu 22.04 aarch64">
  <a href="https://github.com/limxdynamics/tron2-navigation-fastlio-gpl/tags"><img src="https://img.shields.io/github/v/tag/limxdynamics/tron2-navigation-fastlio-gpl?color=3d7eff" alt="Latest tag"></a>
  <a href="https://github.com/hku-mars/FAST_LIO"><img src="https://img.shields.io/badge/upstream-FAST--LIO-5a6b85" alt="Upstream: FAST-LIO"></a>
</p>

# FAST-LIO：先建一张楼层图，再在图里找到自己

`fast_lio` 负责建图：带着机器人把一栋楼走一遍，它把每一帧扫描累积成一个 `.pcd` 文件。`fast_lio_localization` 加载这份地图，之后每次开机都把实时扫描与地图配准，报出机器人站在图里的哪一点。不需要 GPS、不需要贴标记、不需要事先测好的平面图，两个节点也都不需要导航栈同时运行。

> 本仓库按 GPL-2.0-only 公开 LimX ROS 2 导航集成所依赖的两个 FAST-LIO 运行时单元的完整源码：`FAST_LIO`（建图/LIO）与 `FAST_LIO_LOCALIZATION2`（地图定位），各自带内嵌的 `ikd-Tree`、GPLv2 全文、上游署名、基线提交与修改声明。本仓库中的任何许可证都没有被改成 Apache 或 BSD。

## 仓库组成

| 路径 | 作用 |
| --- | --- |
| `FAST_LIO/` | 建图包 `fast_lio`：EKF scan-to-map、`/map_save`、RoboSense 配置、launch、RViz |
| `FAST_LIO_LOCALIZATION2/` | 定位包 `fast_lio_localization`：对已保存地图做 Open3D ICP、变换融合、初始位姿工具 |
| `FAST_LIO*/include/ikd-Tree/` | EKF 更新用的增量 kd-tree |
| `SOURCE_INVENTORY.sha256`、`audit_source.sh` | 逐文件哈希清单与校验脚本 |

## 环境要求

Ubuntu 22.04 上的 ROS 2 Humble，加上两个 manifest 里声明的依赖。其中 Python 的几项不是每个镜像都带：

```bash
sudo apt install ros-humble-pcl-ros python3-transforms3d python3-open3d
```

RoboSense RSFAIRY 需要单独构建并 source 它自己的驱动（独立许可证）；配置期望它提供的 `/rslidar_points`（`PointCloud2`）与 `/rslidar_imu_data`（`Imu`），中间没有桥接节点。`livox_ros_driver2` 在 configure 阶段可选——没有它，包会构建成只支持 PointCloud2 的形态，也就是本配置用的那一路；上游 FAST-LIO 是强制要求的。让 Ubuntu 自带的 NumPy 优先于 `/usr/local` 下的版本，才能跟系统打包的 Open3D 保持 ABI 兼容。IMU 初始化期间机器人必须静止：`rs_fairy.yaml` 设了 `mapping.gravity_align_world: true`，而这一步对齐只有在 IMU 静止时才是对的。

## 编译

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

导航集成会把本仓库编译进一个独立前缀；两者在运行期通过 ROS 2 话题与 TF 相连。

## 运行建图部分

```bash
ros2 launch fast_lio mapping.launch.py map_file_path:=$HOME/maps/rs_fairy_map.pcd
```

开着或推着机器人把区域走一遍，然后保存：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

`/map_save` 写 `map_file_path`；配了 `pct_map_file_path` 时一并写它，并在应答里返回实际写入的路径。它要求 `pcd_save.pcd_save_en: true`（`rs_fairy.yaml` 已设好）；否则应答 `Map save disabled.`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `config_file` | `rs_fairy.yaml` | `FAST_LIO/config/` 下的配置 |
| `map_file_path` | `rs_fairy_map.pcd` | `/map_save` 写出的地图；覆盖 YAML 里的值 |
| `pct_map_file_path` | 空 | 可选的近场 PCT 源地图 |
| `pct_max_range` | `0.0` | PCT 源地图保留的量程；`0` 表示关闭 |
| `registration_max_range` | `15.0` | scan-to-map 配准使用的量程 |
| `rviz` | `true` | 用 `rviz/fastlio.rviz` 启动 RViz 2 |

## 运行定位部分

```bash
ros2 launch fast_lio_localization localization.launch.py map:=$HOME/maps/rs_fairy_map.pcd
```

然后给起始位姿，用 RViz 2 的 **2D Pose Estimate**，或用下面的工具（参数顺序 `x y z yaw pitch roll`，单位弧度）：

```bash
ros2 run fast_lio_localization publish_initial_pose 0.0 0.0 0.0 0.0 0.0 0.0
```

它连发三次 `/initialpose` 后退出；五秒内没有订阅者则以非零状态退出。在 ICP 拿到可接受的结果之前节点保持沉默，因为 `publish_before_localization` 默认 `false`：要么把它设成 `true` 拿一个按单位矩阵对齐的位姿，要么在首次匹配前不要期待输出。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `map` | 空，必填 | `.pcd` 地图路径 |
| `config_file` | `rs_fairy.yaml` | 定位节点用的 LIO 配置 |
| `frame_id` | `map` | 固定的导航坐标系 |
| `pose_topic` | `/pose_stamped` | 给导航栈的位姿输出 |
| `base_yaw_offset_deg` | `60.0` | 从雷达/IMU 位姿到机器人底盘的固定偏航 |
| `z_offset` | `0.0` | 输出位姿的高度偏移 |
| `publish_before_localization` | `false` | ICP 成功前是否发布单位矩阵对齐的位姿 |
| `rviz` | `true` | 用 `rviz/fastlio_localization.rviz` 启动 RViz 2 |

`base_yaw_offset_deg` 取决于雷达怎么固定在车上：`60.0` 是既有 TRON2 安装的取值，换一种安装就得给对应的值，否则位姿会绕 Z 轴差一个常量。ICP 的参数不是启动参数——`map_voxel_size` `0.4`、`scan_voxel_size` `0.1`、`localization_threshold` `0.8`、`freq_localization` `0.5` Hz——要改就编辑 `localization.launch.py`。

两个 launch 都会各自起一个 LIO 节点并订阅同一组雷达话题，所以一次只跑一个。

## 话题与坐标系

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 入 | `/rslidar_points` | `sensor_msgs/PointCloud2` | `common.lid_topic` |
| 入 | `/rslidar_imu_data` | `sensor_msgs/Imu` | `common.imu_topic` |
| 入 | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 仅定位 |
| 出 | `/cloud_registered` | `sensor_msgs/PointCloud2` | 地图坐标系下的扫描 |
| 出 | `/Laser_map` | `sensor_msgs/PointCloud2` | 累积地图，仅建图 |
| 出 | `/Odometry` | `nav_msgs/Odometry` | LIO 里程计 |
| 出 | `/path` | `nav_msgs/Path` | 轨迹 |
| 出 | `/pose_stamped` | `geometry_msgs/PoseStamped` | 定位输出 |
| 出 | `/map_to_odom` | `nav_msgs/Odometry` | ICP 修正量，仅定位 |
| 出 | `/corrected_current_pcd` | `sensor_msgs/PointCloud2` | 放到 `map` 里的实时扫描 |
| 服务 | `/map_save` | `std_srvs/srv/Trigger` | 写出地图 PCD 文件 |

建图在 `camera_init` 下发布；定位发布 `map → camera_init` 与 `map → odom`，并在 `map` 下给出底盘位姿。

## v1.1.0 建图约定

三套彼此独立的量程策略：

- `preprocess.max_range` —— 保留并导出的全量点（`100.0`）；
- `mapping.registration_max_range` —— EKF 与 ikd-tree 参与匹配的点（`15.0`）；
- `pcd_save.pct_max_range` —— 可选 PCT 源地图保留的点（`0.0`）。

节点在启动时校验，不一致就拒绝运行：`registration_max_range` 必须有限、非负、大于 `preprocess.blind`（`0.5`）且不超过 `preprocess.max_range`；`pct_map_file_path` 与 `pcd_save.pct_max_range` 必须同时设置，使用 PCT 源地图还要求 `pcd_save.use_all_frames: true`、`pcd_save.interval <= 0`，且路径与 `map_file_path` 不同。

同时配置两份地图时，`/map_save` 各写临时文件再整体提交：旧文件先备份，新文件改名落位，任一步失败就恢复备份。一份新地图不可能配上一份旧地图，所以不要混用不同建图会话产出的地图。伴生的集成仓会记录把这些文件绑定到 `camera_init` 坐标系的会话清单。

## 源码完整性

```bash
./audit_source.sh
```

`SOURCE_INVENTORY.sha256` 锁定除它自己以外的每个分发文件。脚本（需要 `file`、`git`、`sha256sum`）校验哈希与文件集、两个 `package.xml` 是否都声明 `GPL-2.0-only`、每份 `NOTICE` 是否记录了基线与修改范围，以及树里有没有生成物、地图、录制包、私网地址、机器路径或超过 25 MiB 的文件；通过时打印 `GPL_SOURCE_AUDIT=PASS`。新增文件就要在同一个提交里重算清单。

## 集成边界

导航系统把本仓库单独 checkout 并编译进自己的前缀；运行期与 RSLIDAR、PCT、SCAN 的通信走 ROS 2/DDS 消息与 TF。这条工程边界不改变 GPL 义务，也不判断任何关于聚合或衍生作品的法律问题。

- <https://github.com/limxdynamics/tron2-navigation-ros2>
- <https://github.com/limxdynamics/tron2-navigation-pct-gpl>

## 源码快照

| 目录 | 上游 | 基线提交 |
| --- | --- | --- |
| `FAST_LIO` | <https://github.com/Ericsii/FAST_LIO.git> | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| `FAST_LIO_LOCALIZATION2` | <https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2.git> | `f04974907c8da976dd0495b272d18ac4c534d41f` |

本地修改是面向 ROS 2 Humble/aarch64 与 RSFAIRY 的适配：建图与地图保存、重力对齐、初始位姿定位、变换融合、底盘偏航处理、退出安全，以及 v1.1.0 的量程拆分。确切的来源见 [RELEASE_INFO.md](RELEASE_INFO.md)、[FAST_LIO/NOTICE](FAST_LIO/NOTICE) 与 [FAST_LIO_LOCALIZATION2/NOTICE](FAST_LIO_LOCALIZATION2/NOTICE)。现场标定、地图、机器人凭据、端点值、生成的可执行文件和安装前缀是刻意不包含的：它们是运行期数据，不是对应源码。

`v1.1.0` 标签标记导航集成所取的那份源码发布；`master` 上带着之后的纯文档提交。

## 许可证

衍生程序仅以 GNU GPL 第 2 版分发（GPL-2.0-only）。请阅读根目录的 [LICENSE](LICENSE)、各组件目录下的 `LICENSE`，以及两份 `NOTICE`。本软件不提供任何担保。
