# Yangpu LiDAR + IMU RTAB-Map bringup

This package is the first-stage mapping configuration for the Yangpu port bags. It uses:

```text
/lidar_preprocessor/meta_cloud -> rtabmap_odom/icp_odometry
                               -> /rtabmap/odom
                               -> rtabmap_slam/rtabmap
                               -> map -> odom_rtabmap, mapData, cloud_map, rtabmap.db
```

The system is **IMU-assisted ICP odometry plus an RTAB-Map pose graph**. It is not a FAST-LIO-style tightly coupled LIO implementation. No GPS, `/localization/ins`, `global_pose`, custom loop detector, Scan Context, external graph optimizer, or RTAB-Map core changes are included.

## Verified bag interface

The static report is generated at [`docs/yangpu_bag_interface_report.md`](../../docs/yangpu_bag_interface_report.md) by `scripts/inspect_yangpu_bag.sh`.

The verified defaults are:

| Interface | Observed value |
|---|---|
| cloud | `/lidar_preprocessor/meta_cloud`, `sensor_msgs/PointCloud2`, about 10 Hz |
| cloud frame | `base_link` |
| cloud fields | `x y z intensity`; no `time/t/timestamp` and no `ring` |
| IMU | `/ins_driver/imu`, `sensor_msgs/Imu`, about 93.9 Hz |
| IMU frame | `chcnav_msg_parser` |
| IMU orientation | finite, non-zero quaternion; `orientation_covariance[0] != -1` |
| static TF | direct `base_link -> chcnav_msg_parser` is present |
| INS | `/localization/ins`, `nav_msgs/Odometry`, `header.frame_id=map`, `child_frame_id=map_harbor`; not connected |
| clock | no `/clock` recorded; use `rosbag play --clock` |

Because the input cloud has no per-point timing/ring fields and the reference preprocessing already produces a fused cloud in `base_link`, `deskewing=false` is the default.

## Build

The repositories were clean before this package was added:

```text
rtabmap      master        bcdb4b454683efc651a36f044cc85d8f2f5f4ac3
rtabmap_ros  noetic-devel  364b7da750f3be3203863c7b913a69c4f6b0b59e
```

Build standalone RTAB-Map first, using a user prefix:

```bash
source /opt/ros/noetic/setup.bash
export RTABMAP_INSTALL_PREFIX=$HOME/opt/rtabmap-noetic
cd $HOME/dataDisk/Study/rtabMap_ws/src/rtabmap/build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$RTABMAP_INSTALL_PREFIX \
  -DWITH_QT=OFF -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF
make -j8
make install
```

The normal `catkin_make` command was attempted, but this source space contains both catkin packages and the plain-CMake standalone `rtabmap` package. Catkin correctly rejected the non-homogeneous workspace. For this first-stage bringup, build the required ROS packages in isolation and skip the optional GUI package:

```bash
source /opt/ros/noetic/setup.bash
export RTABMAP_INSTALL_PREFIX=$HOME/opt/rtabmap-noetic
export ROS_DEPS_PREFIX=$HOME/opt/ros-deps/opt/ros/noetic
cd $HOME/dataDisk/Study/rtabMap_ws
export ROS_PACKAGE_PATH="$ROS_DEPS_PREFIX/share:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$PWD/install_isolated:$ROS_DEPS_PREFIX:$RTABMAP_INSTALL_PREFIX:/opt/ros/noetic"
catkin_make_isolated -j8 --install --ignore-pkg rtabmap \
  --only-pkg-with-deps rtabmap_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PWD/install_isolated;$ROS_DEPS_PREFIX;$RTABMAP_INSTALL_PREFIX;/opt/ros/noetic"

# Build this independent bringup package after the optional GUI package is skipped.
catkin_make_isolated --install --pkg rtabmap_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PWD/install_isolated;$ROS_DEPS_PREFIX;$RTABMAP_INSTALL_PREFIX;/opt/ros/noetic"
```

`rtabmap_slam` requires the real ROS package `move_base_msgs`. If it is not present in `/opt/ros/noetic`, install `ros-noetic-move-base-msgs` normally, or unpack that official ROS package into the user-only `$HOME/opt/ros-deps` prefix as used above. No message or TF stub is provided.

The current system Qt/VTK installation has missing imported files. The standalone core therefore builds without Qt. `rtabmap_viz` and RViz remain optional at launch time (`rtabmap_viz:=false`, `rviz:=false` by default); use RViz if the ROS GUI packages are available.

At runtime, put the user prefix before `/opt/ros/noetic`:

```bash
export CMAKE_PREFIX_PATH=$HOME/opt/rtabmap-noetic:/opt/ros/noetic
export LD_LIBRARY_PATH=$HOME/opt/rtabmap-noetic/lib:/opt/ros/noetic/lib:$LD_LIBRARY_PATH
```

The final runtime must resolve `librtabmap_core.so.0.23` and `librtabmap_utilite.so.0.23` from `$HOME/opt/rtabmap-noetic/lib`, not from `/usr/local` or a system RTAB-Map install. The ROS package declarations on this branch are `0.21.13`; the linked standalone library is `0.23.9`, which is why the actual link check is part of the build handoff.

## Inspect the bags

```bash
source /opt/ros/noetic/setup.bash
rosrun rtabmap_bringup inspect_yangpu_bag.sh \
  --bag-dir $HOME/dataDisk/hainan/yangpu/qc
```

It writes `rtabmap_ros/docs/yangpu_bag_interface_report.md` and emits PASS/WARN status lines. `inspect_bag.sh` is an alias for the same check.

## Run the first stage

First run a 30-second test in a clean pair of terminals. The output directory must be new, or an explicit delete authorization is required.

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source $HOME/dataDisk/Study/rtabMap_ws/install_isolated/setup.bash
export CMAKE_PREFIX_PATH=$HOME/dataDisk/Study/rtabMap_ws/install_isolated:$HOME/opt/rtabmap-noetic:/opt/ros/noetic
export LD_LIBRARY_PATH=$HOME/opt/rtabmap-noetic/lib:$HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
roslaunch rtabmap_bringup lidar_imu_mapping.launch \
  use_sim_time:=true \
  delete_db_on_start:=true \
  cloud_topic:=/lidar_preprocessor/meta_cloud \
  imu_topic:=/ins_driver/imu \
  output_dir:=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_output
```

Terminal 2:

```bash
cd $HOME/dataDisk/hainan/yangpu/qc
rosbag play 2026-06-22-12-* --clock -u 30
```

For a guarded launch through the helper script:

```bash
RTABMAP_OUTPUT_DIR=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_output_new \
  $HOME/dataDisk/Study/rtabMap_ws/src/rtabmap_ros/rtabmap_bringup/scripts/run_mapping.sh
```

If reusing an existing output directory, set `ALLOW_DB_DELETE=1` only when deleting that database is intentional. The helper will otherwise refuse to start.

After the 30-second test, run:

```bash
scripts/check_runtime.sh
```

For the full requested test, repeat with:

```bash
cd $HOME/dataDisk/hainan/yangpu/qc
rosbag play 2026-06-22-12-* --clock -u 515
```

The runtime checker samples topic rates, odometry/odom_info, both dynamic TF links, node presence, and database growth. The expected database is `rtabmap_output/rtabmap.db`.

## A/B tuning order

Expose only one change at a time:

```text
wait_imu_to_init:=false
point_to_plane:=false
scan_voxel_size:=0.30
icp_max_correspondence_distance:=1.5
icp_correspondence_ratio:=0.05
max_update_rate:=5.0
```

If CPU is high, disable both GUI options first, then increase `scan_voxel_size` or lower `rtabmap_detection_rate`. If ICP is unstable, verify that `meta_cloud` is a single-frame cloud rather than an accumulated global cloud before changing more parameters.

## Deferred GPS/INS stage

`/localization/ins` is intentionally not remapped to `/rtabmap/gps/fix`; it is `nav_msgs/Odometry`, not `sensor_msgs/NavSatFix`. A later stage should convert it to `geometry_msgs/PoseWithCovarianceStamped` only after deciding the ENU/map convention, whether the pose is for `base_link` or `ins`, covariance handling, state gating, separate XY/Z thresholds, and a non-overconstraining update policy.
