# Yangpu LiDAR + IMU RTAB-Map bringup

This package is the first-stage mapping configuration for the Yangpu port bags. It uses:

```text
/lidar_preprocessor/meta_cloud -> rtabmap_odom/icp_odometry
                               -> /rtabmap/odom
                               -> rtabmap_slam/rtabmap
                               -> map -> odom_rtabmap, mapData, cloud_map, rtabmap.db
```

The default `lidar_imu_mapping.launch` path is **IMU-assisted ICP odometry plus
an RTAB-Map pose graph**. It is not a FAST-LIO-style tightly coupled LIO
implementation and it does not enable GPS by default. This package also
contains the optional Mode B FAST-LIO + RTAB-Map + GNSS path documented below.

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

## GPS/INS input note

`/localization/ins` must not be remapped to `/rtabmap/gps/fix`: it is
`nav_msgs/Odometry`, not `sensor_msgs/NavSatFix`, and the bag version has zero
covariance. The verified GPS path starts `gnss_poser`, consumes its
`/localization/gnss_odom`, validates/floors the covariance, and publishes
`geometry_msgs/PoseWithCovarianceStamped` on `/rtabmap/global_pose`. The adapter
can also accept `/localization/ins` directly with conservative fallback
covariance, but that is not the default tested path.

## Mode B: FAST-LIO frontend + RTAB-Map backend

Mode B keeps FAST-LIO as the only odometry estimator and uses RTAB-Map only for pose-graph/map construction. The measured interface and frame rationale are in [`../docs/fastlio_interface_report.md`](../docs/fastlio_interface_report.md).

```text
/fast_lio_ns/loc_result                -> RTAB-Map external odometry
/fast_lio_ns/cloud_registered_body     -> RTAB-Map local scan cloud
rtabmap_map -> map -> base_link_fast_lio
```

Do not start `rtabmap_odom/icp_odometry` for this mode, and do not remap `/ins_driver/imu` into RTAB-Map. The frontend helper below starts only `fastlio_mapping`, not the existing `mapping_hainan.launch` auxiliary GPS/localization nodes.

Start a ROS master, then use three terminals.

Terminal 1, FAST-LIO frontend:

```bash
source /opt/ros/noetic/setup.bash
source $HOME/dataDisk/Study/rtabMap_ws/install_isolated/setup.bash
export CMAKE_PREFIX_PATH=$HOME/dataDisk/Study/rtabMap_ws/install_isolated:$CMAKE_PREFIX_PATH
rosrun rtabmap_bringup run_fastlio_frontend_mode_b.sh
```

Terminal 2, RTAB-Map backend (the directory must exist and be writable):

```bash
source /opt/ros/noetic/setup.bash
source $HOME/dataDisk/Study/rtabMap_ws/install_isolated/setup.bash
source $HOME/proj/FAST_LIO_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/dataDisk/Study/rtabMap_ws/install_isolated/share:$HOME/opt/ros-deps/opt/ros/noetic/share:$ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=$HOME/dataDisk/Study/rtabMap_ws/install_isolated:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/opt/rtabmap-noetic/lib:$HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib:$HOME/opt/ros-deps/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export PYTHONPATH=$HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}

MODE_B_OUT=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_output
mkdir -p "$MODE_B_OUT"
roslaunch rtabmap_bringup fastlio_rtabmap_mapping.launch \
  use_sim_time:=true output_dir:="$MODE_B_OUT" delete_db_on_start:=false rviz:=false
```

Terminal 3, short validation then the requested full replay:

```bash
cd $HOME/dataDisk/hainan/yangpu/qc
rosbag play 2026-06-22-12-* --clock -u 30
# Stop the two nodes cleanly, choose a new output directory, then:
rosbag play 2026-06-22-12-* --clock -u 515
```

During the replay, run `rosrun rtabmap_bringup check_fastlio_rtabmap_runtime.sh`. After the nodes exit, analyze the database and compare the sampled FAST-LIO trajectory:

```bash
rosrun rtabmap_bringup analyze_rtabmap_db.py \
  "$MODE_B_OUT/rtabmap.db" --output-dir "$MODE_B_OUT/analysis" \
  --fastlio-trajectory "$MODE_B_OUT/fastlio_odom.csv"
```

`fastlio_rtabmap_mapping.launch` uses exact odometry/cloud synchronization (`approx_sync=false`); this was measured as a 0 ms maximum header-stamp difference across 296 pairs. `Icp/PointToPlaneRadius=0.50` belongs to RTAB-Map's pose-graph registration only, not FAST-LIO odometry.

For a fully isolated, headless replay (especially useful for the 515 s run), launch the orchestrator in a separate session so that it survives terminal/session changes:

```bash
MODE_B_RUN=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_515_run
mkdir -p "$MODE_B_RUN"
setsid nohup bash $HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_bag_test.sh \
  --output "$MODE_B_RUN/output" --duration 515 --port 11337 \
  > "$MODE_B_RUN/console.log" 2>&1 < /dev/null &
```

It writes `mode_b_run_status.txt`, `sync.json`, runtime checkpoints, logs, the database, and `analysis/db_report.txt` in `MODE_B_RUN`. The output path must be new; the script refuses to overwrite an existing `rtabmap.db`.

The completed Yangpu 30 s and 515 s evidence, database statistics, TF/synchronization measurements and caveats are recorded in [`../docs/mode_b_yangpu_test_report.md`](../docs/mode_b_yangpu_test_report.md).

### Mode B multi-session continuation

To append the segment beginning at `rosbag play ... -s 555` to a copy of the
515-second database, use the multi-session helper. It never modifies the seed
database in place and refuses to overwrite an existing destination database.

```bash
SEED_DB=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_515_final_okW4v1/output/rtabmap.db
MULTI_RUN=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_multisession_new

setsid nohup bash $HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_multisession_test.sh \
  --seed-database "$SEED_DB" --output "$MULTI_RUN/output" \
  --start 555 --duration 402.058 --warmup 0 --port 11338 \
  > "$MULTI_RUN/console.log" 2>&1 < /dev/null &
```

The backend loads all old scans while paused. Playback then starts and the
helper resumes RTAB-Map only after FAST-LIO publishes a non-zero pose. This
preserves the old optimized poses for spatial ICP while avoiding the automatic
identity-odom reset. The dedicated configuration is
`config/fastlio_rtabmap_multisession.yaml`.

The verified full result, nine inter-session constraints, database checks and
parameter rationale are in
[`../docs/mode_b_yangpu_multisession_report.md`](../docs/mode_b_yangpu_multisession_report.md).

### Mode B GPS priors and GPS-aligned multi-session continuation

Add `--gps` to both guarded runners. The first run creates
`output/gps_origin.json`; the second run requires and copies that file beside
the seed database so both sessions use exactly the same local GNSS frame.

```bash
GPS_RUN1=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_gps_session1_new
GPS_RUN2=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_gps_multisession_new

bash $HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_bag_test.sh \
  --gps --output "$GPS_RUN1/output" --duration 515 --port 11347

bash $HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_multisession_test.sh \
  --gps --seed-database "$GPS_RUN1/output/rtabmap.db" \
  --output "$GPS_RUN2/output" --start 555 --duration 402.058 --port 11348
```

In the second session, `fastlio_odom_covariance.py` estimates the rigid
FAST-LIO-to-GNSS-local alignment from synchronized poses and publishes odometry
in `gps_local_odom`. This gives RTAB-Map a common raw search frame across a
FAST-LIO restart; GPS remains a position-only graph prior, while LiDAR ICP must
still validate every loop closure.

Export all optimized map components directly from the database scans:

```bash
rosrun rtabmap_bringup export_rtabmap_pcd.sh \
  --database "$GPS_RUN2/output/rtabmap.db" \
  --output "$GPS_RUN2/pcd_export" --voxel-size 0.25
```

The exporter opens a temporary database copy read-only, applies saved poses and
re-optimizes any disconnected component missing from the saved optimized pose
set. It excludes rehearsal nodes (`weight=-9`), writes binary `PointXYZI` PCD,
and generates `inspection/pcd_report.json` plus `pcd_topdown.png`. The completed
GPS two-session run, 41 cross-session closures, GPS-prior coverage, and PCD QA
are recorded in
[`../docs/mode_b_yangpu_gps_multisession_report.md`](../docs/mode_b_yangpu_gps_multisession_report.md).

For any other Yangpu collection, select the bag sequence explicitly. A
multi-session continuation from a different recording starts at zero rather
than inheriting the historical `555 s` offset:

```bash
rosrun rtabmap_bringup run_fastlio_rtabmap_bag_test.sh \
  --gps --bag-dir $HOME/dataDisk/hainan/yangpu/A208 \
  --bag-pattern '2026-06-08-16-*' --duration 697.730 --output "$GPS_RUN1/output"

rosrun rtabmap_bringup run_fastlio_rtabmap_multisession_test.sh \
  --gps --bag-dir $HOME/dataDisk/hainan/yangpu/A205 \
  --bag-pattern '2026-06-09-14-*' --start 0 --duration 932.980 \
  --seed-database "$GPS_RUN1/output/rtabmap.db" --output "$GPS_RUN2/output"
```

The complete A208-to-A205 workflow, validation queries, expected results and
PCD checks are in
[`../docs/a208_a205_gps_multisession_reproduction.md`](../docs/a208_a205_gps_multisession_reproduction.md).

### FAST-LIO rigid alignment to `/localization/ins`

The verified coordinate-first workflow is documented in
[`../docs/fastlio_ins_alignment_reproduction.md`](../docs/fastlio_ins_alignment_reproduction.md).
It first estimates the fixed FAST-LIO-to-INS-local SE(2)+Z transform and the
time offset, then runs RTAB-Map without priors, and only after that enables
translation-only, position-only INS priors. The adapter uses the recorded
`base_link <- ins` static transform; `map_harbor` is never treated as a body
frame alias.
