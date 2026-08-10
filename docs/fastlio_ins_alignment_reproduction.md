# FAST-LIO + RTAB-Map + INS rigid-alignment reproduction

This procedure is for Ubuntu 20.04, ROS Noetic and the Yangpu QC bags
`2026-06-22-12-*`. Every output directory must be new: the helpers refuse to
overwrite an existing database or calibration.

## 1. Build

```bash
source /opt/ros/noetic/setup.bash
cd $HOME/dataDisk/Study/rtabMap_ws
catkin_make_isolated --install --pkg rtabmap_bringup
source install_isolated/setup.bash
```

FAST-LIO is expected at `$HOME/proj/FAST_LIO_ws` and must already be built.

## 2. Estimate the fixed coordinate/time alignment (120 s)

```bash
ALIGN_RUN=$HOME/dataDisk/hainan/yangpu/qc/fastlio_ins_alignment_new
rosrun rtabmap_bringup run_fastlio_ins_alignment_diagnostic.sh \
  --output "$ALIGN_RUN" --duration 120 --port 11373 \
  --bag-dir $HOME/dataDisk/hainan/yangpu/qc \
  --bag-pattern '2026-06-22-12-*'

ALIGNMENT=$ALIGN_RUN/alignment/ins_fastlio_alignment.yaml
cat "$ALIGNMENT"
```

The estimator independently records FAST-LIO and INS, searches -0.5 to +0.5 s
at 0.01 s intervals, keeps scale fixed at one, fits XY with SE(2), and uses the
median Z residual. Its convention is:

```text
INS time = FAST-LIO time + time_offset_sec
```

Do not continue if the aligned XY plot still has a fixed 90/180 degree error,
if fewer than 20 m of motion was used, or if the post-alignment error has a
clear turn-dependent lead/lag.

## 3. Validate fixed alignment without priors

```bash
RUN120=$HOME/dataDisk/hainan/yangpu/qc/fastlio_ins_aligned_120_new
rosrun rtabmap_bringup run_fastlio_rtabmap_ins_aligned_test.sh \
  --alignment "$ALIGNMENT" --output "$RUN120" --duration 120 --port 11374

RUN515=$HOME/dataDisk/hainan/yangpu/qc/fastlio_ins_aligned_515_new
rosrun rtabmap_bringup run_fastlio_rtabmap_ins_aligned_test.sh \
  --alignment "$ALIGNMENT" --output "$RUN515" --duration 515 --port 11375
```

This mode has no GNSS poser, global pose prior, RTAB-Map IMU or
`rtabmap_odom`. Verify `sync_status=0`, no `FATAL` in `rtabmap.log`, SQLite
`integrity_check=ok`, and a 100% largest connected component in
`analysis/db_report.txt`.

## 4. Enable INS XYZ priors

Only after step 3 passes:

```bash
PRIOR120=$HOME/dataDisk/hainan/yangpu/qc/fastlio_ins_prior_120_new
rosrun rtabmap_bringup run_fastlio_rtabmap_ins_aligned_test.sh \
  --alignment "$ALIGNMENT" --output "$PRIOR120" \
  --duration 120 --port 11376 --ins-prior

PRIOR515=$HOME/dataDisk/hainan/yangpu/qc/fastlio_ins_prior_515_new
rosrun rtabmap_bringup run_fastlio_rtabmap_ins_aligned_test.sh \
  --alignment "$ALIGNMENT" --output "$PRIOR515" \
  --duration 515 --port 11377 --ins-prior
```

The prior path performs all of the following:

- checks `child_frame_id=map_harbor` strictly;
- converts the measured INS pose to physical `base_link` using the recorded
  `base_link <- ins` transform;
- uses `translation_only`, preserving the INS/ENU axes;
- shifts the prior stamps with the calibrated time offset;
- sets angular covariance to 10000 so yaw/roll/pitch are not graph priors;
- uses conservative 2 m, 2 m and 3 m fallback standard deviations because
  `/localization/ins` has all-zero covariance;
- enables GTSAM with `Optimizer/PriorsIgnored=false`.

Check that `analysis/db_report.txt` contains `Pose Prior (7)` and that
`analysis/ins_body_local.csv` is non-empty. The four-trajectory metrics and
plots are `analysis/alignment_report.txt`, `trajectory_metrics.yaml`,
`xy_compare.png`, `z_compare.png` and `error_xy_vs_time.png`.

## 5. Export PCD

```bash
rosrun rtabmap_bringup export_rtabmap_pcd.sh \
  --database "$PRIOR515/rtabmap.db" \
  --output "$PRIOR515/pcd_export" --voxel-size 0.25 \
  --filename yangpu_ins_prior_515.pcd \
  --plot-title 'Yangpu INS-prior 515 s map (top view)'
```

Inspect `pcd_export/inspection/pcd_report.json` and
`pcd_export/inspection/pcd_topdown.png`. The report must show zero non-finite
points and the exporter log must show no empty scans or odometry-fallback graph
components.

## Verified reference run (2026-08-09)

The calibration was `time_offset=-0.20 s`, `yaw=-130.778827 deg`, and
translation `(0.047852, 0.132468, -0.542548) m`. The calibration used 1,195
matched samples and 253.270 m of motion; its XY RMSE changed from 186.094 m to
0.0434 m.

| Run | FAST-LIO raw XY APE | rigid-aligned XY APE | RTAB-Map XY APE | RTAB-Map Z APE |
|---|---:|---:|---:|---:|
| 120 s, no prior | 186.023 m | 0.0452 m | 0.0452 m | 0.2914 m |
| 515 s, no prior | 560.513 m | 0.5011 m | 0.4797 m | 1.5170 m |
| 120 s, INS XYZ prior | 186.216 m | 0.2761 m | 0.0535 m | 0.0482 m |
| 515 s, INS XYZ prior | 560.612 m | 0.4504 m | 0.0551 m | 0.0306 m |

The no-prior rows use the raw INS local trajectory as the diagnostic reference;
the prior rows use the more exact `base_link <- ins` extrinsic-corrected INS
base trajectory.

The raw values are dominated by the fixed coordinate-frame mismatch and are
not trajectory drift. After rigid alignment, the increasing 515 s XY/Z error
is genuine FAST-LIO drift. Position priors correct that drift but intentionally
leave attitude unconstrained; the final heading RMSE was 1.283 deg.

The final 515 s database had 889 active nodes, 777 active pose priors, one
100%-connected component, zero local/global closures and SQLite
`integrity_check=ok`. Its odometry/cloud synchronization had 5,140 exact stamp
pairs and zero unmatched clouds. The exported 0.25 m PCD had 1,731,619 finite
points, no non-finite points and no empty scans or fallback graph components.

Verified artifacts:

```text
alignment  /home/glf/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_ins_alignment_diagnostic_120_20260809/alignment/ins_fastlio_alignment.yaml
no prior  /home/glf/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_ins_aligned_515/rtabmap.db
INS prior /home/glf/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_ins_prior_final_515/rtabmap.db
PCD       /home/glf/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_ins_prior_final_515/pcd_export_final/yangpu_ins_prior_515.pcd
```
