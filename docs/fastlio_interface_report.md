# Yangpu FAST-LIO interface report (Mode B)

This report records the interface actually produced by the local FAST-LIO checkout, rather than assuming the upstream FAST-LIO topic names.

## Checked source and configuration

| Item | Value |
|---|---|
| FAST-LIO checkout | `/home/glf/proj/FAST_LIO_ws/src/FAST_LIO` |
| revision | `8f815acd5135afe08b9d49ac16f9e70dd3ce95ba` |
| input configuration | `config/hainan.yaml` (not edited by Mode B) |
| bag cloud / IMU | `/lidar_preprocessor/meta_cloud` / `/ins_driver/imu` |
| standalone verification | `fastlio_sync_LEtISq`, 30 s of `2026-06-22-12-08-26_0.bag` |

The existing `mapping_hainan.launch` also starts `localization-pillar` and RViz. Mode B does **not** edit or use that launch. `run_fastlio_frontend_mode_b.sh` loads the same YAML and sets its existing frontend-only outer parameters at runtime (`point_filter_num=4`, `max_iteration=3`, filters and `cube_side_length`) before starting only `fastlio_mapping`.

`point_filter_num=4` is important: it is set in the old launch, but is not present in `hainan.yaml`. Leaving it at the binary default made the first scan too dense to process within the 30 s smoke test.

## Actual FAST-LIO output contract

| Output | ROS type | Header frame | Child frame | Stamp source | Mode B use |
|---|---|---|---|---|---|
| `/fast_lio_ns/loc_result` | `nav_msgs/Odometry` | `map` | `base_link_fast_lio` | `lidar_end_time` | RTAB-Map external odometry |
| `/fast_lio_ns/cloud_registered_body` | `sensor_msgs/PointCloud2` | `base_link_fast_lio` | — | `lidar_end_time` | RTAB-Map scan cloud |
| `/fast_lio_ns/cloud_registered` | `sensor_msgs/PointCloud2` | `map` | — | `lidar_end_time` | **not used**; it is already world-frame |
| `/fast_lio_ns/path` | `nav_msgs/Path` | `map` | — | `lidar_end_time` | visual comparison only |
| `/tf` | `tf2_msgs/TFMessage` | `map` | `base_link_fast_lio` | odometry stamp | dynamic FAST-LIO pose TF |

The local source hard-codes the FAST-LIO world frame as `map`; it is the frontend odometry frame for this integration. RTAB-Map therefore uses `rtabmap_map` as its map root, producing:

```text
rtabmap_map --(RTAB-Map correction)--> map --(FAST-LIO)--> base_link_fast_lio
```

Using `map` for both map roots would make the correction TF ambiguous. No static transform is injected between these frames.

## Measured synchronization

The standalone 30 s test received 296 odometry messages and 296 body-frame clouds. Both measured `9.9990 Hz`; every cloud had a corresponding odometry stamp, with mean, median and maximum stamp difference of `0.0 ms`.

The integrated test separately observed 295 `map -> base_link_fast_lio` TF messages at `10.0002 Hz`. Thus the current launch deliberately uses `approx_sync=false`.

## Input-data caveat

The Yangpu fused cloud does not contain the local custom `feature`, `ring` or `azimuth` fields expected by the modified `MERGED` preprocessing branch. PCL reports those fields as absent (three informational messages per cloud), but those values are only copied to unused normal components in this source. With the runtime frontend settings above, odometry and both registered clouds still publish continuously at about 10 Hz. No timestamp loop-back or FAST-LIO reset was observed in the smoke test.

## RTAB-Map subscriptions

Mode B remaps only the external odometry and local registered cloud:

```text
/rtabmap/rtabmap/odom       <- /fast_lio_ns/loc_result
/rtabmap/rtabmap/scan_cloud <- /fast_lio_ns/cloud_registered_body
```

`odom_frame_id` is empty, so RTAB-Map consumes the odometry topic rather than launching an odometry node. `rtabmap_odom/icp_odometry` is never launched. `CoreWrapper` has an asynchronous `imu` subscriber in this RTAB-Map version, but no Mode B IMU topic is remapped or published beneath `/rtabmap`; RTAB-Map therefore does not perform a second IMU fusion loop.
