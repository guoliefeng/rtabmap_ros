# Yangpu rosbag interface report

Generated: 2026-08-08T12:30:21+08:00

## Input files

- Matched files: **130**
- Total indexed duration: **932.980 s**

## Static checks

PASS: **PointCloud2 topics** — /lidar_left_front, /lidar_preprocessor/meta_cloud, /lidar_right_rear
PASS: **/lidar_preprocessor/meta_cloud** — present
PASS: **meta_cloud frame** — base_link
PASS: **/ins_driver/imu type** — sensor_msgs/Imu
PASS: **IMU quaternion** — finite and non-zero in 702 sampled messages
PASS: **IMU orientation covariance** — covariance[0] != -1 in 702 sampled messages
PASS: **base_link <- cloud frame** — base_link (identity or static TF)
PASS: **base_link <- IMU frame** — static TF base_link -> chcnav_msg_parser
PASS: **/localization/ins** — nav_msgs/Odometry, frame=map, child_frame_id=map_harbor
WARN: **/clock in bag** — absent; rosbag play --clock must publish it at runtime
PASS: **/tf_static** — present
WARN: **/tf** — absent; odometry will publish dynamic TF

## PointCloud2 topics

| Topic | Type | Messages | Indexed rate | First frame | Fields | x y z intensity | time/timestamp | ring |
|---|---|---:|---:|---|---|---|---|---|
| `/lidar_left_front` | `sensor_msgs/PointCloud2` | 9292 | 10.000 Hz | `lidar_left_front` | `x(FLOAT32@0), y(FLOAT32@4), z(FLOAT32@8), intensity(FLOAT32@16), timestamp(FLOAT64@24), ring(UINT16@32)` | PASS | yes | yes |
| `/lidar_preprocessor/meta_cloud` | `sensor_msgs/PointCloud2` | 9291 | 9.995 Hz | `base_link` | `x(FLOAT32@0), y(FLOAT32@4), z(FLOAT32@8), intensity(FLOAT32@16)` | PASS | no | no |
| `/lidar_right_rear` | `sensor_msgs/PointCloud2` | 9292 | 9.999 Hz | `lidar_right_rear` | `x(FLOAT32@0), y(FLOAT32@4), z(FLOAT32@8), intensity(FLOAT32@16), timestamp(FLOAT64@24), ring(UINT16@32)` | PASS | yes | yes |

## Candidate topics

- `/ins_driver/imu`: `sensor_msgs/Imu`, **93278** messages, indexed rate **94.347 Hz**
- `/lidar_preprocessor/meta_cloud`: `sensor_msgs/PointCloud2`, **9291** messages, indexed rate **9.995 Hz**
- `/localization/ins`: `nav_msgs/Odometry`, **84734** messages, indexed rate **45913.184 Hz**

## TF static summary

- `base_link -> chcnav_msg_parser`
- `base_link -> ins`
- `base_link -> lidar_left_front`
- `chcnav -> c_rs232`
- `lidar_left_front -> lidar_front`
- `lidar_left_front -> lidar_left`
- `lidar_left_front -> lidar_rear`
- `lidar_left_front -> lidar_right`
- `lidar_left_front -> lidar_right_rear`
- `map -> chcnav`

## Interpretation

- The selected cloud is already expressed in `base_link`; no cloud-frame static transform is needed.
- The selected IMU frame is `chcnav_msg_parser`; the bag contains a direct static transform from `base_link` to that frame, so the launch default is verified from the bag.
- The cloud fields do not include per-point time or ring metadata; first-stage bringup therefore keeps `deskewing=false` and does not invent a ring/time interface.
- `/localization/ins` is recorded as `nav_msgs/Odometry`, but GPS/INS is intentionally not connected in this stage.

