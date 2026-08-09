# A208 + A205：FAST-LIO、RTAB-Map、GPS 多会话建图与 PCD 导出手册

本文给出已经实跑验证的完整流程：

1. 用 A208 从零创建第一会话地图；
2. 复制第一会话数据库，用 A205 创建第二会话并续建；
3. 检查时间同步、GPS Pose Prior、数据库完整性和跨会话约束；
4. 从最终数据库导出包含两个会话的二进制 `PointXYZI` PCD。

所有命令均为无界面运行。脚本会自行启动独立 `roscore`、FAST-LIO、
`gnss_poser`、GPS 适配器、RTAB-Map 和 `rosbag play`。不要再在其他终端手动
启动 `gnss_poser` 或播放同一组 bag。

## 1. 已验证的输入和工作空间

```text
Ubuntu / ROS       Ubuntu 20.04 / ROS Noetic
RTAB-Map 工作空间  /home/glf/dataDisk/Study/rtabMap_ws
FAST-LIO 工作空间  /home/glf/proj/FAST_LIO_ws
GNSS 工作空间      /home/glf/proj/fast_lio-sam_loop-gps_ws
会话 1             /home/glf/dataDisk/hainan/yangpu/A208/2026-06-08-16-*
会话 2             /home/glf/dataDisk/hainan/yangpu/A205/2026-06-09-14-*
原始 INS            /localization/ins
ENU GNSS odometry   /localization/gnss_odom
```

`gnss_poser` 把 `/localization/ins` 转成带 covariance 的
`/localization/gnss_odom`。第一会话生成 `gps_origin.json`；续建脚本将它原样复制
到第二会话，使两次 FAST-LIO 重启后的结果仍处于同一个局部 ENU 坐标系。
GPS 作为位置先验进入图优化，LiDAR ICP 仍负责确认回环。

运行脚本设置了 `rviz:=false`，因此建图和导出不需要启动 `rtabmap_viz`。

## 2. 一次性准备

先确认输入和三个工作空间已经存在：

```bash
test -x /home/glf/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_bag_test.sh
test -x /home/glf/proj/FAST_LIO_ws/devel/lib/fast_lio/fastlio_mapping
test -f /home/glf/proj/fast_lio-sam_loop-gps_ws/devel/setup.bash
find /home/glf/dataDisk/hainan/yangpu/A208 -maxdepth 1 -name '2026-06-08-16-*' -type f | sort
find /home/glf/dataDisk/hainan/yangpu/A205 -maxdepth 1 -name '2026-06-09-14-*' -type f | sort
df -h /home/glf/dataDisk/hainan/yangpu
```

若是新环境，可先补充手册和检查脚本使用的系统工具。ROS/PCL 等依赖建议交给
`rosdep` 根据当前源码解析：

```bash
sudo apt update
sudo apt install sqlite3 python3-numpy python3-matplotlib python3-rosdep
cd /home/glf/dataDisk/Study/rtabMap_ws
rosdep install --from-paths src --ignore-src -r -y
```

当前机器已经编译安装完成。仅当源码被修改或换到新机器时重新编译：

```bash
cd /home/glf/dataDisk/Study/rtabMap_ws
catkin_make_isolated --install --pkg rtabmap_bringup
```

## 3. 设置本次输出目录

每次复现使用新的目录名。保护脚本拒绝覆盖已有数据库或 PCD，这是为了避免误删
已完成的长时间结果。

```bash
RTABMAP_WS=/home/glf/dataDisk/Study/rtabMap_ws
RUN1=/home/glf/dataDisk/hainan/yangpu/rtabmap_a208_gps_repro_01
RUN2=/home/glf/dataDisk/hainan/yangpu/rtabmap_a208_a205_gps_repro_01

test ! -e "$RUN1/output/rtabmap.db"
test ! -e "$RUN2/output/rtabmap.db"
mkdir -p "$RUN1" "$RUN2"
```

如果 `test ! -e` 返回非零，说明该名字已经使用；修改 `repro_01`，不要直接删除
原结果。

## 4. 会话 1：A208 从零建图

在前台运行下面一条命令，并等待约 12 分钟直到它自行返回：

```bash
bash "$RTABMAP_WS/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_bag_test.sh" \
  --gps \
  --bag-dir /home/glf/dataDisk/hainan/yangpu/A208 \
  --bag-pattern '2026-06-08-16-*' \
  --duration 697.730 \
  --port 11367 \
  --output "$RUN1/output"
```

该命令从 bag 开头播放，内部等价于 `rosbag play ... --clock`，并自动关闭节点、
让 RTAB-Map 正常保存数据库、生成同步检查和离线分析。

检查第一会话：

```bash
cat "$RUN1/mode_b_run_status.txt"
cat "$RUN1/sync.log"
sed -n '1,160p' "$RUN1/analysis/db_report.txt"
test -s "$RUN1/output/rtabmap.db"
test -s "$RUN1/output/gps_origin.json"

sqlite3 "$RUN1/output/rtabmap.db" \
  "PRAGMA integrity_check;
   SELECT 'active_nodes', count(*) FROM Node WHERE weight > -9;
   SELECT 'active_pose_priors', count(*)
     FROM Link l JOIN Node n ON n.id=l.from_id
    WHERE l.type=7 AND n.weight > -9;"
```

成功标志包括：状态文件含 `finished=` 和 `sync_status=0`，SQLite 输出 `ok`，并且
`gps_origin.json` 非空。本次已验证的 A208 结果是 404 个有效节点、404 条有效 GPS
Pose Prior。

## 5. 会话 2：A205 复制数据库后续建

A205 是另一组录制，所以必须从 `--start 0` 开始，不能沿用旧 QC 测试中的
`-s 555`。脚本先复制 seed 数据库，绝不会原地修改 `$RUN1`。

```bash
bash "$RTABMAP_WS/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_multisession_test.sh" \
  --gps \
  --bag-dir /home/glf/dataDisk/hainan/yangpu/A205 \
  --bag-pattern '2026-06-09-14-*' \
  --start 0 \
  --duration 932.980 \
  --warmup 0 \
  --port 11368 \
  --seed-database "$RUN1/output/rtabmap.db" \
  --output "$RUN2/output"
```

RTAB-Map 先暂停载入旧图；FAST-LIO 发布第一个有效非零位姿后，脚本调用
`/rtabmap/resume`。GPS 对齐适配器随后把新 FAST-LIO 轨迹发布在
`gps_local_odom` 中。

检查第二会话状态和 GPS 原点是否完全一致：

```bash
cat "$RUN2/multisession_run_status.txt"
cat "$RUN2/sync.log"
sed -n '1,220p' "$RUN2/analysis/db_report.txt"
sha256sum "$RUN1/output/gps_origin.json" "$RUN2/output/gps_origin.json"
```

两个 SHA-256 应相同。再做数据库验收：

```bash
sqlite3 "$RUN2/output/rtabmap.db" \
  "PRAGMA integrity_check;
   SELECT 'active_nodes', count(*) FROM Node WHERE weight > -9;
   SELECT 'active_nodes_by_map', map_id, count(*)
     FROM Node WHERE weight > -9 GROUP BY map_id ORDER BY map_id;
   SELECT 'active_pose_priors_by_map', n.map_id, count(*)
     FROM Link l JOIN Node n ON n.id=l.from_id
    WHERE l.type=7 AND n.weight > -9 GROUP BY n.map_id ORDER BY n.map_id;
   SELECT 'cross_session_edges', count(*)
     FROM Link l
     JOIN Node a ON a.id=l.from_id
     JOIN Node b ON b.id=l.to_id
    WHERE a.weight > -9 AND b.weight > -9
      AND a.map_id <> b.map_id AND l.from_id <> l.to_id;"
```

本次完整实跑的结果为：

```text
数据库大小                 514,457,600 bytes（约 491 MiB）
数据库完整性               ok
数据库总节点               2,854
有效建图节点               957
weight=-9 历史中间节点     1,897（不参与导出）
map_id 0 / A208             404
map_id 1 / A205             553
有效 GPS Pose Prior         404 + 328 = 732
odom/cloud 精确时间戳配对   9,276 / 9,276，最大误差 0 ms
跨会话边                   0
有效图连通分量             2
```

这里的 `cross_session_edges=0` 不是程序故障。A208 和 A205 的 GPS ENU 轨迹最近
仍相距约 91.206 m，没有足够的同地点 LiDAR 观测可供 ICP 验证，因此 RTAB-Map
正确地没有制造跨会话闭环。GPS 负责把两个独立分量放进同一 ENU 空间，但 GPS
先验本身不能冒充 LiDAR 回环。若改用确实重叠的数据，`db_report.txt` 中应出现
跨 map 的 closure，并且连通分量数应变为 1。

## 6. 从最终数据库导出完整双会话 PCD

导出不需要启动 `roscore`，也不要复制正在写入的数据库；必须等会话 2 命令正常
结束后再执行。

```bash
bash "$RTABMAP_WS/install_isolated/lib/rtabmap_bringup/export_rtabmap_pcd.sh" \
  --database "$RUN2/output/rtabmap.db" \
  --output "$RUN2/pcd" \
  --voxel-size 0.25 \
  --port 11372
```

导出器会：

- 只读打开数据库临时副本；
- 排除 `weight=-9` 的历史中间节点；
- 使用数据库已保存的优化位姿；
- 若多个会话尚未由 LiDAR 回环连通，按各分量的图约束和 GPS Pose Prior 分别优化；
- 合并全部分量，过滤 2–100 m 点并做 0.25 m voxel；
- 写出二进制 `x y z intensity` PCD、JSON 数值报告和俯视图。

检查导出：

```bash
cat "$RUN2/pcd/export_status.txt"
cat "$RUN2/pcd/database_to_pcd.log"
cat "$RUN2/pcd/inspection/pcd_report.json"
file "$RUN2/pcd/yangpu_gps_multisession.pcd"
ls -lh "$RUN2/pcd/yangpu_gps_multisession.pcd" \
       "$RUN2/pcd/inspection/pcd_topdown.png"
```

必须重点确认：

```text
export_complete=true
exported_nodes=957
connected_components=2
odometry_fallback_components=0
finite_points 与 declared_points 相同
nonfinite_points=0
```

本次实际生成的最终文件位于：

```text
/home/glf/dataDisk/hainan/yangpu/rtabmap_a208_a205_gps_safe_full_20260808/pcd_final/yangpu_gps_multisession.pcd
```

它包含 1,608,502 个有限 XYZI 点，大小 25,736,224 bytes；范围为
`X=518.007 m`、`Y=472.199 m`、`Z=43.851 m`。对应检查文件是同目录下的
`inspection/pcd_report.json` 和 `inspection/pcd_topdown.png`。

## 7. 常见问题

### `unable to open database file`

不要用 `sudo roslaunch`。确认输出父目录存在且当前用户可写，并换用一个新的
`RUN1`/`RUN2` 名称：

```bash
mkdir -p "$RUN1" "$RUN2"
test -w "$RUN1"
test -w "$RUN2"
```

保护脚本会自动创建 `output`，并在目标已有 `rtabmap.db` 时停止。

### 没有闭环或最终仍有两个连通分量

先看 `$RUN2/analysis/db_report.txt` 的 `Cross-session constraints`。起终点接近只对
同一条轨迹内部闭环有意义；不同会话必须在同一 ENU 位置附近具有足够相似的
LiDAR 几何。A208/A205 当前没有空间重叠，两个分量是预期结果，不代表漂移失控。

### RTAB-Map/PCL 因异常点崩溃

当前 launch 默认让 `/fast_lio_ns/cloud_registered_body` 经过
`pointcloud_safety_filter` 后再送给 RTAB-Map。它会移除 NaN/Inf 和超过安全范围
的坏点。不要把 RTAB-Map 输入重新映射回未过滤话题；查看 `rtabmap.log` 和
`fastlio.log` 定位异常。

### 端口占用

修改 `--port` 为未使用的本机端口。两个会话顺序执行时不要求端口相同。

### 中途 Ctrl-C

等待脚本让 RTAB-Map 正常关闭并保存。不要复制仍在增长的 `rtabmap.db`。如果状态
文件没有 `finished=`，该次结果视为不完整，使用新输出目录重跑。

## 8. 最短操作清单

1. 设置新的 `RUN1`、`RUN2`。
2. 执行第 4 节命令，确认 A208 的 `sync_status=0`、SQLite `ok`。
3. 执行第 5 节命令，确认 A205 的 `sync_status=0`、SQLite `ok`、GPS 原点哈希一致。
4. 阅读 `analysis/db_report.txt`，判断跨会话回环是否符合数据的实际空间重叠。
5. 执行第 6 节导出命令，确认 957 个节点、无 fallback、无非有限点。
6. 用 `pcd_topdown.png` 快速检查，再用 CloudCompare/PCL 打开最终 PCD。
