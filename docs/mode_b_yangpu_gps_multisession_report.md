# 洋浦港 Mode B + GPS 多会话建图实跑报告

执行日期：2026-08-08。环境为 Ubuntu 20.04 / ROS Noetic；FAST-LIO 负责
LiDAR+IMU 前端，RTAB-Map 负责三维扫描关键帧、LiDAR ICP 回环和 GTSAM 位姿图，
`gnss_poser` 提供带协方差的 GNSS odometry。

## 结论

GPS 单会话建图、从 `-s 555` 开始的第二会话续建、跨会话 LiDAR 回环以及最终
PCD 导出均已实际跑通。最终数据库包含 1553 个节点和 2 个 `map_id`，建立了
41 条唯一跨会话 Local Space Closure；GTSAM 优化失败为 0。数据库中的 1399
条 Pose Prior 覆盖约 90.1% 的关键帧，GPS 约束已进入图优化，但 GPS 本身没有
被当作回环：所有跨会话连接仍需通过 LiDAR ICP。

正式结果目录：

```text
/home/glf/dataDisk/hainan/yangpu/qc/rtabmap_gps_multisession_final_20260808/
├── output/
│   ├── rtabmap.db              # 281,407,488 bytes
│   └── gps_origin.json         # 从第一会话原样复制
├── analysis/db_report.txt
├── multisession_run_status.txt
├── sync.json
├── global_pose.csv
└── pcd_export/
    ├── yangpu_gps_multisession.pcd
    ├── database_to_pcd.log
    └── inspection/
        ├── pcd_report.json
        └── pcd_topdown.png
```

## GPS 接入和坐标策略

bag 中 `/localization/ins` 是 `nav_msgs/Odometry`，但 covariance 全零。正式测试
启动：

```bash
source $HOME/proj/fast_lio-sam_loop-gps_ws/devel/setup.bash
roslaunch gnss_poser gnss_poser.launch
```

并使用 `/localization/gnss_odom`。接入链路如下：

```text
/localization/gnss_odom
  -> gnss_odom_to_global_pose.py
  -> /rtabmap/global_pose (PoseWithCovarianceStamped)
  -> RTAB-Map Pose Prior (Link type 7)
  -> GTSAM
```

适配器检查 frame、四元数和 covariance，并设置位置标准差下限；图中只约束
GPS 位置，姿态 covariance 被设为很大，避免用 GNSS 姿态过约束 LiDAR/IMU
前端。第一条 GNSS 位姿的完整 SE(3) 逆变换被保存到 `gps_origin.json`，绝对
ENU 转为局部坐标；第二会话必须复用该文件，不能重新取原点。

FAST-LIO 每次启动都会从自身原点重新开始。第二会话中，odometry 适配器用
时间邻近的 GNSS 局部位姿计算刚体变换
`T_align = T_gps_local * inverse(T_fastlio)`，输出 `gps_local_odom`。RTAB-Map
因此可以在公共 raw pose 坐标中检索旧会话候选，再由 LiDAR ICP 验证。该对齐
只改变送给后端的 odometry 坐标和 covariance，不改 FAST-LIO 算法或输出。

## 两次回放

第一会话：

```bash
bash $HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_bag_test.sh \
  --gps \
  --output $HOME/dataDisk/hainan/yangpu/qc/rtabmap_gps_session1_515_20260808/output \
  --duration 515 --port 11347
```

结果为 922 个节点、920 条 Pose Prior；5140 对 FAST-LIO odom/body cloud
时间戳完全一致，SQLite `integrity_check=ok`，无 GTSAM 优化失败。

第二会话：

```bash
bash $HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_multisession_test.sh \
  --gps \
  --seed-database $HOME/dataDisk/hainan/yangpu/qc/rtabmap_gps_session1_515_20260808/output/rtabmap.db \
  --output $HOME/dataDisk/hainan/yangpu/qc/rtabmap_gps_multisession_final_20260808/output \
  --start 555 --duration 402.058 --warmup 0 --port 11348
```

脚本先复制 seed DB 和 GPS 原点，暂停载入旧图，在 FAST-LIO 发布非零位姿后
恢复后端。第二段产生 631 个节点；4004 对原始 FAST-LIO odom/body cloud
全部精确配对，频率约 10.0006 Hz，未匹配为 0。

## 数据库和回环验收

| 项目 | 结果 |
|---|---:|
| SQLite `integrity_check` | `ok` |
| Node / Data / Statistics | 1553 / 1553 / 1553 |
| `map_id=0` / `map_id=1` | 922 / 631 |
| Neighbor 记录 | 3060 |
| Local Space Closure 记录 | 1254 |
| Pose Prior | 1399（约 90.1% 节点） |
| 唯一跨会话 closure | 41 |
| 最大连通分量 | 1532 / 1553（98.65%） |
| GTSAM optimization failure | 0 |
| FATAL / DB error | 0 / 0 |

最大连通分量是第一会话原有 901 个主分量节点加第二会话全部 631 个节点；
剩余 21 个孤立节点来自 seed 第一会话，不是续建产生。第二会话全部接入旧图。

在第二会话末尾，跨会话约束连续收敛到旧图起点附近：

```text
node 6 <-> 1549   1.379 m
node 7 <-> 1550   1.215 m
node 7 <-> 1551   0.908 m
node 8 <-> 1552   0.586 m
node 9 <-> 1553   0.605 m
```

这说明起终点接近处确实发生了跨会话闭环。图中也存在更远节点间的跨会话
相对变换；其数值是两关键帧的空间间距，不是 ICP 优化增量，不能用来判断
闭环误差。最终 raw 关键帧轨迹长 1973.773 m，在线 optimized 统计轨迹长
1974.380 m，优化过程稳定。

GPS 是异步输入，在高负载 ICP 回放中并非每个关键帧都获得先验。最终覆盖
1399/1553；第一会话为 920/922，第二会话新增 479/631。已检查缺先验节点
附近仍有时间差不超过 5 ms 的 GNSS 消息；扩大 ROS 队列和增加 0.2 秒延迟
没有提高短测覆盖，因此保留稳定的零延迟方案。90.1% 的先验密度足以提供
连续全局约束，但报告不声称 GPS 覆盖 100%。

## PCD 导出和检查

`map_assembler` 的 occupancy-grid 重建路径在该数据库上发布了 width=0 的
空 cloud；数据库本身的 1553 个 `Data.scan` 都非空。最终导出器因此直接用
RTAB-Map Core 只读解压 `LaserScan`，应用数据库保存的 optimized pose，再做
距离过滤和全局体素化：

```bash
rosrun rtabmap_bringup export_rtabmap_pcd.sh \
  --database /home/glf/dataDisk/hainan/yangpu/qc/rtabmap_gps_multisession_final_20260808/output/rtabmap.db \
  --output /home/glf/dataDisk/hainan/yangpu/qc/rtabmap_gps_multisession_final_20260808/pcd_export \
  --voxel-size 0.25
```

| PCD 项目 | 结果 |
|---|---:|
| 优化位姿 / 有效 scan | 1532 / 1532 |
| 解压输入点 | 25,442,557 |
| 2–100 m 有限点 | 24,855,653 |
| 0.25 m 体素化输出 | 3,191,116 |
| 格式 | binary PCD，`x y z intensity` |
| 文件大小 | 51,058,048 bytes |
| NaN / Inf | 0 |
| X 范围 | -583.987 ～ 481.662 m |
| Y 范围 | -124.233 ～ 156.055 m |
| Z 范围 | -8.823 ～ 36.741 m |

俯视图显示约 1.07 km 长的港区主结构连续，两个会话没有明显整体坐标跳变；
高度范围和远端扫描边界与港区 LiDAR 场景相符。导出前后最终数据库 SHA-256
均为：

```text
2480aba7dae5c3ab00d4f45d44c43e8f84282b3f3e6edd8d3809ba4d245213db
```

因此 PCD 导出没有改写最终数据库。

## 已知限制

- GPS prior 覆盖率为 90.1%，不是 100%。
- 当前使用位置先验；若以后要引入 GNSS heading，必须先标定其 body frame、
  方向定义和协方差，不能直接降低姿态 covariance。
- 最终 PCD 只导出最大优化连通分量的 1532 个节点；seed 中 21 个历史孤立
  节点没有 optimized pose，故不混入主地图。
- 本次 archived `sync.json` 仍按旧的 `rtabmap_map -> map` child 检查 TF，GPS
  模式实际 child 是 `gps_local_odom`。修正后的短测已验证
  `rtabmap_map -> gps_local_odom` 约 20 Hz；这不影响数据库或 PCD 结果。
