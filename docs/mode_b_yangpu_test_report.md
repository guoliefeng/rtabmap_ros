# 洋浦港 Mode B 实跑报告：FAST-LIO 前端 + RTAB-Map 后端

执行日期：2026-08-07。系统为 Ubuntu 20.04 / ROS Noetic，未使用 GPS。

## 结论

Mode B 已实际跑通。FAST-LIO 是唯一的里程计前端，RTAB-Map 使用其外部 odom 与机体系局部点云建立增量图；未启动 `rtabmap_odom/icp_odometry`，也没有向 RTAB-Map 重映射 IMU。515 秒真实 bag 回放以 `sync_status=0` 完成，数据库正常保存为 154 MB。

最终运行目录：

```text
/home/glf/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_515_final_okW4v1/
├── output/rtabmap.db            # 162,164,736 bytes (保存日志报 154 MB)
├── sync.json                    # odom/cloud/TF 实测
├── fastlio_odom.csv
├── runtime_30.log
└── analysis/
    ├── db_report.txt
    ├── trajectory_xy.png / trajectory_z.png / map_id_timeline.png
    ├── fastlio_xy.png / rtabmap_xy.png
    └── fastlio_vs_rtabmap_xy.png / fastlio_vs_rtabmap_z.png
```

## 版本和改动范围

| 项目 | 版本 / 状态 |
|---|---|
| FAST-LIO | `8f815acd5135afe08b9d49ac16f9e70dd3ce95ba` |
| rtabmap_ros | `454b1b11cff3c403270eeca912cb4b126c9e7d9a` |
| FAST-LIO 原有本地修改 | `config/hainan.yaml`、`launch/mapping_hainan.launch`、`src/IMU_Processing.hpp`、`src/laserMapping.cpp`；未覆盖或修改 |
| Mode A | `lidar_imu_mapping.launch` 未修改 |
| 新增 Mode B 说明 | [`fastlio_interface_report.md`](fastlio_interface_report.md) |

Mode B 新增的 launch、配置、前端/长回放脚本和数据库分析脚本均位于 `rtabmap_bringup/`，不修改 RTAB-Map core。

## 接口、同步和 TF

| 检查项 | 515 秒实测结果 |
|---|---|
| FAST-LIO odom | `/fast_lio_ns/loc_result`，5140 条，`9.99998 Hz` |
| FAST-LIO 局部 cloud | `/fast_lio_ns/cloud_registered_body`，5140 条，`9.99998 Hz` |
| odom/cloud 配对 | 5140/5140；未匹配为 0 |
| 时间戳差 | mean / median / max 均为 `0.0 ms` |
| FAST-LIO TF | `map -> base_link_fast_lio`，5139 条，`10.00005 Hz` |
| RTAB-Map TF | `rtabmap_map -> map`，10273 条，`20.00006 Hz` |
| 同步结论 | `approx_sync=false` |
| 时间回退 | LiDAR / IMU 均为 0 |

因此实际 TF 树为：

```text
rtabmap_map -> map -> base_link_fast_lio
```

RTAB-Map 的 `odom_frame_id` 为空，确认它消费 `/fast_lio_ns/loc_result`，而非运行内部 odometry。30 秒运行检查中只有 `/fast_lio_ns/fastlio_mapping` 和 `/rtabmap/rtabmap` 两个算法节点；没有 GPS/定位辅助节点。

## 数据库和轨迹

| 项目 | 结果 |
|---|---|
| Node / Data / Statistics | 922 / 922 / 922 |
| Link | 1802，均为 Neighbor (0) |
| map_id | 仅 `map_id=0`，922 节点 |
| 最大连通分量 | 902 节点（97.83%） |
| RTAB-Map raw / optimized 轨迹长度 | 1083.793 m / 1083.793 m |
| FAST-LIO raw 轨迹长度 | 1084.146 m |
| FAST-LIO X/Y/Z 范围 | X `[-484.747, 16.196]`，Y `[0.005, 88.716]`，Z `[-2.224, 0.759]` |

RTAB-Map 与 FAST-LIO 轨迹长度只差约 0.353 m（约 0.03%）。数据库包含每个节点的扫描数据，因此可在后续有 GUI/导出工具的环境中重新加载、滤波与导出点云。

## 闭环与告警解释

本次数据库中没有 Global/Local Space/Loop Closure 类型 link，只有 1802 条邻接约束；优化轨迹与原始轨迹也相同。这意味着本段数据的 Mode B 已完成连续局部三维建图和图数据库保存，但**没有观测到成功的闭环约束或全局漂移校正**。

原因和后续动作：

- 这是纯 LiDAR 云输入，RTAB-Map 日志按预期禁用了 bag-of-words 图像闭环；本轮 proximity ICP 也没有形成额外约束。
- `Icp/PointToPlaneRadius=0.50` 已被加载，但日志出现 232 次“结构复杂度不足，回退 PointToPoint”。这不是崩溃；该段融合云无法始终可靠估计点到面法向。后续可比较 `Icp/PointToPlane:=false`（减少回退噪声）与经过体素化/法向估计的输入云，且应一次只改一项。
- FAST-LIO 有 15432 条 `feature/ring/azimuth` 缺失提示（每帧三个可选字段）；本地 `MERGED` 分支只将这些值写入未使用的 normal 分量。输出频率、TF、时间同步都正常，故它是已记录的数据格式告警而非运行失败。
- 启动阶段出现 2 次空扫描跳过，未发生后续重置或时间回退。

若下一阶段目标是可靠闭环，建议在同一数据上增加可验证的回环/描述子策略（例如提供 scan descriptor 或在存在重访区段时调试 proximity ICP），并以 Link 表中出现非 Neighbor link、优化轨迹相对 raw 变化作为验收证据。

## 验收记录

- FAST-LIO Release 构建成功。
- `rtabmap_bringup` 安装成功；launch 解析仅包含 `/rtabmap/rtabmap`。
- 独立 FAST-LIO 30 秒测得 296 对消息、最大时间差 0 ms。
- 端到端 30 秒测得 53 Node、102 Neighbor Link，数据库 9 MB。
- 端到端 515 秒完成且显式记录数据库保存完成；RTAB-Map `FATAL` / `DB error` 为 0。
