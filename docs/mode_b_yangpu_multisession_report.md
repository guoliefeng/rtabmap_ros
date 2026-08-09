# 洋浦港 Mode B 多会话续建实跑报告

执行日期：2026-08-07。环境为 Ubuntu 20.04 / ROS Noetic，前端为本地
FAST-LIO，后端为 RTAB-Map，未接 GPS，也未启动 `rtabmap_odom/icp_odometry`。

## 结论

多会话续建已实际跑通。第一会话数据库的副本载入后，从整组 bag 的
`-s 555` 回放到末尾。新数据写入 `map_id=1`，并与 `map_id=0` 建立 9 条
唯一 Local Space Closure。新会话全部 623 个节点均接入旧图主连通分量。

正式结果目录：

```text
/home/glf/dataDisk/hainan/yangpu/qc/rtabmap_multisession_full_T09bK4/
├── output/rtabmap.db       # 278,540,288 bytes，保存日志显示 265 MB
├── multisession_run_status.txt
├── sync.json
├── fastlio_odom.csv
├── rtabmap.log / fastlio.log / rosbag.log
└── analysis/
    ├── db_report.txt
    └── trajectory/map_id/FAST-LIO 对比图
```

## 执行方式

输入数据库是第一会话 515 秒结果：

```text
/home/glf/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_515_final_okW4v1/output/rtabmap.db
```

正式回放等价于：

```bash
rosbag play 2026-06-22-12-*.bag --clock -s 555 -u 402.058
```

27 个 bag 的时间跨度为 957.058 秒；从 555 秒偏移后，实际消息回放长度为
400.723 秒。脚本先复制数据库并暂停加载 RTAB-Map，等 FAST-LIO 发布非零
odom 后立即恢复，避免首帧单位位姿触发 odom reset 并清空旧图空间索引。

## 最终参数

多会话专用配置为
`rtabmap_bringup/config/fastlio_rtabmap_multisession.yaml`。关键值：

| 参数 | 值 | 作用 |
|---|---:|---|
| `Mem/InitWMWithAllNodes` | `true` | 载入旧会话扫描供 LiDAR ICP 检索 |
| `RGBD/ProximityMaxGraphDepth` | `0` | 允许检索尚未连通的旧 map |
| `RGBD/LocalRadius` | `15 m` | 初始空间候选半径 |
| `RGBD/ProximityPathMaxNeighbors` | `10` | 合并邻近扫描提高 ICP 稳定性 |
| `Icp/PointToPlane` | `false` | 避免该数据反复从低复杂度点到面回退 |
| `Icp/VoxelSize` | `0.30 m` | 保证图约束计算能跟上回放；不改变 FAST-LIO odom |
| `Icp/MaxTranslation` | `1.5 m` | 接纳实测 1.097 m 修正，继续拒绝约 2.15 m 差候选 |
| `Icp/CorrespondenceRatio` | `0.10` | multi-scan 内部翻倍为 0.20；成功候选实测 0.234 |

## 数据库验收

| 项目 | 结果 |
|---|---:|
| SQLite `integrity_check` | `ok` |
| Node / Data / Statistics | 1545 / 1545 / 1545 |
| `map_id=0` | 922 节点 |
| `map_id=1` | 623 节点 |
| Link | 3068 条双向记录 |
| Neighbor | 3046 条双向记录 |
| Local Space Closure | 22 条双向记录，即 11 条唯一边 |
| 唯一跨会话约束 | 9 条 |
| 最大连通分量 | 1525 节点（98.71%） |

旧图原本主分量有 902 个节点和 20 个孤立节点。最终最大分量 1525 恰好等于
`902 + 623`，说明第二会话所有节点都通过跨会话约束接入旧图主分量；剩余
20 个分量是第一会话中原有的孤立节点，并非本次续建产生。

9 条唯一跨会话边为：

```text
5   <-> 923
6   <-> 924
16  <-> 926
20  <-> 928
890 <-> 1531
898 <-> 1534
904 <-> 1537
910 <-> 1541
911 <-> 1542
```

前四条出现在第二会话开头，后五条出现在末尾，说明后续轨迹在两个时段都与
旧图形成几何重叠，而不是只依赖单条偶然约束。Link 中较大的平移是节点间
相对位姿，不是 `Icp/MaxTranslation` 所限制的 ICP 增量修正。

## 同步、轨迹和修正

| 项目 | 结果 |
|---|---:|
| FAST-LIO odom / body cloud | 4004 / 4004 |
| 精确时间戳配对 | 4004，未匹配 0 |
| 最大时间差 | 0 ms |
| odom / cloud 频率 | 约 10.0006 Hz |
| FAST-LIO TF | 4002，约 9.9997 Hz |
| RTAB-Map TF | 7930，约 19.9996 Hz |
| FAST-LIO 第二段轨迹长度 | 874.932 m |
| RTAB-Map `map_id=1` 原始关键帧轨迹 | 871.089 m |
| 第二会话末节点 raw 到在线 optimized 位移 | 3.514 m |

RTAB-Map 保存了 623 个第二会话 Signature，少于 2 Hz 乘总时长，因为
`RGBD/LinearUpdate` / `AngularUpdate` 会跳过小运动帧，末段近邻 ICP 较慢时同步
队列也会舍弃中间帧。FAST-LIO 的 4004 对 odom/cloud 数据完整且严格同步。

## 告警解释和原库保护

- `FATAL=0`、`DB error=0`、`Odometry is reset=0`。
- 点到点 ICP 对 0.30 m 体素副本发出 1249 条“输入有 normals、输出不保留
  normals”提示；点到点注册不使用 normals，数据库内原始扫描未被改写。
- 纯 LiDAR 模式没有视觉特征，少量 `Missing visual features` 提示符合预期；
  成功约束类型是 Local Space Closure，而不是视觉 Global Closure。
- 第一会话原库与启动前只读副本 SHA-256 均为
  `7bdc06e1c3842b58c7ae0969f24d716eedb11d9e8c29857a4426565d2cb61412`，
  大小和修改时间也一致，确认正式续建没有覆盖原库。

## 复现命令

```bash
SEED_DB=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_fastlio_515_final_okW4v1/output/rtabmap.db
RUN=$HOME/dataDisk/hainan/yangpu/qc/rtabmap_multisession_new

setsid nohup bash $HOME/dataDisk/Study/rtabMap_ws/install_isolated/lib/rtabmap_bringup/run_fastlio_rtabmap_multisession_test.sh \
  --seed-database "$SEED_DB" --output "$RUN/output" \
  --start 555 --duration 402.058 --warmup 0 --port 11338 \
  > "$RUN/console.log" 2>&1 < /dev/null &
```

输出目录必须是新的；脚本检测到目标 `rtabmap.db` 已存在时会拒绝覆盖。
