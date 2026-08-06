# slam-runner

> 通用 SLAM 一键跑 + 评估框架 | Universal LiDAR SLAM run & evaluate framework

基于 ROS1 的一键化工具集：**启动 SLAM → 自动录制轨迹 → 回放数据 → 触发落盘 → 归档 → 自动评估**（ATE/RPE），支持 FAST-LIO 与 LIO-SAM，可扩展到其他算法与数据集。

One-click toolkit on ROS1: start SLAM -> record trajectory -> play data -> trigger save -> archive -> auto-evaluate (ATE/RPE). Supports FAST-LIO and LIO-SAM, extensible to more algorithms & datasets.

---

## Features | 特性

- **一键执行** One command for the whole pipeline (run_slam.sh)
- **自动录制** Auto-record trajectory (no more forgotten `rosbag record`)
- **自动评估** Auto ATE/RPE with evo (SE3 Umeyama alignment, headless Agg backend)
- **自动归档** Time-stamped result archive, never overwritten
- **可扩展** Add a new algorithm = one `run_slam.sh` invocation + one eval script
- **中英双语文档** Bilingual docs

## Requirements | 依赖

- Ubuntu 20.04 + ROS Noetic
- catkin workspace with algorithm packages: `lio_sam`, `fast_lio`, `points_time_fix`
- Python: `evo` (conda env), numpy, scipy, rosbag
- Xvfb (optional, for headless RViz via VNC)

## Quick Start | 快速开始

```bash
# LIO-SAM on KITTI 07 (sim-time + evaluate)
bash run_slam.sh lio_sam run_kitti_simtime.launch \
  /hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag \
  kitti_07 --clock --eval

# FAST-LIO on KITTI 07 (evaluate)
bash run_slam.sh fast_lio mapping_velodyne_kitti.launch \
  /hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag \
  kitti_07 --eval

# FAST-LIO on water dataset W06 (2D evaluate)
bash run_slam.sh fast_lio mapping_water.launch \
  /hy-tmp/datasets/water/w06.bag w06 --eval

# Disconnection-safe (run in background, watch log)
setsid nohup bash run_slam.sh ... > /tmp/run_slam.log 2>&1 &
tail -f /tmp/run_slam.log
```

## Options | 参数

| Option | Meaning |
|---|---|
| `--rviz` | enable RViz (needs Xvfb :1) |
| `--eval` | run evaluation after finish |
| `--clock` | `rosbag play --clock` (required for sim-time launch) |
| `--rate X` | bag play speed (default 1.0) |
| `--no-record` | skip trajectory recording |

## Directory | 目录结构

```
slam-runner/
├── run_slam.sh           # main entry: run + record + play + archive + eval
├── scripts/
│   ├── eval_lio_sam.sh   # LIO-SAM evaluation (bag -> TUM -> ATE/RPE)
│   ├── eval_fast_lio.sh  # FAST-LIO evaluation (mat_out -> TUM -> ATE/RPE, kitti/water)
│   ├── archive.sh        # archive results to results/<algo>/<date>/<time>/<dataset>/
│   ├── csv2bag.py        # water dataset csv -> rosbag converter
│   ├── eval_2d.py        # straight-line trajectory 2D alignment evaluation
│   └── kitti_est_gt_to_tum.py  # KITTI mat_out/GT -> TUM
├── configs/              # launch & param files (deploy to catkin_ws when used)
├── docs/                 # server operation handbook
└── results/
    └── SUMMARY.md        # metric summary table (the only results tracked in git)
```

## Results | 实验结果

See [results/SUMMARY.md](results/SUMMARY.md)

## Algorithm Notes | 算法说明

- **LIO-SAM**: savePCD on Ctrl-C; trajectory only via topic `/lio_sam/mapping/odometry` -> must record
- **FAST-LIO**: writes `Log/mat_out.txt` on SIGINT; uses points_time_fix for KITTI bags (missing per-point time)
- **KITTI bags** need `points_time_fix` (adds per-point time field by azimuth model) and sim-time launch for RViz

## License

MIT
