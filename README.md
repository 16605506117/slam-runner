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
- **多算法支持** Supports FAST-LIO / LIO-SAM / FAST-LIVO2 / Point-LIO / LiDAR_IMU_Init / R3LIVE / FAST-Calib
- **双目视觉融合** FAST-LIVO2 visual-inertial-lidar fusion with official USVInland calibration
- **中英双语文档** Bilingual docs

## Requirements | 依赖

- Ubuntu 20.04 + ROS Noetic
- catkin workspace with algorithm packages: `lio_sam`, `fast_lio`, `fast_livo`, `point_lio`, `points_time_fix`
- Python: `evo` (conda env), numpy, scipy, rosbag
- Xvfb (optional, for headless RViz via VNC)
- Sophus (non-templated version, see FAST-LIVO2 README)

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

# FAST-LIVO2 on water dataset (visual fusion, official calibration)
bash run_slam.sh fast_livo mapping_water.launch \
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
│   ├── eval_fast_livo.sh # FAST-LIVO2 evaluation (bag -> TUM -> ATE/RPE, kitti/water)
│   ├── archive.sh        # archive results to results/<algo>/<date>/<time>/<dataset>/
│   ├── csv2bag.py        # water dataset csv -> rosbag converter (rectified images + official calib)
│   ├── eval_2d.py        # straight-line trajectory 2D alignment evaluation
│   ├── update_summary.sh # append metrics row to results/SUMMARY.md
│   ├── setup_instance.sh # one-command server instance recovery (ROS + packages + data)
│   └── kitti_est_gt_to_tum.py  # KITTI mat_out/GT -> TUM
├── configs/              # launch & param files per algorithm (fast_lio/fast_livo/lio_sam/point_lio)
├── src/                  # full algorithm source code (see Algorithm Sources below)
├── docs/                 # server operation handbook
└── results/
    └── SUMMARY.md        # metric summary table (the only results tracked in git)
```

## Algorithm Sources | 算法来源

All algorithms in `src/` are from open-source projects; their licenses belong to the original authors. This repo only adds water-dataset configs & minimal patches on top.

`src/` 下全部算法来自开源项目，版权归原作者；本项目仅在其上新增水上数据配置与少量补丁。

| Package | Original project | Purpose | Our changes |
|---|---|---|---|
| `FAST_LIO-main` | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) (HKU MARS) | LiDAR-inertial odometry (pure LiDAR+IMU) | water config (`mapping_water.launch` / `velodyne_water.yaml`, 16-line front FOV) |
| `LIO-SAM-master` | [TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) | LiDAR-inertial odometry w/ optional GPS fusion | water config + sim-time launch + full-topic RViz + rviz arg passthrough |
| `FAST-LIVO2` | [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) (HKU MARS) | LiDAR-inertial-visual tightly-coupled odometry | **main visual fusion used in this project**; `timestamp_unit` param patch (water bag uses seconds), `img_en=0` pure-LiDAR mode, water config + official USVInland calibration |
| `Point-LIO-point-lio-with-grid-map` | [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) (HKU MARS) | High-bandwidth LiDAR-inertial odometry (robust to aggressive motion) | water config; note: mirrored trajectory on water data (under investigation) |
| `r3live-master` | [hku-mars/r3live](https://github.com/hku-mars/r3live) (HKU MARS) | LiDAR-inertial-visual odometry + RGB mapping | not adapted yet: `velo16_handler` is an empty TODO in upstream |
| `LiDAR_IMU_Init-main` | [hku-mars/LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init) (HKU MARS, IROS'22) | LiDAR-IMU extrinsic & time-offset calibration | built, ready to use |
| `FAST-Calib-main` | [hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib) (HKU MARS) | LiDAR-camera extrinsic calibration | built, calibration tools |
| `livox_ros_driver` | [Livox-SDK/livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) | Livox LiDAR ROS driver (with Livox-SDK) | not used on water data (Velodyne VLP-16) |
| `points_time_fix` | **self-written** (syw2000) | adds per-point `time` to PointCloud2 missing it (azimuth model, needed by KITTI bags) | n/a (our own package) |
| `rpg_vikit` | [xuankuzcr/rpg_vikit](https://github.com/xuankuzcr/rpg_vikit) (fork specified by FAST-LIVO2 README) | visual-odometry toolkit, dependency of FAST-LIVO2 | none |

> System dependency (not in `src/`): [Sophus](https://github.com/strasdat/Sophus) at commit `a621ff` (non-templated/double-only version required by FAST-LIVO2's README). Note: that commit hash has been rewritten/hidden upstream; keep a local copy or use a fork.

## Dataset | 数据集

- **KITTI 07** (road): `kitti_2011_09_30_drive_0027_synced.bag` + `poses/07.txt` ground truth
- **USVInland** (water, from [USVInland dataset](https://github.com/OPV2V/USVInland) / RA-L 2021): W06 / N03 / H05 sequences (16-line Velodyne VLP-16 90° front FOV + stereo camera + IMU + GPS), official SDK provides `Lidar_to_Camera_Left.mat` calibration and `PIC_Left_Rectified` rectified images

## Results | 实验结果

See [results/SUMMARY.md](results/SUMMARY.md)

## Algorithm Notes | 算法说明

- **LIO-SAM**: savePCD on Ctrl-C; trajectory only via topic `/lio_sam/mapping/odometry` -> must record
- **FAST-LIO**: writes `Log/mat_out.txt` on SIGINT; uses points_time_fix for KITTI bags (missing per-point time)
- **KITTI bags** need `points_time_fix` (adds per-point time field by azimuth model) and sim-time launch for RViz

## License

MIT
