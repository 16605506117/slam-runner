# Results Summary | 结果汇总

> 自动归档位置：服务器 `/hy-tmp/results/<algo>/<日期>/<时间>/<数据集>/`
> Archived on server: `/hy-tmp/results/<algo>/<date>/<time>/<dataset>/`
> 一键跑评估：`bash run_slam.sh <algo> <launch> <bag> <ds> --eval [--clock]`

## KITTI 07（道路，HDL-64E，~698m，~115s）

| Date | Algorithm | ATE RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|
| 2026-08-05 | LIO-SAM | 0.937 | 2.395 | 547 | GPS fusion ON |
| 2026-08-05 | FAST-LIO | 1.714 | 2.002 | 1100 | pure LiDAR+IMU |
| 2026-08-06 | LIO-SAM | 0.944 | 2.395 | 543 | one-click repro, GPS ON |
| 2026-08-06 | FAST-LIO | 1.701 | 2.010 | 1100 | one-click repro |

## W06（水上 16 线 90°FOV，~79m 直线，~58s）

| Date | Algorithm | ATE 2D RMSE (m) | ATE 3D RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|---|
| 2026-08-05 | FAST-LIO | 0.182 | 0.706 | — | 574 | 2D Umeyama (straight line) |
| 2026-08-06 | FAST-LIO | 0.203 | 0.719 | 2.188 | 575 | one-click repro, evo 全套图 |
| 2026-08-06 | LIO-SAM | 3.575 | 3.580 | 0.635 | 289 | 90°FOV 对 LIO-SAM 不友好 |

## N03 / H05（水上 16 线 90°FOV，用户 2026-08-06 晚已跑 FAST-LIO）

| Sequence | LiDAR 帧 | GT 行 | bag | FAST-LIO ATE 2D RMSE (m) | evo APE RMSE (m) | evo RPE RMSE (m) | 状态 |
|---|---|---|---|---|---|---|---|
| N03 (440-523s) | 823 | 415 | n03.bag | **0.747** (818 poses) | 1.154 | 0.256 | ✅ 已评估 (22-39)，全套图 |
| H05 (160-270s) | 1100 | 550 | h05.bag | **1.042** (1094 poses) | 7.869 | 0.306 | ✅ 已评估 (22-40)，全套图 |

> N03/H05 为用户 2026-08-06 21:35/22:03 用 run_slam.sh 跑的 FAST-LIO 结果；H05 evo APE 大（7.9m）主因是 z 轴高程误差（eval_2d 3D ATE 7.9m vs 2D 1.0m），GPSBase 高程噪声大

## Note | 说明

- 每个 eval 目录含全套图：`est_aligned_traj.png`（轨迹对比）、`est_aligned_err.png`（误差时序+分布）、`traj_cmp_{xy,xyz}_*.png`（evo_traj 叠加）、`ape_plot_*.png` + `ape.zip`（evo_ape）、`rpe_plot_*.png` + `rpe.zip`（evo_rpe），可直接用于论文
- evo_ape/rpe 用水上时用**已 2D 对齐的 est_aligned.tum**（直线轨迹 evo 3D Umeyama 退化，不能直接 -a）
- LIO-SAM 结果含 GPS/navsat 融合；FAST-LIO 为纯 LiDAR+IMU，跨算法对比非严格公平
- 公平对比需关闭 LIO-SAM GPS 重跑（待办）
- 水上 16 线 90° 前视 FOV：FAST-LIO 适配良好（~0.2m）；LIO-SAM 依赖 360° 扫描假设，精度显著下降（~3.6m）
- 直线轨迹 evo 3D Umeyama 会退化（绕前进轴不可观），水上数据用 2D 对齐（eval_2d.py）
- KITTI 指标用 `evo_ape/rpe -a`（SE(3) Umeyama 对齐），RPE delta=1m 平移
