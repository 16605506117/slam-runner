# Results Summary | 结果汇总

> 自动归档位置：服务器 `/hy-tmp/results/<algo>/<日期>/<时间>/<数据集>/`
> Archived on server: `/hy-tmp/results/<algo>/<date>/<time>/<dataset>/`
> 一键跑评估：`bash run_slam.sh <algo> <launch> <bag> <ds> --eval [--clock]`

## KITTI 07

| Date | Algorithm | ATE RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|
| 2026-08-05 | LIO-SAM | 0.937 | 2.395 | 547 | GPS fusion ON |
| 2026-08-05 | FAST-LIO | 1.714 | 2.002 | 1100 | pure LiDAR+IMU |
| 2026-08-06 | LIO-SAM | 0.944 | 2.395 | 543 | one-click repro, GPS ON |
| 2026-08-06 | FAST-LIO | 1.701 | 2.010 | 1100 | one-click repro |

## W06

| Date | Algorithm | ATE 2D RMSE (m) | ATE 3D RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|---|
| 2026-08-05 | FAST-LIO | 0.182 | 0.706 | — | 574 | 2D Umeyama (straight line) |
| 2026-08-06 | FAST-LIO | 0.203 | 0.719 | 2.188 | 575 | one-click repro, evo 全套图 |
| 2026-08-06 | LIO-SAM | 3.575 | 3.580 | 0.635 | 289 | 90°FOV 对 LIO-SAM 不友好 |
| 2026-08-08 | FAST-LIVO2 | 0.170 | 0.846 | 0.099 | 573 | pure LiDAR (img_en=0) |
| 2026-08-08 | FAST-LIVO2 | 0.172 | 0.404 | 0.083 | 1146 | **LIVO (img_en=1, 官方标定+校正图)** |

## N03

| Date | Algorithm | ATE 2D RMSE (m) | ATE 3D RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|---|
| 2026-08-06 | FAST-LIO | 0.747 | 1.154 | 0.256 | 818 | evo 全套图 |
| 2026-08-08 | FAST-LIVO2 | 0.257 | 2.585 | 0.195 | 1607 | **LIVO (img_en=1, 官方标定+校正图)** ← 视觉融合大幅领先纯LiDAR |

## H05

| Date | Algorithm | ATE 2D RMSE (m) | ATE 3D RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|---|
| 2026-08-06 | FAST-LIO | 1.042 | 7.869 | 0.306 | 1094 | evo 全套图 |
| 2026-08-08 | FAST-LIVO2 | 0.260 | 0.589 | 0.125 | 1690 | **LIVO (img_en=1, 官方标定+校正图)** ← 视觉融合大幅领先纯LiDAR |

## N02 (SDK 官方序列)

| Date | Algorithm | ATE 2D RMSE (m) | ATE 3D RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|---|
| 2026-08-08 | FAST-LIVO2 | 0.118 | 0.152 | 0.109 | 184 | LIVO (img_en=1, 官方标定原生) |

## Note | 说明

- 每个 eval 目录含全套图：`est_aligned_traj.png`（轨迹对比）、`est_aligned_err.png`（误差时序+分布）、`traj_cmp_{xy,xyz}_*.png`（evo_traj 叠加）、`ape_plot_*.png` + `ape.zip`（evo_ape）、`rpe_plot_*.png` + `rpe.zip`（evo_rpe），可直接用于论文
- **2026-08-08 视觉模式修复**：图像改用官方校正图 `PIC_Left_Rectified`（640×320）+ 官方标定（`USVInland_SLAM_SDK_1.0.1/parameter/Lidar_to_Camera_Left.mat`）+ intensity 用反射强度列（col7）→ FAST-LIVO2 双目融合 N03/H05 大幅领先纯 LiDAR（0.26 vs 0.75/1.04m）
- **GPSBase 无高程列**（README 定义第10列是航向角），GT z=0；水上 3D 指标仅作参考，论文报 2D 为主
- evo_ape/rpe 用水上时用**已 2D 对齐的 est_aligned.tum**（直线轨迹 evo 3D Umeyama 退化，不能直接 -a）
- LIO-SAM 结果含 GPS/navsat 融合；FAST-LIO / FAST-LIVO2 为纯 LiDAR+IMU，跨算法对比非严格公平
- 公平对比需关闭 LIO-SAM GPS 重跑（待办）
- 水上 16 线 90° 前视 FOV：FAST-LIO / FAST-LIVO2 适配良好；LIO-SAM 依赖 360° 扫描假设，精度显著下降（~3.6m）
- KITTI 指标用 `evo_ape/rpe -a`（SE(3) Umeyama 对齐），RPE delta=1m 平移
