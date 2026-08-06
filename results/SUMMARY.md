# Results Summary | 结果汇总

> 自动归档位置：服务器 `/hy-tmp/results/<algo>/<日期>/<时间>/<数据集>/`
> Archived on server: `/hy-tmp/results/<algo>/<date>/<time>/<dataset>/`

## KITTI 07（道路，HDL-64E，~698m）

| Date | Algorithm | ATE RMSE (m) | RPE RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|
| 2026-08-05 | LIO-SAM | 0.937 | 2.395 | 547 | GPS fusion ON |
| 2026-08-05 | FAST-LIO | 1.714 | 2.002 | 1100 | pure LiDAR+IMU |
| 2026-08-06 | LIO-SAM | 0.944 | 2.395 | 543 | one-click repro, GPS ON |
| 2026-08-06 | FAST-LIO | - | - | - | one-click repro (TBD) |

## W06（水上 16 线 90°FOV，~79m，直线）

| Date | Algorithm | ATE 2D RMSE (m) | ATE 3D RMSE (m) | Poses | Notes |
|---|---|---|---|---|---|
| 2026-08-05 | FAST-LIO | 0.182 | 0.706 | 574 | 2D Umeyama (straight line) |

## Note | 说明

- LIO-SAM 结果含 GPS/navsat 融合；FAST-LIO 为纯 LiDAR+IMU，跨算法对比非严格公平
- 公平对比需关闭 LIO-SAM GPS 重跑（待办）
- 直线轨迹 evo 3D Umeyama 会退化（绕前进轴不可观），水上数据用 2D 对齐（eval_2d.py）
- 所有指标用 `evo_ape/rpe -a`（SE(3) Umeyama 对齐），RPE delta=1m 平移
