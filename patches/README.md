# Patches: SC-PGO 水面高度先验 (Plan B)

本补丁给 gisbi-kim/FAST_LIO_SLAM 的 aloam_velodyne 包（laserPosegraphOptimization.cpp）
增加**水面高度先验因子**，用于水上无人艇数据（90° 前视雷达无地面点，z 高度仅靠
IMU 积分 → bias 不可观 → 线性漂移，H05 达 ±7m）。

## 用法

```bash
# 1. 应用补丁
cd <aloam_velodyne_pkg>/src
patch -p1 < water_height_prior.patch
# (文件较多时手动按 diff 改 5 处更稳)

# 2. 编译
catkin_make --pkg aloam_velodyne
# 注意 CATKIN_DEVEL_PREFIX 可能指向 build/devel，产物需 cp 到 devel/lib

# 3. launch 开启 (见 scpgo_water_z.launch)
<param name="water_height_prior_en" type="bool" value="true"/>
<param name="water_height_value" type="double" value="0.0"/>
<param name="water_height_noise" type="double" value="0.05"/>
```

## 改动内容（5 处）

1. 全局变量：`waterHeightPriorEn` / `waterHeightValue` / `waterHeightNoiseScore`
2. noise 指针：`waterHeightNoise`（Diagonal）
3. `initNoises()`：xy=1e9 大噪声（不约束）、z=waterHeightNoiseScore 方差
4. 关键帧循环：每节点加 `GPSFactor(node, (recentX, recentY, waterHeightValue), waterHeightNoise)`
   - ⚠️ 必须在 `if(hasGPSforThisKF)` 块**外**（并列），否则无 GPS 话题时永不执行
5. `main()`：读三个 nh.param（默认关，兼容原行为）

## 效果（KITTI 格式评估，ATE RMSE 3D）

| 数据集 | 关键帧 | 无先验 | +水高先验 | 2D ATE | z 范围收敛 |
|---|---|---|---|---|---|
| W06 | 143 | 0.42 | 0.30 | 0.215 | [-0.13,0.22]→[-0.16,0.06] |
| N03 | 179 | 1.56 | 0.63 | 0.483 | [-0.47,0.60]→[-0.54,0.48] |
| H05 | 247 | 8.32 | 0.84 | 0.668 | [-3.35,6.67]→[-0.27,0.21] |

2D 精度基本不变（z 约束对 xy 仅微耦合）；结果归档：
`/hy-tmp/results/scpgo/2026-08-11/18-50_zprior/`（每数据集 eval/ 含全套 evo 图）

## 文件

- `water_height_prior.patch`：完整 diff（原始文件来自 FAST_LIO_SLAM-main/SC-PGO/src/）
- `laserPosegraphOptimization.cpp.zprior`：修改后完整源码
