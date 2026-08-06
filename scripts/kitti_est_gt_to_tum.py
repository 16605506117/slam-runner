#!/usr/bin/env python3
"""生成带时间戳的 TUM 轨迹：est（FAST-LIO mat_out）+ GT（KITTI 07.txt）。

对齐依据：bag /points_raw 保留 KITTI 原始时间戳，bag 帧 i ↔ GT 帧 i；
FAST-LIO 丢掉开头 3 帧（mat_out 首帧相对时间 0.3118 ≈ 3×0.1039），
所以 est 帧 j 的绝对时间 = bag 帧 (j+3) 的时间戳。
"""
import os
import numpy as np
import rosbag

BAG = "/hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag"
GT = "/hy-tmp/datasets/road/kitti/poses/07.txt"
MAT_OUT = "/hy-tmp/catkin_ws/src/FAST_LIO-main/Log/mat_out.txt"
HERE = os.path.dirname(os.path.abspath(__file__))
DST_GT = os.path.join(HERE, "gt07.tum")
DST_EST = os.path.join(HERE, "est_kitti.tum")


def R2quat(R):
    """旋转矩阵 -> 四元数 [x y z w]（标准算法，w>=0）"""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return [x, y, z, w]


# 1. bag 帧时间戳
with rosbag.Bag(BAG) as b:
    stamps = np.array([m.header.stamp.to_sec() for _, m, _ in b.read_messages("/points_raw")])
print(f"bag 帧数: {len(stamps)}, 首帧 {stamps[0]:.3f}")

# 2. GT 07.txt -> gt07.tum（GT 帧 i 时间 = bag 帧 i）
gt = np.loadtxt(GT).reshape(-1, 3, 4)
n = min(len(gt), len(stamps))
with open(DST_GT, "w") as f:
    for i in range(n):
        R, t = gt[i, :3, :3], gt[i, :3, 3]
        q = R2quat(R)
        f.write(f"{stamps[i]:.9f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
print(f"GT 07: {n} 帧 -> {DST_GT}")

# 3. mat_out -> est_kitti.tum（est 帧 j 时间 = bag 帧 j+3）
DROP = 3  # FAST-LIO 丢弃的开头帧数
rows = []
with open(MAT_OUT) as f:
    for ln in f:
        v = ln.split()
        if len(v) >= 7:
            rows.append((float(v[0]), float(v[1]), float(v[2]), float(v[3]), float(v[4]), float(v[5]), float(v[6])))
with open(DST_EST, "w") as f:
    for j, (rel_t, r, p, y, x, y_, z) in enumerate(rows):
        if j + DROP >= len(stamps):
            break
        # est 帧 j 对应 bag 帧 (j+DROP)，绝对时间 = 该帧的 bag 时间戳
        t = stamps[j + DROP]
        # ZYX 度 -> 四元数
        r, p, y = np.deg2rad([r, p, y])
        cr, sr, cp, sp, cy, sy = (np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2))
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        f.write(f"{t:.9f} {x:.6f} {y_:.6f} {z:.6f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
print(f"est: {len(rows)} 帧 -> {DST_EST}")
