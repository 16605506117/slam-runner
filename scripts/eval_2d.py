#!/usr/bin/env python3
"""通用直线轨迹 2D 对齐评估: 时间最近邻匹配 -> 2D Umeyama -> ATE (+ 轨迹/误差图)
用法: eval_2d.py <est.tum> <gt.tum> [out_aligned.tum]
输出: <out_aligned>.tum 对齐轨迹 + <out_aligned>_traj.png 轨迹对比图 + <out_aligned>_err.png 误差图
默认(兼容旧调用): est=w06_fastlio_est.tum gt=w06_gt_rel.tum out=w06_fastlio_est_aligned.tum
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # headless 安全
import matplotlib.pyplot as plt

est_fn = sys.argv[1] if len(sys.argv) > 1 else 'w06_fastlio_est.tum'
gt_fn = sys.argv[2] if len(sys.argv) > 2 else 'w06_gt_rel.tum'
out_fn = sys.argv[3] if len(sys.argv) > 3 else 'w06_fastlio_est_aligned.tum'

def load(fn):
    ts, pts = [], []
    for ln in open(fn):
        v = ln.split()
        ts.append(float(v[0])); pts.append([float(v[1]), float(v[2]), float(v[3])])
    return np.array(ts), np.array(pts)

def umeyama2d(src, dst):
    s, d = src[:, :2], dst[:, :2]
    mu_s, mu_d = s.mean(0), d.mean(0)
    S = (s - mu_s).T @ (d - mu_d) / len(s)
    U, _, Vt = np.linalg.svd(S)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0: R[1] *= -1
    t = mu_d - R @ mu_s
    return R, t

gt_ts, gt = load(gt_fn)
est_ts, est = load(est_fn)

# 时间最近邻匹配对 (只匹配时间重叠区间)
idx = [np.argmin(np.abs(gt_ts - t)) for t in est_ts]
R, t = umeyama2d(est, gt[idx])
est_al = est.copy()
est_al[:, :2] = (R @ est[:, :2].T).T + t
z_off = np.median(gt[idx][:, 2] - est_al[:, 2])
est_al[:, 2] += z_off

# 误差 (最近邻)
nearest = np.array([np.argmin(np.abs(gt_ts - t)) for t in est_ts])
errs = np.linalg.norm(est_al - gt[nearest], axis=1)
err2 = np.linalg.norm(est_al[:, :2] - gt[nearest][:, :2], axis=1)
len_est = np.sum(np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1))
len_gt = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
print(f"匹配 {len(est_al)} 帧 | 对齐角 {np.degrees(np.arctan2(R[1,0],R[0,0])):.1f}deg 平移({t[0]:.2f},{t[1]:.2f})")
print(f"轨迹长度: est {len_est:.1f}m | gt {len_gt:.1f}m")
print(f"=== ATE 3D: RMSE {np.sqrt((errs**2).mean()):.3f} | mean {errs.mean():.3f} | median {np.median(errs):.3f} | max {errs.max():.3f} | std {errs.std():.3f}")
print(f"=== ATE 2D: RMSE {np.sqrt((err2**2).mean()):.3f} | mean {err2.mean():.3f} | max {err2.max():.3f}")
# 机器可读行（供 eval 脚本自动追加 SUMMARY.md）
print(f"SUMMARY|ate2d={np.sqrt((err2**2).mean()):.4f}|ate3d={np.sqrt((errs**2).mean()):.4f}|len_est={len_est:.1f}|len_gt={len_gt:.1f}|poses={len(est_al)}")
np.savetxt(out_fn, np.column_stack([est_ts, est_al, np.zeros((len(est_al),4))]), fmt='%.6f')
print(f"保存: {out_fn}")

# ============ 画图 ============
base = os.path.splitext(out_fn)[0]
rel_t = est_ts - est_ts[0]

# 图1: 轨迹叠加 (xy 平面)
fig, ax = plt.subplots(figsize=(9, 8))
ax.plot(gt[:, 0], gt[:, 1], 'k-', lw=2.2, label='GT (GPSBase)')
ax.plot(est[:, 0], est[:, 1], 'r--', lw=1.3, alpha=0.8, label='Est (raw)')
ax.plot(est_al[:, 0], est_al[:, 1], 'b-', lw=1.5, label='Est (aligned)')
ax.scatter(est_al[0, 0], est_al[0, 1], c='b', marker='o', s=40, zorder=5)
ax.scatter(gt[0, 0], gt[0, 1], c='k', marker='s', s=40, zorder=5)
ax.set_aspect('equal', adjustable='box')
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.set_title('Trajectory comparison (XY)')
ax.legend(); ax.grid(True, alpha=0.3)
plt.tight_layout()
png1 = base + '_traj.png'
plt.savefig(png1, dpi=150); plt.close()
print(f"保存图: {png1}")

# 图2: 误差 vs 时间 + 2D 误差直方图
fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
axes[0].plot(rel_t, err2, 'b-', lw=1.2, label='2D error')
axes[0].plot(rel_t, errs, 'r--', lw=1.0, alpha=0.8, label='3D error')
axes[0].axhline(np.sqrt((err2**2).mean()), color='b', ls=':', lw=1, label=f"2D RMSE={np.sqrt((err2**2).mean()):.3f}m")
axes[0].set_xlabel('time (s)'); axes[0].set_ylabel('error (m)')
axes[0].set_title('ATE over time'); axes[0].legend(); axes[0].grid(True, alpha=0.3)
axes[1].hist(err2, bins=40, color='steelblue', edgecolor='white')
axes[1].set_xlabel('2D error (m)'); axes[1].set_ylabel('count')
axes[1].set_title('2D error distribution')
plt.tight_layout()
png2 = base + '_err.png'
plt.savefig(png2, dpi=150); plt.close()
print(f"保存图: {png2}")
