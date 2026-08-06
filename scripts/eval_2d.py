#!/usr/bin/env python3
"""直线轨迹 2D 对齐评估: 时间最近邻匹配 -> 2D Umeyama -> ATE"""
import numpy as np

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

gt_ts, gt = load('w06_gt_rel.tum')
est_ts, est = load('w06_fastlio_est.tum')

# 时间最近邻匹配对 (只匹配时间重叠区间)
idx = [np.argmin(np.abs(gt_ts - t)) for t in est_ts]
R, t = umeyama2d(est, gt[idx])
est_al = est.copy()
est_al[:, :2] = (R @ est[:, :2].T).T + t
z_off = np.median(gt[idx][:, 2] - est_al[:, 2])
est_al[:, 2] += z_off

# 误差 (最近邻)
errs = np.array([np.linalg.norm(est_al[i] - gt[np.argmin(np.abs(gt_ts - est_ts[i]))]) for i in range(len(est_al))])
err2 = np.array([np.linalg.norm(est_al[i, :2] - gt[np.argmin(np.abs(gt_ts - est_ts[i])), :2]) for i in range(len(est_al))])
print(f"匹配 {len(est_al)} 帧 | 对齐角 {np.degrees(np.arctan2(R[1,0],R[0,0])):.1f}deg 平移({t[0]:.2f},{t[1]:.2f})")
print(f"轨迹长度: est {np.sum(np.linalg.norm(np.diff(est[:,:2],axis=0),axis=1)):.1f}m | gt {np.sum(np.linalg.norm(np.diff(gt[:,:2],axis=0),axis=1)):.1f}m")
print(f"=== ATE 3D: RMSE {np.sqrt((errs**2).mean()):.3f} | mean {errs.mean():.3f} | median {np.median(errs):.3f} | max {errs.max():.3f} | std {errs.std():.3f}")
print(f"=== ATE 2D: RMSE {np.sqrt((err2**2).mean()):.3f} | mean {err2.mean():.3f} | max {err2.max():.3f}")
np.savetxt('w06_fastlio_est_aligned.tum', np.column_stack([est_ts, est_al, np.zeros((len(est_al),4))]), fmt='%.6f')
print("保存: w06_fastlio_est_aligned.tum")
