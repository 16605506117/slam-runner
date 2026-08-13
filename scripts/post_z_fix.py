#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案 B1: 水面高度先验 - 后处理 z 去漂移
水面平台 z 应近似恒定(水面平面), 90度前视雷达无地面点导致 z 线性漂移
做法: 对 est.tum 的 z 做多项式去漂移(默认1阶), 输出修正后 tum, 打印修正前后 ATE
用法: python3 post_z_fix.py est.tum gt.tum [deg]
"""
import sys, os
import numpy as np

def load_tum(fn):
    ts, poses = [], []
    for ln in open(fn):
        ln = ln.strip()
        if not ln or ln.startswith('#'):
            continue
        v = ln.split()
        if len(v) < 8:
            continue
        ts.append(float(v[0]))
        poses.append([float(v[1]), float(v[2]), float(v[3]),
                      float(v[4]), float(v[5]), float(v[6]), float(v[7])])
    return np.array(ts), np.array(poses)

def save_tum(fn, ts, poses):
    with open(fn, 'w') as f:
        for i in range(len(ts)):
            f.write(f"{ts[i]:.6f} {poses[i,0]:.6f} {poses[i,1]:.6f} {poses[i,2]:.6f} "
                    f"{poses[i,3]:.6f} {poses[i,4]:.6f} {poses[i,5]:.6f} {poses[i,6]:.6f}\n")

def eval_ate(est_fn, gt_fn):
    """调用 eval_2d.py 并解析 SUMMARY 行"""
    import subprocess
    r = subprocess.run([sys.executable, '/hy-tmp/slam-runner/scripts/eval_2d.py',
                        est_fn, gt_fn], capture_output=True, text=True)
    out = r.stdout
    for ln in out.splitlines():
        if ln.startswith('SUMMARY'):
            parts = dict(kv.split('=') for kv in ln.strip().split('|')[1:])
            return parts.get('ate2d', '?'), parts.get('ate3d', '?'), parts.get('poses', '?')
    return '?', '?', '?'

if __name__ == '__main__':
    est_fn = sys.argv[1]
    gt_fn = sys.argv[2]
    deg = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    base = os.path.splitext(est_fn)[0]
    out_fn = base + '_zfix.tum'

    ts, poses = load_tum(est_fn)
    t0 = ts - ts[0]
    z = poses[:, 2]

    # 去漂移: z_fix = z - polyfit(t0, z, deg)
    coeff = np.polyfit(t0, z, deg)
    z_drift = np.polyval(coeff, t0)
    z_fix = z - z_drift
    poses[:, 2] = z_fix
    save_tum(out_fn, ts, poses)

    print(f"[*] {os.path.basename(est_fn)}: {len(ts)} poses, z 范围 [{z.min():.3f}, {z.max():.3f}]")
    print(f"    z 漂移拟合(deg={deg}): {['%.4f' % c for c in coeff]}")
    print(f"    z 修正后范围: [{z_fix.min():.3f}, {z_fix.max():.3f}]")

    a2_b, a3_b, n = eval_ate(est_fn, gt_fn)
    a2_a, a3_a, _ = eval_ate(out_fn, gt_fn)
    print(f"[*] ATE 修正前: 2D={a2_b} 3D={a3_b} ({n} poses)")
    print(f"[*] ATE 修正后: 2D={a2_a} 3D={a3_a}")
    print(f"[+] 输出: {out_fn}")
