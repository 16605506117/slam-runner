#!/bin/bash
# ============================================================
# 水上数据集真值生成 (GPSBase RTK -> TUM)
# 用法: bash gen_water_gt.sh <数据集目录名>
#   bash gen_water_gt.sh W06_2_Sequence_57_115   -> w06_gt.tum + w06_gt_rel.tum
#   bash gen_water_gt.sh N03_4_Sequence_440_523   -> n03_gt.tum + n03_gt_rel.tum
# 输出: /hy-tmp/datasets/water/<短名>_gt.tum (绝对 unix 时间) + _gt_rel.tum (相对 0 起)
# ============================================================
set -e
SEQ=${1:?用法: gen_water_gt.sh <数据集目录名>}
BASE=/hy-tmp/datasets/water/$SEQ
SHORT=$(echo "$SEQ" | cut -d_ -f1)   # W06 / N03 / H05

if [ ! -f "$BASE/GPSBase.csv" ] || [ ! -f "$BASE/IMU.csv" ]; then
  echo "[!] $BASE 缺少 GPSBase.csv 或 IMU.csv"; exit 1
fi

python3 - "$BASE" "$SHORT" <<'PYEOF'
import sys
import numpy as np
base, short = sys.argv[1], sys.argv[2]

# base_unix: IMU.csv 里 t=0 对应的 unix 秒
imu = open(f'{base}/IMU.csv').read().strip().split('\n')
first = imu[0].split(',')
base_unix = float(first[-1]) - float(first[0])

# GPSBase: col1=t(IMU轴), col2=x(东), col3=y(北), col10=z(高)
gt = np.loadtxt(f'{base}/GPSBase.csv', delimiter=',')
t, x, y = gt[:, 0], gt[:, 1], gt[:, 2]
z = gt[:, 9] - gt[0, 9]
n = len(t)

abs_fn = f'/hy-tmp/datasets/water/{short}_gt.tum'
rel_fn = f'/hy-tmp/datasets/water/{short}_gt_rel.tum'
with open(abs_fn, 'w') as f:
    for i in range(n):
        f.write(f"{base_unix + t[i]:.6f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f} 0 0 0 1\n")
t0 = t[0]
with open(rel_fn, 'w') as f:
    for i in range(n):
        f.write(f"{t[i] - t0:.6f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f} 0 0 0 1\n")
print(f"{short}: GT {n} 行 | base_unix={base_unix:.6f} | 首帧 {t0:.3f}s")
print(f"  -> {abs_fn}")
print(f"  -> {rel_fn}")
PYEOF
