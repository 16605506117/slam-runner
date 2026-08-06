#!/bin/bash
# ============================================================
# LIO-SAM 一键评估 (KITTI + 水上通用)
# 输入: 轨迹 bag (rosbag record /lio_sam/mapping/odometry)
# 用法:
#   bash eval_lio_sam.sh [traj.bag] [数据集名] [源bag]
#   KITTI : bash eval_lio_sam.sh traj.bag kitti_07 /hy-tmp/datasets/road/kitti/bags/kitti_xxx.bag
#   水上  : bash eval_lio_sam.sh traj.bag w06  /hy-tmp/datasets/water/w06.bag
#   KITTI 走 evo APE/RPE (SE3 Umeyama); 水上走 eval_2d.py (2D Umeyama)
# ============================================================
set -e
BAG=${1:-/hy-tmp/results/lio_sam/traj.bag}
DS=${2:-kitti_07}
SRC_BAG=${3:-/hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag}
TOPIC=/lio_sam/mapping/odometry
SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

source /opt/ros/noetic/setup.bash
source /hy-tmp/catkin_ws/devel/setup.bash
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
source /usr/local/miniconda3/etc/profile.d/conda.sh
conda activate base

if [ ! -f "$BAG" ]; then
  echo "[!] 找不到轨迹 bag: $BAG"
  echo "    跑之前先: rosbag record /lio_sam/mapping/odometry -O $BAG"
  exit 1
fi

WORK=/tmp/ls_eval_$(date +%H%M%S)
mkdir -p "$WORK"
echo "[*] 工作目录: $WORK"
cd "$WORK"

echo "[*] 1/4 提取轨迹 bag -> est.tum"
evo_traj bag "$BAG" "$TOPIC" --save_as_tum 2>&1 | tail -3
EST=$(ls "$WORK"/*.tum 2>/dev/null | head -1)
if [ -z "$EST" ]; then
  echo "[!] evo_traj failed, no tum generated"
  exit 1
fi
echo "    est: $EST ($(wc -l < "$EST") poses)"

if [ "$DS" = "kitti_07" ]; then
  # ============ KITTI 分支: evo APE/RPE ============
  GT_RAW=/hy-tmp/datasets/road/kitti/poses/07.txt
  echo "[*] 2/4 生成 GT (KITTI poses -> TUM, 时间轴对齐 bag)"
  python3 - "$SRC_BAG" "$GT_RAW" "$WORK/gt.tum" <<'PYEOF'
import sys, rosbag
import numpy as np
from scipy.spatial.transform import Rotation as R

bag_path, gt_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
ts = []
bag = rosbag.Bag(bag_path)
for topic, msg, t in bag.read_messages(topics=['/points_raw']):
    ts.append(msg.header.stamp.to_sec())
bag.close()
gt = np.loadtxt(gt_path)
n = min(len(ts), len(gt))
print(f"    bag 帧 {len(ts)}, GT 行 {len(gt)}, 取 {n}")
with open(out_path, 'w') as f:
    for i in range(n):
        T = gt[i].reshape(3, 4)
        q = R.from_matrix(T[:, :3]).as_quat()  # xyzw
        f.write(f"{ts[i]:.6f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")
print(f"    gt: {out_path} ({n} poses)")
PYEOF

  echo "[*] 3/4 计算 ATE (SE3 Umeyama 对齐)"
  evo_ape tum "$WORK/gt.tum" "$EST" -a --plot --plot_mode xyz \
    --save_results "$WORK/ape.zip" --save_plot "$WORK/ape_plot" 2>&1 | grep -iE "rmse|mean|median|max|std" | head -8

  echo "[*] 4/4 计算 RPE (平移, delta=1m)"
  evo_rpe tum "$WORK/gt.tum" "$EST" -a --delta 1 --delta_unit m --plot --plot_mode xyz \
    --save_results "$WORK/rpe.zip" --save_plot "$WORK/rpe_plot" 2>&1 | grep -iE "rmse|mean|median|max|std" | head -8

  DEST="/hy-tmp/results/lio_sam/$(date +%F)/$(date +%H-%M)/$DS/eval"
  mkdir -p "$DEST"
  cp "$EST" "$DEST/est.tum"
  cp "$WORK/gt.tum" "$DEST/gt.tum"
  cp "$WORK"/*.zip "$WORK"/*.png "$DEST/" 2>/dev/null || true
  cp "$BAG" "$DEST/traj.bag" 2>/dev/null || true
else
  # ============ 水上分支: eval_2d.py ============
  echo "[*] 2/4 est 时间归零 (与 gt_rel 对齐)"
  python3 - "$EST" "$WORK/est_rel.tum" <<'PYEOF'
import sys
import numpy as np
d = np.loadtxt(sys.argv[1])
d[:, 0] -= d[0, 0]
np.savetxt(sys.argv[2], d, fmt='%.6f')
print(f"    est_rel: {len(d)} poses, t0={d[0,0]:.4f}")
PYEOF

  echo "[*] 3/4 准备 GT (GPSBase RTK 真值)"
  GT_REL=$(ls /hy-tmp/datasets/water/${DS}*_gt_rel.tum 2>/dev/null | head -1)
  if [ -z "$GT_REL" ]; then
    GT_ABS=$(ls /hy-tmp/datasets/water/${DS}*_gt.tum 2>/dev/null | head -1)
    if [ -n "$GT_ABS" ]; then
      python3 - "$GT_ABS" "$WORK/gt_rel.tum" <<'PYEOF'
import sys
import numpy as np
d = np.loadtxt(sys.argv[1])
d[:, 0] -= d[0, 0]
np.savetxt(sys.argv[2], d, fmt='%.6f')
print(f"    gt_rel: {len(d)} poses (from abs)")
PYEOF
      GT_REL="$WORK/gt_rel.tum"
    fi
  fi
  if [ -z "$GT_REL" ]; then
    echo "[!] no gt file for $DS under /hy-tmp/datasets/water/, skip eval"
    echo "    先生成: bash $(dirname "$SELF_DIR")/scripts/gen_water_gt.sh <数据集目录名>"
    exit 0
  fi

  echo "[*] 4/4 计算 ATE 2D (eval_2d.py)"
  python3 "$SELF_DIR/eval_2d.py" "$WORK/est_rel.tum" "$GT_REL" "$WORK/est_aligned.tum"

  DEST="/hy-tmp/results/lio_sam/$(date +%F)/$(date +%H-%M)/$DS/eval"
  mkdir -p "$DEST"
  cp "$EST" "$DEST/est.tum"
  cp "$WORK/est_rel.tum" "$DEST/est_rel.tum"
  cp "$WORK/est_aligned.tum" "$DEST/est_aligned.tum"
  cp "$GT_REL" "$DEST/gt_rel.tum"
  cp "$WORK"/*.png "$DEST/" 2>/dev/null || true
  cp "$BAG" "$DEST/traj.bag" 2>/dev/null || true
fi

echo ""
echo "[+] 完成! 结果在: $DEST"
ls -la "$DEST"
rm -rf "$WORK"
