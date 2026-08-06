#!/bin/bash
# ============================================================
# LIO-SAM 一键评估脚本 (KITTI)
# 输入: 轨迹 bag (rosbag record /lio_sam/mapping/odometry)
# 输出: est.tum / gt.tum / ATE / RPE 指标 + 图, 存到结果目录
# 用法:
#   bash eval_lio_sam.sh [traj.bag] [数据集名]
# 示例:
#   bash eval_lio_sam.sh /hy-tmp/results/lio_sam/traj.bag kitti_07
# 前置: 已 record 轨迹 bag; evo 已装 (pip install evo)
# ============================================================
set -e
BAG=${1:-/hy-tmp/results/lio_sam/traj.bag}
DS=${2:-kitti_07}
KITTI_BAG=${3:-/hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag}
GT_RAW=/hy-tmp/datasets/road/kitti/poses/07.txt
TOPIC=/lio_sam/mapping/odometry

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

echo "[*] 1/4 提取轨迹 bag -> est.tum"
cd "$WORK"
evo_traj bag "$BAG" "$TOPIC" --save_as_tum 2>&1 | tail -3
EST=$(ls "$WORK"/*.tum 2>/dev/null | head -1)
if [ -z "$EST" ]; then
  echo "[!] evo_traj failed, no tum generated"
  exit 1
fi
echo "    est: $EST ($(wc -l < "$EST") poses)"

echo "[*] 2/4 生成 GT (KITTI poses -> TUM, 时间轴对齐 bag)"
python3 - "$KITTI_BAG" "$GT_RAW" "$WORK/gt.tum" <<'PYEOF'
import sys, rosbag
import numpy as np
from scipy.spatial.transform import Rotation as R

bag_path, gt_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]

# 读 bag 里每帧点云的时间戳作为时间轴
ts = []
bag = rosbag.Bag(bag_path)
for topic, msg, t in bag.read_messages(topics=['/points_raw']):
    ts.append(msg.header.stamp.to_sec())
bag.close()

# 读 KITTI GT (N x 12, 3x4 位姿)
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

# 归档到结果目录 (与 archive.sh 同款结构)
DEST="/hy-tmp/results/lio_sam/$(date +%F)/$(date +%H-%M)/$DS/eval"
mkdir -p "$DEST"
cp "$EST" "$DEST/est.tum"
cp "$WORK/gt.tum" "$DEST/gt.tum"
cp "$WORK"/*.zip "$WORK"/*.png "$DEST/" 2>/dev/null || true
cp "$BAG" "$DEST/traj.bag" 2>/dev/null || true
echo ""
echo "[+] 完成! 结果在: $DEST"
ls -la "$DEST"
rm -rf "$WORK"
