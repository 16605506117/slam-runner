#!/bin/bash
# ============================================================
# FAST-LIO one-click evaluation
# Usage: bash eval_fast_lio.sh <dataset> [bag]
#   kitti_07 : mat_out -> TUM via kitti_est_gt_to_tum.py, then evo APE/RPE
#   w06/n03  : mat_out -> est.tum (relative time), gt -> rel, eval_2d.py
# ============================================================
DS=${1:-kitti_07}
BAG=${2:-/hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag}
MAT_OUT=/hy-tmp/catkin_ws/src/FAST_LIO-main/Log/mat_out.txt
# self-locating: assume sibling scripts in same dir
SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

source /opt/ros/noetic/setup.bash
source /hy-tmp/catkin_ws/devel/setup.bash
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
source /usr/local/miniconda3/etc/profile.d/conda.sh
conda activate base

if [ ! -f "$MAT_OUT" ]; then
  echo "[!] no mat_out.txt: $MAT_OUT (run FAST-LIO first, kill -INT to save)"
  exit 1
fi

WORK=/tmp/fl_eval_$(date +%H%M%S)
mkdir -p "$WORK"
cd "$WORK"
echo "[*] workdir: $WORK"

if [ "$DS" = "kitti_07" ]; then
  echo "[*] KITTI: mat_out -> TUM"
  sed -e "s|^BAG=.*|BAG=\"$BAG\"|" \
      -e "s|^DST_GT=.*|DST_GT=\"$WORK/gt07.tum\"|" \
      -e "s|^DST_EST=.*|DST_EST=\"$WORK/est_kitti.tum\"|" \
      "$SELF_DIR/kitti_est_gt_to_tum.py" > conv.py
  python3 conv.py
  echo "[*] ATE (SE3 Umeyama)..."
  evo_ape tum "$WORK/gt07.tum" "$WORK/est_kitti.tum" -a --plot --plot_mode xyz \
    --save_results "$WORK/ape.zip" --save_plot "$WORK/ape_plot" 2>&1 | grep -iE "rmse|mean|median|max|std" | head -6
  echo "[*] RPE (trans, delta 1m)..."
  evo_rpe tum "$WORK/gt07.tum" "$WORK/est_kitti.tum" -a --delta 1 --delta_unit m --plot --plot_mode xyz \
    --save_results "$WORK/rpe.zip" --save_plot "$WORK/rpe_plot" 2>&1 | grep -iE "rmse|mean|median|max|std" | head -6
else
  echo "[*] WATER: mat_out -> est.tum (relative time)"
  python3 - "$MAT_OUT" "$WORK/est.tum" <<'PYEOF'
import sys
import numpy as np
src, dst = sys.argv[1], sys.argv[2]
rows = []
with open(src) as f:
    for ln in f:
        v = ln.split()
        if len(v) >= 7:
            t, r, p, y, x, yy, z = (float(v[0]), float(v[1]), float(v[2]),
                                    float(v[3]), float(v[4]), float(v[5]), float(v[6]))
            r, p, y = np.deg2rad([r, p, y])
            cr, sr, cp, sp, cy, sy = (np.cos(r/2), np.sin(r/2), np.cos(p/2),
                                      np.sin(p/2), np.cos(y/2), np.sin(y/2))
            qw = cr*cp*cy + sr*sp*sy
            qx = sr*cp*cy - cr*sp*sy
            qy = cr*sp*cy + sr*cp*sy
            qz = cr*cp*sy - sr*sp*cy
            rows.append((t, x, yy, z, qx, qy, qz, qw))
with open(dst, 'w') as f:
    for t, x, y, z, qx, qy, qz, qw in rows:
        f.write(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
print(f"    est: {len(rows)} poses -> {dst}")
PYEOF

  echo "[*] WATER: prepare gt"
  GT_ABS=$(ls /hy-tmp/datasets/water/${DS}*_gt.tum 2>/dev/null | head -1)
  GT_REL=$(ls /hy-tmp/datasets/water/${DS}*_gt_rel.tum 2>/dev/null | head -1)
  if [ -z "$GT_REL" ] && [ -n "$GT_ABS" ]; then
    python3 - "$GT_ABS" "$WORK/gt_rel.tum" <<'PYEOF'
import sys
import numpy as np
src, dst = sys.argv[1], sys.argv[2]
d = np.loadtxt(src)
d[:, 0] -= d[0, 0]
np.savetxt(dst, d, fmt='%.6f')
print(f"    gt_rel: {len(d)} poses -> {dst}")
PYEOF
    GT_REL="$WORK/gt_rel.tum"
  fi
  if [ -z "$GT_REL" ]; then
    echo "[!] no gt file for $DS under /hy-tmp/datasets/water/, skip eval"
    exit 0
  fi
  cp "$WORK/est.tum" w06_fastlio_est.tum
  cp "$GT_REL" w06_gt_rel.tum
  echo "[*] WATER: eval_2d.py"
  python3 "$SELF_DIR/eval_2d.py"
fi

DEST="/hy-tmp/results/fast_lio/$(date +%F)/$(date +%H-%M)/$DS/eval"
mkdir -p "$DEST"
cp "$WORK"/*.tum "$DEST/" 2>/dev/null || true
cp "$WORK"/*.zip "$WORK"/*.png "$DEST/" 2>/dev/null || true
echo ""
echo "[+] Done! results in: $DEST"
ls -la "$DEST"
