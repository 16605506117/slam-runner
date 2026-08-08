#!/bin/bash
# ============================================================
# FAST-LIVO2 one-click evaluation (water datasets W06/N03/H05)
# Usage: bash eval_fast_livo.sh <traj_bag> <dataset>
#   extracts /aft_mapped_to_init from bag -> est.tum (t zeroed),
#   eval_2d.py (2D Umeyama) + evo traj/APE/RPE full plot set
# ============================================================
BAG=${1:?usage: eval_fast_livo.sh <traj_bag> <dataset>}
DS=${2:?usage: eval_fast_livo.sh <traj_bag> <dataset>}
SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

source /opt/ros/noetic/setup.bash
source /hy-tmp/catkin_ws/devel/setup.bash
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
source /usr/local/miniconda3/etc/profile.d/conda.sh
conda activate base

if [ ! -f "$BAG" ]; then
  echo "[!] no bag: $BAG"; exit 1
fi

WORK=/tmp/fl2_eval_$(date +%H%M%S)
mkdir -p "$WORK" && cd "$WORK"
echo "[*] workdir: $WORK"

echo "[*] 1/5 extract trajectory from bag"
evo_traj bag "$BAG" /aft_mapped_to_init --save_as_tum 2>&1 | tail -1
EST=$(ls *.tum | head -1)
python3 - "$EST" "$WORK/est.tum" <<'PYEOF'
import sys
import numpy as np
d = np.loadtxt(sys.argv[1])
d[:, 0] -= d[0, 0]
np.savetxt(sys.argv[2], d, fmt='%.6f')
print(f"    est: {len(d)} poses -> {sys.argv[2]} (t zeroed)")
PYEOF

echo "[*] 2/5 prepare gt"
GT_ABS=$(ls /hy-tmp/datasets/water/${DS}*_gt.tum 2>/dev/null | head -1)
GT_REL=$(ls /hy-tmp/datasets/water/${DS}*_gt_rel.tum 2>/dev/null | head -1)
if [ -z "$GT_REL" ] && [ -n "$GT_ABS" ]; then
  python3 - "$GT_ABS" "$WORK/gt_rel.tum" <<'PYEOF'
import sys
import numpy as np
d = np.loadtxt(sys.argv[1])
d[:, 0] -= d[0, 0]
np.savetxt(sys.argv[2], d, fmt='%.6f')
print(f"    gt_rel: {len(d)} poses -> {sys.argv[2]}")
PYEOF
  GT_REL="$WORK/gt_rel.tum"
fi
if [ -z "$GT_REL" ]; then
  echo "[!] no gt for $DS under /hy-tmp/datasets/water/, skip eval"; exit 0
fi

echo "[*] 3/5 eval_2d.py (2D Umeyama)"
E2D=$(python3 "$SELF_DIR/eval_2d.py" "$WORK/est.tum" "$GT_REL" "$WORK/est_aligned.tum")
echo "$E2D"
ATE2D=$(echo "$E2D" | grep '^SUMMARY' | grep -oP 'ate2d=\K[0-9.]+')
ATE3D=$(echo "$E2D" | grep '^SUMMARY' | grep -oP 'ate3d=\K[0-9.]+')
LEN_EST=$(echo "$E2D" | grep '^SUMMARY' | grep -oP 'len_est=\K[0-9.]+')
LEN_GT=$(echo "$E2D" | grep '^SUMMARY' | grep -oP 'len_gt=\K[0-9.]+')
POSES=$(echo "$E2D" | grep '^SUMMARY' | grep -oP 'poses=\K[0-9]+')

echo "[*] 4/5 evo full plot set (traj_cmp / APE / RPE)"
evo_traj tum "$GT_REL" "$WORK/est_aligned.tum" --ref "$GT_REL" --align_origin -p --plot_mode xy --save_plot "$WORK/traj_cmp_xy" 2>&1 | tail -1 || true
evo_traj tum "$GT_REL" "$WORK/est_aligned.tum" --ref "$GT_REL" --align_origin -p --plot_mode xyz --save_plot "$WORK/traj_cmp_xyz" 2>&1 | tail -1 || true
APE_OUT=$(evo_ape tum "$GT_REL" "$WORK/est_aligned.tum" --pose_relation trans_part --plot --plot_mode xy --save_results "$WORK/ape.zip" --save_plot "$WORK/ape_plot" 2>&1) || true
echo "$APE_OUT" | grep -iE "rmse|mean|median|max|std" | head -6
APE=$(echo "$APE_OUT" | grep -i "rmse" | awk '{print $2}')
RPE_OUT=$(evo_rpe tum "$GT_REL" "$WORK/est_aligned.tum" --pose_relation trans_part --delta 1 --delta_unit m --plot --plot_mode xy --save_results "$WORK/rpe.zip" --save_plot "$WORK/rpe_plot" 2>&1) || true
echo "$RPE_OUT" | grep -iE "rmse|mean|median|max|std" | head -6
RPE=$(echo "$RPE_OUT" | grep -i "rmse" | awk '{print $2}')

echo "[*] 5/5 archive + summary"
DEST="/hy-tmp/results/fast_livo2/$(date +%F)/$(date +%H-%M)/$DS/eval"
mkdir -p "$DEST"
cp "$WORK"/*.tum "$WORK"/*.zip "$WORK"/*.png "$DEST/" 2>/dev/null || true
bash "$SELF_DIR/update_summary.sh" "$(echo "$DS" | tr '[:lower:]' '[:upper:]')" "| $(date +%F) | FAST-LIVO2 | $ATE2D | $ATE3D | $RPE | $POSES | len ${LEN_EST}/${LEN_GT}m |"
echo ""
echo "[+] Done! results in: $DEST"
ls -la "$DEST" | awk '{print $5, $9}'
