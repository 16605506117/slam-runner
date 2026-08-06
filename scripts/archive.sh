#!/bin/bash
# ============================================================
# 结果自动归档脚本 | Result archive script
# 用法: bash archive.sh <算法> <数据集名> [额外文件...]
#   <算法>  : lio_sam | fast_lio
#   <数据集>: 如 kitti_07 / w06 / n03 ...
# 效果: 产物复制到 $RESULT_ROOT/<算法>/<日期>/<时间>/<数据集>/
#       同一天多次跑自动分到不同时间文件夹，互不覆盖
# 例: bash archive.sh lio_sam kitti_07
#     bash archive.sh fast_lio w06
#     bash archive.sh lio_sam kitti_07 /hy-tmp/lio_sam_traj.bag
# ============================================================
set -e
ALGO=$1; DS=$2
RESULT_ROOT=${RESULT_ROOT:-/hy-tmp/results}
if [ -z "$ALGO" ] || [ -z "$DS" ]; then
  echo "用法: archive.sh <算法: lio_sam|fast_lio> <数据集名> [额外文件...]"
  exit 1
fi
DEST="$RESULT_ROOT/$ALGO/$(date +%F)/$(date +%H-%M)/$DS"
mkdir -p "$DEST"
COPIED=0

if [ "$ALGO" = "lio_sam" ]; then
  if [ -d "$RESULT_ROOT/lio_sam/pcd" ]; then
    cp -r "$RESULT_ROOT/lio_sam/pcd" "$DEST/pcd"; COPIED=1
    echo "[OK] 地图 pcd -> $DEST/pcd"
  else
    echo "[!] 没有新 pcd（先跑 LIO-SAM，Ctrl-C 退出会自动存地图）"
  fi
fi

if [ "$ALGO" = "fast_lio" ]; then
  if [ -f /hy-tmp/catkin_ws/src/FAST_LIO-main/Log/mat_out.txt ]; then
    cp /hy-tmp/catkin_ws/src/FAST_LIO-main/Log/mat_out.txt "$DEST/mat_out.txt"; COPIED=1
    echo "[OK] mat_out.txt -> $DEST/mat_out.txt"
  else
    echo "[!] 没有 mat_out.txt（FAST-LIO 跑完要 kill -INT 才落盘）"
  fi
fi

for f in "${@:3}"; do
  if [ -e "$f" ]; then cp -r "$f" "$DEST/"; COPIED=1; echo "[OK] $f"; fi
done

if [ $COPIED -eq 0 ]; then echo "[!] 没找到产物，检查一下"; fi
echo "[完成] 结果已归档: $DEST"
