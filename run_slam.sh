#!/bin/bash
# ============================================================
# Universal SLAM one-click runner (FAST-LIO / LIO-SAM)
# Usage:
#   bash run_slam.sh <algo> <launch> <bag> <dataset> [options]
#   algo: lio_sam | fast_lio
# Options:
#   --rviz       enable rviz (needs Xvfb :1; default off)
#   --eval       run evaluation after finish
#   --clock      add --clock to rosbag play (required for sim-time launch)
#   --rate X     bag play speed (default 1.0)
#   --no-record  skip trajectory recording
#   --auto-oss   auto-upload archived result to OSS after finish
# Examples:
#   bash run_slam.sh lio_sam run_kitti_simtime.launch /hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag kitti_07 --clock --eval
#   bash run_slam.sh fast_lio mapping_velodyne_kitti.launch /hy-tmp/datasets/road/kitti/bags/kitti_2011_09_30_drive_0027_synced.bag kitti_07 --eval
#   bash run_slam.sh fast_lio mapping_water.launch /hy-tmp/datasets/water/w06.bag w06 --eval
# Safe for disconnection:
#   setsid nohup bash run_slam.sh ... > /tmp/run_slam.log 2>&1 &
# ============================================================

ALGO=${1:-}; LAUNCH=${2:-}; BAG=${3:-}; DS=${4:-}
if [ -z "$ALGO" ] || [ -z "$LAUNCH" ] || [ -z "$BAG" ] || [ -z "$DS" ]; then
  echo "Usage: run_slam.sh <lio_sam|fast_lio> <launch> <bag> <dataset> [--rviz] [--eval] [--clock] [--rate X] [--no-record] [--auto-oss]"
  exit 1
fi

RVIZ=0; EVAL=0; CLOCK=0; RATE=1.0; RECORD=1; AUTO_OSS=0
i=5
while [ $i -le $# ]; do
  case "${!i}" in
    --rviz) RVIZ=1 ;;
    --eval) EVAL=1 ;;
    --clock) CLOCK=1 ;;
    --rate) i=$((i+1)); RATE="${!i}" ;;
    --no-record) RECORD=0 ;;
    --auto-oss) AUTO_OSS=1 ;;
    *) echo "[!] unknown option: ${!i}"; exit 1 ;;
  esac
  i=$((i+1))
done

source /opt/ros/noetic/setup.bash
source /hy-tmp/catkin_ws/devel/setup.bash
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages

# self-locating: works from anywhere (repo root or server)
TOOLS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SCRIPTS="$TOOLS_DIR/scripts"
DATA_ROOT=${DATA_ROOT:-/hy-tmp/datasets}
RESULT_ROOT=${RESULT_ROOT:-/hy-tmp/results}

if [ "$ALGO" = "lio_sam" ]; then
  LAUNCH_DIR=/hy-tmp/catkin_ws/src/LIO-SAM-master/launch
  REC_TOPIC=/lio_sam/mapping/odometry
  WAIT_S=25
  KILL_PAT="roslaunch lio_sam"
elif [ "$ALGO" = "fast_lio" ]; then
  LAUNCH_DIR=/hy-tmp/catkin_ws/src/FAST_LIO-main/launch
  REC_TOPIC=/Odometry
  WAIT_S=10
  KILL_PAT=fastlio_mapping
else
  echo "[!] algo must be lio_sam or fast_lio"; exit 1
fi

if [ ! -f "$LAUNCH_DIR/$LAUNCH" ]; then
  echo "[!] launch not found: $LAUNCH_DIR/$LAUNCH"; exit 1
fi
if [ ! -f "$BAG" ]; then
  echo "[!] bag not found: $BAG"; exit 1
fi

# sim-time guard: launch sets /use_sim_time=true but --clock not given -> abort early
if [ $CLOCK -eq 0 ]; then
  if grep -q "use_sim_time" "$LAUNCH_DIR/$LAUNCH" && grep -q "true" "$LAUNCH_DIR/$LAUNCH"; then
    echo "[!] $LAUNCH 是 sim-time 模式 (use_sim_time=true)，必须加 --clock！"
    echo "    正确用法: bash run_slam.sh $ALGO $LAUNCH $BAG $DS --clock [--eval] [--rviz]"
    exit 1
  fi
fi

LOG_DIR=/tmp/run_slam_$(date +%H%M%S)
mkdir -p "$LOG_DIR"
START_TIME=$(date '+%F %T %Z')
REC_BAG=$RESULT_ROOT/$ALGO/traj_$(date +%H%M%S).bag
echo "[*] algo=$ALGO launch=$LAUNCH ds=$DS"
echo "[*] bag=$BAG rate=$RATE clock=$CLOCK rviz=$RVIZ eval=$EVAL record=$RECORD auto_oss=$AUTO_OSS"
echo "[*] logs: $LOG_DIR"

# 1. cleanup
echo "[1/7] cleaning old processes..."
pkill -f "rosbag play" 2>/dev/null
pkill -f "rosbag record" 2>/dev/null
pkill -f "roslaunch" 2>/dev/null
pkill -f "rosmaster" 2>/dev/null
pkill -f "fastlio_mapping" 2>/dev/null
pkill -f "laserMapping" 2>/dev/null
sleep 3

# 2. start SLAM
echo "[2/7] starting $ALGO ($LAUNCH)..."
RV_ARG="rviz:=false"
LAUNCH_PREFIX=""
[ $RVIZ -eq 1 ] && { RV_ARG="rviz:=true"; export DISPLAY=:1 VGL_DISPLAY=egl; LAUNCH_PREFIX="vglrun"; }
setsid nohup bash -c "source /opt/ros/noetic/setup.bash && source /hy-tmp/catkin_ws/devel/setup.bash && $LAUNCH_PREFIX roslaunch $ALGO $LAUNCH $RV_ARG" > "$LOG_DIR/slam.log" 2>&1 &
echo "    started, waiting ${WAIT_S}s..."
sleep $WAIT_S
if ! pgrep -f "$KILL_PAT" > /dev/null; then
  echo "[!] SLAM did not start. log tail:"
  tail -20 "$LOG_DIR/slam.log"
  exit 1
fi
echo "    [OK] $ALGO running"

# 3. record
if [ $RECORD -eq 1 ]; then
  echo "[3/7] recording $REC_TOPIC -> $REC_BAG"
  setsid nohup rosbag record "$REC_TOPIC" -O "$REC_BAG" > "$LOG_DIR/record.log" 2>&1 &
  sleep 3
  if [ ! -s "$REC_BAG" ]; then
    echo "[!] 警告: $REC_BAG 为空/未创建 (sim-time 下等不到 /clock 会这样；确认已加 --clock)"
  fi
else
  echo "[3/7] recording skipped"
fi

# 4. play bag
echo "[4/7] playing bag..."
PLAY_OPTS=""
[ $CLOCK -eq 1 ] && PLAY_OPTS="$PLAY_OPTS --clock"
[ "$RATE" != "1.0" ] && PLAY_OPTS="$PLAY_OPTS --rate $RATE"
rosbag play $PLAY_OPTS "$BAG" 2>&1 | tee "$LOG_DIR/play.log"
echo "    [OK] bag finished"

# 5. stop record
if [ $RECORD -eq 1 ]; then
  echo "[5/7] stopping record..."
  pkill -INT -f "rosbag record"
  sleep 5
fi

# 6. stop SLAM (trigger save)
echo "[6/7] stopping $ALGO (SIGINT to save)..."
if [ "$ALGO" = "lio_sam" ]; then
  pkill -INT -f "roslaunch lio_sam"
  sleep 20
else
  pkill -INT -f "fastlio_mapping"
  sleep 5
  pkill -f "roslaunch fast_lio" 2>/dev/null
  sleep 3
fi

# 7. archive + eval
echo "[7/7] archiving..."
ARCH_OUT=$(RESULT_ROOT=$RESULT_ROOT bash "$SCRIPTS/archive.sh" "$ALGO" "$DS" "$REC_BAG" 2>&1 | tee "$LOG_DIR/archive.log")
echo "$ARCH_OUT"
DEST=$(echo "$ARCH_OUT" | grep -oP '结果已归档: \K.*' | tail -1)
if [ -n "$DEST" ] && [ -d "$DEST" ]; then
  {
    echo "algo: $ALGO"
    echo "launch: $LAUNCH"
    echo "bag: $BAG"
    echo "dataset: $DS"
    echo "options: rviz=$RVIZ eval=$EVAL clock=$CLOCK rate=$RATE record=$RECORD auto_oss=$AUTO_OSS"
    echo "start: $START_TIME"
    echo "end: $(date '+%F %T %Z')"
    echo "git_commit: $(cd "$TOOLS_DIR" && git rev-parse --short HEAD 2>/dev/null || echo n/a)"
  } > "$DEST/RUN_INFO.txt"
  echo "[OK] RUN_INFO.txt -> $DEST"
fi

if [ $EVAL -eq 1 ]; then
  echo "    evaluating..."
  if [ "$ALGO" = "lio_sam" ]; then
    bash "$SCRIPTS/eval_lio_sam.sh" "$REC_BAG" "$DS" "$BAG" || echo "[!] eval_lio_sam failed"
  else
    bash "$SCRIPTS/eval_fast_lio.sh" "$DS" "$BAG" || echo "[!] eval_fast_lio failed"
  fi
fi

if [ $AUTO_OSS -eq 1 ] && [ -n "$DEST" ] && [ -d "$DEST" ]; then
  echo "[8/8] auto-upload to OSS..."
  TS=$(date +%Y%m%d_%H%M%S)
  PKG=/tmp/auto_${ALGO}_${DS}_${TS}.tar.gz
  if tar czf "$PKG" -C "$(dirname "$DEST")" "$(basename "$DEST")" && \
     oss cp "$PKG" "oss://results/auto_${ALGO}_${DS}_${TS}.tar.gz" 2>&1 | tail -1; then
    rm -f "$PKG"
    echo "[OK] 结果已自动上传 OSS: oss://results/auto_${ALGO}_${DS}_${TS}.tar.gz"
  else
    echo "[!] auto-oss 失败（检查 oss login 状态）"
  fi
fi

echo ""
echo "[+] ALL DONE. logs in $LOG_DIR"
