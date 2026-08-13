#!/bin/bash
# ============================================================
# setup_instance.sh — 恒源云新实例一键配置脚本
# 用途：新开实例后，一条命令恢复 SLAM 全套环境（ROS + 算法 + 数据 + 工具）
# 兼容两种情况：
#   A. 从"备份镜像"创建的实例（已含 ROS/依赖，本脚本跳过安装，只拉数据）→ ~5 分钟
#   B. 全新裸机实例（无 ROS）→ 自动装 ROS + 拉数据 → ~20 分钟
#
# 三层备份策略（对应）：
#   1. 备份镜像 = 环境（ROS/依赖/配置）——控制台手动创建
#   2. OSS 个人数据 = catkin_ws 编译产物 + 水上数据 + 工具 + 结果 ——本脚本负责拉取
#   3. 百度网盘 = 原始数据集（W06/N03）——独立保险，不在此脚本范围
#
# 用法：bash setup_instance.sh [--with-ros-install]
#   --with-ros-install : 强制执行 ROS 安装（即使检测到 /opt/ros 存在也重装，一般不用）
# 日志：/tmp/setup_instance.log
# ============================================================
set -u
LOG=/tmp/setup_instance.log
exec > $LOG 2>&1

echo "=============================================="
echo "setup_instance.sh 开始 $(date '+%F %T %Z')"
echo "=============================================="

# ---------- [0] 基础环境 ----------
echo ""
echo "=== [0] 基础检查 ==="
echo "CPU: $(nproc) 核 | 内存: $(free -g | awk '/Mem/{print $2}')G"
df -h /hy-tmp | tail -1
mkdir -p /hy-tmp/datasets/water /hy-tmp/tools /hy-tmp/results /hy-tmp/slam-runner

# ---------- [1] OSS 登录态 ----------
echo ""
echo "=== [1] OSS 可用性检查 ==="
if oss ls oss:// > /dev/null 2>&1; then
  echo "[OK] OSS 已登录"
else
  echo "[!] OSS 未登录，尝试交互登录..."
  echo "    账号: 16605506117  密码: (你自己的恒源云密码)"
  oss login || { echo "[FAIL] OSS 登录失败，脚本中止"; exit 1; }
fi
# 若 .hycloud_ossutil_config 存在但失效，可手动替换：
#   scp 旧实例:/root/.hycloud_ossutil_config /root/.hycloud_ossutil_config

# ---------- [2] ROS 检查/安装 ----------
echo ""
echo "=== [2] ROS Noetic 检查 ==="
NEED_ROS=0
if [ -d /opt/ros/noetic ]; then
  echo "[OK] ROS Noetic 已存在（镜像实例，跳过安装）"
else
  echo "[!] 未检测到 ROS，需要安装（裸机实例）"
  NEED_ROS=1
fi

if [ "$NEED_ROS" = "1" ] || [ "${1:-}" = "--with-ros-install" ]; then
  echo "--- 配置 USTC ROS 源 + key ---"
  # 从 OSS 拉取 ROS 官方 key（oss 不支持 .gpg 扩展名，故打包为 tar.gz）
  oss cp oss://code/trusted_gpg.tar.gz /tmp/trusted_gpg.tar.gz 2>/dev/null && tar xzf /tmp/trusted_gpg.tar.gz -C /tmp/ || \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/trusted.gpg
  cp /tmp/trusted.gpg /etc/apt/trusted.gpg 2>/dev/null
  echo "deb [arch=amd64] https://mirrors.ustc.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros-fish.list
  apt-get update 2>&1 | tail -2

  echo "--- 安装 ROS 核心包 ---"
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-ros-base ros-noetic-rosbag ros-noetic-roslaunch \
    ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-robot-state-publisher \
    ros-noetic-robot-localization ros-noetic-gtsam ros-noetic-rviz \
    ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
    ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
    ros-noetic-angles ros-noetic-urdf ros-noetic-xacro ros-noetic-message-filters \
    ros-noetic-diagnostic-updater ros-noetic-image-transport ros-noetic-camera-info-manager \
    ros-noetic-eigen-conversions ros-noetic-tf-conversions ros-noetic-tf2-eigen \
    python3-rosdep python3-catkin-tools python3-pip 2>&1 | tail -3
  echo "[OK] ROS 安装完成"
fi
source /opt/ros/noetic/setup.bash

# ---------- [3] 从 OSS 拉取迁移包并解压 ----------
echo ""
echo "=== [3] OSS 迁移包下载解压 ==="
cd /tmp
# 每个包：已存在且解压目标完整则跳过
PULL_OK=1

echo "--- 3.1 catkin_ws (含编译产物, 466M) ---"
if [ -d /hy-tmp/catkin_ws/devel/lib/fast_lio ]; then
  echo "[SKIP] catkin_ws 已存在"
else
  oss cp oss://code/catkin_ws_full.tar.gz /tmp/catkin_ws_full.tar.gz 2>&1 | tail -1
  tar xzf /tmp/catkin_ws_full.tar.gz -C /hy-tmp/ && echo "[OK] catkin_ws 解压完成"
  PULL_OK=0
fi

echo "--- 3.2 Sophus 系统库 ---"
if [ -f /usr/local/include/sophus/se3.h ]; then
  echo "[SKIP] Sophus 已存在"
else
  oss cp oss://code/sophus_usr.tar.gz /tmp/sophus_usr.tar.gz 2>&1 | tail -1
  tar xzf /tmp/sophus_usr.tar.gz -C / && ldconfig && echo "[OK] Sophus 就位"
fi

echo "--- 3.3 水上数据 (w06/n03/h05 bag + 真值, 590M) ---"
if ls /hy-tmp/datasets/water/*.bag > /dev/null 2>&1; then
  echo "[SKIP] 水上数据已存在"
else
  oss cp oss://datasets/road/water_data.tar.gz /tmp/water_data.tar.gz 2>&1 | tail -1
  tar xzf /tmp/water_data.tar.gz -C /hy-tmp/datasets/water/ 2>/dev/null
  # 修复可能的嵌套路径（datasets/water/datasets/water/...）
  if [ -d /hy-tmp/datasets/water/datasets ]; then
    mv /hy-tmp/datasets/water/datasets/water/* /hy-tmp/datasets/water/ 2>/dev/null
    rm -rf /hy-tmp/datasets/water/datasets
  fi
  ls /hy-tmp/datasets/water/*.bag > /dev/null 2>&1 && echo "[OK] 水上数据就位"
fi

echo "--- 3.4 tools + slam-runner ---"
if [ -f /hy-tmp/tools/oss_backup.sh ]; then
  echo "[SKIP] tools 已存在"
else
  oss cp oss://tools/tools_full.tar.gz /tmp/tools_full.tar.gz 2>&1 | tail -1
  tar xzf /tmp/tools_full.tar.gz -C /hy-tmp/ && echo "[OK] tools 解压完成"
fi
# slam-runner 单独确保（小，直接拉最新）
if [ ! -d /hy-tmp/slam-runner/.git ]; then
  oss cp oss://tools/slam_runner_src.tar.gz /tmp/slam_runner_src.tar.gz 2>/dev/null && \
    tar xzf /tmp/slam_runner_src.tar.gz -C /hy-tmp/ 2>/dev/null && echo "[OK] slam-runner 就位" || \
    echo "[!] slam-runner 源码包未在 OSS（可从 GitHub clone）"
fi

echo "--- 3.5 历史结果 (785M, 可选) ---"
if [ "$(ls /hy-tmp/results/ 2>/dev/null | wc -l)" -gt 2 ]; then
  echo "[SKIP] results 已存在"
else
  oss cp oss://results/results_full.tar.gz /tmp/results_full.tar.gz 2>&1 | tail -1
  tar xzf /tmp/results_full.tar.gz -C /hy-tmp/ && echo "[OK] results 解压完成"
fi

# ---------- [4] evo 检查/安装 ----------
echo ""
echo "=== [4] evo 检查 ==="
if which evo_traj > /dev/null 2>&1; then
  echo "[OK] evo 已安装 ($(evo_traj --version 2>/dev/null | head -1))"
else
  echo "[!] 安装 evo..."
  pip3 install -q "argcomplete<2.0" evo 2>&1 | tail -2
  which evo_traj && echo "[OK] evo 安装完成"
fi

# ---------- [5] 系统配置 ----------
echo ""
echo "=== [5] 系统配置 ==="
# 时区
timedatectl set-timezone Asia/Shanghai 2>/dev/null || ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
date '+%F %T %Z'
# bashrc
grep -q '/opt/ros/noetic/setup.bash' ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/noetic/setup.bash
source /hy-tmp/catkin_ws/devel/setup.bash
EOF
echo "[OK] bashrc 已配置"

# ---------- [6] 验证 ----------
echo ""
echo "=== [6] 最终验证 ==="
source /hy-tmp/catkin_ws/devel/setup.bash 2>/dev/null
echo "--- 算法可执行文件 ---"
for d in fast_lio lio_sam fast_livo r3live point_lio fast_calib lidar_imu_init; do
  n=$(ls /hy-tmp/catkin_ws/devel/lib/$d/ 2>/dev/null | wc -l)
  echo "  $d: $n 个文件"
done
echo "--- 数据 ---"
ls /hy-tmp/datasets/water/*.bag 2>/dev/null | wc -l | xargs echo "  水上 bag 数:"
echo "--- 工具 ---"
ls /hy-tmp/slam-runner/run_slam.sh 2>/dev/null && echo "  run_slam.sh OK"

echo ""
echo "=============================================="
echo "setup_instance.sh 完成 $(date '+%F %T %Z')"
echo "下次跑实验: cd /hy-tmp/slam-runner && bash run_slam.sh <算法> <launch> <bag> <数据集> [--eval]"
echo "=============================================="
