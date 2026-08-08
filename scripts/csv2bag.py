#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""水上数据集 csv -> ROS bag 转换器
W06/N03: Lidar/*.csv(16线 90°FOV 10Hz) + IMU.csv(50Hz) + GPSBase.csv(5Hz ENU真值)
用法: python3 water_csv2bag.py W06 [帧数] [输出bag]
"""
import os, sys, math, struct
import numpy as np
import rosbag, zipfile, cv2
from rospy import Time as RTime
from sensor_msgs.msg import PointCloud2, PointField, Imu, Image as ImgMsg
from std_msgs.msg import Header

SEQ = sys.argv[1] if len(sys.argv) > 1 else 'W06'
N_FRAMES = int(sys.argv[2]) if len(sys.argv) > 2 else 10**9
OUT = sys.argv[3] if len(sys.argv) > 3 else None

if SEQ == 'W06':
    BASE = '/hy-tmp/datasets/water/W06_2_Sequence_57_115'
    OUT = OUT or '/hy-tmp/datasets/water/w06.bag'
elif SEQ == 'N03':
    BASE = '/hy-tmp/datasets/water/N03_4_Sequence_440_523'
    OUT = OUT or '/hy-tmp/datasets/water/n03.bag'
elif SEQ == 'H05':
    BASE = '/hy-tmp/datasets/water/H05_7_Sequence_160_270'
    OUT = OUT or '/hy-tmp/datasets/water/h05.bag'
elif SEQ == 'N02':
    # SDK 自带序列（低分辨率版），含官方标定 parameter/Lidar_to_Camera_Left.mat
    BASE = '/hy-tmp/datasets/water/USVInland_SLAM_SDK_1.0.1/Data_Low_Res_N02_6_Sequence_1275_1285'
    OUT = OUT or '/hy-tmp/datasets/water/n02.bag'
else:
    print(f'[!] 未知序列 {SEQ}, 支持 W06/N03/H05/N02'); sys.exit(1)

LID = os.path.join(BASE, 'Lidar', 'Lidar') if SEQ == 'N02' else os.path.join(BASE, 'Lidar')
# ---------- 0. 双目图像 (可选) ----------
img_zip = os.path.join(os.path.dirname(BASE), SEQ + '_Image.zip')
IMG_LEFT = os.path.join(os.path.dirname(BASE), SEQ + '_img_left', 'PIC_Left')
if SEQ == 'N02':
    # SDK 已解压图像 + 时间戳在 Image/ 下
    img_zip = os.path.join(BASE, 'Image')
    IMG_LEFT = os.path.join(BASE, 'Image', 'PIC_Left')
stereo_ts = []
if os.path.isdir(IMG_LEFT) and os.path.isdir(os.path.join(BASE, 'Image')):
    ts_file = os.path.join(BASE, 'Image', 'Stereo_Timestamp.txt')
    if os.path.isfile(ts_file):
        stereo_ts = [float(l) for l in open(ts_file)]
        print(f"[*] 图像: 左目 {len(os.listdir(IMG_LEFT))} 张, {len(stereo_ts)} 个时间戳, "
              f"{stereo_ts[0]:.4f}~{stereo_ts[-1]:.4f}s (IMU 轴, ~20Hz)")
elif os.path.isfile(img_zip) and os.path.isdir(IMG_LEFT):
    zf = zipfile.ZipFile(img_zip)
    stereo_ts = [float(l) for l in zf.read('Stereo_Timestamp.txt').decode().strip().split('\n')]
    print(f"[*] 图像: 左目 {len(os.listdir(IMG_LEFT))} 张, {len(stereo_ts)} 个时间戳, "
          f"{stereo_ts[0]:.4f}~{stereo_ts[-1]:.4f}s (IMU 轴, ~20Hz)")
print(f"[*] SEQ={SEQ} 输出={OUT}")

# ---------- 1. IMU 时间基准 (t=0 的 unix 秒) ----------
INS_DIR = os.path.join(BASE, 'INS') if SEQ == 'N02' else BASE
imu_lines = open(os.path.join(INS_DIR, 'IMU.csv')).read().strip().split('\n')
first = imu_lines[0].split(',')
base_unix = float(first[-1]) - float(first[0])
print(f"[*] base_unix = {base_unix:.6f}  (IMU {len(imu_lines)} 行)")

# ---------- 2. 激光帧时间戳 ----------
TS_DIR = os.path.join(BASE, 'Lidar') if SEQ == 'N02' else BASE
ts_start = [float(l) for l in open(os.path.join(TS_DIR, 'Laser_Timestamp_Start.txt'))]
print(f"[*] laser timestamps: {len(ts_start)} 帧, 首帧 {ts_start[0]:.4f}s")

# ---------- 3. 点云字段定义 (与 KITTI points_time_fix 输出一致, 22B/点) ----------
FIELDS = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
    PointField('ring', 16, PointField.UINT16, 1),
    PointField('time', 18, PointField.FLOAT32, 1),
]
PACK = struct.Struct('<ffffHf')  # x,y,z,intensity,ring,time = 22B

def make_pc2(xyz, intensity, ring, t_rel, stamp):
    n = len(xyz)
    buf = bytearray(n * 22)
    for k in range(n):
        PACK.pack_into(buf, k*22, xyz[k,0], xyz[k,1], xyz[k,2],
                       intensity[k], ring[k], t_rel[k])
    pc = PointCloud2()
    pc.header = Header(stamp=stamp, frame_id='camera_init')
    pc.height, pc.width, pc.fields = 1, n, FIELDS
    pc.is_bigendian, pc.is_dense = False, True
    pc.point_step, pc.row_step = 22, 22 * n
    pc.data = bytes(buf)
    return pc

# ---------- 4. 写 bag ----------
bag = rosbag.Bag(OUT, 'w')
try:
    # IMU (50Hz): acc 单位 g -> m/s^2, gyro 单位 deg/s -> rad/s
    n_imu = 0
    for line in imu_lines:
        v = line.split(',')
        t, ax, ay, az = float(v[0]), float(v[1]), float(v[2]), float(v[3])
        wx, wy, wz = float(v[4]), float(v[5]), float(v[6])
        imu = Imu()
        imu.header = Header(stamp=RTime.from_sec(base_unix + t), frame_id='camera_init')
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = ax*9.8, ay*9.8, az*9.8
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = math.radians(wx), math.radians(wy), math.radians(wz)
        # 单位四元数 (9-axis 有效), 否则 LIO-SAM imuPreintegration 报 Invalid quaternion 丢弃全部 IMU
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = 1.0
        # 协方差给个小值, 避免部分节点认为是无效 IMU
        imu.linear_acceleration_covariance[0] = 0.01
        imu.angular_velocity_covariance[0] = 0.01
        bag.write('/imu_correct', imu, t=imu.header.stamp)
        bag.write('/imu_raw', imu, t=imu.header.stamp)
        n_imu += 1
    print(f"[*] IMU 写入 {n_imu} 条 -> /imu_correct, /imu_raw")

    # LiDAR (10Hz): ring 负数归一化 0-15, time 归一化帧内 0~0.1s
    lidar_files = sorted(f for f in os.listdir(LID) if f.endswith('.csv'))
    lidar_files = lidar_files[:N_FRAMES]
    for i, fn in enumerate(lidar_files):
        pts = np.loadtxt(os.path.join(LID, fn), delimiter=',')
        if pts.ndim == 1:  # 空帧/单点
            continue
        t0 = float(np.min(pts[:, 7]))  # LiDAR 轴帧开始 (57~115s)
        frame_ts = ts_start[i] if i < len(ts_start) else t0  # IMU 轴帧时间 (0~58s)
        xyz = pts[:, :3].astype(np.float32)
        intensity = pts[:, 3].astype(np.float32)
        ring = ((pts[:, 4].astype(np.int16) + 15) // 2).astype(np.uint16)  # -15..15 -> 0..15
        t_rel = (pts[:, 7] - t0).astype(np.float32)  # 帧内 0~0.1s
        stamp = RTime.from_sec(base_unix + frame_ts)  # 用 IMU 轴时间戳
        pc = make_pc2(xyz, intensity, ring, t_rel, stamp)
        bag.write('/points_raw', pc, t=stamp)
        if (i+1) % 100 == 0:
            print(f"[*] LiDAR {i+1}/{len(lidar_files)}")
    print(f"[*] LiDAR 写入 {len(lidar_files)} 帧 -> /points_raw")

    # 图像 (20Hz): 左目 bgr8 -> /left_camera/image (FAST-LIVO2 img_topic)
    if stereo_ts:
        img_files = sorted(f for f in os.listdir(IMG_LEFT) if f.endswith('.jpg'))
        n_img = 0
        for i, fn in enumerate(img_files):
            ts = stereo_ts[i] if i < len(stereo_ts) else None
            if ts is None:
                continue
            bgr = cv2.imread(os.path.join(IMG_LEFT, fn))
            if bgr is None:
                continue
            h, w = bgr.shape[:2]
            img = ImgMsg()
            img.header = Header(stamp=RTime.from_sec(base_unix + ts), frame_id='camera_init')
            img.height, img.width, img.encoding, img.is_bigendian, img.step = h, w, 'bgr8', False, w * 3
            img.data = bgr.tobytes()
            bag.write('/left_camera/image', img, t=img.header.stamp)
            n_img += 1
            if (i+1) % 300 == 0:
                print(f"[*] Image {i+1}/{len(img_files)}")
        print(f"[*] 图像写入 {n_img} 张 -> /left_camera/image")

    # GPSBase -> ENU 真值 TUM (col2=x 东, col3=y 北, col10=高)
    gt_lines = open(os.path.join(INS_DIR, 'GPSBase.csv')).read().strip().split('\n')
    z0 = float(gt_lines[0].split(',')[9])
    gt_tum = OUT.replace('.bag', '_gt.tum')
    with open(gt_tum, 'w') as f:
        for line in gt_lines:
            v = line.split(',')
            t, x, y, z = float(v[0]), float(v[1]), float(v[2]), float(v[9]) - z0
            f.write(f"{base_unix + t:.6f} {x:.6f} {y:.6f} {z:.6f} 0 0 0 1\n")
    print(f"[*] GT 真值 {len(gt_lines)} 行 -> {gt_tum}")
finally:
    bag.close()
print(f"[+] 完成: {OUT}")
