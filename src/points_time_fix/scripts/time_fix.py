#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
points_time_fix: 给点云补充 per-point 'time' 字段（秒），使 LIO-SAM 去畸变生效。

KITTI bag 的 /points_raw 只有 x,y,z,intensity,ring，没有 time 字段，
LIO-SAM imageProjection 因此禁用 deskew（系统会漂移）。本节点按方位角模型
补上 time：KITTI HDL-64E 是旋转扫描（约 10Hz，周期 0.1s），某一点的
测量时刻约等于 方位角/360 * 扫描周期。

- 点云已有 'time' 或 't' 字段时直接透传（如 casual_walk.bag）。
- 话题通过 remap 配置：默认订阅 /points_raw，发布 /points_raw_fixed。
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField

_DTYPE = {
    PointField.FLOAT64: ('<f8', 8), PointField.FLOAT32: ('<f4', 4),
    PointField.INT32: ('<i4', 4), PointField.UINT32: ('<u4', 4),
    PointField.INT16: ('<i2', 2), PointField.UINT16: ('<u2', 2),
    PointField.INT8: ('<i1', 1), PointField.UINT8: ('<u1', 1),
}


def _extract_field(cloud, name):
    """按字段偏移从 PointCloud2 原始缓冲提取数组，返回形状 (n,) 或 (n, count)。"""
    f = next((f for f in cloud.fields if f.name == name), None)
    if f is None:
        return None
    n = cloud.width * cloud.height
    if n == 0:
        return None
    dtype, size = _DTYPE[f.datatype]
    data = np.frombuffer(cloud.data, dtype=np.uint8)
    byte_idx = f.offset + np.arange(n) * cloud.point_step
    gather = data[byte_idx[:, None] + np.arange(f.count * size)[None, :]]
    return gather.view(dtype).reshape(n, -1)


def add_time_field(cloud, scan_period):
    """为没有 time 字段的点云添加 time（秒，方位角模型）。"""
    n = cloud.width * cloud.height
    x = _extract_field(cloud, 'x').ravel()
    y = _extract_field(cloud, 'y').ravel()
    z = _extract_field(cloud, 'z').ravel()
    intensity = _extract_field(cloud, 'intensity')
    ring = _extract_field(cloud, 'ring')

    azimuth = np.degrees(np.arctan2(x, y)) % 360.0
    time = (azimuth / 360.0 * scan_period).astype('<f4')

    out = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                             ('intensity', '<f4'), ('ring', '<u2'), ('time', '<f4')])
    out['x'] = x
    out['y'] = y
    out['z'] = z
    out['intensity'] = intensity.ravel() if intensity is not None else np.zeros(n, '<f4')
    out['ring'] = ring.ravel() if ring is not None else np.zeros(n, '<u2')
    out['time'] = time

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('ring', 16, PointField.UINT16, 1),
        PointField('time', 18, PointField.FLOAT32, 1),
    ]
    return PointCloud2(
        header=cloud.header, height=cloud.height, width=cloud.width,
        fields=fields, is_bigendian=False, point_step=22, row_step=22 * n,
        is_dense=True, data=out.tobytes())


class TimeFixNode:
    def __init__(self):
        self.scan_period = rospy.get_param('~scan_period', 0.1)
        self.pub = rospy.Publisher('out', PointCloud2, queue_size=5)
        self.sub = rospy.Subscriber('in', PointCloud2, self.callback, queue_size=5)
        self.passthrough = 0
        self.fixed = 0

    def callback(self, cloud):
        names = [f.name for f in cloud.fields]
        if 'time' in names or 't' in names:
            self.passthrough += 1
            self.pub.publish(cloud)
        else:
            self.fixed += 1
            self.pub.publish(add_time_field(cloud, self.scan_period))
        if (self.passthrough + self.fixed) % 10 == 0:
            rospy.loginfo('fixed: %d, passthrough: %d', self.fixed, self.passthrough)


if __name__ == '__main__':
    rospy.init_node('points_time_fix')
    TimeFixNode()
    rospy.spin()
