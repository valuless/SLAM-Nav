#!/usr/bin/env python3
"""
地面标定 ROS2 节点：订阅 /cloud_registered_body，采集地面点云，
拟合地平面，计算修正后的 LiDAR-IMU 外参。

用法：
  1. 把机器人放在水平地面上
  2. 启动 FAST_LIO（确保 scan_bodyframe_pub_en: true）
  3. python3 calibrate_ground.py
  4. 等待采集完成，控制台输出修正后的 extrinsic_R 和 extrinsic_T
  5. 填入 rslidar.yaml

可选参数：
  --frames N       采集帧数（默认 50）
  --extrinsic-r    当前 extrinsic_R（9个数，按行），默认使用收敛后的值
  --extrinsic-t    当前 extrinsic_T（3个数）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import argparse
import sys


class GroundCalibrator(Node):
    def __init__(self, num_frames, current_R, current_T):
        super().__init__('ground_calibrator')
        self.sub = self.create_subscription(
            PointCloud2, '/cloud_registered_body', self.callback, 10)
        self.num_frames = num_frames
        self.current_R = current_R
        self.current_T = current_T
        self.all_ground_pts = []
        self.frame_count = 0
        self.get_logger().info(f'开始采集 {num_frames} 帧地面点云...')

    def callback(self, msg):
        if self.frame_count >= self.num_frames:
            return

        pts = self._pc2_to_array(msg)
        if len(pts) == 0:
            return

        # 提取地面候选点：取 Z 最低的 %30
        z = pts[:, 2]
        z_min, z_max = z.min(), z.max()
        z_cutoff = z_min + (z_max - z_min) * 0.3
        ground = pts[z <= z_cutoff]

        if len(ground) > 50:
            self.all_ground_pts.append(ground)

        self.frame_count += 1
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'  已采集 {self.frame_count}/{self.num_frames} 帧')

        if self.frame_count >= self.num_frames:
            self.compute()

    def _pc2_to_array(self, msg):
        fields = [f.name for f in msg.fields]
        x_idx = fields.index('x')
        y_idx = fields.index('y')
        z_idx = fields.index('z')

        data = np.frombuffer(msg.data, dtype=np.float32)
        pts_per_point = len(fields) + int(msg.point_step / 4 - len(fields))
        if pts_per_point < 3:
            pts_per_point = max(len(fields), 3)

        # 处理可能存在的 padding
        n_points = msg.width * msg.height
        pts = np.zeros((n_points, 3), dtype=np.float32)
        step = msg.point_step // 4

        raw = data.reshape(-1, step) if step > 0 else data.reshape(n_points, -1)
        pts[:, 0] = raw[:, x_idx]
        pts[:, 1] = raw[:, y_idx]
        pts[:, 2] = raw[:, z_idx]

        # 过滤 NaN
        mask = np.isfinite(pts).all(axis=1)
        return pts[mask]

    def compute(self):
        if not self.all_ground_pts:
            self.get_logger().error('未采集到地面点')
            rclpy.shutdown()
            return

        all_pts = np.vstack(self.all_ground_pts)
        self.get_logger().info(f'共采集 {len(all_pts)} 个地面候选点')

        # RANSAC 拟合平面
        normal, centroid = self._ransac_plane(all_pts)

        self.get_logger().info(f'地面法向量 (IMU/body系): '
                               f'[{normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f}]')
        self.get_logger().info(f'地面中心: [{centroid[0]:.3f}, {centroid[1]:.3f}, '
                               f'{centroid[2]:.3f}]')

        # 目标：法向量指向上方 [0, 0, 1]
        target = np.array([0., 0., 1.])
        angle = np.arccos(np.clip(np.dot(normal, target), -1, 1))
        self.get_logger().info(f'地面倾斜角度: {np.degrees(angle):.3f}°')

        if np.degrees(angle) < 0.05:
            self.get_logger().info('地面已水平，无需修正！')
            rclpy.shutdown()
            return

        # 修正旋转
        R_corr = self._rotation_to_align(normal, target)
        R_new = R_corr @ self.current_R
        T_new = R_corr @ self.current_T

        print('\n' + '=' * 55)
        print('修正后的 extrinsic_R:')
        for i, row in enumerate(R_new):
            vals = ', '.join(f'{v:.8f}' for v in row)
            if i == 0:
                print(f'  [ {vals},')
            elif i == 1:
                print(f'    {vals},')
            else:
                print(f'    {vals}]')

        print(f'\n修正后的 extrinsic_T:')
        print(f'  [ {T_new[0]:.8f}, {T_new[1]:.8f}, {T_new[2]:.8f} ]')

        # 验证
        n_corrected = R_corr @ normal
        print(f'\n验证 - 修正后法向量: [{n_corrected[0]:.6f}, '
              f'{n_corrected[1]:.6f}, {n_corrected[2]:.6f}]')
        print(f'行列式: {np.linalg.det(R_new):.6f}')
        print('=' * 55)
        rclpy.shutdown()

    def _ransac_plane(self, points, dist_thresh=0.03, max_iters=200):
        n = len(points)
        best_inliers = 0
        best_normal = None
        best_centroid = None

        for _ in range(max_iters):
            i, j, k = np.random.choice(n, 3, replace=False)
            p1, p2, p3 = points[i], points[j], points[k]
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            nlen = np.linalg.norm(normal)
            if nlen < 1e-9:
                continue
            normal /= nlen
            if normal[2] < 0:
                normal = -normal

            dists = np.abs(np.dot(points - p1, normal))
            inliers = np.sum(dists < dist_thresh)
            if inliers > best_inliers:
                best_inliers = inliers
                best_normal = normal
                best_centroid = p1

        if best_normal is None:
            raise RuntimeError('RANSAC 未找到平面')

        # 用内点精炼
        dists = np.abs(np.dot(points - best_centroid, best_normal))
        inlier_pts = points[dists < dist_thresh]
        refined_centroid = inlier_pts.mean(axis=0)
        centered = inlier_pts - refined_centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        refined_normal = vh[2]
        if refined_normal[2] < 0:
            refined_normal = -refined_normal

        return refined_normal, refined_centroid

    def _rotation_to_align(self, src, dst):
        src = src / np.linalg.norm(src)
        dst = dst / np.linalg.norm(dst)
        v = np.cross(src, dst)
        c = np.dot(src, dst)

        if abs(c + 1) < 1e-9:
            perp = np.array([1., 0., 0.])
            if abs(np.dot(perp, src)) > 0.9:
                perp = np.array([0., 1., 0.])
            perp = perp - np.dot(perp, src) * src
            perp /= np.linalg.norm(perp)
            vx = np.array([[0, -perp[2], perp[1]],
                           [perp[2], 0, -perp[0]],
                           [-perp[1], perp[0], 0]])
            return np.eye(3) + 2 * vx @ vx

        vx = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]])
        return np.eye(3) + vx + vx @ vx * (1 - c) / np.dot(v, v)


def main():
    parser = argparse.ArgumentParser(description='LiDAR-IMU 地面标定')
    parser.add_argument('--frames', type=int, default=50,
                        help='采集帧数（默认 50）')
    parser.add_argument('--extrinsic-r', nargs=9, type=float,
                        default=[-0.02186906, -0.99971067, -0.01001592,
                                 -0.99975282, 0.02190793, -0.00378698,
                                 0.00400531, 0.00993063, -0.99994267],
                        help='当前 extrinsic_R')
    parser.add_argument('--extrinsic-t', nargs=3, type=float,
                        default=[-0.02305, 0.01213, 0.00135],
                        help='当前 extrinsic_T')
    args = parser.parse_args()

    current_R = np.array(args.extrinsic_r).reshape(3, 3)
    current_T = np.array(args.extrinsic_t)

    rclpy.init(args=sys.argv)
    node = GroundCalibrator(args.frames, current_R, current_T)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
