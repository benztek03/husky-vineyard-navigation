#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class RowFollower(Node):
    def __init__(self) -> None:
        super().__init__('row_follower')

        self.declare_parameter('scan_topic', '/a200_1071/sensors/lidar2d_0/scan')
        self.declare_parameter('cmd_topic', '/a200_1071/cmd_vel')
        self.declare_parameter('enable_topic', '/a200_1071/vineyard_row_follow/enable')
        self.declare_parameter('status_topic', '/a200_1071/vineyard_row_follow/status')

        self.declare_parameter('base_speed', 0.28)
        self.declare_parameter('max_speed', 0.36)
        self.declare_parameter('max_angular', 0.70)
        self.declare_parameter('lookahead_min_x', 0.6)
        self.declare_parameter('lookahead_max_x', 4.0)
        self.declare_parameter('side_min_abs_y', 0.35)
        self.declare_parameter('side_max_abs_y', 2.5)
        self.declare_parameter('min_side_points', 8)
        self.declare_parameter('center_gain', 1.20)
        self.declare_parameter('heading_gain', 0.85)
        self.declare_parameter('fit_residual_threshold', 0.15)
        self.declare_parameter('lost_row_stop_scans', 4)
        self.declare_parameter('x_eval', 1.8)

        self.declare_parameter('front_half_angle_rad', 0.10)
        self.declare_parameter('front_open_min_range', 3.0)
        self.declare_parameter('end_detect_scans', 2)
        self.declare_parameter('lock_rearm_scans', 6)
        self.declare_parameter('front_open_steer_scale', 0.20)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.enable_topic = str(self.get_parameter('enable_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.lookahead_min_x = float(self.get_parameter('lookahead_min_x').value)
        self.lookahead_max_x = float(self.get_parameter('lookahead_max_x').value)
        self.side_min_abs_y = float(self.get_parameter('side_min_abs_y').value)
        self.side_max_abs_y = float(self.get_parameter('side_max_abs_y').value)
        self.min_side_points = int(self.get_parameter('min_side_points').value)
        self.center_gain = float(self.get_parameter('center_gain').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.fit_residual_threshold = float(self.get_parameter('fit_residual_threshold').value)
        self.lost_row_stop_scans = int(self.get_parameter('lost_row_stop_scans').value)
        self.x_eval = float(self.get_parameter('x_eval').value)

        self.front_half_angle_rad = float(self.get_parameter('front_half_angle_rad').value)
        self.front_open_min_range = float(self.get_parameter('front_open_min_range').value)
        self.end_detect_scans = int(self.get_parameter('end_detect_scans').value)
        self.lock_rearm_scans = int(self.get_parameter('lock_rearm_scans').value)
        self.front_open_steer_scale = float(self.get_parameter('front_open_steer_scale').value)

        self.enabled = False
        self.lost_count = 0
        self.end_count = 0
        self.had_row_lock = False
        self.end_latched = False
        self.lock_count = 0
        self.last_debug_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.enable_sub = self.create_subscription(Bool, self.enable_topic, self.enable_cb, 10)

        self.get_logger().info(
            f'row_follower ARC DEBUG listening to {self.scan_topic}, publishing to {self.cmd_topic}, enable topic {self.enable_topic}'
        )

    def enable_cb(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        self.end_latched = False
        self.end_count = 0
        self.lost_count = 0
        self.had_row_lock = False
        self.lock_count = 0
        if not self.enabled:
            self.publish_stop('DISABLED')
            self.get_logger().info('COMMAND: follower disabled')
        else:
            self.get_logger().info('COMMAND: follower enabled; waiting for line fits / row lock')
            self.publish_status('SEEKING_ROW')

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_stop(self, status: str) -> None:
        self.cmd_pub.publish(Twist())
        self.publish_status(status)

    def debug_log(self, text: str) -> None:
        now = time.time()
        if now - self.last_debug_time > 1.0:
            self.get_logger().info(text)
            self.last_debug_time = now

    def robust_fit(self, xs: np.ndarray, ys: np.ndarray) -> Optional[Tuple[float, float]]:
        if len(xs) < self.min_side_points:
            return None

        keep = np.ones_like(xs, dtype=bool)
        for _ in range(2):
            if np.count_nonzero(keep) < self.min_side_points:
                return None
            m, b = np.polyfit(xs[keep], ys[keep], 1)
            residuals = np.abs(ys - (m * xs + b))
            keep = residuals < self.fit_residual_threshold

        if np.count_nonzero(keep) < self.min_side_points:
            return None

        m, b = np.polyfit(xs[keep], ys[keep], 1)
        return float(m), float(b)

    def front_is_open(self, angles: np.ndarray, ranges: np.ndarray, valid: np.ndarray) -> bool:
        front_mask = valid & (np.abs(angles) <= self.front_half_angle_rad)
        if np.count_nonzero(front_mask) < 4:
            return False
        front_ranges = ranges[front_mask]
        return float(np.median(front_ranges)) >= self.front_open_min_range

    def publish_end_candidate_or_end(self, reason: str, front_open: bool) -> bool:
        self.lost_count += 1

        if self.had_row_lock:
            self.end_count += 1
            self.publish_status(
                f'END_CANDIDATE reason={reason} lost_count={self.lost_count} end_count={self.end_count} front_open={front_open}'
            )
            self.debug_log(
                f'END_CANDIDATE reason={reason} lost_count={self.lost_count} end_count={self.end_count} front_open={front_open}'
            )

            # Important: once we had a solid row lock, losing side lines at the headland is an end condition,
            # even if the front is not fully open because posts / vines may still be visible ahead.
            if self.lost_count >= self.lost_row_stop_scans or self.end_count >= self.end_detect_scans:
                if not self.end_latched:
                    self.end_latched = True
                    self.publish_stop(f'END_OF_ROW reason={reason} front_open={front_open}')
                return True

            self.cmd_pub.publish(Twist())
            return True

        self.publish_status(f'SEEKING_ROW reason={reason} front_open={front_open} lost_count={self.lost_count}')
        self.debug_log(f'SEEKING_ROW reason={reason} front_open={front_open} lost_count={self.lost_count}')
        self.cmd_pub.publish(Twist())
        return True

    def scan_cb(self, scan: LaserScan) -> None:
        if not self.enabled:
            return

        angles = scan.angle_min + np.arange(len(scan.ranges), dtype=np.float64) * scan.angle_increment
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges > max(scan.range_min, 0.05)) & (ranges < scan.range_max)

        if not np.any(valid):
            self.publish_end_candidate_or_end('NO_VALID_SCAN', False)
            return

        front_open = self.front_is_open(angles, ranges, valid)

        x_all = ranges[valid] * np.cos(angles[valid])
        y_all = ranges[valid] * np.sin(angles[valid])

        fwd_mask = (x_all >= self.lookahead_min_x) & (x_all <= self.lookahead_max_x)
        side_mask = (np.abs(y_all) >= self.side_min_abs_y) & (np.abs(y_all) <= self.side_max_abs_y)
        mask = fwd_mask & side_mask
        x = x_all[mask]
        y = y_all[mask]

        if len(x) < self.min_side_points * 2:
            self.publish_end_candidate_or_end('TOO_FEW_POINTS', front_open)
            return

        left_mask = y > 0.0
        right_mask = y < 0.0
        left_count = int(np.count_nonzero(left_mask))
        right_count = int(np.count_nonzero(right_mask))

        left_fit = self.robust_fit(x[left_mask], y[left_mask])
        right_fit = self.robust_fit(x[right_mask], y[right_mask])

        if left_fit is None or right_fit is None:
            self.publish_end_candidate_or_end(
                f'ROW_NOT_FOUND left_pts={left_count} right_pts={right_count}', front_open
            )
            return

        self.lock_count += 1
        if self.lock_count >= self.lock_rearm_scans:
            self.had_row_lock = True

        self.lost_count = 0
        self.end_count = 0

        m_left, b_left = left_fit
        m_right, b_right = right_fit

        y_left = m_left * self.x_eval + b_left
        y_right = m_right * self.x_eval + b_right
        center_offset = 0.5 * (y_left + y_right)
        heading_error = math.atan(0.5 * (m_left + m_right))

        angular = self.center_gain * center_offset + self.heading_gain * heading_error
        if front_open and self.had_row_lock:
            angular *= self.front_open_steer_scale
        angular = max(-self.max_angular, min(self.max_angular, angular))

        slowdown = min(0.7, abs(center_offset) * 0.8 + abs(heading_error) * 0.8)
        linear = max(0.10, self.base_speed * (1.0 - slowdown))
        if front_open and self.had_row_lock:
            linear = min(linear, self.base_speed * 0.75)
        linear = min(self.max_speed, linear)

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

        status = (
            f'FOLLOWING center_offset={center_offset:.3f} heading_error={heading_error:.3f} '
            f'linear={linear:.3f} angular={angular:.3f} front_open={front_open} '
            f'lock_count={self.lock_count} had_lock={self.had_row_lock} left_pts={left_count} right_pts={right_count}'
        )
        self.publish_status(status)
        self.debug_log(status)


def main() -> None:
    rclpy.init()
    node = RowFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop('SHUTDOWN')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
