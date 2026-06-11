#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')

        self.declare_parameter('enable_topic', '/a200_1071/vineyard_row_follow/enable')
        self.declare_parameter('status_topic', '/a200_1071/vineyard_row_follow/status')
        self.declare_parameter('odom_topic', '/a200_1071/platform/odom/filtered')
        self.declare_parameter('cmd_topic', '/a200_1071/cmd_vel')

        # 7 ft 5 in = 2.2606 m. Arc radius = row_spacing_m / 2.
        self.declare_parameter('row_spacing_m', 2.2606)
        self.declare_parameter('forward_clearance_m', 1.5)
        self.declare_parameter('arc_speed_mps', 0.18)
        self.declare_parameter('heading_hold_gain', 1.8)
        self.declare_parameter('max_heading_correction', 0.40)
        self.declare_parameter('reacquire_pause_sec', 0.8)
        self.declare_parameter('right_first', True)
        self.declare_parameter('arc_radius_scale', 1.15)
        self.declare_parameter('arc_radial_gain', 0.35)
        self.declare_parameter('arc_max_angular', 0.45)

        # Prevent false END_OF_ROW right after a re-enable.
        self.declare_parameter('min_row_distance_before_end_m', 0.8)
        self.declare_parameter('min_row_time_before_end_sec', 2.0)

        self.enable_topic = str(self.get_parameter('enable_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.row_spacing_m = float(self.get_parameter('row_spacing_m').value)
        self.forward_clearance_m = float(self.get_parameter('forward_clearance_m').value)
        self.arc_speed_mps = float(self.get_parameter('arc_speed_mps').value)
        self.heading_hold_gain = float(self.get_parameter('heading_hold_gain').value)
        self.max_heading_correction = float(self.get_parameter('max_heading_correction').value)
        self.reacquire_pause_sec = float(self.get_parameter('reacquire_pause_sec').value)
        self.right_first = bool(self.get_parameter('right_first').value)
        self.arc_radius_scale = float(self.get_parameter('arc_radius_scale').value)
        self.arc_radial_gain = float(self.get_parameter('arc_radial_gain').value)
        self.arc_max_angular = float(self.get_parameter('arc_max_angular').value)
        self.min_row_distance_before_end_m = float(self.get_parameter('min_row_distance_before_end_m').value)
        self.min_row_time_before_end_sec = float(self.get_parameter('min_row_time_before_end_sec').value)

        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.status_sub = self.create_subscription(String, self.status_topic, self.status_cb, 10)

        self.current_xy: Optional[Tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.latest_status: str = ''
        self.status_seq: int = 0
        self.last_status_log_time: float = 0.0
        self.last_wait_log_time: float = 0.0

        self.row_index: int = 0
        self.enable_seq: int = 0
        self.seen_following_since_enable: bool = False
        self.row_start_xy: Optional[Tuple[float, float]] = None
        self.row_start_time: Optional[float] = None

        self.end_statuses = ('END_OF_ROW',)
        self.lost_statuses = ('ROW_NOT_FOUND', 'TOO_FEW_POINTS', 'NO_VALID_SCAN')

        self.get_logger().info(
            'mission_manager ARC DEBUG ready: automatic 1.5 m clear, alternating arcs, accepts lost-row-as-end after row lock.'
        )
        self.get_logger().info(
            f'params: row_spacing={self.row_spacing_m:.3f} m nominal_radius={0.5*self.row_spacing_m:.3f} m '
            f'effective_radius={0.5*self.row_spacing_m*self.arc_radius_scale:.3f} m radius_scale={self.arc_radius_scale:.2f} '
            f'clearance={self.forward_clearance_m:.2f} m min_end_distance={self.min_row_distance_before_end_m:.2f} m '
            f'min_end_time={self.min_row_time_before_end_sec:.1f} sec'
        )

    def odom_cb(self, msg: Odometry) -> None:
        self.current_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def status_cb(self, msg: String) -> None:
        self.latest_status = str(msg.data)
        self.status_seq += 1

        if self.status_seq > self.enable_seq and self.latest_status.startswith('FOLLOWING'):
            self.seen_following_since_enable = True

        now = time.time()
        if (not self.latest_status.startswith('FOLLOWING')) or (now - self.last_status_log_time > 1.0):
            self.get_logger().info(f'row_follower_status: {self.latest_status}')
            self.last_status_log_time = now

    def wait_for_odom(self, timeout_sec: float = 15.0) -> None:
        start = time.time()
        while rclpy.ok() and (self.current_xy is None or self.current_yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                raise RuntimeError('Timed out waiting for odometry')

    def set_row_follow_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_pub.publish(msg)

        if enabled:
            self.latest_status = ''
            self.enable_seq = self.status_seq
            self.seen_following_since_enable = False
            if self.current_xy is not None:
                self.row_start_xy = (self.current_xy[0], self.current_xy[1])
                self.row_start_time = time.time()
            self.get_logger().info('COMMAND: row follower ENABLED. Waiting for row lock / FOLLOWING.')
        else:
            self.get_logger().info('COMMAND: row follower DISABLED.')

    def publish_stop(self) -> None:
        for _ in range(4):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def row_progress(self) -> Tuple[float, float]:
        if self.row_start_xy is None or self.current_xy is None or self.row_start_time is None:
            return 0.0, 0.0
        dx = self.current_xy[0] - self.row_start_xy[0]
        dy = self.current_xy[1] - self.row_start_xy[1]
        traveled = math.hypot(dx, dy)
        dt = time.time() - self.row_start_time
        return traveled, dt

    def row_end_is_allowed(self) -> bool:
        if not self.seen_following_since_enable:
            return False
        traveled, dt = self.row_progress()
        return traveled >= self.min_row_distance_before_end_m and dt >= self.min_row_time_before_end_sec

    def reset_follower_after_false_end(self, reason: str) -> None:
        traveled, dt = self.row_progress()
        self.get_logger().warn(
            f'Ignoring early/stale end status "{reason}". seen_following={self.seen_following_since_enable} '
            f'traveled={traveled:.2f} m dt={dt:.1f} sec. Resetting follower.'
        )
        self.publish_stop()
        self.set_row_follow_enabled(False)
        time.sleep(0.25)
        self.set_row_follow_enabled(True)

    def drive_forward_with_heading_hold(self, distance_m: float) -> None:
        self.wait_for_odom()
        start_xy = self.current_xy
        target_yaw = self.current_yaw
        if start_xy is None or target_yaw is None:
            raise RuntimeError('No odom before forward move')

        self.get_logger().info(f'ACTION: auto-clearing forward {distance_m:.2f} m with heading hold.')
        last_log = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_xy is None or self.current_yaw is None:
                continue

            dx = self.current_xy[0] - start_xy[0]
            dy = self.current_xy[1] - start_xy[1]
            traveled = math.hypot(dx, dy)
            if traveled >= distance_m:
                break

            yaw_err = wrap_to_pi(target_yaw - self.current_yaw)
            angular = self.heading_hold_gain * yaw_err
            angular = max(-self.max_heading_correction, min(self.max_heading_correction, angular))

            cmd = Twist()
            cmd.linear.x = self.arc_speed_mps
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)

            now = time.time()
            if now - last_log > 0.5:
                self.get_logger().info(f'clearance progress: {traveled:.2f}/{distance_m:.2f} m yaw_err={math.degrees(yaw_err):.1f} deg')
                last_log = now

        self.publish_stop()
        time.sleep(0.25)
        self.get_logger().info('ACTION COMPLETE: clearance forward done.')

    def run_arc_to_next_row(self, side: str) -> None:
        self.wait_for_odom()
        start_xy = self.current_xy
        start_yaw = self.current_yaw
        if start_xy is None or start_yaw is None:
            raise RuntimeError('No odom before arc')

        nominal_radius = 0.5 * self.row_spacing_m
        radius = nominal_radius * self.arc_radius_scale
        sign = -1.0 if side == 'right' else 1.0
        target_yaw = wrap_to_pi(start_yaw + sign * math.pi)

        # Virtual circle center. This is the point the robot arcs around.
        # It is not a detected post. It is a geometry target based on row spacing.
        center_angle = start_yaw + sign * (math.pi / 2.0)
        center_x = start_xy[0] + radius * math.cos(center_angle)
        center_y = start_xy[1] + radius * math.sin(center_angle)
        nominal_angular_mag = abs(self.arc_speed_mps / radius)

        self.get_logger().info(
            f'ACTION: running {side} feedback arc: row_spacing={self.row_spacing_m:.3f} m, '
            f'nominal_radius={nominal_radius:.3f} m, effective_radius={radius:.3f} m, '
            f'radius_scale={self.arc_radius_scale:.2f}, linear={self.arc_speed_mps:.2f} m/s.'
        )
        self.get_logger().info(
            f'ARC CENTER: x={center_x:.3f}, y={center_y:.3f}; this is a virtual center, not a detected post.'
        )

        last_log = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_xy is None or self.current_yaw is None:
                continue

            remaining = wrap_to_pi(target_yaw - self.current_yaw)
            if abs(remaining) <= math.radians(3.0):
                break

            dx = self.current_xy[0] - center_x
            dy = self.current_xy[1] - center_y
            dist_to_center = math.hypot(dx, dy)
            radial_error = dist_to_center - radius

            # Feedback: if we drift outside the circle, turn harder into the circle.
            # If we drift inside the circle, turn less. This helps with skid/wheel slip.
            angular_mag = nominal_angular_mag + self.arc_radial_gain * radial_error
            angular_mag = max(0.05, min(self.arc_max_angular, angular_mag))
            angular = sign * angular_mag

            cmd = Twist()
            cmd.linear.x = self.arc_speed_mps
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)

            now = time.time()
            if now - last_log > 0.7:
                self.get_logger().info(
                    f'arc progress: remaining_yaw={math.degrees(remaining):.1f} deg '
                    f'radial_error={radial_error:.2f} m angular={angular:.3f}'
                )
                last_log = now

        self.publish_stop()
        time.sleep(self.reacquire_pause_sec)
        self.get_logger().info('ACTION COMPLETE: feedback arc done.')

    def next_side(self) -> str:
        if self.row_index % 2 == 0:
            return 'right' if bool(self.right_first) else 'left'
        return 'left' if bool(self.right_first) else 'right'

    def handle_accepted_end(self, status: str) -> None:
        traveled, dt = self.row_progress()
        self.publish_stop()
        self.set_row_follow_enabled(False)
        self.get_logger().info(f'ACCEPTED ROW END from status="{status}" traveled={traveled:.2f} m dt={dt:.1f} sec')

        self.drive_forward_with_heading_hold(self.forward_clearance_m)

        side = self.next_side()
        self.run_arc_to_next_row(side)

        self.row_index += 1
        self.latest_status = ''
        self.set_row_follow_enabled(True)

    def run(self) -> None:
        self.wait_for_odom()
        self.set_row_follow_enabled(True)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = time.time()
            if now - self.last_wait_log_time > 2.0:
                traveled, dt = self.row_progress()
                self.get_logger().info(
                    f'WAITING: status="{self.latest_status}" seen_following={self.seen_following_since_enable} '
                    f'traveled={traveled:.2f} m dt={dt:.1f} sec row_index={self.row_index}'
                )
                self.last_wait_log_time = now

            if not self.latest_status:
                continue

            status_is_end = self.latest_status.startswith(self.end_statuses)
            status_is_lost = self.latest_status.startswith(self.lost_statuses)

            if status_is_end or status_is_lost:
                if self.row_end_is_allowed():
                    self.handle_accepted_end(self.latest_status)
                else:
                    self.reset_follower_after_false_end(self.latest_status)
                self.latest_status = ''

        self.publish_stop()


def main() -> None:
    rclpy.init()
    node = MissionManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.set_row_follow_enabled(False)
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
