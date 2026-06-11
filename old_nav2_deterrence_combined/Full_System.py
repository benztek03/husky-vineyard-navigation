#!/usr/bin/env python3
import math
import os
import random
import threading
import time
from typing import List, Tuple, Optional

import pygame
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from vision_msgs.msg import Detection2DArray


def yaw_to_quat(yaw_rad: float):
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


CLASS_MAP = {
    "0": "bird",
    "1": "deer",
    "2": "rodent",
}


class Nav2GoalRunner(Node):
    def __init__(self, namespace: str):
        super().__init__("nav2_goal_runner")

        # ----------------------------
        # Nav2 setup
        # ----------------------------
        self.namespace = namespace.rstrip("/")
        self.action_name = f"{self.namespace}/navigate_to_pose"
        self.client = ActionClient(self, NavigateToPose, self.action_name)
        self.cmd_pub = self.create_publisher(Twist, f"{self.namespace}/cmd_vel", 10)

        # ----------------------------
        # Detection subscriber
        # ----------------------------
        self.pest_sub = self.create_subscription(
            Detection2DArray, "/pest_detections", self.on_pest_detection, 10
        )

        # ----------------------------
        # Audio setup
        # ----------------------------
        self.sound_root = "/home/administrator/Predator Sounds"
        self.sound_duration = 10.0
        self.sound_cooldown = 5.0
        self.valid_exts = (".wav",)

        self.sound_folder_map = {
            "deer": ["coyote"],
            "rodent": ["owl", "falcon"],
            "bird": ["hawk", "falcon"],
        }

        try:
            pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=4096)
            pygame.mixer.init()
            self.get_logger().info("pygame mixer initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize pygame mixer: {e}")

        # ----------------------------
        # Detection / deterrence state
        # ----------------------------
        self.interrupt_requested = False
        self.deterring = False
        self.sound_playing = False

        self.last_px: Optional[float] = None
        self.last_py: Optional[float] = None
        self.last_sx: Optional[float] = None
        self.last_sy: Optional[float] = None
        self.last_class: Optional[str] = None
        self.last_score: float = 0.0
        self.last_detection_wall_time = 0.0

        # Smoothed target tracking
        self.track_px: Optional[float] = None
        self.track_score: float = 0.0
        self.track_last_update = 0.0

        # Goal state
        self.goal_handle = None
        self.current_goal_pose: Optional[PoseStamped] = None

        # ----------------------------
        # Tunables
        # ----------------------------
        self.image_width_px = 640.0
        self.center_deadband_px = 18.0

        # Turning
        self.kp_turn = 0.0012
        self.max_wz = 0.18
        self.turn_rate_hz = 20.0

        # Detection confidence / filtering
        self.min_score = 0.65
        self.required_hits = 2
        self.time_window = 2.0
        self.detection_times = {
            "bird": [],
            "deer": [],
            "rodent": [],
        }

        # Tracking
        self.ema_alpha = 0.50
        self.max_jump_px = 220.0
        self.target_timeout_s = 0.8
        self.detection_cooldown_s = 0.10

        # Centering / deterrence timing
        self.max_center_time_s = 2.5
        self.center_hold_cycles = 8
        self.target_lost_abort_s = 0.5

    # ----------------------------
    # Nav2 helpers
    # ----------------------------
    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        ok = self.client.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            self.get_logger().error("Action server not available.")
        return ok

    def make_goal(self, x: float, y: float, yaw_deg: float, frame_id: str = "map") -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        yaw = math.radians(float(yaw_deg))
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def cancel_current_goal(self):
        if self.goal_handle is None:
            return
        try:
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
        except Exception as e:
            self.get_logger().warn(f"Cancel goal exception: {e}")

    # ----------------------------
    # Motion helpers
    # ----------------------------
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _center_px(self) -> float:
        return self.image_width_px * 0.5

    def compute_turn_cmd_from_err(self, err_px: float) -> float:
        if abs(err_px) < self.center_deadband_px:
            return 0.0
        wz = -self.kp_turn * err_px
        wz = max(-self.max_wz, min(self.max_wz, wz))
        return float(wz)

    def compute_center_error_px(self) -> Optional[float]:
        # Prefer fresh raw detection over smoothed track
        if self._last_detection_is_fresh() and self.last_px is not None:
            return float(self.last_px - self._center_px())
        if self._target_is_fresh() and self.track_px is not None:
            return float(self.track_px - self._center_px())
        if self.last_px is not None:
            return float(self.last_px - self._center_px())
        return None

    # ----------------------------
    # Audio helpers
    # ----------------------------
    def list_sound_files(self, pest_name: str):
        folder_names = self.sound_folder_map.get(pest_name)
        if folder_names is None:
            self.get_logger().warn(f"No sound folder mapped for pest '{pest_name}'")
            return []

        # Allow either one folder name as a string or several folder names as a list.
        if isinstance(folder_names, str):
            folder_names = [folder_names]

        files = []
        for folder_name in folder_names:
            folder_path = os.path.join(self.sound_root, folder_name)
            if not os.path.isdir(folder_path):
                self.get_logger().warn(f"Sound folder does not exist: {folder_path}")
                continue

            for name in os.listdir(folder_path):
                full_path = os.path.join(folder_path, name)
                if os.path.isfile(full_path) and name.lower().endswith(self.valid_exts):
                    files.append(full_path)

        return files

    def choose_sound_file(self, pest_name: str):
        files = self.list_sound_files(pest_name)
        if not files:
            return None
        return random.choice(files)

    def play_deterrent_sound(self, pest_name: str):
        try:
            sound_file = self.choose_sound_file(pest_name)
            if sound_file is None:
                self.get_logger().warn(f"No sound file found for pest '{pest_name}'")
                return

            self.get_logger().warn(f"[DETER] Playing {pest_name} deterrent: {sound_file}")

            pygame.mixer.music.load(sound_file)
            pygame.mixer.music.play()

            start_time = time.time()
            while (
                rclpy.ok()
                and time.time() - start_time < self.sound_duration
                and pygame.mixer.music.get_busy()
            ):
                time.sleep(0.1)

            pygame.mixer.music.stop()

        except Exception as e:
            self.get_logger().error(f"Playback failed: {e}")
        finally:
            self.sound_playing = False

    # ----------------------------
    # Detection handling
    # ----------------------------
    def _update_track(self, new_px: float, score: float, now: float):
        if self.track_px is None:
            self.track_px = float(new_px)
            self.track_score = float(score)
            self.track_last_update = now
            return

        # Allow large jumps by snapping to new target instead of ignoring forever
        if abs(new_px - self.track_px) > self.max_jump_px:
            self.track_px = float(new_px)
            self.track_score = float(score)
            self.track_last_update = now
            return

        a = self.ema_alpha
        self.track_px = (1.0 - a) * self.track_px + a * float(new_px)
        self.track_score = float(score)
        self.track_last_update = now

    def on_pest_detection(self, msg: Detection2DArray):
        now = time.time()

        if (now - self.last_detection_wall_time) < self.detection_cooldown_s:
            return

        if not msg.detections:
            return

        best_score = -1.0
        best_class = None
        best_det = None

        for det in msg.detections:
            for r in det.results:
                cid = str(r.hypothesis.class_id)
                if cid not in CLASS_MAP:
                    continue

                score = float(r.hypothesis.score)
                if score > best_score:
                    best_score = score
                    best_class = cid
                    best_det = det

        if best_det is None or best_class is None:
            return

        px = float(best_det.bbox.center.position.x)
        py = float(best_det.bbox.center.position.y)
        sx = float(best_det.bbox.size_x)
        sy = float(best_det.bbox.size_y)

        pest_name = CLASS_MAP.get(best_class, best_class)

        self.last_detection_wall_time = now
        self.last_class = best_class
        self.last_score = best_score
        self.last_px = px
        self.last_py = py
        self.last_sx = sx
        self.last_sy = sy

        if best_score >= self.min_score:
            self._update_track(px, best_score, now)

            self.detection_times[pest_name].append(now)
            self.detection_times[pest_name] = [
                t for t in self.detection_times[pest_name]
                if now - t <= self.time_window
            ]

            hit_count = len(self.detection_times[pest_name])
            err = self.compute_center_error_px()
            err_str = "?" if err is None else f"{err:+.1f}px"

            self.get_logger().info(
                f"[DETECTION] {pest_name} conf={best_score:.2f} "
                f"hits={hit_count}/{self.required_hits} "
                f"last_px={px:.1f} "
                f"track_px={None if self.track_px is None else round(self.track_px, 1)} "
                f"center_err={err_str}"
            )

            if (not self.deterring) and hit_count >= self.required_hits:
                self.get_logger().warn(
                    f"[DETECTION] CONFIRMED {pest_name.upper()} "
                    f"bbox_center=({px:.1f},{py:.1f}) size=({sx:.1f},{sy:.1f}) "
                    f"track_px={None if self.track_px is None else round(self.track_px, 1)} "
                    f"center_err={err_str}"
                )
                self.interrupt_requested = True
                self.detection_times[pest_name].clear()

    def _target_is_fresh(self) -> bool:
        if self.track_last_update == 0.0:
            return False
        return (time.time() - self.track_last_update) <= self.target_timeout_s

    def _last_detection_is_fresh(self) -> bool:
        if self.last_detection_wall_time == 0.0:
            return False
        return (time.time() - self.last_detection_wall_time) <= self.target_timeout_s

    def pest_still_present_after_cooldown(self, pest_name: str) -> bool:
        self.get_logger().info(f"[DETER] Cooldown watch for {self.sound_cooldown:.1f}s...")

        end = time.time() + self.sound_cooldown
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)

        still_present = (
            self.last_class is not None
            and CLASS_MAP.get(self.last_class) == pest_name
            and self.last_score >= self.min_score
            and self._last_detection_is_fresh()
        )

        if still_present:
            self.get_logger().warn(f"[DETER] {pest_name.upper()} still present after cooldown.")
        else:
            self.get_logger().info(f"[DETER] {pest_name.upper()} gone. Resuming navigation.")

        return still_present

    # ----------------------------
    # Deterrence behavior
    # ----------------------------
    def orient_to_target(self) -> bool:
        start = time.time()
        dt = 1.0 / self.turn_rate_hz
        centered_count = 0
        lost_start = None

        self.get_logger().info("[DETER] Orienting to target...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if (time.time() - start) > self.max_center_time_s:
                self.get_logger().warn("[DETER] Orientation timeout.")
                self.stop_robot()
                return False

            fresh = self._last_detection_is_fresh() or self._target_is_fresh()
            if not fresh:
                if lost_start is None:
                    lost_start = time.time()
                elif (time.time() - lost_start) > self.target_lost_abort_s:
                    self.get_logger().warn("[DETER] Lost target during orientation.")
                    self.stop_robot()
                    return False

                self.stop_robot()
                time.sleep(dt)
                continue

            lost_start = None

            err = self.compute_center_error_px()
            if err is None:
                self.stop_robot()
                time.sleep(dt)
                continue

            wz = self.compute_turn_cmd_from_err(err)

            if wz == 0.0:
                centered_count += 1
                self.stop_robot()
                if centered_count >= self.center_hold_cycles:
                    self.get_logger().info("[DETER] Target centered.")
                    return True
            else:
                centered_count = 0
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = wz
                self.cmd_pub.publish(cmd)

            time.sleep(dt)

        self.stop_robot()
        return False

    def deterrence_sequence(self):
        pest_name = CLASS_MAP.get(self.last_class or "", self.last_class or "unknown")
        conf = self.last_score

        self.deterring = True
        self.get_logger().info(
            f"[DETER] Interrupting navigation -> DETERRING {pest_name.upper()} (conf={conf:.2f})"
        )

        self.cancel_current_goal()
        self.stop_robot()

        # 1) Try to face the pest quickly
        centered = self.orient_to_target()
        if not centered:
            self.get_logger().warn("[DETER] Proceeding with sound despite failed centering.")

        # 2) Play sound
        self.sound_playing = True
        sound_thread = threading.Thread(
            target=self.play_deterrent_sound,
            args=(pest_name,),
            daemon=True
        )
        sound_thread.start()

        # Hold still while sound plays
        while rclpy.ok() and self.sound_playing:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.stop_robot()
            time.sleep(0.05)

        self.stop_robot()

        # 3) Wait and check if pest is still there
        still_present = self.pest_still_present_after_cooldown(pest_name)

        self.stop_robot()
        self.deterring = False
        self.interrupt_requested = False

        if still_present:
            self.get_logger().warn("[DETER] Pest still present -> re-triggering deterrence")
            self.interrupt_requested = True
        else:
            self.get_logger().info("[DETER] Pest gone -> resuming navigation")

    # ----------------------------
    # Navigation loop
    # ----------------------------
    def send_goal_and_wait_with_interrupts(self, pose: PoseStamped, timeout_sec: float = 300.0) -> bool:
        self.current_goal_pose = pose

        while rclpy.ok():
            goal = NavigateToPose.Goal()
            goal.pose = pose

            self.get_logger().info(
                f"Sending goal: frame={pose.header.frame_id} "
                f"x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f}"
            )

            send_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            self.goal_handle = send_future.result()

            if self.goal_handle is None or not self.goal_handle.accepted:
                self.get_logger().error("Goal rejected.")
                return False

            self.get_logger().info("Goal accepted.")

            result_future = self.goal_handle.get_result_async()
            start = time.time()

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.interrupt_requested and (not self.deterring):
                    self.deterrence_sequence()
                    break

                if result_future.done():
                    res = result_future.result()
                    if res is None:
                        self.get_logger().error("No result received.")
                        return False

                    status = res.status
                    if status == 4:
                        self.get_logger().info("SUCCEEDED")
                        return True
                    else:
                        self.get_logger().error(f"FAILED with status={status}")
                        return False

                if (time.time() - start) > timeout_sec:
                    self.get_logger().error("Timed out waiting for result.")
                    return False

            # deterrence happened, resend same goal
            continue

        return False

    def destroy_node(self):
        try:
            pygame.mixer.music.stop()
            pygame.mixer.quit()
        except Exception:
            pass
        super().destroy_node()


def main():
    NAMESPACE = "/a200_1071"

    GOALS: List[Tuple[float, float, float]] = [
        (-1.82, 0.322, 90.0),
        (76.7, -7.18, -90.0),
        (76.4, -9.48, 180.0),
	(-1.91,-2.3,90)
    ]

    rclpy.init()
    node = Nav2GoalRunner(namespace=NAMESPACE)

    if not node.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("Is Nav2 running? Exiting.")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        return

    ok_all = True
    for i, (x, y, yaw) in enumerate(GOALS, start=1):
        pose = node.make_goal(x, y, yaw, frame_id="map")
        node.get_logger().info(f"=== Goal {i}/{len(GOALS)} ===")
        ok = node.send_goal_and_wait_with_interrupts(pose, timeout_sec=300.0)
        if not ok:
            ok_all = False
            node.get_logger().error("Stopping sequence due to failure.")
            break

    node.get_logger().info("All done." if ok_all else "Done (with failures).")

    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
