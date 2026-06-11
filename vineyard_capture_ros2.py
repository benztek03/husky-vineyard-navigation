import os
import cv2
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from datetime import datetime

IMAGE_TOPIC = "/camera/camera/color/image_raw"
PEST_TOPIC = "/pest_detections"
NORMAL_INTERVAL = 10.0
ALERT_INTERVAL = 0.5
SAVE_BASE_DIR = "/home/administrator/Vineyard_Images"

MIN_CONFIDENCE = 0.60
DETECTION_ON_DELAY = 0.75
DETECTION_HOLD_TIME = 2.0


class VineyardCapture(Node):
    def __init__(self):
        super().__init__("vineyard_capture")

        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(SAVE_BASE_DIR, run_name)
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(f"Saving images to: {self.save_dir}")

        self.bridge = CvBridge()
        self.latest_frame = None

        self.frame_counter = 0
        self.last_saved_frame_counter = -1
        self.last_image_rx_time = 0.0

        self.alert_mode = False
        self.last_save_time = 0.0
        self.image_count = 0

        self.first_good_detection_time = None
        self.last_good_detection_time = None

        self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Detection2DArray,
            PEST_TOPIC,
            self.pest_callback,
            10
        )

        self.create_timer(0.1, self.timer_callback)
        self.create_timer(2.0, self.watchdog_callback)

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.frame_counter += 1
            self.last_image_rx_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def detection_is_good(self, msg):
        for det in msg.detections:
            for result in det.results:
                if result.hypothesis.score >= MIN_CONFIDENCE:
                    return True
        return False

    def pest_callback(self, msg):
        now = time.time()
        good_detection = self.detection_is_good(msg)

        if good_detection:
            self.last_good_detection_time = now
            if self.first_good_detection_time is None:
                self.first_good_detection_time = now
        else:
            self.first_good_detection_time = None

        previous_mode = self.alert_mode

        if self.first_good_detection_time is not None:
            if (now - self.first_good_detection_time) >= DETECTION_ON_DELAY:
                self.alert_mode = True

        if self.alert_mode and self.last_good_detection_time is not None:
            if (now - self.last_good_detection_time) > DETECTION_HOLD_TIME:
                self.alert_mode = False
                self.first_good_detection_time = None

        if self.alert_mode != previous_mode:
            mode = "ALERT" if self.alert_mode else "NORMAL"
            self.get_logger().info(f"Switched to {mode} mode")

    def timer_callback(self):
        if self.latest_frame is None:
            return

        now = time.time()
        interval = ALERT_INTERVAL if self.alert_mode else NORMAL_INTERVAL

        if self.last_save_time == 0.0 or (now - self.last_save_time) >= interval:
            if self.frame_counter != self.last_saved_frame_counter:
                self.save_image()
                self.last_save_time = now
                self.last_saved_frame_counter = self.frame_counter

    def watchdog_callback(self):
        now = time.time()
        if self.last_image_rx_time == 0.0:
            self.get_logger().warn("No images received yet from camera topic")
            return

        age = now - self.last_image_rx_time
        if age > 2.0:
            self.get_logger().warn(
                f"No new camera frames for {age:.1f} sec on {IMAGE_TOPIC}"
            )

    def save_image(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        mode = "alert" if self.alert_mode else "normal"
        filename = f"{mode}_{timestamp}.jpg"
        filepath = os.path.join(self.save_dir, filename)

        ok = cv2.imwrite(filepath, self.latest_frame)
        if ok:
            self.image_count += 1
            self.get_logger().info(
                f"Saved image {self.image_count}: {filepath}"
            )
        else:
            self.get_logger().error(f"Failed to save: {filepath}")


def main(args=None):
    rclpy.init(args=args)
    node = VineyardCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
