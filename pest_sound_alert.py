import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from playsound import playsound
import threading
import time


class PestSoundAlert(Node):

    def __init__(self):
        super().__init__('pest_sound_alert')

        # ---- DETECTION FILTER SETTINGS ----
        self.conf_threshold = 0.4
        self.required_hits = 3
        self.time_window = 2.0  # seconds

        self.detection_times = []

        # ---- SOUND SETTINGS ----
        self.sound_file = "/home/administrator/sounds/hawk.wav"
        self.sound_duration = 10.0   # seconds playing
        self.sound_cooldown = 5.0    # seconds silence

        self.pest_present = False
        self.sound_playing = False
        self.last_cycle_end = 0.0

        # ---- SUBSCRIBER ----
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/pest_detections',
            self.detection_callback,
            10
        )

        # Timer to manage sound state machine
        self.timer = self.create_timer(0.1, self.sound_manager)

        self.get_logger().info("Pest Sound Alert Node Started")


    # --------------------------------------------------
    # Detection Logic
    # --------------------------------------------------
    def detection_callback(self, msg):

        current_time = time.time()
        valid_detection = False

        for det in msg.detections:
            for result in det.results:
                if float(result.hypothesis.score) >= self.conf_threshold:
                    valid_detection = True
                    break

        if valid_detection:
            self.detection_times.append(current_time)

        # Keep only detections in time window
        self.detection_times = [
            t for t in self.detection_times
            if current_time - t <= self.time_window
        ]

        # Update pest state
        if len(self.detection_times) >= self.required_hits:
            self.pest_present = True
        else:
            self.pest_present = False


    # --------------------------------------------------
    # Sound State Machine
    # --------------------------------------------------
    def sound_manager(self):

        current_time = time.time()

        # If pest gone → stop immediately
        if not self.pest_present:
            self.sound_playing = False
            return

        # If sound currently playing → let thread finish
        if self.sound_playing:
            return

        # Cooldown check
        if current_time - self.last_cycle_end >= self.sound_cooldown:
            self.start_sound_thread()


    # --------------------------------------------------
    # Play Sound in Background Thread
    # --------------------------------------------------
    def start_sound_thread(self):

        self.sound_playing = True

        thread = threading.Thread(target=self.play_sound_cycle)
        thread.daemon = True
        thread.start()


    def play_sound_cycle(self):

        self.get_logger().info("Playing alert sound")

        start_time = time.time()

        # Play sound repeatedly for 10 seconds
        while time.time() - start_time < self.sound_duration and self.pest_present:
            playsound(self.sound_file)

        self.get_logger().info("Sound cycle complete")

        self.sound_playing = False
        self.last_cycle_end = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = PestSoundAlert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
