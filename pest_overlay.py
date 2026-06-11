import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2

class Overlay(Node):
    def __init__(self):
        super().__init__('pest_overlay')
        self.bridge = CvBridge()
        self.last = None
        self.hdr = None

        self.create_subscription(Image,
                                 '/camera/camera/color/image_raw',
                                 self.on_img, 10)

        self.create_subscription(Detection2DArray,
                                 '/pest_detections',
                                 self.on_det, 10)

        self.pub = self.create_publisher(Image,
                                         '/pest_overlay_image', 10)

        self.get_logger().info('Overlay running')

    def on_img(self, msg):
        self.hdr = msg.header
        self.last = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def on_det(self, msg):
        if self.last is None:
            return

        img = self.last.copy()

        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y

            x1 = int(cx - w/2)
            y1 = int(cy - h/2)
            x2 = int(cx + w/2)
            y2 = int(cy + h/2)

            cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)

            if det.results:
                hyp = det.results[0].hypothesis
                cv2.putText(img,
                            f"{hyp.class_id} {hyp.score:.2f}",
                            (x1, max(0, y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0,255,0),
                            2)

        out = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        out.header = self.hdr
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(Overlay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
