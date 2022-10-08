import array
import os
import sys

import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# YOLOv5 checked out at the commit the weed model was trained with
# (see part2_detection/weed_detection). best.pt needs its modules on the path to load
sys.path.insert(0, os.environ.get('YOLOV5_DIR', '/opt/yolov5'))
from models.common import DetectMultiBackend  # noqa: E402
from utils.augmentations import letterbox  # noqa: E402
from utils.general import non_max_suppression, scale_coords  # noqa: E402


# Looks for weeds in the USB webcam that is tilted at the ground and keeps
# the sprayer pump on while a weed is in view
class WeedDetector(Node):
    def __init__(self):
        super().__init__('weed_detector')
        weights = os.path.expanduser(
            self.declare_parameter('weights', '~/agridoc_models/best.pt').value)
        camera = self.declare_parameter('camera', '/dev/video1').value
        self.img_size = self.declare_parameter('img_size', 640).value
        self.confidence = self.declare_parameter('confidence', 0.4).value
        # keep spraying this long after the last weed was seen
        self.hold_time = self.declare_parameter('hold_time', 1.0).value

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.model = DetectMultiBackend(weights, device=self.device, fp16=self.device.type != 'cpu')
        self.model.warmup(imgsz=(1, 3, self.img_size, self.img_size))
        self.names = self.model.names

        self.cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # only the newest frame, old ones would spray where the weed was
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open the weed camera %s' % camera)

        self.last_weed = None
        self.spraying = False
        self.pump_pub = self.create_publisher(Bool, 'pump', 10)
        self.image_pub = self.create_publisher(Image, 'weed_camera/detections', 1)
        self.create_timer(0.01, self.step)
        self.get_logger().info('Weed detector ready on %s (%s)' % (camera, self.device))

    def detect(self, frame):
        # same steps as detect.py in YOLOv5
        img = letterbox(frame, self.img_size, stride=self.model.stride, auto=True)[0]
        img = np.ascontiguousarray(img.transpose((2, 0, 1))[::-1])
        im = torch.from_numpy(img).to(self.device)
        im = im.half() if self.model.fp16 else im.float()
        im = im[None] / 255.0
        with torch.no_grad():
            pred = self.model(im)
        det = non_max_suppression(pred, self.confidence, 0.45)[0]
        if len(det):
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], frame.shape).round()
        # one row per weed: x1 y1 x2 y2 confidence class
        return det.cpu().numpy()

    def step(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('No frame from the weed camera', throttle_duration_sec=5.0)
            return
        weeds = self.detect(frame)

        now = self.get_clock().now()
        if len(weeds) > 0:
            self.last_weed = now
        spraying = (self.last_weed is not None and
                    (now - self.last_weed).nanoseconds / 1e9 < self.hold_time)
        if spraying != self.spraying:
            if spraying:
                found = ', '.join(self.names[int(c)] for c in weeds[:, 5])
                self.get_logger().info('Spraying: %s' % found)
            else:
                self.get_logger().info('Stopped spraying')
            self.spraying = spraying
        # published every frame, the bridge turns the pump off if these stop
        self.pump_pub.publish(Bool(data=spraying))

        if self.image_pub.get_subscription_count() > 0:
            self.publish_image(frame, weeds, now)

    def publish_image(self, frame, weeds, now):
        for x1, y1, x2, y2, conf, cls in weeds:
            label = '%s %.2f' % (self.names[int(cls)], conf)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(frame, label, (int(x1), max(int(y1) - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        msg = Image()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'weed_camera_optical_link'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = frame.shape[1] * 3
        msg.data = array.array('B', frame.tobytes())
        self.image_pub.publish(msg)

def main():
    rclpy.init()
    node = WeedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # no pump message is sent here, the bridge switches the pump off when they stop coming
    node.cap.release()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
