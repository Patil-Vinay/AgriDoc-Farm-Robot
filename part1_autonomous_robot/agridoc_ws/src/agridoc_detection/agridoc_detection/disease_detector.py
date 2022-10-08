import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
import tensorflow as tf
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

IMAGE_SIZE = 256

# IMX219 on the CSI port, full sensor at 30 fps scaled down on the GPU
CSI_PIPELINE = ('nvarguscamerasrc sensor-id=%d ! '
                'video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1 ! '
                'nvvidconv ! video/x-raw, width=820, height=616, format=BGRx ! '
                'videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1')


# Classifies the plants in front of the top camera with the leaf disease model
# and logs every disease it finds with the robot's position on the map
class DiseaseDetector(Node):
    def __init__(self):
        super().__init__('disease_detector')
        model_dir = os.path.expanduser(
            self.declare_parameter('model_dir', '~/agridoc_models/disease_model').value)
        classes_file = self.declare_parameter('classes_file', '').value
        sensor_id = self.declare_parameter('sensor_id', 0).value
        period = self.declare_parameter('period', 1.0).value
        self.confidence = self.declare_parameter('confidence', 0.8).value
        # the same disease closer than this to the last one logged is the same plant
        self.min_distance = self.declare_parameter('min_distance', 0.5).value
        log_file = self.declare_parameter('log_file', '~/.ros/agridoc/disease_log.csv').value

        # TF takes the whole GPU memory by default, the weed detector needs some too
        for gpu in tf.config.list_physical_devices('GPU'):
            tf.config.experimental.set_memory_growth(gpu, True)

        # train.py writes class_names.txt next to the SavedModel
        with open(classes_file or os.path.join(model_dir, 'class_names.txt')) as f:
            self.class_names = [line.strip() for line in f if line.strip()]
        # only for predictions, the optimizer state saved with it is not needed
        self.model = tf.keras.models.load_model(model_dir, compile=False)

        self.cap = cv2.VideoCapture(CSI_PIPELINE % sensor_id, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open the IMX219 camera')

        self.log_file = os.path.expanduser(log_file)
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                csv.writer(f).writerow(['time', 'x', 'y', 'disease', 'confidence'])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_logged = {}
        self.create_timer(period, self.step)
        self.get_logger().info('Disease detector ready, logging to %s' % self.log_file)

    def classify(self, frame):
        # PlantVillage images are square, so take the middle square of the frame.
        # The model scales on its own, it takes RGB pixels from 0 to 255
        h, w = frame.shape[:2]
        side = min(h, w)
        top, left = (h - side) // 2, (w - side) // 2
        rgb = cv2.cvtColor(frame[top:top + side, left:left + side], cv2.COLOR_BGR2RGB)
        img = tf.image.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))
        probs = self.model(img[None], training=False).numpy()[0]
        best = int(np.argmax(probs))
        return self.class_names[best], float(probs[best])

    def robot_position(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def step(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('No frame from the IMX219', throttle_duration_sec=5.0)
            return
        name, conf = self.classify(frame)
        if name.endswith('healthy') or conf < self.confidence:
            return

        position = self.robot_position()
        if position is None:
            self.get_logger().warn('%s seen but there is no map position yet' % name,
                                   throttle_duration_sec=5.0)
            return
        last = self.last_logged.get(name)
        if last is not None:
            if math.hypot(position[0] - last[0], position[1] - last[1]) < self.min_distance:
                return
        self.last_logged[name] = position

        with open(self.log_file, 'a', newline='') as f:
            csv.writer(f).writerow([time.strftime('%Y-%m-%d %H:%M:%S'),
                                    '%.2f' % position[0], '%.2f' % position[1],
                                    name, '%.2f' % conf])
        self.get_logger().info('%s (%.0f%%) at x=%.2f y=%.2f'
                               % (name, 100 * conf, position[0], position[1]))


def main():
    rclpy.init()
    node = DiseaseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
