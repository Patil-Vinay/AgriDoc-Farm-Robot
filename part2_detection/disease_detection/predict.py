#!/usr/bin/env python3
# Predict the disease of tomato leaf images with a trained model.
import argparse
import os

import numpy as np
import tensorflow as tf

IMAGE_SIZE = 256


def main():
    parser = argparse.ArgumentParser(description='Predict tomato leaf diseases')
    parser.add_argument('model_dir', help='SavedModel folder written by train.py')
    parser.add_argument('images', nargs='+', help='leaf images')
    parser.add_argument('--classes', help='class names file (default: class_names.txt in model_dir)')
    args = parser.parse_args()

    classes_file = args.classes or os.path.join(args.model_dir, 'class_names.txt')
    with open(classes_file) as f:
        class_names = [line.strip() for line in f if line.strip()]

    model = tf.keras.models.load_model(args.model_dir)

    for path in args.images:
        img = tf.keras.utils.load_img(path, target_size=(IMAGE_SIZE, IMAGE_SIZE))
        batch = tf.expand_dims(tf.keras.utils.img_to_array(img), 0)
        probs = model.predict(batch)[0]
        print('{}: {} ({:.2f}%)'.format(path, class_names[np.argmax(probs)], 100 * np.max(probs)))


if __name__ == '__main__':
    main()
