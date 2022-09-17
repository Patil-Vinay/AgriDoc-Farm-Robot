#!/usr/bin/env python3
# Train the tomato leaf disease CNN. The data folder has one sub folder per class.
import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models

IMAGE_SIZE = 256
CHANNELS = 3


def list_images(data_dir):
    class_names = sorted(d for d in os.listdir(data_dir) if os.path.isdir(os.path.join(data_dir, d)))
    paths, labels = [], []
    for label, name in enumerate(class_names):
        folder = os.path.join(data_dir, name)
        for f in sorted(os.listdir(folder)):
            if f.lower().endswith(('.jpg', '.jpeg', '.png')):
                paths.append(os.path.join(folder, f))
                labels.append(label)
    return class_names, np.array(paths), np.array(labels)


def load_image(path, label):
    img = tf.io.decode_image(tf.io.read_file(path), channels=CHANNELS, expand_animations=False)
    img = tf.image.resize(img, (IMAGE_SIZE, IMAGE_SIZE))
    return img, label


def make_dataset(paths, labels, batch_size, shuffle):
    ds = tf.data.Dataset.from_tensor_slices((paths, labels))
    if shuffle:
        ds = ds.shuffle(len(paths), reshuffle_each_iteration=True)
    ds = ds.map(load_image, num_parallel_calls=tf.data.AUTOTUNE)
    return ds.batch(batch_size).prefetch(tf.data.AUTOTUNE)


def build_model(n_classes):
    resize_and_rescale = tf.keras.Sequential([
        layers.Resizing(IMAGE_SIZE, IMAGE_SIZE),
        layers.Rescaling(1.0 / 255),
    ])
    data_augmentation = tf.keras.Sequential([
        layers.RandomFlip('horizontal_and_vertical'),
        layers.RandomRotation(0.2),
    ])
    model = models.Sequential([
        resize_and_rescale,
        data_augmentation,
        layers.Conv2D(32, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Flatten(),
        layers.Dense(64, activation='relu'),
        layers.Dense(n_classes, activation='softmax'),
    ])
    model.build(input_shape=(None, IMAGE_SIZE, IMAGE_SIZE, CHANNELS))
    return model


def plot_history(history, path):
    acc = history.history['accuracy']
    val_acc = history.history['val_accuracy']
    loss = history.history['loss']
    val_loss = history.history['val_loss']
    epochs = range(1, len(acc) + 1)

    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(epochs, acc, label='Training Accuracy')
    plt.plot(epochs, val_acc, label='Validation Accuracy')
    plt.legend(loc='lower right')
    plt.title('Training and Validation Accuracy')

    plt.subplot(1, 2, 2)
    plt.plot(epochs, loss, label='Training Loss')
    plt.plot(epochs, val_loss, label='Validation Loss')
    plt.legend(loc='upper right')
    plt.title('Training and Validation Loss')
    plt.savefig(path)
    plt.close()


def plot_predictions(model, test_ds, class_names, path):
    images, labels = next(iter(test_ds))
    probs = model.predict(images)
    n = min(9, len(images))

    plt.figure(figsize=(15, 15))
    for i in range(n):
        plt.subplot(3, 3, i + 1)
        plt.imshow(images[i].numpy().astype('uint8'))
        plt.title('Actual: {}\nPredicted: {}\nConfidence: {:.2f}%'.format(
            class_names[labels[i]], class_names[np.argmax(probs[i])], 100 * np.max(probs[i])))
        plt.axis('off')
    plt.savefig(path)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Train the tomato disease classifier')
    parser.add_argument('data_dir', help='folder with one sub folder of images per class')
    parser.add_argument('--model-dir', default='model', help='where the SavedModel is written')
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--val-split', type=float, default=0.1)
    parser.add_argument('--test-split', type=float, default=0.1)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    class_names, paths, labels = list_images(args.data_dir)
    print('Found {} images in {} classes'.format(len(paths), len(class_names)))

    # split the file list once so the three sets never share an image
    order = np.random.RandomState(args.seed).permutation(len(paths))
    paths, labels = paths[order], labels[order]
    n_test = int(len(paths) * args.test_split)
    n_val = int(len(paths) * args.val_split)
    test = slice(0, n_test)
    val = slice(n_test, n_test + n_val)
    train = slice(n_test + n_val, None)

    train_ds = make_dataset(paths[train], labels[train], args.batch_size, shuffle=True)
    val_ds = make_dataset(paths[val], labels[val], args.batch_size, shuffle=False)
    test_ds = make_dataset(paths[test], labels[test], args.batch_size, shuffle=False)
    print('Training: {}  Validation: {}  Test: {}'.format(len(paths[train]), n_val, n_test))

    model = build_model(len(class_names))
    model.summary()
    model.compile(
        optimizer='adam',
        loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=False),
        metrics=['accuracy'],
    )
    history = model.fit(train_ds, epochs=args.epochs, validation_data=val_ds)

    loss, acc = model.evaluate(test_ds)
    print('Test loss {:.4f}  accuracy {:.4f}'.format(loss, acc))

    model.save(args.model_dir)
    with open(os.path.join(args.model_dir, 'class_names.txt'), 'w') as f:
        f.write('\n'.join(class_names) + '\n')
    plot_history(history, os.path.join(args.model_dir, 'training.png'))
    plot_predictions(model, test_ds, class_names, os.path.join(args.model_dir, 'predictions.png'))
    print('Saved model to ' + args.model_dir)


if __name__ == '__main__':
    main()
