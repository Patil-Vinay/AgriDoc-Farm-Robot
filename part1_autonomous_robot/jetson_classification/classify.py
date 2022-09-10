#!/usr/bin/env python3
# Classify images, video or a camera feed on the Jetson Nano with jetson-inference.
import argparse

import jetson.inference
import jetson.utils

parser = argparse.ArgumentParser(description='Classify plants with imageNet')
parser.add_argument('input', help='image, folder, video or camera (csi://0, /dev/video0)')
parser.add_argument('output', nargs='?', default='display://0', help='output file or display://0')
parser.add_argument('--network', default='googlenet', help='pretrained network (googlenet, resnet-18)')
parser.add_argument('--model', help='retrained ONNX model, used instead of --network')
parser.add_argument('--labels', help='labels.txt of the retrained model')
args = parser.parse_args()

if args.model:
    if not args.labels:
        parser.error('--model needs --labels')
    net = jetson.inference.imageNet(argv=['--model=' + args.model,
                                          '--labels=' + args.labels,
                                          '--input_blob=input_0',
                                          '--output_blob=output_0'])
else:
    net = jetson.inference.imageNet(args.network)

source = jetson.utils.videoSource(args.input)
output = jetson.utils.videoOutput(args.output)
font = jetson.utils.cudaFont()

while True:
    img = source.Capture()
    if img is None:
        continue

    class_id, confidence = net.Classify(img)
    label = net.GetClassDesc(class_id)
    text = '{:05.2f}% {}'.format(confidence * 100, label)
    print(text)

    font.OverlayText(img, img.width, img.height, text, 5, 5, font.White, font.Gray40)
    output.Render(img)
    output.SetStatus('{} | {:.0f} FPS'.format(net.GetNetworkName(), net.GetNetworkFPS()))

    if not source.IsStreaming() or not output.IsStreaming():
        break
