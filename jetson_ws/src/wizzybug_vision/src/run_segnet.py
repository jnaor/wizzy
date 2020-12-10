#!/usr/bin/python2

import sys

import jetson.inference
import jetson.utils
import segnet_utils 

import argparse

import rospy
from cv_bridge import CvBridge, CvBridgeError

# inputs are images
from sensor_msgs.msg import Image

import numpy as np
import cv2

# callback. do this when a new image arrives
def callback(msg):

    # get numpy image from ROS topic
    img = bridge.imgmsg_to_cv2(msg)

    cv2.imshow("img", img)
    cv2.waitKey(1)

    rospy.logdebug("new image received")

    # convert to CUDA
    img_input = jetson.utils.cudaFromNumpy(img.astype(np.float32))

    # allocate buffers for this size image
    buffers.Alloc(img_input.shape, img_input.format)

    # process the segmentation network
    net.Process(img_input, ignore_class=opt.ignore_class)

    # generate the overlay
    if buffers.overlay:
        print('overlay')
        net.Overlay(buffers.overlay, filter_mode=opt.filter_mode)

    # generate the mask
    if buffers.mask:
        print('mask')
        net.Mask(buffers.mask, filter_mode=opt.filter_mode)

    # composite the images
    if buffers.composite:
        jetson.utils.cudaOverlay(buffers.overlay, buffers.composite, 0, 0)
        jetson.utils.cudaOverlay(buffers.mask, buffers.composite, buffers.overlay.width, 0)

    # render the output image
    output.Render(buffers.output)

    # update the title bar
    output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

    # print out performance info
    jetson.utils.cudaDeviceSynchronize()
    net.PrintProfilerTimes()

    # compute segmentation class stats
    if opt.stats:
        buffers.ComputeStats()


if __name__ == '__main__':
    # parse the command line
    parser = argparse.ArgumentParser(description="Segment a live camera stream using an semantic segmentation DNN.", 
                                    formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.segNet.Usage() +
                                    jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

    parser.add_argument("--network", type=str, default="fcn-resnet18-voc", help="pre-trained model to load, see below for options")
    parser.add_argument("--filter-mode", type=str, default="linear", choices=["point", "linear"], help="filtering mode used during visualization, options are:\n  'point' or 'linear' (default: 'linear')")
    parser.add_argument("--visualize", type=str, default="overlay", help="Visualization options (can be 'overlay' 'mask' 'overlay,mask'")
    parser.add_argument("--ignore-class", type=str, default="void", help="optional name of class to ignore in the visualization results (default: 'void')")
    parser.add_argument("--alpha", type=float, default=150.0, help="alpha blending value to use during overlay, between 0.0 and 255.0 (default: 150.0)")
    parser.add_argument("--stats", action="store_true", help="compute statistics about segmentation mask class output")
    parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")

    is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

    try:
        opt = parser.parse_known_args()[0]
    except:
        print("")
        parser.print_help()
        sys.exit(0)

    # load the segmentation network
    net = jetson.inference.segNet(opt.network, sys.argv)

    # set the alpha blending value
    net.SetOverlayAlpha(opt.alpha)

    # create buffer manager
    buffers = segnet_utils.segmentationBuffers(net, opt)

    # opencv "bridge" for incoming images
    bridge = CvBridge()

    # initialize ROS node
    rospy.init_node('run_segnet', log_level=rospy.DEBUG)

    # input topic to listen to
    input_topic = rospy.get_param('input_topic', '/camera/color/image_raw')

    rospy.loginfo('listening for input images topic {}'.format(input_topic))
    rospy.Subscriber(input_topic, Image, callback)

    # create video output
    output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)

    # start spinning
    rospy.spin()
