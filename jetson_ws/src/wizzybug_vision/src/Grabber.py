import pyrealsense2 as rs
import numpy as np
import cv2

import sys
import os


class Grabber(object):

    def __init__(self, cam_serial, width, height, framerate):
        # save parameters
        self.cam_serial, self.width, self.height, self.framerate = cam_serial, width, height, framerate
        
        # create pipeline
        self.pipeline = rs.pipeline()

        # create a config object
        self.config = rs.config()

        # enable device
        self.config.enable_device(cam_serial)

        # to mark which streams are to be grabbed
        self.depth, self.color = False, False

    def start(self, depth=True, color=True, record_filename=None):
        # remember what we're streaming
        self.depth, self.color = depth, color

        if depth:
            self.config.enable_stream(rs.stream.depth, self.width, self.height,
                                      rs.format.z16, self.framerate)
        
        if color:
            self.config.enable_stream(rs.stream.color, self.width, self.height,
                                      rs.format.rgb8, self.framerate)

        if record_filename is not None:
            self.config.enable_record_to_file(record_filename)

        # Start streaming from file
        self.pipeline.start(self.config)

    def grab(self):
        """ return depth and color """

        # to hold result
        result = dict()

        # Get frames
        frames = self.pipeline.wait_for_frames()

        # Get depth frame
        if self.depth:
            depth_frame = frames.get_depth_frame()
            result["depth"] = np.asanyarray(depth_frame.get_data())

        # Get color frame
        if self.color:
            color_frame = frames.get_color_frame()
            result["color"] = np.asanyarray(color_frame.get_data())

        return result

    def stop(self):
        self.pipeline.stop()


class FileGrabber(Grabber):
    def __init__(self, filename):
        # regular stream grabber init
        super().__init__()

        # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
        rs.config.enable_device_from_file(self.config, filename)


def start_cameras(width, height, framerate, depth=True, color=False):
    # realsense context
    realsense_ctx = rs.context()

    # initialize grabbers
    grabbers = dict()

    # iterate over connected devices
    for cam_serial in range(len(realsense_ctx.devices)):

        # current serial number
        cam_serial = realsense_ctx.devices[cam_serial].get_info(rs.camera_info.serial_number)

        grabbers[cam_serial] = Grabber(cam_serial=cam_serial,
                                       width=width, height=height, framerate=framerate)
        # start streams
        grabbers[cam_serial].start(depth=depth, color=color)


if __name__ == '__main__':
    start_cameras(width=640, height=480, framerate=15)

    print("helllloooo")





