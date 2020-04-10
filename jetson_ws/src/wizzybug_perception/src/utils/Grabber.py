import pyrealsense2 as rs
import numpy as np
import cv2

import sys
import os

# PACKAGE_PARENT = '..'
# SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
# sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
# from ..wizzy_vision import DEPTH_WIDTH, DEPTH_HEIGHT

DEPTH_WIDTH, DEPTH_HEIGHT = 640, 480


class Grabber(object):
    """ read depth and color from file or stream """
    def __init__(self):
        # create pipeline
        self.pipeline = rs.pipeline()

        # create a config object
        self.config = rs.config()

        # Create colorizer object
        self.colorizer = rs.colorizer()

    def start(self, record_filename=None):
        # configure the pipeline to stream the depth stream
        self.config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.rgb8, 30)

        if record_filename is not None:
            self.config.enable_record_to_file(record_filename)

        # Start streaming from file
        self.pipeline.start(self.config)

    def grab(self):
        """ return depth and color """

        # Get frameset of depth
        frames = self.pipeline.wait_for_frames()

        # Get depth frame
        depth_frame = frames.get_depth_frame()

        # Get color frame
        color_frame = frames.get_color_frame()

        # Colorize depth frame to jet colormap
        depth_color_frame = self.colorizer.colorize(depth_frame)

        # Convert to numpy arrays
        return np.asanyarray(depth_frame.get_data()), np.asanyarray(color_frame.get_data()), \
               np.asanyarray(depth_color_frame.get_data())


class FileGrabber(Grabber):
    def __init__(self, filename):
        # regular stream grabber init
        super().__init__()

        # Tell config that we will use a recorded device from filem to be used by the pipeline through playback.
        rs.config.enable_device_from_file(self.config, filename)




