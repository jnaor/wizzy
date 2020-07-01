import os
import numpy as np
import cv2
import logging
import json

from skimage.measure import label, regionprops
from sklearn.cluster import MeanShift, estimate_bandwidth

from matplotlib import pylab as plt
import matplotlib.patches as mpatches

from scipy.stats import mode

# percentile to take as distance estimation for cluster
CLUSTER_PERCENTILE = 5

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255),
                       (128, 0, 0), (0, 128, 0), (0, 0, 128),
                  (128, 0, 128),
                  (128, 128, 0), (0, 128, 128)]

labels = {0: 'other', 1: 'wall', 2: 'floor', 3: 'cabinet/shelves',
            4: 'bed/pillow', 5: 'chair', 6: 'sofa', 7: 'table',
            8: 'door', 9: 'window', 10: 'picture/tv', 11: 'blinds/curtain',
            12: 'clothes', 13: 'ceiling', 14: 'books', 15: 'fridge',
            16: 'person', 17: 'toilet', 18: 'sink', 19: 'lamp',
            20: 'bathtub'}


def calc_attributes(depth_image, bounding_box, distance, fov=[40, 48], percentile=10):
    """ approximate distance of object based upon its bounding box in depth image
    :param depth_image assumed to be in the same resolution and viewpoint bounding box was taken from
    :param bounding_box in [r1, c1, r2, c2] format
    :param percentile of depth values. object will usually not fill all of its BB. also avoid noise"""

    # get bb coordinates
    x1, y1, x2, y2 = bounding_box

    # estimate how much of field of view we are using
    pitch = ((y2 - y1) / depth_image.shape[0]) * fov[0]
    yaw = ((x2 - x1) / depth_image.shape[1]) * fov[1]

    # now get height and width according to depth
    width = 2 * distance * np.tan(np.deg2rad(yaw / 2.0))
    height = 2 * distance * np.tan(np.deg2rad(pitch / 2.0))
    length = 0

    # calculate x-location of center (in mm)
    x = distance * np.tan(np.deg2rad((((y1 + y2) / 2.0 - depth_image.shape[0] / 2.0) / depth_image.shape[0]) * fov[0]))
    y = distance
    z = 0

    f = 1/1000.0

    # return. notice the usual coordinates drek and mm->m conversion
    return f*y, -x*f, z*f, f*width, f*height, length*f


def cluster_depth(D, min_range=30, max_range=5500, min_area_percentage=0.2, num_decimations=3):

    # if by some off-chance depth image is not one channel then make it so
    if len(D.shape) > 2:
        D = cv2.cvtColor(D, cv2.COLOR_BGR2GRAY)

    # to hold result
    obstacle_list, obstacle_mask = list(), list()

    # to hold decimated image
    C = None

    # decimate
    for ind in range(num_decimations):
        if C is None:
            C = cv2.pyrDown(D)
        else:
            C = cv2.pyrDown(C)

    # indices of valid depth
    valid_depth_indices = C.nonzero()

    # get rid of zeros
    X = C[valid_depth_indices]

    # mean-shift likes it's data this way
    X = X.reshape(-1, 1)

    # estimate band-width for mean-shift
    bandwidth = estimate_bandwidth(X, quantile=0.2, n_samples=500)

    # handle degenerate case
    if bandwidth <= 0:
        bandwidth = 100

    # clustering
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(X)

    # number of detected clusters
    num_clusters = len(np.unique(ms.labels_))

    # for each cluster
    for label in range(num_clusters):

        # ignore distance 0 and far objects
        if ms.cluster_centers_[label][0] < min_range or ms.cluster_centers_[label][0] > max_range:
            continue

        # mask for this label
        B = np.zeros_like(C).astype(np.bool)
        B[valid_depth_indices[0][ms.labels_ == label], valid_depth_indices[1][ms.labels_ == label]] = True

        # if area too small disregard this
        if(np.count_nonzero(B) < B.shape[0]*B.shape[1]*min_area_percentage):
            continue

        cv2.imshow("B", 255*B.astype(np.uint8))
        cv2.waitKey(1)

        # distance to closest element (take a percentile to avoid noise)
        distance = np.percentile(C[B], CLUSTER_PERCENTILE, interpolation='nearest')
        if distance < min_range:
            continue

        print(distance)


        # find bounding values
        nonzero_indices = np.where(B)
        bb_y_min, bb_y_max, _, _ = cv2.minMaxLoc(nonzero_indices[0])
        bb_x_min, bb_x_max, _, _  = cv2.minMaxLoc(nonzero_indices[1])

        # bounding box in original image
        bounding_box = np.array([bb_x_min, bb_y_min, bb_x_max, bb_y_max]) * 2**num_decimations

        # to hold results for this object
        detection = dict()

        # no classification yet
        detection['classification'] = 'unlabeled'

        # save mask for debug purposes
        obstacle_mask.append(B)

        # get location attributes
        detection['x'], detection['y'], detection['z'], detection['width'], detection['height'], detection['length'] = \
            calc_attributes(D, bounding_box, distance=distance)

        # add to result
        obstacle_list.append(detection)

    # for ind, mask in enumerate(obstacle_mask):
    #     cv2.imshow(str(ind), 255*mask.astype(np.uint8))
    #
    # cv2.waitKey(1)

    return obstacle_list, obstacle_mask


def segment_depth(D, max_range=5000, min_size_ratio=20):

    # minimum area to be considered an obstacle
    min_size = D.shape[0]*D.shape[1] // min_size_ratio

    # to hold result
    obstacle_list, obstacle_mask = list(), list()

    # calculate histogram
    hist = cv2.calcHist([D], [0], None, [2**16], [0, 2**16])

    # kernel for morphological operations
    kernel = np.ones(5, np.uint8)

    # closing to remove small holes
    hist = cv2.morphologyEx(hist, cv2.MORPH_CLOSE, kernel)

    # morphological opening to remove noise
    hist = cv2.morphologyEx(hist, cv2.MORPH_OPEN, kernel)

    # after opencv forced us to use a floating point vector for morphological operations
    hist = hist.astype(np.int)

    # remove places with depth zero
    hist[0] = 0

    # clip at max_range
    hist[max_range:] = 0

    # connected components for places where histogram is nonzero
    nz = label(hist > 0)

    # ugly hack to make regionprops work with 1D
    for region in regionprops(np.hstack((nz, nz))):
        # forget label 0 (the one where there is no depth)
        if region.label == 0:
            continue

        # get range of graylevels for this region
        start, _, end, _ = region.bbox

        # disregard very small regions
        if np.sum(hist[start:end]) < min_size:
            continue

        # pixels in this region
        B = np.bitwise_and(D >= start, D < end)

        # find bounding values
        nonzero_indices = np.where(B)
        bb_y_min, bb_y_max, _, _ = cv2.minMaxLoc(nonzero_indices[0])
        bb_x_min, bb_x_max, _, _  = cv2.minMaxLoc(nonzero_indices[1])

        # bounding box in image (notice before we found the bounding box in the histogram region)
        bounding_box = [bb_x_min, bb_y_min, bb_x_max, bb_y_max]

        # to hold results for this object
        detection = dict()

        # no classification yet
        detection['classification'] = 'unlabeled'

        # save mask for debug purposes
        obstacle_mask.append(B)

        # get location attributes
        detection['x'], detection['y'], detection['z'], detection['width'], detection['height'], detection['length'] = \
            calc_attributes(D, bounding_box, distance=start)

        # add to result
        obstacle_list.append(detection)

    return obstacle_list, obstacle_mask


def label_detections(obstacle_list, obstacle_mask, segmentation_image):
    # TODO: a more elegant way than a global variable
    global labels

    # for each detection
    for obstacle, mask in zip(obstacle_list, obstacle_mask):
        
        # mask segmentation image
        MS = cv2.bitwise_and(segmentation_image, mask)

        # find most common element
        most_common_class = mode(MS.flatten())

        obstacle['class'] = labels[most_common_class]

if __name__ == '__main__':

    D = cv2.imread('grab.png', cv2.IMREAD_ANYDEPTH)

    # obstacle_list, obstacle_mask = cluster_depth(D)
    obstacle_list, obstacle_mask = segment_depth(D)

    drek = 6

# if __name__ == '__main__':

#     # start reading images
#     # grabber = FileGrabber('drek.bag')
#     grabber = Grabber()
#     grabber.start()

#     # initialize display
#     fig, ax = None, None

#     # handles to images
#     handles = dict()

#     # for _ in range(30):
#     while True:
#         # grab images
#         depth_image, vis_image, colorized_depth = grabber.grab()

#         # segment depth image
#         obstacle_list, obstacle_mask = segment_depth(depth_image)

#         # max number for display
#         max_objects = 5

#         # create figure if it does not already exist
#         if fig is None:
#             fig, ax = plt.subplots(1, max_objects)

#             # remove ticks
#             for ind in range(max_objects):

#                 # get segment if it exists
#                 if ind < len(obstacle_mask):
#                     handles[ind] = ax[ind].imshow(cv2.bitwise_and(vis_image, vis_image,
#                                                                   mask=obstacle_mask[ind].astype(np.uint8)))
#                 else:
#                     handles[ind] = ax[ind].imshow(np.zeros((480, 640), dtype=np.uint8))

#                 ax[ind].set_xticklabels([])
#                 ax[ind].set_yticklabels([])

#             plt.tight_layout()

#         else:
#             for ind in range(max_objects):
#                 # get segment if it exists
#                 if ind < len(obstacle_mask):
#                     handles[ind].set_data(cv2.bitwise_and(vis_image, vis_image,
#                                                           mask=obstacle_mask[ind].astype(np.uint8)))

#                     ax[ind].set_title(f'depth {obstacle_list[ind]["x"]}')
#                 else:
#                     handles[ind].set_data(np.zeros((480, 640), dtype=np.uint8))
#                     ax[ind].set_title('')

#             fig.canvas.draw()
#             plt.pause(0.01)

        # cv2.imshow("depth", colorized_depth)
        # cv2.imshow("vis", vis_image)
        # cv2.waitKey(1)