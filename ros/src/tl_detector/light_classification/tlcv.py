"""
Computer vision rutines for traffic ligth classification
"""

from __future__ import print_function
import cv2
import random
import pandas as pd


def open_image(fname, read_flag=cv2.IMREAD_ANYCOLOR, color_transform=None):
    """
    Open image usign OpenCV.

    Possible `read_flag`s are cv2.IMREAD_GRAYSCALE, cv2.IMREAD_COLOR, cv2.IMREAD_ANYCOLOR.
    Example of a`color_transform`: cv2.COLOR_BGR2RGB, cv2.COLOR_BGR2GRAY
    """

    im = cv2.imread(fname, read_flag)

    if color_transform is not None:
        im = cv2.cvtColor(im, color_transform)

    return im


def threshold_binary_inv(im, t):
    """
    All pixels with intensity < t become 255
    and the rest become 0
    """
    
    _, im_t = cv2.threshold(im, t, 255, cv2.THRESH_BINARY_INV)
    return im_t


def threshold_binary(im, t):
    """
    All pixels with intensity > t become 255
    and the rest become 0
    """
    
    _, im_t = cv2.threshold(im, t, 255, cv2.THRESH_BINARY)
    return im_t


def find_ccomp(im, *args, **kwargs):
    """
    Finds connected components in a binary image.
    Returns the label image and a Pandas data frame
    with the connected components' statistics.

    *args and **kwargs are forwarded to the
    cv2.connectedComponentsWithStats call
    """

    num, labels, stats, centroids = cv2.connectedComponentsWithStats(im, *args, **kwargs)
    
    stats_df = pd.DataFrame(stats, columns=['left', 'top', 'width', 'height', 'area'])
    stats_df['x'] = centroids[:,0]
    stats_df['y'] = centroids[:,1]
    
    return labels, stats_df


def get_channels(im):
    
    im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    
    B = im[:, :, 0]
    G = im[:, :, 1]
    R = im[:, :, 2]

    S = im_hsv[:, :, 1]
    
    return R, G, B, S


def favor_pure_red(G, B, t):
    
    not_so_green = threshold_binary_inv(G, t)
    not_so_blue = threshold_binary_inv(B, t)
    
    return not_so_green & not_so_blue


def find_potential_tl(ccomp_stats, max_ratio_overhead, min_side_len):
    
    if len(ccomp_stats) == 1:
        return 0
    
    df = ccomp_stats.iloc[1:]
    
    hw_ratio = df.height / df.width
    min_side = df[['width', 'height']].min(axis=1)
    
    hw_bottom = 1. - max_ratio_overhead
    hw_top = 1. + max_ratio_overhead
    
    query = (min_side >= min_side_len) & (hw_ratio >= hw_bottom) & (hw_ratio <= hw_top)
    
    return query.sum()


def detect_red_light(im_bgr, t_R=200, t_S=100, t_GB=150, max_ratio_overhead=0.1, min_side_len=5):
    """
    Detect whether a red traffic light is present in the image.

    The idea is to detect regions in the image with high value of 
    the red channel, high saturation, and low levels of the
    green and blue components. Further, the resulting binary image 
    is used to detect connected components and filter them by 
    size and with-to-height ratio.
    """
    
    R, G, B, S = get_channels(im_bgr)
    
    binary_R = threshold_binary(R, t_R)
    binary_S = threshold_binary(S, t_S)
    binary_notGB = favor_pure_red(G, B, t_GB)
    
    binary_pure_red = binary_R & binary_S & binary_notGB
    
    ccomp_dlabels, ccomp_stats = find_ccomp(binary_pure_red)
    
    n_good = find_potential_tl(ccomp_stats, max_ratio_overhead, min_side_len)
    
    if n_good == 0:
        return False
    
    return True


if __name__ == '__main__':

    from glob import glob

    def shuffled_file_list(color):
        im_files = glob('/home/alex/carnd_tl/{0}/*.jpg'.format(color))
        random.shuffle(im_files)
        return im_files

    for fname in shuffled_file_list('red'):
        im = open_image(fname)
        res = detect_red_light(im)
        if res == False:
            print('Wrong (red): {0}'.format(fname))

    for fname in shuffled_file_list('green') + shuffled_file_list('yellow'):
        im = open_image(fname)
        res = detect_red_light(im)
        if res == True:
            print('Wrong (not red): {0}'.format(fname))