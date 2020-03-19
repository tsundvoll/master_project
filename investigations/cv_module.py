#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image


import matplotlib.pyplot as plt
import numpy as np
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
drone_image_raw = None


# Constants

# Image size
IMG_WIDTH = 640
IMG_HEIGHT = 360


#############
# Callbacks #
#############
def video_callback(data):
    global drone_image_raw
    drone_image_raw = data


######################
# Help functions #
######################
def make_blurry(image, blur):
    return cv2.medianBlur(image, blur)


def make_gaussian_blurry(image, blur):
    return cv2.GaussianBlur(image, (blur, blur), 0)


def hsv_save_image(image, label='image', is_gray=False):
    # folder = 'image_processing/detect_h/'
    folder = 'image_processing/cv_module/'
    if is_gray:
        cv2.imwrite(folder+label+".png", image)
    else:
        cv2.imwrite(folder+label+".png", cv2.cvtColor(image, cv2.COLOR_HSV2BGR))
    return image


def load_image(filename):
    img = cv2.imread(filename) # import as BGR
    return img


def rgb_color_to_hsv(red, green, blue):
    bgr_color = np.uint8([[[blue,green,red]]])
    hsv_color = cv2.cvtColor(bgr_color,cv2.COLOR_BGR2HSV)
    return hsv_color[0][0].tolist()


def normalize_vector(vector):
    return vector / np.linalg.norm(vector) #, ord=1)


def draw_dot(img, position, color):
    cX = position[1]
    cY = position[0]
    cv2.circle(img, (cX, cY), 3, color, -1)


def hsv_to_opencv_hsv(hue, saturation, value):
    """ 
    Function that takes in hue, saturation and value in the ranges
        hue: [0, 360] degrees,  saturation: [0, 100] %,     value: [0, 100] %
    and converts it to OpenCV hsv which operates with the ranges
        hue: [0, 180],          saturation: [0, 255],       value: [0, 255]
    """
    converting_constant = np.array([0.5, 2.55, 2.55]) 
    return np.array([ hue, saturation, value])*converting_constant


# Colors to draw with
HSV_RED_COLOR = rgb_color_to_hsv(255,0,0)
HSV_BLUE_COLOR = rgb_color_to_hsv(0,0,255)
HSV_BLACK_COLOR = rgb_color_to_hsv(0,0,0)
HSV_YELLOW_COLOR = [30, 255, 255]
HSV_LIGHT_ORANGE_COLOR = [15, 255, 255]


##################
# Main functions #
##################
def hsv_keep_white_only_simple(hsv):    
    lower_white = hsv_to_opencv_hsv(0, 0, 50)
    upper_white = hsv_to_opencv_hsv(360, 50, 100)

    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Threshold image
    _ , img_binary = cv2.threshold(mask, 128, 255, cv2.THRESH_BINARY)
    
    white_x, white_y = np.where(img_binary==255)
    if len(white_x) == 0: # No white visible
        return None
    else:
        return img_binary


def hsv_keep_white_only(hsv):
    lower_white_all = hsv_to_opencv_hsv(0, 0, 90)
    upper_white_all = hsv_to_opencv_hsv(360, 20, 100)

    lower_white_except_orange = hsv_to_opencv_hsv(70, 0, 85)
    upper_white_except_orange = hsv_to_opencv_hsv(360, 40, 100)

    lower_white_almost_green = hsv_to_opencv_hsv(0, 0, 85)
    upper_white_almost_green = hsv_to_opencv_hsv(70, 10, 100)

    mask_all = cv2.inRange(hsv, lower_white_all, upper_white_all)
    mask_except_orange = cv2.inRange(hsv, lower_white_except_orange, upper_white_except_orange)
    mask_except_almost_green = cv2.inRange(hsv, lower_white_almost_green, upper_white_almost_green)
    combined_mask = mask_all + mask_except_orange + mask_except_almost_green 
    
    return combined_mask


def detect_moment(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_save_image(hsv, '1_hsv')

    img_centroid = hsv.copy()

    white = hsv_keep_white_only(hsv)
    hsv_save_image(white, "2_white", is_gray=True)

    # Threshold image
    _ , img_binary = cv2.threshold(white, 128, 255, cv2.THRESH_BINARY)
    hsv_save_image(img_binary, "3_binary", is_gray=True)

	# calculate moments of binary image
    M = cv2.moments(img_binary)

    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    cv2.circle(img_centroid, (IMG_WIDTH/2, IMG_HEIGHT/2),
        5, (128, 128, 128), -1)

    cv2.circle(img_centroid, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(img_centroid, "centroid", (cX - 25, cY - 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    hsv_save_image(img_centroid, "4_centroid")


def keep_orange_only(hsv):
    lower_orange = hsv_to_opencv_hsv(25, 50, 50)
    upper_orange = hsv_to_opencv_hsv(35, 100, 100)

    mask = cv2.inRange(hsv, lower_orange, upper_orange) 

    return mask


def find_pixels_inside_orange(hsv):
    """ 
        Function that finds the orange in an image, make a bounding box around it,
        fits an ellipse in the bounding box
        and paints everything outside the ellipse in black.

        Returns the painted image and a boolean stating wheather any orange was found.
     """

    # hsv_save_image(hsv, '1_hsv')
    hsv_inside_orange = hsv.copy()
    hsv_ellipse_mask = np.zeros((IMG_HEIGHT, IMG_WIDTH))
    hsv_ellipse = hsv.copy()


    hsv_orange_only = keep_orange_only(hsv)  
    hsv_save_image(hsv_orange_only, "2b_orange_mask", is_gray=True)

    orange_x, orange_y = np.where(hsv_orange_only==255)
    # If no orange in image: return original image
    if len(orange_x) == 0:
        return hsv, False

    x_min = np.amin(orange_x)
    x_max = np.amax(orange_x)
    y_min = np.amin(orange_y)
    y_max = np.amax(orange_y)
    # print x_min, x_max, y_min, y_max



    hsv_inside_orange[0:x_min,] = HSV_BLACK_COLOR
    hsv_inside_orange[x_max+1:,] = HSV_BLACK_COLOR

    hsv_inside_orange[:,0:y_min] = HSV_BLACK_COLOR
    hsv_inside_orange[:,y_max+1:] = HSV_BLACK_COLOR
    # hsv_inside_orange[x_min:x_max+1, y_min:y_max+1] = HSV_BLACK_COLOR

    # hsv_save_image(hsv_inside_orange, '3_inside_orange')


    # center_x = np.int0((x_min + x_max) / 2.0)
    # center_y = np.int0((y_min + y_max) / 2.0)

    # l_x = np.int0(x_max - center_x)+1
    # l_y = np.int0(y_max - center_y)+1

    # cv2.ellipse(img=hsv_ellipse_mask, center=(center_y, center_x),
    #     axes=(l_y, l_x), angle=0, startAngle=0, endAngle=360,
    #     color=(255), thickness=-1, lineType=8, shift=0)


    # # hsv_save_image(hsv_ellipse_mask, '4_ellipse_mask', is_gray=True)

    # hsv_ellipse[hsv_ellipse_mask==0] = HSV_BLACK_COLOR

    # hsv_save_image(hsv_ellipse, '5_ellipse')

    # return hsv_ellipse, True
    return hsv_inside_orange, True


def find_orange_arrow_point(hsv):
    # Parameters ###########################
    first_blur_size = 5
    second_blur_size = 49
    ignore_border_size = 3
    corner_harris_block_size = 4
    ignore_border_size = 3


    # Define valid intensity range for the median of a corner
    min_intensity_average = 10 # 20 # 170
    max_intensity_average = 85 # 100 # 240
    ########################################
    
    hsv_orange_only = keep_orange_only(hsv)
    # hsv_save_image(hsv_orange_only, "3_orange_only", is_gray=True)

    blur = make_gaussian_blurry(hsv_orange_only, first_blur_size) 
    double_blur = make_gaussian_blurry(blur, second_blur_size)


    # Sub-pixel corner detection:
    dst = cv2.cornerHarris(blur,corner_harris_block_size,3,0.04)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(blur,np.float32(centroids),(5,5),(-1,-1),criteria)

    corner_x = np.int0(corners[:,1])
    corner_y = np.int0(corners[:,0])
    number_of_corners = len(corner_x)


    # Keep corners within the border limit
    x_min = np.array([ignore_border_size]*number_of_corners)
    x_max = np.array([IMG_HEIGHT - ignore_border_size]*number_of_corners)
    y_min = np.array([ignore_border_size]*number_of_corners)
    y_max = np.array([IMG_WIDTH - ignore_border_size]*number_of_corners)

    corners_clipped_on_border = corners[
        np.logical_and(
            np.logical_and(
                np.greater(corners[:,1], x_min),            # Add top limit
                np.less(corners[:,1], x_max)                # Add bottom limit
            ),
            np.logical_and(
                np.greater(corners[:,0], y_min),            # Add left limit
                np.less(corners[:,0], y_max)                # Add right limit
            )
        )
    ]

    corner_x = np.int0(corners_clipped_on_border[:,1])
    corner_y = np.int0(corners_clipped_on_border[:,0])
    
    if np.ndim(corner_x) == 0:
        number_of_corners = 1
    else:
        number_of_corners = len(corner_x)

    # Filter corner on median
    min_intensity = np.array([min_intensity_average]*number_of_corners)
    max_intensity = np.array([max_intensity_average]*number_of_corners)

    # Filter out the corners that belong to an "outer corner"
    # and outliers that are inside the white area
    corners_clipped_on_intensity = corners_clipped_on_border[
        np.logical_and(
            np.greater(
                double_blur[corner_x,corner_y],
                min_intensity
            ),                                                  # Add top limit
            np.less(
                double_blur[corner_x,corner_y],
                max_intensity
            )                                                   # Add bottom limit
        )
    ]

    corner_x = np.int0(corners_clipped_on_intensity[:,1])
    corner_y = np.int0(corners_clipped_on_intensity[:,0])



    # Draw corners
    img_all_corners = hsv.copy()
    img_all_corners[corner_x, corner_y] = HSV_RED_COLOR
    hsv_save_image(img_all_corners, "4_corners")

    if len(corners_clipped_on_intensity) == 1:
        return np.array([corner_x, corner_y])
    elif len(corners_clipped_on_intensity) == 0:
        print "Found zero corners"
    elif len(corners_clipped_on_intensity) > 1:
        print "Found too many corners"
    else:
        print "ERROR in find_orange_arrow_point()"
    
    return None


def find_white_centroid(hsv_white_only):
	# calculate moments of binary image
    M = cv2.moments(hsv_white_only)

    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    # Returned the transposed point,
    #  because of difference from OpenCV axis
    return np.array([cY, cX])


def calc_angle_centroid_arrowhead(centroid, arrowhead):
    """
        Calculates the angle between
        the line from the centroid to the arrowhead
        and the horizontal axis.
    """
    v_1 = arrowhead - centroid
    # print "v_1: ", v_1

    dx, dy = v_1[0], v_1[1]
    theta = np.degrees(np.arctan2(dy, -dx))
    return theta


def calc_angle_centroid_mid_point(centroid, mid_point):
    """
        Calculates the angle between
        the line from the centroid to the mid-point
        and the horizontal axis.
        The mid-point is between the two inner corners in the H.
    """
    v_2 = mid_point - centroid
    # print "v_2: ", v_2

    dx, dy = v_2[0], v_2[1]
    alpha = np.degrees(np.arctan2(-dy, -dx))
    return alpha


def alpha_to_theta(alpha):

    # Change alpha to be in range (-90, 270]
    # alpha -90 => 270
    # alpha -100 => 260
    if alpha < -90:
        alpha_sat = 360 + alpha
    else:
        alpha_sat = alpha

    # print "alpha_sat: ", alpha_sat



    theta = 90 - alpha_sat

    # print "Theta (from alpha): ", theta
    # return theta


def test_of_functions():
    centroid = np.array([0, 0])
    for quadrant in range(1,5):
        if quadrant == 1:
            ################
            # 1st quadrant #
            ################
            print "################"
            print "# 1st quadrant #"
            print "################"
            arrowhead = np.array([-2, 3])
            theta = calc_angle_centroid_arrowhead(centroid, arrowhead)
            print "Theta to arrowhead: ", theta

            mid_point = np.array([-3, -2])
            alpha = calc_angle_centroid_mid_point(centroid, mid_point)
            print "Alpha to mid_point: ", alpha

            # print "Theta (from alpha): ", alpha_to_theta(alpha)
            theta_from_alpha = alpha_to_theta(alpha)
            print

        elif quadrant == 2:
            ################
            # 2nd quadrant #
            ################
            print "################"
            print "# 2nd quadrant #"
            print "################"
            arrowhead = np.array([2, 3])
            theta = calc_angle_centroid_arrowhead(centroid, arrowhead)
            print "Theta to arrowhead: ", theta

            mid_point = np.array([-3, 2])
            alpha = calc_angle_centroid_mid_point(centroid, mid_point)
            print "Alpha to mid_point: ", alpha
            
            # print "Theta (from alpha): ", alpha_to_theta(alpha)
            theta_from_alpha = alpha_to_theta(alpha)
            print

        elif quadrant == 3:    
            ################
            # 3rd quadrant #
            ################

            print "################"
            print "# 3rd quadrant #"
            print "################"
            arrowhead = np.array([2, -3])
            theta = calc_angle_centroid_arrowhead(centroid, arrowhead)
            print "Theta to arrowhead: ", theta

            mid_point = np.array([3, 2])
            alpha = calc_angle_centroid_mid_point(centroid, mid_point)
            print "Alpha to mid_point: ", alpha 
            
            # print "Theta (from alpha): ", alpha_to_theta(alpha)
            theta_from_alpha = alpha_to_theta(alpha)
            print

        elif quadrant == 4: 
            ################
            # 4th quadrant #
            ################
            print "################"
            print "# 4th quadrant #"
            print "################"
            arrowhead = np.array([-2, -3])
            theta = calc_angle_centroid_arrowhead(centroid, arrowhead)
            print "Theta to arrowhead: ", theta

            mid_point = np.array([3, -2])
            alpha = calc_angle_centroid_mid_point(centroid, mid_point)
            print "Alpha to mid_point: ", alpha
            
            # print "Theta (from alpha): ", alpha_to_theta(alpha)
            theta_from_alpha = alpha_to_theta(alpha)
            print


def detect_inner_corners(hsv_white_only, img_id = 0):
    # Parameters ###########################
    second_blur_size = 49
    ignore_border_size = 3
    corner_harris_block_size = 4

    # Define valid intensity range for the median of a corner
    min_intensity_average = 170
    max_intensity_average = 240
    ########################################

    # hsv_origin = hsv.copy()
    
    blur = make_gaussian_blurry(hsv_white_only, 5) 
    double_blur = make_gaussian_blurry(blur, second_blur_size)
    
    # Sub-pixel:
    dst = cv2.cornerHarris(blur,corner_harris_block_size,3,0.04)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(blur,np.float32(centroids),(5,5),(-1,-1),criteria)

    # Filter the corners
    number_of_corners = len(corners)
    x_min = np.array([ignore_border_size]*number_of_corners)
    x_max = np.array([IMG_HEIGHT - ignore_border_size]*number_of_corners)
    y_min = np.array([ignore_border_size]*number_of_corners)
    y_max = np.array([IMG_WIDTH - ignore_border_size]*number_of_corners)

    # Keep corners within the border limit
    corners_clipped_on_border = corners[
        np.logical_and(
            np.logical_and(
                np.greater(corners[:,1], x_min),            # Add top limit
                np.less(corners[:,1], x_max)                # Add bottom limit
            ),
            np.logical_and(
                np.greater(corners[:,0], y_min),            # Add left limit
                np.less(corners[:,0], y_max)                # Add right limit
            )
        )
    ]
    corner_x = np.int0(corners_clipped_on_border[:,1])
    corner_y = np.int0(corners_clipped_on_border[:,0])
    number_of_corners = len(corner_x)

    min_intensity = np.array([min_intensity_average]*number_of_corners)
    max_intensity = np.array([max_intensity_average]*number_of_corners)

    # Filter out the corners that belong to an "outer corner"
    # and outliers that are inside the white area
    corners_clipped_on_intensity = corners_clipped_on_border[
        np.logical_and(
            np.greater(
                double_blur[corner_x,corner_y],
                min_intensity
            ),                                                  # Add top limit
            np.less(
                double_blur[corner_x,corner_y],
                max_intensity
            )                                                   # Add bottom limit
        )
    ]
    corner_x = np.int0(corners_clipped_on_intensity[:,1])
    corner_y = np.int0(corners_clipped_on_intensity[:,0])

    if np.ndim(corner_x) == 0:
        number_of_corners = 1
        corners = np.array([[corner_x, corner_y]])
    else:
        corners = np.stack((corner_x, corner_y), axis=1)
        number_of_corners = len(corners)

    # print "Number of corners:", number_of_corners

    if number_of_corners == 0 or number_of_corners > 4:
        print("Invalid number of corners")
        return None, None


    ######################
    # Define the corners #

    if number_of_corners == 1:
        corner_0 = corners[0]

    if number_of_corners == 2:
        left_corner_id = np.argmin(corner_y)
        right_corner_id = 1 - left_corner_id

        corner_0 = corners[left_corner_id]
        corner_1 = corners[right_corner_id]


    if number_of_corners == 3:
        distances = np.array([
            np.linalg.norm(corners[0] - corners[1]),
            np.linalg.norm(corners[1] - corners[2]),
            np.linalg.norm(corners[2] - corners[0])
        ])

        min_dist_id = np.argmin(distances)

        if min_dist_id == 0:
            relevant_corners = np.stack((corners[0], corners[1]), axis=0)
        elif min_dist_id == 1:
            relevant_corners = np.stack((corners[1], corners[2]), axis=0)
        elif min_dist_id == 2:
            relevant_corners = np.stack((corners[2], corners[0]), axis=0)
        else:
            print("ERROR: In fn. detect_sub_pixecl_corners(); min_dist_id out of bounds")
            return None, None

        dist_0_1, dist_1_2, dist_2_0 = distances[0], distances[1], distances[2]

        # Pick the corner with the lowest y-coordinate
        left_corner_id = np.argmin(relevant_corners, axis=0)[1]
        right_corner_id = 1 - left_corner_id

        corner_0 = relevant_corners[left_corner_id]
        corner_1 = relevant_corners[right_corner_id]


    if number_of_corners == 4:
        # For the first corner, chose the corner closest to the top
        # This will belong to the top cross-bar
        top_corner_id = np.argmin(corner_x)
        top_corner = corners[top_corner_id]
        top_corner_stack = np.array([top_corner]*3)
        rest_corners = np.delete(corners, top_corner_id, 0)
        dist = np.linalg.norm(rest_corners - top_corner_stack, axis=1)

        # For the second corner, chose the corner closest to top corner
        top_corner_closest_id = np.argmin(dist)
        top_corner_closest = rest_corners[top_corner_closest_id]

        relevant_corners = np.stack((top_corner, top_corner_closest), axis=0)

        # Choose the corner with the lowest y-coordinate as the first corner
        left_corner_id = np.argmin(relevant_corners, axis=0)[1]
        right_corner_id = 1 - left_corner_id

        corner_0 = relevant_corners[left_corner_id]
        corner_1 = relevant_corners[right_corner_id]


    ##################################
    # Define mid point and direction #'
    mid_point = None
    dir_vector = None

    # Calculate spatial gradient
    dy, dx	= cv2.spatialGradient(blur) # Invert axis, since OpenCV operates with x=column, y=row
    
    if number_of_corners == 1:
        grad_0 = np.array([dx[corner_0[0]][corner_0[1]], dy[corner_0[0]][corner_0[1]]])
        grad_0_normalized = normalize_vector(grad_0)

        mid_point_offset = 5 # px-distance
        mid_point = np.int0(corner_0 + grad_0_normalized*mid_point_offset)

        dir_vector = grad_0_normalized

    else:
        grad_0 = np.array([dx[corner_0[0]][corner_0[1]], dy[corner_0[0]][corner_0[1]]])
        grad_1 = np.array([dx[corner_1[0]][corner_1[1]], dy[corner_1[0]][corner_1[1]]])
        grad_sum = grad_0 + grad_1
        grad_sum_normalized = normalize_vector(grad_sum)

        cross_over_vector = corner_1 - corner_0
        mid_point = np.int0(corner_0 + 0.5*(cross_over_vector))
        mid_value = double_blur[mid_point[0]][mid_point[1]]


    if number_of_corners == 2 and mid_value < 200:
        print "The two corners form the longitudinal bar"
        # Choose to focus on the left point

        v_long = corner_0 - corner_1
        v_long_norm = normalize_vector(v_long)

        grad_0_normalized = normalize_vector(grad_0)

        if grad_0_normalized[0] < 0:
            long_bar_norm = np.array([-cross_over_vector[1], cross_over_vector[0]])
        else:
            long_bar_norm = np.array([cross_over_vector[1], -cross_over_vector[0]])
        long_bar_norm_normalized = normalize_vector(long_bar_norm)
        

        dist_to_top_edge = corner_0[0]
        dist_to_bottom_edge = IMG_HEIGHT - corner_0[0]
        dist_to_nearest_edge = min(dist_to_top_edge, dist_to_bottom_edge)
        mid_point_offset = dist_to_nearest_edge / 2 # px-distance
        mid_point = np.int0(corner_0 + long_bar_norm_normalized*mid_point_offset)

        dir_vector = v_long_norm
    
    elif number_of_corners != 1:
        if grad_sum_normalized[0] < 0:
            cross_over_norm = np.array([-cross_over_vector[1], cross_over_vector[0]])
        else:
            cross_over_norm = np.array([cross_over_vector[1], -cross_over_vector[0]])

        dir_vector = normalize_vector(cross_over_norm)


    ###################
    # Draw the result #

    direction_length = 20
    direction_point = np.int0(mid_point + dir_vector*direction_length)

    # img_grad = hsv_origin.copy()
    # img_grad = cv2.arrowedLine(img_grad, (mid_point[1], mid_point[0]),
    #     (direction_point[1], direction_point[0]),
    #     color = (255,0,0), thickness = 2, tipLength = 0.5)

    # hsv_save_image(img_grad, "5_gradient")
    # hsv_save_image(img_grad, str(img_id) +"_gradient")


    return mid_point, corners


def run(img_count = 0):




    filepath = "dataset/low_flight_dataset_02/image_"+str(img_count)+".png"
    img = load_image(filepath)


    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_save_image(hsv, "1_hsv")
    img_marked = hsv.copy()


    hsv_inside_orange, isOrangeVisible = find_pixels_inside_orange(hsv)
    hsv_save_image(hsv_inside_orange, "2_inside_orange")


    hsv_white_only = hsv_keep_white_only_simple(hsv_inside_orange)
    if hsv_white_only is None:
        hsv_white_only = hsv_keep_white_only_simple(hsv)

    hsv_save_image(hsv_white_only, "3_white_only", is_gray=True)

    centroid = find_white_centroid(hsv_white_only)
    # print "centroid:", centroid

    if isOrangeVisible:
        arrowhead = find_orange_arrow_point(hsv) # for testing with an angle : + np.array([1, 1])
        if arrowhead is None: # Arrowhead is not visible or too many corners found
            print "Arrowhead is not found"
        else:
            theta = calc_angle_centroid_arrowhead(centroid, arrowhead)
            # print "Theta to arrowhead: ", theta

            draw_dot(img_marked, arrowhead, HSV_BLUE_COLOR)


    heading, inner_corners = detect_inner_corners(hsv_white_only)
    if inner_corners is None:
        print "No inner corners found"
    else:
        for corner in inner_corners:
            draw_dot(img_marked, corner, HSV_LIGHT_ORANGE_COLOR)

        draw_dot(img_marked, heading, HSV_YELLOW_COLOR)
    
    draw_dot(img_marked, centroid, HSV_RED_COLOR)


    hsv_save_image(img_marked, "3_marked")


i = 35
# for i in range(42):
#     answer = raw_input("Press enter for next image")
print ""
print "##########################"
print "# Now testing on image", i, "#"
print "##########################"
run(i)

# run()