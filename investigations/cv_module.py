#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image


import matplotlib.pyplot as plt
import numpy as np
import math
import json

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
drone_image_raw = None


# Constants
L1 = 79.0
L2 = 341.0
L3 = 863.0
L4 = 1110.0

RELATION_R_L1 = L4 / L1
RELATION_R_L2 = L4 / L2
RELATION_R_L3 = L4 / L3
RELATION_R_L4 = L4 / L4

# Image size
IMG_WIDTH = 640
IMG_HEIGHT = 360

# Relation between the traverse vector length and the longitudinal vector length
REL_TRAV_LONG = 0.23


######################
# Help functions #
######################
def make_median_blurry(image, blur_size):
    return cv2.medianBlur(image, blur_size)


def make_gaussian_blurry(image, blur_size):
    return cv2.GaussianBlur(image, (blur_size, blur_size), 0)


def make_circle_average_blurry(image, blur_size):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (blur_size,blur_size))
    n_elements = np.float64(np.count_nonzero(kernel))
    kernel_norm = (kernel/n_elements)
    
    img_blurred = cv2.filter2D(image,-1,kernel_norm)

    return img_blurred


def hsv_save_image(image, label='image', is_gray=False):
    # folder = 'image_processing/detect_h/'
    folder = 'image_processing/cv_module/'
    if is_gray:
        cv2.imwrite(folder+label+".png", image)
    else:
        cv2.imwrite(folder+label+".png", cv2.cvtColor(image, cv2.COLOR_HSV2BGR))
    return image


def load_hsv_image(filename):
    img = cv2.imread(filename) # import as BGR
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert to HSV
    hsv_save_image(hsv, "1_hsv") # Save image
    return hsv


def rgb_color_to_hsv(red, green, blue):
    bgr_color = np.uint8([[[blue,green,red]]])
    hsv_color = cv2.cvtColor(bgr_color,cv2.COLOR_BGR2HSV)
    return hsv_color[0][0].tolist()


def normalize_vector(vector):
    return vector / np.linalg.norm(vector) #, ord=1)


def hsv_to_opencv_hsv(hue, saturation, value):
    """ 
    Function that takes in hue, saturation and value in the ranges
        hue: [0, 360] degrees,  saturation: [0, 100] %,     value: [0, 100] %
    and converts it to OpenCV hsv which operates with the ranges
        hue: [0, 180],          saturation: [0, 255],       value: [0, 255]
    """
    converting_constant = np.array([0.5, 2.55, 2.55]) 
    return np.array([ hue, saturation, value])*converting_constant


def draw_dot(img, position, color):
    cX = position[1]
    cY = position[0]
    cv2.circle(img, (cX, cY), 3, color, -1)


def draw_arrow(img, start, end):
    return cv2.arrowedLine(img, (start[1], start[0]),
        (end[1], end[0]),
        color = (75,0,0), thickness = 1, tipLength = 0.4)


def calc_angle_between_vectors(vector_1, vector_2):
    v1_x = vector_1[0]
    v1_y = vector_1[1]

    v2_x = vector_2[0]
    v2_y = vector_2[1]

    angle = np.arctan2( v1_x*v2_y - v1_y*v2_x, v1_x*v2_x + v1_y*v2_y)

    # unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    # unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    # dot_product = np.dot(unit_vector_1, unit_vector_2)
    # angle = np.arccos(dot_product)
    return angle


def limit_point_to_be_inside_image(point):
    """ Make sure the point is inside the image 
        if it is not, move it to the closest border
    """
    smallest_x = 0
    smallest_y = 0
    largest_x = IMG_HEIGHT-1
    largest_y = IMG_WIDTH-1

    limited_point = np.int0(np.array([
        max(smallest_x, min(point[0], largest_x)),
        max(smallest_y, min(point[1], largest_y))
    ]))

    return limited_point


def print_header(text):
    text_length = len(text)
    border_line = "#"*(text_length+4)
    text_line = "# " + text + " #"

    print ""
    print border_line
    print text_line
    print border_line


def get_gradient_of_point(point, dx, dy):
    point_x = point[0]
    point_y = point[1]
    gradient = np.array([dx[point_x][point_y], dy[point_x][point_y]])
    return gradient


# Colors to draw with
HSV_RED_COLOR = rgb_color_to_hsv(255,0,0)
HSV_BLUE_COLOR = rgb_color_to_hsv(0,0,255)
HSV_BLACK_COLOR = rgb_color_to_hsv(0,0,0)
HSV_YELLOW_COLOR = [30, 255, 255]
HSV_LIGHT_ORANGE_COLOR = [15, 255, 255]


##################
# Main functions #
##################
def get_white_mask(hsv):    
    lower_white = hsv_to_opencv_hsv(0, 0, 50)
    upper_white = hsv_to_opencv_hsv(360, 50, 100)

    mask = cv2.inRange(hsv, lower_white, upper_white)
    _ , img_binary = cv2.threshold(mask, 128, 255, cv2.THRESH_BINARY)
    
    white_x, white_y = np.where(img_binary==255)
    if len(white_x) == 0: # No white visible
        return None
    else:
        return img_binary


def get_orange_mask(hsv):
    lower_orange = hsv_to_opencv_hsv(25, 50, 50)
    upper_orange = hsv_to_opencv_hsv(35, 100, 100)

    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange) 
    return orange_mask


def find_white_centroid(hsv):
    hsv_white_only = get_white_mask(hsv)
    hsv_white_only = make_gaussian_blurry(hsv_white_only, 5)

	# calculate moments of binary image
    M = cv2.moments(hsv_white_only)

    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    # Returned the transposed point,
    #  because of difference from OpenCV axis
    return np.array([cY, cX])


def find_harris_corners(img, block_size):
    """ Using sub-pixel method from OpenCV """
    dst = cv2.cornerHarris(img, block_size, 3, 0.04)
    dst = cv2.dilate(dst, None)
    ret, dst = cv2.threshold(dst,0.01*dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(img,np.float32(centroids),(5,5),(-1,-1),criteria)

    # Flip axis
    corners[:,[0, 1]] = corners[:,[1, 0]]

    return corners


def clip_corners_on_border(corners, border_size):
    # Filter the corners
    number_of_corners = len(corners)
    x_min = np.array([border_size]*number_of_corners)
    x_max = np.array([IMG_HEIGHT - border_size]*number_of_corners)
    y_min = np.array([border_size]*number_of_corners)
    y_max = np.array([IMG_WIDTH - border_size]*number_of_corners)

    corner_x = np.int0(corners[:,0])
    corner_y = np.int0(corners[:,1])

    # Keep corners within the border limit
    corners_clipped_on_border = corners[
        np.logical_and(
            np.logical_and(
                np.greater(corner_x, x_min),            # Add top limit
                np.less(corner_x, x_max)),              # Add bottom limit
            np.logical_and(
                np.greater(corner_y, y_min),            # Add left limit
                np.less(corner_y, y_max))               # Add right limit
        )
    ]
    corner_x = np.int0(corners_clipped_on_border[:,0])
    corner_y = np.int0(corners_clipped_on_border[:,1])
    
    if np.ndim(corner_x) == 0:
        number_of_corners = 1
        corners = np.array([[corner_x, corner_y]])
    else:
        corners = np.stack((corner_x, corner_y), axis=1)
        number_of_corners = len(corners)
    if number_of_corners == 0:
        return None
    else:
        return corners


def clip_corners_on_intensity(corners, img, average_filter_size):
    """
        Filter out the corners that belong to a right-angled corner
        i.e. corners with a mean intensity value around 255/4~64
    """
    value_per_degree = 255.0/360.0
    min_degree, max_degree = 60, 120 # +- 30 from 90 degrees

    # Since 255 is white and 0 is black, subtract from 255
    # to get black intensity instead of white intensity
    min_average_intensity = 255 - max_degree*value_per_degree
    max_average_intensity = 255 - min_degree*value_per_degree

    number_of_corners = len(corners)
    min_intensity = np.array([min_average_intensity]*number_of_corners)
    max_intensity = np.array([max_average_intensity]*number_of_corners)

    img_average_intensity = make_circle_average_blurry(img, average_filter_size)

    corner_x = np.int0(corners[:,0])
    corner_y = np.int0(corners[:,1])

    corners_clipped_on_intensity = corners[
        np.logical_and(
            np.greater(                                                 # Add top limit
                img_average_intensity[corner_x,corner_y],
                min_intensity),                                                  
            np.less(                                                    # Add bottom limit
                img_average_intensity[corner_x,corner_y],
                max_intensity)                                                   
        )
    ]
    corner_x = np.int0(corners_clipped_on_intensity[:,0])
    corner_y = np.int0(corners_clipped_on_intensity[:,1])
    
    if np.ndim(corner_x) == 0:
        corners = np.array([[corner_x, corner_y]])
        intensities = np.array([img_average_intensity[corner_x, corner_y]])
        number_of_corners = 1
    else:
        corners = np.stack((corner_x, corner_y), axis=1)
        intensities = np.array(img_average_intensity[corner_x, corner_y])
        number_of_corners = len(corners)

    if number_of_corners == 0:
        return None, None
    else:
        return corners, intensities


def find_right_angled_corners(img):
    # Parameters ###########################
    average_filter_size = 19 # 19
    ignore_border_size = 3
    corner_harris_block_size = 4

    # Define valid intensity range for the median of a corner
    min_intensity_average = 170
    max_intensity_average = 240
    ########################################

    corners = find_harris_corners(img, corner_harris_block_size)
    corners = clip_corners_on_border(corners, ignore_border_size)
    if corners is None:
        # Found no corners
        return None

    corners, intensities = clip_corners_on_intensity(corners, img, average_filter_size)
    if corners is None:
        # Found no corners
        return None, None

    return corners, intensities


def find_goal_point(hsv_white_only, inner_corners):
    number_of_corners = len(inner_corners)
    average_filter_size = 19
    img_average_intensity = make_circle_average_blurry(hsv_white_only, average_filter_size)

    ######################
    # Define the inner_corners #

    if number_of_corners == 1:
        corner_0 = inner_corners[0]

    elif number_of_corners == 2:
        left_corner_id = np.argmin(inner_corners[:,1])
        right_corner_id = 1 - left_corner_id

        corner_0 = inner_corners[left_corner_id]
        corner_1 = inner_corners[right_corner_id]
        
    elif number_of_corners == 3:
        distances = np.array([
            np.linalg.norm(inner_corners[0] - inner_corners[1]),
            np.linalg.norm(inner_corners[1] - inner_corners[2]),
            np.linalg.norm(inner_corners[2] - inner_corners[0])
        ])

        median = np.median(distances)
        median_id = np.where(distances == median)[0][0]

        if median_id == 0:
            relevant_corners = np.stack((inner_corners[0], inner_corners[1]), axis=0)
        elif median_id == 1:
            relevant_corners = np.stack((inner_corners[1], inner_corners[2]), axis=0)
        elif median_id == 2:
            relevant_corners = np.stack((inner_corners[2], inner_corners[0]), axis=0)
        else:
            print("ERROR: In fn. detect_sub_pixecl_corners(); min_dist_id out of bounds")
            return None, None

        dist_0_1, dist_1_2, dist_2_0 = distances[0], distances[1], distances[2]

        # Pick the corner with the lowest y-coordinate
        left_corner_id = np.argmin(relevant_corners, axis=0)[1]
        right_corner_id = 1 - left_corner_id

        corner_0 = relevant_corners[left_corner_id]
        corner_1 = relevant_corners[right_corner_id]

    elif number_of_corners == 4:
        # For the first corner, chose the corner closest to the top
        # This will belong to the top cross-bar
        top_corner_id = np.argmin(inner_corners[:,0]) # Find lowest x-index
        top_corner = inner_corners[top_corner_id]
        top_corner_stack = np.array([top_corner]*3)
        rest_corners = np.delete(inner_corners, top_corner_id, 0)
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

    else:
        # Invalid number of corners
        return None, None

    ##################################
    # Define goal point and direction #'
    goal_point = None
    goal_direction = None

    # Calculate spatial gradient
    dy, dx	= cv2.spatialGradient(hsv_white_only) # Invert axis, since OpenCV operates with x=column, y=row
    
    if number_of_corners == 1:
        grad_0 = np.array([dx[corner_0[0]][corner_0[1]], dy[corner_0[0]][corner_0[1]]])
        grad_0_normalized = normalize_vector(grad_0)

        goal_point_offset = 5 # px-distance
        goal_point = np.int0(corner_0 + grad_0_normalized*goal_point_offset)

        goal_direction = grad_0_normalized
    else:
        grad_0 = np.array([dx[corner_0[0]][corner_0[1]], dy[corner_0[0]][corner_0[1]]])
        grad_1 = np.array([dx[corner_1[0]][corner_1[1]], dy[corner_1[0]][corner_1[1]]])
        grad_sum = grad_0 + grad_1
        grad_sum_normalized = normalize_vector(grad_sum)

        cross_over_vector = corner_1 - corner_0
        goal_point = np.int0(corner_0 + 0.5*(cross_over_vector))
        goal_value = img_average_intensity[goal_point[0]][goal_point[1]]


    if (number_of_corners == 2 or number_of_corners == 3) and goal_value < 200:
        # The two inner_corners form the longitudinal bar

        # Choose to focus on the uppermost corner,
        # since it is assumed the orientation is approximately right
        longitudinal_corners = np.stack((corner_0, corner_1))

        upper_corner_id = np.argmin(longitudinal_corners[:,0])
        lower_corner_id = 1 - upper_corner_id

        upper_corner = longitudinal_corners[upper_corner_id]
        lower_corner = longitudinal_corners[lower_corner_id]

        longitudinal_vector = upper_corner - lower_corner
        longitudinal_unit_vector = normalize_vector(longitudinal_vector)

        upper_corner_gradient = get_gradient_of_point(upper_corner, dx, dy)
        upper_corner_unit_gradient = normalize_vector(upper_corner_gradient)


        longitudinal_length = np.linalg.norm(longitudinal_vector)
        travese_length = longitudinal_length*REL_TRAV_LONG
        length_from_upper_corner_to_goal = travese_length/2.0

        arrow_length = 10
        grad_start = upper_corner
        grad_end = np.int0(upper_corner + upper_corner_unit_gradient*arrow_length)

        angle = calc_angle_between_vectors(longitudinal_vector, upper_corner_unit_gradient)
        l_x, l_y = longitudinal_unit_vector[0], longitudinal_unit_vector[1]
        if angle > 0:
            # This means the gradient is pointing to the left
            dir_vector = np.array([-l_y, l_x])
        else:
            # The gradient is pointing to the right (or is parallel)
            dir_vector = np.array([l_y, -l_x])
        goal_point = np.int0(upper_corner + dir_vector*length_from_upper_corner_to_goal)
        goal_point = limit_point_to_be_inside_image(goal_point)
        
        goal_direction = longitudinal_unit_vector
    
    elif number_of_corners != 1:
        if grad_sum_normalized[0] < 0:
            cross_over_norm = np.array([-cross_over_vector[1], cross_over_vector[0]])
        else:
            cross_over_norm = np.array([cross_over_vector[1], -cross_over_vector[0]])

        goal_direction = normalize_vector(cross_over_norm)

    return goal_point, goal_direction


def find_orange_arrowhead(hsv):
    hsv_orange_mask = get_orange_mask(hsv)
    hsv_orange_mask = make_gaussian_blurry(hsv_orange_mask, 5) 

    hsv_orange_mask_inverted = cv2.bitwise_not(hsv_orange_mask)

    orange_corners, intensities = find_right_angled_corners(hsv_orange_mask_inverted)

    if orange_corners is None:
        return None

    number_of_corners_found = len(orange_corners)

    value_per_degree = 255.0/360.0
    ideal_angle = 90
    ideal_intensity = 255-ideal_angle*value_per_degree
    ideal_intensities = np.array([ideal_intensity]*number_of_corners_found)

    diff_intensities = np.absolute(np.array(ideal_intensities-intensities))

    if number_of_corners_found == 1:
        return orange_corners[0]
    elif number_of_corners_found > 1:
        # Too many orange corners found, choose the best
        best_corner_id = np.argmin(diff_intensities)
        best_corner = orange_corners[best_corner_id]
        return best_corner
    else:
        # No corners found
        return None


def calculate_position(center_px, radius_px):
    focal_length = 374.67
    real_radius = 375 # mm (750mm in diameter / 2)
    # real_radius = 288 # previous value

    # Center of image
    x_0 = IMG_HEIGHT/2.0
    y_0 = IMG_WIDTH/2.0

    # Find distances from center of image to center of LP
    d_x = x_0 - center_px[0]
    d_y = y_0 - center_px[1]

    est_z = real_radius*focal_length / radius_px # - 59.4 # (adjustment)
    
    # Camera is placed 150 mm along x-axis of the drone
    # Since the camera is pointing down, the x and y axis of the drone
    # is the inverse of the x and y axis of the camera
    est_x = -(est_z * d_x / focal_length) - 150 
    est_y = -(est_z * d_y / focal_length)

    return np.array([est_x, est_y, est_z])


def evaluate_arrow(hsv):
    """ Use the arrow to find: 
        center, radius, angle 
    """
    center_px = find_white_centroid(hsv)
    arrowhead_px = find_orange_arrowhead(hsv)

    if (center_px is not None) and (arrowhead_px is not None):

        arrow_vector = np.array(arrowhead_px - center_px)
        arrow_unit_vector = normalize_vector(arrow_vector)
        ref_vector = np.array([0,1])
        
        angle = np.degrees(calc_angle_between_vectors(arrow_vector, ref_vector))

        arrow_length_px = np.linalg.norm(arrow_vector)
        # Use known relation between the real radius and the real arrow length
        # to find the radius length in pixels
        radius_length_px = arrow_length_px * RELATION_R_L3

        return center_px, radius_length_px, angle
        
    else:
        return None, None, None


def evaluate_inner_corners(hsv):
    # wp
    """ Use the inner corners to find: 
        center, radius, angle 
    """
    hsv_canvas = hsv.copy()

    hsv_white_only = get_white_mask(hsv)
    hsv_white_only = make_gaussian_blurry(hsv_white_only, 5)


    inner_corners, intensities = find_right_angled_corners(hsv_white_only)

    for corner in inner_corners:
        draw_dot(hsv_canvas, corner, HSV_LIGHT_ORANGE_COLOR)

    # goal, goal_direction = find_goal_point(hsv_white_only, inner_corners)

    hsv_save_image(hsv_canvas, "2_canvas")


    return None, None, None


def run(img_count = 0):
    filepath = "dataset/low_flight_dataset_02/image_"+str(img_count)+".png"
    hsv = load_hsv_image(filepath)

    center_px_from_arrow, radius_length_px_from_arrow, angle_from_arrow = evaluate_arrow(hsv)
    center_px_from_inner_corners, radius_length_px_from_inner_corners, angle_from_inner_corners = evaluate_inner_corners(hsv)

    if (center_px_from_arrow is not None):
        center_px, radius_length_px, angle = center_px_from_arrow, radius_length_px_from_arrow, angle_from_arrow
        print "Position from arrow:", calculate_position(center_px, radius_length_px)
    else:
        print "Position from arrow: [Not available]"

    if (center_px_from_inner_corners is not None):
        center_px, radius_length_px, angle = center_px_from_inner_corners, radius_px_length_from_inner_corners, angle_from_inner_corners
        print "Position from inner corners:", calculate_position(center_px, radius_length_px)
    else:
        print "Position from inner corners: [Not available]"

    print ""




    # inner_corners, intensities = find_right_angled_corners(hsv_white_only)

    # goal, goal_direction = find_goal_point(hsv_white_only, inner_corners)
    

############
# Examples #
############

# In dataset_01 (20 images)
# 18 for two traverse corners
#
#
# In dataset_02 (42 images)
# 3 for three corners, where the two traversal is pointing down
# 29 arrowhead not visible (should be)
# 1 three corners showing


single_image_index = 4
single_image = False

results_gt = []
results_est = []

if single_image:
    print "###########################"
    print "# Now testing on image", str(single_image_index).rjust(2), "#"
    print "###########################"
    run(single_image_index)
else:
    for i in range(3):
        # answer = raw_input("Press enter for next image")
        print ""
        print "###########################"
        print "# Running CV module on image", i, "#"
        print "###########################"
        length, centroid = run(i)
        print ""
        print "# Preprocessing"
        print "length, centroid:", length, centroid
        print ""

        if length is not None:
            json_filepath = "dataset/low_flight_dataset_02/low_flight_dataset.json"
            with open(json_filepath) as json_file:
                data = json.load(json_file)
                gt_x, gt_y, gt_z = np.array(data[str(i)]['ground_truth'][0:3])*1000
                results_gt.append([gt_x, gt_y, gt_z])
                print "GT: ", gt_x, gt_y, gt_z

            est_x, est_y, est_z = calculate_position(centroid, length)

            print "Est:", est_x, est_y, est_z
            results_est.append([est_x, est_y, est_z])


    np_results_gt = np.array(results_gt)
    np_results_est = np.array(results_est)
    # print "Results ground truth"
    # print np_results_gt
    # print "Results estimate"
    # print np_results_est

    print_header("Showing results")

    n_results = len(np_results_gt)
    print "n_results:", n_results
    rjust = 7
    print "||  gt_x   |  est_x  ||  gt_y   |  est_y  ||  gt_z   |  est_z  ||"
    print "-----------------------------------------------------------------"
    for i in range(n_results):

        text_gt_x = '{:.2f}'.format(round(np_results_gt[i][0], 2)).rjust(rjust)
        text_est_x = '{:.2f}'.format(round(np_results_est[i][0], 2)).rjust(rjust)

        text_gt_y = '{:.2f}'.format(round(np_results_gt[i][1], 2)).rjust(rjust)
        text_est_y = '{:.2f}'.format(round(np_results_est[i][1], 2)).rjust(rjust)

        text_gt_z = '{:.2f}'.format(round(np_results_gt[i][2], 2)).rjust(rjust)
        text_est_z = '{:.2f}'.format(round(np_results_est[i][2], 2)).rjust(rjust)

        print "||", text_gt_x, "|",text_est_x, \
            "||", text_gt_y, "|",text_est_y, \
            "||", text_gt_z, "|",text_est_z, "||"