#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import math
import json


import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
drone_image_raw = None
global_image = None
global_rel_gt = None
save_images = False

# Constants
D_H_SHORT = 3.0
D_H_LONG = 9.0
D_ARROW = 30.0
D_RADIUS = 40.0

# Image size
IMG_WIDTH = 640
IMG_HEIGHT = 360

#################
# ROS functions #
#################
def image_callback(data):
    global global_image

    try:
        global_image = bridge.imgmsg_to_cv2(data, 'bgr8') # {'bgr8' or 'rgb8}
    except CvBridgeError as e:
        rospy.loginfo(e)


def rel_gt_callback(data):
    global global_rel_gt
    global_rel_gt = data


##################
# Help functions #
##################
def make_median_blurry(image, blur_size):
    return cv2.medianBlur(image, blur_size)


def make_gaussian_blurry(image, blur_size):
    return cv2.GaussianBlur(image, (blur_size, blur_size), 0)


def make_circle_average_blurry(image, blur_size):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (blur_size,blur_size))
    # print ""
    # print kernel
    n_elements = np.float64(np.count_nonzero(kernel))
    kernel_norm = (kernel/n_elements)
    
    img_blurred = cv2.filter2D(image,-1,kernel_norm)

    return img_blurred


def hsv_save_image(image, label='image', is_gray=False):
    # folder = 'image_processing/detect_h/'
    folder = './image_processing/'
    if save_images:
        if is_gray:
            cv2.imwrite(folder+label+".png", image)
        else:
            cv2.imwrite(folder+label+".png", cv2.cvtColor(image, cv2.COLOR_HSV2BGR))
    return image


def load_hsv_image(filename):
    img = cv2.imread(filename) # import as BGR
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert to HSV
    hsv_save_image(hsv, "1a_hsv") # Save image
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


def draw_dot(img, position, color, size=3):
    cX = np.int0(position[1])
    cY = np.int0(position[0])
    cv2.circle(img, (cX, cY), size, color, -1)


def draw_arrow(img, start, end):
    return cv2.arrowedLine(img,
        (np.int0(start[1]), np.int0(start[0])),
        (np.int0(end[1]), np.int0(end[0])),
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

    # print ""
    print border_line
    print text_line
    print border_line


def get_mid_point(a, b):
    return (a+b)/2.0


def get_normal_vector(bw_white_mask, corner_a, corner_b, is_short_side):
    hsv_normals = bw_white_mask.copy()

    vector_between_a_b = corner_a - corner_b

    vector_length = np.linalg.norm(vector_between_a_b)
    unit_vector_between_a_b = normalize_vector(vector_between_a_b)

    v_x, v_y = vector_between_a_b
    normal_unit_vector_left = normalize_vector(np.array([-v_y, v_x]))
    normal_unit_vector_right = normalize_vector(np.array([v_y, -v_x]))

    if is_short_side:
        check_length = vector_length / 3.0
        sign = 1 # Go outwards from the corners
    else:
        short_length = vector_length * D_H_SHORT/D_H_LONG
        
        check_length = short_length / 3.0
        sign = -1 # Go inwards from the corners

    check_left_a = np.int0(corner_a + \
        sign*unit_vector_between_a_b*check_length + \
        normal_unit_vector_left*check_length)
    check_left_b = np.int0(corner_b - \
        sign*unit_vector_between_a_b*check_length + \
        normal_unit_vector_left*check_length)

    check_right_a = np.int0(corner_a + \
        sign*unit_vector_between_a_b*check_length + \
        normal_unit_vector_right*check_length)
    check_right_b = np.int0(corner_b - \
        sign*unit_vector_between_a_b*check_length + \
        normal_unit_vector_right*check_length)

    value_left_a = bw_white_mask[check_left_a[0]][check_left_a[1]]
    value_left_b = bw_white_mask[check_left_b[0]][check_left_b[1]]
    value_right_a = bw_white_mask[check_right_a[0]][check_right_a[1]]
    value_right_b = bw_white_mask[check_right_b[0]][check_right_b[1]]

    avr_left = value_left_a/2.0 + value_left_b/2.0
    avr_right = value_right_a/2.0 + value_right_b/2.0

    # print "avr_left:", avr_left
    # print "avr_right:", avr_right

    draw_dot(hsv_normals, check_left_a, (225))
    draw_dot(hsv_normals, check_left_b, (225))
    draw_dot(hsv_normals, check_right_a, (75))
    draw_dot(hsv_normals, check_right_b, (75))

    # hsv_save_image(hsv_normals, "5_normals", is_gray=True)
    
    if avr_left > avr_right:
        return normal_unit_vector_left
    else:
        return normal_unit_vector_right


# Colors to draw with
HSV_RED_COLOR = rgb_color_to_hsv(255,0,0)
HSV_BLUE_COLOR = rgb_color_to_hsv(0,0,255)
HSV_BLACK_COLOR = rgb_color_to_hsv(0,0,0)
HSV_YELLOW_COLOR = [30, 255, 255]
HSV_LIGHT_ORANGE_COLOR = [15, 255, 255]

HSV_REAL_WHITE = [0, 0, 74]
HSV_REAL_ORANGE = [36, 100, 74]
HSV_REAL_GREEN = [120, 100, 30]

hue_margin = 15
sat_margin = 15
val_margin = 15


##################
# Main functions #
##################
def get_mask(hsv, hue_low, hue_high, sat_low, sat_high, val_low, val_high):
    lower_color = hsv_to_opencv_hsv(hue_low, sat_low, val_low)
    upper_color = hsv_to_opencv_hsv(hue_high, sat_high, val_high)

    mask = cv2.inRange(hsv, lower_color, upper_color) 

    mask_x, mask_y = np.where(mask==255)
    if len(mask_x) == 0: # No color visible
        return None
    else:
        return mask
    # return mask


def get_white_mask(hsv):
    hue_low = 0
    hue_high = 360

    sat_low = max(0, HSV_REAL_WHITE[1] - sat_margin) 
    sat_high = min(100, HSV_REAL_WHITE[1] + sat_margin)

    val_low = max(0, HSV_REAL_WHITE[2] - val_margin) 
    val_high = min(100, HSV_REAL_WHITE[2] + sat_margin)

    return get_mask(hsv, hue_low, hue_high, sat_low, sat_high, val_low, val_high)


def get_orange_mask(hsv):
    hue_low = max(0, HSV_REAL_ORANGE[0] - hue_margin) 
    hue_high = min(360, HSV_REAL_ORANGE[0] + hue_margin)

    sat_low = max(0, HSV_REAL_ORANGE[1] - sat_margin) 
    sat_high = min(100, HSV_REAL_ORANGE[1] + sat_margin)

    val_low = max(0, HSV_REAL_ORANGE[2] - val_margin) 
    val_high = min(100, HSV_REAL_ORANGE[2] + sat_margin)

    return get_mask(hsv, hue_low, hue_high, sat_low, sat_high, val_low, val_high)


def get_green_mask(hsv):
    hue_low = max(0, HSV_REAL_GREEN[0] - hue_margin) 
    hue_high = min(360, HSV_REAL_GREEN[0] + hue_margin)

    sat_low = max(0, HSV_REAL_GREEN[1] - sat_margin) 
    sat_high = min(100, HSV_REAL_GREEN[1] + sat_margin)

    val_low = max(0, HSV_REAL_GREEN[2] - val_margin) 
    val_high = min(100, HSV_REAL_GREEN[2] + sat_margin)

    return get_mask(hsv, hue_low, hue_high, sat_low, sat_high, val_low, val_high)


def flood_fill(img, start=(0,0)):
    h,w = img.shape
    seed = start

    mask = np.zeros((h+2,w+2),np.uint8) # Adding a padding of 1

    if img[start[1]][start[0]] == 255: # If the starting point is already filled, return empty mask
        mask = mask[1:h+1,1:w+1] # Removing the padding
        return mask

    floodflags = 8
    # floodflags |= cv2.FLOODFILL_FIXED_RANGE
    floodflags |= cv2.FLOODFILL_MASK_ONLY
    floodflags |= (255 << 8)

    num,img,mask,rect = cv2.floodFill(img, mask, seed, (255,0,0), (10,)*3, (10,)*3, floodflags)
    mask = mask[1:h+1,1:w+1] # Removing the padding
    
    return mask


def get_pixels_inside_green(hsv):
    """ 
        Function that finds the green in an image, make a bounding box around it,
        fits an ellipse in the bounding box
        and paints everything outside the ellipse in black.

        Returns the painted image and a boolean stating wheather any green was found.
     """

    hsv_inside_green = hsv.copy()

    # bw_green_mask = get_green_mask(hsv)  
    # hsv_save_image(bw_green_mask, "1b_green_mask", is_gray=True)

    # bw_green_mask = make_gaussian_blurry(bw_green_mask, 5)

    # bw_green_mask_flood_01 = flood_fill(bw_green_mask, start=(0,0))
    # bw_green_mask_flood_02 = flood_fill(bw_green_mask, start=(IMG_WIDTH-1,0))
    # bw_green_mask_flood_03 = flood_fill(bw_green_mask, start=(0,IMG_HEIGHT-1))
    # bw_green_mask_flood_04 = flood_fill(bw_green_mask, start=(IMG_WIDTH-1,IMG_HEIGHT-1))

    # bw_green_mask_flood_combined = cv2.bitwise_or(bw_green_mask_flood_01, bw_green_mask_flood_02, bw_green_mask_flood_03, bw_green_mask_flood_04)

    # mask = cv2.bitwise_not(bw_green_mask_flood_combined)
    # hsv_inside_green = cv2.bitwise_or(hsv_inside_green, hsv, mask=mask)

    ## Ellipse method
    bw_green_mask = get_green_mask(hsv)  
    # If no green in image: return original image
    if bw_green_mask is None:
        return hsv


    bw_ellipse_mask = np.zeros((IMG_HEIGHT, IMG_WIDTH))
    hsv_ellipse = hsv.copy()

    green_x, green_y = np.where(bw_green_mask==255)
    x_min = np.amin(green_x)
    x_max = np.amax(green_x)
    y_min = np.amin(green_y)
    y_max = np.amax(green_y)

    center_x = np.int0((x_min + x_max) / 2.0)
    center_y = np.int0((y_min + y_max) / 2.0)

    l_x = np.int0(x_max - center_x)+1
    l_y = np.int0(y_max - center_y)+1

    cv2.ellipse(img=bw_ellipse_mask, center=(center_y, center_x),
        axes=(l_y, l_x), angle=0, startAngle=0, endAngle=360,
        color=(255), thickness=-1, lineType=8, shift=0)

    hsv_ellipse[bw_ellipse_mask==0] = HSV_BLACK_COLOR

    return hsv_ellipse


def get_pixels_inside_orange(hsv):
    """ 
        Function that finds the orange in an image, make a bounding box around it,
        and paints everything outside the box in black.

        Returns the painted image and a boolean stating wheather any orange was found.
     """
    
    hsv_inside_orange = hsv.copy()

    bw_orange_mask = get_orange_mask(hsv) 
    if bw_orange_mask is None:
        return hsv
    hsv_save_image(bw_orange_mask, "2b_orange_mask", is_gray=True)

    orange_x, orange_y = np.where(bw_orange_mask==255)
    # If no orange in image: return original image
    if len(orange_x) == 0:
        x_min = 0
        x_max = IMG_HEIGHT - 1
        y_min = 0
        y_max = IMG_WIDTH - 1
    else:
        x_min = np.amin(orange_x)
        x_max = np.amax(orange_x)
        y_min = np.amin(orange_y)
        y_max = np.amax(orange_y)
    

    hsv_inside_orange[0:x_min,] = HSV_BLACK_COLOR
    hsv_inside_orange[x_max+1:,] = HSV_BLACK_COLOR

    hsv_inside_orange[:,0:y_min] = HSV_BLACK_COLOR
    hsv_inside_orange[:,y_max+1:] = HSV_BLACK_COLOR

    hsv_save_image(hsv_inside_orange, '3_inside_orange')

    return hsv_inside_orange


def find_white_centroid(hsv):

    hsv_inside_orange = get_pixels_inside_orange(hsv)

    bw_white_mask = get_white_mask(hsv_inside_orange)
    if bw_white_mask is None:
        rospy.loginfo("bw_white_mask is none")
        return None
    bw_white_mask = make_gaussian_blurry(bw_white_mask, 5)

	# calculate moments of binary image
    M = cv2.moments(bw_white_mask)

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
        return None, None

    corners, intensities = clip_corners_on_intensity(corners, img, average_filter_size)
    if corners is None:
        # Found no corners
        return None, None

    return corners, intensities


def find_orange_arrowhead(hsv):
    bw_orange_mask = get_orange_mask(hsv)
    if bw_orange_mask is None:
        return None

    bw_orange_mask = make_gaussian_blurry(bw_orange_mask, 5) 

    bw_orange_mask_inverted = cv2.bitwise_not(bw_orange_mask)

    hsv_save_image(bw_orange_mask_inverted, "0_orange_mask_inverted", is_gray=True)


    orange_corners, intensities = find_right_angled_corners(bw_orange_mask_inverted)

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


def fit_ellipse(points):
    x = points[1]
    y = IMG_HEIGHT-points[0]

    D11 = np.square(x)
    D12 = x*y
    D13 = np.square(y)
    D1 = np.array([D11, D12, D13]).T
    D2 = np.array([x, y, np.ones(x.shape[0])]).T

    S1 = np.dot(D1.T,D1)
    S2 = np.dot(D1.T,D2)
    S3 = np.dot(D2.T,D2)

    try:
        inv_S3 = np.linalg.inv(S3)
    except np.linalg.LinAlgError:
        print("fit_ellipse(): Got singular matrix")
        return None

    T = - np.dot(inv_S3, S2.T) # for getting a2 from a1

    M = S1 + np.dot(S2, T)

    C1 = np.array([
        [0, 0, 0.5],
        [0, -1, 0],
        [0.5, 0, 0]
    ])

    M = np.dot(C1, M) # This premultiplication can possibly be made more efficient
    
    eigenvalues, eigenvectors = np.linalg.eig(M)
    cond = 4*eigenvectors[0]*eigenvectors[2] - np.square(eigenvectors[0])
    a1 = eigenvectors[:,cond > 0]
    
    # Choose the first if there are two eigenvectors with cond > 0
    # NB! I am not sure if this is always correct
    if a1.shape[1] > 1:
        a1 = np.array([a1[:,0]]).T

    if a1.shape != (3,1): # Make sure a1 has content
        print("fit_ellipse(): a1 not OK")
        return None

    a = np.concatenate((a1, np.dot(T, a1)))[:,0] # Choose the inner column with [:,0]

    if np.any(np.iscomplex(a)):
        print("Found complex number")
        return None
    else:
        return a


def get_ellipse_parameters(green_ellipse):
    edges = cv2.Canny(green_ellipse,100,200)
    result = np.where(edges == 255)

    ellipse = fit_ellipse(result)

    if ellipse is None:
        return None

    A = ellipse[0]
    B = ellipse[1]
    C = ellipse[2]
    D = ellipse[3]
    E = ellipse[4]
    F = ellipse[5]

    # print(A)
    if B**2 - 4*A*C >= 0:
        print("get_ellipse_parameters(): Shape found is not an ellipse")
        return None

    inner_square = math.sqrt( (A-C)**2 + B**2)
    outside = 1.0 / (B**2 - 4*A*C)
    a = outside * math.sqrt(2*(A*E**2 + C*D**2 - B*D*E + (B**2 - 4*A*C)*F) * ( (A+C) + inner_square))
    b = outside * math.sqrt(2*(A*E**2 + C*D**2 - B*D*E + (B**2 - 4*A*C)*F) * ( (A+C) - inner_square))

    x_raw = (2.0*C*D - B*E) / (B*B - 4.0*A*C) 
    y_raw = (2.0*A*E - B*D) / (B*B - 4.0*A*C)

    # y_0 = (2.0*C*D - B*E) / (B*B - 4.0*A*C) 
    # x_0 = IMG_HEIGHT - (2.0*A*E - B*D) / (B*B - 4.0*A*C) - 1
    
    x_0 = (IMG_HEIGHT - 1) - y_raw
    y_0 = x_raw

    # print(a)
    # ellipse_and_a_b = np.array([A,B,C,D,E,F,a,b])
    ellipse_and_a_b = np.array([x_0,y_0,a,b])

    return ellipse_and_a_b


def evaluate_ellipse(hsv):
    """ Use the green ellipse to find: 
        center, radius, angle 
    """
    bw_green_mask = get_green_mask(hsv)
    if bw_green_mask is None:
        return None, None, None

    hsv_save_image(bw_green_mask, "2_green_mask", is_gray=True)

    top_border =    bw_green_mask[0,:]
    bottom_border = bw_green_mask[IMG_HEIGHT-1,:]
    left_border =   bw_green_mask[:,0]
    right_border =  bw_green_mask[:,IMG_WIDTH-1]
    
    sum_top_border = np.sum(top_border) + \
        np.sum(bottom_border) + \
        np.sum(left_border) + \
        np.sum(right_border)

    # print sum_top_border

    if sum_top_border != 0: 
        # Then the green ellipse is toughing the border
        return None, None, None
    
    bw_green_ellipse = flood_fill(bw_green_mask, start=(0,0))
    hsv_save_image(bw_green_ellipse, "3_green_ellipse", is_gray=True)


    ellipse_parameters = get_ellipse_parameters(bw_green_ellipse)
    if ellipse_parameters is None:
        return None, None, None

    # Choose the largest of the a and b parameter for the radius
    # Choose angle = 0 since it is not possible to estimate from the ellipse
    center_px = ellipse_parameters[0:2]
    radius_px = np.amax(np.abs(ellipse_parameters[2:4]))
    angle = 0

    # print "center_px:", center_px

    hsv_canvas_ellipse = hsv.copy()
    draw_dot(hsv_canvas_ellipse, center_px, HSV_BLUE_COLOR)
    hsv_save_image(hsv_canvas_ellipse, "4_canvas_ellipse") #, is_gray=True)

    return center_px, radius_px, angle


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
        
        angle = calc_angle_between_vectors(arrow_vector, ref_vector)

        arrow_length_px = np.linalg.norm(arrow_vector)
        # Use known relation between the real radius and the real arrow length
        # to find the radius length in pixels

        # (old way) radius_length_px = arrow_length_px * RELATION_R_L3
        radius_length_px = arrow_length_px * D_RADIUS / D_ARROW

        hsv_canvas_arrow = hsv.copy()
        draw_dot(hsv_canvas_arrow, center_px, HSV_RED_COLOR)
        draw_dot(hsv_canvas_arrow, arrowhead_px, HSV_RED_COLOR)
        hsv_save_image(hsv_canvas_arrow, "4_canvas_arrow")

        return center_px, radius_length_px, angle
        
    else:
        return None, None, None



def get_relevant_corners(inner_corners):
    n_inner_corners = len(inner_corners)

    # For the first corner, chose the corner closest to the top
    # This will belong to the top cross-bar
    top_corner_id = np.argmin(inner_corners[:,0]) # Find lowest x-index
    top_corner = inner_corners[top_corner_id]
    top_corner_stack = np.array([top_corner]*(n_inner_corners-1))
    rest_corners = np.delete(inner_corners, top_corner_id, 0)
    dist = np.linalg.norm(rest_corners - top_corner_stack, axis=1)

    # For the second corner, chose the corner closest to top corner
    top_corner_closest_id = np.argmin(dist)
    top_corner_closest = rest_corners[top_corner_closest_id]

    relevant_corners = np.stack((top_corner, top_corner_closest), axis=0)

    return relevant_corners


def evaluate_inner_corners(hsv):
    """ Use the inner corners to find: 
        center, radius, angle 
    """
    hsv_canvas = hsv.copy()

    bw_white_mask = get_white_mask(hsv)
    if bw_white_mask is None:
        return None, None, None
    bw_white_mask = make_gaussian_blurry(bw_white_mask, 5)

    hsv_save_image(bw_white_mask, "0_white_only", is_gray=True)

    inner_corners, intensities = find_right_angled_corners(bw_white_mask)

    average_filter_size = 19
    img_average_intensity = make_circle_average_blurry(bw_white_mask, average_filter_size)

    if (inner_corners is not None):
        n_inner_corners = len(inner_corners)
        # print "n_inner_corners:", n_inner_corners

        if (n_inner_corners > 1) and (n_inner_corners <= 5):
            
            unique_corners = np.vstack({tuple(row) for row in inner_corners}) # Removes duplicate corners
            
            for corner in unique_corners:
                draw_dot(hsv_canvas, corner, HSV_YELLOW_COLOR)

            corner_a, corner_b = get_relevant_corners(unique_corners)

            draw_dot(hsv_canvas, corner_a, HSV_RED_COLOR)
            draw_dot(hsv_canvas, corner_b, HSV_LIGHT_ORANGE_COLOR)

            c_m = get_mid_point(corner_a, corner_b)
            draw_dot(hsv_canvas, c_m, HSV_BLUE_COLOR)

            hsv_save_image(hsv_canvas, "3_canvas")

            c_m_value = img_average_intensity[np.int0(c_m[0])][np.int0(c_m[1])]
            # print "c_m_value", c_m_value

            if c_m_value > 190: # The points are on a short side
                # print "Short side"
                is_short_side = True
                normal_vector = get_normal_vector(bw_white_mask, corner_a, corner_b, is_short_side)
                normal_unit_vector = normalize_vector(normal_vector)

                length_short_side = np.linalg.norm(corner_a - corner_b)
                length_long_side = length_short_side * D_H_LONG/D_H_SHORT
                length_to_center = - length_long_side / 2.0
                length_radius = length_short_side * D_RADIUS/D_H_SHORT

                forward_unit_vector = normal_unit_vector

            else: # The points are on a long side
                print "Long side"
                is_short_side = False
                normal_vector = get_normal_vector(bw_white_mask, corner_a, corner_b, is_short_side)
                normal_unit_vector = normalize_vector(normal_vector)

                length_long_side = np.linalg.norm(corner_a - corner_b)
                length_short_side = length_long_side * D_H_SHORT/D_H_LONG
                length_to_center = length_short_side / 2.0
                length_radius = length_long_side * D_RADIUS/D_H_LONG

                forward_unit_vector = normalize_vector(corner_a - corner_b)
            
            end = c_m + forward_unit_vector*10
            draw_arrow(hsv_canvas, c_m, end)

            center = c_m + normal_unit_vector*length_to_center
            draw_dot(hsv_canvas, center, HSV_BLUE_COLOR)

            # hsv_save_image(hsv_canvas, "3_canvas")
                      
            neg_x_axis = np.array([-1,0])
            angle = calc_angle_between_vectors(forward_unit_vector, neg_x_axis)
        
            hsv_canvas_inner_corners = hsv.copy()
            draw_dot(hsv_canvas_inner_corners, center, HSV_LIGHT_ORANGE_COLOR)
            hsv_save_image(hsv_canvas_inner_corners, "4_canvas_inner_corners")

            return center, length_radius, angle

    return None, None, None


def print_data_on_a_line(title, data_gt, data_est_e, data_est_a, data_est_i):
    rjust = 12
    rjust_2 = 10
    # print "|     || Ground Truth ||   Method 1 |   Method 2 |   Method 3 ||"
    print "||-----||--------------||------------|------------|------------||"

    text_gt = '{:.2f}'.format(round(data_gt, 2)).rjust(rjust)
    text_est_e = '{:.2f}'.format(round(data_est_e, 2)).rjust(rjust_2)
    text_est_a = '{:.2f}'.format(round(data_est_a, 2)).rjust(rjust_2)
    text_est_i = '{:.2f}'.format(round(data_est_i, 2)).rjust(rjust_2)

    print "||", title.rjust(3), "||", text_gt, "||",text_est_e, \
            "|", text_est_a, "|",text_est_i, "||"


def print_data_on_a_line_2(title, data):
    rjust_gt = 12
    rjust_est = 10
    # print "|     || Ground Truth ||   Method 1 |   Method 2 |   Method 3 ||"
    print "||-----||--------------||------------|------------|------------||"

    text_gt = data[0].rjust(rjust_gt)
    text_est_e = data[1].rjust(rjust_est)
    text_est_a = data[2].rjust(rjust_est)
    text_est_i = data[3].rjust(rjust_est)

    print "||", title.rjust(3), "||", text_gt, "||",text_est_e, \
            "|", text_est_a, "|",text_est_i, "||"




def run(img_count = 0):
    dataset_id = 2
    filepath = "dataset/low_flight_dataset_0"+str(dataset_id)+"/image_"+str(img_count)+".png"
    # filepath = "dataset/image_2_corners_long_side.jpg"
    # filepath = "dataset/white_corner_test.png"
    
    hsv = load_hsv_image(filepath)

    hsv_inside_green = get_pixels_inside_green(hsv)

    center_px_from_ellipse, radius_length_px_from_ellipse, angle_from_ellipse = evaluate_ellipse(hsv)
    center_px_from_arrow, radius_length_px_from_arrow, angle_from_arrow = evaluate_arrow(hsv) # or use hsv_inside_green
    center_px_from_inner_corners, radius_px_length_from_inner_corners, angle_from_inner_corners = evaluate_inner_corners(hsv_inside_green)


    hsv_canvas_all = hsv.copy()


    ############
    # Method 1 #
    ############
    if (center_px_from_ellipse is not None):
        center_px, radius_length_px, angle_rad = center_px_from_ellipse, radius_length_px_from_ellipse, angle_from_ellipse
        est_ellipse_x, est_ellipse_y, est_ellipse_z = calculate_position(center_px, radius_length_px)
        est_ellipse_angle = np.degrees(angle_rad)

        draw_dot(hsv_canvas_all, center_px, HSV_BLUE_COLOR, size=5)
        # print "Position from ellipse:", est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle
    else:
        # est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle = None, None, None, None
        est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle = 0.0, 0.0, 0.0, 0.0
        # print "Position from ellipse: [Not available]"

    ############
    # Method 2 #
    ############
    if (center_px_from_arrow is not None):
        center_px, radius_length_px, angle_rad = center_px_from_arrow, radius_length_px_from_arrow, angle_from_arrow
        est_arrow_x, est_arrow_y, est_arrow_z = calculate_position(center_px, radius_length_px)
        est_arrow_angle = np.degrees(angle_rad)

        draw_dot(hsv_canvas_all, center_px, HSV_RED_COLOR, size=4)
        # print "Position from arrow:", est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle
    else:
        # est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle = None, None, None, None
        est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle = 0.0, 0.0, 0.0, 0.0
        # print "Position from arrow: [Not available]"

    ############
    # Method 3 #
    ############
    if (center_px_from_inner_corners is not None):
        center_px, radius_length_px, angle_rad = center_px_from_inner_corners, radius_px_length_from_inner_corners, angle_from_inner_corners
        est_inner_corners_x, est_inner_corners_y, est_inner_corners_z = calculate_position(center_px, radius_length_px)
        est_inner_corners_angle = np.degrees(angle_rad)

        draw_dot(hsv_canvas_all, center_px, HSV_LIGHT_ORANGE_COLOR, size=3)
        # print "Position from inner corners:", est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle
    else:
        # est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle = None, None, None, None
        est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle = 0.0, 0.0, 0.0, 0.0
        # print "Position from inner corners: [Not available]"

    hsv_save_image(hsv_canvas_all, "5_canvas_all")

    json_filepath = "dataset/low_flight_dataset_0"+str(dataset_id)+"/low_flight_dataset.json"
    with open(json_filepath) as json_file:
        data = json.load(json_file)
        gt_x, gt_y, gt_z = np.array(data[str(img_count)]['ground_truth'][0:3])*1000
        gt_yaw_raw = np.degrees(np.array(data[str(img_count)]['ground_truth'][3]))
        
        # Counter-clockwise is positive angle and zero degrees is rotated 90 degrees to the left
        gt_yaw = gt_yaw_raw*(-1) - 90

        if gt_yaw <= -180:
            gt_yaw += 360

    results = np.array([
        [gt_x, gt_y, gt_z, gt_yaw],
        [est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle],
        [est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle],
        [est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle]])



def ros_run(hsv, count):
    hsv_save_image(hsv, '0_hsv')


    white_mask = get_white_mask(hsv)
    if white_mask is not None:
        hsv_save_image(white_mask, '1_white_mask', is_gray=True)

    orange_mask = get_orange_mask(hsv)
    if orange_mask is not None:
        hsv_save_image(orange_mask, '1_orange_mask', is_gray=True)

    green_mask = get_green_mask(hsv)
    if green_mask is not None:
        hsv_save_image(green_mask, '1_green_mask', is_gray=True)


    hsv_inside_green = get_pixels_inside_green(hsv)

    center_px_from_ellipse, radius_length_px_from_ellipse, angle_from_ellipse = evaluate_ellipse(hsv)
    center_px_from_arrow, radius_length_px_from_arrow, angle_from_arrow = evaluate_arrow(hsv) # or use hsv_inside_green
    center_px_from_inner_corners, radius_px_length_from_inner_corners, angle_from_inner_corners = evaluate_inner_corners(hsv_inside_green)

    hsv_canvas_all = hsv.copy()

    ############
    # Method 1 #
    ############
    if (center_px_from_ellipse is not None):
        center_px, radius_length_px, angle_rad = center_px_from_ellipse, radius_length_px_from_ellipse, angle_from_ellipse
        est_ellipse_x, est_ellipse_y, est_ellipse_z = calculate_position(center_px, radius_length_px)
        est_ellipse_angle = np.degrees(angle_rad)

        draw_dot(hsv_canvas_all, center_px, HSV_BLUE_COLOR, size=5)
        # print "Position from ellipse:", est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle
    else:
        # est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle = None, None, None, None
        est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle = 0.0, 0.0, 0.0, 0.0
        # print "Position from ellipse: [Not available]"

    ############
    # Method 2 #
    ############
    if (center_px_from_arrow is not None):
        center_px, radius_length_px, angle_rad = center_px_from_arrow, radius_length_px_from_arrow, angle_from_arrow
        est_arrow_x, est_arrow_y, est_arrow_z = calculate_position(center_px, radius_length_px)
        est_arrow_angle = np.degrees(angle_rad)

        draw_dot(hsv_canvas_all, center_px, HSV_RED_COLOR, size=4)
        # print "Position from arrow:", est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle
    else:
        # est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle = None, None, None, None
        est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle = 0.0, 0.0, 0.0, 0.0
        # print "Position from arrow: [Not available]"

    ############
    # Method 3 #
    ############
    if (center_px_from_inner_corners is not None):
        center_px, radius_length_px, angle_rad = center_px_from_inner_corners, radius_px_length_from_inner_corners, angle_from_inner_corners
        est_inner_corners_x, est_inner_corners_y, est_inner_corners_z = calculate_position(center_px, radius_length_px)
        est_inner_corners_angle = np.degrees(angle_rad)

        draw_dot(hsv_canvas_all, center_px, HSV_LIGHT_ORANGE_COLOR, size=3)
        # print "Position from inner corners:", est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle
    else:
        # est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle = None, None, None, None
        est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle = 0.0, 0.0, 0.0, 0.0
        # print "Position from inner corners: [Not available]"

    hsv_save_image(hsv_canvas_all, "5_canvas_all_"+str(count))

    result = np.array([
        # [est_ellipse_x, est_ellipse_y, est_ellipse_z, est_ellipse_angle],
        [est_arrow_x, est_arrow_y, est_arrow_z, est_arrow_angle],
        # [est_inner_corners_x, est_inner_corners_y, est_inner_corners_z, est_inner_corners_angle]        
    ])

    return result
    

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


# single_image_index = 40
# single_image = False
# len_dataset = 40

# results_gt = []
# results_est = []

# if single_image:
#     print "###########################"
#     print "# Now testing on image", str(single_image_index).rjust(2), "#"
#     print "###########################"
#     # text = "Now testing on image " + str(single_image_index).rjust(2)
#     # print_header(str())
#     run(single_image_index)
# else:
#     for image_index in range(len_dataset):
#         text = "Now testing on image " + str(image_index).rjust(2)
#         print_header(text)
#         # print "###########################"
#         # print "# Now testing on image", str(image_index).rjust(2), "#"
#         # print "###########################"
#         run(image_index)
#         # answer = raw_input("Press enter for next image")
#         answer = raw_input("")


# else:
#     for i in range(3):
#         # answer = raw_input("Press enter for next image")
#         print ""
#         print "###########################"
#         print "# Running CV module on image", i, "#"
#         print "###########################"
#         length, centroid = run(i)
#         print ""
#         print "# Preprocessing"
#         print "length, centroid:", length, centroid
#         print ""

#         if length is not None:
#             json_filepath = "dataset/low_flight_dataset_02/low_flight_dataset.json"
#             with open(json_filepath) as json_file:
#                 data = json.load(json_file)
#                 gt_x, gt_y, gt_z = np.array(data[str(i)]['ground_truth'][0:3])*1000
#                 results_gt.append([gt_x, gt_y, gt_z])
#                 print "GT: ", gt_x, gt_y, gt_z

#             est_x, est_y, est_z = calculate_position(centroid, length)

#             print "Est:", est_x, est_y, est_z
#             results_est.append([est_x, est_y, est_z])


#     np_results_gt = np.array(results_gt)
#     np_results_est = np.array(results_est)
#     # print "Results ground truth"
#     # print np_results_gt
#     # print "Results estimate"
#     # print np_results_est

#     print_header("Showing results")

#     n_results = len(np_results_gt)
#     print "n_results:", n_results
#     rjust = 7
#     print "||  gt_x   |  est_x  ||  gt_y   |  est_y  ||  gt_z   |  est_z  ||"
#     print "-----------------------------------------------------------------"
#     for i in range(n_results):

#         text_gt_x = '{:.2f}'.format(round(np_results_gt[i][0], 2)).rjust(rjust)
#         text_est_x = '{:.2f}'.format(round(np_results_est[i][0], 2)).rjust(rjust)

#         text_gt_y = '{:.2f}'.format(round(np_results_gt[i][1], 2)).rjust(rjust)
#         text_est_y = '{:.2f}'.format(round(np_results_est[i][1], 2)).rjust(rjust)

#         text_gt_z = '{:.2f}'.format(round(np_results_gt[i][2], 2)).rjust(rjust)
#         text_est_z = '{:.2f}'.format(round(np_results_est[i][2], 2)).rjust(rjust)

#         print "||", text_gt_x, "|",text_est_x, \
#             "||", text_gt_y, "|",text_est_y, \
#             "||", text_gt_z, "|",text_est_z, "||"


def rel_gt_converter(rel_gt):
    """ Convert from twist message data to x, y, z, yaw in cm and degrees """
    gt_x = rel_gt.linear.x * 1000
    gt_y = rel_gt.linear.y * 1000
    gt_z = rel_gt.linear.z * 1000
    yaw = -np.degrees(rel_gt.angular.z) - 90
    
    if yaw < -180:
        gt_yaw = 360 + yaw
    else:
        gt_yaw = yaw

    return np.array([[gt_x, gt_y, gt_z, gt_yaw]])

def main():
    rospy.init_node('cv_module', anonymous=True)

    rospy.Subscriber('/ardrone/bottom/image_raw', Image, image_callback)
    rospy.Subscriber('/drone_pose', Twist, rel_gt_callback)

    pub_heartbeat = rospy.Publisher("/heartbeat", Empty, queue_size=10)
    heartbeat_msg = Empty()

    rospy.loginfo("Starting CV module")

    count = 0
    rate = rospy.Rate(20) # Hz
    while not rospy.is_shutdown():

        if (global_image is not None):
            pub_heartbeat.publish(heartbeat_msg)

            # rospy.loginfo("Received image")
            hsv = cv2.cvtColor(global_image, cv2.COLOR_BGR2HSV) # convert to HSV
            est = ros_run(hsv, count)
            gt = rel_gt_converter(global_rel_gt)
            result = np.concatenate((est, gt))

            # print rel_gt_converter(global_rel_gt)
            # print result
            # print ""
            count += 1
        
        else:
            rospy.loginfo("Waiting for image")

        rate.sleep()
    
    
if __name__ == '__main__':
    main()