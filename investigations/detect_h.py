import matplotlib.pyplot as plt
import cv2
import numpy as np
import math


IMG_WIDTH = 640
IMG_HEIGHT = 360

# For converting between units
conv_scale_to_bits = np.array([0.5, 2.55, 2.55]) # unit bits with ranges [[0,180], [0,255], [0,255]]

def make_blurry(image, blur):
    return cv2.medianBlur(image, blur)


def make_gaussian_blurry(image, blur):
    return cv2.GaussianBlur(image, (blur, blur), 0)


def hsv_make_orange_to_green(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    lower_orange = np.array([8,128,64])
    upper_orange = np.array([28,255,255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # change the orange to green
    imask = mask>0
    orange_to_green = hsv.copy()
    orange_to_green[imask] = hsv_green

    return orange_to_green


def hsv_keep_orange_only(hsv):
    bgr_white = np.uint8([[[255,255,255]]])
    hsv_white = cv2.cvtColor(bgr_white,cv2.COLOR_BGR2HSV)

    bgr_black = np.uint8([[[0,0,0]]])
    hsv_black = cv2.cvtColor(bgr_black,cv2.COLOR_BGR2HSV)

    lower_orange = np.array([8,128,64])
    upper_orange = np.array([28,255,255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # change the orange to green
    imask = mask>0
    b_imask = mask<=0
    orange_to_green = hsv.copy()
    orange_to_green[imask] = hsv_white
    orange_to_green[b_imask] = hsv_black

    return orange_to_green





def hsv_find_green_mask(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    lower_green_h = 50
    lower_green_s = 50 * 0.01*255
    lower_green_v = 25 * 0.01*255

    upper_green_h = 70
    upper_green_s = 100 * 0.01*255
    upper_green_v = 70 * 0.01*255

    lower_green = np.array([lower_green_h,lower_green_s,lower_green_v])
    upper_green = np.array([upper_green_h,upper_green_s,upper_green_v])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
  
    # keep only the green
    imask = green_mask>0
    green = np.zeros_like(hsv, np.uint8)
    green[imask] = hsv_green
  
    return green



def hsv_make_grayscale(image):
    bgr = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return gray


def hsv_save_image(image, label='image', is_gray=False):
    folder = 'image_processing/detect_h/'
    if is_gray:
        cv2.imwrite(folder+label+".png", image)
    else:
        cv2.imwrite(folder+label+".png", cv2.cvtColor(image, cv2.COLOR_HSV2BGR))

    return image


def load_image(filename):
    img = cv2.imread(filename) # import as BGR
    return img

    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # hsv = hsv_save_image(hsv, '1_hsv')
    # # ### Preprocessing ###
    # hsv = hsv_save_image(hsv_make_orange_to_green(hsv), '2_orange_to_green')
    # hsv = make_blurry(hsv, 9)
    # hsv = hsv_save_image(make_blurry(hsv, 3), '3_make_blurry')
    # hsv = hsv_save_image(hsv_find_green_mask(hsv), '4_green_mask')
    # # hsv = make_blurry(hsv, 9)
    # # hsv = hsv_save_image(make_blurry(hsv, 9), '5_make_blurry_2')
    # # gray = hsv_save_image(hsv_make_grayscale(hsv), '6_make_grayscale', is_gray=True)
    # gray = hsv
    # # gray = hsv_save_image(make_blurry(gray, 31), "6_make_blurry_3")
    # # gray = hsv_save_image(make_blurry(gray, 21), '7_make_blurry_3', is_gray=True)

    # return gray



def find_contours(img):
    ret, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # create hull array for convex hull points
    hull = []
    
    # calculate points for each contour
    for i in range(len(contours)):
        # creating convex hull object for each contour
        hull.append(cv2.convexHull(contours[i], False))

    # create an empty black image
    drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)
    
    # draw contours and hull points
    for i in range(len(contours)):
        color_contours = (0, 255, 0) # green - color for contours
        color = (255, 0, 0) # blue - color for convex hull
        # draw ith contour
        cv2.drawContours(drawing, contours, i, color_contours, 1, 8, hierarchy)
        # draw ith convex hull object
        cv2.drawContours(drawing, hull, i, color, 1, 8)

    gray = hsv_save_image(drawing, '0_hull', is_gray=True)



def flood_fill(filename = "image_above.jpg"):
    im = load_image(filename)
    h,w,chn = im.shape
    seed = (w/2,h/2)
    seed = (0,0)
    # print(h, ", ", w)

    mask = np.zeros((h+2,w+2),np.uint8) # Adding a padding of 1

    floodflags = 8
    # floodflags |= cv2.FLOODFILL_FIXED_RANGE
    floodflags |= cv2.FLOODFILL_MASK_ONLY
    floodflags |= (255 << 8)

    num,im,mask,rect = cv2.floodFill(im, mask, seed, (255,0,0), (10,)*3, (10,)*3, floodflags)
    mask = mask[1:h+1,1:w+1] # Removing the padding


    hsv_save_image(mask, "flood_fill", is_gray=True)


    print(mask)

    # plt.imshow(im)
    # plt.show()

    # plt.imshow(mask)
    # plt.show()

    return mask

#############
# New stuff #
#############

def hsv_keep_white_only(hsv):
    bgr_white = np.uint8([[[255,255,255]]])
    hsv_white = cv2.cvtColor(bgr_white,cv2.COLOR_BGR2HSV)

    bgr_black = np.uint8([[[0,0,0]]])
    hsv_black = cv2.cvtColor(bgr_black,cv2.COLOR_BGR2HSV)

    # In degrees, %, %, ranges [[0,360], [0,100], [0,100]]
    lower_white = np.array([  0,   0,  95])*conv_scale_to_bits
    upper_white = np.array([360,  40, 100])*conv_scale_to_bits

    mask = cv2.inRange(hsv, lower_white, upper_white)

    # change the white to black
    imask = mask>0
    b_imask = mask<=0
    orange_to_green = hsv.copy()
    orange_to_green[imask] = hsv_white
    orange_to_green[b_imask] = hsv_black

    return mask


def sharpen(img):
    kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    img = cv2.filter2D(img, -1, kernel)
    return img


def crop_on_orange(hsv):
    bgr_black = np.uint8([[[0,0,0]]])   
    hsv_black = cv2.cvtColor(bgr_black,cv2.COLOR_BGR2HSV)

    lower_orange = np.array([8,128,64])
    upper_orange = np.array([28,255,255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    orange_ids = np.where(mask==255)

    orange_ids_x = orange_ids[0]
    orange_ids_y = orange_ids[1]

    if orange_ids_x.shape[0] == 0: # No orange in image
        return hsv

    # x_min = max(np.amin(orange_ids_x), 0)
    # x_max = min(np.amax(orange_ids_x), IMG_HEIGHT)
    # y_min = max(np.amin(orange_ids_y), 0)
    # y_max = min(np.amax(orange_ids_y), IMG_WIDTH)

    x_min = np.amin(orange_ids_x)
    x_max = np.amax(orange_ids_x)
    y_min = np.amin(orange_ids_y)
    y_max = np.amax(orange_ids_y)


    x_range = range(x_min) + range(x_max, IMG_HEIGHT)
    y_range = range(y_min) + range(y_max, IMG_WIDTH)

    # print('x_range:', x_range)
    # print('y_range:', y_range)


    crop = hsv.copy()
    print(crop.shape)
    crop[x_range] = hsv_black
    crop[:,y_range] = hsv_black


    return crop
    


def hsv_find_green_mask_2(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    # In degrees, %, %, ranges [[0,360], [0,100], [0,100]]
    lower_green = np.array([ 65,  10,  25])*conv_scale_to_bits
    upper_green = np.array([ 175, 100,  95])*conv_scale_to_bits
    
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
  
    # keep only the green
    imask = green_mask>0
    green = np.zeros_like(hsv, np.uint8)
    green[imask] = hsv_green
  
    return green



def find_corners(img, hsv_origin):

    bgr_red = np.uint8([[[0,0,255]]])
    hsv_red = cv2.cvtColor(bgr_red,cv2.COLOR_BGR2HSV)

    edges = cv2.Canny(img,100,200)

    # hsv_save_image(edges, "3_edges", is_gray=True)

    contour = np.where(edges==255)

    # print(contour)

    x_axis = contour[0]
    y_axis = contour[1]

    # x_mean = np.mean()

    x_min = np.amin(x_axis)
    x_min_ids = np.where(x_axis == x_min)

    x_max = np.amax(x_axis)
    x_max_ids = np.where(x_axis == x_max)

    y_min = np.amin(y_axis)
    y_min_ids = np.where(y_axis == y_min)

    y_max = np.amax(y_axis)
    y_max_ids = np.where(y_axis == y_max)

    x_avr = (x_min + x_max) / 2.0
    y_avr = (y_min + y_max) / 2.0

    # print('x_min:', x_min)
    # print('x_min_ids:', x_min_ids)
    # print('x_max:', x_max)
    # print('x_max_ids:', x_max_ids)

    # print('y_min:', y_min)
    # print('y_min_ids:', y_min_ids)
    # print('y_max:', y_max)
    # print('y_max_ids:', y_max_ids)

    # print('x_avr:', x_avr)
    # print('y_avr:', y_avr)


    # corners = hsv_origin.copy()
    bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    corners = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Top-left corner
    x = x_min
    ys = y_axis[x_min_ids]
    if np.mean(ys) - y_avr <= 0: # Choose the left (smallest) y, if zero go left
        y = np.amin(ys)
    else: # Choose the right (largest) y
        y = np.amax(ys)
    corners[x, y] = hsv_red

    # # Top-right corner
    xs = x_axis[y_max_ids]
    y = y_max
    if np.mean(xs) - x_avr <= 0: # Choose the top (smallest) x, if zero go up
        x = np.amin(xs)
    else: # Choose the bottom (largest) x
        x = np.amax(xs)
    corners[x, y] = hsv_red

    # # Bottom-left corner
    xs = x_axis[y_min_ids]
    y = y_min
    if np.mean(xs) - x_avr < 0: # Choose the top (smallest) x, if zero go down
        x = np.amin(xs)
    else: # Choose the bottom (largest) x
        x = np.amax(xs)
    corners[x, y] = hsv_red

    # # Bottom-right corner
    x = x_max
    ys = y_axis[x_max_ids]
    if np.mean(ys) - y_avr < 0: # Choose the left (smallest) y, if zero go right
        y = np.amin(ys)
    else: # Choose the right (largest) y
        y = np.amax(ys)
    corners[x, y] = hsv_red

    return corners


def find_red_dot(hsv):
    
    # In unit [degrees, %, %] with ranges [[0,360], [0,100], [0,100]]
    lower_red = np.array([  0,   80,  80])*conv_scale_to_bits
    upper_red = np.array([ 10,  100, 100])*conv_scale_to_bits

    mask = cv2.inRange(hsv, lower_red, upper_red)

    dot = np.where(mask == 255)

    if dot[0].shape[0] == 0: # No dot found
        print("No dot found")
        return mask, None, None

    x_mean = np.mean(dot[0])
    y_mean = np.mean(dot[1])

    return mask, x_mean, y_mean



def detect_h(img):
    hsv_origin = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsv_save_image(hsv_origin, '1_hsv')

    hsv = hsv_save_image(crop_on_orange(hsv), "2_crop_orange")

    gray = hsv_save_image(hsv_keep_white_only(hsv), "3_keep_white", is_gray=True)
    gray = hsv_save_image(make_blurry(gray, 3), "4a_make_blurry", is_gray=True)
    gray1 = hsv_save_image(make_gaussian_blurry(gray, 3), "4b_make_gaussian_blurry", is_gray=True)

    hsv = hsv_save_image(find_corners(gray, hsv_origin), "5a_find_corners")
    hsv = hsv_save_image(find_corners(gray1, hsv_origin), "5b_find_corners")
    
    # gray = hsv_save_image(sharpen(gray), "4_sharpen", is_gray=True)
    
    # # ### Preprocessing ###
    # hsv = hsv_save_image(hsv_make_orange_to_green(hsv), '2_orange_to_green')
    # hsv = make_blurry(hsv, 9)
    # hsv = hsv_save_image(make_blurry(hsv, 3), '3_make_blurry')
    # hsv = hsv_save_image(hsv_find_green_mask(hsv), '4_green_mask')
    # # hsv = make_blurry(hsv, 9)
    # # hsv = hsv_save_image(make_blurry(hsv, 9), '5_make_blurry_2')
    # # gray = hsv_save_image(hsv_make_grayscale(hsv), '6_make_grayscale', is_gray=True)
    # gray = hsv
    # # gray = hsv_save_image(make_blurry(gray, 31), "6_make_blurry_3")
    # # gray = hsv_save_image(make_blurry(gray, 21), '7_make_blurry_3', is_gray=True)

    # return gray
    return None



def detect_dot(img):
    hsv_origin = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsv_save_image(hsv_origin, '1_hsv')

    hsv_dot, x_mean, y_mean = find_red_dot(hsv)

    hsv = hsv_save_image(hsv_dot, "2_red_dot", is_gray=True)

    print('x_mean:', x_mean)
    print('y_mean:', y_mean)


def detect_lines(img):
    hsv_origin = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsv_save_image(hsv_origin, '1_hsv')

    # blur = hsv_save_image(make_gaussian_blurry(hsv, 9), "2_blur")
    white = hsv_save_image(hsv_keep_white_only(hsv), "2_white", is_gray=True)

    blur = hsv_save_image(make_blurry(white, 7), "3a_blur", is_gray=True)
    blur = hsv_save_image(make_gaussian_blurry(blur, 21), "3b_blur", is_gray=True)

    # green = hsv_save_image(hsv_find_green_mask_2(blur), "3_green")

    dst = cv2.Canny(blur, 60, 60, None, 3)

    # Copy edges to the images that will display the results in HSV
    bgr = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdst = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    cdstP = np.copy(cdst)


    # Standard Hough Line Transform
    rho = 1
    theta = np.pi / 200
    threshold = 100
    lines = None
    min_theta = 0
    max_theta = np.pi

    lines = cv2.HoughLines(dst, rho, theta, threshold, lines, 0, 0, min_theta, max_theta)
    
    if lines is not None:
        # print("len(lines):", len(lines))
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    
    # Probabilistic Line Transform
    rho = 2                                                     # rho = 2
    theta = (np.pi / 180) * 0.1                                 # theta = np.pi / 1500
    threshold = 150                                             # threshold = 150
    lines = None                                                # lines = None
    min_line_length = 30                                        # min_line_length = 30
    max_line_gap = 25                                           # max_line_gap = 30

    linesP = cv2.HoughLinesP(dst, rho, theta, threshold, lines, min_line_length, max_line_gap)

    if linesP is not None:
        print("len(linesP):", len(linesP))
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)


    # hsv_save_image(cdst, "Standard Hough Line Transform")
    hsv_save_image(cdstP, "Probabilistic Line Transform")



def rgb_color_to_hsv(red, green, blue):
    bgr_color = np.uint8([[[blue,green,red]]])
    hsv_color = cv2.cvtColor(bgr_color,cv2.COLOR_BGR2HSV)
    return hsv_color


def detect_corners(img):
    hsv_origin = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsv_save_image(hsv_origin, '1_hsv')

    hsv_red_color = rgb_color_to_hsv(255, 0, 0)
    

    white = hsv_save_image(hsv_keep_white_only(hsv), "2_white", is_gray=True)

    blur = hsv_save_image(make_gaussian_blurry(white, 5), "3_blur", is_gray=True)

    # Harris Corner Detection
    dst = cv2.cornerHarris(blur,2,3,0.04)

    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    img_corner = hsv_origin.copy()
    img_corner[dst>0.01*dst.max()]=hsv_red_color

    hsv_save_image(img_corner, '4_result')



def detect_sub_pixecl_corners(img):
    hsv_origin = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsv_save_image(hsv_origin, '1_hsv')

    hsv_red_color = rgb_color_to_hsv(255, 0, 0)
    hsv_green_color = rgb_color_to_hsv(0, 255, 0)
    

    white = hsv_save_image(hsv_keep_white_only(hsv), "2a_white", is_gray=True)

    dilated = sharpen(white) 
    hsv_save_image(dilated, "2b_dilated", is_gray=True)

    # white = hsv_save_image(hsv_keep_white_only(hsv), "2_white", is_gray=True)


    second_blur_size = 49

    blur = make_gaussian_blurry(white, 5) 
    double_blur = make_gaussian_blurry(blur, second_blur_size)

    
    ignore_border_size = 3


    blur = hsv_save_image(blur, "3a_blur", is_gray=True)
    double_blur = hsv_save_image(double_blur, "3b_double_blur", is_gray=True)
    

    # Sub-pixel:
    block_size = 4
    dst = cv2.cornerHarris(blur,block_size,3,0.04)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
    dst = np.uint8(dst)

    # hsv_save_image(dst, "dst", is_gray=True)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(blur,np.float32(centroids),(5,5),(-1,-1),criteria)

    # Now draw them
    res = np.hstack((centroids,corners))
    res = np.int0(res)
    img_all_corners = hsv_origin.copy()
    img_best_corners = hsv_origin.copy()
    
    # img_corner[res[:,1],res[:,0]] = hsv_red_color
    # img_corner[res[:,3],res[:,2]] = hsv_green_color

    # print('corner_x')
    # print(corners)
    # print()

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
    img_all_corners[corner_x, corner_y] = hsv_red_color
    

    # Define valid intensity range for the median of a corner
    min_intensity_average = 170
    max_intensity_average = 240
    
    number_of_corners = len(corner_x)
    min_intensity = np.array([min_intensity_average]*number_of_corners)
    max_intensity = np.array([max_intensity_average]*number_of_corners)

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
    img_best_corners[corner_x, corner_y] = img_all_corners[corner_x, corner_y] = hsv_red_color



    # Save images with marked corners
    hsv_save_image(img_all_corners, '4_all_corners')
    hsv_save_image(img_best_corners, '4_best_corners')



    # Use the found corners to estimate position in x, y and yaw
    x_est = None
    y_est = None
    yaw_est = None

    number_of_corners = len(corner_x)

    corners = np.stack((corner_x, corner_y), 1)
    print "Corners:"
    print corners

    if number_of_corners == 4:
        # Define that the corner closest to the top belongs to the top cross-bar

        top_corner_id = np.argmin(corner_x)
        print "Top corner id:", top_corner_id

        top_corner = corners[top_corner_id]
        print "Top corner:", top_corner

        top_corner_stack = np.array([top_corner]*3)
        print "Top corner stack:"
        print top_corner_stack

        rest_corners = np.delete(corners, top_corner_id, 0)
        print "Rest corners:"
        print rest_corners

        dist = np.linalg.norm(rest_corners - top_corner_stack, axis=1)
        print "Distances:", dist

        top_corner_closest_id = np.argmin(dist)
        print "Top corner closest id:", top_corner_closest_id

        top_corner_closest = rest_corners[top_corner_closest_id]
        print "Top corner closest:", top_corner_closest


        # To do:
        # Find the angle between the norm of the line between those points and the x-axis of the image

    if number_of_corners == 2:
        top_corner_id = np.argmin(corner_x)
        print "Top corner id:", top_corner_id


        p_x_0 = 299
        p_y_0 = 120

        p_x_1 = 305
        p_y_1 = 73

        # p_x = 10
        # p_y = 10

        # Calculate spatial gradient
        dx, dy	= cv2.spatialGradient(blur)

        corner_0 = corners[0]
        corner_1 = corners[1]

        grad_0 = np.array([dx[corner_0[0]][corner_0[1]], dy[corner_0[0]][corner_0[1]]])
        grad_1 = np.array([dx[corner_1[0]][corner_1[1]], dy[corner_1[0]][corner_1[1]]])


        grad_0_norm = grad_0/np.linalg.norm(grad_0, ord=1)
        grad_1_norm = grad_1/np.linalg.norm(grad_1, ord=1)

        print(grad_0)
        print(grad_0_norm)
        print
        print(grad_1)
        print(grad_1_norm)

        print "Length of vector:"
        print(np.linalg.norm(corner_1 - corner_0))

        print(dx.shape)

        scale = 5

        gradient_0 = np.int0(corner_0 + grad_0_norm*scale)
        print(gradient_0)



        mid_point = np.int0(corner_0 + 0.5*(corner_1 - corner_0))

        img_grad = hsv_origin.copy()
        img_grad[corner_0[0]][corner_0[1]] = hsv_green_color
        img_grad[gradient_0[0]][gradient_0[1]] = hsv_red_color

        img_grad[mid_point[0]][mid_point[1]] = hsv_red_color

        hsv_save_image(img_grad, "5_gradient")



def detect_with_sift(img):
    hsv_origin = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsv_save_image(hsv_origin, '1_hsv')
    

    white = hsv_save_image(hsv_keep_white_only(hsv), "2_white", is_gray=True)

    blur = make_gaussian_blurry(white, 5) 
    blur = hsv_save_image(blur, "3_blur", is_gray=True)


    # sift = cv2.xfeatures2d.SIFT_create()
    # kp = sift.detect(white,None)

    sift = cv2.xfeatures2d.SIFT_create()
    kp, des = sift.detectAndCompute(blur,None)

    img_keypoints = cv2.drawKeypoints(blur,kp,hsv_origin,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # img_keypoints = cv2.drawKeypoints(white,kp,hsv_origin)
    
    hsv_save_image(img_keypoints, "4_sift_keypoints", is_gray=True)

    print(kp)


def run():
    # filepath = 'dataset/image_h.jpg'
    filepath = 'dataset/image_lines_2.jpg'
    # img = load_image('image_above_clean.jpg')
    # img = load_image('image_no_orange.jpg')
    # img = load_image('image_edge_orange.jpg')
    img = load_image(filepath)

    detect_sub_pixecl_corners(img)

    # detect_with_sift(img)




run()
