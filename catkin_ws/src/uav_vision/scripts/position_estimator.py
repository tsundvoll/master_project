#!/usr/bin/env python
import rospy
import numpy as np
import json
import math
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from keras.models import load_model

import config as cfg


# For converting between units
conv_scale_to_bits = np.array([0.5, 2.55, 2.55]) # unit bits with ranges [[0,180], [0,255], [0,255]]

bridge = CvBridge()

nn_model = None

global_gt_pose = None
global_image = None
save_images = True
is_at_lab = True

filename = '/home/thomas/master_project/catkin_ws/src/uav_vision/scripts/real_image.png'
global_image = cv2.imread(filename)

IMG_WIDTH = 640
IMG_HEIGHT = 360

# x_mean = np.array([ 
#     6.90849561e-01,
#     5.27306617e-06,
#     6.92069939e-01,
#     -4.43393840e+02,
#     -2.51004351e+02,
#     1.12344295e+05,
#     -2.92006836e+01,
#     -2.90295541e+01
# ])
# x_std = np.array([
#     1.47666745e-01,
#     6.88734929e-03,
#     1.48031010e-01,
#     2.24058127e+02,
#     1.31239825e+02,
#     7.37506773e+04,
#     1.52626994e+01,
#     1.52033420e+01
# ])

x_mean = np.array([320.46095096, 183.92657922, -21.81760804, -21.66377995])
x_std = np.array([145.8736188, 90.35919202, 16.05925017, 16.01824747])

y_mean = np.array([-0.21072352, 0.03519739, 8.08887296])
y_std = np.array([2.15498895, 3.41088765, 2.8263245])


def gt_callback(data):
    global global_gt_pose
    global_gt_pose = data.pose.pose


def image_callback(data):
    global global_image

    

    try:
        global_image = bridge.imgmsg_to_cv2(data, 'bgr8') # {'bgr8' or 'rgb8}
    except CvBridgeError as e:
        rospy.loginfo(e)


def get_relative_position(gt_pose):
    # Transform ground truth in body frame wrt. world frame to body frame wrt. landing platform

    ##########
    # 0 -> 2 #
    ##########

    # Position
    p_x = gt_pose.position.x
    p_y = gt_pose.position.y
    p_z = gt_pose.position.z

    # Translation of the world frame to body frame wrt. the world frame
    d_0_2 = np.array([p_x, p_y, p_z])

    # Orientation
    q_x = gt_pose.orientation.x
    q_y = gt_pose.orientation.y
    q_z = gt_pose.orientation.z
    q_w = gt_pose.orientation.w

    # Rotation of the body frame wrt. the world frame
    r_0_2 = R.from_quat([q_x, q_y, q_z, q_w])
    r_2_0 = r_0_2.inv()
    

    ##########
    # 0 -> 1 #
    ##########
    
    # Translation of the world frame to landing frame wrt. the world frame
    offset_x = 1.0
    offset_y = 0.0
    offset_z = 0.54
    d_0_1 = np.array([offset_x, offset_y, offset_z])

    # Rotation of the world frame to landing frame wrt. the world frame
    # r_0_1 = np.identity(3) # No rotation, only translation
    r_0_1 = np.identity(3) # np.linalg.inv(r_0_1)


    ##########
    # 2 -> 1 #
    ##########
    # Transformation of the body frame to landing frame wrt. the body frame
    
    # Translation of the landing frame to bdy frame wrt. the landing frame
    d_1_2 = d_0_2 - d_0_1

    # Rotation of the body frame to landing frame wrt. the body frame
    r_2_1 = r_2_0

    yaw = r_2_1.as_euler('xyz')[2]
    r_2_1_yaw = R.from_euler('z', yaw)

    # Translation of the body frame to landing frame wrt. the body frame
    # Only yaw rotation is considered
    d_2_1 = -r_2_1_yaw.apply(d_1_2)

    # Translation of the landing frame to body frame wrt. the body frame
    # This is more intuitive for the controller
    d_2_1_inv = -d_2_1

    return d_2_1_inv


# Computer Vision
def make_blurry(image, blurr):
    return cv2.medianBlur(image, blurr)


def hsv_make_orange_to_green(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    lower_orange = np.array([8,128,64])
    upper_orange = np.array([29,255,255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # change the orange to green
    imask = mask>0
    orange_to_green = hsv.copy()
    orange_to_green[imask] = hsv_green

    return orange_to_green


def hsv_make_orange_to_green_at_lab(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)


    # In degrees, %, %, ranges [[0,360], [0,100], [0,100]]
    lower_orange = np.array([ 5,  10,  55])*conv_scale_to_bits
    upper_orange = np.array([ 70, 100, 100])*conv_scale_to_bits

    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # change the orange to green
    imask = mask>0
    orange_to_green = hsv.copy()
    orange_to_green[imask] = hsv_green

    return orange_to_green


def hsv_find_green_mask(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    lower_green_h = 45
    lower_green_s = 50 * 0.01*255
    lower_green_v = 25 * 0.01*255

    upper_green_h = 75
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


def hsv_find_green_mask_at_lab(hsv):
    bgr_green = np.uint8([[[30,90,30]]])
    hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)

    # In degrees, %, %, ranges [[0,360], [0,100], [0,100]]
    lower_green = np.array([ 75,  35,  25])*conv_scale_to_bits
    upper_green = np.array([200, 100, 100])*conv_scale_to_bits

    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # keep only the green
    imask = green_mask>0
    green = np.zeros_like(hsv, np.uint8)
    green[imask] = hsv_green
  
    return green


def save_image(image, label='image', is_gray=False):
    if save_images:
        folder = '/home/thomas/Desktop/image_processing/'
        if is_gray:
            cv2.imwrite(folder+label+".png", image)
        else:
            cv2.imwrite(folder+label+".png", cv2.cvtColor(image, cv2.COLOR_HSV2BGR))

    return image


def flood_fill(img):
    h,w,chn = img.shape
    seed = (w/2,h/2)
    seed = (0,0)

    mask = np.zeros((h+2,w+2),np.uint8) # Adding a padding of 1

    floodflags = 8
    # floodflags |= cv2.FLOODFILL_FIXED_RANGE
    floodflags |= cv2.FLOODFILL_MASK_ONLY
    floodflags |= (255 << 8)

    num,img,mask,rect = cv2.floodFill(img, mask, seed, (255,0,0), (10,)*3, (10,)*3, floodflags)
    mask = mask[1:h+1,1:w+1] # Removing the padding

    return mask


def preprocessing(raw_img):

    if is_at_lab:
        hsv = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV)
        hsv = save_image(hsv, 'LAB_1_hsv')
        
        hsv = save_image(hsv_make_orange_to_green_at_lab(hsv), 'LAB_2_orange_to_green')
        hsv = save_image(make_blurry(hsv, 9), 'LAB_3_make_blurry')
        hsv = save_image(hsv_find_green_mask_at_lab(hsv), 'LAB_4_green_mask')
        hsv = save_image(make_blurry(hsv, 9), 'LAB_5_make_blurry')
        gray = save_image(flood_fill(hsv), 'LAB_6_flood_fill', is_gray=True)

    else:
        hsv = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV)
        hsv = save_image(hsv, '1_hsv')
        
        hsv = save_image(hsv_make_orange_to_green(hsv), '2_orange_to_green')
        hsv = save_image(make_blurry(hsv, 3), '3_make_blurry')
        hsv = save_image(hsv_find_green_mask(hsv), '4_green_mask')
        hsv = save_image(make_blurry(hsv, 3), '5_make_blurry')
        gray = save_image(flood_fill(hsv), '6_flood_fill', is_gray=True)

    return gray


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


def get_ellipse_parameters(raw_img):
    gray = preprocessing(raw_img)

    edges = cv2.Canny(gray,100,200)
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


    x_0 = (2.0*C*D - B*E) / (B*B - 4.0*A*C) 
    y_0 = (2.0*A*E - B*D) / (B*B - 4.0*A*C)

    # print(a)
    # ellipse_and_a_b = np.array([A,B,C,D,E,F,a,b])
    ellipse_and_a_b = np.array([x_0,y_0,a,b])

    return ellipse_and_a_b


def main():
    rospy.init_node('position_estimator', anonymous=True)

    rospy.Subscriber('/ardrone/bottom/image_raw', Image, image_callback)
    rospy.Subscriber('/ground_truth/state', Odometry, gt_callback)

    rospy.loginfo("Initializing model...")

    model_path = '/home/thomas/master_project/catkin_ws/src/uav_vision/scripts/nn_models/nn_model_4_1.h5'
    weights_path = '/home/thomas/master_project/catkin_ws/src/uav_vision/scripts/nn_models/best_model_run_4_1.h5'

    nn_model = load_model(model_path)
    nn_model.load_weights(weights_path)

    # nn_model.summary()

    pub_heartbeat = rospy.Publisher("/heartbeat", Empty, queue_size=10)
    pub_estimate = rospy.Publisher("/drone_estimate", Point, queue_size=10)
    pub_estimate_filtered = rospy.Publisher("/drone_estimate_filtered", Point, queue_size=10)
    pub_gt = rospy.Publisher("/drone_gt", Point, queue_size=10)
    
    heartbeat_msg = Empty()
    estimate_msg = Point()
    estimate_filtered_msg = Point()
    gt_msg = Point()

    # time.sleep(1)

    rospy.loginfo("Starting estimating position")


    median_filter_size = 3
    average_filter_size = 10


    initial_estimate = 0.1
    prediction_history_size = median_filter_size + average_filter_size - 1
    # prediction_history = np.array([initial_estimate]*prediction_history_size)
    prediction_history = np.zeros((prediction_history_size,3))
    # prediction_history = np.arange(5, dtype=np.uint8)

    rospy.loginfo("Prediction history size: " + str(prediction_history_size))


    rate = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        if global_gt_pose is not None:
            relative_position = get_relative_position(global_gt_pose)

            gt_msg.x = relative_position[0]
            gt_msg.y = relative_position[1]
            gt_msg.z = relative_position[2]
            pub_gt.publish(gt_msg)


        if global_image is not None:
            x_test = get_ellipse_parameters(global_image)
            # print(x_test)

            if x_test is None:
                print("No estimate available")
            else:

                x_test_norm = (x_test - x_mean) / x_std
                
                # x_test_x_y = x_test_in_use[:,:6]
                # x_test_z = x_test_in_use[:,6:]

                # print(x_test_norm)


                x_test_norm_x_y = [x_test_norm]
                x_test_norm_z = [x_test_norm[2:]]

                # x_test_norm_x_y = [x_test_norm[:6]]
                # x_test_norm_z = [x_test_norm[6:]]
                # print(x_test_norm_x_y)
                # print(x_test_norm_z)

                # input_nn = [x_test_norm_x_y, x_test_norm_z]

                pt_pred_raw = nn_model.predict([x_test_norm_x_y, x_test_norm_z])

                pt_pred_concat = np.array([pt_pred_raw[0][0], pt_pred_raw[1][0], pt_pred_raw[2][0]])[:,0]

                point_prediction = pt_pred_concat*y_std + y_mean

                estimate_msg.x = point_prediction[0]
                estimate_msg.y = point_prediction[1]
                estimate_msg.z = point_prediction[2]

                pub_estimate.publish(estimate_msg)

                # Perform filtering
                prediction_history = np.concatenate((prediction_history[1:], [point_prediction]))
                # prediction_history = np.concatenate((prediction_history[1:], [2]))
                # median_filtered = np.array([np.median(prediction_history[0:3]),
                #                             np.median(prediction_history[1:4]),
                #                             np.median(prediction_history[2:5])
                # ])
                # average_filtered = np.average(median_filtered)

                # estimate_filtered_msg.x = average_filtered

                # Perform filtering (scalable)
                strides = np.array(
                    [prediction_history[i:median_filter_size+i] for i in range(average_filter_size)]
                )


                median_filtered = np.median(strides, axis = 1)
                average_filtered = np.average(median_filtered[-average_filter_size:], axis=0)

                estimate_filtered_msg.x = average_filtered[0]
                estimate_filtered_msg.y = average_filtered[1]
                estimate_filtered_msg.z = average_filtered[2]

                # Publish filtered estimate
                pub_estimate_filtered.publish(estimate_filtered_msg)

                print("x:", average_filtered[0], "y:", average_filtered[1], "z:", average_filtered[2])


            pub_heartbeat.publish(heartbeat_msg)

        rate.sleep()
    
    
if __name__ == '__main__':
    main()