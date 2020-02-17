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
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from keras.models import load_model

import config as cfg


bridge = CvBridge()

nn_model = None

global_gt_pose = None
global_image = None
save_images = False
global_signal = False

IMG_WIDTH = 640
IMG_HEIGHT = 360

x_mean = np.array([ 
    6.90849561e-01,
    5.27306617e-06,
    6.92069939e-01,
    -4.43393840e+02,
    -2.51004351e+02,
    1.12344295e+05,
    -2.92006836e+01,
    -2.90295541e+01
])
x_std = np.array([
    1.47666745e-01,
    6.88734929e-03,
    1.48031010e-01,
    2.24058127e+02,
    1.31239825e+02,
    7.37506773e+04,
    1.52626994e+01,
    1.52033420e+01
])
y_mean = np.array([-0.15933413, 0.03610403, 5.56289522])
y_std = np.array([1.37827808, 2.27332637, 1.74280007])



def image_callback(data):
    global global_image

    try:
        global_image = bridge.imgmsg_to_cv2(data, 'bgr8') # {'bgr8' or 'rgb8}
    except CvBridgeError as e:
        rospy.loginfo(e)



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
    inv_S3 = np.linalg.inv(S3)

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

    A = ellipse[0]
    B = ellipse[1]
    C = ellipse[2]
    D = ellipse[3]
    E = ellipse[4]
    F = ellipse[5]

    # print(A)

    inner_square = math.sqrt( (A-C)**2 + B**2)
    outside = 1.0 / (B**2 - 4*A*C)
    a = outside * math.sqrt(2*(A*E**2 + C*D**2 - B*D*E + (B**2 - 4*A*C)*F) * ( (A+C) + inner_square))
    b = outside * math.sqrt(2*(A*E**2 + C*D**2 - B*D*E + (B**2 - 4*A*C)*F) * ( (A+C) - inner_square))

    # print(a)
    ellipse_and_a_b = np.array([A,B,C,D,E,F,a,b])

    return ellipse_and_a_b


def main():
    rospy.init_node('position_estimator', anonymous=True)

    rospy.Subscriber('/ardrone/bottom/image_raw', Image, image_callback)

    rospy.loginfo("Initializing model...")

    model_path = '/home/thomas/master_project/catkin_ws/src/uav_vision/scripts/nn_models/nn_model.h5'
    weights_path = '/home/thomas/master_project/catkin_ws/src/uav_vision/scripts/nn_models/weights.h5'

    nn_model = load_model(model_path)
    nn_model.load_weights(weights_path)

    # nn_model.summary()

    pub_heartbeat = rospy.Publisher("/heartbeat", Empty, queue_size=10)
    heartbeat_msg = Empty()

    # time.sleep(1)

    rospy.loginfo("Starting estimating position")


    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():

        if global_image is not None:
            x_test = get_ellipse_parameters(global_image)
            # print(x_test)

            x_test_norm = (x_test - x_mean) / x_std
            
            # x_test_x_y = x_test_in_use[:,:6]
            # x_test_z = x_test_in_use[:,6:]

            # print(x_test_norm)



            x_test_norm_x_y = [x_test_norm[:6]]
            x_test_norm_z = [x_test_norm[6:]]
            # print(x_test_norm_x_y)
            # print(x_test_norm_z)

            # input_nn = [x_test_norm_x_y, x_test_norm_z]

            pt_pred_raw = nn_model.predict([x_test_norm_x_y, x_test_norm_z])

            pt_pred_concat = np.array([pt_pred_raw[0][0], pt_pred_raw[1][0], pt_pred_raw[2][0]])[:,0]

            point_prediction = pt_pred_concat*y_std + y_mean

            print(np.round(point_prediction,4))
            # print("x:", point_prediction[0], "y:", point_prediction[1],"z:", point_prediction[2])


            pub_heartbeat.publish(heartbeat_msg)

        rate.sleep()
    
    
if __name__ == '__main__':
    main()