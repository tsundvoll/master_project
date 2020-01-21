#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np
from scipy.misc import imsave
import os
from scipy.spatial.transform import Rotation as R


print(os.getcwd())
os.chdir('/home/thomas/master_project/investigations')
print(os.getcwd())


previous_image = None
prev_rel_position = [None, None, None]

PLATFORM_OFFSET_X = 2
PLATFORM_OFFSET_Y = 2
PLATFORM_OFFSET_Z = 0

def position_callback(data):
    global prev_rel_position

    # prev_position[0] = data.pose.pose.position.x - PLATFORM_OFFSET_X
    # prev_position[1] = data.pose.pose.position.y - PLATFORM_OFFSET_Y
    # prev_position[2] = data.pose.pose.position.z - PLATFORM_OFFSET_Z

    # global prev_ground_truth

    # Transform ground truth in world frame to body frame

    # Position
    p_x = data.pose.pose.position.x
    p_y = data.pose.pose.position.y
    p_z = data.pose.pose.position.z

    # Orientation
    q_x = data.pose.pose.orientation.x
    q_y = data.pose.pose.orientation.y
    q_z = data.pose.pose.orientation.z
    q_w = data.pose.pose.orientation.w

    r = R.from_quat([q_x, q_y, q_z, q_w])
    r_transpose = r.inv()
    
    offset_x = PLATFORM_OFFSET_X
    offset_y = PLATFORM_OFFSET_Y
    offset_z = PLATFORM_OFFSET_Z

    p_0 = np.array([offset_x, offset_y, offset_z])      # The helipad's position in world coordinates
    d_0_1 = np.array([p_x, p_y, p_z])      # Translation from origo in the world frame to origo in the body frame

    diff = p_0 - d_0_1

    p_1 = r_transpose.apply(diff)           # The helipad's position in body coordinates

    prev_rel_position = p_1
    # rospy.loginfo("Ground truth data " + str(prev_ground_truth))


def video_callback(data):
    global previous_image
    previous_image = data


def save_image():
    if previous_image == None:
        return False

    this_image = previous_image

    rospy.loginfo("Saving image")

    height = this_image.height # height = 360
    width = this_image.width # width = 640
    depth = 3
    num_elements = height*width*3

    image_array = np.zeros((num_elements))

    for i in range(num_elements):
        image_array[i] = float(ord(this_image.data[i]))
    image_array.resize(height,width,depth)


    filename = 'image'
    filetype = '.jpg' # or 'png'
    filepath = 'dataset/' + filename + filetype
    imsave(filepath, image_array)

    f = open("position.txt", "w")
    x_pos = str(prev_rel_position[0])
    y_pos = str(prev_rel_position[1])
    z_pos = str(prev_rel_position[2])
    f.write('x_pos: ' + x_pos + ' | y_pos: ' + y_pos + ' | z_pos: ' + z_pos)
    f.close()

    return True


def run():   
    
    rospy.init_node('save_image', anonymous=True)

    rospy.Subscriber('ardrone/bottom/image_raw', Image, video_callback)
    rospy.Subscriber('ground_truth/state', Odometry, position_callback)

    rospy.loginfo("Starting saving of images")

    rate = rospy.Rate(0.5) # Hz
    while not rospy.is_shutdown():
        # Do work
        if save_image():
            break
        
        rate.sleep()
    
    

if __name__ == '__main__':
    run()
