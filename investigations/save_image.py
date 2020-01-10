#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
import numpy as np
from scipy.misc import imsave
import os

print(os.getcwd())
os.chdir('/home/thomas/master_project/investigations')
print(os.getcwd())


previous_image = None


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

    return True


def run():   
    
    rospy.init_node('save_image', anonymous=True)

    rospy.Subscriber('ardrone/front/image_raw', Image, video_callback)

    rospy.loginfo("Starting saving of images")

    rate = rospy.Rate(0.5) # Hz
    while not rospy.is_shutdown():
        # Do work
        if save_image():
            break
        
        rate.sleep()
    
    

if __name__ == '__main__':
    run()
