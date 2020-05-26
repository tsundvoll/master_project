#!/usr/bin/env python

"""
When the 'Triangle'-button on controller is pressed:
The missions start, so collect data

When the 'Square'-button on the controller is pressed:
Stop collecting data and save plot
"""

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool

import time
import matplotlib.pyplot as plt




#############
# Callbacks #
#############
estimate_ground_truth = None
estimate_ground_truth_array = []
def ground_truth_callback(data):
    global estimate_ground_truth
    estimate_ground_truth = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
    curr_time = rospy.get_time() - start_time

    data_point = np.array([curr_time, estimate_ground_truth])
    
    estimate_ground_truth_array.append(data_point)


estimate_ellipse = None
def estimate_ellipse_callback(data):
    global estimate_ellipse
    estimate_ellipse = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])

estimate_arrow = None
def estimate_arrow_callback(data):
    global estimate_arrow
    estimate_arrow = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])

estimate_corners = None
def estimate_corners_callback(data):
    global estimate_corners
    estimate_corners = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])

estimate_dead_reckoning = None
def estimate_dead_reckoning_callback(data):
    global estimate_dead_reckoning
    estimate_dead_reckoning = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])

global_collect_data = False

def start_data_collection_callback(data):
    global_collect_data = True
    start_time = rospy.get_time()

def stop_data_collection_callback(data):
    global global_collect_data
    global_collect_data = False
    stop_time = rospy.get_time()



def main():
    global global_state
    global start_time
    rospy.init_node('planner', anonymous=True)

    rospy.Subscriber('/drone_pose', Twist, ground_truth_callback)
    rospy.Subscriber('/estimate/ellipse', Twist, estimate_ellipse_callback)
    rospy.Subscriber('/estimate/arrow', Twist, estimate_arrow_callback)
    rospy.Subscriber('/estimate/corners', Twist, estimate_corners_callback)
    rospy.Subscriber('/estimate/dead_reckoning', Twist, estimate_dead_reckoning_callback)

    # rospy.Subscriber('/initiate_mission', Empty, start_data_collection_callback)
    rospy.Subscriber('/take_still_photo', Empty, stop_data_collection_callback)

    rospy.loginfo("Starting data collection")

    time.sleep(1)
        
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        curr_time = rospy.get_time()

        if not global_collect_data:
            break
        rate.sleep()


    print estimate_ground_truth_array
    
    
if __name__ == '__main__':
    main()