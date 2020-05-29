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


# Global variables
global_collect_data = False
received_signal = False
start_time = None

#############
# Callbacks #
#############
global_est_ground_truth = np.zeros(6)
def ground_truth_callback(data):
    global global_est_ground_truth
    global_est_ground_truth = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])

# Estimate callbacks
global_est_ellipse = np.zeros(6)
def estimate_ellipse_callback(data):
    global global_est_ellipse
    global_est_ellipse = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
#
global_est_arrow = np.zeros(6)
def estimate_arrow_callback(data):
    global global_est_arrow
    global_est_arrow = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
#
global_est_corners = np.zeros(6)
def estimate_corners_callback(data):
    global global_est_corners
    global_est_corners = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
#
global_est_dead_reckoning = np.zeros(6)
def estimate_dead_reckoning_callback(data):
    global global_est_dead_reckoning
    global_est_dead_reckoning = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
######################

# Estimate error callbacks
global_est_error_ellipse = np.zeros(6)
def estimate_error_ellipse_callback(data):
    global global_est_error_ellipse
    global_est_error_ellipse = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
#
global_est_error_arrow = np.zeros(6)
def estimate_error_arrow_callback(data):
    global global_est_error_arrow
    global_est_error_arrow = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
#
global_est_error_corners = np.zeros(6)
def estimate_error_corners_callback(data):
    global global_est_error_corners
    global_est_error_corners = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
#
global_est_error_dead_reckoning = np.zeros(6)
def estimate_error_dead_reckoning_callback(data):
    global global_est_error_dead_reckoning
    global_est_error_dead_reckoning = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])
######################


def start_data_collection_callback(data):
    global global_collect_data
    global received_signal
    global start_time
    global_collect_data = True
    received_signal = True
    start_time = rospy.get_time()
    rospy.loginfo("Start data collection")


def stop_data_collection_callback(data):
    global global_collect_data
    global_collect_data = False
    stop_time = rospy.get_time()
    rospy.loginfo("Stop data collection")


def main(test_number):
    global global_state
    global start_time
    rospy.init_node('planner', anonymous=True)

    rospy.Subscriber('/drone_ground_truth', Twist, ground_truth_callback)

    rospy.Subscriber('/estimate/ellipse', Twist, estimate_ellipse_callback)
    rospy.Subscriber('/estimate/arrow', Twist, estimate_arrow_callback)
    rospy.Subscriber('/estimate/corners', Twist, estimate_corners_callback)
    rospy.Subscriber('/estimate/dead_reckoning', Twist, estimate_dead_reckoning_callback)

    rospy.Subscriber('/estimate_error/ellipse', Twist, estimate_error_ellipse_callback)
    rospy.Subscriber('/estimate_error/arrow', Twist, estimate_error_arrow_callback)
    rospy.Subscriber('/estimate_error/corners', Twist, estimate_error_corners_callback)
    rospy.Subscriber('/estimate_error/dead_reckoning', Twist, estimate_error_dead_reckoning_callback)

    rospy.Subscriber('/initiate_mission', Empty, start_data_collection_callback)
    rospy.Subscriber('/take_still_photo', Empty, stop_data_collection_callback)

    pub_heartbeat = rospy.Publisher("/heartbeat_data_collection", Empty, queue_size=10)
    heartbeat_msg = Empty()

    rospy.loginfo("Starting data collection...")
    time.sleep(1)
    rospy.loginfo("... Ready!")

    prev_est_ground_truth = np.zeros(6)
    prev_est_ellipse = np.zeros(6)
    prev_est_arrow = np.zeros(6)
    prev_est_corners = np.zeros(6)
    prev_est_dead_reckoning = np.zeros(6) 

    data_array = []
        
    rate = rospy.Rate(20) # Hz
    while not rospy.is_shutdown():
        curr_time = rospy.get_time()

        if global_collect_data:
            curr_time = rospy.get_time() - start_time

            data_point = np.concatenate((
                # Time
                np.array([curr_time]),
                # Ground truth
                global_est_ground_truth,
                # Estimate
                global_est_ellipse,
                global_est_arrow,
                global_est_corners,
                global_est_dead_reckoning,
                # Estimate errors
                global_est_error_ellipse,
                global_est_error_arrow,
                global_est_error_corners,
                global_est_error_dead_reckoning,
                )
            )
            print len(data_array)

            data_array.append(data_point)

        if not global_collect_data and received_signal:
            break

        pub_heartbeat.publish(heartbeat_msg)

        rate.sleep()
    folder = './catkin_ws/src/uav_vision/data_storage/'


    filename = 'test_'+str(test_number)+'.npy'
    path = folder+filename
    np.save(path, np.array(data_array))
    
    
if __name__ == '__main__':
    test_number = 13

    main(test_number)