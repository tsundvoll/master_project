#!/usr/bin/env python

import rospy
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu

import numpy as np


global_imu = None
def imu_callback(data):
    global global_imu
    global_imu = data


global_velocities = None
def navdata_callback(data):
    global global_velocities
    global_velocities = np.array([data.vx, data.vy, data.vz])


def filter_acceleration(acceleration, acceleration_history, median_filter_size, average_filter_size):
    """
        Filters the acceleration with a sliding window median and average filter.
    """

    acceleration_history = np.concatenate((acceleration_history[1:], [acceleration]))

    strides = np.array(
        [acceleration_history[i:median_filter_size+i] for i in range(average_filter_size)]
    )

    median_filtered = np.median(strides, axis = 1)
    average_filtered = np.average(median_filtered[-average_filter_size:], axis=0)

    return average_filtered, acceleration_history


def main():
    rospy.init_node('dead_reckoning', anonymous=True)

    rospy.Subscriber('/ardrone/imu', Imu, imu_callback)
    # rospy.Subscriber('/ardrone/navdata', Navdata, navdata_callback)

    

    rospy.loginfo("Starting Dead Reckoning module")

    count = 0
    N_CALIBRATION_STEPS = 1000
    calibration_sum = np.zeros(3)
    calibration = None

    acc_min = -0.2
    acc_max = 0.2

    pos_prev = np.zeros(3)
    vel_prev = np.zeros(3)

    # Set up filter
    median_filter_size = 1
    average_filter_size = 10

    acceleration_history_size = median_filter_size + average_filter_size - 1
    acceleration_history = np.zeros((acceleration_history_size,3))
    
    rate = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():
        if global_imu is not None:
            imu = global_imu
            if count == 0:
                start_time = rospy.get_time()
            elif count < N_CALIBRATION_STEPS:
                acc = np.array([imu.linear_acceleration.x,
                                imu.linear_acceleration.y,
                                imu.linear_acceleration.z])
                calibration_sum += acc
            elif count == N_CALIBRATION_STEPS:
                calibration = calibration_sum / float(N_CALIBRATION_STEPS)
                print calibration

                end_time = rospy.get_time()
                duration = end_time - start_time
                print "Duration:", duration
                prev_time = end_time
            else: # Perform dead reckoning
                curr_time = rospy.get_time()
                duration = curr_time - prev_time
                prev_time = curr_time
                acc = np.array([imu.linear_acceleration.x,
                                imu.linear_acceleration.y,
                                imu.linear_acceleration.z]) - calibration
                small_values_filter = np.logical_and(np.less(acc, acc_max), np.greater(acc, acc_min))
                acc[small_values_filter] = 0.0

                acc_filtered, acceleration_history = filter_acceleration(acc, acceleration_history, median_filter_size, average_filter_size)

                vel_curr = vel_prev + duration*acc_filtered
                pos_curr = pos_prev + duration*vel_prev + 0.5*acc_filtered*duration**2
                vel_prev = vel_curr
                pos_prev = pos_curr

                print ""
                # print "Acceleration after filtering:", acc_filtered
                print "Position :", pos_curr



            
            
            
            count += 1
        rate.sleep()
    
    
if __name__ == '__main__':
    main()