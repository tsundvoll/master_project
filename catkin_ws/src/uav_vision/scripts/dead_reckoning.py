#!/usr/bin/env python

import rospy
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


import numpy as np

ONE_G = 9.80665

global_imu = None
def imu_callback(data):
    global global_imu
    global_imu = data


global_velocities = None
global_acceleration = None
def navdata_callback(data):
    global global_velocities
    global global_acceleration
    global_velocities = np.array([data.vx, data.vy, data.vz])
    global_acceleration = np.array([data.ax*ONE_G*1000, data.ay*ONE_G*1000, data.az*ONE_G*1000])


def filter_measurement(measurement, measurement_history, median_filter_size, average_filter_size):
    """
        Filters the measurement with a sliding window median and average filter.
    """

    measurement_history = np.concatenate((measurement_history[1:], [measurement]))

    strides = np.array(
        [measurement_history[i:median_filter_size+i] for i in range(average_filter_size)]
    )

    median_filtered = np.median(strides, axis = 1)
    average_filtered = np.average(median_filtered[-average_filter_size:], axis=0)

    return average_filtered, measurement_history


def main():
    rospy.init_node('dead_reckoning', anonymous=True)

    # rospy.Subscriber('/ardrone/imu', Imu, imu_callback)
    rospy.Subscriber('/ardrone/navdata', Navdata, navdata_callback)
    pub_dead_reckoning = rospy.Publisher("/estimate/dead_reckoning", Twist, queue_size=10)


    rospy.loginfo("Starting Dead Reckoning module")

    count = 0
    N_CALIBRATION_STEPS = 100
    calibration_sum_vel = np.zeros(3)
    calibration_sum_acc = np.zeros(3)
    calibration_vel = None
    calibration_acc = None

    vel_min = -3.0
    vel_max = 3.0

    acc_min = -500
    acc_max = 500

    pos_prev = np.zeros(3)
    vel_prev = np.zeros(3)

    # Set up filter
    median_filter_size = 1
    average_filter_size = 3

    history_size = median_filter_size + average_filter_size - 1
    vel_history = np.zeros((history_size,3))
    acc_history = np.zeros((history_size,3))

    dead_reckoning_msg = Twist()
    
    rate = rospy.Rate(20) # Hz
    while not rospy.is_shutdown():
        if global_velocities is not None:
            new_vel = global_velocities
            new_acc = global_acceleration
            if count == 0:
                start_time = rospy.get_time()
            elif count < N_CALIBRATION_STEPS:
                calibration_sum_vel += new_vel
                calibration_sum_acc += new_acc
            elif count == N_CALIBRATION_STEPS:
                calibration_vel = calibration_sum_vel / float(N_CALIBRATION_STEPS)
                calibration_acc = calibration_sum_acc / float(N_CALIBRATION_STEPS)

                end_time = rospy.get_time()
                duration = end_time - start_time
                print "Calibration ready. Duration:", duration
                prev_time = end_time
            else: # Perform dead reckoning
                curr_time = rospy.get_time()
                duration = curr_time - prev_time
                prev_time = curr_time

                vel = new_vel - calibration_vel
                acc = new_acc - calibration_acc

                # vel, vel_history = filter_measurement(vel, vel_history, median_filter_size, average_filter_size)
                # acc, acc_history = filter_measurement(acc, acc_history, median_filter_size, average_filter_size)
                


                small_values_filter_val = np.logical_and(np.less(vel, vel_max), np.greater(vel, vel_min))
                small_values_filter_acc = np.logical_and(np.less(acc, acc_max), np.greater(acc, acc_min))
                vel[small_values_filter_val] = 0.0
                acc[small_values_filter_acc] = 0.0

                # print "Velocity :", vel
                # print "Acceleration :", acc

                pos_curr = pos_prev + duration*vel + 0.5*acc*duration**2
                pos_prev = pos_curr

                # print ""
                # print "Velocity :", vel
                # print "Velocity after filtering:", vel_filtered
                # print "Position :", pos_curr

                dead_reckoning_msg.linear.x = pos_curr[0] / 1000
                dead_reckoning_msg.linear.y = pos_curr[1] / 1000
                dead_reckoning_msg.linear.z = pos_curr[2] / 1000
                pub_dead_reckoning.publish(dead_reckoning_msg)

            
            count += 1
        rate.sleep()
    
    
if __name__ == '__main__':
    main()