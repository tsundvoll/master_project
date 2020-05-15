#!/usr/bin/env python

import rospy
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

import numpy as np
from scipy.spatial.transform import Rotation as R

ONE_G = 9.80665

global_velocities = None
global_acceleration = None
global_yaw = None

global_last_estimate = None

def navdata_callback(data):
    global global_velocities
    global global_acceleration
    global global_yaw
    global_velocities = np.array([data.vx, data.vy, data.vz])
    global_acceleration = np.array([data.ax*ONE_G*1000, data.ay*ONE_G*1000, data.az*ONE_G*1000])
    yaw = data.rotZ-90 # Rotation about the Z axis, measured in degrees

    if yaw < -180:
        global_yaw = 360 + yaw
    else:
        global_yaw = yaw


def estimate_callback(data):
    global global_last_estimate

    position = np.array([data.linear.x, data.linear.y, data.linear.z])*1000

    if not np.array_equal(position, np.zeros(3)):
        # Only save last estimate, if there is an estimate available
        global_last_estimate = position


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
    global global_last_estimate
    rospy.init_node('dead_reckoning', anonymous=True)

    rospy.Subscriber('/ardrone/navdata', Navdata, navdata_callback)
    rospy.Subscriber('/filtered_estimate', Twist, estimate_callback)
    pub_dead_reckoning = rospy.Publisher("/estimate/dead_reckoning", Twist, queue_size=10)


    rospy.loginfo("Starting Dead Reckoning module")

    count = 0
    N_CALIBRATION_STEPS = 1000
    calibration_sum_vel = np.zeros(3)
    calibration_sum_acc = np.zeros(3)
    calibration_vel = 0.0
    calibration_acc = 0.0

    # Avoid small fluxtuations by removing small values
    vel_min = -3.0
    vel_max = 3.0

    acc_min = -500
    acc_max = 500

    pos_prev = np.zeros(3)
    vel_prev = np.zeros(3)
    yaw_prev = 0.0

    # Set up filter
    median_filter_size = 1
    average_filter_size = 3

    history_size = median_filter_size + average_filter_size - 1
    vel_history = np.zeros((history_size,3))
    acc_history = np.zeros((history_size,3))

    dead_reckoning_msg = Twist()
    
    rate = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():
        if global_velocities is not None:
            new_vel = global_velocities
            new_acc = global_acceleration
            new_yaw = global_yaw
            if count == 0:
                start_time = rospy.get_time()
                prev_time = start_time
            elif count < N_CALIBRATION_STEPS:
                calibration_sum_vel += new_vel
                calibration_sum_acc += new_acc
            elif count == N_CALIBRATION_STEPS:
                calibration_vel = calibration_sum_vel / float(N_CALIBRATION_STEPS)
                calibration_acc = calibration_sum_acc / float(N_CALIBRATION_STEPS)

                end_time = rospy.get_time()
                duration = end_time - start_time
                rospy.loginfo("Calibration ready. Duration: " + str(duration))
                prev_time = end_time
            else: # Perform dead reckoning

                # Get last estimate if available
                if global_last_estimate is not None:
                    pos_curr = global_last_estimate
                    global_last_estimate = None
                else:
                    curr_time = rospy.get_time()
                    duration = curr_time - prev_time
                    prev_time = curr_time

                    vel = new_vel - calibration_vel
                    acc = new_acc - calibration_acc
                    delta_yaw = new_yaw - yaw_prev
                    yaw_prev = new_yaw

                    # vel, vel_history = filter_measurement(vel, vel_history, median_filter_size, average_filter_size)
                    # acc, acc_history = filter_measurement(acc, acc_history, median_filter_size, average_filter_size)
                    
                    small_values_filter_val = np.logical_and(np.less(vel, vel_max), np.greater(vel, vel_min))
                    small_values_filter_acc = np.logical_and(np.less(acc, acc_max), np.greater(acc, acc_min))
                    vel[small_values_filter_val] = 0.0
                    acc[small_values_filter_acc] = 0.0

                    pos_curr = pos_prev + duration*vel + 0.5*acc*duration**2

                    rotation = R.from_euler('z', -np.radians(delta_yaw))
                    pos_curr = rotation.apply(pos_curr)

                pos_prev = pos_curr

                dead_reckoning_msg.linear.x = pos_curr[0] / 1000
                dead_reckoning_msg.linear.y = pos_curr[1] / 1000
                dead_reckoning_msg.linear.z = pos_curr[2] / 1000
                dead_reckoning_msg.angular.z = new_yaw
                pub_dead_reckoning.publish(dead_reckoning_msg)

            count += 1
        rate.sleep()
    
    
if __name__ == '__main__':
    main()