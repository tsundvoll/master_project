#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

import time
import math

import config as cfg



relative_position = None
def prev_gt_callback(data):
    global relative_position
    gt_pose = data.pose.pose

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

    # Translation of the body frame to landing frame wrt. the body frame
    d_2_1 = -r_2_1.apply(d_1_2)


    # Translation of the landing frame to body frame wrt. the body frame
    # This is more intuitive for the controller
    d_2_1_inv = -d_2_1

    relative_position = np.concatenate((d_2_1_inv, r_2_1.as_euler('xyz')))
    # rospy.loginfo("x: " + str(relative_position[0]))
    # rospy.loginfo("y: " + str(relative_position[1]))
    # rospy.loginfo("z: " + str(relative_position[2]))


# Setup for the PID controller
error_prev = np.array([0.0]*6)
error_integral = np.array([0.0]*6)
error_derivative = np.array([0.0]*6)
freeze_integral = np.array([False]*6)

desired_pose = cfg.controller_desired_pose

# Kp = np.array([Kp_x] + [Kp_y] + [Kp_position_z] + [0.0]*2 + [-Kp_orientation])
Kp = np.array([cfg.Kp_position_x] + [cfg.Kp_position_y] + [cfg.Kp_position_z] + [0.0]*2 + [cfg.Kp_orientation])
Ki = np.array([cfg.Ki_position_x] + [cfg.Ki_position_y] + [cfg.Ki_position_z] + [0.0]*2 + [cfg.Ki_orientation])
Kd = np.array([cfg.Kd_position_x] + [cfg.Kd_position_y] + [cfg.Kd_position_z] + [0.0]*2 + [cfg.Kd_orientation])

actuation_saturation = cfg.actuation_saturation
error_integral_limit = cfg.error_integral_limit


def controller(state):
    global error_prev
    global error_integral
    global error_derivative
    global freeze_integral

    error = desired_pose - state
    error_integral += error*np.invert(freeze_integral)
    error_derivative = error - error_prev
    error_prev = error

    # error_integral = np.clip(error_integral, -error_integral_limit, error_integral_limit)

    actuation = (Kp*error + Kd*error_derivative + Ki*error_integral)
    
    actuation_clipped = np.clip(actuation, -actuation_saturation, actuation_saturation)

    # Stop integration when the controller saturates and the system error and the manipulated variable have the same sign
    saturated = np.not_equal(actuation, actuation_clipped)

    dot = error * actuation
    same_sign = np.greater(dot, np.array([0]*6))

    freeze_integral = np.logical_and(saturated, same_sign)


    return actuation_clipped



def main():
    rospy.init_node('pid_controller', anonymous=True)

    rospy.Subscriber('/ground_truth/state', Odometry, prev_gt_callback)

    
    reference_pub = rospy.Publisher('/drone_reference', Twist, queue_size=10)
    pose_pub = rospy.Publisher('/drone_pose', Twist, queue_size=10)
    error_pub = rospy.Publisher('/drone_error', Twist, queue_size=10)
    error_integral_pub = rospy.Publisher('/drone_error_integral', Twist, queue_size=10)

    control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    time.sleep(1)

    rospy.loginfo("Starting doing PID control with ar_pose as feedback")

    reference_msg = Twist()
    reference_msg.linear.x = desired_pose[0]
    reference_msg.linear.y = desired_pose[1]
    reference_msg.linear.z = desired_pose[2]
    reference_msg.angular.x = desired_pose[3]
    reference_msg.angular.y = desired_pose[4]
    reference_msg.angular.z = desired_pose[5]

    pose_msg = Twist()
    error_msg = Twist()
    error_integral_msg = Twist()

    rate = rospy.Rate(20) # Hz
    while not rospy.is_shutdown():
        actuation = controller(relative_position)
        msg = Twist()
        msg.linear.x = actuation[0]
        msg.linear.y = actuation[1]
        msg.linear.z = actuation[2]

        control_pub.publish(msg)

        # Publish values for tuning
        reference_pub.publish(reference_msg)

        pose_msg.linear.x = relative_position[0]
        pose_msg.linear.y = relative_position[1]
        pose_msg.linear.z = relative_position[2]
        pose_msg.angular.x = relative_position[3]
        pose_msg.angular.y = relative_position[4]
        pose_msg.angular.z = relative_position[5]
        pose_pub.publish(pose_msg)

        error_msg.linear.x = error_prev[0]
        error_msg.linear.y = error_prev[1]
        error_msg.linear.z = error_prev[2]
        error_msg.angular.x = error_prev[3]
        error_msg.angular.y = error_prev[4]
        error_msg.angular.z = error_prev[5]
        error_pub.publish(error_msg)

        error_integral_msg.linear.x = error_integral[0]
        error_integral_msg.linear.y = error_integral[1]
        error_integral_msg.linear.z = error_integral[2]
        error_integral_msg.angular.x = error_integral[3]
        error_integral_msg.angular.y = error_integral[4]
        error_integral_msg.angular.z = error_integral[5]
        error_integral_pub.publish(error_integral_msg)


        rate.sleep()
    
    
if __name__ == '__main__':
    main()