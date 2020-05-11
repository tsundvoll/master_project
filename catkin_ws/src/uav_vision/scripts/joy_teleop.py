#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import numpy as np

# Higher sensitivity, higher speed
sensitivity_x_y = 2.0
sensitivity_z = 2.0
sensitivity_yaw = 1.0

set_point_linear_x = 0.0
set_point_linear_y = 0.0
set_point_linear_z = 2.0
set_point_angular_z = 0.0

set_points = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0])

def teleop_callback(data):
    global set_points

    # axes [8] = [left_js left-right(0), left_js up-down(1), L2(2), right_js left-right(3), right_js up-down(4), R2(5), upper_js left-right(6), upper_js up-down(7)]
    # buttons [13] = [cross(0), circle(1), triangle(2), square(3), L1(4), R1(5), L2(6), R2(7), share(8), options(9), start(10), js_left(11), js_right(12)]
    axes = data.axes
    buttons = data.buttons

    left_js_horizontal = axes[0]
    left_js_vertical = axes[1]

    right_js_horizontal = axes[3]
    right_js_vertical = axes[4]

    if buttons[0]:
        # "Cross"-button
        pub_take_off.publish(Empty())

    if buttons[1]:
        # "Circle"-button
        pub_land.publish(Empty())

    if buttons[3]:
        # "Square"-button
        pub_take_still_photo.publish(Empty())

    control_msg = Twist()
    control_msg.linear.x = right_js_vertical*sensitivity_x_y
    control_msg.linear.y = right_js_horizontal*sensitivity_x_y
    control_msg.linear.z = left_js_vertical*sensitivity_z
    control_msg.angular.z = left_js_horizontal*sensitivity_yaw
    # pub_controller.publish(control_msg)

    const = 0.001

    set_points += np.array([const*right_js_vertical*sensitivity_x_y,  const*right_js_horizontal*sensitivity_x_y,   const*left_js_vertical*sensitivity_z,
                            0.0,                                0.0,                                    const*left_js_horizontal*sensitivity_yaw])

    set_point_msg = Twist()
    set_point_msg.linear.x = set_points[0]
    set_point_msg.linear.y = set_points[1]
    set_point_msg.linear.z = set_points[2]
    set_point_msg.angular.z = set_points[5]
    pub_set_point.publish(set_point_msg)


def main():
    global pub_take_off
    global pub_land
    global pub_controller
    global pub_take_still_photo

    global pub_set_point

    pub_take_off = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=10)
    pub_land = rospy.Publisher("/ardrone/land", Empty, queue_size=10)
    pub_controller = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

    pub_set_point = rospy.Publisher("/set_point", Twist, queue_size=10)

    pub_take_still_photo = rospy.Publisher("/take_still_photo", Empty, queue_size=10)

    rospy.Subscriber("joy", Joy, teleop_callback)


    rospy.init_node('joy_teleop', anonymous=True)
    rospy.loginfo("Joystick teleoperation ready")


    rospy.spin()


if __name__ == '__main__':
    main()