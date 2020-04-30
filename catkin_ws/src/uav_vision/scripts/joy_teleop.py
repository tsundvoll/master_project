#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Higher sensitivity, higher speed
sensitivity_x_y = 2.0
sensitivity_z = 2.0
sensitivity_yaw = 1.0

def teleop_callback(data):
    # axes [8] = [left_js left-right(0), left_js up-down(1), L2(2), right_js left-right(3), right_js up-down(4), R2(5), upper_js left-right(6), upper_js up-down(7)]
    # buttons [13] = [cross(0), circle(1), triangle(2), square(3), L1(4), R1(5), L2(6), R2(7), share(8), options(9), start(10), js_left(11), js_right(12)]
    axes = data.axes
    buttons = data.buttons

    left_js_horizontal = axes[0]
    left_js_vertical = axes[1]

    right_js_horizontal = axes[3]
    right_js_vertical = axes[4]

    if buttons[0]:
        pub_take_off.publish(Empty())

    if buttons[1]:
        pub_land.publish(Empty())

    control_msg = Twist()
    control_msg.linear.x = right_js_vertical*sensitivity_x_y
    control_msg.linear.y = right_js_horizontal*sensitivity_x_y
    control_msg.linear.z = left_js_vertical*sensitivity_z
    control_msg.angular.z = left_js_horizontal*sensitivity_yaw
    pub_controller.publish(control_msg)


def main():
    global pub_take_off
    global pub_land
    global pub_controller

    pub_take_off = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=10)
    pub_land = rospy.Publisher("/ardrone/land", Empty, queue_size=10)
    pub_controller = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

    pub_set_point = rospy.Publisher("/set_point")

    rospy.Subscriber("joy", Joy, teleop_callback)


    rospy.init_node('joy_teleop', anonymous=True)
    rospy.loginfo("Joystick teleoperation ready")


    rospy.spin()


if __name__ == '__main__':
    main()