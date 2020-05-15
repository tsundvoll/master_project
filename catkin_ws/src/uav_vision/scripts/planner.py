#!/usr/bin/env python

"""
'Triangle'-button on controller to start the planner
'publish setpoints to '/set_point', Twist'
"""

import rospy
import numpy as np

from geometry_msgs.msg import Twist


est_relative_position = None

# States of the quadcopter
# S_INIT
# S_ON_GROUND
# S_TAKE_OFF
# S_HOVER
# S_MISSION
# S_LAND

mission_delta_x = 1.5
mission_delta_y = 3.0
mission_height = 2.0

mission = np.array([
    [0.0                , 0.0               , mission_height],
    [-mission_delta_x   , 0.0               , mission_height],
    [-mission_delta_x   , -mission_delta_y  , mission_height],
    [mission_delta_x    , -mission_delta_y  , mission_height],
    [mission_delta_x    , 0.0               , mission_height],
    [mission_delta_x    , mission_delta_y   , mission_height],
    [-mission_delta_x   , mission_delta_y   , mission_height],
    [-mission_delta_x   , 0.0               , mission_height],
    [0.0                , 0.0               , mission_height]
])

def estimate_callback(data):
    global est_relative_position
    est_relative_position = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])


def get_distance(point_a, point_b):
    translation = point_b - point_a
    distance = np.linalg.norm(translation)
    return distance


def main():
    rospy.init_node('planner', anonymous=True)

    rospy.Subscriber('/estimate/dead_reckoning', Twist, estimate_callback)

    pub_set_point = rospy.Publisher("/set_point", Twist, queue_size=10)
    set_point_msg = Twist()

    rospy.loginfo("Starting planner")

    speed = 1 # m/s
    publish_rate = 20 # Hz
    distance_margin = 0.2 # m

    mission_count = 0

    prev_major_set_point = mission[0]
    next_major_set_point = mission[1]
    next_minor_set_point = next_major_set_point


    rate = rospy.Rate(publish_rate) # Hz
    while not rospy.is_shutdown():

        # Time to change to next major setpoint
        if get_distance(next_minor_set_point, next_major_set_point) < distance_margin:
            if mission_count == len(mission):
                break

            next_major_set_point = mission[mission_count+1]

    
            translation = next_major_set_point - prev_major_set_point
            distance = np.linalg.norm(translation)

            step_time = distance / speed
            num_steps = step_time * publish_rate
            step_distance = translation / num_steps
            next_minor_set_point = prev_major_set_point
            
            prev_major_set_point = next_major_set_point

            mission_count += 1
        else:
            next_minor_set_point += step_distance

        # Publish set point
        set_point_msg.linear.x = next_minor_set_point[0]
        set_point_msg.linear.y = next_minor_set_point[1]
        set_point_msg.linear.z = next_minor_set_point[2]
        pub_set_point.publish(set_point_msg)

        rate.sleep()
    
    
if __name__ == '__main__':
    main()