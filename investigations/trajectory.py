#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
from scipy.misc import imsave
import os
from scipy.spatial.transform import Rotation as R

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math


HELIPAD_POS_X = 1.0
HELIPAD_POS_Y = 1.0
HELIPAD_POS_Z = 0.54

# set_points = np.array([
#     [0.0, 0.0, 1.0],
#     [-1.0, -1.0, 1.0],
#     [1.0, -1.0, 1.0],
#     [1.0, 1.0, 1.0],
#     [-1.0, 1.0, 1.0],
#     [0.0, 0.0, 1.0]
# ])

# Dimentions
#         width        
# *                    *
# *                    *
# *                    * length
# *                    *
# *                    *
# *                    *

# ODD index round (starting at round 1)
# 0 5 l/10             *
# *                    0 3 l/10
# 0 1 l/10             *
# *                    0 -1 l/10
# 0 -3 l /10           *
# *                    0 -5 l/10

# EVEN index round
# *                    0 5 l/10
# 0 3 l/10             *
# *                    0 1 l/10
# 0 -1 l/10            *
# *                    0 -3 l /10
# 0 -5 l/10            *

speed = 2.0 # m/s

min_height = 1.0
max_height = 4.0
height_step = 0.5

# Settings for speed = 0.5
# At 1m
# length = 0.1 # in UAV's x direction
# width = 0.7 # in UAV's y direction

# At 3m
# length = 1.0 # in UAV's x direction
# width = 2.8 # in UAV's y direction

# At 5m
# length = 1.9 # in UAV's x direction
# width = 4.9 # in UAV's y direction

# At 7m
# length = 2.8 # in UAV's x direction
# width = 7.0 # in UAV's y direction



length_increase_with_height = 0.0 # per meter
width_increase_with_height = 0.0 # per meter


# sign = 1 or sign = -1
# set_points_element = np.array([
#     [0.0        , 0.0 , height],
#     [sign*5*l_10, -w_2, height],
#     [sign*3*l_10,  w_2, height],
#     [sign*1*l_10, -w_2, height],
#     [-sign*1*l_10,  w_2, height],
#     [-sign*3*l_10, -w_2, height],
#     [-sign*5*l_10,  w_2, height],
# ])


# Add all the setpoints to an array
set_points = np.empty((0,3))

height = min_height
sign = 1
while height < max_height:

    l = max(-0.35 + 0.45*height, 0)
    w = max(-0.35 + 1.05*height, 0)

    l_10 = l/10.0
    w_2 = w/2.0

    set_points_element = np.array([
    [0.0        , 0.0 , height],
    [sign*5*l_10, -w_2, height],
    [sign*3*l_10,  w_2, height],
    [sign*1*l_10, -w_2, height],
    [-sign*1*l_10,  w_2, height],
    [-sign*3*l_10, -w_2, height],
    [-sign*5*l_10,  w_2, height],
    ])

    # set_points_element = np.array([
    # [0.0        , 0.0 , height],
    # [0.0, w_2, height],
    # [0.0, -w_2, height],
    # # [5*l_10,  0.0, height],
    # # [-5*l_10,  0.0, height],
    # ])

    set_points = np.concatenate((set_points, set_points_element))

    height += height_step
    sign *= -1

# Add the final set point
final_set_point = np.array([
    [0.0, 0.0, max_height]
])
set_points = np.concatenate((set_points, final_set_point))

print(set_points)

# Running setting

frequency = 10

time_step = 1.0 / frequency


set_point_counter = 0
step_counter = 0
transition_steps = 0
step_vector = np.empty(3)
def get_next_set_point(uav_time):
    global set_point_counter
    global step_counter
    global step_vector
    global transition_steps

    if set_point_counter+1 >= len(set_points):
        return set_points[len(set_points)-1]

    start_point = set_points[set_point_counter]
    end_point = set_points[set_point_counter+1]

    if step_counter == 0:
        vector = end_point - start_point
        distance = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
        # rospy.loginfo("Distance to next setpoint: " + str(distance))
    
        transition_duration = distance / speed # 1.41421 s

        transition_steps = max(1, math.ceil(transition_duration / time_step))

        step_vector = vector / transition_steps
        # rospy.loginfo("step_vector: " + str(step_vector))
        
        next_set_point = start_point
        step_counter += 1

    elif step_counter == transition_steps:
        step_counter = 0
        set_point_counter += 1
        next_set_point = end_point

    else:
        next_set_point = start_point + step_vector*step_counter
        step_counter += 1

    

    return next_set_point

def run():   
    rospy.init_node('trajectory', anonymous=True)

    topic = 'visualization_marker_array'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
    set_point_pub = rospy.Publisher("/set_point", Point, queue_size=10)

    rospy.loginfo("Starting trajectory module")

    markerArray = MarkerArray()

    # Landing platform cylinder
    helipad = Marker()
    helipad.header.frame_id = "/ocean"
    helipad.id = 0
    helipad.type = helipad.CYLINDER
    helipad.action = helipad.ADD
    helipad.scale.x = 1.0
    helipad.scale.y = 1.0
    helipad.scale.z = 0.1
    helipad.color.r = 0.04
    helipad.color.g = 0.8
    helipad.color.b = 0.04
    helipad.color.a = 0.3
    helipad.pose.orientation.w = 1.0
    helipad.pose.position.x = HELIPAD_POS_X
    helipad.pose.position.y = HELIPAD_POS_Y
    helipad.pose.position.z = HELIPAD_POS_Z

    # UAV sphere
    marker = Marker()
    marker.header.frame_id = "/ocean"
    marker.id = 1
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0

    
    uav_time = 0.0

    set_point_msg = Point()

    rate = rospy.Rate(frequency) # Hz
    while not rospy.is_shutdown():


        uav_set_point = get_next_set_point(uav_time)
        uav_time += time_step
        rospy.loginfo("Uav_time: " + str(uav_set_point))

        if len(markerArray.markers) == 0:
            markerArray.markers.append(helipad)
        else:
            markerArray.markers[0] = helipad

        if uav_set_point is not None:
            
            marker.pose.position.x = HELIPAD_POS_X + uav_set_point[0]
            marker.pose.position.y = HELIPAD_POS_Y + uav_set_point[1]
            marker.pose.position.z = HELIPAD_POS_Z + uav_set_point[2]

            if len(markerArray.markers) <= 1:
                markerArray.markers.append(marker)
            else:
                markerArray.markers[1] = marker
        

        # Publish the MarkerArray
        publisher.publish(markerArray)

        # Publish the setpoint
        set_point_msg.x = uav_set_point[0]
        set_point_msg.y = uav_set_point[1]
        set_point_msg.z = uav_set_point[2]
        set_point_pub.publish(set_point_msg)

        # if uav_time >= 3.0:
        #     break

        rate.sleep()
    
    

if __name__ == '__main__':
    run()