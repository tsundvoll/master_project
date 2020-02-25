#!/usr/bin/env python
import numpy as np
import math



reference_height = 1.5
offset_setpoint_x = -0.2
offset_setpoint_y = 0.0
controller_desired_pose = np.array([offset_setpoint_x, offset_setpoint_y, reference_height, 0.0, 0.0, -math.pi/2])

####################
#  PID parameters  #
####################
Kp_position_x = 1.5
Ki_position_x = 0.01
Kd_position_x = 0.3
####################
Kp_position_y = Kp_position_x
Ki_position_y = Ki_position_x
Kd_position_y = Kd_position_x
####################
Kp_position_z = 1.5
Ki_position_z = 0.01
Kd_position_z = 0.3
####################
Kp_orientation = 0.0
Ki_orientation = 0.0
Kd_orientation = 0.0
####################

actuation_saturation = 5 # % of maximum velocity
error_integral_limit = 40