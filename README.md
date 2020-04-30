# master_project


ROS package names alternatives:
* master
* auto_lander
* drone_vision
* lander_vision
* uav_lander

* uav_vision!



## Installation

Prerequisites:
* Ubuntu Kinetic (16.04)


### Install ROS Kinetic
Follow this guide: http://wiki.ros.org/kinetic/Installation/Ubuntu

### gazebo_ros (Gazebo 7.0)
sudo apt-get install ros-kinetic-gazebo-ros

### uuv_simulator
* uuv_simulator (https://github.com/uuvsimulator/uuv_simulator)
sudo apt install ros-kinetic-uuv-simulator

## Packages used:
* ardrone_autonomy (https://ardrone-autonomy.readthedocs.io/en/latest/installation.html)
```
sudo apt-get install ros-kinetic-ardrone-autonomy
```

* tum_simulator on Kinetic and Gazebo 7 (https://github.com/eborghi10/AR.Drone-ROS)
    (Download files and place them in folder /ardrone_simulator_gazebo7)


* Matplotlib 2.2.4

## Add the necessary models
* Add models to the hidden folder .gazebo/models


## Scipy
Scipy 1.4.1 seems to be necessary to have to run pid_controller.py.
However, it is a bit hard to install with python 2.7, and I have not yet solved this problem.
Scipy 1.2.1 works

### Other useful things:
```
sudo apt-get install python-pip
python -m pip install --user numpy scipy matplotlib 
```

* Terminator
```
sudo apt-get install terminator
```

## TO DO 14. February:
* Collect a larger dataset:
From 1m to 10m.

Ascending 1 meter with speed 0.1 and step 0.1 takes approximately 709 s

70s per runde 0.1 m/s = 7 meter per runde

Speed 0.1
Height 1.0 to 11.0

At speed 100, estimated time was 18.488685s
At speed 0.1, this becomes 18489s


* Add extra setpoint before turning, to get smoother corners

* Create an automatic report from the NN training over the architecture and the hyperparameters


## Install Olympe
Follow this guide: https://developer.parrot.com/docs/olympe/installation.html


To investigate topics, use the command:
```
parrot-gz topic
```
With these options:
```
Options:
  -h [ --help ]           Print this help message
  -w [ --world-name ] arg World name.
  -l [ --list ]           List all topics.
  -i [ --info ] arg       Get information about a topic.
  -e [ --echo ] arg       Output topic data to screen.
  -p [ --publish ] arg    Publish message on a topic.
  -r [ --request ] arg    Send a request.
  -v [ --view ] arg       View topic data using a QT widget.
  -z [ --hz ] arg         Get publish frequency.
  -b [ --bw ] arg         Get topic bandwidth.
  -u [ --unformatted ]    Output data from echo without formatting.
  -d [ --duration ] arg   Duration (seconds) to run. Applicable with echo, hz, 
                          and bw
  -m [ --msg ] arg        Message to send on topic. Applicable with publish and
                          request
  -f [ --file ] arg       Path to a file containing the message to send on 
                          topic. Applicable with publish and request
```

Topics from the Sphinx simulator:

/gazebo/default/anafi4k/body/ultrasound/scan



/gazebo/default/anafi4k/body/vertical_camera/image
/gazebo/default/anafi4k/body/wrench
/gazebo/default/anafi4k/gimbal_1/wrench
/gazebo/default/anafi4k/gimbal_2/horizontal_camera/image
/gazebo/default/anafi4k/gimbal_2/wrench
/gazebo/default/anafi4k/joint_cmd
/gazebo/default/animated_revolt/body/wrench
/gazebo/default/animated_revolt/helipad_ar/link_01/wrench
/gazebo/default/atmosphere
/gazebo/default/default::ultrasound/cmd
/gazebo/default/diagnostics
/gazebo/default/factory
/gazebo/default/factory/light
/gazebo/default/gui
/gazebo/default/gzclient_camera/cmd
/gazebo/default/horizontal_camera/cmd
/gazebo/default/joint
/gazebo/default/light/modify
/gazebo/default/log/control
/gazebo/default/log/status
/gazebo/default/marker
/gazebo/default/model/info
/gazebo/default/model/modify
/gazebo/default/ocean/ocean_link/wrench
/gazebo/default/omniscient_anafi4k/contacts
/gazebo/default/physics
/gazebo/default/physics/contacts
/gazebo/default/playback_control
/gazebo/default/pose/info
/gazebo/default/pose/modify
/gazebo/default/request
/gazebo/default/response
/gazebo/default/roads
/gazebo/default/scene
/gazebo/default/selection
/gazebo/default/sensor
/gazebo/default/skeleton_pose/info
/gazebo/default/sky
/gazebo/default/user_camera/joy_pose
/gazebo/default/user_camera/joy_twist
/gazebo/default/user_camera/pose
/gazebo/default/vertical_camera/cmd
/gazebo/default/visual
/gazebo/default/wind
/gazebo/default/world_control
/gazebo/default/world_stats
/gazebo/server/control
/gazebo/world/modify

# Sensor msg imu
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z


# Joystick:
* Connect to PS4 controller via Bluetooth
* run rosrun joy joy_node
* run rosrun uav_vision joy_teleop.py