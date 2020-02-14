# master_project


ROS package names:
* master
* auto_lander
* drone_vision
* lander_vision
* uav_lander

* uav_vision


## Packages used:
* uuv_simulator (https://github.com/uuvsimulator/uuv_simulator)

* tum_simulator on Kinetic and Gazebo 7 (https://github.com/eborghi10/AR.Drone-ROS)

* key_teleop (https://github.com/ros-teleop/teleop_tools)

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