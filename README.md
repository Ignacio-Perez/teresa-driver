# Teresa Driver
A ROS driver for TERESA by using the IdMind hardware

The package is composed by two programs:

* *teresa_driver* is a driver for controlling the robot by using ROS.
* *teresa_teleop_joy* is a program for teleoperating the robot with a joystick by using ROS (*optional feature*).

The teresa_teleop_joy program is based on the following tutorial: 
*Writing a Teleoperation Node for a Linux-Supported Joystick* 
http://wiki.ros.org/joy/Tutorials/WritingTeleopNode 
under license http://creativecommons.org/licenses/by/3.0/


- See http://teresaproject.eu for more information about TERESA. 
- See http://www.idmind.pt for more information about IdMind company.
- See http://www.ros.org for more information about the ROS framework.

## General requirements

* TERESA robot with IdMind hardware setup.
  
* Computer to be located on the platform running Ubuntu Linux 14.04 and ROS Indigo.

* An Inertial Measurement Unit (IMU) compatible with ROS. The system has been tested with the *Xsens Mti 30* IMU.

* A wireless joystick or gamepad with at least 8 buttons and 1 axis compatible with ROS (*optional*). The system has been tested with the *Logitech Wireless F710* gamepad. 


## Compilation
In order to build the package, clone it to the *src* directory of your Catkin workspace and compile it by using *catkin_make* as normal.


## DCDC output

The DCDC output is configured by using a binary mask HGFEDCBA:

* A = 1 Enable RD0 5V output
* A = 0 Disable RD0 5V output
* B = 1 Enable RD1 5V output
* B = 0 Disable RD1 5V output
* C = 1 Enable RD2 12V output
* C = 0 Disable RD2 12V output
* D = 1 Enable RD3 12V output
* D = 0 Disable RD3 12V output
* E = 1 Enable RD4 12V output
* E = 0 Disable RD4 12V output
* F = 1 Enable RD5 12V output
* F = 0 Disable RD5 12V output
* G = 1 Enable RD6 12V output
* G = 0 Disable RD6 12V output
* H = 1 Enable RD7 12V output
* H = 0 Disable RD7 12V output


The DCDC mask can be set by using the */teresa_dcdc* service (see services section) and/or the *initial_dcdc_mask* and *final_dcdc_mask* (see parameters section)


## How to command the robot by using ROS

The driver accepts motion commands from the next ROS topics:

* **/cmd_vel** of type **geometry_msgs::Twist** in order to get instant angular and linear velocities.

* **/stalk** of type **teresa_driver::stalk** in order to get the commands for the heigth and tilt of the head. This topic is built-in with the package.

The format of the */stalk* topic is as follows:

* bool head_up
* bool head_down
* bool tilt_up
* bool tilt_down

If the value of *head_up* is *true*, the heigth of the head is increased at a constant rate. If the value of *head_down* is *true*, the heigth of the head is decreased at a constant rate. The behavior of *tilt_up* and *tilt_down* is analogous for the angle of the head.

* **/stalk_ref** of type **teresa_driver::stalk_ref** in order to directly set the stalk (meters) and tilt (radians) reference

## Required ROS topics

The next topic is required in order to run the *teresa_driver* program:

* **/imu/data** of type **sensor_msgs::Imu** in order to get the information of the IMU.

The next topics is required in order to run the *teresa_teleop_joy* program:

* **/joy** of type **sensor_msgs::Joy** in order get the input of the joystick/gamepad.


## Published ROS topics

The next topics are published by the *teresa_driver*: 

* **/odom** of type **nav_msgs::Odometry**

* **/batteries** of type **teresa_driver::batteries** in order to publish the status of the batteries. This topic is built-in with the package.

* **/arcade_buttons** of type **teresa_driver::arcade_buttons** in order to publish the status of the red/green arcade buttons. This topic is built-in with the package.

* **/temperatures** of type **teresa_driver::temperatures** in order to publish hardware temperatures and overheat alarms.

* **/teresa_diagnostics** of type **teresa_driver::diagnostics** in order to publish diagnostics information about voltages, currents and average main loop time

* **/volume_increment** of type **teresa_driver::volume_increment** in order to publish information about the incremental rotary encoder (volume)

The next topics are published by the *teresa_teleop_joy*:

* **/cmd_vel** of type **geometry_msgs::Twist** in order to command the robot by reading the status of the joystick.

* **/stalk** of type **teresa_driver::cmd_vel_avr** in order to command the height and tilt of the head of the robot by reading the status of the joystick. 


## Provided services

The *teresa_driver* provides the next services:

* **/set_teresa_dcdc** in order to set the current DCDC mask (see DCDC output section)

  * Input:
    1. uint8 req.mode: 
      - If req.mode is 0: mask = req.mask
      - If req.mode is 1: mask = mask | req.mask
      - If req.mode is 2: mask = mask & ~req.mask 
    2. uint8 req.mask 
  * Output:
    1. bool res.success

* **/get_teresa_dcdc** in order to get the current DCDC mask (see DCDC output section)

  * Input: nothing
  * Output:
    1. uint8 res.mask
    2. bool res.success

* **/teresa_leds** in order to set the RGB leds

  * Input: 
    - uint8[] req.rgb_values: [Red_1, Green_1, Blue_1,..., Red_N, Green_N, Blue_N] where N is the number of existent leds
  * Output:
    - bool res.success
 
## ROS parameters

Parameters of the *teresa_driver* program:

* **board1**: device of the sensors board (i.e. /dev/ttyUSB0)

* **board2**: device of the motors board (i.e. /dev/ttyUSB1)

* **base_frame_id**: base_frame identifier

* **odom_frame_id**: odom_frame identifier

* **head_frame_id**: head_frame identifier

* **stalk_frame_id**: stalk_frame identifier

* **freq**: Frequency in hertzs of the main loop.

* **using_imu**: 1 if using IMU, 0 otherwise (angular velocity will be calculated by using the motor encoders)

* **simulation**: 1 if using a simulated robot for debugging and testing, 0 if using the actual robot

* **publish_temperatures**: 1 if temperatures should be published, 0 otherwise

* **publish_buttons**: 1 if arcade buttons should be published, 0 otherwise

* **publish_volume**: 1 if incremental rotary encoder status should be published, 0 otherwise

* **publish_diagnostics**: 1 if power, current and loop frequency diagnostics should be published, 0 otherwise

* **number_of_leds**: number of existent leds

* **initial_dcdc_mask**: DCDC mask to be set after starting the node (see DCDC output section)

* **final_dcdc_mask**: DCDC mask to be set before finishing the node (see DCDC output section)

* **height_velocity**: Velocity of the height motor (in  mm/s)

* **tilt_velocity**: Velocity of the tilt motor (in degrees/s)

* **leds_pattern**: Name of the light pattern to use while the node is running (optionally). If no led pattern is configured, the leds could be set by using the teresa_leds service (see services section)


Parameters of the *teresa_teleop_joy* program:

* **freq**: Frequency in hertzs of the main loop.

* **panic_freq**: Frequency in hertzs of the main loop in case of pushing the panic button in the joystick.

* **linear_velocity_axis**: Id of the axis to control the linear velocity.

* **angular_velocity_axis**: Id of the axis to control the angular velocity.

* **max_velocity_button**: Id of the button to allow the maximun velocity.

* **head_up_button**: Id of the button to move up the head of the robot.

* **head_down_button**: Id of the button to move down the head of the robot.

* **panic_button**: Id of the panic button in the joystick.

* **move_primary_button**: Id of the primary button to allow movements with the axis.

* **move_secundary_button**: Id of the secundary button to allow movements with the axis.

* **tilt_up_button**: Id of the button to increase the tilt angle of the head.

* **tilt_down_button**: Id of the button to decrease the tilt angle of the head.

* **max_linear_velocity**: Maximun allowed linear velocity in m/s.

* **max_angular_velocity**: Maximun allowd angular velocity in rad/s.


