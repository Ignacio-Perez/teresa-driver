/***********************************************************************/
/**                                                                    */
/** teresa_node.hpp                                                    */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Noe Perez                                                          */
/** Rafael Ramon                                                       */
/** David Alejo Teissière                                              */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/
#ifndef _TERESA_NODE_HPP_
#define _TERESA_NODE_HPP_

#include <ros/ros.h>
#include <string>
#include <cmath>	
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			
#include <geometry_msgs/Twist.h>		
#include <sensor_msgs/Imu.h>
#include <teresa_driver/Stalk.h>
#include <teresa_driver/StalkRef.h>
#include <teresa_driver/Temperature.h>
#include <teresa_driver/Buttons.h>
#include <teresa_driver/Volume.h>
#include <teresa_driver/Batteries.h>
#include <teresa_driver/Teresa_DCDC.h>
#include <teresa_driver/Teresa_leds.h>
#include <teresa_driver/Diagnostics.h>
#include <teresa_driver/simulated_teresa_robot.hpp>
#include <teresa_driver/idmind_teresa_robot.hpp>



namespace Teresa
{

enum MotorStatus {MOTOR_UP, MOTOR_DOWN, MOTOR_STOP};

class Node
{
public:
	Node(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Node();
	
private:
	void loop();
	void imuReceived(const sensor_msgs::Imu::ConstPtr& imu);
	void stalkReceived(const teresa_driver::Stalk::ConstPtr& stalk);
	void stalkRefReceived(const teresa_driver::StalkRef::ConstPtr& stalk_ref);
	void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	bool teresaDCDC(teresa_driver::Teresa_DCDC::Request  &req,
			teresa_driver::Teresa_DCDC::Response &res);

	bool teresaLeds(teresa_driver::Teresa_leds::Request &req,
				teresa_driver::Teresa_leds::Response &res);

	static void printInfo(const std::string& message){ROS_INFO("%s",message.c_str());}	
	static void printError(const std::string& message){ROS_ERROR("%s",message.c_str());}
	
	ros::NodeHandle& n;
	double ang_vel;
	bool imu_error;
	double yaw;
	bool imu_first_time;
	int using_imu;
	int publish_temperature;
	int publish_buttons;
	int publish_volume;
	int publish_power_diagnostics;
	int height_velocity;
	int tilt_velocity;
	double freq;
	std::string base_frame_id;
	std::string odom_frame_id;
	std::string head_frame_id;
	std::string stalk_frame_id;
	ros::Publisher odom_pub;
	ros::Subscriber cmd_vel_sub;
	ros::Subscriber imu_sub;
	ros::Subscriber stalk_sub;
	ros::Subscriber stalk_ref_sub;
	ros::Publisher buttons_pub;
	ros::Publisher batteries_pub;
	ros::Publisher volume_pub;
	ros::Publisher diagnostics_pub;
	ros::Publisher temperature_pub;	
	ros::ServiceServer dcdc_service;
	ros::ServiceServer leds_service;
	ros::Time imu_time;
	ros::Time imu_past_time;
	ros::Time cmd_vel_time;
	Robot *teresa;
	MotorStatus tiltMotor;
	MotorStatus heightMotor;
};

inline
Node::Node(ros::NodeHandle& n, ros::NodeHandle& pn)
: n(n), 
  ang_vel(0.0),
  imu_error(false),
  yaw(0.0),
  imu_first_time(true),
  teresa(NULL),
  tiltMotor(MOTOR_STOP),
  heightMotor(MOTOR_STOP)
{
	try
	{
		int simulation;
		std::string board1;
		std::string board2;
		int initial_dcdc_mask,final_dcdc_mask,number_of_leds;
		pn.param<std::string>("board1",board1,"/dev/ttyUSB0");
		pn.param<std::string>("board2",board2,"/dev/ttyUSB1");
		pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
		pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
		pn.param<std::string>("head_frame_id", head_frame_id, "/teresa_head");	
	    	pn.param<std::string>("stalk_frame_id", stalk_frame_id, "/teresa_stalk");
		pn.param<int>("simulation",simulation,0);	
		pn.param<int>("using_imu", using_imu, 1);
		pn.param<int>("publish_temperature", publish_temperature, 1);
		pn.param<int>("publish_buttons", publish_buttons, 1);
		pn.param<int>("publish_volume", publish_volume, 1);
		pn.param<int>("publish_power_diagnostics",publish_power_diagnostics,1);
		pn.param<int>("number_of_leds",number_of_leds,38);
		pn.param<int>("initial_dcdc_mask",initial_dcdc_mask,0xFF);
		pn.param<int>("final_dcdc_mask",final_dcdc_mask,0x00);
		pn.param<double>("freq",freq,20);
		pn.param<int>("height_velocity",height_velocity,20);
		pn.param<int>("tilt_velocity",tilt_velocity,2);
		if (simulation) {
			using_imu=0;
			teresa = new SimulatedRobot();
		} else {
			teresa = new IdMindRobot(board1,board2,initial_dcdc_mask,final_dcdc_mask,number_of_leds,printInfo,printError);
		}
		odom_pub = pn.advertise<nav_msgs::Odometry>(odom_frame_id, 5);
		cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1,&Node::cmdVelReceived,this);
		if (using_imu) {
			imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data",1,&Node::imuReceived,this);	
		}
		stalk_sub = n.subscribe<teresa_driver::Stalk>("/stalk",1,&Node::stalkReceived,this);
		stalk_ref_sub =n.subscribe<teresa_driver::StalkRef>("/stalk_ref",1,&Node::stalkRefReceived,this);
		if (publish_buttons) {
			buttons_pub = pn.advertise<teresa_driver::Buttons>("/arcade_buttons",5);
		}
		if (publish_volume) {
			volume_pub = pn.advertise<teresa_driver::Volume>("/volume_increment",5);
		}
		if (publish_temperature) {
			temperature_pub = pn.advertise<teresa_driver::Temperature>("/temperatures",5);
		}
		if (publish_power_diagnostics) {
			diagnostics_pub = pn.advertise<teresa_driver::Diagnostics>("/teresa_diagnostics",5);
		}
		batteries_pub = pn.advertise<teresa_driver::Batteries>("/batteries",5);	
		dcdc_service = n.advertiseService("teresa_dcdc", &Node::teresaDCDC,this);		
		leds_service = n.advertiseService("teresa_leds", &Node::teresaLeds,this);
		loop();
	} catch (const char* msg) {
		ROS_FATAL("%s",msg);
	}	
}

inline
Node::~Node()
{
	delete teresa;
}

inline
void Node::imuReceived(const sensor_msgs::Imu::ConstPtr& imu)
{
	imu_time = ros::Time::now();
	if (imu_first_time) {
		imu_past_time = imu->header.stamp;
		imu_first_time=false;
		return;
	}
	double duration = (imu->header.stamp - imu_past_time).toSec();
	imu_past_time=imu->header.stamp; 
	tf::Quaternion imu_quaternion;
    	geometry_msgs::Quaternion imu_quaternion_msg;
    	if (teresa->isStopped() || fabs(imu->angular_velocity.z) < 0.04) {
		ang_vel = 0.0;
		return;
	}
	ang_vel = imu->angular_velocity.z;
	yaw += ang_vel * duration;
}

inline
void Node::stalkReceived(const teresa_driver::Stalk::ConstPtr& stalk)
{
	if (stalk->head_up && heightMotor!=MOTOR_UP) {
		teresa->setHeightVelocity(height_velocity);
		teresa->setHeight(MAX_HEIGHT_MM);
		heightMotor = MOTOR_UP;
	}
	else
	if (stalk->head_down && heightMotor!=MOTOR_DOWN) {
		teresa->setHeightVelocity(height_velocity);
		teresa->setHeight(MIN_HEIGHT_MM);
		heightMotor = MOTOR_DOWN;
	}
	else if (heightMotor!=MOTOR_STOP){
		teresa->setHeightVelocity(0);
		heightMotor = MOTOR_STOP;
	}
	
	if (stalk->tilt_up && tiltMotor!=MOTOR_UP) {
		teresa->setTiltVelocity(tilt_velocity);
		teresa->setTilt(MAX_TILT_ANGLE_DEGREES);
		tiltMotor = MOTOR_UP;
	}
	else
	if (stalk->tilt_down && tiltMotor!=MOTOR_DOWN) {
		teresa->setTiltVelocity(tilt_velocity);
		teresa->setTilt(MIN_TILT_ANGLE_DEGREES);
		tiltMotor = MOTOR_DOWN;
	}
	else if (tiltMotor!=MOTOR_STOP){
		teresa->setTiltVelocity(0);
		tiltMotor = MOTOR_STOP;
	}
}

inline
void Node::stalkRefReceived(const teresa_driver::StalkRef::ConstPtr& stalk_ref)
{
	teresa->setHeightVelocity(height_velocity);
	teresa->setTiltVelocity(tilt_velocity);
	teresa->setHeight((int)std::round(stalk_ref->head_height*1000));
        teresa->setTilt((int)std::round(stalk_ref->head_tilt * 57.2958));
}

inline
void Node::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	cmd_vel_time = ros::Time::now();
	if (!imu_error) {
		teresa->setVelocity( cmd_vel->linear.x,cmd_vel->angular.z);
	}
}

inline
bool Node::teresaDCDC(teresa_driver::Teresa_DCDC::Request  &req,
			teresa_driver::Teresa_DCDC::Response &res)
{
	res.success = teresa->enableDCDC(req.mask);
	return true;
}

inline
bool Node::teresaLeds(teresa_driver::Teresa_leds::Request &req,
			teresa_driver::Teresa_leds::Response &res)
{
	res.success = teresa->setLeds(req.rgb_values);
	return true;
}

inline
void Node::loop()
{
	double lin_vel=0.0;
	double pos_x=0.0;
	double pos_y=0.0;
	ros::Time current_time,last_time;
	if (using_imu) {
		imu_time = ros::Time::now();
	}
	last_time = ros::Time::now();
	cmd_vel_time = ros::Time::now();
	ros::Rate r(freq);
	tf::TransformBroadcaster tf_broadcaster;
	double imdl,imdr;
	double dt;
	bool first_time=true;
	double height_in_meters=0;
	int height_in_millimeters=0;
	double tilt_in_radians=0;
	int tilt_in_degrees=0;
	bool button1=false,button2=false;
	bool button1_tmp,button2_tmp;
	int rotaryEncoder;
	unsigned char elec_level, PC1_level, motorH_level, motorL_level, charger_status;
	int temperature_left_motor,temperature_right_motor,temperature_left_driver,temperature_right_driver;
	bool tilt_overheat,height_overheat;
	PowerDiagnostics diagnostics;

	double loopDurationSum=0;
	unsigned long loopCounter=0;
	while (n.ok()) {
		current_time = ros::Time::now();
		if (using_imu) {		
			double imu_sec = (current_time - imu_time).toSec();
			if(imu_sec >= 0.25){
				teresa->setVelocity(0,0);
				ang_vel = 0;
				imu_error = true;
				ROS_WARN("-_-_-_-_-_- IMU STOP -_-_-_-_-_- imu_sec=%.3f sec",imu_sec);
			} else {
				imu_error = false;
			}
		}
		double cmd_vel_sec = (current_time - cmd_vel_time).toSec();
		if (cmd_vel_sec >= 0.5) {
			teresa->setVelocity(0,0);
		}
		teresa->getIMD(imdl,imdr);
		dt = (current_time - last_time).toSec();
		if (!using_imu) {
			double vr = imdr/dt;
			double vl = imdl/dt;
			ang_vel = (vr-vl)/ROBOT_DIAMETER_M;
			yaw += ang_vel*dt;
		}
		last_time = current_time;
		if (!first_time) {
			double imd = (imdl+imdr)/2;
			lin_vel = imd / dt;
			pos_x += imd*std::cos(yaw + ang_vel*dt/2);
			pos_y += imd*std::sin(yaw + ang_vel*dt/2);
		} 
		
		
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = odom_frame_id;
		odom_trans.child_frame_id = base_frame_id;
		odom_trans.transform.translation.x = pos_x;
		odom_trans.transform.translation.y = pos_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
		tf_broadcaster.sendTransform(odom_trans);
		if (teresa->getHeight(height_in_millimeters)) {
			height_in_meters= (double)height_in_millimeters * 0.001;
		}
		
        	if (teresa->getTilt(tilt_in_degrees)) {
			tilt_in_radians = tilt_in_degrees * 0.0174533;
		}

		geometry_msgs::TransformStamped stalk_trans;
		stalk_trans.header.stamp = current_time;
		stalk_trans.header.frame_id = base_frame_id;
		stalk_trans.child_frame_id = stalk_frame_id;
		stalk_trans.transform.translation.x = 0.0;
		stalk_trans.transform.translation.y = 0.0;
		stalk_trans.transform.translation.z = height_in_meters;
		stalk_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
		tf_broadcaster.sendTransform(stalk_trans);

		geometry_msgs::TransformStamped head_trans;
		head_trans.header.stamp = current_time;
		head_trans.header.frame_id = stalk_frame_id;
		head_trans.child_frame_id = head_frame_id;
		head_trans.transform.translation.x = 0.0;
		head_trans.transform.translation.y = 0.0;
		head_trans.transform.translation.z = 0.0;
		head_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -tilt_in_radians, 0.0);
		tf_broadcaster.sendTransform(head_trans);

		// ******************************************************************************************
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = odom_frame_id;
		
		//set the position
		odom.pose.pose.position.x = pos_x;
		odom.pose.pose.position.y = pos_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
		
		//set the velocity
		odom.child_frame_id = base_frame_id;
		odom.twist.twist.linear.x = lin_vel;
		odom.twist.twist.linear.y = 0.0; 
		odom.twist.twist.angular.z = ang_vel;
		
		//publish the odometry
		odom_pub.publish(odom);

		//publish the state of the batteries
		if (teresa->getBatteryStatus(elec_level,PC1_level,motorH_level,motorL_level,charger_status)) {
			teresa_driver::Batteries battmsg;
			battmsg.header.stamp = current_time;
			battmsg.elec_level = elec_level;
			battmsg.PC1_level = PC1_level;
			battmsg.motorH_level = motorH_level;
			battmsg.motorL_level = motorL_level;
			battmsg.charger_status = charger_status;
			batteries_pub.publish(battmsg);	
		}

		//publish the state of the buttons
		if (publish_buttons &&	teresa->getButtons(button1_tmp,button2_tmp) && 
			(first_time || button1!=button1_tmp || button2!=button2_tmp)) {
			button1 = button1_tmp;
			button2 = button2_tmp;
			teresa_driver::Buttons buttonsmsg;
			buttonsmsg.header.stamp = current_time;
			buttonsmsg.button1=button1;
			buttonsmsg.button2=button2;
			buttons_pub.publish(buttonsmsg);
		}
		//publish the state of the rotaryEncoder				
		if (publish_volume && teresa->getRotaryEncoder(rotaryEncoder) && rotaryEncoder!=0) {
			teresa_driver::Volume volumemsg;
			volumemsg.header.stamp = current_time;
			volumemsg.volume_inc=rotaryEncoder;
			volume_pub.publish(volumemsg);
		}
		//publish the temperatures
		if (publish_temperature &&
			teresa->getTemperature(temperature_left_motor,
						temperature_right_motor,
						temperature_left_driver,
						temperature_right_driver,
						tilt_overheat,
						height_overheat)) {
			teresa_driver::Temperature temperaturemsg;
			temperaturemsg.header.stamp = current_time;
			temperaturemsg.left_motor_temperature = temperature_left_motor;
			temperaturemsg.right_motor_temperature = temperature_right_motor;
			temperaturemsg.left_driver_temperature = temperature_left_driver;
			temperaturemsg.right_driver_temperature = temperature_right_driver;
			temperaturemsg.tilt_driver_overheat = tilt_overheat;
			temperaturemsg.height_driver_overheat = height_overheat;
			temperature_pub.publish(temperaturemsg);
		}

		//publish power diagnostics
		if (publish_power_diagnostics && teresa->getPowerDiagnostics(diagnostics)) {
			teresa_driver::Diagnostics diagnosticsmsg;
			diagnosticsmsg.header.stamp = current_time;
			diagnosticsmsg.elec_bat_voltage = diagnostics.elec_bat_voltage;
			diagnosticsmsg.PC1_bat_voltage = diagnostics.PC1_bat_voltage;
			diagnosticsmsg.cable_bat_voltage = diagnostics.cable_bat_voltage;
			diagnosticsmsg.motor_voltage = diagnostics.motor_voltage;
			diagnosticsmsg.motor_h_voltage = diagnostics.motor_h_voltage;
			diagnosticsmsg.motor_l_voltage = diagnostics.motor_l_voltage;
			diagnosticsmsg.elec_instant_current = diagnostics.elec_instant_current;
			diagnosticsmsg.motor_instant_current = diagnostics.motor_instant_current;
			diagnosticsmsg.elec_integrated_current = diagnostics.elec_integrated_current;
			diagnosticsmsg.motor_integrated_current = diagnostics.motor_integrated_current;
			diagnosticsmsg.average_loop_freq = 1.0 / (loopDurationSum/(double)loopCounter);
			diagnostics_pub.publish(diagnosticsmsg);
		}
		first_time=false;
		r.sleep();	
		ros::spinOnce();
		loopDurationSum += (ros::Time::now() - current_time).toSec();
		loopCounter++;
		
	}	

}

}

#endif

