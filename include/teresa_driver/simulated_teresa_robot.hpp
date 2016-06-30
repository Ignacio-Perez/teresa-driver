/***********************************************************************/
/**                                                                    */
/** simulated_teresa_robot.hpp                                         */
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

#ifndef _SIMULATED_TERESA_ROBOT_HPP_
#define _SIMULATED_TERESA_ROBOT_HPP_


#include "teresa_robot.hpp"
#include "timer.hpp"

namespace Teresa
{



class SimulatedRobot : public Robot
{
public:
	SimulatedRobot();
	virtual ~SimulatedRobot() {}
	virtual bool setVelocity(double linear, double angular);
	virtual bool isStopped();
	virtual bool getIMD(double& imdl, double& imdr);
	virtual bool incHeight();
	virtual bool decHeight();
	virtual bool incTilt();
	virtual bool decTilt();
	virtual bool setHeight(double height);
	virtual bool setTilt(double tilt);
	virtual bool getHeight(double& height) {height = SimulatedRobot::height; return true;}
	virtual bool getTilt(double& tilt) {tilt = SimulatedRobot::tilt; return true;}
	virtual bool getTemperature(int& leftMotor, 
			int& rightMotor, 
			int& leftDriver, 
			int& rightDriver, 
			bool& tiltDriverOverheat, 
			bool& heightDriverOverheat)
	{
		leftMotor=35;
		rightMotor=35;
		leftDriver=35;
		rightDriver=35;
		tiltDriverOverheat=false;
		heightDriverOverheat=false;
		return true;
	}
	virtual bool getBatteryStatus(unsigned char& elec_level, 
					unsigned char& PC1_level, 
					unsigned char& motorH_level, 
					unsigned char& motorL_level,
					unsigned char& charger_status)
	{
		elec_level=100;
		PC1_level=100;
		motorH_level=100;
		motorL_level=100;
		charger_status = 0x0F;
		return true;
	}
	virtual bool getButtons(bool& button1, bool& button2)
	{
		button1=false;
		button2=false;
		return true;
	}
	virtual bool getRotaryEncoder(int& rotaryEncoder)
	{
		rotaryEncoder = 0;
		return true;
	}
	virtual bool enableDCDC(unsigned char mask)
	{
		char buffer[32];
		sprintf(buffer,"DCDC mask: %02X",mask);
		std::cout<<buffer<<std::endl;
		return true;
	}
	virtual bool setLeds(const std::vector<unsigned char>& leds){std::cout<<"Set Leds"<<std::endl;return true;}
	virtual bool getPowerDiagnosis(PowerDiagnosis& diagnosis)
	{
		diagnosis.elec_bat_voltage = 0;
		diagnosis.PC1_bat_voltage = 0;
		diagnosis.cable_bat_voltage = 0;
		diagnosis.motor_voltage = 0;
		diagnosis.motor_h_voltage = 0;
		diagnosis.motor_l_voltage = 0;
		diagnosis.elec_instant_current = 0;
		diagnosis.motor_instant_current = 0;
		diagnosis.elec_integrated_current = 0;
		diagnosis.motor_integrated_current = 0; 
		return true;
	}	

private:
	double left_wheel_velocity;
	double right_wheel_velocity;
	double height;
        double tilt;
	double left_meters;
	double right_meters;
	double current_left_meters;
	double current_right_meters;
	bool is_stopped;
	utils::Timer timer;

};



inline
SimulatedRobot::SimulatedRobot()
: left_wheel_velocity(0),
  right_wheel_velocity(0),
  height(MAX_HEIGHT_MM),
  tilt(0), 
  left_meters(0),
  right_meters(0),
  current_left_meters(0),
  current_right_meters(0),
  is_stopped(true)
{
}

inline
bool SimulatedRobot::setVelocity(double linear, double angular)
{
	linear=saturateLinearVelocity(linear);
	angular=saturateAngularVelocity(angular);
	left_meters+= left_wheel_velocity * timer.elapsed();
	right_meters+= right_wheel_velocity * timer.elapsed();
	left_wheel_velocity = saturateLinearVelocity(linear - ROBOT_RADIUS_M*angular);
	right_wheel_velocity = saturateLinearVelocity(linear + ROBOT_RADIUS_M*angular);
	timer.init();
	is_stopped = std::abs(left_wheel_velocity)<= LINEAR_VELOCITY_ZERO_THRESHOLD && 
			std::abs(right_wheel_velocity)<=LINEAR_VELOCITY_ZERO_THRESHOLD;
	return true;
}	

inline
bool SimulatedRobot::isStopped()
{
	return is_stopped;
}

inline
bool SimulatedRobot::getIMD(double& imdl, double& imdr)
{
	left_meters+= left_wheel_velocity * timer.elapsed();
	right_meters+= right_wheel_velocity * timer.elapsed();
	timer.init();
	imdl = left_meters - current_left_meters;
	imdr = right_meters - current_right_meters;
	current_left_meters = left_meters;
	current_right_meters = right_meters;
	return true;
	
}

inline
bool SimulatedRobot::incHeight()
{
	height+=10;
	if (height>MAX_HEIGHT_MM) {
		height=MAX_HEIGHT_MM;
	}
	return true;
}

inline
bool SimulatedRobot::decHeight()
{
	height-=10;
	if (height<MIN_HEIGHT_MM) {
		height=MIN_HEIGHT_MM;
	}
	return true;
}

inline
bool SimulatedRobot::incTilt()
{
	tilt+=0.0349066;
	if (tilt>MAX_TILT_ANGLE_RADIANS) {
		tilt = MAX_TILT_ANGLE_RADIANS;
	}
	return true;
}

inline
bool SimulatedRobot::decTilt()
{
	tilt-=0.0349066;
	if (tilt<MIN_TILT_ANGLE_RADIANS) {
		tilt=MIN_TILT_ANGLE_RADIANS;
	}
	return true;
}

inline
bool SimulatedRobot::setHeight(double height)
{
	if (height<MIN_HEIGHT_MM) {
		height=MIN_HEIGHT_MM;
	} else if (height>MAX_HEIGHT_MM) {
		height=MAX_HEIGHT_MM;
	}
	SimulatedRobot::height = height;
	return true;
}

inline
bool SimulatedRobot::setTilt(double tilt)
{
	if (tilt < MIN_TILT_ANGLE_RADIANS) {
		tilt=MIN_TILT_ANGLE_RADIANS;
	} else if (tilt>MAX_TILT_ANGLE_RADIANS) {
		tilt=MAX_TILT_ANGLE_RADIANS;
	}
	SimulatedRobot::tilt = tilt;
	return true;
}

}

#endif
