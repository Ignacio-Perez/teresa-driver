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

/**
 * An implementation of Teresa::Robot for debugging and testing
 */
class SimulatedRobot : public Robot
{
public:
	SimulatedRobot();
	virtual ~SimulatedRobot() {}
	virtual bool setVelocity(double linear, double angular);
	virtual bool setVelocityRaw(int16_t leftWheelRef, int16_t rightWheelRef) {return true;}
	virtual bool isStopped();
	virtual bool getIMD(double& imdl, double& imdr);
	virtual bool setHeightVelocity(int velocity) {return true;}
	virtual bool setTiltVelocity(int velocity) {return true;}
	virtual bool setHeight(int height);
	virtual bool setTilt(int tilt);
	virtual bool getHeight(int& height) {height = SimulatedRobot::height; return true;}
	virtual bool getTilt(int& tilt) {tilt = SimulatedRobot::tilt; return true;}
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
		dcdc_mask=mask;
		sprintf(buffer,"DCDC mask: %02X",mask);
		std::cout<<buffer<<std::endl;
		return true;
	}
	virtual bool getDCDC(unsigned char& mask)
	{
		mask=dcdc_mask;
		return true;
	}
	virtual bool setLeds(const std::vector<unsigned char>& leds){std::cout<<"Set Leds"<<std::endl;return true;}
	virtual bool getPowerDiagnostics(PowerDiagnostics& diagnostics)
	{
		diagnostics.elec_bat_voltage = 0;
		diagnostics.PC1_bat_voltage = 0;
		diagnostics.cable_bat_voltage = 0;
		diagnostics.motor_voltage = 0;
		diagnostics.motor_h_voltage = 0;
		diagnostics.motor_l_voltage = 0;
		diagnostics.elec_instant_current = 0;
		diagnostics.motor_instant_current = 0;
		diagnostics.elec_integrated_current = 0;
		diagnostics.motor_integrated_current = 0; 
		return true;
	}	

private:
	double left_wheel_velocity;
	double right_wheel_velocity;
	int height;
        int tilt;
	double left_meters;
	double right_meters;
	double current_left_meters;
	double current_right_meters;
	bool is_stopped;
	unsigned char dcdc_mask;	
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
  is_stopped(true),
  dcdc_mask(0xFF)
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
bool SimulatedRobot::setHeight(int height)
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
bool SimulatedRobot::setTilt(int tilt)
{
	if (tilt < MIN_TILT_ANGLE_DEGREES) {
		tilt=MIN_TILT_ANGLE_DEGREES;
	} else if (tilt>MAX_TILT_ANGLE_DEGREES) {
		tilt=MAX_TILT_ANGLE_DEGREES;
	}
	SimulatedRobot::tilt = tilt;
	return true;
}

}

#endif
