/***********************************************************************/
/**                                                                    */
/** teresa_robot.hpp                                                   */
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

#ifndef _TERESA_ROBOT_HPP_
#define _TERESA_ROBOT_HPP_

#include <cmath>

namespace Teresa
{

#define MIN_HEIGHT_MM                         1210
#define MAX_HEIGHT_MM                         1425

#define MIN_TILT_ANGLE_DEGREES                 -37
#define MAX_TILT_ANGLE_DEGREES                 180

#define MIN_TILT_ANGLE_RADIANS           -0.645772 
#define MAX_TILT_ANGLE_RADIANS                M_PI

#define ROBOT_DIAMETER_M                      0.47
#define ROBOT_RADIUS_M                       0.235

#define MAX_LINEAR_VELOCITY                    0.6
#define MAX_ANGULAR_VELOCITY             1.5707963

#define LINEAR_VELOCITY_ZERO_THRESHOLD       0.001
#define ANGULAR_VELOCITY_ZERO_THRESHOLD       0.01



struct PowerDiagnosis
{
	double elec_bat_voltage;     // V
	double PC1_bat_voltage;      // V
	double cable_bat_voltage;    // V
	double motor_voltage;        // V
	double motor_h_voltage;      // V
	double motor_l_voltage;      // V

	int elec_instant_current;     // mA
	int motor_instant_current;    // mA
	int elec_integrated_current;  // mA
	int motor_integrated_current; // mA
};


/**
 * An interface for the Teresa Robot
 */
class Robot
{
public:
	Robot() {}
	virtual ~Robot() {}
        /**
	 * Set the velocity reference
	 *
	 * @param[in] linear the linear velocity reference in m/s
	 * @param[in] angular the angular velocity reference in rad/s
	 * @return true if success, false otherwise
	 */
	virtual bool setVelocity(double linear, double angular) = 0;
	/**
	 * Check if the robot is stopped
	 *
	 * @return true if robot is stopped, false otherwise
	 */ 
	virtual bool isStopped() = 0;
	/**
	 * Get the distance traveled by each wheel since the last call to this function
	 *
	 * @param[out] imdl the distance traveled by the left wheel in meters
	 * @param[out] imdr the distance traveled by the right wheel in meters
	 * @return true if success, false otherwise
	 */ 
	virtual bool getIMD(double& imdl, double& imdr) = 0;
	/**
	 * Increment the height by 10mm
         * @return true if success, false otherwise
	 */
	virtual bool incHeight() = 0;
	/**
	 * Decrement the height by 10mm
         * @return true if success, false otherwise
	 */
	virtual bool decHeight() = 0;
	/**
	 * Increment the tilt angle by 2 degrees
         * @return true if success, false otherwise
	 */
	virtual bool incTilt() = 0;
	/**
	 * Decrement the tilt angle by 2 degrees
         * @return true if success, false otherwise
	 */
	virtual bool decTilt() = 0;
	/**
	 * Set the height
	 *
	 * @param[in] height the height in millimeters
         * @return true if success, false otherwise
	 */
	virtual bool setHeight(double height) = 0;
	/**
	 * Set the tilt angle
	 *
	 * @param[in] tilt the tilt angle in radians
         * @return true if success, false otherwise
	 */
	virtual bool setTilt(double tilt) = 0;
	/**
	 * Get the height
	 *
	 * height[out] the height in millimeters
	 * @return true if success, false otherwise
	 */
	virtual bool getHeight(double& height) = 0;
	/**
	 * Get the tilt angle
	 *
	 * tilt[out] the tilt angle in radians
	 * @return true if success, false otherwise
	 */
	virtual bool getTilt(double& tilt) = 0;
	/**
	 * Check if some button has been pressed
	 *
	 * @param[out] button1 true if button1 has been pressed since the last read, false otherwise
	 * @param[out] button2 true if button2 has been pressed since the last read, false otherwise
         * @return true if success, false otherwise
	 */
	virtual bool getButtons(bool& button1, bool& button2) = 0;
	/**
	 * Get the encoder steps positive or negative counted since the last reading
	 *
	 * @param[out] rotaryEncoder the encoder steps since the last reading
	 * @return true if success, false otherwise	
	 */
	virtual bool getRotaryEncoder(int& rotaryEncoder) = 0;	

	/**
	 * Get temperature information
	 * 
	 * @param[out] leftMotor temperature of the left motor in celsius degrees
	 * @param[out] rightMotor temperature of the right motor in celsius degrees
	 * @param[out] leftDriver temperature of the left driver in celsius degrees
	 * @param[out] rightDriver temperature of the right driver in celsius degrees
	 * @param[out] tiltDriverOverheat true if tilt driver is overheat, false otherwise
	 * @param[out] heightDriverOverheat true if height driver is overheat, false otherwise
	 * @return true if success, false otherwise
	 */
	virtual bool getTemperature(int& leftMotor, 
					int& rightMotor, 
					int& leftDriver, 
					int& rightDriver, 
					bool& tiltDriverOverheat, 
					bool& heightDriverOverheat) = 0;

	/**
	 * enable/disable DCDC outputs
	 *
	 * @param[in] mask binary mask [HGFEDCBA] where:
	 *             A = 1 Enable RD0 5V output
         *             A = 0 Disable RD0 5V output
         *             B = 1 Enable RD1 5V output
         *             B = 0 Disable RD1 5V output
         *             C = 1 Enable RD2 12V output
         *             C = 0 Disable RD2 12V output
         *             D = 1 Enable RD3 12V output
         *             D = 0 Disable RD3 12V output
         *             E = 1 Enable RD4 12V output
         *             E = 0 Disable RD4 12V output
         *             F = 1 Enable RD5 12V output
         *             F = 0 Disable RD5 12V output
         *             G = 1 Enable RD6 12V output
         *             G = 0 Disable RD6 12V output
         *             H = 1 Enable RD7 12V output
         *             H = 0 Disable RD7 12V output
	 * @return true if success, false otherwise
	 */
	virtual bool enableDCDC(unsigned char mask) = 0;
	/**
	 * set RGB leds
	 *
	 * @param[in] leds array of desired RGB values R0,G0,B0,R1,G1,B1,...,Rn,Gn,Bn
	 * @return true if success, false otherwise
	 */
	virtual bool setLeds(const std::vector<unsigned char>& leds) = 0;
	/**
	 * get Battery status
	 *
	 * @param[out] elec_level percentage of power available in electronic battery [0,100]%
	 * @param[out] PC1_level percentage of power available in PC1 battery [0,100]%
	 * @param[out] motorH_level percentage of power available in motorH battery [0,100]%
	 * @param[out] motorL_level percentage of power available in motorL battery [0,100]%
	 * @param[out] charger_status [XXXXDCBA]
	 * 		A = 1 Charger 1 charge ended
	 *		A = 0 Charger 1 charge ongoing
	 *		B = 1 Charger 2 charge ended
	 *		B = 0 Charger 2 charge ongoing
	 *		C = 1 Charger 3 charge ended
	 *		C = 0 Charger 3 charge ongoing
	 *		D = 1 Charger 4 charge ended
	 *		D = 0 Charger 4 charge ongoing
	 * @return true if success, false otherwise
	 */
	virtual bool getBatteryStatus(unsigned char& elec_level, 
					unsigned char& PC1_level, 
					unsigned char& motorH_level, 
					unsigned char& motorL_level, 
					unsigned char& charger_status) = 0;


	/**
	 * get Power diagnosis
	 *
	 * @param[out] diagnosis struct of PowerDiagnosis with information
	 * @return true if success, false otherwise
	 */
	virtual bool getPowerDiagnosis(PowerDiagnosis& diagnosis) = 0;
protected:
	/**
	 * Saturate a linear velocity value
	 * 
	 * Note: See function double saturate(double v, double max, double zero) below
	 * @param[in] v the linear velocity to saturate in m/s
	 * @return the saturated linear velocity in m/s
	 */
	static double saturateLinearVelocity(double v);
	/**
	 * Saturate an angular velocity value
	 * 
	 * Note: See function double saturate(double v, double max, double zero) below
	 * @param[in] v the angular velocity to saturate in rad/s
	 * @return the saturated angular velocity in rad/s
	 */
	static double saturateAngularVelocity(double v);	

private:
	/**
	 * Saturate a velocity value
	 *
	 * The returned value will be 0.0 if std::abs(v)<=zero_threshold
	 * The returned value will be max_value if v>max_value
	 * The returned value will be -max_value if v<-max_value 
	 * @param[in] v the velocity to saturate
	 * @param[in] max_value the maximum velocity value
	 * @param[in] zero_threshold the zero threshold
	 * @return The saturated velocity
	 */
	static double saturate(double v, double max_value, double zero_threshold);
};

inline
double Robot::saturateLinearVelocity(double v)
{
	return saturate(v,MAX_LINEAR_VELOCITY,LINEAR_VELOCITY_ZERO_THRESHOLD);
}

inline
double Robot::saturateAngularVelocity(double v)
{
	return saturate(v,MAX_ANGULAR_VELOCITY,ANGULAR_VELOCITY_ZERO_THRESHOLD);
}	

inline
double Robot::saturate(double v, double max_value, double zero_threshold)
{
	if (std::abs(v)<=zero_threshold) {
		v = 0.0;
	}
	else if (v>max_value) {
		v = max_value;	
	}
	else if (v < -max_value) {
		v = -max_value;
	}	
	return v;
}


}

#endif
