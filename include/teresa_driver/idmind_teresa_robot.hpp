/***********************************************************************/
/**                                                                    */
/** idmind_teresa_robot.hpp                                            */
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

#ifndef _IDMIND_TERESA_ROBOT_HPP_
#define _IDMIND_TERESA_ROBOT_HPP_

#include <iostream>
#include <vector>
#include "teresa_robot.hpp"
#include "serial_interface.hpp"
#include "timer.hpp"


namespace Teresa
{

#define WRITTING_TRIES                  5

// BOARD1 COMMANDS
#define SET_NUMBER_RGB_LEDS          0x35
#define SET_RGB_LEDS_VALUES          0x36
#define SET_ENABLE_DCDC_OUTPUT       0x37
#define GET_POWER_VOLTAGE            0x50
#define GET_POWER_CURRENT            0x51
#define GET_BATTERIES_LEVEL          0x52
#define GET_CHARGER_STATUS           0x53

// BOARD2 COMMANDS
#define SET_MOTOR_VELOCITY           0x30
#define SET_TILT_POSITION_DEGREES    0x31
#define SET_HEIGHT_POSITION_MM       0x34
#define SET_FANS                     0x37
#define SET_CALIBRATION              0x38
#define SET_TILT_DRIVER_STATE        0x39
#define SET_HEIGHT_DRIVER_STATE      0x3A

#define GET_ARCADE_BUTTONS           0x51
#define GET_ROTARY_ENCODER           0x52
#define GET_MOTOR_VELOCITY_TICKS     0x57
#define GET_TILT_ACTUAL_POSITION     0x58
#define GET_HEIGHT_ACTUAL_POSITION   0x59
#define GET_TEMPERATURE_SENSORS      0x5A
#define GET_TILT_STATUS              0x5B
#define GET_HEIGHT_STATUS            0x5C

// BOARD1 & BOARD2 COMMANDS
#define GET_FIRMWARE_VERSION_NUMBER  0x20


class IdMindBoard
{
public:
	IdMindBoard(const std::string& device, const std::string& name,
			void (*printInfo)(const std::string& message),
			void (*printError)(const std::string& message));
	~IdMindBoard();
	bool open();
	bool communicate(int command_size, int response_size);
	const std::string& getName() const {return name;}
	unsigned char command[512];
	unsigned char response[512];
private:
	bool checksum(int response_size);
	bool flush();
	utils::SerialInterface board;
	std::string name;
	void (*printInfo)(const std::string& message);
	void (*printError)(const std::string& message);
	int counter;	
	
};


class IdMindRobot : public Robot
{
public:
	IdMindRobot(const std::string& board1,
			const std::string& board2,
                        unsigned char initial_dcdc_mask,
			unsigned char final_dcdc_mask,
			unsigned char number_of_leds,
			void (*printInfo)(const std::string& message) = defaultPrint,
			void (*printError)(const std::string& message) = defaultPrint);
			
 
	virtual ~IdMindRobot();
	virtual bool setVelocity(double linear, double angular);
	virtual bool isStopped();
	virtual bool getIMD(double& imdl, double& imdr);
	virtual bool incHeight();
	virtual bool decHeight();
	virtual bool incTilt();
	virtual bool decTilt();
	virtual bool setHeight(double height);
	virtual bool setTilt(double tilt);
	virtual bool getHeight(double& height);
	virtual bool getTilt(double& tilt);
	virtual bool getButtons(bool& button1, bool& button2);
	virtual bool getRotaryEncoder(int& rotaryEncoder);
	virtual bool getTemperature(int& leftMotor, 
					int& rightMotor, 
					int& leftDriver, 
					int& rightDriver, 
					bool& tiltDriverOverheat, 
					bool& heightDriverOverheat);
	virtual bool enableDCDC(unsigned char mask);
	virtual bool setLeds(const std::vector<unsigned char>& leds);
	virtual bool getBatteryStatus(unsigned char& elec_level, 
					unsigned char& PC1_level, 
					unsigned char& motorH_level, 
					unsigned char& motorL_level, 
					unsigned char& charger_status);
	virtual bool getPowerDiagnostics(PowerDiagnostics& diagnostics);
private:
	static int16_t bufferToInt(const unsigned char* buffer);
	static uint16_t bufferToUnsignedInt(const unsigned char* buffer);

	bool setFans(bool fans);
	bool enableTiltMotor(bool enable);
	bool enableHeightMotor(bool enable);
	bool calibrate(bool calibrate_tilt_system, bool calibrate_height_system);
	bool setNumberOfLeds(unsigned char number_of_leds);	
	static void defaultPrint(const std::string& message){std::cout<<message<<std::endl;}

	IdMindBoard board1;
	IdMindBoard board2;
	unsigned char number_of_leds;

	void (*printInfo)(const std::string& message);
	void (*printError)(const std::string& message);

	bool is_stopped;
	int final_dcdc_mask;

};


inline
IdMindBoard::IdMindBoard(const std::string& device, const std::string& name,
		void (*printInfo)(const std::string& message),
		void (*printError)(const std::string& message))
: board(device,false),
  name(name),
  printInfo(printInfo),
  printError(printError),
  counter(-1){}

inline
IdMindBoard::~IdMindBoard()
{
	board.close();
}

inline
bool IdMindBoard::open()
{
	if (!board.open()) {
		printError("Cannot open "+name+" in device "+board.getDeviceName());
		return false;
	}
	printInfo(name+" device: "+board.getDeviceName());
	command[0] = GET_FIRMWARE_VERSION_NUMBER;
	if (!communicate(1,29)) {
		printError("Cannot get firmware version from "+name);
		return false;
	}
	std::string version((const char*)response+1,25);
	printInfo(name+" firmware version: "+version);
	return true;
}

inline
bool IdMindBoard::checksum(int response_size)
{
	uint16_t checksum1 = (int)response[response_size-2];
	checksum1 <<= 8; 
	checksum1 += (int)response[response_size-1];
	uint16_t checksum2=0;
	for(int i=0; i<response_size-2; i++)	{
		checksum2 += (int)response[i];
   	}
	return checksum1 == checksum2;	
}

inline
bool IdMindBoard::flush()
{
	int bytes;
	if (!board.incomingBytes(bytes)) {
		printError("Cannot flush");
		return false;
	}
	while(bytes>0) {
		int aux = board.read(response,bytes);
		if (aux==-1) {
			printError("Cannot flush, reading error");
			return false;
		}
		bytes-=aux;
	}
	return true;
}


inline
bool IdMindBoard::communicate(int command_size, int response_size)
{
	flush();
	int i=0;
	while(i<WRITTING_TRIES && !board.write(command,command_size)) {
		printError("Cannot write to "+name);
		i++;
	}
	if (i==WRITTING_TRIES) {
		printError("Communication with "+name+ " aborted due to maximum writting tries reached");
		return false;
	}
	usleep(1);
	int read_bytes=0;
	int bytes;
	utils::Timer timer;
	bool counting_timeout=false;
	while (read_bytes<response_size) {
		if (!board.incomingBytes(bytes)) {
			printError("Communication with "+name+ " aborted due to reading error (cannot get incoming bytes)");
			return false;
		}
		if (bytes==0) {
			if (!counting_timeout) {
				counting_timeout=true;				
				timer.init();
			} else if (timer.elapsed()>0.05) {
				printError("Communication with "+name+ " aborted due to reading timeout");
				return false;
			}
			usleep(1);
		} else {
			counting_timeout=false;
			bytes = std::min(bytes,response_size-read_bytes);
			int aux = board.read(response+read_bytes,bytes);
			if (aux==-1) {
				printError("Communication with "+name+ " aborted due to reading error");
				return false;
			}
			read_bytes += aux;
		}
	}	
	
	if (response[0] != command[0]) {
		printError("Invalid response header from "+name);
		return false;
	}
	if (counter==-1) {
		counter = response[response_size-3]; 
	} else {
		counter++;
		if (counter==256) {
			counter=0;
		}
	}
	if (counter != response[response_size-3]) {
		printError("Invalid response counter from "+name);
		return false;
	}
	if (!checksum(response_size)) {
		printError("Invalid checksum from "+name);
		return false;
	}
	return true;
}

inline
IdMindRobot::IdMindRobot(const std::string& board1,const std::string& board2,
				unsigned char initial_dcdc_mask,
				unsigned char final_dcdc_mask,
				unsigned char number_of_leds,
				void (*printInfo)(const std::string& message),
				void (*printError)(const std::string& message))
: board1(board1,"board1",printInfo,printError),
  board2(board2,"board2",printInfo,printError),
  number_of_leds(number_of_leds),
  printInfo(printInfo),
  printError(printError),
  is_stopped(true),
  final_dcdc_mask(final_dcdc_mask)
{
	bool board1_open = IdMindRobot::board1.open();
	bool board2_open = IdMindRobot::board2.open();
	
	if (!board1_open || 
		!board2_open || 
		!enableDCDC(initial_dcdc_mask) ||
		!setFans(true) ||
		!enableTiltMotor(true) ||
		!enableHeightMotor(true) ||
		!calibrate(true,true) ||
		!setNumberOfLeds(number_of_leds)) {
		throw ("Teresa initialization aborted");
	}

}

inline
IdMindRobot::~IdMindRobot()
{
	enableDCDC(final_dcdc_mask);
}


inline
int16_t IdMindRobot::bufferToInt(const unsigned char* buffer)
{
	int16_t value = (int16_t)buffer[0];
	value <<= 8;
	value |= (int16_t)buffer[1];
	return value;
}

inline
uint16_t IdMindRobot::bufferToUnsignedInt(const unsigned char* buffer)
{
	uint16_t value = (uint16_t)buffer[0];
	value <<= 8;
	value |= (uint16_t)buffer[1];
	return value;
}


inline
bool IdMindRobot::setFans(bool fans)
{
	board2.command[0] = SET_FANS;
	board2.command[1] = fans ? 0x01 : 0x00;
	if (!board2.communicate(2,4)) {
		if (fans) {
			printError("Cannot enable fans");
		} else {
			printError("Cannot disable fans");
		}
		return false;
	}
	return true;
}

inline
bool IdMindRobot::setNumberOfLeds(unsigned char number_of_leds)
{
	if (number_of_leds>84) {
		printError("Too many RGB leds (max. is 84)");
		return false;
	}
	board1.command[0] = SET_NUMBER_RGB_LEDS;
	board1.command[1] = number_of_leds;
	if (!board1.communicate(2,4)) {
		printError("Cannot set the number of RGB leds");
		return false;	
	}
	return true;
}

inline
bool IdMindRobot::enableTiltMotor(bool enable)
{
	board2.command[0] = SET_TILT_DRIVER_STATE;
	board2.command[1] = enable ? 0x01 : 0x00;
	if (!board2.communicate(2,4)) {
		if (enable) {
			printError("Cannot enable tilt motor");
		} else {
			printError("Cannot disable tilt motor");
		}
		return false;
	}
	return true;
}

inline
bool IdMindRobot::enableHeightMotor(bool enable)
{
	board2.command[0] = SET_HEIGHT_DRIVER_STATE;
	board2.command[1] = enable ? 0x01 : 0x00;
	if (!board2.communicate(2,4)) {
		if (enable) {
			printError("Cannot enable height motor");
		} else {
			printError("Cannot disable height motor");
		}
		return false;
	}
	return true;
}

inline
bool IdMindRobot::calibrate(bool calibrate_tilt_system, bool calibrate_height_system)
{
	board2.command[0] = SET_CALIBRATION;
	board2.command[1] = calibrate_tilt_system ? 0x01 : 0x00;
	if (calibrate_height_system) {
		board2.command[1] |= 0x02;
	}
	if (!board2.communicate(2,4)) {
		printError("Cannot calibrate system");
		return false;
	} 
	return true;
}

inline
bool IdMindRobot::setVelocity(double linear, double angular)
{
	linear=saturateLinearVelocity(linear);
	angular=saturateAngularVelocity(angular);
	double left_wheel_velocity = saturateLinearVelocity(linear - ROBOT_RADIUS_M*angular);
	double right_wheel_velocity = saturateLinearVelocity(linear + ROBOT_RADIUS_M*angular);
	int16_t v_left=0;
	int16_t v_right=0;
	if (left_wheel_velocity > LINEAR_VELOCITY_ZERO_THRESHOLD || left_wheel_velocity < -LINEAR_VELOCITY_ZERO_THRESHOLD) {
		v_left = -(int16_t)std::round(left_wheel_velocity * 210.0 + 8.35);
	}
	if (right_wheel_velocity > LINEAR_VELOCITY_ZERO_THRESHOLD || right_wheel_velocity < -LINEAR_VELOCITY_ZERO_THRESHOLD) {
		v_right = (int16_t)std::round(right_wheel_velocity * 210.0 + 8.35);
	}
	board2.command[0] = SET_MOTOR_VELOCITY;
	board2.command[1] = (unsigned char)(v_left >> 8);
	board2.command[2] = (unsigned char)(v_left & 0xFF);	
	board2.command[3] = (unsigned char)(v_right >> 8);
	board2.command[4] = (unsigned char)(v_right & 0xFF);
	if (!board2.communicate(5,4)) {
		printError("Cannot set velocity");
		return false;
	}
	return true;
}

inline
bool IdMindRobot::isStopped()
{
	return is_stopped;
}

inline
bool IdMindRobot::getIMD(double& imdl, double& imdr)
{
	board2.command[0] = GET_MOTOR_VELOCITY_TICKS;
	if (!board2.communicate(1,8)) {
		printError("Cannot get motor velocity ticks");
		return false;
	}
	int16_t inc_left = bufferToInt(board2.response+1);
	int16_t inc_right = bufferToInt(board2.response+3);
	imdl = inc_left==0?0:(double)inc_left*0.00024802;
	imdr = inc_right==0?0:(double)inc_right*0.00024802;
	is_stopped = inc_left==0 && inc_right==0;
	return true;
}

inline
bool IdMindRobot::incHeight()
{ /*
	board2.command[0] = GET_HEIGHT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot get height position");
		return false;
	}
	int16_t height_mm = bufferToInt(board2.response+1);
	if (height_mm==MAX_HEIGHT_MM) {
		return true;
	}
	height_mm = std::min(MAX_HEIGHT_MM,height_mm+10);
	board2.command[0] = SET_HEIGHT_POSITION_MM;
	board2.command[1] = (unsigned char)(height_mm >> 8);
	board2.command[2] = (unsigned char)(height_mm & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot increment height");
		return false;
	}
	return true;
	*/
	
	
	return setHeight(MAX_HEIGHT_MM);
}


inline
bool IdMindRobot::decHeight()
{
	/*
	board2.command[0] = GET_HEIGHT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot get height position");
		return false;
	}
	int16_t height_mm = bufferToInt(board2.response+1);
	if (height_mm==MIN_HEIGHT_MM) {
		return true;
	}
	height_mm = std::max(MIN_HEIGHT_MM,height_mm-10);
	board2.command[0] = SET_HEIGHT_POSITION_MM;
	board2.command[1] = (unsigned char)(height_mm >> 8);
	board2.command[2] = (unsigned char)(height_mm & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot decrement height");
		return false;
	}
	return true;
	*/
	int vref = -100;
	board2.command[0] = 0x36;
	board2.command[1] = (unsigned char)(vref<<8);
	board2.command[2] = (unsigned char)(vref&0xff);
	board2.communicate(3,4);
	return true;
	//return setHeight(MIN_HEIGHT_MM);
}

inline
bool IdMindRobot::incTilt()
{
	board2.command[0] = GET_TILT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot get tilt angle");
		return false;
	}
	int16_t tilt_degrees = bufferToInt(board2.response+1);
	if (tilt_degrees==MAX_TILT_ANGLE_DEGREES) {
		return true;
	}
	tilt_degrees = std::min(MAX_TILT_ANGLE_DEGREES,tilt_degrees+2);
	board2.command[0] = SET_TILT_POSITION_DEGREES;
	board2.command[1] = (unsigned char)(tilt_degrees >> 8);
	board2.command[2] = (unsigned char)(tilt_degrees & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot increment tilt angle");
		return false;
	}
	return true;
}

inline
bool IdMindRobot::decTilt()
{
	board2.command[0] = GET_TILT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot get tilt angle");
		return false;
	}
	int16_t tilt_degrees = bufferToInt(board2.response+1);
	if (tilt_degrees==MIN_TILT_ANGLE_DEGREES) {
		return true;
	}
	tilt_degrees = std::max(MIN_TILT_ANGLE_DEGREES,tilt_degrees-2);
	board2.command[0] = SET_TILT_POSITION_DEGREES;
	board2.command[1] = (unsigned char)(tilt_degrees >> 8);
	board2.command[2] = (unsigned char)(tilt_degrees & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot decrement tilt angle");
		return false;
	}
	return true;
}

inline
bool IdMindRobot::setHeight(double height)
{
	int16_t height_ref = (int16_t)std::round(height);
	if (height_ref<MIN_HEIGHT_MM) {
		height_ref=MIN_HEIGHT_MM;
	} else if (height_ref>MAX_HEIGHT_MM) {
		height_ref=MAX_HEIGHT_MM;
	}
	board2.command[0] = SET_HEIGHT_POSITION_MM;
	board2.command[1] = (unsigned char)(height_ref >> 8);
	board2.command[2] = (unsigned char)(height_ref & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot set height");
		return false;
	}
	return true;
	
}

inline
bool IdMindRobot::setTilt(double tilt)
{
	int16_t tilt_ref = (int16_t)std::round(tilt * 57.2958);
	if (tilt_ref<MIN_TILT_ANGLE_DEGREES) {
		tilt_ref=MIN_TILT_ANGLE_DEGREES;
	} else if (tilt_ref>MAX_TILT_ANGLE_DEGREES) {
		tilt_ref=MAX_TILT_ANGLE_DEGREES;
	}
	board2.command[0] = SET_TILT_POSITION_DEGREES;
	board2.command[1] = (unsigned char)(tilt_ref >> 8);
	board2.command[2] = (unsigned char)(tilt_ref & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot set tilt angle");
		return false;
	}
	return true;
}

inline
bool IdMindRobot::getHeight(double& height)
{
	board2.command[0] = GET_HEIGHT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot get height");
		return false;
	}
	height = (double)bufferToInt(board2.response+1);
	return true;
}

inline
bool IdMindRobot::getTilt(double& tilt)
{
	board2.command[0] = GET_TILT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot set tilt angle");
		return false;
	}
	tilt = (double)bufferToInt(board2.response+1) * 0.0174533;
	return true;
}

inline
bool IdMindRobot::getButtons(bool& button1, bool& button2)
{
	board2.command[0] = GET_ARCADE_BUTTONS;
	if (!board2.communicate(1,5)) {
		printError("Cannot get buttons");
		return false;
	}
	button1 = board2.response[1]&0x01;
	button2 = board2.response[1]&0x02;
	return true;	
}

inline
bool IdMindRobot::getRotaryEncoder(int& rotaryEncoder)
{
	board2.command[0] = GET_ROTARY_ENCODER;
	if (!board2.communicate(1,5)) {
		printError("Cannot get rotary encoder");
		return false;
	}
	rotaryEncoder = (int8_t)board2.response[1];
	return true;
}

inline
bool IdMindRobot::getTemperature(int& leftMotor, 
					int& rightMotor, 
					int& leftDriver, 
					int& rightDriver, 
					bool& tiltDriverOverheat, 
					bool& heightDriverOverheat)
{
	board2.command[0] = GET_TEMPERATURE_SENSORS;
	if (!board2.communicate(1,9)) {
		printError("Cannot get temperature sensors");
		return false;
	}
	leftMotor = (int8_t)board2.response[1];
	rightMotor = (int8_t)board2.response[2];
	leftDriver = (int8_t)board2.response[3];
	rightDriver = (int8_t)board2.response[4];
		
	board2.command[0] = GET_TILT_STATUS;
	if (!board2.communicate(1,5)) {
		printError("Cannot get tilt status");
		return false;
	}
	tiltDriverOverheat = board2.response[1]&0x80;

	board2.command[0] = GET_HEIGHT_STATUS;
	if (!board2.communicate(1,5)) {
		printError("Cannot get height status");
		return false;
	}
	heightDriverOverheat = board2.response[1]&0x80;
	return true;
}

inline
bool IdMindRobot::enableDCDC(unsigned char mask)
{
	board1.command[0] = SET_ENABLE_DCDC_OUTPUT;
	board1.command[1] = mask;
	if (!board1.communicate(2,4)) {
		printError("Cannot set DCDC outputs");
		return false;
	}
	char buffer[32];
	sprintf(buffer,"DCDC mask: %02X",mask);
	printInfo(buffer);
	return true;
}

inline
bool IdMindRobot::setLeds(const std::vector<unsigned char>& leds)
{
	if (leds.size() != number_of_leds*3) {
		printError("Invalid number of RGB values");
		return false;
	}
	board1.command[0] = SET_RGB_LEDS_VALUES;
	for (unsigned i=0;i<leds.size();i++) {
		board1.command[i+1] = leds[i];
	}
	if (!board1.communicate(number_of_leds*3+1,4)) {
		printError("Cannot set RGB led values");
		return false;
	}
	return true;
}

inline
bool IdMindRobot::getBatteryStatus(unsigned char& elec_level, 
					unsigned char& PC1_level, 
					unsigned char& motorH_level, 
					unsigned char& motorL_level, 
					unsigned char& charger_status)
{
	board1.command[0] = GET_BATTERIES_LEVEL;
	if (!board1.communicate(1,8)) {
		printError("Cannot get batteries level");
		return false;
	}
	elec_level = board1.response[1];
	PC1_level = board1.response[2];
	motorH_level = board1.response[3];
	motorL_level = board1.response[4]; 	
	board1.command[0] = GET_CHARGER_STATUS;
	if (!board1.communicate(1,5)) {
		printError("Cannor get charger status");
		return false;
	}	
	charger_status = board1.response[1];
	return true;
}

inline
bool IdMindRobot::getPowerDiagnostics(PowerDiagnostics& diagnostics)
{
	board1.command[0]=GET_POWER_VOLTAGE;
	if (!board1.communicate(1,11)) {
		printError("Cannot get power voltage information");
		return false;
	}

	diagnostics.elec_bat_voltage = (double)board1.response[1]/10.0;
	diagnostics.PC1_bat_voltage = (double)board1.response[2]/10.0;
	diagnostics.cable_bat_voltage = (double)board1.response[3]/10.0;
	diagnostics.motor_voltage = (double)bufferToUnsignedInt(board1.response+4)/10.0;
	diagnostics.motor_h_voltage = (double)board1.response[6]/10.0;
	diagnostics.motor_l_voltage = (double)board1.response[7]/10.0;

	board1.command[0]=GET_POWER_CURRENT;
	if (!board1.communicate(1,12)) {
		printError("Cannot get power current information");
		return false;
	}
	diagnostics.elec_instant_current = bufferToUnsignedInt(board1.response+1);
	diagnostics.motor_instant_current = bufferToUnsignedInt(board1.response+3);
	diagnostics.elec_integrated_current = bufferToUnsignedInt(board1.response+5);
	diagnostics.motor_integrated_current = bufferToUnsignedInt(board1.response+7);
	return true;
}	


}

#endif
