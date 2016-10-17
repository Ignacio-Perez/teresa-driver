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

struct Calibration
{
	double A_left;
	double B_left;
	double A_right;
	double B_right;
	bool inverse_left_motor;
	bool inverse_right_motor;
};


#define WRITTING_TRIES                  5

// BOARD1 COMMANDS
#define SET_NUMBER_RGB_LEDS          0x35
#define SET_RGB_LEDS_VALUES          0x36
#define SET_ENABLE_DCDC_OUTPUT       0x37
#define GET_POWER_VOLTAGE            0x50
#define GET_POWER_CURRENT            0x51
#define GET_BATTERIES_LEVEL          0x52
#define GET_CHARGER_STATUS           0x53
#define GET_ENABLE_DCDC_OUTPUT       0x57

// BOARD2 COMMANDS
#define SET_MOTOR_VELOCITY           0x30
#define SET_TILT_POSITION_DEGREES    0x31
#define SET_TILT_VELOCITY            0x33
#define SET_HEIGHT_POSITION_MM       0x34
#define SET_HEIGHT_VELOCITY          0x36
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
#define GET_HEIGHT_DRIVER_STATE      0x5E

// BOARD1 & BOARD2 COMMANDS
#define GET_FIRMWARE_VERSION_NUMBER  0x20

/**
 * Generic IdMind board
 */
class IdMindBoard
{
public:
	/**
	 * Constructor
	 *
	 * @param device to connect (i.e. /dev/ttyUSB0)
	 * @param name to show in messages (i.e. board2)
	 * @param printInfo function to print information messages
	 * @param printError function to print error messages
	 */
	IdMindBoard(const std::string& device, const std::string& name,
			void (*printInfo)(const std::string& message),
			void (*printError)(const std::string& message));
	~IdMindBoard();
	/**
	 * Open device
	 *
	 * @return true if success, false otherwise
	 */
	bool open();
	/**
	 * Communicate with board, including error management
	 *
	 * @param command_size number of bytes of the command to send
	 * @param response_size number of bytes of the response to read
	 * @precondition the command buffer (see below) should contain the data to send
	 * @return true if success, false otherwise
	 */
	bool communicate(int command_size, int response_size);
	/**
	 * Get the name of the board
	 *
	 * @return the name of the board
	 */
	const std::string& getName() const {return name;}
	
	unsigned char command[512]; // Command buffer
	unsigned char response[512]; // Response buffer
private:
	bool checksum(int response_size); // Checksum function
	bool flush(); // Flush incoming bytes
	utils::SerialInterface board; // Serial interface for communications
	std::string name; // Name of the board
	void (*printInfo)(const std::string& message); // Function to print Information
	void (*printError)(const std::string& message);  // Function to print Errors
	int counter; // Message counter (from 0 to 255)	
};

/**
 * The IdMind Teresa Robot interface
 */
class IdMindRobot : public Robot
{
public:
	/**
	 * Constructor
	 *
	 * @param board1 device of the board1 (i.e. /dev/ttyUSB0)
	 * @param board2 device of the board2 (i.e. /dev/ttyUSB1)
	 * @param calibration the wheel calibration parameter
	 * @param initial_dcdc_mask initial mask for DCDC to be set now (see IdMind documentation)
	 * @param final_dcdc_mask final mask for DCDC, to be set in the destructor
	 * @param printInfo function to print information 
	 * @param printError function to print errors
	 */
	IdMindRobot(const std::string& board1,
			const std::string& board2,
			const Calibration& calibration,
                        unsigned char initial_dcdc_mask,
			unsigned char final_dcdc_mask,
			unsigned char number_of_leds,
			void (*printInfo)(const std::string& message) = defaultPrint,
			void (*printError)(const std::string& message) = defaultPrint);
			
	virtual ~IdMindRobot();
	// Implementation of inherited virtual functions (robot interface)
	virtual bool setVelocity(double linear, double angular);
	virtual bool setVelocity2(double linear, double angular);
	virtual bool setVelocityRaw(int16_t leftWheelRef, int16_t rightWheelRef);
	virtual bool isStopped();
	virtual bool getIMD(double& imdl, double& imdr);
	virtual bool setHeightVelocity(int velocity);
	virtual bool setTiltVelocity(int velocity);
	virtual bool setHeight(int height);
	virtual bool setTilt(int tilt);
	virtual bool getHeight(int& height);
	virtual bool getTilt(int& tilt);
	virtual bool getButtons(bool& button1, bool& button2);
	virtual bool getRotaryEncoder(int& rotaryEncoder);
	virtual bool getTemperature(int& leftMotor, 
					int& rightMotor, 
					int& leftDriver, 
					int& rightDriver, 
					bool& tiltDriverOverheat, 
					bool& heightDriverOverheat);
	virtual bool enableDCDC(unsigned char mask);
	virtual bool getDCDC(unsigned char& mask);
	virtual bool setLeds(const std::vector<unsigned char>& leds);
	virtual bool getBatteryStatus(unsigned char& elec_level, 
					unsigned char& PC1_level, 
					unsigned char& motorH_level, 
					unsigned char& motorL_level, 
					unsigned char& charger_status);
	virtual bool getPowerDiagnostics(PowerDiagnostics& diagnostics);
private:

	static int16_t bufferToInt(const unsigned char* buffer); // buffer [High_byte:Low_byte] to signed int16
	static uint16_t bufferToUnsignedInt(const unsigned char* buffer); // buffer [High_byte:Low_biyte] to unsigned int16

	bool setFans(bool fans); // Enable or disable fans
	bool enableTiltMotor(bool enable); // Enable or disable tilt motor
	bool enableHeightMotor(bool enable); // Enable or disable height motor
	bool calibrate(bool calibrate_tilt_system, bool calibrate_height_system); // calibrate height and/or tilt system
	bool setNumberOfLeds(unsigned char number_of_leds); // Set the number of leds	 
	static void defaultPrint(const std::string& message){std::cout<<message<<std::endl;} // A default printing function

	bool getHeightDriverState(unsigned char& state);
	bool setHeightDriverState(unsigned char state);

	bool getHeightStatus(unsigned char& status);
	

	IdMindBoard board1; // Sensors board
	IdMindBoard board2; // Motors board
	Calibration calibration;
	unsigned char number_of_leds; // Number of configured leds

	void (*printInfo)(const std::string& message); // Function to print information
	void (*printError)(const std::string& message); // Function to print errors

	bool is_stopped; // Is robot stopped?
	int final_dcdc_mask;  // The DCDC mask to set in the destructor
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
	flush();
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
	// Get firmware version
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
{ // last two bytes of each response are [Checksum_high_byte:Checksum_low_byte]
	uint16_t checksum1 = (int)response[response_size-2]; 
	checksum1 <<= 8; 
	checksum1 += (int)response[response_size-1];
	uint16_t checksum2=0;
	// The checksum should be the sum of all bytes as a unsigned int16
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
	// Read incoming bytes
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
	flush(); // Flush the current incoming bytes
	int i=0;
	while(i<WRITTING_TRIES && !board.write(command,command_size)) { // We will try to write the command message
		printError("Cannot write to "+name);
		i++;
	}
	if (i==WRITTING_TRIES) {
		printError("Communication with "+name+ " aborted due to maximum writting tries reached");
		return false;
	}
	usleep(1); // Time for the board to to begin to proccess
	int read_bytes=0;
	int bytes; 
	utils::Timer timer; // Timer to control the timeout  
	bool counting_timeout=false; // Are we counting?
	while (read_bytes<response_size) { // While we need more bytes
		if (!board.incomingBytes(bytes)) { // How many incoming bytes?
			printError("Communication with "+name+ " aborted due to reading error (cannot get incoming bytes)");
			return false;
		}
		if (bytes==0) { // No incoming bytes... we start/check the timer
			if (!counting_timeout) {
				counting_timeout=true;				
				timer.init(); // Start the timer
			} else if (timer.elapsed()>0.05) { // timeout error
				printError("Communication with "+name+ " aborted due to reading timeout");
				return false;
			}
			usleep(1); // Give a little more time to the board
		} else {
			counting_timeout=false; // no counting timeout
			bytes = std::min(bytes,response_size-read_bytes); // upper bound for the bytes to read
			int aux = board.read(response+read_bytes,bytes); // read bytes
			if (aux==-1) {
				printError("Communication with "+name+ " aborted due to reading error");
				return false;
			}
			read_bytes += aux; // update number of read bytes
		}
	}
	// Response: [Header]...[Message_counter][Checksum_High][Checksum_Low]	
	
	if (response[0] != command[0]) { // The first response byte should be equal to the first command byte
		printError("Invalid response header from "+name);
		return false;
	}
	if (counter==-1) { // Start message counter
		counter = response[response_size-3]; 
	} else {
		counter++; // Increment message counter
		if (counter==256) { // from 0 to 255
			counter=0;
		}
	}
	if (counter != response[response_size-3]) { // Check the message counter
		printError("Invalid response counter from "+name);
		return false;
	}
	if (!checksum(response_size)) { // Validate the checksum
		printError("Invalid checksum from "+name);
		return false;
	}
	return true;
}

inline
IdMindRobot::IdMindRobot(const std::string& board1,const std::string& board2,
				const Calibration& calibration,
				unsigned char initial_dcdc_mask,
				unsigned char final_dcdc_mask,
				unsigned char number_of_leds,
				void (*printInfo)(const std::string& message),
				void (*printError)(const std::string& message))
: board1(board1,"board1",printInfo,printError),
  board2(board2,"board2",printInfo,printError),
  calibration(calibration),
  number_of_leds(number_of_leds),
  printInfo(printInfo),
  printError(printError),
  is_stopped(true),
  final_dcdc_mask(final_dcdc_mask)
{
	bool board1_open = IdMindRobot::board1.open(); // Open Board1 (sensors)
	bool board2_open = IdMindRobot::board2.open(); // Open Board2 (motors)
	
	// Initialization
	if (!board1_open || 
		!board2_open || 
		!enableDCDC(initial_dcdc_mask) || 
		!setNumberOfLeds(number_of_leds)) {
		throw ("Teresa initialization aborted");
	}



	// TEST
	/*
	unsigned char status;
	char buffer[256];	
	getHeightDriverState(status);
	sprintf(buffer,"Height driver state (0x5E): 0x%X",status);
	printInfo(buffer);

	getHeightStatus(status);
	sprintf(buffer,"Height status (0x5C): 0x%X",status);
	printInfo(buffer);	
	
		
	for (unsigned char i = 0; i<3; i++) {
		setHeightDriverState(i);
		sprintf(buffer,"Escrito %x",i);
		printInfo(buffer);	
		getHeightDriverState(status);
		sprintf(buffer,"Leido %x",status);
		printInfo(buffer);
		
	}*/

}

inline
IdMindRobot::~IdMindRobot()
{
	// Switch off leds
	if (number_of_leds>0) {
		std::vector<unsigned char> leds;
		leds.resize(number_of_leds*3);
		for (unsigned i=0;i<leds.size();i++) {
			leds[i]=0;
		}
		setLeds(leds);
	}
	// Configure DCDC with the final mask
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
	IdMindRobot::number_of_leds = number_of_leds;
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
bool IdMindRobot::getHeightDriverState(unsigned char& state)
{
	board2.command[0] = GET_HEIGHT_DRIVER_STATE;
	if (!board2.communicate(1,5)) {
		printError("Cannot get height driver state");
		return false;
	}
	state = board2.response[1];
	return true;
}

inline
bool IdMindRobot::getHeightStatus(unsigned char& status)
{
	board2.command[0] = GET_HEIGHT_STATUS;
	if (!board2.communicate(1,5)) {
		printError("Cannot get height status");
		return false;
	}
	status = board2.response[1];
	return true;
}


inline
bool IdMindRobot::setHeightDriverState(unsigned char state)
{
	board2.command[0] = SET_HEIGHT_DRIVER_STATE;
	board2.command[1] = state;
	if (!board2.communicate(2,4)) {
		printError("Cannot set height driver state");
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
bool IdMindRobot::setHeightVelocity(int velocity)
{
	if (velocity<0 || velocity>40) {
		printError("Invalid height velocity. It should be in [0,40]");
		return false;
	}
	uint16_t v_ref = (uint16_t)velocity; 
	board2.command[0] = SET_HEIGHT_VELOCITY;
	board2.command[1] = (unsigned char)(v_ref >> 8);
	board2.command[2] = (unsigned char)(v_ref & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot set height velocity");
		return false;
	}
	return true;
}

inline
bool IdMindRobot::setTiltVelocity(int velocity)
{
	if (velocity<2 || velocity>8) {
		printError("Invalid tilt velocity. It should be in [0,8]");
		return false;
	}
	uint16_t v_ref = (uint16_t)velocity; 
	board2.command[0] = SET_TILT_VELOCITY;
	board2.command[1] = (unsigned char)(v_ref >> 8);
	board2.command[2] = (unsigned char)(v_ref & 0xFF);
	if (!board2.communicate(3,4)) {
		printError("Cannot set tilt velocity");
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
bool IdMindRobot::setVelocityRaw(int16_t v_left, int16_t v_right)
{
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
bool IdMindRobot::setVelocity(double linear, double angular)
{
	linear=saturateLinearVelocity(linear);
	angular=saturateAngularVelocity(angular);
	double left_wheel_velocity = saturateLinearVelocity(linear - ROBOT_RADIUS_M*angular);
	double right_wheel_velocity = saturateLinearVelocity(linear + ROBOT_RADIUS_M*angular);
	int16_t v_left=0;
	int16_t v_right=0;
	if (left_wheel_velocity > LINEAR_VELOCITY_ZERO_THRESHOLD || left_wheel_velocity < -LINEAR_VELOCITY_ZERO_THRESHOLD) {
		v_left = (int16_t)std::round(fabs(left_wheel_velocity) * calibration.A_left + calibration.B_left);
		if (calibration.inverse_left_motor) v_left = -v_left;
		if (left_wheel_velocity<0) v_left *= -1; 
	}
	if (right_wheel_velocity > LINEAR_VELOCITY_ZERO_THRESHOLD || right_wheel_velocity < -LINEAR_VELOCITY_ZERO_THRESHOLD) {
		v_right = (int16_t)std::round(fabs(right_wheel_velocity) * calibration.A_right + calibration.B_right);
		if (calibration.inverse_right_motor) v_right = -v_right;
		if (right_wheel_velocity<0) v_right *= -1; 
	}
	return setVelocityRaw(v_left,v_right);
}


inline
bool IdMindRobot::setVelocity2(double linear, double angular)
{
	linear=saturateLinearVelocity(linear);
	angular=saturateAngularVelocity(angular);
	double left_wheel_velocity = saturateLinearVelocity(linear - ROBOT_RADIUS_M*angular);
	double right_wheel_velocity = saturateLinearVelocity(linear + ROBOT_RADIUS_M*angular);
	int16_t v_left=0;
	int16_t v_right=0;
	if (left_wheel_velocity > LINEAR_VELOCITY_ZERO_THRESHOLD || left_wheel_velocity < -LINEAR_VELOCITY_ZERO_THRESHOLD) {
		//v_left = (int16_t)std::round(fabs(left_wheel_velocity) * calibration.A_left + calibration.B_left);
		v_left = (int16_t)std::round(left_wheel_velocity * calibration.A_left + calibration.B_left); 
		if (calibration.inverse_left_motor) v_left = -v_left;
		//if (left_wheel_velocity<0) v_left *= -1; 
	}
	if (right_wheel_velocity > LINEAR_VELOCITY_ZERO_THRESHOLD || right_wheel_velocity < -LINEAR_VELOCITY_ZERO_THRESHOLD) {
		//v_right = (int16_t)std::round(fabs(right_wheel_velocity) * calibration.A_right + calibration.B_right);
		v_right = (int16_t)std::round(right_wheel_velocity * calibration.A_right + calibration.B_right);
		if (calibration.inverse_right_motor) v_right = -v_right;
		//if (right_wheel_velocity<0) v_right *= -1; 
	}
	return setVelocityRaw(v_left,v_right);
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
	int16_t inc_left = -bufferToInt(board2.response+1);
	int16_t inc_right = bufferToInt(board2.response+3);
	imdl = inc_left==0?0:(double)inc_left*0.00024802;
	imdr = inc_right==0?0:(double)inc_right*0.00024802;
	is_stopped = inc_left==0 && inc_right==0;
	return true;
}

inline
bool IdMindRobot::setHeight(int height)
{
	int16_t height_ref = (int16_t)height;
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
bool IdMindRobot::setTilt(int tilt)
{
	int16_t tilt_ref = (int16_t)tilt;
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
bool IdMindRobot::getHeight(int& height)
{
	board2.command[0] = GET_HEIGHT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot get height");
		return false;
	}
	height = bufferToInt(board2.response+1);
	return true;
}

inline
bool IdMindRobot::getTilt(int& tilt)
{
	board2.command[0] = GET_TILT_ACTUAL_POSITION;
	if (!board2.communicate(1,8)) {
		printError("Cannot set tilt angle");
		return false;
	}
	tilt = bufferToInt(board2.response+1);
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
bool IdMindRobot::getDCDC(unsigned char& mask)
{
	board1.command[0] = GET_ENABLE_DCDC_OUTPUT;
	if (!board1.communicate(1,5)) {
		printError("Cannot get DCDC outputs");
		return false;
	}
	mask = board1.response[1];
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
