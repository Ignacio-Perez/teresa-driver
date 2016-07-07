/***********************************************************************/
/**                                                                    */
/** serial_interface.hpp                                               */
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

#ifndef _SERIAL_INTERFACE_HPP_
#define _SERIAL_INTERFACE_HPP_

#include <sstream>
#include <string>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <unistd.h>


namespace utils
{

class SerialInterface
{
public:
	SerialInterface(const std::string& devicename, bool hardware_flow_control);
	virtual ~SerialInterface();
	virtual bool open(int baudrate = B115200, int mode = O_RDWR | O_NOCTTY | O_NDELAY);
	virtual bool close();
	bool isOpen();
	const std::string& getLastError();
	bool incomingBytes(int& bytes);
	bool write(const unsigned char *buf, int buffer_size);
	int read(unsigned char* buffer, int buffer_size);
	const std::string& getDeviceName() {return devicename;}

protected:
	void setLastError(const std::string& lastError);
	virtual void closeNow();
	
private:
	std::string devicename; 
	bool hardware_flow_control;
	int fd;
	std::string lastError;

};



inline 
SerialInterface::SerialInterface(const std::string& devicename, bool hardware_flow_control) : 
devicename(devicename), 
hardware_flow_control(hardware_flow_control),
fd(-1)
{}

inline SerialInterface::~SerialInterface()
{
	if (fd!=-1) {
		::close(fd);
	}
}

inline bool SerialInterface::open(int baudrate, int mode )
{
	if (fd!=-1) {
		lastError = std::string("Cannot open ") + devicename + std::string(" because it's already open.");
		return false;	
	}
	struct termios attr;
	bool success =  ((fd = ::open(devicename.c_str(), mode)) != -1) &&
		(fcntl(fd, F_SETFL, 0) != -1) &&
		(tcgetattr(fd, &attr) != -1) &&
		(cfsetospeed (&attr, baudrate) != -1) &&
		(cfsetispeed (&attr, baudrate) != -1) &&
		(tcflush(fd, TCIOFLUSH) != -1);	
	if (success) {
		attr.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
		attr.c_cflag |= (CS8 | CLOCAL | CREAD);
		if (hardware_flow_control) {
			attr.c_cflag |= CRTSCTS;
		}
	else {
		attr.c_cflag &= ~CRTSCTS;
	}

	attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	attr.c_iflag = 0;
	attr.c_oflag = 0;
	attr.c_cc[VMIN]  = 0;
	attr.c_cc[VTIME] = 1;
	success = (tcsetattr(fd, TCSANOW, &attr) != -1);
  }

	if (!success) {
		lastError = std::string(strerror(errno));
		closeNow();
	}
	return success;
}

inline 
bool SerialInterface::close()
{
	if (fd==-1) {
		lastError = std::string("Cannot close ") + devicename + std::string(" because it isn't open.");
		return false;	
	}

	bool success = (::close(fd) != -1);
	
	if (success) {
		fd = -1;
	} else {
		lastError = std::string(strerror(errno));	
	}
	return success;
}

inline 
bool SerialInterface::isOpen()
{
	return fd!=-1;
}

inline const std::string& SerialInterface::getLastError()
{
	return lastError;
}


inline bool SerialInterface::incomingBytes(int& bytes)
{
	bool success= (ioctl(fd, FIONREAD, &bytes)!=-1);
	if (!success) {
		lastError = std::string(strerror(errno));
	}
	return success;
	
}

inline bool SerialInterface::write(const unsigned char *buf, int buffer_size)
{
	bool success =  (::write (fd, buf, buffer_size) == buffer_size);
	if (!success) {
		lastError = std::string(strerror(errno));
	}
	return success;
}	

inline int SerialInterface::read(unsigned char* buffer, int buffer_size)
{
	int bytes = ::read(fd,buffer,buffer_size);
	
	if (bytes==-1) {
		lastError = std::string(strerror(errno));
	}
	return bytes;
}

inline void SerialInterface::setLastError(const std::string& lastError)
{
	this->lastError = lastError;
}

inline void SerialInterface::closeNow()
{
	if (fd!=-1) {
		::close(fd);
		fd = -1;
	}	
}


}












#endif
