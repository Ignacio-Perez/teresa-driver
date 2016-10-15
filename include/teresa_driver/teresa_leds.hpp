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


#ifndef _TERESA_LEDS_HPP_
#define _TERESA_LEDS_HPP_

namespace Teresa
{

/**
 * A virtual class to generate light patterns by using the leds of the robot
 *
 * This will be done updating a vector of RGB values 
 */
class Leds
{
public:
	Leds() {}
	virtual ~Leds() {}
	/**
	 * Update the RGB array, this will be called in the main loop
	 */	 
	virtual void update() = 0;
	/**
	 * Get the current RGB array of size N_LEDS*3
	 *
	 * @return vector of leds [RED_1,GREEN_1,BLUE_1,...,RED_N,GREEN_N,BLUE_N]
	 */
	virtual const std::vector<unsigned char>& getLeds() const = 0;
};


/**
 * A red light moving side to side
 */ 
class KnightRiderLeds : public Leds
{
public:
	KnightRiderLeds(unsigned char numberOfLeds)
	:forward(true), 
         activated(0)
	{
		leds.resize(numberOfLeds*3);
		leds[0]=255;
	}
	virtual ~KnightRiderLeds() {}	
	virtual void update()
	{
		leds[activated]=0;
		if (forward) {
			activated+=3;
			if (activated>=(int)leds.size()) {
				activated=leds.size()-3;
				forward=false;
			}
		} else {
			activated-=3;
			if (activated<0) {
				activated=0;
				forward=true;
			}
		}
		
		leds[activated]=255;
	}
	virtual const std::vector<unsigned char>& getLeds() const {return leds;}
private:
	bool forward;
	int activated;
	std::vector<unsigned char> leds;

};



/**
 * Another red light moving 
 */ 
class KnightRiderLeds2 : public Leds
{
public:
	KnightRiderLeds2(unsigned char numberOfLeds)
	: activated(0)
	{
		leds.resize(numberOfLeds*3);
		set(0,true);
	}
	virtual ~KnightRiderLeds2() {}	
	virtual void update()
	{
		set(activated,false);
		activated++;
		int numberOfLeds = leds.size()/3;
		if (activated==numberOfLeds) {
			activated=0;
		}
		set(activated,true);	
	}
	virtual const std::vector<unsigned char>& getLeds() const {return leds;}
private:
	void set(int led, bool on) {
		int a = led * 3;
		int b = led * 3 + 3;
		if (b >= (int)leds.size()) {
			b=0;
		}
		int c = led * 3 - 3;
		if (c<0) {
			c = leds.size()*3 - 3;
		}
		leds[a] = on?255:0;
		leds[b] = on?255:0;
		leds[c] = on?255:0;
	}

	int activated;
	std::vector<unsigned char> leds;

};


/**
 * Get the led pattern object from a name, update this function if more patterns are implemented
 * The name can be used in the parameter "leds_pattern" of the launch file
 */
Leds *getLedsPattern(const std::string& leds_pattern, unsigned char number_of_leds)
{
	if (leds_pattern == "knight_rider") {
		return new KnightRiderLeds2(number_of_leds); 
	}
	return NULL;
}


};

#endif
