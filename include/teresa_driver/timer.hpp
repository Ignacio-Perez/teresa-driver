/***********************************************************************/
/**                                                                    */
/** timer.hpp                                                          */
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


#ifndef _TIMER_HPP_
#define _TIMER_HPP_

#include <chrono>

namespace utils
{

class Timer
{
public:
    
    Timer() : initiated(false) {}
    void init() { beg_ = clock_::now(); initiated=true;}
    double elapsed() const { 
        return initiated?std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count():0; }
 
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    bool initiated; 
    std::chrono::time_point<clock_> beg_;
  
};



}

#endif
