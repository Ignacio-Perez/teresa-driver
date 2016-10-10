/***********************************************************************/
/**                                                                    */
/** teresa_node.h                                                      */
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


#include <teresa_driver/teresa_node_calib.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "TeresaNode");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	Teresa::NodeCalib node_calib(n,pn);
	return 0;
}
