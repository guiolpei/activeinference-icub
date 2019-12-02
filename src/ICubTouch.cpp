/*
 * ICubTouch.cpp
 *
 * Description: Dedicated class to process the touch sensors of the robot.
 *
 *  Created on: Feb 11, 2019
 *     Authors: Guillermo Oliver, Pablo Lanillos, Gordon Cheng
 *
 * Copyright 2019 Institute for Cognitive Systems, Technical University of Munich. All rights reserved.
 *
 * This file is part of the ActiveInferenceICub project.
 *
 * ActiveInferenceICub is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "ICubTouch.h"

ICubTouch::ICubTouch(string name)
{
	//Constructor
	robot_name = name; //robot name: icub/icubSim
	device_status = false;
}

ICubTouch::~ICubTouch()
{
	//Destructor
	closeTouch();
}

void ICubTouch::setupTouch(Touchsel ts, string touch_port, bool debug)
{
	//Setup port reading
	
	switch (ts)
	{
		case right_hand:
			// right hand port: /icubSim/skin/right_hand_comp
			if (debug) cout << "Setting up right hand touch sensor reading..." << endl;
			remotePort = "/" + robot_name + "/skin/right_hand_comp";
			localPort = touch_port;
			targetPort.open(localPort);
			Network::connect(remotePort, localPort);
			break;
		case left_hand:
			// left hand port: /icubSim/skin/left_hand_comp
			if (debug) cout << "Setting up left hand touch sensor reading..." << endl;
			remotePort = "/" + robot_name + "/skin/left_hand_comp";
			localPort = touch_port;
			targetPort.open(localPort);
			Network::connect(remotePort, localPort);
			break;
	}
	
	device_status = true;
	touch_selection = ts;
	this->debug = debug;
	
}

void ICubTouch::closeTouch()
{
	//Check if open
	if (device_status)
	{
		Network::disconnect(remotePort, localPort);	
	}

}

yarp::sig::Vector ICubTouch::readTouchSensors()
{	
	Vector *sensor_data;
	
	if (debug) cout << "Reading touch sensor data for " << remotePort << "." << endl;
	
	sensor_data = targetPort.read();
	
	if (debug) cout << (*sensor_data).length() << " bytes read from " << remotePort << "." << endl;
	
	// data format: 192 byte sensor data
	//		index(12) middle(12) ring(12) little(12) thumb(12) 
	//		empty(12) empty(12) empty(12) 
	//		palm(12) palm(12) palm(12) palm(12) 
	//		empty(12) empty(12) empty(12) empty(12)
	
	return *sensor_data;

}

TouchState ICubTouch::identifyTouchSensors(yarp::sig::Vector touch_data)
{
	TouchState touched;
	int i = 0;	

	if (touch_data.length() != 192) //check correct data size
	{
		cout << "Data size must be 192 bytes." << remotePort << "." << endl;
		return touched;
	}
	
	//check touched extremities
	for (i = 0; i < 12; i++){
		if (touch_data[i] > 50.0) touched.index = true; //index
		if (touch_data[i+12] > 50.0) touched.middle = true; //middle
		if (touch_data[i+12+12] > 50.0) touched.ring = true; //ring
		if (touch_data[i+12+12+12] > 50.0) touched.little = true; //little
		if (touch_data[i+12+12+12+12] > 50.0) touched.thumb = true; //thumb
		if (touch_data[i+12+12+12+12+12+12+12+12] > 50.0) touched.palm1 = true; //palm1
		if (touch_data[i+12+12+12+12+12+12+12+12+12] > 50.0) touched.palm2 = true; //palm2
		if (touch_data[i+12+12+12+12+12+12+12+12+12+12] > 50.0) touched.palm3 = true; //palm3
		if (touch_data[i+12+12+12+12+12+12+12+12+12+12+12] > 50.0) touched.palm4 = true; //palm4
	}


	
	return touched;
}

bool ICubTouch::checkTouch(yarp::sig::Vector touch_data)
{
	int i = 0;	

	if (touch_data.length() != 192) //check correct data size
	{
		cout << "Data size must be 192 bytes." << remotePort << "." << endl;
		return false;
	}
	
	//check touched extremities
	for (i = 0; i < 12; i++){
		if (touch_data[i] > 50.0) return true; //index
		if (touch_data[i+12] > 50.0) return true; //middle
		if (touch_data[i+12+12] > 50.0) return true; //ring
		if (touch_data[i+12+12+12] > 50.0) return true; //little
		if (touch_data[i+12+12+12+12] > 50.0) return true; //thumb
		if (touch_data[i+12+12+12+12+12+12+12+12] > 50.0) return true; //palm1
		if (touch_data[i+12+12+12+12+12+12+12+12+12] > 50.0) return true; //palm2
		if (touch_data[i+12+12+12+12+12+12+12+12+12+12] > 50.0) return true; //palm3
		if (touch_data[i+12+12+12+12+12+12+12+12+12+12+12] > 50.0) return true; //palm4
	}


	
	return false;
}


