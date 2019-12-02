/*
 * ICubTouch.h
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

#ifndef ICUBTOUCH_H_
#define ICUBTOUCH_H_

//Includes
//Standard Library
#include <iostream> //cout
#include <chrono> //sleep and time
#include <thread> //threads
//Yarp
#include <yarp/os/all.h> //OS
#include <yarp/sig/all.h> //Signal processing
#include <yarp/dev/all.h> //Devices

//Namespaces
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

enum Touchsel { left_hand, right_hand };

struct TouchState
{
	bool index = false;
	bool middle = false;
	bool ring = false;
	bool little = false;
	bool thumb = false;
	bool palm1 = false;
	bool palm2 = false;
	bool palm3 = false;
	bool palm4 = false;
};

class ICubTouch
{
public:
	ICubTouch(string name); //Constructor
	~ICubTouch(); //Destructor

	void setupTouch(Touchsel ts, string touch_port, bool debug);
	void closeTouch();

	yarp::sig::Vector readTouchSensors();
	TouchState identifyTouchSensors(yarp::sig::Vector touch_data);
	bool checkTouch(yarp::sig::Vector touch_data);

private:
	
	Network yarp; //Setup yarp
		
	//Data vectors
	BufferedPort<yarp::sig::Vector> targetPort; //Port for receiving data
	
	//Internal status
	string robot_name;
	string remotePort, localPort; //Name of ports
	bool device_status = false;
	Touchsel touch_selection;
	bool debug;
};

#endif /* ICUBTOUCH_H_ */
