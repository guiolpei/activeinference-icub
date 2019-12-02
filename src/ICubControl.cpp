/*
 * ICubControl.cpp
 *
 * Description: Dedicated class for the motor control of the robot.
 *
 *  Created on: Nov 12, 2018
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

#include "ICubControl.h"

ICubControl::ICubControl(string name)
{
	//Constructor
	robot_name = name; //robot name: icub/icubSim
	joints = 0;
	device_status = false;
}

ICubControl::~ICubControl()
{
	//Destructor
	closeSubsystem();
}

bool ICubControl::setupSubsystem(Subsystem ss, bool debug)
{
	//Setup subsystem device and interfaces
	Property options;
	
	switch (ss)
	{
		case head:
			remotePort = "/" + robot_name + "/head";
			localPort = "/controller/head";
			break;
		case left_arm:
			remotePort = "/" + robot_name + "/left_arm";
			localPort = "/controller/left_arm";
			break;
		case right_arm:
			remotePort = "/" + robot_name + "/right_arm";
			localPort = "/controller/right_arm";
			break;
		case torso:
			remotePort = "/" + robot_name + "/torso";
			localPort = "/controller/torso";
			break;
		case left_leg:
			remotePort = "/" + robot_name + "/left_leg";
			localPort = "/controller/left_leg";
			break;
		case right_leg:
			remotePort = "/" + robot_name + "/right_leg";
			localPort = "/controller/right_leg";
			break;
	}
	
	if (debug) cout << "Setting up " << remotePort << " device." << endl;
	
	//Build configuraion options for device
	options.put("device", "remote_controlboard"); //Remote controlboard
	options.put("local", localPort);   //Local port
	options.put("remote", remotePort); //Remote ports

	//Create the device
	robotDevice = new PolyDriver(options);
	if (!robotDevice->isValid()) {
		if (debug) cout << "Error setting up " << remotePort << " device. Device is not available." << endl;
		return device_status;
	}
	
	//Acquire interfaces for position control and encoders
	bool ok;
	ok = robotDevice->view(posControl); //Position control
	ok = ok && robotDevice->view(velControl); //Velocity control
	ok = ok && robotDevice->view(tauControl); //Torque control
	ok = ok && robotDevice->view(encData); //Encoder data
	ok = ok && robotDevice->view(controlMode); //Control mode

	if (!ok) {
		if (debug) cout << "Problems acquiring interfaces for " << remotePort << " device." << endl;
		return device_status;
	}
	
	//Get number of joints in subsystem
	posControl->getAxes(&joints);
	
	if (debug) cout << "The " << remotePort << " subsystem contains " << to_string(joints) << " joints." << endl;
	
	//Set to zero all the vectors
	encoders.resize(joints);
	setpoints.resize(joints);
	commands.resize(joints);
	velocities.resize(joints);
	
	for (int i = 0; i<joints; i++){
		encoders[i] = 0;
		setpoints[i] = 0;
		commands[i] = 0;
		velocities[i] = 0;
	}
 
	
	device_status = true;
	subsystem_selection = ss;
	this->debug = debug;
	
	return device_status;
	
}

void ICubControl::closeSubsystem()
{
	//Check if open
	if (robotDevice->isValid()) 
	{
		robotDevice->close();
		if (debug) cout << "Subsystem closed." << endl;
		device_status = false;
	}

}

Vector ICubControl::readEncoders()
{	
	if (debug) cout << "Reading encoder data for " << remotePort << "." << endl;
	
	while(!encData->getEncoders(encoders.data()))
	{
		//this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	if (debug) cout << "Encoder data ready." << endl;
	
	return encoders;
}

void ICubControl::changeControlMode(CtrlMode mode)
{
	//Set control mode (position/velocity)
	uint i = 0;
	
	switch (mode)
	{
		case position:
			
			if (debug) cout << "Setting position control for " << to_string(joints) << " joints in " << remotePort << "." << endl;
			
			for (uint i = 0; i < joints; i++) {
				controlMode->setControlMode(i, VOCAB_CM_POSITION);
			} 
			
			break;
			
		case velocity:
			
			if (debug) cout << "Setting velocity control for " << to_string(joints) << " joints in " << remotePort << "." << endl;
			
			for (i = 0; i < joints; i++) {
				controlMode->setControlMode(i, VOCAB_CM_VELOCITY);
			} 
			
			if (debug) cout << "Setting velocity to 0." << endl;
	
			for (i = 0; i < joints; i++){
				commands[i] = 0;
			}
			
			velControl->velocityMove(commands.data());
			
			break;
			
		case torque:
			
			if (debug) cout << "Setting torque control for " << to_string(joints) << " joints in " << remotePort << "." << endl;
			
			for (i = 0; i < joints; i++) {
				controlMode->setControlMode(i, VOCAB_CM_TORQUE);
			} 
			
			if (debug) cout << "Setting torque to 0." << endl;
	
			for (i = 0; i < joints; i++){
				commands[i] = 0;
			}
			
			tauControl->setRefTorques(commands.data());
			
			break;
	}
	
}

void ICubControl::changeControlMode(unsigned int i, CtrlMode mode)
{
	//Set control mode (position/velocity)
	
	switch (mode)
	{
		case position:
			
			if (debug) cout << "Setting position control for " << to_string(joints) << " joints in " << remotePort << "." << endl;
			

			controlMode->setControlMode(i, VOCAB_CM_POSITION);
			
			break;
			
		case velocity:
			
			if (debug) cout << "Setting velocity control for " << to_string(joints) << " joints in " << remotePort << "." << endl;
			
			controlMode->setControlMode(i, VOCAB_CM_VELOCITY);
			
			if (debug) cout << "Setting velocity to 0." << endl;
			
			velControl->velocityMove(i, 0);
			
			break;
			
		case torque:
			
			if (debug) cout << "Setting torque control for " << to_string(joints) << " joints in " << remotePort << "." << endl;
			
			controlMode->setControlMode(i, VOCAB_CM_TORQUE);
			
			tauControl->setRefTorque(i, 0);
			
			break;
	}
	
}

void ICubControl::setPosition(Vector positions, bool blocking)
{
	//Set position move and wait for it (if requiered)
	bool done=false;
	
	if (debug) cout << "Moving " << remotePort << " to desired position." << endl;
	
	setpoints = positions;
	posControl->positionMove(setpoints.data());

	if (blocking){
	
		while(!done)
		{
			posControl->checkMotionDone(&done);
			this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (debug) cout << "Motion finished." << endl;
		
	}
	
}

void ICubControl::setPositionMotionSpeed(unsigned int i, double s)
{
	//Set reference speed for position movement
	posControl->setRefSpeed(i, s);
	
}

void ICubControl::setPositionMotionSpeeds(Vector speeds)
{
	//Set reference speeds for position movement
	velocities = speeds;
	posControl->setRefSpeeds(velocities.data());
	
}

void ICubControl::checkMotionDone(bool *flag)
{
	//Check if position motion is finished
	posControl->checkMotionDone(flag);
	
}

void ICubControl::velocityMove(unsigned int i, double v)
{
	//Move joint i at desired velocity v
	velControl->velocityMove(i, v);
}

void ICubControl::setRefTorque(unsigned int i, double t)
{
	//Apply torque t (Nm) to joint i
	tauControl->setRefTorque(i, t);
}

void ICubControl::lookAtLocation(string dataPort)
{
	//Open input port
	targetPort.open("/controller/look/in");
	Network::connect(dataPort,"/controller/look/in");
	
	if (debug) cout << "Starting look for location..." << endl;
	
	while (1) { // repeat forever

		Vector *target = targetPort.read();  //Read data port
		
		if (target!=NULL) { //Check if data was obtained
			if (debug) cout << "Got vector with data: ";
			for (size_t i=0; i<target->size(); i++) {
				cout << " " << to_string((*target)[i]);
			}
			cout << endl;

			double x = (*target)[0];
			double y = (*target)[1];
			double conf = (*target)[2];

			x -= 320/2;
			y -= 240/2;

			double vx = -x*0.1;
			double vy = -y*0.1;

			// prepare command
			for (int i=0; i<joints; i++) {
				commands[i] = 0;
			}

			if (conf>0.5) {
				commands[0] = vy;
				commands[2] = vx;
			} else {
				commands[0] = 0;
				commands[2] = 0;
			}

			if (debug) cout << "Sending commands " << to_string(vx) << " " << to_string(vy) << endl;

			if (!velControl->velocityMove(commands.data()))
			{
				if (debug) cout << "Error in velocity control" << endl;
			}
			
		} else {
			if (debug) cout << "Waiting for object..." << endl;
		}
	}
	
}






