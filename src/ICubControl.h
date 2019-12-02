/*
 * ICubControl.h
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

#ifndef ICUBCONTROL_H_
#define ICUBCONTROL_H_

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

enum Subsystem { head, left_arm, right_arm, torso, left_leg, right_leg };
enum CtrlMode { position, velocity, torque };

class ICubControl
{
public:
	ICubControl(string name); //Constructor
	~ICubControl(); //Destructor

	bool setupSubsystem(Subsystem ss, bool debug);
	void closeSubsystem();

	Vector readEncoders();
	void changeControlMode(CtrlMode mode);
	void changeControlMode(unsigned int i, CtrlMode mode); 
	void setPosition(Vector positions, bool blocking);
	void setPositionMotionSpeed(unsigned int i, double s);
	void setPositionMotionSpeeds(Vector speeds);
	void velocityMove(unsigned int i, double v);
	void setRefTorque(unsigned int i, double t);
	void checkMotionDone(bool *flag);
	
	void lookAtLocation(string dataPort);

private:
	
	Network yarp; //Setup yarp
	
	//Subsystem
	PolyDriver *robotDevice; //Selected robot device (subsystem) for the instance
	IPositionControl *posControl; //Position control interface
	IVelocityControl *velControl; //Velocity control interface
	ITorqueControl *tauControl; //Torque control interface
	IEncoders *encData; //Encoders interface
	IControlMode2 *controlMode; //Control mode selector
	int joints = 0; //Number of joints in subsystem
	
	//Data vectors
	Vector setpoints; //Setpoints for movement
	Vector velocities; //Velocities for movement
	Vector encoders; //Value of encoders in joints
	Vector commands; //Commands for velocity/torque control
	BufferedPort<Vector> targetPort; //Port for receiving data
	
	//Internal status
	string robot_name;
	string remotePort, localPort; //Name of ports
	bool device_status = false;
	Subsystem subsystem_selection;
	bool debug;
};

#endif /* ICUBCONTROL_H_ */
