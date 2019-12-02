/*
 * ICubKinematics.cpp
 *
 * Description: Dedicated class for the kinematic manipulation of the iCub robot.
 *
 *  Created on: Nov 28, 2018
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

#ifndef ICUBKINEMATICS_H_
#define ICUBKINEMATICS_H_

//Includes
//Standard Library
#include <iostream> //cout
#include <chrono> //sleep and time
#include <thread> //threads
#include <vector> //vectors
//Yarp
#include <yarp/os/all.h> //OS
#include <yarp/sig/all.h> //Signal processing
#include <yarp/math/Math.h> //Math
#include <iCub/iKin/iKinFwd.h> //iKin forward kinematics

//Namespaces
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace std;

enum KinChain { kin_left_arm, kin_right_arm, kin_head, kin_torso, kin_left_eye, kin_right_eye, kin_left_leg, kin_right_leg };

class ICubKinematics
{
public:
	ICubKinematics(KinChain ss); //Constructor
	~ICubKinematics(); //Destructor
	
    yarp::sig::Vector getJointValues();
	int getNumDOF();
	yarp::sig::Vector getEndEffectorPose(bool axisRep);
	yarp::sig::Vector getEndEffectorPosition();
    yarp::sig::Vector setJointValues(yarp::sig::Vector &q);
	double getJointMaxValue(int joint);
	double getJointMinValue(int joint);
    yarp::sig::Vector convertRobot2WorldFrame(yarp::sig::Vector q);
	yarp::sig::Matrix getH();
	yarp::sig::Matrix getAnalyticalJacobian();
	yarp::sig::Matrix getGeometricalJacobian();
	yarp::sig::Vector getHessian_ij(int i, int j);

private:
	void init(KinChain ss); //Initialization

	yarp::sig::Matrix Trw; //Robot to world transformation matrix
	
	//Kinematic chain selection
	KinChain kinematic_selection;
	string selected;
	iCubArm *iArm;
	iCubLeg *iLeg;
	iCubTorso *iTorso;
	iCubHeadCenter *iHead;
	iCubEye *iEye;
	iKinChain *chain;
	
	bool kinematic_status = false;
	
};

#endif /* ICUBKINEMATICS_H_ */
