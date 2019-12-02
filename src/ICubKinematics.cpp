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

#include "ICubKinematics.h"

ICubKinematics::ICubKinematics(KinChain kc)
{
	//Constructor
	
	//Transformation matrix creation (robot to world)
	Trw.resize(4, 4);
	Vector row;
	row.push_back(0); row.push_back(-1); row.push_back(0); row.push_back(0);
	Trw.setRow(0, row);
	row.clear();
	row.push_back(0); row.push_back(0); row.push_back(1); row.push_back(0.5976);
	Trw.setRow(1, row);
	row.clear();
	row.push_back(-1); row.push_back(0); row.push_back(0); row.push_back(-0.026);
	Trw.setRow(2, row);
	row.clear();
	row.push_back(0); row.push_back(0); row.push_back(0); row.push_back(1);
	Trw.setRow(3, row);
	//Trw = [ 0 -1 0 0; 0 0 1 0.5976; -1 0 0 -0.026; 0 0 0 1 ];
	
	//Kinematic chain initialization
	init(kc);
	
}

ICubKinematics::~ICubKinematics()
{
	//Destructor
	delete chain;

	switch (kinematic_selection)
	{
		case kin_left_arm:
		case kin_right_arm:
			delete iArm;
			cout << "Kinematic chain " << selected << " deleted." << endl;
			break;
		case kin_head:
			delete iHead;
			cout << "Kinematic chain " << selected << " deleted." << endl;
			break;
		case kin_torso:
			delete iTorso;
			cout << "Kinematic chain " << selected << " deleted." << endl;
			break;
		case kin_left_eye:
		case kin_right_eye:
			delete iEye;
			cout << "Kinematic chain " << selected << " deleted." << endl;
			break;
		case kin_left_leg:
		case kin_right_leg:
			delete iLeg;
			cout << "Kinematic chain " << selected << " deleted." << endl;
			break;
	}

}

void ICubKinematics::init(KinChain kc)
{
	//Kinematic chain selection
	
	// 1. iKin already provides internally coded limbs for iCub, such as
	// iCubArm, iCubLeg, iCubEye, ..., along with the proper H0 matrix	
	// 2. Get a chain on the limb; you can use limb object directly but then some
	// methods will not be available, such as the access to links through
	// [] or () operators. This prevent the user from adding/removing links
	// to iCub limbs as well as changing their properties too easily.
	// Anyway, limb object is affected by modifications on the chain.
	switch (kc)
	{
		case kin_left_arm:
			iArm = new iCubArm("left");
			chain = new iKinChain(*iArm->asChain());
			selected = "left_arm";
			break;
		case kin_right_arm:
			iArm = new iCubArm("right");
			chain = new iKinChain(*iArm->asChain());
			selected = "right_arm";
			break;
		case kin_head:
			iHead = new iCubHeadCenter();
			chain = new iKinChain(*iHead->asChain());
			selected = "head";
			break;
		case kin_torso:
			iTorso = new iCubTorso();
			chain = new iKinChain(*iTorso->asChain());
			selected = "torso";
			break;
		case kin_left_eye:
			iEye = new iCubEye("left");
			chain = new iKinChain(*iEye->asChain());
			selected = "left_eye";
			break;
		case kin_right_eye:
			iEye = new iCubEye("right");
			chain = new iKinChain(*iEye->asChain());
			selected = "right_eye";
			break;
		case kin_left_leg:
			iLeg = new iCubLeg("left");
			chain = new iKinChain(*iLeg->asChain());
			selected = "left_leg";
			break;
		case kin_right_leg:
			iLeg = new iCubLeg("right");
			chain = new iKinChain(*iLeg->asChain());
			selected = "right_leg";
			break;
	}
	
	cout << "Kinematic chain " << selected << " selected." << endl;
	kinematic_selection = kc;
	
	switch (kc)
	{
		case kin_left_arm:
		case kin_right_arm:
			kinematic_status = iArm->isValid();
			break;
		case kin_head:
			kinematic_status = iHead->isValid();
			break;
		case kin_torso:
			kinematic_status = iTorso->isValid();
			break;
		case kin_left_eye:
		case kin_right_eye:
			kinematic_status = iEye->isValid();
			break;
		case kin_left_leg:
		case kin_right_leg:
			kinematic_status = iLeg->isValid();
			break;
	}
	
	if (kinematic_status)
		cout << "Kinematic chain " << selected << " correctly configured." << endl;
	else
		cout << "Error in configuration of " << selected << "!" << endl;
	
}

yarp::sig::Vector ICubKinematics::getJointValues()
{
	return chain->getAng();
}

int ICubKinematics::getNumDOF()
{
	return chain->getDOF();
}

yarp::sig::Vector ICubKinematics::getEndEffectorPose(bool axisRep)
{
	return chain->EndEffPose(axisRep);
}

yarp::sig::Vector ICubKinematics::getEndEffectorPosition()
{
	return chain->EndEffPosition();
}

yarp::sig::Vector ICubKinematics::setJointValues(yarp::sig::Vector &q)
{
	return chain->setAng(q);
}

double ICubKinematics::getJointMaxValue(int joint)
{
	double max;
	
	if ( (joint < 0) || (joint > chain->getDOF() - 1) ){
		cout << "Index out of bounds, chain has " << chain->getDOF() << " DOF." << endl;
		max = 0.0;
	}else{
		max = (*chain)(joint).getMax();
	}
			
	return max;
}

double ICubKinematics::getJointMinValue(int joint)
{
	double min;
	
	if ( (joint < 0) || (joint > chain->getDOF() - 1) ){
		cout << "Index out of bounds, chain has " << chain->getDOF() << " DOF." << endl;
		min = 0.0;
	}else{
		min = (*chain)(joint).getMin();
	}
			
	return min;
}

Vector ICubKinematics::convertRobot2WorldFrame(Vector q)
{
	Vector qp;
	
	q.push_back(1);
	qp = Trw*q;
	qp.resize(3);
	
	return qp;
}

Matrix ICubKinematics::getH()
{
	return chain->getH();
}

Matrix ICubKinematics::getAnalyticalJacobian()
{
	return chain->AnaJacobian();
}

Matrix ICubKinematics::getGeometricalJacobian()
{
	return chain->GeoJacobian();
}

Vector ICubKinematics::getHessian_ij(int i, int j)
{
	return chain->Hessian_ij(i, j);
}



