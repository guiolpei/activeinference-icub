/*
 * FreeEnergyOptimization.cpp
 *
 * Description: Dedicated class for the implementation of the 
 *              free energy optimization algorithm.
 *
 *  Created on: Dec 7, 2018
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

#include "FreeEnergyOptimization.h"

//----------------------------------------------------------------------------------
//Initialization -------------------------------------------------------------------
//----------------------------------------------------------------------------------

FreeEnergyOptimization::FreeEnergyOptimization()
{
	//Constructor
	
}

FreeEnergyOptimization::~FreeEnergyOptimization()
{
	//Destructor
	endLogging();
}

void FreeEnergyOptimization::setParameters(string description, int dof, int derivatives, vector<double> initialState, double deltaT, 
						bool actionOn, vector<double> gainPerception, vector<double> weightedPerception, vector<double> gainAction, IntegrationMethod intMethod, bool debug)
{
	unsigned int internalTerms = 0;
	unsigned int lldim = 0;
	
	if (dof >= 0)
	{
		initialized = false;
		
		if (derivatives <= 0 || derivatives > 2){
			cout << "The number of derivatives calculated must be 1 or 2." << endl;
			return;
		}
		
		internalTerms = derivatives*dof; //position, velocity, etc. for each dof
		
		matrixDerivatives.clear();
		matrixDerivativeAddends.clear();
		matrixDerivatives.resize(derivatives);
		matrixDerivativeAddends.resize(derivatives);
		
		vectorActions.clear();
		vectorActions.resize(dof);
		
		for (unsigned int i = 0; i < derivatives; i++)
		{
			matrixDerivatives[i].resize(dof);
			
		}
		
		//interal state (mu)
		internalState.clear();
		internalState.resize(internalTerms);
		//action (a)
		action.clear();
		action.resize(dof);
		//free-energy
		freeEnergy = 0;
		//other log variables
		logVars.clear();
		
		if (gainAction.size() != dof)
		{
			cout << "Vector of action proportional gain must have the size of DOF." << endl;
			return;
		}

		if (weightedPerception.size() != dof)
		{
			cout << "Vector of weighted perception must have the size of DOF." << endl;
			return;
		}

		if (gainPerception.size() != internalTerms)
		{
			cout << "Vector of perception gain must have the size of derivatives*DOF." << endl;
			return;
		}

		if (initialState.size() == internalTerms) 
		{
			this->description = description;			
			this->initialState = initialState;
			internalState = initialState;
			this->dof = dof;
			this->debug = debug;
			this->derivatives = derivatives;
			this->deltaT = deltaT;
			this->actionOn = actionOn;
			this->gainPerception = gainPerception;
			this->weightedPerception = weightedPerception;
			this->gainAction = gainAction;
			this->intMethod = intMethod;
			
			for (unsigned int i = 0; i < dof; i++)
				action[i] = 0;
			
			initialized = true;
		}
		else
			cout << "Initial state vector must have the size of derivatives*DOF." << endl;
	
	}else
		cout << "DOF must be greater than 0." << endl;
}

unsigned int FreeEnergyOptimization::getDOF()
{
	return dof;
}

bool FreeEnergyOptimization::isInitialized()
{
	return initialized;
}

void FreeEnergyOptimization::setPerceptionSaturation(bool activated, vector<double> sat_low, vector<double> sat_high)
{
	if (activated)
	{
		if ( (sat_low.size() != dof) || (sat_high.size() != dof) )
		{
			cout << "Saturation values for perception should be a vector of size equal to DOF." << endl;
			return;
		}

		//activate perception saturation limits
		perceptionSaturation = true;

		perceptionSaturationLow = sat_low;
		perceptionSaturationHigh = sat_high;

		if (debug)
		{
			cout << "Perception saturation activated low: {";
			for (int i = 0; i < dof; i++){
				cout << " " << sat_low[i];
			}
			cout <<  " }" << endl; 
			cout << "Perception saturation activated high: {";
			for (int i = 0; i < dof; i++){
				cout << " " << sat_high[i];
			}
			cout <<  " }" << endl; 
		}

	}
	else
	{ 
		perceptionSaturation = false;

		if (debug) cout << "Perception saturation deactivated." << endl;
	}
}

void FreeEnergyOptimization::setDerivativeSaturation(bool activated, vector<double> sat_low, vector<double> sat_high)
{
	if (activated)
	{
		if ( (sat_low.size() != dof) || (sat_high.size() != dof) )
		{
			cout << "Saturation values for perception should be a vector of size equal to DOF." << endl;
			return;
		}

		//activate derivative saturation limits
		derivativeSaturation = true;

		derivativeSaturationLow = sat_low;
		derivativeSaturationHigh = sat_high;

		if (debug)
		{
			cout << "Derivative saturation activated low: {";
			for (int i = 0; i < dof; i++){
				cout << " " << sat_low[i];
			}
			cout <<  " }" << endl; 
			cout << "Derivative saturation activated high: {";
			for (int i = 0; i < dof; i++){
				cout << " " << sat_high[i];
			}
			cout <<  " }" << endl; 
		}

	}
	else
	{ 
		derivativeSaturation = false;

		if (debug) cout << "Derivative saturation deactivated." << endl;
	}
}

void FreeEnergyOptimization::setActionSaturation(bool activated, double sat_low, double sat_high)
{
	if (activated)
	{
		//activate action saturation limits
		actionSaturation = true;

		actionSaturationLow = sat_low;
		actionSaturationHigh = sat_high;

		if (debug) cout << "Action saturation activated: {" << sat_low << ", " << sat_high << "}" << endl; 

	}
	else
	{ 
		actionSaturation = false;

		if (debug) cout << "Action saturation deactivated." << endl;
	}
}

//----------------------------------------------------------------------------------
//Derivative terms -----------------------------------------------------------------
//----------------------------------------------------------------------------------

void FreeEnergyOptimization::addNewDerivativeTerm(int derivative, DerivativeTerm t)
{
	if (validateDerivativeTerm(t))
	{
		cout << "Derivative term is valid, added to the derivative list #" << derivative << endl;
		matrixDerivativeAddends[derivative].push_back(t);
	}
	else
	{
		cout << "Derivative term is not consistent, will not be added to the list." << endl;
	}
	
}

DerivativeTerm FreeEnergyOptimization::getDerivativeTerm(int derivative, int index)
{
	DerivativeTerm buf;

	if (index < matrixDerivativeAddends[derivative].size())
	{
		buf = (matrixDerivativeAddends[derivative])[index];
	}
	else
	{
		cout << "Derivative term index out of bounds, derivative #" << derivative << " array size: " << matrixDerivativeAddends[derivative].size() << "." << endl;
	}
	
	return buf;
}

void FreeEnergyOptimization::removeDerivativeTerm(int derivative, int index)
{
	
	if (index < matrixDerivativeAddends[derivative].size())
	{
		(matrixDerivativeAddends[derivative]).erase((matrixDerivativeAddends[derivative]).begin()+index);
	}
	else
	{
		cout << "Derivative term index out of bounds, derivative #" << derivative << " array size: " << matrixDerivativeAddends[derivative].size() << "." << endl;
	}
			
}

unsigned int FreeEnergyOptimization::getNumDerivativeTerms(int derivative)
{
	unsigned int size = 0;
	
	size = (matrixDerivativeAddends[derivative]).size();
			
	return size;
}

bool FreeEnergyOptimization::validateDerivativeTerm(DerivativeTerm t)
{
	bool valid = true;
	
	//check real and calculated states function vectors
	if (t.realState.size() != t.calcState.size())
	{
		cout << "Dimensions for function vectors of real and calculated states do not match." << endl;
		valid = false;
	}
	
	//check dimensions for matrix-vector multiplication
	if (t.partialDerivativesOn)
	{
		//first dimension must be the number of dof of the system
		if (t.partialDerivatives.size() != dof)
		{
			cout << "Dimensions for partial derivative matrix and DOF of system do not match." << endl;
			valid = false;
		}
		
		//second dimension must be the same as the term state vectors
		for(unsigned int i = 0; i < t.partialDerivatives.size(); i++)
		{
			if (t.partialDerivatives[i].size() != t.realState.size())
			{
				cout << "Dimensions for partial derivative matrix and term state vectors do not match." << endl;
				valid = false;
			}
		}
	}
	
	return valid;
}

vector<double> FreeEnergyOptimization::testDerivativeTermCalc(int derivative, int index, vector<double> q, vector<double> a)
{
	vector<double> x;
	x.clear();
	
	if (index < matrixDerivativeAddends[derivative].size())
	{
		for (unsigned int i = 0; i < (matrixDerivativeAddends[derivative])[index].calcState.size(); i++)
		{
			x.push_back((matrixDerivativeAddends[derivative])[index].calcState[i](q, a));
		}
	}
	else
	{
		cout << "Derivative term index out of bounds, derivative #" << derivative << " array size: " << matrixDerivativeAddends[derivative].size() << "." << endl;
	}
	
	return x;
}

//----------------------------------------------------------------------------------
//Action terms ---------------------------------------------------------------------
//----------------------------------------------------------------------------------

void FreeEnergyOptimization::addNewActionTerm(ActionTerm t)
{
	if (validateActionTerm(t))
	{
		cout << "Action term is valid, added to the action list." << endl;
		vectorActionAddends.push_back(t);
	}
	else
	{
		cout << "Action term is not consistent, will not be added to the list." << endl;
	}
	
}

ActionTerm FreeEnergyOptimization::getActionTerm(int index)
{
	ActionTerm buf;

	if (index < vectorActionAddends.size())
	{
		buf = vectorActionAddends[index];
	}
	else
	{
		cout << "Action term index out of bounds, action array size: " << vectorActionAddends.size() << "." << endl;
	}
	
	return buf;
}

void FreeEnergyOptimization::removeActionTerm(int index)
{
	
	if (index < vectorActionAddends.size())
	{
		vectorActionAddends.erase(vectorActionAddends.begin()+index);
	}
	else
	{
		cout << "Action term index out of bounds, action array size: " << vectorActionAddends.size() << "." << endl;
	}
			
}

unsigned int FreeEnergyOptimization::getNumActionTerms()
{
	unsigned int size = 0;
	
	size = vectorActionAddends.size();
			
	return size;
}

bool FreeEnergyOptimization::validateActionTerm(ActionTerm t)
{
	bool valid = true;
	
	//check real and calculated states function vectors
	if (t.realState.size() != t.calcState.size())
	{
		cout << "Dimensions for function vectors of real and calculated states do not match." << endl;
		valid = false;
	}
	
	//check dimensions for matrix-vector multiplication
	if (t.partialDerivativesOn)
	{
		//first dimension must be the number of dof of the system
		if (t.partialDerivatives.size() != dof)
		{
			cout << "Dimensions for partial derivative matrix and DOF of system do not match." << endl;
			valid = false;
		}
		
		//second dimension must be the same as the term state vectors
		for(unsigned int i = 0; i < t.partialDerivatives.size(); i++)
		{
			if (t.partialDerivatives[i].size() != t.realState.size())
			{
				cout << "Dimensions for partial derivative matrix and term state vectors do not match." << endl;
				valid = false;
			}
		}
	}
	
	return valid;
}

vector<double> FreeEnergyOptimization::testActionTermCalc(int index, vector<double> q, vector<double> a)
{
	vector<double> x;
	x.clear();
	
	if (index < vectorActionAddends.size())
	{
		for (unsigned int i = 0; i < vectorActionAddends[index].calcState.size(); i++)
		{
			x.push_back(vectorActionAddends[index].calcState[i](q, a));
		}
	}
	else
	{
		cout << "Action term index out of bounds, action array size: " << vectorActionAddends.size() << "." << endl;
	}
	
	return x;
}

void FreeEnergyOptimization::setZeroConditionFunction(zfunc function)
{
	zeroCondition = true;
	zeroConditionFunction = function;
}


//----------------------------------------------------------------------------------
//Free-energy terms ----------------------------------------------------------------
//----------------------------------------------------------------------------------

void FreeEnergyOptimization::addNewFreeEnergyTerm(FreeEnergyTerm t)
{
	if (validateFreeEnergyTerm(t))
	{
		cout << "Free-energy term is valid, added to the free-energy list." << endl;
		vectorFreeEnergyAddends.push_back(t);
	}
	else
	{
		cout << "Free-energy term is not consistent, will not be added to the list." << endl;
	}
	
}

FreeEnergyTerm FreeEnergyOptimization::getFreeEnergyTerm(int index)
{
	FreeEnergyTerm buf;

	if (index < vectorFreeEnergyAddends.size())
	{
		buf = vectorFreeEnergyAddends[index];
	}
	else
	{
		cout << "Free-energy term index out of bounds, free-energy array size: " << vectorFreeEnergyAddends.size() << "." << endl;
	}
	
	return buf;
}

void FreeEnergyOptimization::removeFreeEnergyTerm(int index)
{
	
	if (index < vectorFreeEnergyAddends.size())
	{
		vectorFreeEnergyAddends.erase(vectorFreeEnergyAddends.begin()+index);
	}
	else
	{
		cout << "Free-energy term index out of bounds, free-energy array size: " << vectorFreeEnergyAddends.size() << "." << endl;
	}
			
}

unsigned int FreeEnergyOptimization::getNumFreeEnergyTerms()
{
	unsigned int size = 0;
	
	size = vectorFreeEnergyAddends.size();
			
	return size;
}

bool FreeEnergyOptimization::validateFreeEnergyTerm(FreeEnergyTerm t)
{
	bool valid = true;
	
	//check real and calculated states function vectors
	if (t.realState.size() != t.calcState.size())
	{
		cout << "Dimensions for function vectors of real and calculated states do not match." << endl;
		valid = false;
	}
	
	return valid;
}

vector<double> FreeEnergyOptimization::testFreeEnergyTermCalc(int index, vector<double> q, vector<double> a)
{
	vector<double> x;
	x.clear();
	
	if (index < vectorFreeEnergyAddends.size())
	{
		for (unsigned int i = 0; i < vectorFreeEnergyAddends[index].calcState.size(); i++)
		{
			x.push_back(vectorFreeEnergyAddends[index].calcState[i](q, a));
		}
	}
	else
	{
		cout << "Free-energy term index out of bounds, free-energy array size: " << vectorFreeEnergyAddends.size() << "." << endl;
	}
	
	return x;
}

//----------------------------------------------------------------------------------
//Internal state, action and free-energy -------------------------------------------
//----------------------------------------------------------------------------------

vector<double> FreeEnergyOptimization::getInternalState()
{	
	return internalState;
}

vector<double> FreeEnergyOptimization::getAction()
{	
	return action;
}

double FreeEnergyOptimization::getFreeEnergy()
{	
	return freeEnergy;
}

void FreeEnergyOptimization::updateInternalState()
{
	double derivativeBuf, actionBuf, freeEnergyBuf;
	double rState, cState;
	double deltaAction, deltaState;
	vector<double> termStateVector;
	
	//clear other logged states
	logVars.clear();
	
	//1. calculate derivatives
	
	//set values of derivatives
	for (unsigned int d = 0; d < derivatives; d++)
	{
		for (unsigned int j = 0; j < dof; j++)
		{
			if (d == 0) //first derivative
				(matrixDerivatives[d])[j] = internalState[j+dof];
			else //other derivatives
				(matrixDerivatives[d])[j] = 0;
		}
	}
	
	//derivatives calculation
	for (unsigned int d = 0; d < derivatives; d++)
	{
		if (debug)
			cout << "* Calculating derivative #" << d << "..." << endl;
		
		for (unsigned int i = 0; i < matrixDerivativeAddends[d].size(); i++)
		{
			if (debug)
				cout << "Calculating term " << (matrixDerivativeAddends[d])[i].description << "..." << endl;
			
			//check partial derivative existence
			if ((matrixDerivativeAddends[d])[i].partialDerivativesOn)
			{
				termStateVector.clear();

				//for each dof of term state, calculate term state vector
				for (unsigned int j = 0; j < (matrixDerivativeAddends[d])[i].realState.size(); j++)
				{
					rState = (matrixDerivativeAddends[d])[i].realState[j](internalState, action);
					cState = (matrixDerivativeAddends[d])[i].calcState[j](internalState, action);
					
					if (debug)
					{
						cout << "Real state for term " << i << " and DOF " << j << " is: "
							<< rState << endl;
						cout << "Calculated state for term " << i << " and DOF " << j << " is: "
							<< cState << endl;
					}
					
					derivativeBuf = ((matrixDerivativeAddends[d])[i].sign/(matrixDerivativeAddends[d])[i].variance)*
						(rState - cState);

					//log real state values
					if ((matrixDerivativeAddends[d])[i].logRealState)
					{
						if (debug) cout << "Logging real state " << j << " for " << (matrixDerivativeAddends[d])[i].description << ": " << rState << endl;
						logVars.push_back(rState);
					}

					//log calculated state values
					if ((matrixDerivativeAddends[d])[i].logCalcState)
					{
						if (debug) cout << "Logging calculated state " << j << " for " << (matrixDerivativeAddends[d])[i].description << ": " << cState << endl;
						logVars.push_back(cState);
					}

                    //patch to log visual values with stereo 3D
                    if ((matrixDerivativeAddends[d])[i].description.compare(0, 7, "d_pos3D") == 0)
                    {
                        if (j == ((matrixDerivativeAddends[d])[i].realState.size() - 1)) //only on last
                        {
                            if (description.compare(0, 9, "right_arm") == 0)
                            {
                                logVars.push_back(re_pos(3)); //l_u
                                logVars.push_back(calc_vision_pos_l(0)); //calc_l_u
                                logVars.push_back(re_pos(4)); //l_v
                                logVars.push_back(calc_vision_pos_l(1)); //calc_l_v
                                logVars.push_back(re_pos(6)); //r_u
                                logVars.push_back(calc_vision_pos_r(0)); //calc_r_u
                                logVars.push_back(re_pos(7)); //r_v
                                logVars.push_back(calc_vision_pos_r(1)); //calc_r_v
                            }
                            if (description.compare(0, 8, "left_arm") == 0) logVars.push_back(calc_vision_pos_b(j));
                        }
                    }

                    //patch to log attractor value and calculated position
                    if ((matrixDerivativeAddends[d])[i].description.compare(0, 11, "d_attractor") == 0)
                    {
                        if (description.compare(0, 9, "right_arm") == 0)
                        {
                            if (j < 1) //only on first
                            {
                                logVars.push_back(attr_3d_pos(0)); //world x
                                logVars.push_back(attr_3d_pos(1)); //world y
                                logVars.push_back(attr_3d_pos(2)); //world z
                                logVars.push_back(att_pos(3)); //l_u
                                logVars.push_back(att_pos(4)); //l_v
                                logVars.push_back(att_pos(6)); //r_u
                                logVars.push_back(att_pos(7)); //r_v
                            }
                        }

                        if (description.compare(0, 4, "head") == 0)
                        {
                            if (j < 1) //only on first
                            {
                                logVars.push_back(att_pos(3)); //l_u
                                logVars.push_back(att_pos(4)); //l_v
                            }
                        }
                    }

                    //patch to log encoder values when only vision + attractor is activated
                    if ((matrixDerivativeAddends[d])[i].description.compare(0, 8, "d_vision") == 0)
                    {
                        //right arm, only once
                        if (description.compare(0, 9, "right_arm") == 0 && j == 0)
                        {
                            if (matrixDerivativeAddends[d].size() == 2)
                            {
                                yarp::sig::Vector ra_encoders = iCubRightArmCtrl->readEncoders();
                                logVars.push_back(ra_encoders[1]*CTRL_DEG2RAD);
                                logVars.push_back(ra_encoders[2]*CTRL_DEG2RAD);
                                logVars.push_back(ra_encoders[3]*CTRL_DEG2RAD);
                            }

                        }
						
						//left arm, only once
                        if (description.compare(0, 8, "left_arm") == 0 && j == 0)
                        {
                            if (matrixDerivativeAddends[d].size() == 2)
                            {
                                yarp::sig::Vector la_encoders = iCubLeftArmCtrl->readEncoders();
                                logVars.push_back(la_encoders[1]*CTRL_DEG2RAD);
                                logVars.push_back(la_encoders[2]*CTRL_DEG2RAD);
                                logVars.push_back(la_encoders[3]*CTRL_DEG2RAD);
                            }

                        }
                    }

					//check null value deactivation
					if ((matrixDerivativeAddends[d])[i].deactivateNull)
					{					
						if (rState == 0)
						{
							if (debug) cout << "Term of DOF " << j << " of " << (matrixDerivativeAddends[d])[i].description
								<< " will not be considered due to null value deactivation." << endl;
							derivativeBuf = 0;
						}
					}								
					
					//add to term state vector
					termStateVector.push_back(derivativeBuf);

					if (debug)
						cout << "Weighted state error for term " << i << " and DOF " << j << " is: "
							<< derivativeBuf << endl;

				}

				//matrix-vector multiplication
				for (unsigned int j = 0; j < dof; j++)
				{
					derivativeBuf = 0;

					for (unsigned int k = 0; k < termStateVector.size(); k++)
					{
						derivativeBuf += ((matrixDerivativeAddends[d])[i].partialDerivatives[j])[k](internalState, action) * termStateVector[k];
					}

					//add to first derivative calculation
					(matrixDerivatives[d])[j]  += derivativeBuf;

					if (debug)
						cout << d << "-derivative for term " << i << " and DOF " << j << " is: "
							<< derivativeBuf << endl;
				}

			}
			else //no partial derivatives
			{
				//for each dof of term state, calculate term state vector
				for (unsigned int j = 0; j < (matrixDerivativeAddends[d])[i].realState.size(); j++)
				{
					rState = (matrixDerivativeAddends[d])[i].realState[j](internalState, action);
					cState = (matrixDerivativeAddends[d])[i].calcState[j](internalState, action);
					
					if (debug)
					{
						cout << "Real state for term " << i << " and DOF " << j << " is: "
							<< rState << endl;
						cout << "Calculated state for term " << i << " and DOF " << j << " is: "
							<< cState << endl;
					}
					
					
					derivativeBuf = ((matrixDerivativeAddends[d])[i].sign/(matrixDerivativeAddends[d])[i].variance)*
						(rState - cState);

					//log real state values
					if ((matrixDerivativeAddends[d])[i].logRealState)
					{
						if (debug) cout << "Logging real state " << j << " for " << (matrixDerivativeAddends[d])[i].description << ": " << rState << endl;
						logVars.push_back(rState);
					}
					//log calculated state values
					if ((matrixDerivativeAddends[d])[i].logCalcState)
					{
						if (debug) cout << "Logging calculated state " << j << " for " << (matrixDerivativeAddends[d])[i].description << ": " << cState << endl;
						logVars.push_back(cState);
					}

					//patch to log attractor value and calculated position
					if ((matrixDerivativeAddends[d])[i].description.compare(0, 11, "d_attractor") == 0)
					{
                        if (description.compare(0, 9, "right_arm") == 0)
                        {
                            if (j < 1) //only on first
                            {
                                logVars.push_back(attr_3d_pos(0)); //world x
                                logVars.push_back(attr_3d_pos(1)); //world y
                                logVars.push_back(attr_3d_pos(2)); //world z
                                logVars.push_back(att_pos(3)); //l_u
                                logVars.push_back(att_pos(4)); //l_v
                                logVars.push_back(att_pos(6)); //r_u
                                logVars.push_back(att_pos(7)); //r_v
                            }
                        }

                        if (description.compare(0, 4, "head") == 0)
                        {
                            if (j < 1) //only on first
                            {
                                logVars.push_back(att_pos(3)); //l_u
                                logVars.push_back(att_pos(4)); //l_v
                            }
                        }
                    }

					//patch to log real encoder values
					if ((matrixDerivativeAddends[d])[i].description.compare(0, 11, "d_encoders") == 0)
					{
						if (description.compare(0, 9, "right_arm") == 0)
						{
                            if (j == 0) logVars.push_back(qTencoders[0]);
                            if (j == 1) logVars.push_back(qRAencoders[1]);
                            if (j == 2) logVars.push_back(qRAencoders[2]);
                            if (j == 3) logVars.push_back(qRAencoders[3]);
						}
						if (description.compare(0, 4, "head") == 0)
						{
							if (j == 0) logVars.push_back(qLEencoders[0]);
							if (j == 1) logVars.push_back(qLEencoders[2]);
                            if (j == 2) logVars.push_back(qLEencoders[3]);
						}
						if (description.compare(0, 8, "left_arm") == 0)
						{
							if (j == 0) logVars.push_back(qLAencoders[1]);
							if (j == 1) logVars.push_back(qLAencoders[2]);
							if (j == 2) logVars.push_back(qLAencoders[3]);
						}
					}

                    //patch to log visual position when only encoders are activated
                    if ((matrixDerivativeAddends[d])[i].description.compare(0, 11, "d_encoders") == 0)
                    {
                        //right arm, only once
                        if (description.compare(0, 9, "right_arm") == 0 && j == 0)
                        {
                            if (matrixDerivativeAddends[d].size() == 2)
                            {
                                /*logVars.push_back(re_pos[0]); //MOD
                                logVars.push_back(re_pos[1]);
                                //publish if needed
                                if (publish){
                                    Bottle& output_u = plot_u.prepare();
                                    output_u.clear();
                                    output_u.addDouble(re_pos[0]);
                                    plot_u.write();
                                    Bottle& output_v = plot_v.prepare();
                                    output_v.clear();
                                    output_v.addDouble(re_pos[1]);
                                    plot_v.write();
                                }*/
                            }

                        }
                    }


					//check null value deactivation
					if ((matrixDerivativeAddends[d])[i].deactivateNull)
					{					
						if (rState == 0)
						{
							if (debug) cout << "Term of DOF " << j << " of " << (matrixDerivativeAddends[d])[i].description
								<< " will not be considered due to null value deactivation." << endl;
							derivativeBuf = 0;
						}
					}

					//add to first derivative calculation
					(matrixDerivatives[d])[j] += derivativeBuf;

					if (debug)
						cout << d << "-derivative for term " << i << " and DOF " << j << " is: "
							<< derivativeBuf << endl;

				}
			}

		}
		
	if (debug)
		cout << "Derivative #" << d << " calculation finished." << endl;

	}
		
	//2. calculate action
	if (actionOn)
	{
		if (debug)
			cout << "Calculating actions..." << endl;
		
		//set values of actions
		for (unsigned int i = 0; i < dof; i++)
		{
			vectorActions[i] = 0;
		}


		for (unsigned int i = 0; i < vectorActionAddends.size(); i++)
		{
			if (debug)
				cout << "Calculating term " << vectorActionAddends[i].description << "..." << endl;

			//check partial derivative existence
			if (vectorActionAddends[i].partialDerivativesOn)
			{
				termStateVector.clear();

				//for each dof of term state, calculate term state vector
				for (unsigned int j = 0; j < vectorActionAddends[i].realState.size(); j++)
				{
					rState = vectorActionAddends[i].realState[j](internalState, action);
					cState = vectorActionAddends[i].calcState[j](internalState, action);
					
					if (debug)
					{
						cout << "Real state for term " << i << " and DOF " << j << " is: "
							<< rState << endl;
						cout << "Calculated state for term " << i << " and DOF " << j << " is: "
							<< cState << endl;
					}
					
					actionBuf = (vectorActionAddends[i].sign/vectorActionAddends[i].variance)*
						(rState - cState);

					//check null value deactivation
					if (vectorActionAddends[i].deactivateNull)
					{					
						if (rState == 0)
						{
							if (debug) cout << "Term of DOF " << j << " of " << vectorActionAddends[i].description
								<< " will not be considered due to null value deactivation." << endl;
							actionBuf = 0;
						}
					}

					//add to term state vector
					termStateVector.push_back(actionBuf);

					if (debug)
						cout << "Weighted error for term " << i << " and DOF " << j << " is: "
							<< actionBuf << endl;

				}

				//matrix-vector multiplication
				for (unsigned int j = 0; j < dof; j++)
				{
					actionBuf = 0;

					for (unsigned int k = 0; k < termStateVector.size(); k++)
					{
						actionBuf += (vectorActionAddends[i].partialDerivatives[j])[k](internalState, action) * termStateVector[k];
					}

					//add to first derivative calculation
					vectorActions[j] += actionBuf;

					if (debug)
						cout << "Action for term " << i << " and DOF " << j << " is: "
							<< actionBuf << endl;
				}

			}
			else //no partial derivatives
			{
				//for each dof of term state, calculate term state vector
				for (unsigned int j = 0; j < vectorActionAddends[i].realState.size(); j++)
				{
					rState = vectorActionAddends[i].realState[j](internalState, action);
					cState = vectorActionAddends[i].calcState[j](internalState, action);
					
					if (debug)
					{
						cout << "Real state for term " << i << " and DOF " << j << " is: "
							<< rState << endl;
						cout << "Calculated state for term " << i << " and DOF " << j << " is: "
							<< cState << endl;
					}
					
					actionBuf = (vectorActionAddends[i].sign/vectorActionAddends[i].variance)*
						(rState - cState);

					//check null value deactivation
					if (vectorActionAddends[i].deactivateNull)
					{					
						if (rState == 0)
						{
							if (debug) cout << "Term of DOF " << j << " of " << vectorActionAddends[i].description
								<< " will not be considered due to null value deactivation." << endl;
							actionBuf = 0;
						}
					}

					//add to first derivative calculation
					vectorActions[j] += actionBuf;

					if (debug)
						cout << "Action for term " << i << " and DOF " << j << " is: "
							<< actionBuf << endl;

				}
			}

		}
		
	}
	
	//3. calculate free-energy
	if (debug)
		cout << "Calculating free-energy..." << endl;

	//set value of free-energy
	freeEnergy = 0;

	for (unsigned int i = 0; i < vectorFreeEnergyAddends.size(); i++)
	{
		if (debug)
			cout << "Calculating term " << vectorFreeEnergyAddends[i].description << "..." << endl;

		
		//for each dof of term state, calculate term state vector
		for (unsigned int j = 0; j < vectorFreeEnergyAddends[i].realState.size(); j++)
		{
			rState = vectorFreeEnergyAddends[i].realState[j](internalState, action);
			cState = vectorFreeEnergyAddends[i].calcState[j](internalState, action);

			if (debug)
			{
				cout << "Real state for term " << i << " and DOF " << j << " is: "
					<< rState << endl;
				cout << "Calculated state for term " << i << " and DOF " << j << " is: "
					<< cState << endl;
			}

			//free-energy term calculation
			freeEnergyBuf = (vectorFreeEnergyAddends[i].sign/(2*vectorFreeEnergyAddends[i].variance))*
				pow((rState - cState), 2); //+ log(1/(sqrt(2*M_PI*vectorFreeEnergyAddends[i].variance)));

			//check null value deactivation
			if (vectorFreeEnergyAddends[i].deactivateNull)
			{					
				if (rState == 0)
				{
					if (debug) cout << "Term of DOF " << j << " of " << vectorFreeEnergyAddends[i].description
						<< " will not be considered due to null value deactivation." << endl;
					freeEnergyBuf = 0;
				}
			}

			//add to free-energy calculation
			freeEnergy += freeEnergyBuf;

			if (debug)
				cout << "Free-energy for term " << i << " and DOF " << j << " is: "
					<< freeEnergyBuf << endl;

		}

	}
	
	//6. log starting values (including logVars)
	if (logOn && time == 0)
	{	
		registerValues(true);
	}

	//4. update internal state for derivatives
	if (debug)
		cout << "Updating internal state of derivatives..." << endl;
	
	for (unsigned int d = 0; d < derivatives; d++)
	{
		for (unsigned int i = 0; i < dof; i++)
		{
			if (debug)
				cout << "Calculated change in " << d << "-derivative for DOF " << i << " is: "
					<< (matrixDerivatives[d])[i] << endl;

			switch(intMethod)
			{
				case Euler:
					//update using Euler first order approximation
					if (d == 0) deltaState = gainPerception[i+d*dof]*(internalState[i+1*dof] + weightedPerception[i]*(matrixDerivatives[d])[i])*deltaT;
					else deltaState = gainPerception[i+d*dof]*(matrixDerivatives[d])[i]*deltaT;
					break;
				case LocalLinear:
					deltaState = 0;
					break;
			}

			if (debug)
				cout << "Integrated change in " << d << "-derivative for DOF " << i << " is: "
					<< deltaState << endl;	
		
			//check action saturation and saturate
			if (d == 0 && perceptionSaturation) //mu
			{
				if ((internalState[i+d*dof] + deltaState) > perceptionSaturationHigh[i]) internalState[i+d*dof] = perceptionSaturationHigh[i];
				else if ((internalState[i+d*dof] + deltaState) < perceptionSaturationLow[i]) internalState[i+d*dof] = perceptionSaturationLow[i];
				else internalState[i+d*dof] = internalState[i+d*dof] + deltaState; //update internal state
			}else if (d == 1 && derivativeSaturation) //mup and above
			{
				if ((internalState[i+d*dof] + deltaState) > derivativeSaturationHigh[i]) internalState[i+d*dof] = derivativeSaturationHigh[i];
				else if ((internalState[i+d*dof] + deltaState) < derivativeSaturationLow[i]) internalState[i+d*dof] = derivativeSaturationLow[i];
				else internalState[i+d*dof] = internalState[i+d*dof] + deltaState; //update internal state
			}else internalState[i+d*dof] = internalState[i+d*dof] + deltaState; //update internal state
			
			//set velocity to zero if condition is satisfied
			if (d == 1 && zeroCondition)
			{
				if (zeroConditionFunction(internalState[i+d*dof], i))
				{
					internalState[i+d*dof] = 0;
					deltaState = 0;
					
					if (debug)
						cout << "Zero condition function set first derivative for DOF " << i << " to 0." << endl;
				}
			}

		}
	}
	
	//5. update actions
	if (actionOn)
	{
		if (debug)
			cout << "Updating actions..." << endl;

		for (unsigned int i = 0; i < dof; i++)
		{
			//sign already considered in term structure
			//vectorActions[i] = -vectorActions[i];
			
			if (debug)
				cout << "Calculated change in action for DOF " << i << " is: "
					<< vectorActions[i] << endl;

			switch(intMethod)
			{
				case Euler:
					//update action using Euler first order approximation considering proportional gain
					deltaAction = gainAction[i]*vectorActions[i]*deltaT;
					break;
				case LocalLinear:
					deltaAction = 0;
					break;
			}

			if (debug)
				cout << "Integrated change in action for DOF " << i << " is: "
					<< deltaAction << endl;

			//check action saturation and saturate
			if (actionSaturation)
			{
				if ((action[i] + deltaAction) > actionSaturationHigh) action[i] = actionSaturationHigh;
				else if ((action[i] + deltaAction) < actionSaturationLow) action[i] = actionSaturationLow;
				else action[i] = action[i] + deltaAction; //update action
			}else action[i] = action[i] + deltaAction; //update action

			//set action to zero if condition is satisfied
			if (zeroCondition)
			{
				if (zeroConditionFunction(action[i], i))
				{
					action[i] = 0;
					deltaAction = 0;
					
					if (debug)
						cout << "Zero condition function set action for DOF " << i << " to 0." << endl;
				}
			}
		
		}
	}
	
	//6. update time
	time += deltaT;
	
	//6. log values
	if (logOn) registerValues(false);
	
	if (debug)
		cout << "Internal state of derivatives and actions update finished." << endl;
}

//----------------------------------------------------------------------------------
//Logging --------------------------------------------------------------------------
//----------------------------------------------------------------------------------

bool FreeEnergyOptimization::startLogging(string filename)
{
	//create log file
  	logFile.open (filename, ios::out); //ios::app
	
	if (!logFile.is_open())
	{
		cout << "Error opening log file: " << filename << endl;
		logOn = false;
		return false;
	}
	
	if (debug)
		cout << "Logging started." << endl;
	
	if (prevLoggingFunction)
	{
		startLoggingFunction(logFile, description);
		if (debug) cout << "Initial logging function executed." << endl;	
	}
	
	logFilename = filename;
	logOn = true;
	return true;
}

void FreeEnergyOptimization::setStartLoggingFunction(lfunc function)
{
	prevLoggingFunction = true;
	startLoggingFunction = function;
}

void FreeEnergyOptimization::setEndLoggingFunction(lfunc function)
{
	postLoggingFunction = true;
	endLoggingFunction = function;
}

void FreeEnergyOptimization::registerValues(bool first)
{
	//check if opened
	if (!logFile.is_open()) return;

	//time
	logFile << time << " ";
	
	//internal states (and derivatives)
	for (unsigned int d = 0; d < derivatives; d++)
	{
		for (unsigned int i = 0; i < dof; i++)
		{
			if (first) logFile << initialState[i+d*dof] << " " << 0 << " ";
			else logFile << internalState[i+d*dof] << " " << (matrixDerivatives[d])[i] << " ";
		}
	}
	
	//action (and derivatives)
	for (unsigned int i = 0; i < dof; i++)
	{
		if (first) logFile << 0 << " " << 0 << " ";
		else logFile << action[i] << " " << vectorActions[i] << " ";
	}

	//other values (real state values)
	for (unsigned int i = 0; i < logVars.size(); i++)
	{
		//log encoder values
		logFile << logVars[i] << " ";
	}
	
	//free-energy
	logFile << freeEnergy << " ";
	
	//time step
	logFile << deltaT << " " << endl;
}

void FreeEnergyOptimization::endLogging()
{
	//close log file
	if (logFile.is_open())
	{
		if (postLoggingFunction) 
		{
			endLoggingFunction(logFile, description);
			if (debug) cout << "Final logging function executed." << endl;	
		}
		
		logFile.close();
		
		if (debug)
			cout << "Logging ended." << endl;
	}
	
	logOn = false;
}

