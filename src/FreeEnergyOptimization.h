/*
 * FreeEnergyOptimization.h
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

#ifndef FREEENERGYOPTIMIZATION_H_
#define FREEENERGYOPTIMIZATION_H_

//Includes
#include <iostream> //cout
#include <vector> //vectors
#include <fstream> //file operations
#include <cmath> //math functions
#include <eigen3/Eigen/Dense> //eigen dense matrix algebra
#include "ICubControl.h" //patch to read encoders
#include "ICubKinematics.h" //patch

//Namespaces
using namespace std;

//Definitions (I)
typedef double (*cfunc)(vector<double>, vector<double>); //pointer to calculating function
typedef vector<cfunc> fvector; //vector of pointers to functions
typedef vector<vector<cfunc>> fmatrix; //matrix of pointers to functions
typedef void (*lfunc)(fstream&, string); //pointer to logging function
typedef bool (*zfunc)(double, int); //pointer to zero condition function

//Structures
struct DerivativeTerm { //derivative terms
	string description = "empty"; //description of term
	int sign = 1; //leading sign for term
	double variance = 1; //variance value
	bool partialDerivativesOn = false; //partial derivatives considered
	fmatrix partialDerivatives; //matrix of partial derivatives
	fvector realState; //real (sensed) state
	fvector calcState; //calculated state
	bool logRealState = false; //log real state
	bool logCalcState = false; //log calculated state
	bool deactivateNull = false; //deactivate contribution if no real state available
};

struct FreeEnergyTerm { //free-energy terms
	string description = "empty"; //description of term
	int sign = 1; //leading sign for term
	double variance = 1; //variance value
	fvector realState; //real (sensed) state
	fvector calcState; //calculated state
	bool deactivateNull = false; //deactivate contribution if no real state available
};

struct ActionTerm { //action terms
	string description = "empty"; //description of term
	int sign = 1; //leading sign for term
	double variance = 1; //variance value
	bool partialDerivativesOn = false; //partial derivatives considered
	fmatrix partialDerivatives; //matrix of partial derivatives
	fvector realState; //real (sensed) state
	fvector calcState; //calculated state
	bool deactivateNull = false; //deactivate contribution if no real state available
};

//Definitions (II)
typedef vector<DerivativeTerm> tvector; //vector of derivative terms
typedef vector<vector<DerivativeTerm>> tmatrix; //matrix of derivative terms
typedef vector<ActionTerm> avector; //vector of action terms
typedef vector<FreeEnergyTerm> evector; //vector of free-energy terms

//Enum definitions
enum IntegrationMethod { Euler, LocalLinear };

//Patch to log attractor position and real encoders
#include <yarp/sig/all.h> //Signal processing
extern yarp::sig::Vector att_pos; //attractor position in visual field
//extern yarp::sig::Vector calc_vision_pos; //calculated right arm position in visual field //MOD
extern yarp::sig::Vector calc_vision_pos_l, calc_vision_pos_r; //calculated right arm position in visual field
extern yarp::sig::Vector calc_vision_pos_b; //calculated left arm position in visual field
extern Eigen::Vector3d attr_3d_pos; //world 3d attractor position
extern yarp::sig::Vector qRAencoders; //RA encoders 1, 2 and 3
extern yarp::sig::Vector qLEencoders; //LE encoders 0 and 2
extern yarp::sig::Vector qLAencoders; //RA encoders 1, 2 and 3
extern yarp::sig::Vector qTencoders; //T encoders 0/2
extern yarp::sig::Vector re_pos; //right arm end-effector position in visual field
extern ICubControl *iCubRightArmCtrl; //read encoders for only vision
extern ICubControl *iCubLeftArmCtrl;
extern bool publish; //publish
extern BufferedPort<Bottle> plot_u, plot_v;

class FreeEnergyOptimization
{
public:
	FreeEnergyOptimization(); //Constructor
	~FreeEnergyOptimization(); //Destructor
	
	//Initialization
	void setParameters(string description, int dof, int derivatives, vector<double> initialState, double deltaT, 
				bool actionOn, vector<double> gainPerception, vector<double> weightedPerception, vector<double> gainAction, IntegrationMethod intMethod, bool debug);
	unsigned int getDOF();
	bool isInitialized();
	void setPerceptionSaturation(bool activated, vector<double> sat_low, vector<double> sat_high);
	void setDerivativeSaturation(bool activated, vector<double> sat_low, vector<double> sat_high);
	void setActionSaturation(bool activated, double sat_low, double sat_high);
	
	//Derivative terms
	void addNewDerivativeTerm(int derivative, DerivativeTerm t);
	DerivativeTerm getDerivativeTerm(int derivative, int index);
	void removeDerivativeTerm(int derivative, int index);
	unsigned int getNumDerivativeTerms(int derivative);
	vector<double> testDerivativeTermCalc(int derivative, int index, vector<double> q, vector<double> a);
	
	//Action terms
	void addNewActionTerm(ActionTerm t);
	ActionTerm getActionTerm(int index);
	void removeActionTerm(int index);
	unsigned int getNumActionTerms();
	vector<double> testActionTermCalc(int index, vector<double> q, vector<double> a);
	void setZeroConditionFunction(zfunc function);
	
	//Free-energy terms
	void addNewFreeEnergyTerm(FreeEnergyTerm t);
	FreeEnergyTerm getFreeEnergyTerm(int index);
	void removeFreeEnergyTerm(int index);
	unsigned int getNumFreeEnergyTerms();
	vector<double> testFreeEnergyTermCalc(int index, vector<double> q, vector<double> a);
	
	//Internal state, action and free-energy
	vector<double> getInternalState();
	vector<double> getAction();
	double getFreeEnergy();
	void updateInternalState();
	
	//Logging
	bool startLogging(string filename);
	void setStartLoggingFunction(lfunc function);
	void setEndLoggingFunction(lfunc function);
	void endLogging();
	
private:
	//Internal parameters
	bool initialized = false;
	string description;
	unsigned int dof = 0;
	unsigned int derivatives = 1;
	double deltaT = 1; //sampling time
	bool actionOn = false;
	bool debug = false;
	bool logOn = false;
	double time = 0;
	IntegrationMethod intMethod = Euler;
	
	//Derivative terms
	tmatrix matrixDerivativeAddends; //matrix of derivative terms
	vector<vector<double>> matrixDerivatives; //matrix of derivative values
	bool validateDerivativeTerm(DerivativeTerm t);
	vector<double> gainPerception; //perception proportional gain for integration
	vector<double> weightedPerception; //weighted perception parameter
	bool perceptionSaturation = false;
	bool derivativeSaturation = false;
	vector<double> perceptionSaturationLow;
	vector<double> perceptionSaturationHigh;
	vector<double> derivativeSaturationLow;
	vector<double> derivativeSaturationHigh;
	
	//Action terms
	avector vectorActionAddends; //vector of action terms
	vector<double> vectorActions; //vector of action values
	bool validateActionTerm(ActionTerm t);
	vector<double> gainAction; //action proportional gain for integration
	bool actionSaturation = false;
	double actionSaturationLow;
	double actionSaturationHigh;
	bool zeroCondition = false;
	zfunc zeroConditionFunction;
	
	//Free-energy terms
	evector vectorFreeEnergyAddends; //vector of free-energy terms
	vector<double> vectorFreeEnergy;
	bool validateFreeEnergyTerm(FreeEnergyTerm t);
	
	//Internal state, action and free-energy
	vector<double> initialState; //internal initial state (mu)
	vector<double> internalState; //internal state (mu)
	vector<double> action; //action (a)
	double freeEnergy; //free-energy value
	
	//Logging
	vector<double> logVars;
	fstream logFile;
	string logFilename;
	void registerValues(bool first);
	bool prevLoggingFunction = false;
	bool postLoggingFunction = false;
	lfunc startLoggingFunction; //function to run at start of logging
	lfunc endLoggingFunction; //function to run at end of logging
	
	
};

#endif /* FREEENERGYOPTIMIZATION_H_ */
