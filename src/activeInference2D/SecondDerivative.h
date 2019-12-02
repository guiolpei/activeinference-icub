/*
 * SecondDerivative.h
 *
 * Description: Second derivative function definitions.
 *
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

#include <vector> //vectors

//Yarp
#include <yarp/os/all.h> //OS
#include <yarp/sig/all.h> //Signal processing
#include <yarp/math/Math.h> //Math
#include <iCub/iKin/iKinFwd.h> //iKin forward kinematics

//Namespaces
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

//----------------------------------------------------------------------------------
//Shared variables -----------------------------------------------------------------
//----------------------------------------------------------------------------------

//Value publishing
extern bool publish;
extern BufferedPort<Bottle> plot_mu1p, plot_mu2p, plot_mu3p;
extern BufferedPort<Bottle> plot_mu1ep, plot_mu2ep;
extern BufferedPort<Bottle> plot_mu1bp, plot_mu2bp, plot_mu3bp;

//----------------------------------------------------------------------------------
//Functions ------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//Internal state -------------------------------------------------------------------
//Right arm
double internal_state_mu1p(vector<double> mu, vector<double> a);
double internal_state_mu2p(vector<double> mu, vector<double> a);
double internal_state_mu3p(vector<double> mu, vector<double> a);
//Left eye
double internal_state_mu1ep(vector<double> mu, vector<double> a);
double internal_state_mu2ep(vector<double> mu, vector<double> a);
//Left arm
double internal_state_mu1bp(vector<double> mu, vector<double> a);
double internal_state_mu2bp(vector<double> mu, vector<double> a);
double internal_state_mu3bp(vector<double> mu, vector<double> a);
