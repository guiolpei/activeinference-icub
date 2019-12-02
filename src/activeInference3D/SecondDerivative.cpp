/*
 * SecondDerivative.cpp
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

#include "SecondDerivative.h"

//----------------------------------------------------------------------------------
//Second derivative terms ----------------------------------------------------------
//----------------------------------------------------------------------------------

//Internal state -------------------------------------------------------------------
//Right arm
double internal_state_mu1p(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu1p = plot_mu1p.prepare();
		output_mu1p.clear();
		output_mu1p.addDouble(CTRL_RAD2DEG*mu[4]);
		plot_mu1p.write();
	}	

	return mu[4];
}

double internal_state_mu2p(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu2p = plot_mu2p.prepare();
		output_mu2p.clear();
		output_mu2p.addDouble(CTRL_RAD2DEG*mu[5]);
		plot_mu2p.write();
	}
	
	return mu[5];
}

double internal_state_mu3p(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu3p = plot_mu3p.prepare();
		output_mu3p.clear();
		output_mu3p.addDouble(CTRL_RAD2DEG*mu[6]);
		plot_mu3p.write();
	}
	
	return mu[6];
}

double internal_state_mu4p(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu4p = plot_mu4p.prepare();
		output_mu4p.clear();
		output_mu4p.addDouble(CTRL_RAD2DEG*mu[7]);
		plot_mu4p.write();
	}
	
	return mu[7];
}

//Left eye
double internal_state_mu1ep(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu1ep = plot_mu1ep.prepare();
		output_mu1ep.clear();
        output_mu1ep.addDouble(CTRL_RAD2DEG*mu[3]);
		plot_mu1ep.write();
	}	

    return mu[3];
}

double internal_state_mu2ep(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu2ep = plot_mu2ep.prepare();
		output_mu2ep.clear();
        output_mu2ep.addDouble(CTRL_RAD2DEG*mu[4]);
		plot_mu2ep.write();
	}
	
    return mu[4];
}

double internal_state_mu3ep(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu3ep = plot_mu3ep.prepare();
        output_mu3ep.clear();
        output_mu3ep.addDouble(CTRL_RAD2DEG*mu[5]);
        plot_mu3ep.write();
    }

    return mu[5];
}

//Left arm
double internal_state_mu1bp(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu1bp = plot_mu1bp.prepare();
        output_mu1bp.clear();
        output_mu1bp.addDouble(CTRL_RAD2DEG*mu[4]);
        plot_mu1bp.write();
    }

    return mu[4];
}

double internal_state_mu2bp(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu2bp = plot_mu2bp.prepare();
        output_mu2bp.clear();
        output_mu2bp.addDouble(CTRL_RAD2DEG*mu[5]);
        plot_mu2bp.write();
    }

    return mu[5];
}

double internal_state_mu3bp(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu3bp = plot_mu3bp.prepare();
        output_mu3bp.clear();
        output_mu3bp.addDouble(CTRL_RAD2DEG*mu[6]);
        plot_mu3bp.write();
    }

    return mu[6];
}

double internal_state_mu4bp(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu4bp = plot_mu4bp.prepare();
        output_mu4bp.clear();
        output_mu4bp.addDouble(CTRL_RAD2DEG*mu[7]);
        plot_mu4bp.write();
    }

    return mu[7];
}
