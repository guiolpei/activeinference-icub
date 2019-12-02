/*
 * Action.cpp
 *
 * Description: Action function definitions.
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

#include "FirstDerivative.h" //first derivative functions
#include "Action.h"

//----------------------------------------------------------------------------------
//Action terms ---------------------------------------------------------------------
//----------------------------------------------------------------------------------

//encoders -------------------------------------------------------------------------
double read_encoders_q1_action(vector<double> mu, vector<double> a)
{
	//already read in previous function
	
	return qTencoders[0];
}

double read_encoders_q1e_action(vector<double> mu, vector<double> a)
{
	//already read in previous function
	
	return qLEencoders[0];
}

double read_encoders_q1b_action(vector<double> mu, vector<double> a)
{
    //already read in previous function

    return qTencoders[0];
}

double d_encoders_q1_a1(vector<double> mu, vector<double> a)
{
	double d_q1_a1 = dT;
	
	return d_q1_a1;
}

double d_encoders_q2_a1(vector<double> mu, vector<double> a)
{
	double d_q2_a1 = 0;
	
	return d_q2_a1;
}

double d_encoders_q3_a1(vector<double> mu, vector<double> a)
{
	double d_q3_a1 = 0;
	
	return d_q3_a1;
}

double d_encoders_q4_a1(vector<double> mu, vector<double> a)
{
	double d_q4_a1 = 0;
	
	return d_q4_a1;
}

double d_encoders_q1_a2(vector<double> mu, vector<double> a)
{
	double d_q1_a2 = 0;
	
	return d_q1_a2;
}

double d_encoders_q2_a2(vector<double> mu, vector<double> a)
{
	double d_q2_a2 = dT;
	
	return d_q2_a2;
}

double d_encoders_q3_a2(vector<double> mu, vector<double> a)
{
	double d_q3_a2 = 0;
	
	return d_q3_a2;
}

double d_encoders_q4_a2(vector<double> mu, vector<double> a)
{
	double d_q4_a2 = 0;
	
	return d_q4_a2;
}

double d_encoders_q1_a3(vector<double> mu, vector<double> a)
{
	double d_q1_a3 = 0;
	
	return d_q1_a3;
}

double d_encoders_q2_a3(vector<double> mu, vector<double> a)
{
	double d_q2_a3 = 0;
	
	return d_q2_a3;
}

double d_encoders_q3_a3(vector<double> mu, vector<double> a)
{
	double d_q3_a3 = dT;
	
	return d_q3_a3;
}

double d_encoders_q4_a3(vector<double> mu, vector<double> a)
{
	double d_q4_a3 = 0;
	
	return d_q4_a3;
}

double d_encoders_q1_a4(vector<double> mu, vector<double> a)
{
	double d_q1_a4 = 0;
	
	return d_q1_a4;
}

double d_encoders_q2_a4(vector<double> mu, vector<double> a)
{
	double d_q2_a4 = 0;
	
	return d_q2_a4;
}

double d_encoders_q3_a4(vector<double> mu, vector<double> a)
{
	double d_q3_a4 = 0;
	
	return d_q3_a4;
}

double d_encoders_q4_a4(vector<double> mu, vector<double> a)
{
	double d_q4_a4 = dT;
	
	return d_q4_a4;
}

//3d position ----------------------------------------------------------------------
//Right arm
double sense_3d_position_x_action(vector<double> mu, vector<double> a)
{
	//already read in previous function
	
	return re_xpos[0];
}

double d_3d_position_x_a1(vector<double> mu, vector<double> a)
{
	double d_x_a1;
	
	d_x_a1 = dT * d_3d_position_x_mu1(mu,a);
	
	return d_x_a1;
}

double d_3d_position_y_a1(vector<double> mu, vector<double> a)
{
	double d_y_a1;
	
	d_y_a1 = dT * d_3d_position_y_mu1(mu,a);
	
	return d_y_a1;
}

double d_3d_position_z_a1(vector<double> mu, vector<double> a)
{
	double d_z_a1;
	
	d_z_a1 = dT * d_3d_position_z_mu1(mu,a);
	
	return d_z_a1;
}

double d_3d_position_x_a2(vector<double> mu, vector<double> a)
{
	double d_x_a2;
	
	d_x_a2 = dT * d_3d_position_x_mu2(mu,a);
	
	return d_x_a2;
}

double d_3d_position_y_a2(vector<double> mu, vector<double> a)
{
	double d_y_a2;
	
	d_y_a2 = dT * d_3d_position_y_mu2(mu,a);
	
	return d_y_a2;
}

double d_3d_position_z_a2(vector<double> mu, vector<double> a)
{
	double d_z_a2;
	
	d_z_a2 = dT * d_3d_position_z_mu2(mu,a);
	
	return d_z_a2;
}

double d_3d_position_x_a3(vector<double> mu, vector<double> a)
{
	double d_x_a3;
	
	d_x_a3 = dT * d_3d_position_x_mu3(mu,a);
	
	return d_x_a3;
}

double d_3d_position_y_a3(vector<double> mu, vector<double> a)
{
	double d_y_a3;
	
	d_y_a3 = dT * d_3d_position_y_mu3(mu,a);
	
	return d_y_a3;
}

double d_3d_position_z_a3(vector<double> mu, vector<double> a)
{
	double d_z_a3;
	
	d_z_a3 = dT * d_3d_position_z_mu3(mu,a);
	
	return d_z_a3;
}

double d_3d_position_x_a4(vector<double> mu, vector<double> a)
{
	double d_x_a4;
	
	d_x_a4 = dT * d_3d_position_x_mu4(mu,a);
	
	return d_x_a4;
}

double d_3d_position_y_a4(vector<double> mu, vector<double> a)
{
	double d_y_a4;
	
	d_y_a4 = dT * d_3d_position_y_mu4(mu,a);
	
	return d_y_a4;
}

double d_3d_position_z_a4(vector<double> mu, vector<double> a)
{
	double d_z_a4;
	
	d_z_a4 = dT * d_3d_position_z_mu4(mu,a);
	
	return d_z_a4;
}

//Left arm
double sense_3d_position_x_action_b(vector<double> mu, vector<double> a)
{
	//already read in previous function
	
	return le_xpos[0];
}

double d_3d_position_x_a1_b(vector<double> mu, vector<double> a)
{
	double d_x_a1;
	
	d_x_a1 = dT * d_3d_position_x_mu1_b(mu,a);
	
	return d_x_a1;
}

double d_3d_position_y_a1_b(vector<double> mu, vector<double> a)
{
	double d_y_a1;
	
	d_y_a1 = dT * d_3d_position_y_mu1_b(mu,a);
	
	return d_y_a1;
}

double d_3d_position_z_a1_b(vector<double> mu, vector<double> a)
{
	double d_z_a1;
	
	d_z_a1 = dT * d_3d_position_z_mu1_b(mu,a);
	
	return d_z_a1;
}

double d_3d_position_x_a2_b(vector<double> mu, vector<double> a)
{
	double d_x_a2;
	
	d_x_a2 = dT * d_3d_position_x_mu2_b(mu,a);
	
	return d_x_a2;
}

double d_3d_position_y_a2_b(vector<double> mu, vector<double> a)
{
	double d_y_a2;
	
	d_y_a2 = dT * d_3d_position_y_mu2_b(mu,a);
	
	return d_y_a2;
}

double d_3d_position_z_a2_b(vector<double> mu, vector<double> a)
{
	double d_z_a2;
	
	d_z_a2 = dT * d_3d_position_z_mu2_b(mu,a);
	
	return d_z_a2;
}

double d_3d_position_x_a3_b(vector<double> mu, vector<double> a)
{
	double d_x_a3;
	
	d_x_a3 = dT * d_3d_position_x_mu3_b(mu,a);
	
	return d_x_a3;
}

double d_3d_position_y_a3_b(vector<double> mu, vector<double> a)
{
	double d_y_a3;
	
	d_y_a3 = dT * d_3d_position_y_mu3_b(mu,a);
	
	return d_y_a3;
}

double d_3d_position_z_a3_b(vector<double> mu, vector<double> a)
{
	double d_z_a3;
	
	d_z_a3 = dT * d_3d_position_z_mu3_b(mu,a);
	
	return d_z_a3;
}

double d_3d_position_x_a4_b(vector<double> mu, vector<double> a)
{
	double d_x_a4;
	
	d_x_a4 = dT * d_3d_position_x_mu4_b(mu,a);
	
	return d_x_a4;
}

double d_3d_position_y_a4_b(vector<double> mu, vector<double> a)
{
	double d_y_a4;
	
	d_y_a4 = dT * d_3d_position_y_mu4_b(mu,a);
	
	return d_y_a4;
}

double d_3d_position_z_a4_b(vector<double> mu, vector<double> a)
{
	double d_z_a4;
	
	d_z_a4 = dT * d_3d_position_z_mu4_b(mu,a);
	
	return d_z_a4;
}

//Vision ---------------------------------------------------------------------------
//Right arm
double d_vision_u_a1(vector<double> mu, vector<double> a)
{
	double d_u_a1;
	
	d_u_a1 = dT * d_vision_u_mu1(mu,a);
	
	return d_u_a1;
}

double d_vision_v_a1(vector<double> mu, vector<double> a)
{
	double d_v_a1;
	
	d_v_a1 = dT * d_vision_v_mu1(mu,a);
	
	return d_v_a1;	
}

double d_vision_u_a2(vector<double> mu, vector<double> a)
{
	double d_u_a2;
	
	d_u_a2 = dT * d_vision_u_mu2(mu,a);
	
	return d_u_a2;
}

double d_vision_v_a2(vector<double> mu, vector<double> a)
{
	double d_v_a2;
	
	d_v_a2 = dT * d_vision_v_mu2(mu,a);
	
	return d_v_a2;
}

double d_vision_u_a3(vector<double> mu, vector<double> a)
{
	double d_u_a3;
	
	d_u_a3 = dT * d_vision_u_mu3(mu,a);
	
	return d_u_a3;
}

double d_vision_v_a3(vector<double> mu, vector<double> a)
{
	double d_v_a3;
	
	d_v_a3 = dT * d_vision_v_mu3(mu,a);
	
	return d_v_a3;
}

//Left arm
double d_vision_u_a1b(vector<double> mu, vector<double> a)
{
    double d_u_a1;

    d_u_a1 = dT * d_vision_u_mu1_b(mu,a);

    return d_u_a1;
}

double d_vision_v_a1b(vector<double> mu, vector<double> a)
{
    double d_v_a1;

    d_v_a1 = dT * d_vision_v_mu1_b(mu,a);

    return d_v_a1;
}

double d_vision_u_a2b(vector<double> mu, vector<double> a)
{
    double d_u_a2;

    d_u_a2 = dT * d_vision_u_mu2_b(mu,a);

    return d_u_a2;
}

double d_vision_v_a2b(vector<double> mu, vector<double> a)
{
    double d_v_a2;

    d_v_a2 = dT * d_vision_v_mu2_b(mu,a);

    return d_v_a2;
}

double d_vision_u_a3b(vector<double> mu, vector<double> a)
{
    double d_u_a3;

    d_u_a3 = dT * d_vision_u_mu3_b(mu,a);

    return d_u_a3;
}

double d_vision_v_a3b(vector<double> mu, vector<double> a)
{
    double d_v_a3;

    d_v_a3 = dT * d_vision_v_mu3_b(mu,a);

    return d_v_a3;
}
