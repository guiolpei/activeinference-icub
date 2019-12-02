/*
 * Action.h
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

#include <vector> //vectors
#include <cmath> //math functions

#include "mdefs.h" //Mathematica equivalents

//Namespaces
using namespace std;

//----------------------------------------------------------------------------------
//Shared variables -----------------------------------------------------------------
//----------------------------------------------------------------------------------
//Additional factors
extern double k, kk;

//Right arm DH parameters
extern double a1, d2, a3, d3, d4, a6, d6, a7, d8, a10, d10;

//Right arm end effector position
extern double xh, yh, zh;

//Left eye DH parameters
extern double a1e, d2e, a3e, d3e, a4e, d5e, a6e, d6e, d7e;

//Camera parameters
extern double w, h, fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2;

//Attractor parameters
extern double ro1, ro2, ro3, ro4;

//global variables and vectors
extern yarp::sig::Vector qRAencoders, qRAendpos; //RA encoders 1, 2 and 3
extern double dT; //sampling time

//----------------------------------------------------------------------------------
//Functions ------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//Encoders -------------------------------------------------------------------------
double read_encoders_q1_action(vector<double> mu, vector<double> a);
double read_encoders_q1e_action(vector<double> mu, vector<double> a);
double read_encoders_q1b_action(vector<double> mu, vector<double> a);
double d_encoders_q1_a1(vector<double> mu, vector<double> a);
double d_encoders_q2_a1(vector<double> mu, vector<double> a);
double d_encoders_q3_a1(vector<double> mu, vector<double> a);
double d_encoders_q4_a1(vector<double> mu, vector<double> a);
double d_encoders_q1_a2(vector<double> mu, vector<double> a);
double d_encoders_q2_a2(vector<double> mu, vector<double> a);
double d_encoders_q3_a2(vector<double> mu, vector<double> a);
double d_encoders_q4_a2(vector<double> mu, vector<double> a);
double d_encoders_q1_a3(vector<double> mu, vector<double> a);
double d_encoders_q2_a3(vector<double> mu, vector<double> a);
double d_encoders_q3_a3(vector<double> mu, vector<double> a);
double d_encoders_q4_a3(vector<double> mu, vector<double> a);
double d_encoders_q1_a4(vector<double> mu, vector<double> a);
double d_encoders_q2_a4(vector<double> mu, vector<double> a);
double d_encoders_q3_a4(vector<double> mu, vector<double> a);
double d_encoders_q4_a4(vector<double> mu, vector<double> a);

//3D position ----------------------------------------------------------------------
//Right arm
double sense_3d_position_x_action(vector<double> mu, vector<double> a);
double d_3d_position_x_a1(vector<double> mu, vector<double> a);
double d_3d_position_y_a1(vector<double> mu, vector<double> a);
double d_3d_position_z_a1(vector<double> mu, vector<double> a);
double d_3d_position_x_a2(vector<double> mu, vector<double> a);
double d_3d_position_y_a2(vector<double> mu, vector<double> a);
double d_3d_position_z_a2(vector<double> mu, vector<double> a);
double d_3d_position_x_a3(vector<double> mu, vector<double> a);
double d_3d_position_y_a3(vector<double> mu, vector<double> a);
double d_3d_position_z_a3(vector<double> mu, vector<double> a);
double d_3d_position_x_a4(vector<double> mu, vector<double> a);
double d_3d_position_y_a4(vector<double> mu, vector<double> a);
double d_3d_position_z_a4(vector<double> mu, vector<double> a);
//Left arm
double sense_3d_position_x_action_b(vector<double> mu, vector<double> a);
double d_3d_position_x_a1_b(vector<double> mu, vector<double> a);
double d_3d_position_y_a1_b(vector<double> mu, vector<double> a);
double d_3d_position_z_a1_b(vector<double> mu, vector<double> a);
double d_3d_position_x_a2_b(vector<double> mu, vector<double> a);
double d_3d_position_y_a2_b(vector<double> mu, vector<double> a);
double d_3d_position_z_a2_b(vector<double> mu, vector<double> a);
double d_3d_position_x_a3_b(vector<double> mu, vector<double> a);
double d_3d_position_y_a3_b(vector<double> mu, vector<double> a);
double d_3d_position_z_a3_b(vector<double> mu, vector<double> a);
double d_3d_position_x_a4_b(vector<double> mu, vector<double> a);
double d_3d_position_y_a4_b(vector<double> mu, vector<double> a);
double d_3d_position_z_a4_b(vector<double> mu, vector<double> a);

//Vision ---------------------------------------------------------------------------
//Right arm
double d_vision_u_a1(vector<double> mu, vector<double> a);
double d_vision_v_a1(vector<double> mu, vector<double> a);
double d_vision_u_a2(vector<double> mu, vector<double> a);
double d_vision_v_a2(vector<double> mu, vector<double> a);
double d_vision_u_a3(vector<double> mu, vector<double> a);
double d_vision_v_a3(vector<double> mu, vector<double> a);
//Left arm
double d_vision_u_a1b(vector<double> mu, vector<double> a);
double d_vision_v_a1b(vector<double> mu, vector<double> a);
double d_vision_u_a2b(vector<double> mu, vector<double> a);
double d_vision_v_a2b(vector<double> mu, vector<double> a);
double d_vision_u_a3b(vector<double> mu, vector<double> a);
double d_vision_v_a3b(vector<double> mu, vector<double> a);
