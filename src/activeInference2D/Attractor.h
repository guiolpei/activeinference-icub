/*
 * Attractor.h
 *
 * Description: Attractor function definitions.
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
#include <mutex> //mutex
#include <eigen3/Eigen/Dense> //eigen dense matrix algebra

#include "mdefs.h" //Mathematica equivalents

//Namespaces
using namespace std;
using namespace Eigen;

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
extern double ro1_head;

extern mutex a_mutex;

//Publishers
extern BufferedPort<Bottle> plot_center_x, plot_center_y;

//Attractor visual recognition
extern yarp::sig::Vector att_pos;

//right arm robot algebra
extern Matrix4d ra_rtm; //rototranslational matrix for right arm
extern Vector4d ra_repv; //relative end effector position vector for right arm
extern Vector4d ra_epv; //end effector position vector for right arm
extern Matrix4d ra_d1rtm; //derivative of rototranslational matrix wrt mu1
extern Matrix4d ra_d2rtm; //derivative of rototranslational matrix wrt mu2
extern Vector4d ra_d1epv; //derivative of end effector position vector wrt mu1
extern Vector4d ra_d2epv; //derivative of end effector position vector wrt mu2
extern Matrix4d ra_d11rtm; //derivative of rototranslational matrix wrt mu1 mu1
extern Matrix4d ra_d22rtm; //derivative of rototranslational matrix wrt mu2 mu2
extern Matrix4d ra_d12rtm; //derivative of rototranslational matrix wrt mu1 mu2
extern Vector4d ra_d11epv; //derivative of end effector position vector wrt mu1 mu1
extern Vector4d ra_d22epv; //derivative of end effector position vector wrt mu2 mu2
extern Vector4d ra_d12epv; //derivative of end effector position vector wrt mu1 mu2

//left eye robot algebra
extern Matrix4d le_rtm; //rototranslational matrix for left eye
extern Vector4d le_repv; //relative end effector position vector for left eye
extern Vector4d le_epv; //end effector position vector for left eye
extern Matrix4d le_rtm_inv; //inverse rototranslational matrix for left eye

//camera robot algebra
extern Vector4d cam_projection; //projection vector for the end effector of the right arm
extern Vector4d d1_cam_projection; //derivative wrt mu1
extern Vector4d d2_cam_projection; //derivative wrt mu2
extern Vector4d d11_cam_projection; //derivative wrt mu1 mu1
extern Vector4d d22_cam_projection; //derivative wrt mu2 mu2
extern Vector4d d12_cam_projection; //derivative wrt mu1 mu2
extern double xp, yp, r2, ck, px, pxx, py, pyy; //camera intermediate results
extern double d1xp, d1yp, d1r2, d1k, d1px, d1pxx, d1py, d1pyy; //camera intermediate results wrt mu1
extern double d2xp, d2yp, d2r2, d2k, d2px, d2pxx, d2py, d2pyy; //camera intermediate results wrt mu2
extern double d11xp, d11yp, d11r2, d11k, d11px, d11pxx, d11py, d11pyy; //camera intermediate results wrt mu1 mu1
extern double d22xp, d22yp, d22r2, d22k, d22px, d22pxx, d22py, d22pyy; //camera intermediate results wrt mu2 mu2
extern double d12xp, d12yp, d12r2, d12k, d12px, d12pxx, d12py, d12pyy; //camera intermediate results wrt mu1 mu2

//attractor algebra
extern Vector4d ra_3d_jacobian_mu1; //mu1 part of end effector velocity jacobian
extern Vector4d ra_3d_jacobian_mu2; //mu2 part of end effector velocity jacobian
extern Vector4d ra_3d_jacobian_mu3; //mu3 part of end effector velocity jacobian
extern Vector4d ra_3d_jacobian_mu1_origin; //mu1 part of end effector origin velocity jacobian
extern Vector4d ra_3d_jacobian_mu2_origin; //mu2 part of end effector origin velocity jacobian
extern Vector4d ra_3d_jacobian_mu3_origin; //mu3 part of end effector origin velocity jacobian
extern Vector4d la_3d_jacobian_mu1; //mu1 part of end effector velocity jacobian
extern Vector4d la_3d_jacobian_mu2; //mu2 part of end effector velocity jacobian
extern Vector4d la_3d_jacobian_mu3; //mu3 part of end effector velocity jacobian
extern Vector4d la_3d_jacobian_mu1_origin; //mu1 part of end effector origin velocity jacobian
extern Vector4d la_3d_jacobian_mu2_origin; //mu2 part of end effector origin velocity jacobian
extern Vector4d la_3d_jacobian_mu3_origin; //mu3 part of end effector origin velocity jacobian
extern Vector4d ra_3d_hessian_mu11; //mu11 part of end effector velocity hessian
extern Vector4d ra_3d_hessian_mu12; //mu12 part of end effector velocity hessian
extern Vector4d ra_3d_hessian_mu22; //mu22 part of end effector velocity hessian
extern Vector4d ra_3d_hessian_mu23; //mu23 part of end effector velocity hessian
extern Vector4d ra_3d_hessian_mu33; //mu33 part of end effector velocity hessian
extern Vector4d ra_3d_hessian_mu13; //mu13 part of end effector velocity hessian
extern Vector4d ra_3d_hessian_mu11_origin; //mu11 part of end effector origin velocity hessian
extern Vector4d ra_3d_hessian_mu12_origin; //mu12 part of end effector origin velocity hessian
extern Vector4d ra_3d_hessian_mu22_origin; //mu22 part of end effector origin velocity hessian
extern Vector4d ra_3d_hessian_mu23_origin; //mu23 part of end effector origin velocity hessian
extern Vector4d ra_3d_hessian_mu33_origin; //mu33 part of end effector origin velocity hessian
extern Vector4d ra_3d_hessian_mu13_origin; //mu13 part of end effector origin velocity hessian
extern Eigen::Matrix<double, 3, 3> ra_3d_jacobian;  //end effector velocity jacobian
extern Eigen::Matrix<double, 3, 3> ra_3d_jacobian_origin;  //end effector origin velocity jacobian
extern Eigen::Matrix<double, 3, 3> la_3d_jacobian;  //end effector velocity jacobian
extern Eigen::Matrix<double, 3, 3> la_3d_jacobian_origin;  //end effector origin velocity jacobian
extern Eigen::Matrix<double, 3, 3> ra_3d_hessian_mu1;  //velocity hessian for wrt mu1
extern Eigen::Matrix<double, 3, 3> ra_3d_hessian_mu2;  //velocity hessian for wrt mu2
extern Eigen::Matrix<double, 3, 3> ra_3d_hessian_mu3;  //velocity hessian for wrt mu2
extern Eigen::Matrix<double, 3, 3> ra_3d_hessian_mu1_origin;  //velocity origin hessian for wrt mu1
extern Eigen::Matrix<double, 3, 3> ra_3d_hessian_mu2_origin;  //velocity origin hessian for wrt mu2
extern Eigen::Matrix<double, 3, 3> ra_3d_hessian_mu3_origin;  //velocity origin hessian for wrt mu2
extern Vector3d ra_3d_attr; //right arm 3d attractor position vector
extern Vector3d ra_3d_f_attr; //right arm 3d attractor joint vector
extern Vector3d d1_3d_f_attractor; //partial derivative vector for the 3d attractor wrt mu1
extern Vector3d d2_3d_f_attractor; //partial derivative vector for the 3d attractor wrt mu2
extern Vector3d d3_3d_f_attractor; //partial derivative vector for the 3d attractor wrt mu3
extern Vector4d ra_visual_hessian_mu11; //mu11 part of end effector visual hessian
extern Vector4d ra_visual_hessian_mu12; //mu12 part of end effector visual hessian
extern Vector4d ra_visual_hessian_mu22; //mu22 part of end effector visual hessian
extern Vector4d ra_visual_hessian_mu23; //mu23 part of end effector visual hessian
extern Vector4d ra_visual_hessian_mu33; //mu33 part of end effector visual hessian
extern Vector4d ra_visual_hessian_mu13; //mu13 part of end effector visual hessian
extern Eigen::Matrix<double, 2, 3> ra_visual_jacobian;  //right arm end effector visual jacobian
extern Eigen::Matrix<double, 2, 2> ra_visual_jacobian_head;  //camera visual jacobian
extern Eigen::Matrix<double, 2, 3> la_visual_jacobian;  //left arm end effector visual jacobian
extern Eigen::Matrix<double, 2, 3> ra_visual_hessian_mu1;  //velocity hessian for wrt mu1
extern Eigen::Matrix<double, 2, 3> ra_visual_hessian_mu2;  //velocity hessian for wrt mu2
extern Eigen::Matrix<double, 2, 3> ra_visual_hessian_mu3;  //velocity hessian for wrt mu2
extern Vector2d ra_visual_attr; //right arm visual attractor position vector
extern Vector3d ra_visual_f_attr; //right arm visual attractor joint vector
extern Vector3d d1_visual_f_attractor; //partial derivative vector for the visual attractor wrt mu1
extern Vector3d d2_visual_f_attractor; //partial derivative vector for the visual attractor wrt mu2
extern Vector3d d3_visual_f_attractor; //partial derivative vector for the visual attractor wrt mu2
extern Vector2d head_visual_attr; //head visual attractor position vector
extern Vector2d head_visual_f_attr; //head visual attractor joint vector
extern Vector2d la_visual_attr; //left arm visual attractor position vector
extern Vector3d la_visual_f_attr; //left arm visual attractor joint vector

//----------------------------------------------------------------------------------
//Functions ------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//jacobians and hessians -----------------------------------------------------------
//Right arm
void calc_3d_jacobian(vector<double> mu, vector<double> a);
void calc_3d_jacobian_origin(vector<double> mu, vector<double> a);
void calc_3d_hessian_terms(vector<double> mu, vector<double> a);
void calc_3d_hessian_origin(vector<double> mu, vector<double> a);
void calc_3d_hessian_mu1(vector<double> mu, vector<double> a);
void calc_3d_hessian_mu2(vector<double> mu, vector<double> a);
void calc_3d_hessian_mu3(vector<double> mu, vector<double> a);
void calc_3d_hessian_mu1_origin(vector<double> mu, vector<double> a);
void calc_3d_hessian_mu2_origin(vector<double> mu, vector<double> a);
void calc_3d_hessian_mu3_origin(vector<double> mu, vector<double> a);
//Left arm
void calc_3d_jacobian_b(vector<double> mu, vector<double> a);
void calc_3d_jacobian_origin_b(vector<double> mu, vector<double> a);
//Visual
void calc_visual_jacobian(vector<double> mu, vector<double> a);
void calc_visual_jacobian_head(vector<double> mu, vector<double> a);
void calc_visual_jacobian_b(vector<double> mu, vector<double> a);
void calc_visual_hessian_terms(vector<double> mu, vector<double> a);
void calc_visual_hessian_mu1(vector<double> mu, vector<double> a);
void calc_visual_hessian_mu2(vector<double> mu, vector<double> a);
void calc_visual_hessian_mu3(vector<double> mu, vector<double> a);

//3d -------------------------------------------------------------------------------
void calc_attractor_3d_vector(vector<double> mu, vector<double> a);
double calc_attractor_3d_f1(vector<double> mu, vector<double> a);
double calc_attractor_3d_f2(vector<double> mu, vector<double> a);
double calc_attractor_3d_f3(vector<double> mu, vector<double> a);
double d_attractor_3d_f1_mu1(vector<double> mu, vector<double> a);
double d_attractor_3d_f2_mu1(vector<double> mu, vector<double> a);
double d_attractor_3d_f3_mu1(vector<double> mu, vector<double> a);
double d_attractor_3d_f1_mu2(vector<double> mu, vector<double> a);
double d_attractor_3d_f2_mu2(vector<double> mu, vector<double> a);
double d_attractor_3d_f3_mu2(vector<double> mu, vector<double> a);
double d_attractor_3d_f1_mu3(vector<double> mu, vector<double> a);
double d_attractor_3d_f2_mu3(vector<double> mu, vector<double> a);
double d_attractor_3d_f3_mu3(vector<double> mu, vector<double> a);

//Visual ---------------------------------------------------------------------------
void calc_attractor_visual_vector(vector<double> mu, vector<double> a);
double calc_attractor_visual_f1(vector<double> mu, vector<double> a);
double calc_attractor_visual_f2(vector<double> mu, vector<double> a);
double calc_attractor_visual_f3(vector<double> mu, vector<double> a);
double d_attractor_visual_f1_mu1(vector<double> mu, vector<double> a);
double d_attractor_visual_f2_mu1(vector<double> mu, vector<double> a);
double d_attractor_visual_f3_mu1(vector<double> mu, vector<double> a);
double d_attractor_visual_f1_mu2(vector<double> mu, vector<double> a);
double d_attractor_visual_f2_mu2(vector<double> mu, vector<double> a);
double d_attractor_visual_f3_mu2(vector<double> mu, vector<double> a);
double d_attractor_visual_f1_mu3(vector<double> mu, vector<double> a);
double d_attractor_visual_f2_mu3(vector<double> mu, vector<double> a);
double d_attractor_visual_f3_mu3(vector<double> mu, vector<double> a);
void calc_attractor_visual_vector_b(vector<double> mu, vector<double> a);
double calc_attractor_visual_f1b(vector<double> mu, vector<double> a);
double calc_attractor_visual_f2b(vector<double> mu, vector<double> a);
double calc_attractor_visual_f3b(vector<double> mu, vector<double> a);

//Visual recognition ---------------------------------------------------------------
void calc_attractor_hsv_vector(vector<double> mu, vector<double> a);
double calc_attractor_hsv_f1(vector<double> mu, vector<double> a);
double calc_attractor_hsv_f2(vector<double> mu, vector<double> a);
double calc_attractor_hsv_f3(vector<double> mu, vector<double> a);
void calc_attractor_hsv_vector_b(vector<double> mu, vector<double> a);
double calc_attractor_hsv_f1b(vector<double> mu, vector<double> a);
double calc_attractor_hsv_f2b(vector<double> mu, vector<double> a);
double calc_attractor_hsv_f3b(vector<double> mu, vector<double> a);

//head -----------------------------------------------------------------------------
void calc_attractor_head_vector(vector<double> mu, vector<double> a);
double calc_attractor_head_f1(vector<double> mu, vector<double> a);
double calc_attractor_head_f2(vector<double> mu, vector<double> a);
