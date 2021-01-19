/*
 * FirstDerivative.h
 *
 * Description: First derivative function definitions.
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

#include "../ICubControl.h"
#include "../ICubKinematics.h"

#include <vector> //vectors
#include <cmath> //math functions
#include <mutex> //mutex
#include <eigen3/Eigen/Dense> //eigen dense matrix algebra
#include <random>

#include "mdefs.h" //Mathematica equivalents

//Yarp
#include <yarp/os/all.h> //OS
#include <yarp/sig/all.h> //Signal processing
#include <yarp/math/Math.h> //Math
#include <iCub/iKin/iKinFwd.h> //iKin forward kinematics

//Namespaces
using namespace std;
using namespace Eigen;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

//----------------------------------------------------------------------------------
//Shared variables -----------------------------------------------------------------
//----------------------------------------------------------------------------------
//Additional factors
extern double k, kk;

//Right arm DH parameters
extern double a1, d2, a3, d3, d4, a6, d6, a7, d8, a10, d10;

//Right arm rotational DOF
extern double ra_q1, ra_q2, ra_q3, ra_q4, ra_q5, ra_q6, ra_q7, ra_q8, ra_q9, ra_q10;

//Right arm angle bias
extern double ra_q1b, ra_q2b, ra_q3b, ra_q4b, ra_q5b, ra_q6b, ra_q7b, ra_q8b, ra_q9b, ra_q10b;

//Right arm end effector position
extern double xh, yh, zh;

//Left arm end effector position
extern double xhb, yhb, zhb;

//Left eye DH parameters
extern double a1e, d2e, a3e, d3e, a4e, d5e, a6e, d6e, d7e;
extern double d7ex; //right eye

//Left eye rotational DOF
extern double le_q1, le_q2, le_q3, le_q4, le_q5, le_q6, le_q7, le_q8;

//Left eye angle bias
extern double le_q1b, le_q2b, le_q3b, le_q4b, le_q5b, le_q6b, le_q7b, le_q8b;

//Left arm DH parameters
extern double a1b, d2b, a3b, d3b, d4b, a6b, d6b, a7b, d8b, a10b, d10b;

//Left arm rotational DOF
extern double la_q1, la_q2, la_q3, la_q4, la_q5, la_q6, la_q7, la_q8, la_q9, la_q10;

//Left arm angle bias
extern double la_q1b, la_q2b, la_q3b, la_q4b, la_q5b, la_q6b, la_q7b, la_q8b, la_q9b, la_q10b;

//Camera parameters
extern double w, h, fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2;
extern double fx_L, fy_L, cx_L, cy_L, k1_L, k2_L, k3_L, k4_L, k5_L, k6_L, p1_L, p2_L;
extern double fx_R, fy_R, cx_R, cy_R, k1_R, k2_R, k3_R, k4_R, k5_R, k6_R, p1_R, p2_R;

//Attractor parameters
extern double ro1, ro2, ro3, ro4;

//Access mutex
extern mutex a_mutex;

//Gaussian noise
extern mt19937 *gen; //Mersenne twister PRNG
extern normal_distribution<double>* ndis; 

//Value publishing for right and left arm and head
extern bool publish;
extern BufferedPort<Bottle> plot_q1, plot_q2, plot_q3, plot_q4;
extern BufferedPort<Bottle> plot_mu1, plot_mu2, plot_mu3, plot_mu4;
extern BufferedPort<Bottle> plot_a1, plot_a2, plot_a3, plot_a4;
extern BufferedPort<Bottle> plot_l_u, plot_l_v;
extern BufferedPort<Bottle> plot_calc_l_u, plot_calc_l_v;
extern BufferedPort<Bottle> plot_r_u, plot_r_v;
extern BufferedPort<Bottle> plot_calc_r_u, plot_calc_r_v;
extern BufferedPort<Bottle> plot_q1e, plot_q2e, plot_q3e;
extern BufferedPort<Bottle> plot_mu1e, plot_mu2e, plot_mu3e;
extern BufferedPort<Bottle> plot_mu1ep, plot_mu2ep, plot_mu3ep;
extern BufferedPort<Bottle> plot_a1e, plot_a2e, plot_a3e;
extern BufferedPort<Bottle> plot_q1b, plot_q2b, plot_q3b, plot_q4b;
extern BufferedPort<Bottle> plot_mu1b, plot_mu2b, plot_mu3b, plot_mu4b;
extern BufferedPort<Bottle> plot_a1b, plot_a2b, plot_a3b, plot_a4b;
extern BufferedPort<Bottle> plot_ub, plot_vb;
extern BufferedPort<Bottle> plot_calc_ub, plot_calc_vb;

//instanciated classes
extern ICubControl *iCubRightArmCtrl; //control and encoders
extern ICubControl *iCubHeadCtrl;
extern ICubControl *iCubLeftArmCtrl;
extern ICubControl *iCubTorsoCtrl;
extern ICubKinematics *iCubRightArmKin; //kinematics
extern ICubKinematics *iCubLeftEyeKin;
extern ICubKinematics *iCubLeftArmKin;
extern ICubKinematics *iCubTorsoKin;

//global variables and vectors
extern vector<double> initialPrior;
extern vector<double> initialPriorb;
extern yarp::sig::Vector qRAencoders, qRAendpos; //RA encoders 1, 2 and 3
extern yarp::sig::Vector qLAencoders, qLAendpos; //LA encoders 1, 2 and 3
extern yarp::sig::Vector qLEencoders; //LE encoders 0 and 2
extern yarp::sig::Vector qTencoders; //T encoders 0/2
extern yarp::sig::Vector re_pos; //right arm end-effector position relative to the left eye
extern yarp::sig::Vector re_xpos; //right arm end-effector position in absolute coordinates
extern yarp::sig::Vector le_pos; //left arm end-effector position relative to the left eye
extern yarp::sig::Vector le_xpos; //left arm end-effector position in absolute coordinates
extern yarp::sig::Vector att_pos; //attractor position relative to the left eye
extern yarp::sig::Vector att_xpos; //attractor position in absolute coordinates
extern yarp::sig::Vector calc_vision_pos_l, calc_vision_pos_r; //right arm calculated position in visual field
extern yarp::sig::Vector calc_vision_pos_b; //left arm calculated position in visual field

//right arm robot algebra
extern Matrix4d ra_rtm; //rototranslational matrix for right arm
extern Vector4d ra_repv; //relative end effector position vector for right arm
extern Vector4d ra_epv; //end effector position vector for right arm
extern Matrix4d ra_d1rtm; //derivative of rototranslational matrix wrt mu1
extern Matrix4d ra_d2rtm; //derivative of rototranslational matrix wrt mu2
extern Matrix4d ra_d3rtm; //derivative of rototranslational matrix wrt mu3
extern Matrix4d ra_d4rtm; //derivative of rototranslational matrix wrt mu4
extern Vector4d ra_d1epv; //derivative of end effector position vector wrt mu1
extern Vector4d ra_d2epv; //derivative of end effector position vector wrt mu2
extern Vector4d ra_d3epv; //derivative of end effector position vector wrt mu3
extern Vector4d ra_d4epv; //derivative of end effector position vector wrt mu4
extern Matrix4d ra_d11rtm; //derivative of rototranslational matrix wrt mu1 mu1
extern Matrix4d ra_d22rtm; //derivative of rototranslational matrix wrt mu2 mu2
extern Matrix4d ra_d33rtm; //derivative of rototranslational matrix wrt mu3 mu3
extern Matrix4d ra_d12rtm; //derivative of rototranslational matrix wrt mu1 mu2
extern Matrix4d ra_d13rtm; //derivative of rototranslational matrix wrt mu1 mu3
extern Matrix4d ra_d23rtm; //derivative of rototranslational matrix wrt mu2 mu3
extern Vector4d ra_d11epv; //derivative of end effector position vector wrt mu1 mu1
extern Vector4d ra_d22epv; //derivative of end effector position vector wrt mu2 mu2
extern Vector4d ra_d33epv; //derivative of end effector position vector wrt mu3 mu3
extern Vector4d ra_d12epv; //derivative of end effector position vector wrt mu1 mu2
extern Vector4d ra_d13epv; //derivative of end effector position vector wrt mu1 mu3
extern Vector4d ra_d23epv; //derivative of end effector position vector wrt mu2 mu3

//left eye robot algebra
extern Matrix4d le_rtm; //rototranslational matrix for left eye
extern Matrix4d le_rtm_h; //rototranslational matrix for left eye
extern Matrix4d le_rtm_b; //rototranslational matrix for left eye
extern Vector4d le_repv; //relative end effector position vector for left eye
extern Vector4d le_epv; //end effector position vector for left eye
extern Matrix4d le_rtm_inv; //inverted rototranslational matrix for left eye
extern Matrix4d le_rtm_inv_b; //inverted rototranslational matrix for left eye
extern Matrix4d inv_le_rtm; //inverse rototranslational matrix for left eye
extern Matrix4d d1_inv_le_rtm; //derivative of inverse rototranslational matrix for left eye wrt mu1e
extern Matrix4d d2_inv_le_rtm; //derivative of inverse rototranslational matrix for left eye wrt mu2e
extern Matrix4d d3_inv_le_rtm; //derivative of inverse rototranslational matrix for left eye wrt mu3e

//right eye
extern Matrix4d re_rtm; //rototranslational matrix for right eye
extern Matrix4d re_rtm_inv; //inverted rototranslational matrix for right eye

//left arm robot algebra
extern Matrix4d la_rtm; //rototranslational matrix for left arm
extern Vector4d la_repv; //relative end effector position vector for left arm
extern Vector4d la_epv; //end effector position vector for left arm
extern Matrix4d la_d1rtm; //derivative of rototranslational matrix wrt mu1
extern Matrix4d la_d2rtm; //derivative of rototranslational matrix wrt mu2
extern Matrix4d la_d3rtm; //derivative of rototranslational matrix wrt mu3
extern Matrix4d la_d4rtm; //derivative of rototranslational matrix wrt mu4
extern Vector4d la_d1epv; //derivative of end effector position vector wrt mu1
extern Vector4d la_d2epv; //derivative of end effector position vector wrt mu2
extern Vector4d la_d3epv; //derivative of end effector position vector wrt mu3
extern Vector4d la_d4epv; //derivative of end effector position vector wrt mu4

//attractor algebra
extern Vector3d attr_3d_pos; //world 3d attractor position

//camera robot algebra
extern Vector4d cam_projection; //projection vector for the end effector of the right arm
extern Vector4d cam_projection_L; //left eye projection vector for the end effector of the right arm
extern Vector4d cam_projection_R; //right eye projection vector for the end effector of the right arm
extern Vector4d attr_projection_L; //left eye projection vector for the virtual attractor
extern Vector4d attr_projection_R; //right eye projection vector for the virtual attractor
extern Vector4d d1_cam_projection; //right arm derivative wrt mu1
extern Vector4d d2_cam_projection; //right arm derivative wrt mu2
extern Vector4d d3_cam_projection; //right arm derivative wrt mu3
extern Vector4d d11_cam_projection; //right arm derivative wrt mu1 mu1
extern Vector4d d22_cam_projection; //right arm derivative wrt mu2 mu2
extern Vector4d d33_cam_projection; //right arm derivative wrt mu2 mu2
extern Vector4d d12_cam_projection; //right arm derivative wrt mu1 mu2
extern Vector4d d13_cam_projection; //right arm derivative wrt mu1 mu3
extern Vector4d d23_cam_projection; //right arm derivative wrt mu2 mu3
extern Vector4d le_projection; //projection vector for the left eye (head) reference
extern Vector4d le_projection; //camera centerpoint projection
extern Vector4d r_le_projection; //relative world camera centerpoint projection
extern Vector4d d1_le_projection; //head derivative wrt mu1e
extern Vector4d d2_le_projection; //head derivative wrt mu2e
extern Vector4d d3_le_projection; //head derivative wrt mu3e
extern Vector4d d11_le_projection; //head derivative wrt mu1 mu1
extern Vector4d d22_le_projection; //head derivative wrt mu2 mu2
extern Vector4d d12_le_projection; //head derivative wrt mu1 mu2
extern Vector4d cam_projection_b; //projection vector for the end effector of the left arm
extern Vector4d d1_cam_projection_b; //left arm derivative wrt mu1
extern Vector4d d2_cam_projection_b; //left arm derivative wrt mu2
extern Vector4d d3_cam_projection_b; //left arm derivative wrt mu3
extern double xp, yp, r2, ck, px, pxx, py, pyy; //right arm camera intermediate results
extern double xp_L, yp_L, r2_L, ck_L, px_L, pxx_L, py_L, pyy_L; //right arm left eye camera intermediate results
extern double xp_R, yp_R, r2_R, ck_R, px_R, pxx_R, py_R, pyy_R; //right arm right eye camera intermediate results
extern double xp_attr_L, yp_attr_L, r2_attr_L, ck_attr_L, px_attr_L, pxx_attr_L, py_attr_L, pyy_attr_L; //attractor left eye camera intermediate results
extern double xp_attr_R, yp_attr_R, r2_attr_R, ck_attr_R, px_attr_R, pxx_attr_R, py_attr_R, pyy_attr_R; //attractor right eye camera intermediate results
extern double d1xp, d1yp, d1r2, d1k, d1px, d1pxx, d1py, d1pyy; //right arm camera intermediate results wrt mu1
extern double d2xp, d2yp, d2r2, d2k, d2px, d2pxx, d2py, d2pyy; //right arm camera intermediate results wrt mu2
extern double d3xp, d3yp, d3r2, d3k, d3px, d3pxx, d3py, d3pyy; //right arm camera intermediate results wrt mu2
extern double d11xp, d11yp, d11r2, d11k, d11px, d11pxx, d11py, d11pyy; //right arm camera intermediate results wrt mu1 mu1
extern double d22xp, d22yp, d22r2, d22k, d22px, d22pxx, d22py, d22pyy; //right arm camera intermediate results wrt mu2 mu2
extern double d33xp, d33yp, d33r2, d33k, d33px, d33pxx, d33py, d33pyy; //right arm camera intermediate results wrt mu3 mu3
extern double d12xp, d12yp, d12r2, d12k, d12px, d12pxx, d12py, d12pyy; //right arm camera intermediate results wrt mu1 mu2
extern double d13xp, d13yp, d13r2, d13k, d13px, d13pxx, d13py, d13pyy; //right arm camera intermediate results wrt mu1 mu3
extern double d23xp, d23yp, d23r2, d23k, d23px, d23pxx, d23py, d23pyy; //right arm camera intermediate results wrt mu2 mu3
extern double xp_h, yp_h, r2_h, ck_h, px_h, pxx_h, py_h, pyy_h; //camera intermediate results
extern double d1xp_h, d1yp_h, d1r2_h, d1k_h, d1px_h, d1pxx_h, d1py_h, d1pyy_h; //camera intermediate results wrt mu1
extern double d2xp_h, d2yp_h, d2r2_h, d2k_h, d2px_h, d2pxx_h, d2py_h, d2pyy_h; //camera intermediate results wrt mu2
extern double d3xp_h, d3yp_h, d3r2_h, d3k_h, d3px_h, d3pxx_h, d3py_h, d3pyy_h; //camera intermediate results wrt mu3
extern double xp_b, yp_b, r2_b, ck_b, px_b, pxx_b, py_b, pyy_b; //left arm camera intermediate results
extern double d1xp_b, d1yp_b, d1r2_b, d1k_b, d1px_b, d1pxx_b, d1py_b, d1pyy_b; //left arm camera intermediate results wrt mu1
extern double d2xp_b, d2yp_b, d2r2_b, d2k_b, d2px_b, d2pxx_b, d2py_b, d2pyy_b; //left arm camera intermediate results wrt mu2
extern double d3xp_b, d3yp_b, d3r2_b, d3k_b, d3px_b, d3pxx_b, d3py_b, d3pyy_b; //left arm camera intermediate results wrt mu3

//----------------------------------------------------------------------------------
//Functions ------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//Encoders -------------------------------------------------------------------------
//Right arm
double read_encoders_q1(vector<double> mu, vector<double> a);
double read_encoders_q2(vector<double> mu, vector<double> a);
double read_encoders_q3(vector<double> mu, vector<double> a);
double read_encoders_q4(vector<double> mu, vector<double> a);
double read_encoders_q1_noise(vector<double> mu, vector<double> a);
double read_encoders_q2_noise(vector<double> mu, vector<double> a);
double read_encoders_q3_noise(vector<double> mu, vector<double> a);
double read_encoders_q4_noise(vector<double> mu, vector<double> a);
double internal_state_mu1(vector<double> mu, vector<double> a);
double internal_state_mu2(vector<double> mu, vector<double> a);
double internal_state_mu3(vector<double> mu, vector<double> a);
double internal_state_mu4(vector<double> mu, vector<double> a);
//Left eye
double read_encoders_q1e(vector<double> mu, vector<double> a);
double read_encoders_q2e(vector<double> mu, vector<double> a);
double read_encoders_q3e(vector<double> mu, vector<double> a);
double internal_state_mu1e(vector<double> mu, vector<double> a);
double internal_state_mu2e(vector<double> mu, vector<double> a);
double internal_state_mu3e(vector<double> mu, vector<double> a);
//Left arm
double read_encoders_q1b(vector<double> mu, vector<double> a);
double read_encoders_q2b(vector<double> mu, vector<double> a);
double read_encoders_q3b(vector<double> mu, vector<double> a);
double read_encoders_q4b(vector<double> mu, vector<double> a);
double read_encoders_q1b_noise(vector<double> mu, vector<double> a);
double read_encoders_q2b_noise(vector<double> mu, vector<double> a);
double read_encoders_q3b_noise(vector<double> mu, vector<double> a);
double read_encoders_q4b_noise(vector<double> mu, vector<double> a);
double internal_state_mu1b(vector<double> mu, vector<double> a);
double internal_state_mu2b(vector<double> mu, vector<double> a);
double internal_state_mu3b(vector<double> mu, vector<double> a);
double internal_state_mu4b(vector<double> mu, vector<double> a);

//3D position ----------------------------------------------------------------------
//Right arm
double sense_3d_position_x(vector<double> mu, vector<double> a);
double sense_3d_position_y(vector<double> mu, vector<double> a);
double sense_3d_position_z(vector<double> mu, vector<double> a);
double sense_3d_attractor_x(vector<double> mu, vector<double> a);
double sense_3d_attractor_y(vector<double> mu, vector<double> a);
double sense_3d_attractor_z(vector<double> mu, vector<double> a);
void calc_3d_rototranslational_matrix(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu1(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu2(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu3(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu4(vector<double> mu, vector<double> a);
void calc_dd_3d_rototranslational_matrix_mu11(vector<double> mu, vector<double> a);
void calc_dd_3d_rototranslational_matrix_mu22(vector<double> mu, vector<double> a);
void calc_dd_3d_rototranslational_matrix_mu33(vector<double> mu, vector<double> a);
void calc_dd_3d_rototranslational_matrix_mu12(vector<double> mu, vector<double> a);
void calc_dd_3d_rototranslational_matrix_mu13(vector<double> mu, vector<double> a);
void calc_dd_3d_rototranslational_matrix_mu23(vector<double> mu, vector<double> a);
double calc_3d_position_x(vector<double> mu, vector<double> a);
double calc_3d_position_y(vector<double> mu, vector<double> a);
double calc_3d_position_z(vector<double> mu, vector<double> a);
double d_3d_position_x_mu1(vector<double> mu, vector<double> a);
double d_3d_position_x_mu2(vector<double> mu, vector<double> a);
double d_3d_position_x_mu3(vector<double> mu, vector<double> a);
double d_3d_position_x_mu4(vector<double> mu, vector<double> a);
double d_3d_position_y_mu1(vector<double> mu, vector<double> a);
double d_3d_position_y_mu2(vector<double> mu, vector<double> a);
double d_3d_position_y_mu3(vector<double> mu, vector<double> a);
double d_3d_position_y_mu4(vector<double> mu, vector<double> a);
double d_3d_position_z_mu1(vector<double> mu, vector<double> a);
double d_3d_position_z_mu2(vector<double> mu, vector<double> a);
double d_3d_position_z_mu3(vector<double> mu, vector<double> a);
double d_3d_position_z_mu4(vector<double> mu, vector<double> a);
//Left arm
double sense_3d_position_x_b(vector<double> mu, vector<double> a);
double sense_3d_position_y_b(vector<double> mu, vector<double> a);
double sense_3d_position_z_b(vector<double> mu, vector<double> a);
void calc_3d_rototranslational_matrix_b(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu1b(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu2b(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu3b(vector<double> mu, vector<double> a);
void calc_d_3d_rototranslational_matrix_mu4b(vector<double> mu, vector<double> a);
double calc_3d_position_x_b(vector<double> mu, vector<double> a);
double calc_3d_position_y_b(vector<double> mu, vector<double> a);
double calc_3d_position_z_b(vector<double> mu, vector<double> a);
double d_3d_position_x_mu1_b(vector<double> mu, vector<double> a);
double d_3d_position_x_mu2_b(vector<double> mu, vector<double> a);
double d_3d_position_x_mu3_b(vector<double> mu, vector<double> a);
double d_3d_position_x_mu4_b(vector<double> mu, vector<double> a);
double d_3d_position_y_mu1_b(vector<double> mu, vector<double> a);
double d_3d_position_y_mu2_b(vector<double> mu, vector<double> a);
double d_3d_position_y_mu3_b(vector<double> mu, vector<double> a);
double d_3d_position_y_mu4_b(vector<double> mu, vector<double> a);
double d_3d_position_z_mu1_b(vector<double> mu, vector<double> a);
double d_3d_position_z_mu2_b(vector<double> mu, vector<double> a);
double d_3d_position_z_mu3_b(vector<double> mu, vector<double> a);
double d_3d_position_z_mu4_b(vector<double> mu, vector<double> a);

//Vision ---------------------------------------------------------------------------
//Left eye
void calc_le_rototranslational_matrix(vector<double> mu, vector<double> a);
void calc_le_rototranslational_matrix_h(vector<double> mu, vector<double> a);
void calc_le_rototranslational_matrix_b(vector<double> mu, vector<double> a);
void calc_le_rototranslational_matrix_inv(vector<double> mu, vector<double> a);
void calc_le_rototranslational_matrix_inv_mu1e(vector<double> mu, vector<double> a);
void calc_le_rototranslational_matrix_inv_mu2e(vector<double> mu, vector<double> a);
double calc_le_endeffpos_x();
double calc_le_endeffpos_y();
double calc_le_endeffpos_z();
void calc_le_camera_projection(vector<double> mu, vector<double> a);
void calc_re_camera_projection(vector<double> mu, vector<double> a);
void calc_le_attractor_projection();
void calc_re_attractor_projection();
void calc_le_camera_origin(vector<double> mu, vector<double> a);
void calc_le_camera_projection_b(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu1(vector<double> mu, vector<double> a); //Right arm
void calc_d_le_camera_projection_mu2(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu3(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu1_b(vector<double> mu, vector<double> a); //Left arm
void calc_d_le_camera_projection_mu2_b(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu3_b(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu11(vector<double> mu, vector<double> a); //Right arm
void calc_d_le_camera_projection_mu22(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu33(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu12(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu13(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu23(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu1(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu2(vector<double> mu, vector<double> a);
void calc_d_le_camera_projection_mu3(vector<double> mu, vector<double> a);
void calc_d_le_camera_origin_mu1(vector<double> mu, vector<double> a);
void calc_d_le_camera_origin_mu2(vector<double> mu, vector<double> a);
void calc_d_le_camera_origin_mu3(vector<double> mu, vector<double> a);
void calc_d_le_camera_origin_mu11(vector<double> mu, vector<double> a);
void calc_d_le_camera_origin_mu22(vector<double> mu, vector<double> a);
void calc_d_le_camera_origin_mu12(vector<double> mu, vector<double> a);
void calc_camera_parameters(Vector4d projection); //Right hand
void calc_camera_parameters_R(Vector4d projection); //Right hand right eye
void calc_camera_parameters_L(Vector4d projection); //Right hand left eye
void calc_camera_parameters_attractor_R(Vector4d projection); //Attractor right eye
void calc_camera_parameters_attractor_L(Vector4d projection); //Attractor hand left eye
void calc_camera_parameters_h(Vector4d projection); //Head
void calc_camera_parameters_b(Vector4d projection); //Left hand
void calc_d_camera_parameters_mu1(Vector4d projection, Vector4d d1_projection); //Right arm
void calc_d_camera_parameters_mu2(Vector4d projection, Vector4d d2_projection);
void calc_d_camera_parameters_mu3(Vector4d projection, Vector4d d3_projection);
void calc_d_camera_parameters_mu1_h(Vector4d projection, Vector4d d1_projection); //Head
void calc_d_camera_parameters_mu2_h(Vector4d projection, Vector4d d2_projection);
void calc_d_camera_parameters_mu3_h(Vector4d projection, Vector4d d3_projection);
void calc_d_camera_parameters_mu1_b(Vector4d projection, Vector4d d1_projection); //Left arm
void calc_d_camera_parameters_mu2_b(Vector4d projection, Vector4d d2_projection);
void calc_d_camera_parameters_mu3_b(Vector4d projection, Vector4d d3_projection);
void calc_d_camera_parameters_mu11(Vector4d projection, Vector4d d1_projection, Vector4d d11_projection);
void calc_d_camera_parameters_mu22(Vector4d projection, Vector4d d2_projection, Vector4d d22_projection);
void calc_d_camera_parameters_mu33(Vector4d projection, Vector4d d3_projection, Vector4d d33_projection);
void calc_d_camera_parameters_mu12(Vector4d projection, Vector4d d1_projection, Vector4d d2_projection, Vector4d d12_projection);
void calc_d_camera_parameters_mu13(Vector4d projection, Vector4d d1_projection, Vector4d d3_projection, Vector4d d13_projection);
void calc_d_camera_parameters_mu23(Vector4d projection, Vector4d d2_projection, Vector4d d3_projection, Vector4d d23_projection);
//Right arm
double sense_vision_l_u(vector<double> mu, vector<double> a);
double sense_vision_l_v(vector<double> mu, vector<double> a);
double calc_vision_l_u(vector<double> mu, vector<double> a);
double calc_vision_l_v(vector<double> mu, vector<double> a);
double sense_vision_r_u(vector<double> mu, vector<double> a);
double sense_vision_r_v(vector<double> mu, vector<double> a);
double calc_vision_r_u(vector<double> mu, vector<double> a);
double calc_vision_r_v(vector<double> mu, vector<double> a);
//Attractor
double calc_attractor_l_u();
double calc_attractor_l_v();
double calc_attractor_r_u();
double calc_attractor_r_v();
//Left arm
double sense_vision_u_b(vector<double> mu, vector<double> a);
double sense_vision_v_b(vector<double> mu, vector<double> a);
double calc_vision_u_b(vector<double> mu, vector<double> a);
double calc_vision_v_b(vector<double> mu, vector<double> a);
//Right arm
double d_vision_u_mu1(vector<double> mu, vector<double> a);
double d_vision_u_mu2(vector<double> mu, vector<double> a);
double d_vision_u_mu3(vector<double> mu, vector<double> a);
double d_vision_v_mu1(vector<double> mu, vector<double> a);
double d_vision_v_mu2(vector<double> mu, vector<double> a);
double d_vision_v_mu3(vector<double> mu, vector<double> a);
//Left eye
double d_vision_u_mu1_head(vector<double> mu, vector<double> a);
double d_vision_u_mu2_head(vector<double> mu, vector<double> a);
double d_vision_u_mu3_head(vector<double> mu, vector<double> a);
double d_vision_v_mu1_head(vector<double> mu, vector<double> a);
double d_vision_v_mu2_head(vector<double> mu, vector<double> a);
double d_vision_v_mu3_head(vector<double> mu, vector<double> a);
//Left arm
double d_vision_u_mu1_b(vector<double> mu, vector<double> a);
double d_vision_u_mu2_b(vector<double> mu, vector<double> a);
double d_vision_u_mu3_b(vector<double> mu, vector<double> a);
double d_vision_v_mu1_b(vector<double> mu, vector<double> a);
double d_vision_v_mu2_b(vector<double> mu, vector<double> a);
double d_vision_v_mu3_b(vector<double> mu, vector<double> a);
//Prior ----------------------------------------------------------------------------
//Right arm
double initial_prior_mu1(vector<double> mu, vector<double> a);
double initial_prior_mu2(vector<double> mu, vector<double> a);
double initial_prior_mu3(vector<double> mu, vector<double> a);
double initial_prior_mu4(vector<double> mu, vector<double> a);
//Left arm
double initial_prior_mu1b(vector<double> mu, vector<double> a);
double initial_prior_mu2b(vector<double> mu, vector<double> a);
double initial_prior_mu3b(vector<double> mu, vector<double> a);
double initial_prior_mu4b(vector<double> mu, vector<double> a);
