/*
 * main.cpp
 *
 * Description: Main program.
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

#include "../FreeEnergyOptimization.h"
#include "../ICubControl.h"
#include "../ICubKinematics.h"
#include "../ICubImage.h"
#include "../YarpPeriodic.h"
#include "../ICubTouch.h"
#include "../ConfigStructs.h"
#include "mdefs.h" //Mathematica equivalents

#include <iostream> //cout
#include <vector> //vectors
#include <cmath> //math functions
#include <chrono> //timing
#include <thread> //threads
#include <mutex> //mutex
#include <csignal> //signal
#include <ctime> //time
#include <eigen3/Eigen/Dense> //eigen dense matrix algebra
#include <random> //random number generator

//First and second derivative, attractor, action terms definition
#include "FirstDerivative.h"
#include "SecondDerivative.h"
#include "Attractor.h"
#include "Action.h"

//namespaces
using namespace std;
using namespace Eigen;

//definitions
#define neck_pitch_min -30 //-40 -> 30
#define neck_pitch_max 25
#define neck_yaw_min -50 //-55 -> 55
#define neck_yaw_max 50
#define eyes_tilt_min -20 //-30 -> 30
#define eyes_tilt_max 5
#define torso_yaw_min -50 //-50 -> 50
#define torso_yaw_max 45
#define shoulder_roll_min 10 //0 -> 160.8
#define shoulder_roll_max 150
#define shoulder_yaw_min -30 //-37 -> 100
#define shoulder_yaw_max 70
#define elbow_min 15 //5.5 -> 106
#define elbow_max 100
#define max_vel_head 10 //hardcoded max velocity for head
#define max_vel_arm 10 //hardcoded max velocity for arm
typedef std::chrono::high_resolution_clock hrClock;

//program execution stage
enum Stage {read_config, start_icub, start_publish, start_camera, start_touch, start_noise, start_feo, exec_feo};

//instanciated classes
FreeEnergyOptimization *feoRightArm; //free energy optimization for right arm
FreeEnergyOptimization *feoHead; //free energy optimization for head
FreeEnergyOptimization *feoLeftArm; //free energy optimization for left arm
ICubControl *iCubRightArmCtrl; //right arm control and encoders
ICubControl *iCubHeadCtrl; //head control and encoders
ICubControl *iCubLeftArmCtrl; //left arm control and encoders
ICubControl *iCubTorsoCtrl; //torso control and encoders
ICubKinematics *iCubRightArmKin; //right arm kinematics
ICubKinematics *iCubLeftEyeKin; //left eye kinematics
ICubKinematics *iCubLeftArmKin; //left arm kinematics
ICubKinematics *iCubTorsoKin; //torso kinematics
ICubImage *iCubImgL; //left eye camera for right arm end-effector position
ICubImage *iCubImgLA; //left eye camera for attractor position
ICubImage *iCubImgLb; //left eye camera for left arm end-effector position
ICubTouch *iCubRightHandTouch; //read right hand touch sensor data
ICubTouch *iCubLeftHandTouch; //read left hand touch sensor data
ICubImage *iCubImgRL; //left and right arm camera for right arm end-effector position
ICubImage *iCubImgRLA; //left and right arm camera for attractor position
ICubImage *iCubImgRLb; //left and right arm camera for right arm end-effector position

//internal state and action
vector<double> state, a, mu0, mu0b, state_head, a_head, state_b, a_b;

//threads
YarpPeriodic *feoRightArmThread;
YarpPeriodic *feoHeadThread;
YarpPeriodic *feoLeftArmThread;
YarpPeriodic *actionRightArmThread;
YarpPeriodic *actionHeadThread;
YarpPeriodic *actionLeftArmThread;
YarpPeriodic *visionThread;
YarpPeriodic *touchThread;

//global variables and vectors
vector<double> initialPrior;
vector<double> initialPriorb;
yarp::sig::Vector qRAencoders, qRAendpos; //RA encoders 1, 2 and 3
yarp::sig::Vector qLEencoders; //LE encoders 0 and 2
yarp::sig::Vector qLAencoders, qLAendpos; //LA encoders 1, 2 and 3
yarp::sig::Vector qTencoders; //T encoders 0/2
yarp::sig::Vector re_pos; //right arm end-effector position relative to the left eye
yarp::sig::Vector re_xpos; //right arm end-effector position in absolute coordinates
yarp::sig::Vector le_pos; //left arm end-effector position relative to the left eye
yarp::sig::Vector le_xpos; //left arm end-effector position in absolute coordinates
yarp::sig::Vector att_pos; //attractor position relative to the left eye (0:x, 1:y, 2:z, 3:l_u, 4:l_v, 5:l_size, 6:r_u, 7:r_v, 8:r_size)
yarp::sig::Vector att_xpos; //attractor position in absolute coordinates
yarp::sig::Vector prev_att_pos; //previous attractor position relative to the left eye
yarp::sig::Vector calc_vision_pos_l, calc_vision_pos_r; //right arm calculated position in visual field
yarp::sig::Vector calc_vision_pos_b; //left arm calculated position in visual field
bool touching = false;
bool touchingLeft = false;
mainConfig main_config; //main configuration structure
armConfig right_arm_config; //right arm configuration structure
armConfig left_arm_config; //left arm configuration structure
headConfig head_config; //head configuration structure
bool t_end = false;
int counter = 0;
mutex a_mutex; //mutex for access to action
double dT; //sampling time (may be in seconds or milliseconds!)
bool publish; //to publish or not to publish, that is the question
string time_str; //date and time of program execution
Stage stage;
int touchCounter = 0; //right hand touch counter filter
int touchCounterLeft = 0; //left hand touch counter filter
string face_cascade_name = "cascades/haarcascade_frontalface_alt.xml"; //cascade for face detection

//random number generators
mt19937 *gen; //Mersenne twister PRNG
normal_distribution<double>* ndis; 

//publish variables for right arm
BufferedPort<Bottle> plot_q1, plot_q2, plot_q3, plot_q4; //encoders
BufferedPort<Bottle> plot_mu1, plot_mu2, plot_mu3, plot_mu4; //internal values
BufferedPort<Bottle> plot_mu1p, plot_mu2p, plot_mu3p, plot_mu4p; //derivative
BufferedPort<Bottle> plot_a1, plot_a2, plot_a3, plot_a4; //action
BufferedPort<Bottle> plot_r_u, plot_r_v, plot_r_size; //right eye 2D end-effector
BufferedPort<Bottle> plot_l_u, plot_l_v, plot_l_size; //left eye 2D end-effector
BufferedPort<Bottle> plot_x, plot_y, plot_z; //end-effector 3D
BufferedPort<Bottle> plot_calc_l_u, plot_calc_l_v; //calculated left eye 2D end-effector
BufferedPort<Bottle> plot_calc_r_u, plot_calc_r_v; //calculated right eye 2D end-effector
BufferedPort<Bottle> plot_calc_x, plot_calc_y, plot_calc_z; //calculated end-effector 3D
BufferedPort<Bottle> plot_attr_l_u, plot_attr_l_v, plot_attr_l_size; //left eye 2D attractor
BufferedPort<Bottle> plot_attr_r_u, plot_attr_r_v, plot_attr_r_size; //right eye 2D attractor
BufferedPort<Bottle> plot_attr_x, plot_attr_y, plot_attr_z; //3D attractor
BufferedPort<Bottle> plot_free_energy;

//publish variables for head
BufferedPort<Bottle> plot_q1e, plot_q2e, plot_q3e;
BufferedPort<Bottle> plot_mu1e, plot_mu2e, plot_mu3e;
BufferedPort<Bottle> plot_mu1ep, plot_mu2ep, plot_mu3ep;
BufferedPort<Bottle> plot_a1e, plot_a2e, plot_a3e;
BufferedPort<Bottle> plot_center_x, plot_center_y;
BufferedPort<Bottle> plot_free_energy_e;

//publish variables for left arm
BufferedPort<Bottle> plot_q1b, plot_q2b, plot_q3b, plot_q4b;
BufferedPort<Bottle> plot_mu1b, plot_mu2b, plot_mu3b, plot_mu4b;
BufferedPort<Bottle> plot_mu1bp, plot_mu2bp, plot_mu3bp, plot_mu4bp;
BufferedPort<Bottle> plot_a1b, plot_a2b, plot_a3b, plot_a4b;
BufferedPort<Bottle> plot_ub, plot_vb;
BufferedPort<Bottle> plot_xb, plot_yb, plot_zb;
BufferedPort<Bottle> plot_calc_ub, plot_calc_vb;
BufferedPort<Bottle> plot_calc_xb, plot_calc_yb, plot_calc_zb;
BufferedPort<Bottle> plot_free_energy_b;

//vision object recognition parameters
int left_hue_low, left_sat_low, left_val_low, left_hue_high, left_sat_high, left_val_high; //right arm left eye
int right_hue_low, right_sat_low, right_val_low, right_hue_high, right_sat_high, right_val_high; //right arm right eye
int left_att_hue_low, left_att_sat_low, left_att_val_low, left_att_hue_high, left_att_sat_high, left_att_val_high; //attractor left eye
int right_att_hue_low, right_att_sat_low, right_att_val_low, right_att_hue_high, right_att_sat_high, right_att_val_high; //attractor right eye
int left_hue_low_b, left_sat_low_b, left_val_low_b, left_hue_high_b, left_sat_high_b, left_val_high_b; //left arm left eye
int right_hue_low_b, right_sat_low_b, right_val_low_b, right_hue_high_b, right_sat_high_b, right_val_high_b; //left arm right eye

//right arm robot algebra
Matrix4d ra_rtm; //rototranslational matrix for right arm
Vector4d ra_repv; //relative end effector position vector for right arm
Vector4d ra_epv; //end effector position vector for right arm
Matrix4d ra_d1rtm; //derivative of rototranslational matrix wrt mu1
Matrix4d ra_d2rtm; //derivative of rototranslational matrix wrt mu2
Matrix4d ra_d3rtm; //derivative of rototranslational matrix wrt mu3
Matrix4d ra_d4rtm; //derivative of rototranslational matrix wrt mu4
Vector4d ra_d1epv; //derivative of end effector position vector wrt mu1
Vector4d ra_d2epv; //derivative of end effector position vector wrt mu2
Vector4d ra_d3epv; //derivative of end effector position vector wrt mu3
Vector4d ra_d4epv; //derivative of end effector position vector wrt mu4
Matrix4d ra_d11rtm; //derivative of rototranslational matrix wrt mu1 mu1
Matrix4d ra_d22rtm; //derivative of rototranslational matrix wrt mu2 mu2
Matrix4d ra_d33rtm; //derivative of rototranslational matrix wrt mu3 mu3
Matrix4d ra_d12rtm; //derivative of rototranslational matrix wrt mu1 mu2
Matrix4d ra_d13rtm; //derivative of rototranslational matrix wrt mu1 mu3
Matrix4d ra_d23rtm; //derivative of rototranslational matrix wrt mu2 mu3
Vector4d ra_d11epv; //derivative of end effector position vector wrt mu1 mu1
Vector4d ra_d22epv; //derivative of end effector position vector wrt mu2 mu2
Vector4d ra_d33epv; //derivative of end effector position vector wrt mu3 mu3
Vector4d ra_d12epv; //derivative of end effector position vector wrt mu1 mu2
Vector4d ra_d13epv; //derivative of end effector position vector wrt mu1 mu3
Vector4d ra_d23epv; //derivative of end effector position vector wrt mu2 mu3

//left eye robot algebra
Matrix4d le_rtm; //rototranslational matrix for left eye
Matrix4d le_rtm_h; //rototranslational matrix for left eye
Matrix4d le_rtm_b; //rototranslational matrix for left eye
Vector4d le_repv; //relative end effector position vector for left eye
Vector4d le_epv; //end effector position vector for left eye
Matrix4d le_rtm_inv; //inverted rototranslational matrix for left eye
Matrix4d le_rtm_inv_b; //inverted rototranslational matrix for left eye
Matrix4d inv_le_rtm; //inverse rototranslational matrix for left eye
Matrix4d d1_inv_le_rtm; //derivative of inverse rototranslational matrix for left eye wrt mu1e
Matrix4d d2_inv_le_rtm; //derivative of inverse rototranslational matrix for left eye wrt mu2e
Matrix4d d3_inv_le_rtm; //derivative of inverse rototranslational matrix for left eye wrt mu3e

//right eye robot algebra
Matrix4d re_rtm; //rototranslational matrix for right eye
Matrix4d re_rtm_inv; //inverted rototranslational matrix for right eye

//left arm robot algebra
Matrix4d la_rtm; //rototranslational matrix for left arm
Vector4d la_repv; //relative end effector position vector for left arm
Vector4d la_epv; //end effector position vector for left arm
Matrix4d la_d1rtm; //derivative of rototranslational matrix wrt mu1
Matrix4d la_d2rtm; //derivative of rototranslational matrix wrt mu2
Matrix4d la_d3rtm; //derivative of rototranslational matrix wrt mu3
Matrix4d la_d4rtm; //derivative of rototranslational matrix wrt mu3
Vector4d la_d1epv; //derivative of end effector position vector wrt mu1
Vector4d la_d2epv; //derivative of end effector position vector wrt mu2
Vector4d la_d3epv; //derivative of end effector position vector wrt mu3
Vector4d la_d4epv; //derivative of end effector position vector wrt mu4

//camera robot algebra
Vector4d cam_projection; //projection vector for the end effector of the right arm
Vector4d cam_projection_L; //left eye projection vector for the end effector of the right arm
Vector4d cam_projection_R; //right eye projection vector for the end effector of the right arm
Vector4d attr_projection_L; //left eye projection vector for the virtual attractor
Vector4d attr_projection_R; //right eye projection vector for the virtual attractor
Vector4d d1_cam_projection; //right arm derivative wrt mu1
Vector4d d2_cam_projection; //right arm derivative wrt mu2
Vector4d d3_cam_projection; //right arm derivative wrt mu3
Vector4d d11_cam_projection; //right arm derivative wrt mu1 mu1
Vector4d d22_cam_projection; //right arm derivative wrt mu2 mu2
Vector4d d33_cam_projection; //right arm derivative wrt mu2 mu2
Vector4d d12_cam_projection; //right arm derivative wrt mu1 mu2
Vector4d d13_cam_projection; //right arm derivative wrt mu1 mu3
Vector4d d23_cam_projection; //right arm derivative wrt mu2 mu3
Vector4d le_projection; //camera centerpoint projection
Vector4d r_le_projection; //relative world camera centerpoint projection
Vector4d d1_le_projection; //head derivative wrt mu1e
Vector4d d2_le_projection; //head derivative wrt mu2e
Vector4d d3_le_projection; //head derivative wrt mu3e
Vector4d d11_le_projection; //head derivative wrt mu1 mu1
Vector4d d22_le_projection; //head derivative wrt mu2 mu2
Vector4d d12_le_projection; //head derivative wrt mu1 mu2
Vector4d cam_projection_b; //projection vector for the end effector of the left arm
Vector4d d1_cam_projection_b; //left arm derivative wrt mu1
Vector4d d2_cam_projection_b; //left arm derivative wrt mu2
Vector4d d3_cam_projection_b; //left arm derivative wrt mu3
double xp, yp, r2, ck, px, pxx, py, pyy; //right arm camera intermediate results
double xp_L, yp_L, r2_L, ck_L, px_L, pxx_L, py_L, pyy_L; //right arm left eye camera intermediate results
double xp_R, yp_R, r2_R, ck_R, px_R, pxx_R, py_R, pyy_R; //right arm right eye camera intermediate results
double xp_attr_L, yp_attr_L, r2_attr_L, ck_attr_L, px_attr_L, pxx_attr_L, py_attr_L, pyy_attr_L; //attractor left eye camera intermediate results
double xp_attr_R, yp_attr_R, r2_attr_R, ck_attr_R, px_attr_R, pxx_attr_R, py_attr_R, pyy_attr_R; //attractor right eye camera intermediate results
double d1xp, d1yp, d1r2, d1k, d1px, d1pxx, d1py, d1pyy; //right arm camera intermediate results wrt mu1
double d2xp, d2yp, d2r2, d2k, d2px, d2pxx, d2py, d2pyy; //right arm camera intermediate results wrt mu2
double d3xp, d3yp, d3r2, d3k, d3px, d3pxx, d3py, d3pyy; //right arm camera intermediate results wrt mu3
double d11xp, d11yp, d11r2, d11k, d11px, d11pxx, d11py, d11pyy; //right arm camera intermediate results wrt mu1 mu1
double d22xp, d22yp, d22r2, d22k, d22px, d22pxx, d22py, d22pyy; //right arm camera intermediate results wrt mu2 mu2
double d33xp, d33yp, d33r2, d33k, d33px, d33pxx, d33py, d33pyy; //right arm camera intermediate results wrt mu3 mu3
double d12xp, d12yp, d12r2, d12k, d12px, d12pxx, d12py, d12pyy; //right arm camera intermediate results wrt mu1 mu2
double d13xp, d13yp, d13r2, d13k, d13px, d13pxx, d13py, d13pyy; //right arm camera intermediate results wrt mu1 mu3
double d23xp, d23yp, d23r2, d23k, d23px, d23pxx, d23py, d23pyy; //right arm camera intermediate results wrt mu2 mu3
double xp_h, yp_h, r2_h, ck_h, px_h, pxx_h, py_h, pyy_h; //camera intermediate results
double d1xp_h, d1yp_h, d1r2_h, d1k_h, d1px_h, d1pxx_h, d1py_h, d1pyy_h; //camera intermediate results wrt mu1
double d2xp_h, d2yp_h, d2r2_h, d2k_h, d2px_h, d2pxx_h, d2py_h, d2pyy_h; //camera intermediate results wrt mu2
double d3xp_h, d3yp_h, d3r2_h, d3k_h, d3px_h, d3pxx_h, d3py_h, d3pyy_h; //camera intermediate results wrt mu3
double xp_b, yp_b, r2_b, ck_b, px_b, pxx_b, py_b, pyy_b; //left arm camera intermediate results
double d1xp_b, d1yp_b, d1r2_b, d1k_b, d1px_b, d1pxx_b, d1py_b, d1pyy_b; //left arm camera intermediate results wrt mu1
double d2xp_b, d2yp_b, d2r2_b, d2k_b, d2px_b, d2pxx_b, d2py_b, d2pyy_b; //left arm camera intermediate results wrt mu2
double d3xp_b, d3yp_b, d3r2_b, d3k_b, d3px_b, d3pxx_b, d3py_b, d3pyy_b; //left arm camera intermediate results wrt mu3

//attractor algebra
Vector3d attr_3d_pos; //world 3d attractor position
Vector4d ra_3d_jacobian_mu1; //mu1 part of end effector velocity jacobian
Vector4d ra_3d_jacobian_mu2; //mu2 part of end effector velocity jacobian
Vector4d ra_3d_jacobian_mu3; //mu3 part of end effector velocity jacobian
Vector4d ra_3d_jacobian_mu4; //mu4 part of end effector velocity jacobian
Vector4d ra_3d_jacobian_mu1_origin; //mu1 part of end effector origin velocity jacobian
Vector4d ra_3d_jacobian_mu2_origin; //mu2 part of end effector origin velocity jacobian
Vector4d ra_3d_jacobian_mu3_origin; //mu3 part of end effector origin velocity jacobian
Vector4d ra_3d_jacobian_mu4_origin; //mu4 part of end effector origin velocity jacobian
Vector4d la_3d_jacobian_mu1; //mu1 part of end effector velocity jacobian
Vector4d la_3d_jacobian_mu2; //mu2 part of end effector velocity jacobian
Vector4d la_3d_jacobian_mu3; //mu3 part of end effector velocity jacobian
Vector4d la_3d_jacobian_mu4; //mu4 part of end effector velocity jacobian
Vector4d la_3d_jacobian_mu1_origin; //mu1 part of end effector origin velocity jacobian
Vector4d la_3d_jacobian_mu2_origin; //mu2 part of end effector origin velocity jacobian
Vector4d la_3d_jacobian_mu3_origin; //mu3 part of end effector origin velocity jacobian
Vector4d la_3d_jacobian_mu4_origin; //mu4 part of end effector origin velocity jacobian
Vector4d ra_3d_hessian_mu11; //mu11 part of end effector velocity hessian
Vector4d ra_3d_hessian_mu12; //mu12 part of end effector velocity hessian
Vector4d ra_3d_hessian_mu22; //mu22 part of end effector velocity hessian
Vector4d ra_3d_hessian_mu23; //mu23 part of end effector velocity hessian
Vector4d ra_3d_hessian_mu33; //mu33 part of end effector velocity hessian
Vector4d ra_3d_hessian_mu13; //mu13 part of end effector velocity hessian
Vector4d ra_3d_hessian_mu11_origin; //mu11 part of end effector origin velocity hessian
Vector4d ra_3d_hessian_mu12_origin; //mu12 part of end effector origin velocity hessian
Vector4d ra_3d_hessian_mu22_origin; //mu22 part of end effector origin velocity hessian
Vector4d ra_3d_hessian_mu23_origin; //mu23 part of end effector origin velocity hessian
Vector4d ra_3d_hessian_mu33_origin; //mu33 part of end effector origin velocity hessian
Vector4d ra_3d_hessian_mu13_origin; //mu13 part of end effector origin velocity hessian
Eigen::Matrix<double, 3, 4> ra_3d_jacobian;  //end effector velocity jacobian
Eigen::Matrix<double, 3, 4> ra_3d_jacobian_origin;  //end effector origin velocity jacobian
Eigen::Matrix<double, 3, 4> la_3d_jacobian;  //end effector velocity jacobian
Eigen::Matrix<double, 3, 4> la_3d_jacobian_origin;  //end effector origin velocity jacobian
Eigen::Matrix<double, 3, 4> ra_3d_hessian_mu1;  //velocity hessian for wrt mu1
Eigen::Matrix<double, 3, 4> ra_3d_hessian_mu2;  //velocity hessian for wrt mu2
Eigen::Matrix<double, 3, 4> ra_3d_hessian_mu3;  //velocity hessian for wrt mu3
Eigen::Matrix<double, 3, 4> ra_3d_hessian_mu1_origin;  //velocity origin hessian for wrt mu1
Eigen::Matrix<double, 3, 4> ra_3d_hessian_mu2_origin;  //velocity origin hessian for wrt mu2
Eigen::Matrix<double, 3, 4> ra_3d_hessian_mu3_origin;  //velocity origin hessian for wrt mu3
Vector3d ra_3d_attr; //right arm 3d attractor dynamic vector
Vector4d ra_3d_f_attr; //right arm 3d attractor joint vector (4 DOF)
Vector3d la_3d_attr; //left arm 3d attractor dynamic vector
Vector4d la_3d_f_attr; //left arm 3d attractor joint vector (4 DOF)
Vector3d d1_3d_f_attractor; //partial derivative vector for the 3d attractor wrt mu1
Vector3d d2_3d_f_attractor; //partial derivative vector for the 3d attractor wrt mu2
Vector3d d3_3d_f_attractor; //partial derivative vector for the 3d attractor wrt mu3
Vector4d ra_visual_hessian_mu11; //mu11 part of end effector visual hessian
Vector4d ra_visual_hessian_mu12; //mu12 part of end effector visual hessian
Vector4d ra_visual_hessian_mu22; //mu22 part of end effector visual hessian
Vector4d ra_visual_hessian_mu23; //mu23 part of end effector visual hessian
Vector4d ra_visual_hessian_mu33; //mu33 part of end effector visual hessian
Vector4d ra_visual_hessian_mu13; //mu13 part of end effector visual hessian
Eigen::Matrix<double, 2, 3> ra_visual_jacobian;  //right arm end effector visual jacobian
Eigen::Matrix<double, 2, 3> ra_visual_jacobian_head;  //camera visual jacobian
Eigen::Matrix<double, 2, 3> la_visual_jacobian;  //left arm end effector visual jacobian
Eigen::Matrix<double, 2, 3> ra_visual_hessian_mu1;  //velocity hessian for wrt mu1
Eigen::Matrix<double, 2, 3> ra_visual_hessian_mu2;  //velocity hessian for wrt mu2
Eigen::Matrix<double, 2, 3> ra_visual_hessian_mu3;  //velocity hessian for wrt mu2
Vector2d ra_visual_attr; //right arm visual attractor position vector
Vector3d ra_visual_f_attr; //right arm visual attractor joint vector
Vector3d d1_visual_f_attractor; //partial derivative vector for the visual attractor wrt mu1
Vector3d d2_visual_f_attractor; //partial derivative vector for the visual attractor wrt mu2
Vector3d d3_visual_f_attractor; //partial derivative vector for the visual attractor wrt mu2
Vector2d head_visual_attr; //head visual attractor position vector
Vector3d head_visual_f_attr; //head visual attractor joint vector
Vector2d la_visual_attr; //left arm visual attractor position vector
Vector3d la_visual_f_attr; //left arm visual attractor joint vector

//----------------------------------------------------------------------------------
//Robot variables ------------------------------------------------------------------
//----------------------------------------------------------------------------------

//Additional factors
double k, kk;

//Right arm DH parameters
double a1, d2, a3, d3, d4, a6, d6, a7, d8, a10, d10;

//Right arm rotational DOF
double ra_q1, ra_q2, ra_q3, ra_q4, ra_q5, ra_q6, ra_q7, ra_q8, ra_q9, ra_q10;

//Right arm angle bias
double ra_q1b, ra_q2b, ra_q3b, ra_q4b, ra_q5b, ra_q6b, ra_q7b, ra_q8b, ra_q9b, ra_q10b;

//Right arm end effector position
double xh, yh, zh;

//Left arm end effector position
double xhb, yhb, zhb;

//Left eye DH parameters
double a1e, d2e, a3e, d3e, a4e, d5e, a6e, d6e, d7e;
double d7ex; //right eye

//Left eye rotational DOF
double le_q1, le_q2, le_q3, le_q4, le_q5, le_q6, le_q7, le_q8;

//Left eye angle bias
double le_q1b, le_q2b, le_q3b, le_q4b, le_q5b, le_q6b, le_q7b, le_q8b;

//Left arm DH parameters
double a1b, d2b, a3b, d3b, d4b, a6b, d6b, a7b, d8b, a10b, d10b;

//Left arm rotational DOF
double la_q1, la_q2, la_q3, la_q4, la_q5, la_q6, la_q7, la_q8, la_q9, la_q10;

//Left arm angle bias
double la_q1b, la_q2b, la_q3b, la_q4b, la_q5b, la_q6b, la_q7b, la_q8b, la_q9b, la_q10b;

//Camera parameters
double w, h;
//Right arm
double fx_R, fy_R, cx_R, cy_R, k1_R, k2_R, k3_R, k4_R, k5_R, k6_R, p1_R, p2_R;
//Left arm
double fx_L, fy_L, cx_L, cy_L, k1_L, k2_L, k3_L, k4_L, k5_L, k6_L, p1_L, p2_L;
//Camera matrices
Mat matrixCam1, distCam1, matrixCam2, distCam2, matrixRot, matrixTrans;

//Camera parameters (2d)
double fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2;

//Attractor parameters
double ro1, ro2, ro3, ro4, ro5, ro6, ro7, ro8, ro9, ro10;
double ro11, ro12, ro13, ro14, ro15, ro16, ro17, ro18, ro19, ro20;
double ro21, ro22, ro23, ro24, ro25;
int att_state = 0;
double ro1_head;

//----------------------------------------------------------------------------------
//Functions ------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//Initialization functions
void robot_parameters();
void get_date_time();
int init_encoders();
void init_kinematics();
void init_publish();
void init_camera();
void init_touch();
void init_noise();

//Config and logging
void config_file_parser(string filename);
void print_config();
void log_config(fstream& logFile, string description);
void log_stats(fstream& logFile, string description);

//Free energy optimization setup
void feo_right_arm_setup();
void feo_right_arm_run();
bool zero_condition_right_arm(double action, int dof);
void feo_head_setup();
void feo_head_run();
bool zero_condition_head(double action, int dof);
void feo_left_arm_setup();
void feo_left_arm_run();
bool zero_condition_left_arm(double action, int dof);

//Threads
void sense_vision_thread();
void set_action_right_arm_thread();
void set_action_head_thread();
void set_action_left_arm_thread();
void set_touch_thread();
void calculate_feo_right_arm();
void calculate_feo_head();
void calculate_feo_left_arm();

//Motion
void rightGrasping(bool all);
void leftGrasping(bool all);

//Other
void set_interrupt_handle();
void ctrl_c_handler(int s);
void prepare_variables();
void end_right_arm();
void end_head();
void end_left_arm();
void clean_up();

//----------------------------------------------------------------------------------
//Initialization functions ---------------------------------------------------------
//----------------------------------------------------------------------------------

//Initialize values
void robot_parameters()
{
	//Factor
	k = 1000; //(mm to m)
	kk = k*k;
	
	//Right arm DH parameters (mm)
	a1 = 32;
	d2 = -55/10.0;
	a3 = -233647/10000.0;
	d3 = -1433/10.0;
	d4 = -10774/100.0;
	a6 = -15;
	d6 = -15228/100.0;
	a7 = 15;
	d8 = -1373/10.0;
	a10 = 625/10.0;
	d10 = 16;

	//Right arm angle bias (degrees -> radians)
	ra_q1b = CTRL_DEG2RAD*0; //torso_pitch
	ra_q2b = CTRL_DEG2RAD*-90; //torso_roll
	ra_q3b = CTRL_DEG2RAD*-105; //torso_yaw
	ra_q4b = CTRL_DEG2RAD*-90; //r_shoulder_pitch
	ra_q5b = CTRL_DEG2RAD*-90; //r_shoulder_roll
	ra_q6b = CTRL_DEG2RAD*-105; //r_shoulder_yaw
	ra_q7b = CTRL_DEG2RAD*0; //r_elbow
	ra_q8b = CTRL_DEG2RAD*-90; //r_wrist_prosup
	ra_q9b = CTRL_DEG2RAD*90; //r_wrist_pitch
	ra_q10b = CTRL_DEG2RAD*180; //r_wrist_yaw

	//Left eye DH parameters (mm)
	a1e = 32;
    d2e = -55/10.0;
	a3e = 231/100.0;
	d3e = -1933/10.0;
    a4e = 33;
	d5e = 1;
	a6e = -54;
	d6e = 825/10.0;
	d7e = -34;
    d7ex = -d7e;

	//Left eye angle bias (degrees -> radians)
	le_q1b = CTRL_DEG2RAD*0; //torso_pitch
	le_q2b = CTRL_DEG2RAD*-90; //torso_roll
	le_q3b = CTRL_DEG2RAD*-90; //torso_yaw
	le_q4b = CTRL_DEG2RAD*90; //neck_pitch
	le_q5b = CTRL_DEG2RAD*-90; //neck_roll
	le_q6b = CTRL_DEG2RAD*90; //neck_yaw
	le_q7b = CTRL_DEG2RAD*0; //eyes_tilt
	le_q8b = CTRL_DEG2RAD*-90; //eyes_version

	//Left arm DH parameters (mm)
	a1b = 32;
	d2b = -55/10.0;
	a3b = 233647/10000.0;
	d3b = -1433/10.0;
	d4b = 10774/100.0;
	a6b = 15;
	d6b = 15228/100.0;
	a7b = -15;
	d8b = 1373/10.0;
	a10b = 625/10.0;
	d10b = -16;

	//Left arm angle bias (degrees -> radians)
	la_q1b = CTRL_DEG2RAD*0; //torso_pitch
	la_q2b = CTRL_DEG2RAD*-90; //torso_roll
	la_q3b = CTRL_DEG2RAD*105; //torso_yaw
	la_q4b = CTRL_DEG2RAD*90; //l_shoulder_pitch
	la_q5b = CTRL_DEG2RAD*-90; //l_shoulder_roll
	la_q6b = CTRL_DEG2RAD*75; //l_shoulder_yaw
	la_q7b = CTRL_DEG2RAD*0; //l_elbow
	la_q8b = CTRL_DEG2RAD*-90; //l_wrist_prosup
	la_q9b = CTRL_DEG2RAD*90; //l_wrist_pitch
	la_q10b = CTRL_DEG2RAD*0; //l_wrist_yaw

    //Camera parameters (2D)
    fx = 223.157; //219.057; //25734/100.0;
    fy = 222.183; //219.028; //25734/100.0;
    cx = 167.434; //174.742; //160;
    cy = 105.917; //102.874; //120;
    k1 = -0.375105; //-0.374173; //0;
    k2 = 0.153483; //0.205428; //0;
    k3 = 0;
    k4 = 0;
    k5 = 0;
    k6 = 0;
    p1 = 0.000383406;//0.00282356; //0;
    p2 = 0.000257203;//0.00270998; //0;

	//Right arm end effector position (mm)
	xh = 5.21;
	yh = -25.84;
	zh = -33.56;
	ra_repv(0) = xh;
	ra_repv(1) = yh;
	ra_repv(2) = zh;
	ra_repv(3) = 1;
	
	//Left eye end effector position (mm)
	le_repv(0) = 0;
	le_repv(1) = 0;
	le_repv(2) = 0;
	le_repv(3) = 1;
	
	//Left arm end effector position (mm)
	xhb = 5.21;
	yhb = -25.84;
	zhb = 33.56;
	la_repv(0) = xhb;
	la_repv(1) = yhb;
	la_repv(2) = zhb;
	la_repv(3) = 1;

	//Left eye camera origin vector
	le_projection(0) = 0;
	le_projection(1) = 0;
	le_projection(2) = 1; //arbitrary distance
	le_projection(3) = 1;
		
	//Attractor parameters (m/pixels)
	//mu1 = 20, mu2 = 50
	//3d: (-0.300304, 0.179838, 0.0443105)
	ro1 = right_arm_config.ro1_attractor;
	ro2 = right_arm_config.ro2_attractor; 
	ro3 = right_arm_config.ro3_attractor; 
	ro4 = right_arm_config.ro4_attractor;
    ro5 = right_arm_config.ro5_attractor;
    ro6 = right_arm_config.ro6_attractor;
    ro7 = right_arm_config.ro7_attractor;
    ro8 = right_arm_config.ro8_attractor;
    ro9 = right_arm_config.ro9_attractor;
    ro10 = right_arm_config.ro10_attractor;
    ro11 = right_arm_config.ro11_attractor;
    ro12 = right_arm_config.ro12_attractor;
    ro13 = right_arm_config.ro13_attractor;
    ro14 = right_arm_config.ro14_attractor;
    ro15 = right_arm_config.ro15_attractor;
    ro16 = right_arm_config.ro16_attractor;
    ro17 = right_arm_config.ro17_attractor;
    ro18 = right_arm_config.ro18_attractor;
    ro19 = right_arm_config.ro19_attractor;
    ro20 = right_arm_config.ro20_attractor;
    ro21 = right_arm_config.ro21_attractor;
    ro22 = right_arm_config.ro22_attractor;
    ro23 = right_arm_config.ro23_attractor;
    ro24 = right_arm_config.ro24_attractor;
    ro25 = right_arm_config.ro25_attractor;
	ro1_head = head_config.ro1_attractor;
	
}

void get_date_time()
{
	time_t T = time(NULL);
	struct tm tm = *localtime(&T);
    time_str.resize(15);
	sprintf( &time_str[0], "%04d%02d%02d_%02d%02d%02d", tm.tm_year+1900, tm.tm_mon+1,
		tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}

int init_encoders()
{
	//Instanciate iCubControl classes
	iCubRightArmCtrl = new ICubControl(main_config.robot);	
	iCubHeadCtrl = new ICubControl(main_config.robot);
	iCubLeftArmCtrl = new ICubControl(main_config.robot);
    iCubTorsoCtrl = new ICubControl(main_config.robot);
	
	//Setup right arm and head control
    bool ra_ok, h_ok, la_ok, t_ok;
	ra_ok = iCubRightArmCtrl->setupSubsystem(right_arm, main_config.debug);
	h_ok = iCubHeadCtrl->setupSubsystem(head, main_config.debug);
	la_ok = iCubLeftArmCtrl->setupSubsystem(left_arm, main_config.debug);
    t_ok = iCubTorsoCtrl->setupSubsystem(torso, main_config.debug);

    if ( !ra_ok || !h_ok || !la_ok || !t_ok)
	{
        cout << "Error adquiring right arm, left arm, head and torso control." << endl;
		delete iCubRightArmCtrl; //control and encoders
		delete iCubHeadCtrl;
		delete iCubLeftArmCtrl;
        delete iCubTorsoCtrl;
		return 0;
	}
	
	//other
    calc_vision_pos_l.resize(2); calc_vision_pos_r.resize(2);
    calc_vision_pos_b.resize(2);

	if (right_arm_config.activated)
	{
		//Move right arm and hand to desired initial position
		yarp::sig::Vector q_init_ra = iCubRightArmCtrl->readEncoders();
		q_init_ra[0] = -25; //r_shoulder_pitch
		q_init_ra[1] = right_arm_config.shoulder_roll; //30; //r_shoulder_roll
		q_init_ra[2] = right_arm_config.shoulder_yaw; //0; //r_shoulder_yaw
		q_init_ra[3] = right_arm_config.elbow; //60; //r_elbow
		q_init_ra[4] = 45; //r_wrist_prosup
		q_init_ra[5] = 0; //r_wrist_pitch
		q_init_ra[6] = 0; //r_wrist_yaw
		q_init_ra[7] = 0; //r_hand_finger
		q_init_ra[8] = 0; //r_thumb_oppose
		q_init_ra[9] = 0; //r_thumb_proximal
		q_init_ra[10] = 0; //r_thumb_distal
		q_init_ra[11] = 0; //r_index_proximal
		q_init_ra[12] = 0; //r_index_distal
		q_init_ra[13] = 0; //r_middle_proximal
		q_init_ra[14] = 0; //r_middle_distal
		q_init_ra[15] = 0; //r_pinky
		iCubRightArmCtrl->changeControlMode(position);
		iCubRightArmCtrl->setPosition(q_init_ra, true);

        //Move torso to desired initial position
        yarp::sig::Vector q_init_t = iCubTorsoCtrl->readEncoders();
        q_init_t[0] = right_arm_config.torso_yaw; //0 //torso_yaw
        q_init_t[1] = 0; //torso_roll
        q_init_t[2] = 0; //torso_pitch
        iCubTorsoCtrl->changeControlMode(position);
        iCubTorsoCtrl->setPosition(q_init_t, true);

	}
	
	//Move head to desired initial position
	yarp::sig::Vector q_init_h = iCubHeadCtrl->readEncoders();
	q_init_h[0] = head_config.neck_pitch; //-30; //neck_pitch
	q_init_h[1] = 0; //neck_roll
	q_init_h[2] = head_config.neck_yaw; //-25; //neck_yaw
    q_init_h[3] = head_config.eyes_tilt; //eyes_tilt
	q_init_h[4] = 0; //eyes_version
	q_init_h[5] = 0; //eyes_vergence
	iCubHeadCtrl->changeControlMode(position);
	iCubHeadCtrl->setPosition(q_init_h, true);
	
	if (left_arm_config.activated)
	{
		//Move left arm and hand to desired initial position
		yarp::sig::Vector q_init_la = iCubLeftArmCtrl->readEncoders();
		q_init_la[0] = -25; //l_shoulder_pitch
		q_init_la[1] = left_arm_config.shoulder_roll; //30; //l_shoulder_roll
		q_init_la[2] = left_arm_config.shoulder_yaw; //0; //l_shoulder_yaw
		q_init_la[3] = left_arm_config.elbow; //60; //l_elbow
		q_init_la[4] = 45; //l_wrist_prosup
		q_init_la[5] = 0; //l_wrist_pitch
		q_init_la[6] = 0; //l_wrist_yaw
		q_init_la[7] = 0; //l_hand_finger
		q_init_la[8] = 0; //l_thumb_oppose
		q_init_la[9] = 0; //l_thumb_proximal
		q_init_la[10] = 0; //l_thumb_distal
		q_init_la[11] = 0; //l_index_proximal
		q_init_la[12] = 0; //l_index_distal
		q_init_la[13] = 0; //l_middle_proximal
		q_init_la[14] = 0; //l_middle_distal
		q_init_la[15] = 0; //l_pinky
		iCubLeftArmCtrl->changeControlMode(position);
		iCubLeftArmCtrl->setPosition(q_init_la, true);
	
	    //Move torso to desired initial position (if not set in right arm)
        if (!right_arm_config.activated)
		{
			yarp::sig::Vector q_init_t = iCubTorsoCtrl->readEncoders();
			q_init_t[0] = left_arm_config.torso_yaw; //0 //torso_yaw
			q_init_t[1] = 0; //torso_roll
			q_init_t[2] = 0; //torso_pitch
			iCubTorsoCtrl->changeControlMode(position);
			iCubTorsoCtrl->setPosition(q_init_t, true);
		}
	
	}	

	//Read encoders to feed them to kinematics library
	yarp::sig::Vector ra_encoders = iCubRightArmCtrl->readEncoders();
	yarp::sig::Vector le_encoders = iCubHeadCtrl->readEncoders();
	yarp::sig::Vector la_encoders = iCubLeftArmCtrl->readEncoders();
    yarp::sig::Vector t_encoders = iCubTorsoCtrl->readEncoders();
	
	if (right_arm_config.activated)
	{

		//Assign value of encoders to DOF values of right arm
        ra_q1 = CTRL_DEG2RAD*t_encoders[2]; //torso_pitch
        ra_q2 = CTRL_DEG2RAD*t_encoders[1]; //torso_roll
        ra_q3 = CTRL_DEG2RAD*t_encoders[0]; //torso_yaw
		ra_q4 = CTRL_DEG2RAD*ra_encoders[0]; //r_shoulder_pitch
		ra_q5 = CTRL_DEG2RAD*ra_encoders[1]; //r_shoulder_roll
		ra_q6 = CTRL_DEG2RAD*ra_encoders[2]; //r_shoulder_yaw
		ra_q7 = CTRL_DEG2RAD*ra_encoders[3]; //r_elbow
		ra_q8 = CTRL_DEG2RAD*ra_encoders[4]; //r_wrist_prosup
		ra_q9 = CTRL_DEG2RAD*ra_encoders[5]; //r_wrist_pitch
		ra_q10 = CTRL_DEG2RAD*ra_encoders[6]; //r_wrist_yaw

	}

	//Assign value of encoders to DOF values of left eye
    le_q1 = CTRL_DEG2RAD*t_encoders[2]; //torso_pitch
    le_q2 = CTRL_DEG2RAD*t_encoders[1]; //torso_roll
    le_q3 = CTRL_DEG2RAD*t_encoders[0]; //torso_yaw
	le_q4 = CTRL_DEG2RAD*le_encoders[0]; //neck_pitch
	le_q5 = CTRL_DEG2RAD*le_encoders[1]; //neck_roll
	le_q6 = CTRL_DEG2RAD*le_encoders[2]; //neck_yaw
	le_q7 = CTRL_DEG2RAD*le_encoders[3]; //eyes_tilt
	le_q8 = CTRL_DEG2RAD*le_encoders[4]; //eyes_version
	
	if (left_arm_config.activated)
	{

		//Assign value of encoders to DOF values of left arm
        la_q1 = CTRL_DEG2RAD*t_encoders[2]; //torso_pitch
        la_q2 = CTRL_DEG2RAD*t_encoders[1]; //torso_roll
        la_q3 = CTRL_DEG2RAD*t_encoders[0]; //torso_yaw
		la_q4 = CTRL_DEG2RAD*la_encoders[0]; //l_shoulder_pitch
		la_q5 = CTRL_DEG2RAD*la_encoders[1]; //l_shoulder_roll
		la_q6 = CTRL_DEG2RAD*la_encoders[2]; //l_shoulder_yaw
		la_q7 = CTRL_DEG2RAD*la_encoders[3]; //l_elbow
		la_q8 = CTRL_DEG2RAD*la_encoders[4]; //l_wrist_prosup
		la_q9 = CTRL_DEG2RAD*la_encoders[5]; //l_wrist_pitch
		la_q10 = CTRL_DEG2RAD*la_encoders[6]; //l_wrist_yaw

	}
	
	//Read from auxiliary function
	if (right_arm_config.activated) cout << "Right arm joint locations from encoders:" << endl << ra_encoders.toString() << endl;
	cout << "Left eye joint locations from encoders:" << endl << le_encoders.toString() << endl;
	if (left_arm_config.activated) cout << "Left arm joint locations from encoders:" << endl << la_encoders.toString() << endl;
    cout << "Torso joint locations from encoders:" << endl << t_encoders.toString() << endl;

	if (right_arm_config.activated)
	{
		cout << "Right arm joint 0 has value: " << CTRL_DEG2RAD*ra_encoders[1] << " (" << ra_encoders[1] << ")" <<  endl;
		cout << "Right arm joint 1 has value: " << CTRL_DEG2RAD*ra_encoders[2] << " (" << ra_encoders[2] << ")" <<  endl;
		cout << "Right arm joint 2 has value: " << CTRL_DEG2RAD*ra_encoders[3] << " (" << ra_encoders[3] << ")" <<  endl;
	}

    cout << "Left eye joint 0 has value: " << CTRL_DEG2RAD*le_encoders[0] << " (" << le_encoders[0] << ")" <<  endl;
	cout << "Left eye joint 1 has value: " << CTRL_DEG2RAD*le_encoders[2] << " (" << le_encoders[2] << ")" <<  endl;
	
	if (left_arm_config.activated)
	{
		cout << "Left arm joint 0 has value: " << CTRL_DEG2RAD*la_encoders[1] << " (" << la_encoders[1] << ")" <<  endl;
		cout << "Left arm joint 1 has value: " << CTRL_DEG2RAD*la_encoders[2] << " (" << la_encoders[2] << ")" <<  endl;
		cout << "Left arm joint 2 has value: " << CTRL_DEG2RAD*la_encoders[3] << " (" << la_encoders[3] << ")" <<  endl;
	}

    cout << "Torso joint 0/2 has value: " << CTRL_DEG2RAD*t_encoders[0] << " (" << t_encoders[0] << ")" <<  endl;

	return 1;
}

void init_kinematics()
{

	//Vectors
	yarp::sig::Vector ra_qf, ra_xf, ra_xfw;
	yarp::sig::Vector le_qf, le_xf, le_xfw;
	yarp::sig::Vector la_qf, la_xf, la_xfw;
    yarp::sig::Vector t_qf;

	//Setup kinematics
	iCubRightArmKin = new ICubKinematics(kin_right_arm);
	iCubLeftEyeKin = new ICubKinematics(kin_left_eye);
	iCubLeftArmKin = new ICubKinematics(kin_left_arm);
    iCubTorsoKin = new ICubKinematics(kin_torso);

	cout << "Right arm has " << iCubRightArmKin->getNumDOF() << " DOF." << endl;
	cout << "Left eye has " << iCubLeftEyeKin->getNumDOF() << " DOF." << endl;
	cout << "Left arm has " << iCubLeftArmKin->getNumDOF() << " DOF." << endl;
    cout << "Torso has " << iCubTorsoKin->getNumDOF() << " DOF." << endl;

	//Change joint angle values for those of encoders
	cout << "Updating values of kinematic chains with the encoder values..." << endl;
	
	//Read encoders and feed them to kinematics library
	yarp::sig::Vector ra_encoders = iCubRightArmCtrl->readEncoders();
	yarp::sig::Vector le_encoders = iCubHeadCtrl->readEncoders();
	yarp::sig::Vector la_encoders = iCubLeftArmCtrl->readEncoders();
    yarp::sig::Vector t_encoders = iCubTorsoCtrl->readEncoders();

	ra_qf = ra_encoders;
	ra_qf.resize(iCubRightArmKin->getNumDOF());
	le_qf = le_encoders;
	le_qf.resize(iCubLeftEyeKin->getNumDOF());
	la_qf = la_encoders;
	la_qf.resize(iCubLeftArmKin->getNumDOF());
    t_qf = t_encoders;
    t_qf.resize(iCubTorsoKin->getNumDOF());
		
	//Convert to radians
	ra_qf = ra_qf*CTRL_DEG2RAD;
	le_qf = le_qf*CTRL_DEG2RAD;
	la_qf = la_qf*CTRL_DEG2RAD;
    t_qf = t_qf*CTRL_DEG2RAD;

	//Set joint values
	ra_qf = iCubRightArmKin->setJointValues(ra_qf);
	cout << "Actual right arm joints set to " << (CTRL_RAD2DEG*ra_qf).toString() << endl;
	le_qf = iCubLeftEyeKin->setJointValues(le_qf);
	cout << "Actual left eye joints set to " << (CTRL_RAD2DEG*le_qf).toString() << endl;
	la_qf = iCubLeftArmKin->setJointValues(la_qf);
	cout << "Actual left arm joints set to " << (CTRL_RAD2DEG*la_qf).toString() << endl;
    t_qf = iCubTorsoKin->setJointValues(t_qf);
    cout << "Actual torso joints set to " << (CTRL_RAD2DEG*t_qf).toString() << endl;
	
    //Initializations
    mu0.clear(); //right hand starting position
    mu0b.clear(); //left hand starting position
	state.clear();
    state_b.clear();
    a.clear(); a.push_back(0); a.push_back(0); a.push_back(0); a.push_back(0); //action for right arm 4 DOF
	a_head.clear(); a_head.push_back(0); a_head.push_back(0); //action for head 2 DOF
    a_b.clear(); a_b.push_back(0); a_b.push_back(0); a_b.push_back(0); a_b.push_back(0); //action for left arm 4 DOF
    mu0.push_back(read_encoders_q1(mu0, a)); mu0.push_back(read_encoders_q2(mu0, a));
    mu0.push_back(read_encoders_q3(mu0, a)); mu0.push_back(read_encoders_q4(mu0, a)); //position
    mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); //velocity
    mu0b.push_back(read_encoders_q1b(mu0b, a)); mu0b.push_back(read_encoders_q2b(mu0b, a));
    mu0b.push_back(read_encoders_q3b(mu0b, a)); mu0b.push_back(read_encoders_q4b(mu0b, a)); //position
    mu0b.push_back(0); mu0b.push_back(0); mu0b.push_back(0); mu0b.push_back(0); //velocity
    le_pos.resize(3); le_pos.zero(); re_pos.resize(3); re_pos.zero(); att_pos.resize(3); att_pos.zero();
    le_xpos.resize(3); le_xpos.zero(); re_xpos.resize(3); re_xpos.zero(); att_xpos.resize(3); att_xpos.zero();
    cout << "Right arm internal variables (mu) set to " << CTRL_RAD2DEG*mu0[0] << ", " << CTRL_RAD2DEG*mu0[1] << ", " << CTRL_RAD2DEG*mu0[2] << ", " << CTRL_RAD2DEG*mu0[3] << endl;
    state = mu0; //set inital state

	//Check end-effector position
	//ra_xf = iCubRightArmKin->getEndEffectorPosition();
    ra_xf.resize(3); la_xf.resize(3);
	ra_xf(0) = sense_3d_position_x(mu0, a);
	ra_xf(1) = sense_3d_position_y(mu0, a);
	ra_xf(2) = sense_3d_position_z(mu0, a);
    la_xf(0) = sense_3d_position_x_b(mu0b, a);
    la_xf(1) = sense_3d_position_y_b(mu0b, a);
    la_xf(2) = sense_3d_position_z_b(mu0b, a);
	ra_xfw = iCubRightArmKin->convertRobot2WorldFrame(ra_xf);
	cout << "Right arm end-effector position: " << ra_xf.toString() << " (" << ra_xfw.toString() << ")"  << endl;
	le_xf = iCubLeftEyeKin->getEndEffectorPosition();
	le_xfw = iCubLeftEyeKin->convertRobot2WorldFrame(le_xf);
	cout << "Left eye end-effector position: " << le_xf.toString() << " (" << le_xfw.toString() << ")"  << endl;
    la_xfw = iCubLeftArmKin->convertRobot2WorldFrame(la_xf);
    cout << "Left arm end-effector position: " << la_xf.toString() << " (" << la_xfw.toString() << ")"  << endl;
	
	//Check calculations
    double x = calc_3d_position_x(mu0, a);
    double y = calc_3d_position_y(mu0, a);
    double z = calc_3d_position_z(mu0, a);
    cout << "Calculated right arm end-effector position: " << x << " " << y << " " << z << endl;
	cout << "Calculated left eye end-effector position: " << calc_le_endeffpos_x() << " " << calc_le_endeffpos_y() << " " << calc_le_endeffpos_z() << endl;
    cout << "Calculated left arm end-effector position: " << calc_3d_position_x_b(mu0b, a) << " " << calc_3d_position_y_b(mu0b, a) << " " << calc_3d_position_z_b(mu0b, a) << endl;
    cout << "Calculated right arm position on left eye camera: " << calc_vision_l_u(mu0, a) << " " << calc_vision_l_v(mu0, a) << endl;
    //cout << "Calculated right arm position on left eye camera (null): " << calc_vision_u(a, a) << " " << calc_vision_v(a, a) << endl;
	calc_le_camera_projection(a, a);
    //cout << "Calculated projection of right arm in camera frame (null): " << cam_projection(0) << " " << cam_projection(1) << " " << cam_projection(2) << endl;
	calc_d_le_camera_projection_mu1(a, a);
    //cout << "Calculated derivative wrt mu1 projection of right arm (null): " << d1_cam_projection(0) << " " << d1_cam_projection(1) << " " << d1_cam_projection(2) << endl;
    //cout << "Calculated derivatives for left eye camera (null): d1u d1v d2u d2v " << d_vision_u_mu1(a,a) << " " << d_vision_v_mu1(a,a) << " " << d_vision_u_mu2(a,a) << " " << d_vision_v_mu2(a,a) << endl;
	
	//Transformation matrix, Jacobian and Hessian
	yarp::sig::Matrix KinAJac, KinH;
	yarp::sig::Vector KinHessian11, KinHessian13, KinHessian33;
	yarp::sig::Vector KinHessian12, KinHessian23, KinHessian22;
	//Transformation matrix
	KinH = iCubRightArmKin->getH();
	cout << "Right arm end-effector transformation matrix: " << endl << KinH.toString() << endl;
	calc_3d_rototranslational_matrix(mu0, a);
	cout << "Calculated right arm end-effector transformation matrix: " << endl << ra_rtm << endl;
    KinH = iCubLeftArmKin->getH();
    cout << "Left arm end-effector transformation matrix: " << endl << KinH.toString() << endl;
    calc_3d_rototranslational_matrix_b(mu0b, a);
    cout << "Calculated left arm end-effector transformation matrix: " << endl << la_rtm << endl;
    KinH = iCubLeftEyeKin->getH();
    cout << "Left eye end-effector transformation matrix: " << endl << KinH.toString() << endl;
    vector<double> dof_le = {le_q4, le_q6};
    calc_le_rototranslational_matrix(dof_le, dof_le);
    cout << "Calculated left eye end-effector transformation matrix: " << endl << le_rtm << endl;
	//Jacobian
	KinAJac = iCubRightArmKin->getAnalyticalJacobian();
	cout << "Right arm end-effector analytical Jacobian: " << endl << KinAJac.toString() << endl;
	calc_3d_jacobian_origin(mu0, a);
	cout << "Calculated right arm end-effector analytical Jacobian: " << endl << ra_3d_jacobian_origin << endl;
    KinAJac = iCubLeftArmKin->getAnalyticalJacobian();
    cout << "Left arm end-effector analytical Jacobian: " << endl << KinAJac.toString() << endl;
    calc_3d_jacobian_origin_b(mu0b, a);
    cout << "Calculated left arm end-effector analytical Jacobian: " << endl << la_3d_jacobian_origin << endl;
    /*//Hessian
	KinHessian11 = iCubRightArmKin->getHessian_ij(1, 1);
	KinHessian13 = iCubRightArmKin->getHessian_ij(1, 3);
	KinHessian12 = iCubRightArmKin->getHessian_ij(1, 2);
	KinHessian22 = iCubRightArmKin->getHessian_ij(2, 2);
	KinHessian23 = iCubRightArmKin->getHessian_ij(2, 3);
	KinHessian33 = iCubRightArmKin->getHessian_ij(3, 3);
	cout << "Right arm end-effector Hessian for (1, 1): " << endl << KinHessian11.toString() << endl;
	cout << "Right arm end-effector Hessian for (1, 2): " << endl << KinHessian12.toString() << endl;
	cout << "Right arm end-effector Hessian for (1, 3): " << endl << KinHessian13.toString() << endl;
	cout << "Right arm end-effector Hessian for (2, 2): " << endl << KinHessian22.toString() << endl;
	cout << "Right arm end-effector Hessian for (2, 3): " << endl << KinHessian23.toString() << endl;
	cout << "Right arm end-effector Hessian for (3, 3): " << endl << KinHessian33.toString() << endl;
	calc_3d_hessian_mu1_origin(mu0, a);
	calc_3d_hessian_mu2_origin(mu0, a);
	calc_3d_hessian_mu3_origin(mu0, a);
	cout << "Calculated right arm end-effector Hessian for (1, 1): " << endl << ra_3d_hessian_mu11_origin/k << endl;
	cout << "Calculated right arm end-effector Hessian for (1, 2): " << endl << ra_3d_hessian_mu12_origin/k << endl;
	cout << "Calculated right arm end-effector Hessian for (1, 3): " << endl << ra_3d_hessian_mu13_origin/k << endl;
	cout << "Calculated right arm end-effector Hessian for (2, 2): " << endl << ra_3d_hessian_mu22_origin/k << endl;
	cout << "Calculated right arm end-effector Hessian for (2, 3): " << endl << ra_3d_hessian_mu23_origin/k << endl;
    cout << "Calculated right arm end-effector Hessian for (3, 3): " << endl << ra_3d_hessian_mu33_origin/k << endl;*/

}

void init_publish()
{
	//right arm
	if (right_arm_config.activated)
	{
		cout << "Setting up publishing ports for right arm..." << endl;

		//open ports for writing
        //encoders and internal variables
		plot_q1.open("/free_energy/q1");
		plot_q2.open("/free_energy/q2");
		plot_q3.open("/free_energy/q3");
		plot_q4.open("/free_energy/q4");
		plot_mu1.open("/free_energy/mu1");
		plot_mu2.open("/free_energy/mu2");
		plot_mu3.open("/free_energy/mu3");
		plot_mu4.open("/free_energy/mu4");
		plot_mu1p.open("/free_energy/mu1p");
		plot_mu2p.open("/free_energy/mu2p");
		plot_mu3p.open("/free_energy/mu3p");
		plot_mu4p.open("/free_energy/mu4p");
		plot_a1.open("/free_energy/a1");
		plot_a2.open("/free_energy/a2");
		plot_a3.open("/free_energy/a3");
		plot_a4.open("/free_energy/a4");
        //end-effector
        plot_calc_l_u.open("/free_energy/calc_l_u");
        plot_calc_l_v.open("/free_energy/calc_l_v");
        plot_calc_r_u.open("/free_energy/calc_r_u");
        plot_calc_r_v.open("/free_energy/calc_r_v");
        plot_calc_x.open("/free_energy/calc_x");
        plot_calc_y.open("/free_energy/calc_y");
        plot_calc_z.open("/free_energy/calc_z");
        plot_x.open("/free_energy/x");
        plot_y.open("/free_energy/y");
        plot_z.open("/free_energy/z");
        plot_l_u.open("/free_energy/l_u");
        plot_l_v.open("/free_energy/l_v");
        plot_l_size.open("/free_energy/l_size");
        plot_r_u.open("/free_energy/r_u");
        plot_r_v.open("/free_energy/r_v");
        plot_r_size.open("/free_energy/r_size");
        //other
		plot_free_energy.open("/free_energy/free_energy");
	}

    if (right_arm_config.on_attractor)
    {
        //attractor
        plot_attr_x.open("/free_energy/attr_x");
        plot_attr_y.open("/free_energy/attr_y");
        plot_attr_z.open("/free_energy/attr_z");
        plot_attr_l_u.open("/free_energy/attr_l_u");
        plot_attr_l_v.open("/free_energy/attr_l_v");
        plot_attr_l_size.open("/free_energy/attr_l_size");
        plot_attr_r_u.open("/free_energy/attr_r_u");
        plot_attr_r_v.open("/free_energy/attr_r_v");
        plot_attr_r_size.open("/free_energy/attr_r_size");
    }

    //head
	if (head_config.activated)
	{
		cout << "Setting up publishing ports for head..." << endl;

		//open ports for writing
		plot_q1e.open("/free_energy/q1e");
		plot_q2e.open("/free_energy/q2e");
        plot_q3e.open("/free_energy/q3e");
		plot_mu1e.open("/free_energy/mu1e");
		plot_mu2e.open("/free_energy/mu2e");
        plot_mu3e.open("/free_energy/mu3e");
		plot_mu1ep.open("/free_energy/mu1ep");
		plot_mu2ep.open("/free_energy/mu2ep");
        plot_mu3ep.open("/free_energy/mu3ep");
		plot_a1e.open("/free_energy/a1e");
		plot_a2e.open("/free_energy/a2e");
        plot_a3e.open("/free_energy/a3e");
		plot_center_x.open("/free_energy/center_x");
		plot_center_y.open("/free_energy/center_y");
        //plot_attr_u.open("/free_energy/attr_u");
        //plot_attr_v.open("/free_energy/attr_v");
        //plot_attr_size.open("/free_energy/attr_size");
		plot_free_energy_e.open("/free_energy/free_energy_e");
	}
	
	//left arm
	if (left_arm_config.activated)
	{
		cout << "Setting up publishing ports for left arm..." << endl;

		//open ports for writing
		plot_q1b.open("/free_energy/q1b");
		plot_q2b.open("/free_energy/q2b");
		plot_q3b.open("/free_energy/q3b");
		plot_q4b.open("/free_energy/q4b");
		plot_mu1b.open("/free_energy/mu1b");
		plot_mu2b.open("/free_energy/mu2b");
		plot_mu3b.open("/free_energy/mu3b");
		plot_mu4b.open("/free_energy/mu4b");
		plot_mu1bp.open("/free_energy/mu1bp");
		plot_mu2bp.open("/free_energy/mu2bp");
		plot_mu3bp.open("/free_energy/mu3bp");
		plot_mu4bp.open("/free_energy/mu4bp");
		plot_a1b.open("/free_energy/a1b");
		plot_a2b.open("/free_energy/a2b");
		plot_a3b.open("/free_energy/a3b");
		plot_a4b.open("/free_energy/a3b");
		plot_ub.open("/free_energy/ub");
		plot_vb.open("/free_energy/vb");
		plot_calc_ub.open("/free_energy/calc_ub");
		plot_calc_vb.open("/free_energy/calc_vb");
		//plot_attr_u.open("/free_energy/attr_u");
		//plot_attr_v.open("/free_energy/attr_v");
		plot_free_energy_b.open("/free_energy/free_energy_b");
	}

}

void init_camera()
{
    //Define camera parameters
    w = 320;
    h = 240;
    //Right camera
    fx_R = 219.333;
    fy_R = 218.401;
    cx_R = 157.286;
    cy_R = 147.51;
    k1_R = -0.381054;
    k2_R = 0.149796;
    p1_R = -0.000598093;
    p2_R = -6.43865e-06;
    //Left camera
    fx_L = 223.157;
    fy_L = 222.183;
    cx_L = 167.434;
    cy_L = 105.917;
    k1_L = -0.375105;
    k2_L = 0.153483;
    p1_L = 0.000383406;
    p2_L = 0.000257203;

	//Define camera matrices
    matrixCam1 = (Mat_<double>(3,3) << fx_L, 0, cx_L, 0, fy_L, cy_L, 0, 0, 1);
    matrixCam2 = (Mat_<double>(3,3) << fx_R, 0, cx_R, 0, fy_R, cy_R, 0, 0, 1);
    distCam1 = (Mat_<double>(4,1) << k1_L, k2_L, p1_L, p2_L);
    distCam2 = (Mat_<double>(4,1) << k1_R, k2_R, p1_R, p2_R);
    matrixRot = (Mat_<double>(3,3) << 0.990555, 0.0163678, 0.136135, -0.0132483, 0.999629, -0.0237897, -0.136474, 0.0217614, 0.990405);
    matrixTrans = (Mat_<double>(3,1) << -0.0678886, -0.000401475, 0.00851381);

    //Correct from R->L to L->R
    //matrixRot = matrixRot.t();
    //matrixTrans = -matrixRot*matrixTrans;

	cout << "Stereo vision camera parameters:" << endl;
    cout << "matrixCam1 (L): " << endl << matrixCam1 << endl;
    cout << "matrixCam2 (R): " << endl << matrixCam2 << endl;
    cout << "distCam1 (L): " << endl << distCam1 << endl;
    cout << "distCam2 (R): " << endl << distCam2 << endl;
    cout << "matrixRot (L->R): " << endl << matrixRot << endl;
    cout << "matrixTrans (L->R): " << endl << matrixTrans << endl;

    //Initialize stereo vision
    if (right_arm_config.activated) iCubImgRL = new ICubImage("icub", main_config.debug);
    if (right_arm_config.on_attractor) iCubImgRLA = new ICubImage("icub", main_config.debug);
    if (left_arm_config.activated) iCubImgRLb = new ICubImage("icub", main_config.debug);

	cout << "Setting up left eye image OpenCV reading..." << endl;
	vector<string> ports, ports_att;
    ports.push_back("/icub_controller/cam/right");
	ports.push_back("/icub_controller/cam/left");
    ports_att.push_back("/icub_attractor/cam/right");
	ports_att.push_back("/icub_attractor/cam/left");
    if (right_arm_config.activated) iCubImgRL->setupImageRead(ports, RL);
    if (right_arm_config.on_attractor || left_arm_config.on_attractor) iCubImgRLA->setupImageRead(ports_att, RL);
    if (left_arm_config.activated) iCubImgRLb->setupImageRead(ports, RL);

    //Set camera parameters
    if (right_arm_config.activated) iCubImgRL->setParameters3D(matrixCam1, distCam1, matrixCam2, distCam2, matrixRot, matrixTrans);
    if (right_arm_config.on_attractor || left_arm_config.on_attractor) iCubImgRLA->setParameters3D(matrixCam1, distCam1, matrixCam2, distCam2, matrixRot, matrixTrans);
    if (left_arm_config.activated) iCubImgRLb->setParameters3D(matrixCam1, distCam1, matrixCam2, distCam2, matrixRot, matrixTrans);

    if (right_arm_config.activated)
    {
        //Set up object color recognition parameters for end-effector
        left_hue_low = right_arm_config.left_vision_hue_low; //vision hue lower bound
        left_sat_low = right_arm_config.left_vision_sat_low; //vision saturation lower bound
        left_val_low = right_arm_config.left_vision_val_low; //vision value lower bound
        left_hue_high = right_arm_config.left_vision_hue_high; //vision hue upper bound
        left_sat_high = right_arm_config.left_vision_sat_high; //vision saturation upper bound
        left_val_high = right_arm_config.left_vision_val_high; //vision value upper bound
        right_hue_low = right_arm_config.right_vision_hue_low; //vision hue lower bound
        right_sat_low = right_arm_config.right_vision_sat_low; //vision saturation lower bound
        right_val_low = right_arm_config.right_vision_val_low; //vision value lower bound
        right_hue_high = right_arm_config.right_vision_hue_high; //vision hue upper bound
        right_sat_high = right_arm_config.right_vision_sat_high; //vision saturation upper bound
        right_val_high = right_arm_config.right_vision_val_high; //vision value upper bound

        //Find object in camera
        Scalar left_col_low = Scalar(left_hue_low, left_sat_low, left_val_low);
        Scalar left_col_high = Scalar(left_hue_high, left_sat_high, left_val_high);
        Scalar right_col_low = Scalar(right_hue_low, right_sat_low, right_val_low);
        Scalar right_col_high = Scalar(right_hue_high, right_sat_high, right_val_high);
        re_pos = iCubImgRL->get3Dfrom2D(RL, left_col_low, left_col_high, right_arm_config.left_erode_dilate_iterations,
								right_col_low, right_col_high, right_arm_config.right_erode_dilate_iterations, 3, false);

        cout << "Left eye relative right arm position: (" << re_pos[0] << ", " << re_pos[1] << ", " << re_pos[2] << ")" <<  endl;
        cout << "Left eye camera right arm position: (" << re_pos[3] << ", " << re_pos[4] << "), size " << re_pos[5] << endl;
        cout << "Right eye camera right arm position: (" << re_pos[6] << ", " << re_pos[7] << "), size " << re_pos[8] << endl;
        double x, y, z;
        x = sense_3d_position_x(mu0,a); y = sense_3d_position_y(mu0,a); z = sense_3d_position_z(mu0,a);
        cout << "Right arm position in world: (" << x << ", " << y << ", " << z << ")" <<  endl;
    }

	if (right_arm_config.on_attractor)
	{
		//initialize vector
        prev_att_pos.resize(3);
        prev_att_pos.zero();

        att_pos.resize(3);
        att_pos.zero();

		//Set up object color recognition parameters for attractor
		left_att_hue_low = right_arm_config.left_attractor_hue_low; //vision hue lower bound
		left_att_sat_low = right_arm_config.left_attractor_sat_low; //vision saturation lower bound
		left_att_val_low = right_arm_config.left_attractor_val_low; //vision value lower bound
		left_att_hue_high = right_arm_config.left_attractor_hue_high; //vision hue upper bound
		left_att_sat_high = right_arm_config.left_attractor_sat_high; //vision saturation upper bound
		left_att_val_high = right_arm_config.left_attractor_val_high; //vision value upper bound
		right_att_hue_low = right_arm_config.right_attractor_hue_low; //vision hue lower bound
		right_att_sat_low = right_arm_config.right_attractor_sat_low; //vision saturation lower bound
		right_att_val_low = right_arm_config.right_attractor_val_low; //vision value lower bound
		right_att_hue_high = right_arm_config.right_attractor_hue_high; //vision hue upper bound
		right_att_sat_high = right_arm_config.right_attractor_sat_high; //vision saturation upper bound
		right_att_val_high = right_arm_config.right_attractor_val_high; //vision value upper bound
		//read image and find location of attractor
		Scalar left_att_col_low = Scalar(left_att_hue_low, left_att_sat_low, left_att_val_low);
		Scalar left_att_col_high = Scalar(left_att_hue_high, left_att_sat_high, left_att_val_high);
		Scalar right_att_col_low = Scalar(right_att_hue_low, right_att_sat_low, right_att_val_low);
		Scalar right_att_col_high = Scalar(right_att_hue_high, right_att_sat_high, right_att_val_high);

		switch (right_arm_config.attractor_type)
		{
			case 0: //stereo vision with hsv recognition (position x-y-z)
                //att_pos = iCubImgLA->findLocationCV(L, att_col_low, att_col_high, right_arm_config.erode_dilate_iterations, false);
                att_pos = iCubImgRLA->get3Dfrom2D(RL, left_att_col_low, left_att_col_high, right_arm_config.left_erode_dilate_iterations, 
									right_att_col_low, right_att_col_high, right_arm_config.right_erode_dilate_iterations, 3, false);
                attr_3d_pos(0) = sense_3d_attractor_x(mu0,a); attr_3d_pos(1) = sense_3d_attractor_y(mu0,a); attr_3d_pos(2) = sense_3d_attractor_z(mu0,a);
				break;
            case 1: //3d fixed position (x-y-z)
                attr_3d_pos(0) = ro2; attr_3d_pos(1) = ro3; attr_3d_pos(2) = ro4;
                att_pos[0] = 0; att_pos[1] = 0; att_pos[2] = 0; //left eye relative position
                att_pos[3] = calc_attractor_l_u(); att_pos[4] = calc_attractor_l_v(); att_pos[5] = 0; //left eye camera position
                att_pos[6] = calc_attractor_r_u(); att_pos[7] = calc_attractor_r_v(); att_pos[8] = 0; //right eye camera position
                break;
            case 2: //3d cube (cycle)
                attr_3d_pos(0) = ro2; attr_3d_pos(1) = ro3; attr_3d_pos(2) = ro4;
                att_pos[0] = 0; att_pos[1] = 0; att_pos[2] = 0; //left eye relative position
                att_pos[3] = calc_attractor_l_u(); att_pos[4] = calc_attractor_l_v(); att_pos[5] = 0; //left eye camera position
                att_pos[6] = calc_attractor_r_u(); att_pos[7] = calc_attractor_r_v(); att_pos[8] = 0; //right eye camera position
                cout << "Starting cube attractor..." << endl;
				att_state = 0; //first position of cube
                break;
		}

        cout << "Left eye relative attractor position: (" << att_pos[0] << ", " << att_pos[1] << ", " << att_pos[2] << ")" <<  endl;
		cout << "Left eye camera attractor position: (" << att_pos[3] << ", " << att_pos[4] << "), size " << att_pos[5] << endl;
        cout << "Right eye camera attractor position: (" << att_pos[6] << ", " << att_pos[7] << "), size " << att_pos[8] << endl;

        cout << "Attractor position in world: (" << attr_3d_pos(0) << ", " << attr_3d_pos(1) << ", " << attr_3d_pos(2) << ")" <<  endl;
	}

    if (left_arm_config.activated && left_arm_config.on_vision)
    {
        //Set up object color recognition parameters for end-effector
        left_hue_low_b = left_arm_config.left_vision_hue_low; //vision hue lower bound
        left_sat_low_b = left_arm_config.left_vision_sat_low; //vision saturation lower bound
        left_val_low_b = left_arm_config.left_vision_val_low; //vision value lower bound
        left_hue_high_b = left_arm_config.left_vision_hue_high; //vision hue upper bound
        left_sat_high_b = left_arm_config.left_vision_sat_high; //vision saturation upper bound
        left_val_high_b = left_arm_config.left_vision_val_high; //vision value upper bound
		right_hue_low_b = left_arm_config.right_vision_hue_low; //vision hue lower bound
        right_sat_low_b = left_arm_config.right_vision_sat_low; //vision saturation lower bound
        right_val_low_b = left_arm_config.right_vision_val_low; //vision value lower bound
        right_hue_high_b = left_arm_config.right_vision_hue_high; //vision hue upper bound
        right_sat_high_b = left_arm_config.right_vision_sat_high; //vision saturation upper bound
        right_val_high_b = left_arm_config.right_vision_val_high; //vision value upper bound

        //Find object in camera
        Scalar left_col_low_b = Scalar(left_hue_low_b, left_sat_low_b, left_val_low_b);
        Scalar left_col_high_b = Scalar(left_hue_high_b, left_sat_high_b, left_val_high_b);
        Scalar right_col_low_b = Scalar(right_hue_low_b, right_sat_low_b, right_val_low_b);
        Scalar right_col_high_b = Scalar(right_hue_high_b, right_sat_high_b, right_val_high_b);
        //le_pos = iCubImgLb->findLocationCV(L, col_low_b, col_high_b, left_arm_config.erode_dilate_iterations, false);
        le_pos = iCubImgRLb->get3Dfrom2D(RL, left_col_low_b, left_col_high_b, left_arm_config.left_erode_dilate_iterations, 
							right_col_low_b, right_col_high_b, left_arm_config.right_erode_dilate_iterations, 0, false);

        cout << "Left eye relative left arm position: (" << le_pos[0] << ", " << le_pos[1] << ", " << le_pos[2] << ")" <<  endl;
        cout << "Left arm position in world: (" << sense_3d_position_x_b(mu0,a) << ", " << sense_3d_position_y_b(mu0,a) << ", " << sense_3d_position_z_b(mu0,a) << ")" <<  endl;
    }

	//Start thread for image reading
	visionThread = new YarpPeriodic(0.5*dT*1000, "vision");
	visionThread->setRun(sense_vision_thread);
	visionThread->start();
}

void init_touch()
{
    if (right_arm_config.on_touch && right_arm_config.activated)
    {

        //Initialize right hand touch
        iCubRightHandTouch = new ICubTouch("icub");

        cout << "Setting up right hand touch sensors..." << endl;

        iCubRightHandTouch->setupTouch(right_hand, "/test/touch/right", main_config.debug);

        yarp::sig::Vector rht = iCubRightHandTouch->readTouchSensors();

    }

    if (left_arm_config.on_touch && left_arm_config.activated)
    {

        //Initialize left hand touch
        iCubLeftHandTouch = new ICubTouch("icub");

        cout << "Setting up left hand touch sensors..." << endl;

        iCubLeftHandTouch->setupTouch(left_hand, "/test/touch/left", main_config.debug);

        yarp::sig::Vector lht = iCubLeftHandTouch->readTouchSensors();

    }

	//cout << "Right hand touch sensor data: " << rht.toString() << endl;
    //cout << "Left hand touch sensor data: " << lht.toString() << endl;

	//Start thread for contact reading
	touchThread = new YarpPeriodic(0.5*dT*1000, "touch");
	touchThread->setRun(set_touch_thread);
	touchThread->start();
}

void init_noise()
{
    cout << "Setting up gaussian noise generator (Mersenne twister PRNG)...";
	cout << " N(" << right_arm_config.gaussian_mean*CTRL_DEG2RAD << ", " << right_arm_config.gaussian_std*CTRL_DEG2RAD << ")" << endl;
	mt19937_64::result_type seed = chrono::high_resolution_clock::now().time_since_epoch().count(); //random seed
	gen = new mt19937(seed); //Mersenne twister PRNG
	ndis = new normal_distribution<double>(right_arm_config.gaussian_mean*CTRL_DEG2RAD, right_arm_config.gaussian_std*CTRL_DEG2RAD); 
	cout << "Random values: " << (*ndis)(*gen) << " " << (*ndis)(*gen) << " " << (*ndis)(*gen) << " " << endl;
}

//----------------------------------------------------------------------------------
//Config and logging ---------------------------------------------------------------
//----------------------------------------------------------------------------------

void config_file_parser(string filename)
{
	ifstream fin(filename);
	string line;
	while (getline(fin, line)) {
		istringstream sin(line.substr(line.find("=") + 1));
		
		//main config
		if (filename == "config.ini")
		{
			if (line.find("robot") != -1)
			    sin >> main_config.robot;
			else if (line.find("debug") != -1)
			    sin >> main_config.debug;
			else if (line.find("print") != -1)
			    sin >> main_config.print;
			else if (line.find("dT") != -1)
			    sin >> main_config.dT;
			else if (line.find("integration") != -1)
			    sin >> main_config.integration_method;
			else if (line.find("log") != -1)
			    sin >> main_config.log;
            else if (line.find("description") != -1)
                sin >> main_config.description;
			else if (line.find("publish") != -1)
			    sin >> main_config.publish;
			else if (line.find("cycles") != -1)
			    sin >> main_config.cycles;
		}

		//right arm config
		if (filename == "ra_config.ini")
		{
			if (line.find("activated") != -1)
			    sin >> right_arm_config.activated;
			//initial position
            else if (line.find("torso_yaw") != -1)
                sin >> right_arm_config.torso_yaw;
			else if (line.find("shoulder_roll") != -1)
			    sin >> right_arm_config.shoulder_roll;
			else if (line.find("shoulder_yaw") != -1)
			    sin >> right_arm_config.shoulder_yaw;
			else if (line.find("elbow") != -1)
			    sin >> right_arm_config.elbow;
			//internal state
            else if (line.find("on_encoders") != -1)
                sin >> right_arm_config.on_encoders;
			else if (line.find("var_d_encoders") != -1)
			    sin >> right_arm_config.var_d_encoders;
			else if (line.find("on_3d") != -1)
			    sin >> right_arm_config.on_3d;
			else if (line.find("var_d_3d") != -1)
			    sin >> right_arm_config.var_d_3d;
			else if (line.find("on_vision") != -1)
			    sin >> right_arm_config.on_vision;
			else if (line.find("var_d_vision") != -1)
			    sin >> right_arm_config.var_d_vision;
			else if (line.find("left_vision_hue_low") != -1)
			    sin >> right_arm_config.left_vision_hue_low;
			else if (line.find("left_vision_sat_low") != -1)
			    sin >> right_arm_config.left_vision_sat_low;
			else if (line.find("left_vision_val_low") != -1)
			    sin >> right_arm_config.left_vision_val_low;
			else if (line.find("left_vision_hue_high") != -1)
			    sin >> right_arm_config.left_vision_hue_high;
			else if (line.find("left_vision_sat_high") != -1)
			    sin >> right_arm_config.left_vision_sat_high;
			else if (line.find("left_vision_val_high") != -1)
			    sin >> right_arm_config.left_vision_val_high;
			else if (line.find("right_vision_hue_low") != -1)
			    sin >> right_arm_config.right_vision_hue_low;
			else if (line.find("right_vision_sat_low") != -1)
			    sin >> right_arm_config.right_vision_sat_low;
			else if (line.find("right_vision_val_low") != -1)
			    sin >> right_arm_config.right_vision_val_low;
			else if (line.find("right_vision_hue_high") != -1)
			    sin >> right_arm_config.right_vision_hue_high;
			else if (line.find("right_vision_sat_high") != -1)
			    sin >> right_arm_config.right_vision_sat_high;
			else if (line.find("right_vision_val_high") != -1)
			    sin >> right_arm_config.right_vision_val_high;
			else if (line.find("left_erode_dilate_iterations") != -1)
			    sin >> right_arm_config.left_erode_dilate_iterations;
			else if (line.find("right_erode_dilate_iterations") != -1)
			    sin >> right_arm_config.right_erode_dilate_iterations;
			else if (line.find("gain_perception_mu1") != -1)
			    sin >> right_arm_config.gain_perception_mu1;
			else if (line.find("gain_perception_mu2") != -1)
			    sin >> right_arm_config.gain_perception_mu2;
			else if (line.find("gain_perception_mu3") != -1)
			    sin >> right_arm_config.gain_perception_mu3;
			else if (line.find("gain_perception_mu4") != -1)
			    sin >> right_arm_config.gain_perception_mu4;
			else if (line.find("weighted_perception_mu1") != -1)
			    sin >> right_arm_config.weighted_perception_mu1;
			else if (line.find("weighted_perception_mu2") != -1)
			    sin >> right_arm_config.weighted_perception_mu2;
			else if (line.find("weighted_perception_mu3") != -1)
			    sin >> right_arm_config.weighted_perception_mu3;
			else if (line.find("weighted_perception_mu4") != -1)
			    sin >> right_arm_config.weighted_perception_mu4;
			else if (line.find("perception_saturation_on") != -1)
			    sin >> right_arm_config.perception_saturation_on;
			else if (line.find("mu1_saturation_low") != -1)
			    sin >> right_arm_config.mu1_saturation_low;	
			else if (line.find("mu1_saturation_high") != -1)
			    sin >> right_arm_config.mu1_saturation_high;
			else if (line.find("mu2_saturation_low") != -1)
			    sin >> right_arm_config.mu2_saturation_low;	
			else if (line.find("mu2_saturation_high") != -1)
			    sin >> right_arm_config.mu2_saturation_high;
			else if (line.find("mu3_saturation_low") != -1)
			    sin >> right_arm_config.mu3_saturation_low;	
			else if (line.find("mu3_saturation_high") != -1)
			    sin >> right_arm_config.mu3_saturation_high;
			else if (line.find("mu4_saturation_low") != -1)
			    sin >> right_arm_config.mu4_saturation_low;	
			else if (line.find("mu4_saturation_high") != -1)
			    sin >> right_arm_config.mu4_saturation_high;
			//prior
			else if (line.find("on_prior") != -1)
			    sin >> right_arm_config.on_prior;
			else if (line.find("var_d_prior") != -1)
			    sin >> right_arm_config.var_d_prior;
			//first derivative
			else if (line.find("on_derivative") != -1)
			    sin >> right_arm_config.on_derivative;
			else if (line.find("var_dd_attractor") != -1)
			    sin >> right_arm_config.var_dd_attractor;
			else if (line.find("gain_derivative_mu1p") != -1)
			    sin >> right_arm_config.gain_derivative_mu1p;
			else if (line.find("gain_derivative_mu2p") != -1)
			    sin >> right_arm_config.gain_derivative_mu2p;
			else if (line.find("gain_derivative_mu3p") != -1)
			    sin >> right_arm_config.gain_derivative_mu3p;
			else if (line.find("derivative_saturation_on") != -1)
			    sin >> right_arm_config.derivative_saturation_on;	
			else if (line.find("mu1p_saturation_low") != -1)
			    sin >> right_arm_config.mu1p_saturation_low;	
			else if (line.find("mu1p_saturation_high") != -1)
			    sin >> right_arm_config.mu1p_saturation_high;
			else if (line.find("mu2p_saturation_low") != -1)
			    sin >> right_arm_config.mu2p_saturation_low;	
			else if (line.find("mu2p_saturation_high") != -1)
			    sin >> right_arm_config.mu2p_saturation_high;
			else if (line.find("mu3p_saturation_low") != -1)
			    sin >> right_arm_config.mu3p_saturation_low;	
			else if (line.find("mu3p_saturation_high") != -1)
			    sin >> right_arm_config.mu3p_saturation_high;
			else if (line.find("mu4p_saturation_low") != -1)
			    sin >> right_arm_config.mu4p_saturation_low;	
			else if (line.find("mu4p_saturation_high") != -1)
			    sin >> right_arm_config.mu4p_saturation_high;
			//action
			else if (line.find("on_action") != -1)
			    sin >> right_arm_config.on_action;
			else if (line.find("var_a_encoders") != -1)
			    sin >> right_arm_config.var_a_encoders;
			else if (line.find("var_a_3d") != -1)
			    sin >> right_arm_config.var_a_3d;
			else if (line.find("var_a_vision") != -1)
			    sin >> right_arm_config.var_a_vision;
			else if (line.find("gain_action_q1") != -1)
			    sin >> right_arm_config.gain_action_q1;
			else if (line.find("gain_action_q2") != -1)
			    sin >> right_arm_config.gain_action_q2;
			else if (line.find("gain_action_q3") != -1)
			    sin >> right_arm_config.gain_action_q3;
			else if (line.find("gain_action_q4") != -1)
			    sin >> right_arm_config.gain_action_q4;
			else if (line.find("action_saturation_on") != -1)
			    sin >> right_arm_config.action_saturation_on;	
			else if (line.find("action_saturation_low") != -1)
			    sin >> right_arm_config.action_saturation_low;	
			else if (line.find("action_saturation_high") != -1)
			    sin >> right_arm_config.action_saturation_high;
			//attractor
			else if (line.find("on_attractor") != -1)
			    sin >> right_arm_config.on_attractor;
			else if (line.find("attractor_type") != -1)
			    sin >> right_arm_config.attractor_type;		
			else if (line.find("ro1_attractor") != -1)
			    sin >> right_arm_config.ro1_attractor;
			else if (line.find("ro2_attractor") != -1)
			    sin >> right_arm_config.ro2_attractor;
			else if (line.find("ro3_attractor") != -1)
			    sin >> right_arm_config.ro3_attractor;
			else if (line.find("ro4_attractor") != -1)
			    sin >> right_arm_config.ro4_attractor;
            else if (line.find("ro5_attractor") != -1)
                sin >> right_arm_config.ro5_attractor;
            else if (line.find("ro6_attractor") != -1)
                sin >> right_arm_config.ro6_attractor;
            else if (line.find("ro7_attractor") != -1)
                sin >> right_arm_config.ro7_attractor;
            else if (line.find("ro8_attractor") != -1)
                sin >> right_arm_config.ro8_attractor;
            else if (line.find("ro9_attractor") != -1)
				sin >> right_arm_config.ro9_attractor;
			else if (line.find("ro10_attractor") != -1)
				sin >> right_arm_config.ro10_attractor;
			else if (line.find("ro11_attractor") != -1)
				sin >> right_arm_config.ro11_attractor;
			else if (line.find("ro12_attractor") != -1)
				sin >> right_arm_config.ro12_attractor;
			else if (line.find("ro13_attractor") != -1)
				sin >> right_arm_config.ro13_attractor;
			else if (line.find("ro14_attractor") != -1)
				sin >> right_arm_config.ro14_attractor;
			else if (line.find("ro15_attractor") != -1)
				sin >> right_arm_config.ro15_attractor;
			else if (line.find("ro16_attractor") != -1)
				sin >> right_arm_config.ro16_attractor;
			else if (line.find("ro17_attractor") != -1)
				sin >> right_arm_config.ro17_attractor;
			else if (line.find("ro18_attractor") != -1)
				sin >> right_arm_config.ro18_attractor;
			else if (line.find("ro19_attractor") != -1)
				sin >> right_arm_config.ro19_attractor;
			else if (line.find("ro20_attractor") != -1)
				sin >> right_arm_config.ro20_attractor;
			else if (line.find("ro21_attractor") != -1)
				sin >> right_arm_config.ro21_attractor;
			else if (line.find("ro22_attractor") != -1)
				sin >> right_arm_config.ro22_attractor;
			else if (line.find("ro23_attractor") != -1)
				sin >> right_arm_config.ro23_attractor;
			else if (line.find("ro24_attractor") != -1)
				sin >> right_arm_config.ro24_attractor;
			else if (line.find("ro25_attractor") != -1)
				sin >> right_arm_config.ro25_attractor;
            else if (line.find("attractor_threshold") != -1)
                sin >> right_arm_config.attractor_threshold;
            else if (line.find("attractor_comparison") != -1)
                sin >> right_arm_config.attractor_comparison;
			else if (line.find("var_d_attractor") != -1)
			    sin >> right_arm_config.var_d_attractor;
			else if (line.find("left_attractor_hue_low") != -1)
			    sin >> right_arm_config.left_attractor_hue_low;
			else if (line.find("left_attractor_sat_low") != -1)
			    sin >> right_arm_config.left_attractor_sat_low;
			else if (line.find("left_attractor_val_low") != -1)
			    sin >> right_arm_config.left_attractor_val_low;
			else if (line.find("left_attractor_hue_high") != -1)
			    sin >> right_arm_config.left_attractor_hue_high;
			else if (line.find("left_attractor_sat_high") != -1)
			    sin >> right_arm_config.left_attractor_sat_high;
			else if (line.find("left_attractor_val_high") != -1)
				sin >> right_arm_config.left_attractor_val_high;
			else if (line.find("right_attractor_hue_low") != -1)
			    sin >> right_arm_config.right_attractor_hue_low;
			else if (line.find("right_attractor_sat_low") != -1)
			    sin >> right_arm_config.right_attractor_sat_low;
			else if (line.find("right_attractor_val_low") != -1)
			    sin >> right_arm_config.right_attractor_val_low;
			else if (line.find("right_attractor_hue_high") != -1)
			    sin >> right_arm_config.right_attractor_hue_high;
			else if (line.find("right_attractor_sat_high") != -1)
			    sin >> right_arm_config.right_attractor_sat_high;
			else if (line.find("right_attractor_val_high") != -1)
			    sin >> right_arm_config.right_attractor_val_high;
			else if (line.find("fixed_attractor") != -1)
			    sin >> right_arm_config.fixed_attractor;
			//touch
			else if (line.find("on_touch") != -1)
			    sin >> right_arm_config.on_touch;
            else if (line.find("touch_threshold") != -1)
                sin >> right_arm_config.touch_threshold;
			else if (line.find("grasping") != -1)
			    sin >> right_arm_config.grasping;
			//noise		
			else if (line.find("gaussian_noise") != -1)
			    sin >> right_arm_config.gaussian_noise;
			else if (line.find("gaussian_mean") != -1)
			    sin >> right_arm_config.gaussian_mean;
			else if (line.find("gaussian_std") != -1)
			    sin >> right_arm_config.gaussian_std;
			//other
			else if (line.find("null_start") != -1)
			    sin >> right_arm_config.null_start;	
			
		}

		
		//head config
		if (filename == "head_config.ini")
		{
			if (line.find("activated") != -1)
			    sin >> head_config.activated;
			//initial position
			else if (line.find("neck_pitch") != -1)
			    sin >> head_config.neck_pitch;
			else if (line.find("neck_yaw") != -1)
			    sin >> head_config.neck_yaw;
            else if (line.find("eyes_tilt") != -1)
                sin >> head_config.eyes_tilt;
			//internal state
			else if (line.find("var_d_encoders") != -1)
			    sin >> head_config.var_d_encoders;
			else if (line.find("gain_perception_mu1") != -1)
			    sin >> head_config.gain_perception_mu1;
			else if (line.find("gain_perception_mu2") != -1)
			    sin >> head_config.gain_perception_mu2;
            else if (line.find("gain_perception_mu3") != -1)
                sin >> head_config.gain_perception_mu3;
			else if (line.find("weighted_perception_mu1") != -1)
			    sin >> head_config.weighted_perception_mu1;
			else if (line.find("weighted_perception_mu2") != -1)
			    sin >> head_config.weighted_perception_mu2;
            else if (line.find("weighted_perception_mu3") != -1)
                sin >> head_config.weighted_perception_mu3;
			else if (line.find("perception_saturation_on") != -1)
			    sin >> head_config.perception_saturation_on;	
			else if (line.find("mu1_saturation_low") != -1)
			    sin >> head_config.mu1_saturation_low;	
			else if (line.find("mu1_saturation_high") != -1)
			    sin >> head_config.mu1_saturation_high;
			else if (line.find("mu2_saturation_low") != -1)
			    sin >> head_config.mu2_saturation_low;	
			else if (line.find("mu2_saturation_high") != -1)
			    sin >> head_config.mu2_saturation_high;
            else if (line.find("mu3_saturation_low") != -1)
                sin >> head_config.mu3_saturation_low;
            else if (line.find("mu3_saturation_high") != -1)
                sin >> head_config.mu3_saturation_high;
			//first derivative
			else if (line.find("on_derivative") != -1)
			    sin >> head_config.on_derivative;
			else if (line.find("var_dd_attractor") != -1)
			    sin >> head_config.var_dd_attractor;
			else if (line.find("gain_derivative_mu1p") != -1)
			    sin >> head_config.gain_derivative_mu1p;
			else if (line.find("gain_derivative_mu2p") != -1)
			    sin >> head_config.gain_derivative_mu2p;
            else if (line.find("gain_derivative_mu3p") != -1)
                sin >> head_config.gain_derivative_mu3p;
			else if (line.find("derivative_saturation_on") != -1)
			    sin >> head_config.derivative_saturation_on;	
			else if (line.find("mu1p_saturation_low") != -1)
			    sin >> head_config.mu1p_saturation_low;	
			else if (line.find("mu1p_saturation_high") != -1)
			    sin >> head_config.mu1p_saturation_high;
			else if (line.find("mu2p_saturation_low") != -1)
			    sin >> head_config.mu2p_saturation_low;	
			else if (line.find("mu2p_saturation_high") != -1)
			    sin >> head_config.mu2p_saturation_high;
            else if (line.find("mu3p_saturation_low") != -1)
                sin >> head_config.mu3p_saturation_low;
            else if (line.find("mu3p_saturation_high") != -1)
                sin >> head_config.mu3p_saturation_high;
			//action
			else if (line.find("on_action") != -1)
			    sin >> head_config.on_action;
			else if (line.find("var_a_encoders") != -1)
			    sin >> head_config.var_a_encoders;
			else if (line.find("gain_action_q1") != -1)
			    sin >> head_config.gain_action_q1;
			else if (line.find("gain_action_q2") != -1)
			    sin >> head_config.gain_action_q2;	
            else if (line.find("gain_action_q3") != -1)
                sin >> head_config.gain_action_q3;
			else if (line.find("action_saturation_on") != -1)
			    sin >> head_config.action_saturation_on;	
			else if (line.find("action_saturation_low") != -1)
			    sin >> head_config.action_saturation_low;	
			else if (line.find("action_saturation_high") != -1)
			    sin >> head_config.action_saturation_high;
			//attractor
			else if (line.find("ro1_attractor") != -1)
			    sin >> head_config.ro1_attractor;
			else if (line.find("var_d_attractor") != -1)
			    sin >> head_config.var_d_attractor;
			//other
			else if (line.find("null_start") != -1)
			    sin >> head_config.null_start;	
		}
		
		
		//left arm config
		if (filename == "la_config.ini")
		{
			if (line.find("activated") != -1)
			    sin >> left_arm_config.activated;
			//initial position
			else if (line.find("shoulder_roll") != -1)
			    sin >> left_arm_config.shoulder_roll;
			else if (line.find("shoulder_yaw") != -1)
			    sin >> left_arm_config.shoulder_yaw;
			else if (line.find("elbow") != -1)
			    sin >> left_arm_config.elbow;
			//internal state
            else if (line.find("on_encoders") != -1)
                sin >> left_arm_config.on_encoders;
			else if (line.find("var_d_encoders") != -1)
			    sin >> left_arm_config.var_d_encoders;
			else if (line.find("on_3d") != -1)
			    sin >> left_arm_config.on_3d;
			else if (line.find("var_d_3d") != -1)
			    sin >> left_arm_config.var_d_3d;
			else if (line.find("on_vision") != -1)
			    sin >> left_arm_config.on_vision;
			else if (line.find("var_d_vision") != -1)
			    sin >> left_arm_config.var_d_vision;
			else if (line.find("left_vision_hue_low") != -1)
			    sin >> left_arm_config.left_vision_hue_low;
			else if (line.find("left_vision_sat_low") != -1)
			    sin >> left_arm_config.left_vision_sat_low;
			else if (line.find("left_vision_val_low") != -1)
			    sin >> left_arm_config.left_vision_val_low;
			else if (line.find("left_vision_hue_high") != -1)
			    sin >> left_arm_config.left_vision_hue_high;
			else if (line.find("left_vision_sat_high") != -1)
			    sin >> left_arm_config.left_vision_sat_high;
            else if (line.find("left_vision_val_high") != -1)
			    sin >> left_arm_config.left_vision_val_high;
			else if (line.find("right_vision_hue_low") != -1)
			    sin >> left_arm_config.right_vision_hue_low;
			else if (line.find("right_vision_sat_low") != -1)
			    sin >> left_arm_config.right_vision_sat_low;
			else if (line.find("right_vision_val_low") != -1)
			    sin >> left_arm_config.right_vision_val_low;
			else if (line.find("right_vision_hue_high") != -1)
			    sin >> left_arm_config.right_vision_hue_high;
			else if (line.find("right_vision_sat_high") != -1)
			    sin >> left_arm_config.right_vision_sat_high;
			else if (line.find("right_vision_val_high") != -1)
			    sin >> left_arm_config.right_vision_val_high;
			else if (line.find("left_erode_dilate_iterations") != -1)
			    sin >> left_arm_config.left_erode_dilate_iterations;
			else if (line.find("right_erode_dilate_iterations") != -1)
			    sin >> left_arm_config.right_erode_dilate_iterations;
			else if (line.find("gain_perception_mu1") != -1)
			    sin >> left_arm_config.gain_perception_mu1;
			else if (line.find("gain_perception_mu2") != -1)
			    sin >> left_arm_config.gain_perception_mu2;
			else if (line.find("gain_perception_mu3") != -1)
			    sin >> left_arm_config.gain_perception_mu3;
			else if (line.find("gain_perception_mu4") != -1)
			    sin >> left_arm_config.gain_perception_mu4;
			else if (line.find("weighted_perception_mu1") != -1)
			    sin >> left_arm_config.weighted_perception_mu1;
			else if (line.find("weighted_perception_mu2") != -1)
			    sin >> left_arm_config.weighted_perception_mu2;
			else if (line.find("weighted_perception_mu3") != -1)
			    sin >> left_arm_config.weighted_perception_mu3;
			else if (line.find("weighted_perception_mu4") != -1)
			    sin >> left_arm_config.weighted_perception_mu4;
			else if (line.find("perception_saturation_on") != -1)
			    sin >> left_arm_config.perception_saturation_on;
			else if (line.find("mu1_saturation_low") != -1)
			    sin >> left_arm_config.mu1_saturation_low;	
			else if (line.find("mu1_saturation_high") != -1)
			    sin >> left_arm_config.mu1_saturation_high;
			else if (line.find("mu2_saturation_low") != -1)
			    sin >> left_arm_config.mu2_saturation_low;	
			else if (line.find("mu2_saturation_high") != -1)
			    sin >> left_arm_config.mu2_saturation_high;
			else if (line.find("mu3_saturation_low") != -1)
			    sin >> left_arm_config.mu3_saturation_low;	
			else if (line.find("mu3_saturation_high") != -1)
			    sin >> left_arm_config.mu3_saturation_high;
			else if (line.find("mu4_saturation_low") != -1)
			    sin >> left_arm_config.mu4_saturation_low;	
			else if (line.find("mu4_saturation_high") != -1)
			    sin >> left_arm_config.mu4_saturation_high;
			//prior
			else if (line.find("on_prior") != -1)
			    sin >> left_arm_config.on_prior;
			else if (line.find("var_d_prior") != -1)
			    sin >> left_arm_config.var_d_prior;
			//first derivative
			else if (line.find("on_derivative") != -1)
			    sin >> left_arm_config.on_derivative;
			else if (line.find("var_dd_attractor") != -1)
			    sin >> left_arm_config.var_dd_attractor;
			else if (line.find("gain_derivative_mu1p") != -1)
			    sin >> left_arm_config.gain_derivative_mu1p;
			else if (line.find("gain_derivative_mu2p") != -1)
			    sin >> left_arm_config.gain_derivative_mu2p;
			else if (line.find("gain_derivative_mu3p") != -1)
			    sin >> left_arm_config.gain_derivative_mu3p;
			else if (line.find("gain_derivative_mu4p") != -1)
			    sin >> left_arm_config.gain_derivative_mu4p;
			else if (line.find("derivative_saturation_on") != -1)
			    sin >> left_arm_config.derivative_saturation_on;	
			else if (line.find("mu1p_saturation_low") != -1)
			    sin >> left_arm_config.mu1p_saturation_low;	
			else if (line.find("mu1p_saturation_high") != -1)
			    sin >> left_arm_config.mu1p_saturation_high;
			else if (line.find("mu2p_saturation_low") != -1)
			    sin >> left_arm_config.mu2p_saturation_low;	
			else if (line.find("mu2p_saturation_high") != -1)
			    sin >> left_arm_config.mu2p_saturation_high;
			else if (line.find("mu3p_saturation_low") != -1)
			    sin >> left_arm_config.mu3p_saturation_low;	
			else if (line.find("mu3p_saturation_high") != -1)
			    sin >> left_arm_config.mu3p_saturation_high;
			else if (line.find("mu4p_saturation_low") != -1)
			    sin >> left_arm_config.mu4p_saturation_low;	
			else if (line.find("mu4p_saturation_high") != -1)
			    sin >> left_arm_config.mu4p_saturation_high;
			//action
			else if (line.find("on_action") != -1)
			    sin >> left_arm_config.on_action;
			else if (line.find("var_a_encoders") != -1)
			    sin >> left_arm_config.var_a_encoders;
			else if (line.find("var_a_3d") != -1)
			    sin >> left_arm_config.var_a_3d;
			else if (line.find("var_a_vision") != -1)
			    sin >> left_arm_config.var_a_vision;
			else if (line.find("gain_action_q1") != -1)
			    sin >> left_arm_config.gain_action_q1;
			else if (line.find("gain_action_q2") != -1)
			    sin >> left_arm_config.gain_action_q2;
			else if (line.find("gain_action_q3") != -1)
			    sin >> left_arm_config.gain_action_q3;
			else if (line.find("gain_action_q4") != -1)
			    sin >> left_arm_config.gain_action_q4;
			else if (line.find("action_saturation_on") != -1)
			    sin >> left_arm_config.action_saturation_on;	
			else if (line.find("action_saturation_low") != -1)
			    sin >> left_arm_config.action_saturation_low;	
			else if (line.find("action_saturation_high") != -1)
			    sin >> left_arm_config.action_saturation_high;
			//attractor
			else if (line.find("on_attractor") != -1)
			    sin >> left_arm_config.on_attractor;
			else if (line.find("attractor_type") != -1)
			    sin >> left_arm_config.attractor_type;		
			else if (line.find("ro1_attractor") != -1)
			    sin >> left_arm_config.ro1_attractor;
			else if (line.find("ro2_attractor") != -1)
			    sin >> left_arm_config.ro2_attractor;
			else if (line.find("ro3_attractor") != -1)
			    sin >> left_arm_config.ro3_attractor;
			else if (line.find("ro4_attractor") != -1)
			    sin >> left_arm_config.ro4_attractor;
			else if (line.find("ro5_attractor") != -1)
			    sin >> left_arm_config.ro5_attractor;
			else if (line.find("ro6_attractor") != -1)
			    sin >> left_arm_config.ro6_attractor;
			else if (line.find("ro7_attractor") != -1)
			    sin >> left_arm_config.ro7_attractor;
			else if (line.find("ro8_attractor") != -1)
			    sin >> left_arm_config.ro8_attractor;
			else if (line.find("ro9_attractor") != -1)
			    sin >> left_arm_config.ro9_attractor;
			else if (line.find("ro10_attractor") != -1)
			    sin >> left_arm_config.ro10_attractor;
			else if (line.find("ro11_attractor") != -1)
			    sin >> left_arm_config.ro11_attractor;
			else if (line.find("ro12_attractor") != -1)
			    sin >> left_arm_config.ro12_attractor;
			else if (line.find("ro13_attractor") != -1)
			    sin >> left_arm_config.ro13_attractor;
			else if (line.find("ro14_attractor") != -1)
			    sin >> left_arm_config.ro14_attractor;
			else if (line.find("ro15_attractor") != -1)
			    sin >> left_arm_config.ro15_attractor;
			else if (line.find("ro16_attractor") != -1)
			    sin >> left_arm_config.ro16_attractor;
			else if (line.find("ro17_attractor") != -1)
			    sin >> left_arm_config.ro17_attractor;
			else if (line.find("ro18_attractor") != -1)
			    sin >> left_arm_config.ro18_attractor;
			else if (line.find("ro19_attractor") != -1)
			    sin >> left_arm_config.ro19_attractor;
			else if (line.find("ro20_attractor") != -1)
			    sin >> left_arm_config.ro20_attractor;
			else if (line.find("ro21_attractor") != -1)
			    sin >> left_arm_config.ro21_attractor;
			else if (line.find("ro22_attractor") != -1)
			    sin >> left_arm_config.ro22_attractor;
			else if (line.find("ro23_attractor") != -1)
			    sin >> left_arm_config.ro23_attractor;
			else if (line.find("ro24_attractor") != -1)
			    sin >> left_arm_config.ro24_attractor;
			else if (line.find("ro25_attractor") != -1)
			    sin >> left_arm_config.ro25_attractor;
            else if (line.find("attractor_threshold") != -1)
                sin >> left_arm_config.attractor_threshold;
            else if (line.find("attractor_comparison") != -1)
                sin >> left_arm_config.attractor_comparison;
			else if (line.find("var_d_attractor") != -1)
			    sin >> left_arm_config.var_d_attractor;
			else if (line.find("left_attractor_hue_low") != -1)
			    sin >> left_arm_config.left_attractor_hue_low;
			else if (line.find("left_attractor_sat_low") != -1)
			    sin >> left_arm_config.left_attractor_sat_low;
			else if (line.find("left_attractor_val_low") != -1)
			    sin >> left_arm_config.left_attractor_val_low;
			else if (line.find("left_attractor_hue_high") != -1)
			    sin >> left_arm_config.left_attractor_hue_high;
			else if (line.find("left_attractor_sat_high") != -1)
			    sin >> left_arm_config.left_attractor_sat_high;
			else if (line.find("left_attractor_val_high") != -1)
			    sin >> left_arm_config.left_attractor_val_high;
			else if (line.find("right_attractor_hue_low") != -1)
			    sin >> left_arm_config.right_attractor_hue_low;
			else if (line.find("right_attractor_sat_low") != -1)
			    sin >> left_arm_config.right_attractor_sat_low;
			else if (line.find("right_attractor_val_low") != -1)
			    sin >> left_arm_config.right_attractor_val_low;
			else if (line.find("right_attractor_hue_high") != -1)
			    sin >> left_arm_config.right_attractor_hue_high;
			else if (line.find("right_attractor_sat_high") != -1)
			    sin >> left_arm_config.right_attractor_sat_high;
			else if (line.find("right_attractor_val_high") != -1)
			    sin >> left_arm_config.right_attractor_val_high;
            else if (line.find("fixed_attractor") != -1)
			    sin >> left_arm_config.fixed_attractor;
			//touch
			else if (line.find("on_touch") != -1)
			    sin >> left_arm_config.on_touch;
            else if (line.find("touch_threshold") != -1)
                sin >> left_arm_config.touch_threshold;
			else if (line.find("grasping") != -1)
			    sin >> left_arm_config.grasping;
			//noise		
			else if (line.find("gaussian_noise") != -1)
			    sin >> left_arm_config.gaussian_noise;
			else if (line.find("gaussian_mean") != -1)
			    sin >> left_arm_config.gaussian_mean;
			else if (line.find("gaussian_std") != -1)
			    sin >> left_arm_config.gaussian_std;
			//other
			else if (line.find("null_start") != -1)
			    sin >> left_arm_config.null_start;	
			
		}
		
		
	}
}

void print_config()
{
    cout << "Main configuration read: " << endl;
	cout << "  robot = " << main_config.robot << endl;
	cout << "  debug = " << main_config.debug << endl;
	cout << "  print = " << main_config.print << endl;
	cout << "  dT = " << main_config.dT << endl;
	cout << "  integration_method = " << main_config.integration_method << endl;
	cout << "  log = " << main_config.log << endl;
    cout << "  description = " << main_config.description << endl;
	cout << "  publish = " << main_config.publish << endl;
	cout << "  cycles = " << main_config.cycles << endl;

	//right arm
    cout << endl << "Right arm configuration read: " << endl;
	cout << "  right_arm_activated = " << right_arm_config.activated << endl;
	//initial position
    cout << "  torso_yaw = " << right_arm_config.torso_yaw << endl;
	cout << "  shoulder_roll = " << right_arm_config.shoulder_roll << endl;
	cout << "  shoulder_yaw = " << right_arm_config.shoulder_yaw << endl;
	cout << "  elbow = " << right_arm_config.elbow << endl;
	//internal states
    cout << "  on_encoders = " << right_arm_config.on_encoders << endl;
	cout << "  var_d_encoders = " << right_arm_config.var_d_encoders << endl;
	cout << "  on_3d = " << right_arm_config.on_3d << endl;
	cout << "  var_d_3d = " << right_arm_config.var_d_3d << endl;
	cout << "  on_vision = " << right_arm_config.on_vision << endl;
	cout << "  var_d_vision = " << right_arm_config.var_d_vision << endl;
	cout << "  left_vision_hue_low = " << right_arm_config.left_vision_hue_low << endl;
	cout << "  left_vision_sat_low = " << right_arm_config.left_vision_sat_low << endl;
	cout << "  left_vision_val_low = " << right_arm_config.left_vision_val_low << endl;
	cout << "  left_vision_hue_high = " << right_arm_config.left_vision_hue_high << endl;
	cout << "  left_vision_sat_high = " << right_arm_config.left_vision_sat_high << endl;
	cout << "  left_vision_val_high = " << right_arm_config.left_vision_val_high << endl;
	cout << "  right_vision_hue_low = " << right_arm_config.right_vision_hue_low << endl;
	cout << "  right_vision_sat_low = " << right_arm_config.right_vision_sat_low << endl;
	cout << "  right_vision_val_low = " << right_arm_config.right_vision_val_low << endl;
	cout << "  right_vision_hue_high = " << right_arm_config.right_vision_hue_high << endl;
	cout << "  right_vision_sat_high = " << right_arm_config.right_vision_sat_high << endl;
	cout << "  right_vision_val_high = " << right_arm_config.right_vision_val_high << endl;
	cout << "  left_erode_dilate_iterations = " << right_arm_config.left_erode_dilate_iterations << endl;
	cout << "  right_erode_dilate_iterations = " << right_arm_config.right_erode_dilate_iterations << endl;
	cout << "  gain_perception_mu1 = " << right_arm_config.gain_perception_mu1 << endl;
	cout << "  gain_perception_mu2 = " << right_arm_config.gain_perception_mu2 << endl;
	cout << "  gain_perception_mu3 = " << right_arm_config.gain_perception_mu3 << endl;
	cout << "  gain_perception_mu4 = " << right_arm_config.gain_perception_mu4 << endl;
	cout << "  weighted_perception_mu1 = " << right_arm_config.weighted_perception_mu1 << endl;
	cout << "  weighted_perception_mu2 = " << right_arm_config.weighted_perception_mu2 << endl;
	cout << "  weighted_perception_mu3 = " << right_arm_config.weighted_perception_mu3 << endl;
	cout << "  weighted_perception_mu4 = " << right_arm_config.weighted_perception_mu4 << endl;
	cout << "  perception_saturation_on = " << right_arm_config.perception_saturation_on << endl;
	cout << "  mu1_saturation_low = " << right_arm_config.mu1_saturation_low << endl;
	cout << "  mu1_saturation_high = " << right_arm_config.mu1_saturation_high << endl;
	cout << "  mu2_saturation_low = " << right_arm_config.mu2_saturation_low << endl;
	cout << "  mu2_saturation_high = " << right_arm_config.mu2_saturation_high << endl;
	cout << "  mu3_saturation_low = " << right_arm_config.mu3_saturation_low << endl;
	cout << "  mu3_saturation_high = " << right_arm_config.mu3_saturation_high << endl;
	cout << "  mu4_saturation_low = " << right_arm_config.mu4_saturation_low << endl;
	cout << "  mu4_saturation_high = " << right_arm_config.mu4_saturation_high << endl;
	//first derivative
	cout << "  on_derivative = " << right_arm_config.on_derivative << endl;
	cout << "  var_dd_attractor = " << right_arm_config.var_dd_attractor << endl;
	cout << "  gain_derivative_mu1p = " << right_arm_config.gain_derivative_mu1p << endl;
	cout << "  gain_derivative_mu2p = " << right_arm_config.gain_derivative_mu2p << endl;
	cout << "  gain_derivative_mu3p = " << right_arm_config.gain_derivative_mu3p << endl;
	cout << "  derivative_saturation_on = " << right_arm_config.derivative_saturation_on << endl;
	cout << "  mu1p_saturation_low = " << right_arm_config.mu1p_saturation_low << endl;
	cout << "  mu1p_saturation_high = " << right_arm_config.mu1p_saturation_high << endl;
	cout << "  mu2p_saturation_low = " << right_arm_config.mu2p_saturation_low << endl;
	cout << "  mu2p_saturation_high = " << right_arm_config.mu2p_saturation_high << endl;
	cout << "  mu3p_saturation_low = " << right_arm_config.mu3p_saturation_low << endl;
	cout << "  mu3p_saturation_high = " << right_arm_config.mu3p_saturation_high << endl;
	cout << "  mu4p_saturation_low = " << right_arm_config.mu4p_saturation_low << endl;
	cout << "  mu4p_saturation_high = " << right_arm_config.mu4p_saturation_high << endl;
	//action
	cout << "  on_action = " << right_arm_config.on_action << endl;
	cout << "  var_a_encoders = " << right_arm_config.var_a_encoders << endl;
	cout << "  var_a_3d = " << right_arm_config.var_a_3d << endl;
	cout << "  var_a_vision = " << right_arm_config.var_a_vision << endl;
	cout << "  gain_action_q1 = " << right_arm_config.gain_action_q1 << endl;
	cout << "  gain_action_q2 = " << right_arm_config.gain_action_q2 << endl;
	cout << "  gain_action_q3 = " << right_arm_config.gain_action_q3 << endl;
	cout << "  gain_action_q4 = " << right_arm_config.gain_action_q4 << endl;
	cout << "  action_saturation_on = " << right_arm_config.action_saturation_on << endl;
	cout << "  action_saturation_low = " << right_arm_config.action_saturation_low << endl;
	cout << "  action_saturation_high = " << right_arm_config.action_saturation_high << endl;
	//prior
	cout << "  on_prior = " << right_arm_config.on_prior << endl;
	cout << "  var_d_prior = " << right_arm_config.var_d_prior << endl;
	//attractor
	cout << "  on_attractor = " << right_arm_config.on_attractor << endl;
	cout << "  attractor_type = " << right_arm_config.attractor_type << endl;
	cout << "  ro1_attractor = " << right_arm_config.ro1_attractor << endl;
	cout << "  ro2_attractor = " << right_arm_config.ro2_attractor << endl;
	cout << "  ro3_attractor = " << right_arm_config.ro3_attractor << endl;
	cout << "  ro4_attractor = " << right_arm_config.ro4_attractor << endl;
    cout << "  ro5_attractor = " << right_arm_config.ro5_attractor << endl;
    cout << "  ro6_attractor = " << right_arm_config.ro6_attractor << endl;
    cout << "  ro7_attractor = " << right_arm_config.ro7_attractor << endl;
    cout << "  ro8_attractor = " << right_arm_config.ro8_attractor << endl;
    cout << "  ro9_attractor = " << right_arm_config.ro9_attractor << endl;
    cout << "  ro10_attractor = " << right_arm_config.ro10_attractor << endl;
    cout << "  ro11_attractor = " << right_arm_config.ro11_attractor << endl;
    cout << "  ro12_attractor = " << right_arm_config.ro12_attractor << endl;
    cout << "  ro13_attractor = " << right_arm_config.ro13_attractor << endl;
    cout << "  ro14_attractor = " << right_arm_config.ro14_attractor << endl;
    cout << "  ro15_attractor = " << right_arm_config.ro15_attractor << endl;
    cout << "  ro16_attractor = " << right_arm_config.ro16_attractor << endl;
    cout << "  ro17_attractor = " << right_arm_config.ro17_attractor << endl;
    cout << "  ro18_attractor = " << right_arm_config.ro18_attractor << endl;
    cout << "  ro19_attractor = " << right_arm_config.ro19_attractor << endl;
    cout << "  ro20_attractor = " << right_arm_config.ro20_attractor << endl;
    cout << "  ro21_attractor = " << right_arm_config.ro21_attractor << endl;
    cout << "  ro22_attractor = " << right_arm_config.ro22_attractor << endl;
    cout << "  ro23_attractor = " << right_arm_config.ro23_attractor << endl;
    cout << "  ro24_attractor = " << right_arm_config.ro24_attractor << endl;
    cout << "  ro25_attractor = " << right_arm_config.ro25_attractor << endl;
	cout << "  attractor_threshold = " << right_arm_config.attractor_threshold << endl;
    cout << "  attractor_comparison = " << right_arm_config.attractor_comparison << endl;
	cout << "  var_d_attractor = " << right_arm_config.var_d_attractor << endl;
	cout << "  left_attractor_hue_low = " << right_arm_config.left_attractor_hue_low << endl;
	cout << "  left_attractor_sat_low = " << right_arm_config.left_attractor_sat_low << endl;
	cout << "  left_attractor_val_low = " << right_arm_config.left_attractor_val_low << endl;
	cout << "  left_attractor_hue_high = " << right_arm_config.left_attractor_hue_high << endl;
	cout << "  left_attractor_sat_high = " << right_arm_config.left_attractor_sat_high << endl;
	cout << "  left_attractor_val_high = " << right_arm_config.left_attractor_val_high << endl;
	cout << "  right_attractor_hue_low = " << right_arm_config.right_attractor_hue_low << endl;
	cout << "  right_attractor_sat_low = " << right_arm_config.right_attractor_sat_low << endl;
	cout << "  right_attractor_val_low = " << right_arm_config.right_attractor_val_low << endl;
	cout << "  right_attractor_hue_high = " << right_arm_config.right_attractor_hue_high << endl;
	cout << "  right_attractor_sat_high = " << right_arm_config.right_attractor_sat_high << endl;
	cout << "  right_attractor_val_high = " << right_arm_config.right_attractor_val_high << endl;
	cout << "  fixed_attractor = " << right_arm_config.fixed_attractor << endl;
	//touch
	cout << "  on_touch = " << right_arm_config.on_touch << endl;
    cout << "  touch_threshold = " << right_arm_config.touch_threshold << endl;
	cout << "  grasping = " << right_arm_config.grasping << endl;
	//noise
	cout << "  gaussian_noise = " << right_arm_config.gaussian_noise << endl;
	cout << "  gaussian_mean = " << right_arm_config.gaussian_mean << endl;
	cout << "  gaussian_std = " << right_arm_config.gaussian_std << endl;
	//other
	cout << "  null_start = " << right_arm_config.null_start << endl;

	//head
    cout << endl << "Head arm configuration read: " << endl;
	cout << "  head_activated = " << head_config.activated << endl;
	//initial position
	cout << "  neck_pitch = " << head_config.neck_pitch << endl;
	cout << "  neck_yaw = " << head_config.neck_yaw << endl;
    cout << "  eyes_tilt = " << head_config.eyes_tilt << endl;
	//internal states
	cout << "  var_d_encoders = " << head_config.var_d_encoders << endl;
	cout << "  gain_perception_mu1 = " << head_config.gain_perception_mu1 << endl;
	cout << "  gain_perception_mu2 = " << head_config.gain_perception_mu2 << endl;
    cout << "  gain_perception_mu3 = " << head_config.gain_perception_mu3 << endl;
	cout << "  weighted_perception_mu1 = " << head_config.weighted_perception_mu1 << endl;
	cout << "  weighted_perception_mu2 = " << head_config.weighted_perception_mu2 << endl;
    cout << "  weighted_perception_mu3 = " << head_config.weighted_perception_mu3 << endl;
	cout << "  perception_saturation_on = " << head_config.perception_saturation_on << endl;	
	cout << "  mu1_saturation_low = " << head_config.mu1_saturation_low << endl;
	cout << "  mu1_saturation_high = " << head_config.mu1_saturation_high << endl;
	cout << "  mu2_saturation_low = " << head_config.mu2_saturation_low << endl;
	cout << "  mu2_saturation_high = " << head_config.mu2_saturation_high << endl;
    cout << "  mu3_saturation_low = " << head_config.mu3_saturation_low << endl;
    cout << "  mu3_saturation_high = " << head_config.mu3_saturation_high << endl;
	//first derivative
	cout << "  on_derivative = " << head_config.on_derivative << endl;
	cout << "  var_dd_attractor = " << head_config.var_dd_attractor << endl;
	cout << "  gain_derivative_mu1p = " << head_config.gain_derivative_mu1p << endl;
	cout << "  gain_derivative_mu2p = " << head_config.gain_derivative_mu2p << endl;
	cout << "  derivative_saturation_on = " << head_config.derivative_saturation_on << endl;
	cout << "  mu1p_saturation_low = " << head_config.mu1p_saturation_low << endl;
	cout << "  mu1p_saturation_high = " << head_config.mu1p_saturation_high << endl;
	cout << "  mu2p_saturation_low = " << head_config.mu2p_saturation_low << endl;
	cout << "  mu2p_saturation_high = " << head_config.mu2p_saturation_high << endl;
    cout << "  mu3p_saturation_low = " << head_config.mu3p_saturation_low << endl;
    cout << "  mu3p_saturation_high = " << head_config.mu3p_saturation_high << endl;
	//action
	cout << "  on_action = " << head_config.on_action << endl;
	cout << "  var_a_encoders = " << head_config.var_a_encoders << endl;
	cout << "  gain_action_q1 = " << head_config.gain_action_q1 << endl;
	cout << "  gain_action_q2 = " << head_config.gain_action_q2 << endl;
    cout << "  gain_action_q3 = " << head_config.gain_action_q3 << endl;
	cout << "  action_saturation_on = " << head_config.action_saturation_on << endl;
	cout << "  action_saturation_low = " << head_config.action_saturation_low << endl;
	cout << "  action_saturation_high = " << head_config.action_saturation_high << endl;
	//attractor
	cout << "  ro1_attractor = " << head_config.ro1_attractor << endl;
	cout << "  var_d_attractor = " << head_config.var_d_attractor << endl;
	//other
	cout << "  null_start = " << head_config.null_start << endl;
	
	//left arm
    cout << endl << "Left arm configuration read: " << endl;
	cout << "  left_arm_activated = " << left_arm_config.activated << endl;
	//initial position
	cout << "  shoulder_roll = " << left_arm_config.shoulder_roll << endl;
	cout << "  shoulder_yaw = " << left_arm_config.shoulder_yaw << endl;
	cout << "  elbow = " << left_arm_config.elbow << endl;
	//internal states
    cout << "  on_encoders = " << left_arm_config.on_encoders << endl;
	cout << "  var_d_encoders = " << left_arm_config.var_d_encoders << endl;
	cout << "  on_3d = " << left_arm_config.on_3d << endl;
	cout << "  var_d_3d = " << left_arm_config.var_d_3d << endl;
	cout << "  on_vision = " << left_arm_config.on_vision << endl;
	cout << "  var_d_vision = " << left_arm_config.var_d_vision << endl;
	cout << "  left_vision_hue_low = " << left_arm_config.left_vision_hue_low << endl;
	cout << "  left_vision_sat_low = " << left_arm_config.left_vision_sat_low << endl;
	cout << "  left_vision_val_low = " << left_arm_config.left_vision_val_low << endl;
	cout << "  left_vision_hue_high = " << left_arm_config.left_vision_hue_high << endl;
	cout << "  left_vision_sat_high = " << left_arm_config.left_vision_sat_high << endl;
	cout << "  left_vision_val_high = " << left_arm_config.left_vision_val_high << endl;
	cout << "  right_vision_hue_low = " << left_arm_config.right_vision_hue_low << endl;
	cout << "  right_vision_sat_low = " << left_arm_config.right_vision_sat_low << endl;
	cout << "  right_vision_val_low = " << left_arm_config.right_vision_val_low << endl;
	cout << "  right_vision_hue_high = " << left_arm_config.right_vision_hue_high << endl;
	cout << "  right_vision_sat_high = " << left_arm_config.right_vision_sat_high << endl;
	cout << "  right_vision_val_high = " << left_arm_config.right_vision_val_high << endl;
	cout << "  left_erode_dilate_iterations = " << left_arm_config.left_erode_dilate_iterations << endl;
	cout << "  right_erode_dilate_iterations = " << left_arm_config.right_erode_dilate_iterations << endl;
	cout << "  gain_perception_mu1 = " << left_arm_config.gain_perception_mu1 << endl;
	cout << "  gain_perception_mu2 = " << left_arm_config.gain_perception_mu2 << endl;
	cout << "  gain_perception_mu3 = " << left_arm_config.gain_perception_mu3 << endl;
	cout << "  gain_perception_mu4 = " << left_arm_config.gain_perception_mu4 << endl;
	cout << "  weighted_perception_mu1 = " << left_arm_config.weighted_perception_mu1 << endl;
	cout << "  weighted_perception_mu2 = " << left_arm_config.weighted_perception_mu2 << endl;
	cout << "  weighted_perception_mu3 = " << left_arm_config.weighted_perception_mu3 << endl;
	cout << "  weighted_perception_mu4 = " << left_arm_config.weighted_perception_mu4 << endl;
	cout << "  perception_saturation_on = " << left_arm_config.perception_saturation_on << endl;
	cout << "  mu1_saturation_low = " << left_arm_config.mu1_saturation_low << endl;
	cout << "  mu1_saturation_high = " << left_arm_config.mu1_saturation_high << endl;
	cout << "  mu2_saturation_low = " << left_arm_config.mu2_saturation_low << endl;
	cout << "  mu2_saturation_high = " << left_arm_config.mu2_saturation_high << endl;
	cout << "  mu3_saturation_low = " << left_arm_config.mu3_saturation_low << endl;
	cout << "  mu3_saturation_high = " << left_arm_config.mu3_saturation_high << endl;
	cout << "  mu4_saturation_low = " << left_arm_config.mu4_saturation_low << endl;
	cout << "  mu4_saturation_high = " << left_arm_config.mu4_saturation_high << endl;
	//first derivative
	cout << "  on_derivative = " << left_arm_config.on_derivative << endl;
	cout << "  var_dd_attractor = " << left_arm_config.var_dd_attractor << endl;
	cout << "  gain_derivative_mu1p = " << left_arm_config.gain_derivative_mu1p << endl;
	cout << "  gain_derivative_mu2p = " << left_arm_config.gain_derivative_mu2p << endl;
	cout << "  gain_derivative_mu3p = " << left_arm_config.gain_derivative_mu3p << endl;
	cout << "  derivative_saturation_on = " << left_arm_config.derivative_saturation_on << endl;
	cout << "  mu1p_saturation_low = " << left_arm_config.mu1p_saturation_low << endl;
	cout << "  mu1p_saturation_high = " << left_arm_config.mu1p_saturation_high << endl;
	cout << "  mu2p_saturation_low = " << left_arm_config.mu2p_saturation_low << endl;
	cout << "  mu2p_saturation_high = " << left_arm_config.mu2p_saturation_high << endl;
	cout << "  mu3p_saturation_low = " << left_arm_config.mu3p_saturation_low << endl;
	cout << "  mu3p_saturation_high = " << left_arm_config.mu3p_saturation_high << endl;
	cout << "  mu4p_saturation_low = " << left_arm_config.mu4p_saturation_low << endl;
	cout << "  mu4p_saturation_high = " << left_arm_config.mu4p_saturation_high << endl;
	//action
	cout << "  on_action = " << left_arm_config.on_action << endl;
	cout << "  var_a_encoders = " << left_arm_config.var_a_encoders << endl;
	cout << "  var_a_3d = " << left_arm_config.var_a_3d << endl;
	cout << "  var_a_vision = " << left_arm_config.var_a_vision << endl;
	cout << "  gain_action_q1 = " << left_arm_config.gain_action_q1 << endl;
	cout << "  gain_action_q2 = " << left_arm_config.gain_action_q2 << endl;
	cout << "  gain_action_q3 = " << left_arm_config.gain_action_q3 << endl;
	cout << "  gain_action_q4 = " << left_arm_config.gain_action_q4 << endl;
	cout << "  action_saturation_on = " << left_arm_config.action_saturation_on << endl;
	cout << "  action_saturation_low = " << left_arm_config.action_saturation_low << endl;
	cout << "  action_saturation_high = " << left_arm_config.action_saturation_high << endl;
	//prior
	cout << "  on_prior = " << left_arm_config.on_prior << endl;
	cout << "  var_d_prior = " << left_arm_config.var_d_prior << endl;
	//attractor
	cout << "  on_attractor = " << left_arm_config.on_attractor << endl;
	cout << "  attractor_type = " << left_arm_config.attractor_type << endl;
	cout << "  ro1_attractor = " << left_arm_config.ro1_attractor << endl;
	cout << "  ro2_attractor = " << left_arm_config.ro2_attractor << endl;
	cout << "  ro3_attractor = " << left_arm_config.ro3_attractor << endl;
	cout << "  ro4_attractor = " << left_arm_config.ro4_attractor << endl;
	cout << "  ro5_attractor = " << left_arm_config.ro5_attractor << endl;
	cout << "  ro6_attractor = " << left_arm_config.ro6_attractor << endl;
	cout << "  ro7_attractor = " << left_arm_config.ro7_attractor << endl;
	cout << "  ro8_attractor = " << left_arm_config.ro8_attractor << endl;
	cout << "  ro9_attractor = " << left_arm_config.ro9_attractor << endl;
	cout << "  ro10_attractor = " << left_arm_config.ro10_attractor << endl;
	cout << "  ro11_attractor = " << left_arm_config.ro11_attractor << endl;
	cout << "  ro12_attractor = " << left_arm_config.ro12_attractor << endl;
	cout << "  ro13_attractor = " << left_arm_config.ro13_attractor << endl;
	cout << "  ro14_attractor = " << left_arm_config.ro14_attractor << endl;
	cout << "  ro15_attractor = " << left_arm_config.ro15_attractor << endl;
	cout << "  ro16_attractor = " << left_arm_config.ro16_attractor << endl;
	cout << "  ro17_attractor = " << left_arm_config.ro17_attractor << endl;
	cout << "  ro18_attractor = " << left_arm_config.ro18_attractor << endl;
	cout << "  ro19_attractor = " << left_arm_config.ro19_attractor << endl;
	cout << "  ro20_attractor = " << left_arm_config.ro20_attractor << endl;
	cout << "  ro21_attractor = " << left_arm_config.ro21_attractor << endl;
	cout << "  ro22_attractor = " << left_arm_config.ro22_attractor << endl;
	cout << "  ro23_attractor = " << left_arm_config.ro23_attractor << endl;
	cout << "  ro24_attractor = " << left_arm_config.ro24_attractor << endl;
	cout << "  ro25_attractor = " << left_arm_config.ro25_attractor << endl;
    cout << "  attractor_threshold = " << left_arm_config.attractor_threshold << endl;
    cout << "  attractor_comparison = " << left_arm_config.attractor_comparison << endl;
	cout << "  var_d_attractor = " << left_arm_config.var_d_attractor << endl;
	cout << "  left_attractor_hue_low = " << left_arm_config.left_attractor_hue_low << endl;
	cout << "  left_attractor_sat_low = " << left_arm_config.left_attractor_sat_low << endl;
	cout << "  left_attractor_val_low = " << left_arm_config.left_attractor_val_low << endl;
	cout << "  left_attractor_hue_high = " << left_arm_config.left_attractor_hue_high << endl;
	cout << "  left_attractor_sat_high = " << left_arm_config.left_attractor_sat_high << endl;
	cout << "  left_attractor_val_high = " << left_arm_config.left_attractor_val_high << endl;
	cout << "  right_attractor_hue_low = " << left_arm_config.right_attractor_hue_low << endl;
	cout << "  right_attractor_sat_low = " << left_arm_config.right_attractor_sat_low << endl;
	cout << "  right_attractor_val_low = " << left_arm_config.right_attractor_val_low << endl;
	cout << "  right_attractor_hue_high = " << left_arm_config.right_attractor_hue_high << endl;
	cout << "  right_attractor_sat_high = " << left_arm_config.right_attractor_sat_high << endl;
	cout << "  right_attractor_val_high = " << left_arm_config.right_attractor_val_high << endl;
	cout << "  fixed_attractor = " << left_arm_config.fixed_attractor << endl;
	//touch
	cout << "  on_touch = " << left_arm_config.on_touch << endl;
    cout << "  touch_threshold = " << left_arm_config.touch_threshold << endl;
	cout << "  grasping = " << left_arm_config.grasping << endl;
	//noise
	cout << "  gaussian_noise = " << left_arm_config.gaussian_noise << endl;
	cout << "  gaussian_mean = " << left_arm_config.gaussian_mean << endl;
	cout << "  gaussian_std = " << left_arm_config.gaussian_std << endl;
	//other
    cout << "  null_start = " << left_arm_config.null_start << endl;
	
}

void log_config(fstream& logFile, string description)
{
	//check if opened
	if (!logFile.is_open()) return;
	
	//log main configuration
	logFile << "[CONFIG]";
	logFile << "  robot = " << main_config.robot << ",";
	logFile << "  debug = " << main_config.debug << ",";
	logFile << "  print = " << main_config.print << ",";
	logFile << "  dT = " << main_config.dT << ",";
	logFile << "  integration_method = " << main_config.integration_method << ",";
	logFile << "  log = " << main_config.log << ",";
    logFile << "  description = " << main_config.description << ",";
	logFile << "  publish = " << main_config.publish << ",";
	logFile << "  cycles = " << main_config.cycles << ",";

	//log right arm configuration
	if (description == "right_arm")
	{
		logFile << "  right_arm_activated = " << right_arm_config.activated << ",";
		//initial position
        logFile << "  torso_yaw = " << right_arm_config.torso_yaw << ",";
		logFile << "  shoulder_roll = " << right_arm_config.shoulder_roll << ",";
		logFile << "  shoulder_yaw = " << right_arm_config.shoulder_yaw << ",";
		logFile << "  elbow = " << right_arm_config.elbow << ",";
		//internal state
        logFile << "  on_encoders = " << right_arm_config.on_encoders << ",";
		logFile << "  var_d_encoders = " << right_arm_config.var_d_encoders << ",";
		logFile << "  on_3d = " << right_arm_config.on_3d << ",";
		logFile << "  var_d_3d = " << right_arm_config.var_d_3d << ",";
		logFile << "  on_vision = " << right_arm_config.on_vision << ",";
		logFile << "  var_d_vision = " << right_arm_config.var_d_vision << ",";
		logFile << "  left_vision_hue_low = " << right_arm_config.left_vision_hue_low << ",";
		logFile << "  left_vision_sat_low = " << right_arm_config.left_vision_sat_low << ",";
		logFile << "  left_vision_val_low = " << right_arm_config.left_vision_val_low << ",";
		logFile << "  left_vision_hue_high = " << right_arm_config.left_vision_hue_high << ",";
		logFile << "  left_vision_sat_high = " << right_arm_config.left_vision_sat_high << ",";
		logFile << "  left_vision_val_high = " << right_arm_config.left_vision_val_high << ",";
		logFile << "  right_vision_hue_low = " << right_arm_config.right_vision_hue_low << ",";
		logFile << "  right_vision_sat_low = " << right_arm_config.right_vision_sat_low << ",";
		logFile << "  right_vision_val_low = " << right_arm_config.right_vision_val_low << ",";
		logFile << "  right_vision_hue_high = " << right_arm_config.right_vision_hue_high << ",";
		logFile << "  right_vision_sat_high = " << right_arm_config.right_vision_sat_high << ",";
		logFile << "  right_vision_val_high = " << right_arm_config.right_vision_val_high << ",";
		logFile << "  left_erode_dilate_iterations = " << right_arm_config.left_erode_dilate_iterations << ",";
		logFile << "  right_erode_dilate_iterations = " << right_arm_config.right_erode_dilate_iterations << ",";
		logFile << "  gain_perception_mu1 = " << right_arm_config.gain_perception_mu1 << ",";
		logFile << "  gain_perception_mu2 = " << right_arm_config.gain_perception_mu2 << ",";
		logFile << "  gain_perception_mu3 = " << right_arm_config.gain_perception_mu3 << ",";
		logFile << "  gain_perception_mu4 = " << right_arm_config.gain_perception_mu4 << ",";
		logFile << "  weighted_perception_mu1 = " << right_arm_config.weighted_perception_mu1 << ",";
		logFile << "  weighted_perception_mu2 = " << right_arm_config.weighted_perception_mu2 << ",";
		logFile << "  weighted_perception_mu3 = " << right_arm_config.weighted_perception_mu3 << ",";
		logFile << "  weighted_perception_mu4 = " << right_arm_config.weighted_perception_mu4 << ",";
		logFile << "  perception_saturation_on = " << right_arm_config.perception_saturation_on << ",";
		logFile << "  mu1_saturation_low = " << right_arm_config.mu1_saturation_low << ",";
		logFile << "  mu1_saturation_high = " << right_arm_config.mu1_saturation_high << ",";
		logFile << "  mu2_saturation_low = " << right_arm_config.mu2_saturation_low << ",";
		logFile << "  mu2_saturation_high = " << right_arm_config.mu2_saturation_high << ",";
		logFile << "  mu3_saturation_low = " << right_arm_config.mu3_saturation_low << ",";
		logFile << "  mu3_saturation_high = " << right_arm_config.mu3_saturation_high << ",";
		logFile << "  mu4_saturation_low = " << right_arm_config.mu4_saturation_low << ",";
		logFile << "  mu4_saturation_high = " << right_arm_config.mu4_saturation_high << ",";
		//first derivative
		logFile << "  on_derivative = " << right_arm_config.on_derivative << ",";
		logFile << "  var_dd_attractor = " << right_arm_config.var_dd_attractor << ",";
		logFile << "  gain_derivative_mu1p = " << right_arm_config.gain_derivative_mu1p << ",";
		logFile << "  gain_derivative_mu2p = " << right_arm_config.gain_derivative_mu2p << ",";
		logFile << "  gain_derivative_mu3p = " << right_arm_config.gain_derivative_mu3p << ",";
		logFile << "  gain_derivative_mu4p = " << right_arm_config.gain_derivative_mu4p << ",";
		logFile << "  derivative_saturation_on = " << right_arm_config.derivative_saturation_on << ",";
		logFile << "  mu1p_saturation_low = " << right_arm_config.mu1p_saturation_low << ",";
		logFile << "  mu1p_saturation_high = " << right_arm_config.mu1p_saturation_high << ",";
		logFile << "  mu2p_saturation_low = " << right_arm_config.mu2p_saturation_low << ",";
		logFile << "  mu2p_saturation_high = " << right_arm_config.mu2p_saturation_high << ",";
		logFile << "  mu3p_saturation_low = " << right_arm_config.mu3p_saturation_low << ",";
		logFile << "  mu3p_saturation_high = " << right_arm_config.mu3p_saturation_high << ",";
		logFile << "  mu4p_saturation_low = " << right_arm_config.mu4p_saturation_low << ",";
		logFile << "  mu4p_saturation_high = " << right_arm_config.mu4p_saturation_high << ",";
		//action
		logFile << "  on_action = " << right_arm_config.on_action << ",";
		logFile << "  var_a_encoders = " << right_arm_config.var_a_encoders << ",";
		logFile << "  var_a_3d = " << right_arm_config.var_a_3d << ",";
		logFile << "  var_a_vision = " << right_arm_config.var_a_vision << ",";
		logFile << "  gain_action_q1 = " << right_arm_config.gain_action_q1 << ",";
		logFile << "  gain_action_q2 = " << right_arm_config.gain_action_q2 << ",";
		logFile << "  gain_action_q3 = " << right_arm_config.gain_action_q3 << ",";
		logFile << "  gain_action_q4 = " << right_arm_config.gain_action_q4 << ",";
		logFile << "  action_saturation_on = " << right_arm_config.action_saturation_on << ",";
		logFile << "  action_saturation_low = " << right_arm_config.action_saturation_low << ",";
		logFile << "  action_saturation_high = " << right_arm_config.action_saturation_high << ",";
		//attractor
        logFile << "  on_attractor = " << right_arm_config.on_attractor << ",";
		logFile << "  attractor_type = " << right_arm_config.attractor_type << ",";
		logFile << "  ro1_attractor = " << right_arm_config.ro1_attractor << ",";
		logFile << "  ro2_attractor = " << right_arm_config.ro2_attractor << ",";
		logFile << "  ro3_attractor = " << right_arm_config.ro3_attractor << ",";
		logFile << "  ro4_attractor = " << right_arm_config.ro4_attractor << ",";
        logFile << "  ro5_attractor = " << right_arm_config.ro5_attractor << ",";
        logFile << "  ro6_attractor = " << right_arm_config.ro6_attractor << ",";
        logFile << "  ro7_attractor = " << right_arm_config.ro7_attractor << ",";
        logFile << "  ro8_attractor = " << right_arm_config.ro8_attractor << ",";
        logFile << "  ro9_attractor = " << right_arm_config.ro9_attractor << ",";
        logFile << "  ro10_attractor = " << right_arm_config.ro10_attractor << ",";
        logFile << "  ro11_attractor = " << right_arm_config.ro11_attractor << ",";
        logFile << "  ro12_attractor = " << right_arm_config.ro12_attractor << ",";
        logFile << "  ro13_attractor = " << right_arm_config.ro13_attractor << ",";
        logFile << "  ro14_attractor = " << right_arm_config.ro14_attractor << ",";
        logFile << "  ro15_attractor = " << right_arm_config.ro15_attractor << ",";
        logFile << "  ro16_attractor = " << right_arm_config.ro16_attractor << ",";
        logFile << "  ro17_attractor = " << right_arm_config.ro17_attractor << ",";
        logFile << "  ro18_attractor = " << right_arm_config.ro18_attractor << ",";
        logFile << "  ro19_attractor = " << right_arm_config.ro19_attractor << ",";
        logFile << "  ro20_attractor = " << right_arm_config.ro20_attractor << ",";
        logFile << "  ro21_attractor = " << right_arm_config.ro21_attractor << ",";
        logFile << "  ro22_attractor = " << right_arm_config.ro22_attractor << ",";
        logFile << "  ro23_attractor = " << right_arm_config.ro23_attractor << ",";
        logFile << "  ro24_attractor = " << right_arm_config.ro24_attractor << ",";
        logFile << "  ro25_attractor = " << right_arm_config.ro25_attractor << ",";
        logFile << "  attractor_threshold = " << right_arm_config.attractor_threshold << ",";
        logFile << "  attractor_comparison = " << right_arm_config.attractor_comparison << ",";
		logFile << "  var_d_attractor = " << right_arm_config.var_d_attractor << ",";
		logFile << "  left_attractor_hue_low = " << right_arm_config.left_attractor_hue_low << ",";
		logFile << "  left_attractor_sat_low = " << right_arm_config.left_attractor_sat_low << ",";
		logFile << "  left_attractor_val_low = " << right_arm_config.left_attractor_val_low << ",";
		logFile << "  left_attractor_hue_high = " << right_arm_config.left_attractor_hue_high << ",";
		logFile << "  left_attractor_sat_high = " << right_arm_config.left_attractor_sat_high << ",";
		logFile << "  left_attractor_val_high = " << right_arm_config.left_attractor_val_high << ",";
		logFile << "  right_attractor_hue_low = " << right_arm_config.right_attractor_hue_low << ",";
		logFile << "  right_attractor_sat_low = " << right_arm_config.right_attractor_sat_low << ",";
		logFile << "  right_attractor_val_low = " << right_arm_config.right_attractor_val_low << ",";
		logFile << "  right_attractor_hue_high = " << right_arm_config.right_attractor_hue_high << ",";
		logFile << "  right_attractor_sat_high = " << right_arm_config.right_attractor_sat_high << ",";
		logFile << "  right_attractor_val_high = " << right_arm_config.right_attractor_val_high << ",";
		logFile << "  fixed_attractor = " << right_arm_config.fixed_attractor << ",";
		//touch
		logFile << "  on_touch = " << right_arm_config.on_touch << ",";
        logFile << "  touch_threshold = " << right_arm_config.touch_threshold << ",";
		logFile << "  grasping = " << right_arm_config.grasping << ",";
		//noise
		logFile << "  gaussian_noise = " << right_arm_config.gaussian_noise << ",";
		logFile << "  gaussian_mean = " << right_arm_config.gaussian_mean << ",";
		logFile << "  gaussian_std = " << right_arm_config.gaussian_std << ",";
		//other
		logFile << "  null_start = " << right_arm_config.null_start;

	}

	//log head configuration
	if (description == "head")
	{
		logFile << "  head_activated = " << head_config.activated << ",";
		//initial position
		logFile << "  neck_pitch = " << head_config.neck_pitch << ",";
		logFile << "  neck_yaw = " << head_config.neck_yaw << ",";
        logFile << "  eyes_tilt = " << head_config.eyes_tilt << ",";
		//internal state
		logFile << "  var_d_encoders = " << head_config.var_d_encoders << ",";
		logFile << "  gain_perception_mu1 = " << head_config.gain_perception_mu1 << ",";
		logFile << "  gain_perception_mu2 = " << head_config.gain_perception_mu2 << ",";
        logFile << "  gain_perception_mu3 = " << head_config.gain_perception_mu3 << ",";
		logFile << "  weighted_perception_mu1 = " << head_config.weighted_perception_mu1 << ",";
		logFile << "  weighted_perception_mu2 = " << head_config.weighted_perception_mu2 << ",";
        logFile << "  weighted_perception_mu3 = " << head_config.weighted_perception_mu3 << ",";
		logFile << "  perception_saturation_on = " << head_config.perception_saturation_on << ",";
		logFile << "  mu1_saturation_low = " << head_config.mu1_saturation_low << ",";
		logFile << "  mu1_saturation_high = " << head_config.mu1_saturation_high << ",";
		logFile << "  mu2_saturation_low = " << head_config.mu2_saturation_low << ",";
		logFile << "  mu2_saturation_high = " << head_config.mu2_saturation_high << ",";
        logFile << "  mu3_saturation_low = " << head_config.mu2_saturation_low << ",";
        logFile << "  mu3_saturation_high = " << head_config.mu2_saturation_high << ",";
		//first derivative
		logFile << "  on_derivative = " << head_config.on_derivative << ",";
		logFile << "  var_dd_attractor = " << head_config.var_dd_attractor << ",";
		logFile << "  gain_derivative_mu1p = " << head_config.gain_derivative_mu1p << ",";
		logFile << "  gain_derivative_mu2p = " << head_config.gain_derivative_mu2p << ",";
        logFile << "  gain_derivative_mu3p = " << head_config.gain_derivative_mu3p << ",";
		logFile << "  derivative_saturation_on = " << head_config.derivative_saturation_on << ",";
		logFile << "  mu1p_saturation_low = " << head_config.mu1p_saturation_low << ",";
		logFile << "  mu1p_saturation_high = " << head_config.mu1p_saturation_high << ",";
		logFile << "  mu2p_saturation_low = " << head_config.mu2p_saturation_low << ",";
		logFile << "  mu2p_saturation_high = " << head_config.mu2p_saturation_high << ",";
        logFile << "  mu3p_saturation_low = " << head_config.mu3p_saturation_low << ",";
        logFile << "  mu3p_saturation_high = " << head_config.mu3p_saturation_high << ",";
		//action
		logFile << "  on_action = " << head_config.on_action << ",";
		logFile << "  var_a_encoders = " << head_config.var_a_encoders << ",";
		logFile << "  gain_action_q1 = " << head_config.gain_action_q1 << ",";
		logFile << "  gain_action_q2 = " << head_config.gain_action_q2 << ",";
        logFile << "  gain_action_q3 = " << head_config.gain_action_q3 << ",";
		logFile << "  action_saturation_on = " << head_config.action_saturation_on << ",";
		logFile << "  action_saturation_low = " << head_config.action_saturation_low << ",";
		logFile << "  action_saturation_high = " << head_config.action_saturation_high << ",";
		//attractor
		logFile << "  ro1_attractor = " << head_config.ro1_attractor << ",";
		logFile << "  var_d_attractor = " << head_config.var_d_attractor << ",";
		//other
		logFile << "  null_start = " << head_config.null_start;
	}
	
	//log right arm configuration
	if (description == "left_arm")
	{
		logFile << "  left_arm_activated = " << left_arm_config.activated << ",";
		//initial position
		logFile << "  shoulder_roll = " << left_arm_config.shoulder_roll << ",";
		logFile << "  shoulder_yaw = " << left_arm_config.shoulder_yaw << ",";
		logFile << "  elbow = " << left_arm_config.elbow << ",";
		//internal state
        logFile << "  on_encoders = " << left_arm_config.on_encoders << ",";
		logFile << "  var_d_encoders = " << left_arm_config.var_d_encoders << ",";
		logFile << "  on_3d = " << left_arm_config.on_3d << ",";
		logFile << "  var_d_3d = " << left_arm_config.var_d_3d << ",";
		logFile << "  on_vision = " << left_arm_config.on_vision << ",";
		logFile << "  var_d_vision = " << left_arm_config.var_d_vision << ",";
		logFile << "  left_vision_hue_low = " << left_arm_config.left_vision_hue_low << ",";
		logFile << "  left_vision_sat_low = " << left_arm_config.left_vision_sat_low << ",";
		logFile << "  left_vision_val_low = " << left_arm_config.left_vision_val_low << ",";
		logFile << "  left_vision_hue_high = " << left_arm_config.left_vision_hue_high << ",";
		logFile << "  left_vision_sat_high = " << left_arm_config.left_vision_sat_high << ",";
		logFile << "  left_vision_val_high = " << left_arm_config.left_vision_val_high << ",";
		logFile << "  right_vision_hue_low = " << left_arm_config.right_vision_hue_low << ",";
		logFile << "  right_vision_sat_low = " << left_arm_config.right_vision_sat_low << ",";
		logFile << "  right_vision_val_low = " << left_arm_config.right_vision_val_low << ",";
		logFile << "  right_vision_hue_high = " << left_arm_config.right_vision_hue_high << ",";
		logFile << "  right_vision_sat_high = " << left_arm_config.right_vision_sat_high << ",";
		logFile << "  right_vision_val_high = " << left_arm_config.right_vision_val_high << ",";
		logFile << "  left_erode_dilate_iterations = " << left_arm_config.left_erode_dilate_iterations << ",";
		logFile << "  right_erode_dilate_iterations = " << left_arm_config.right_erode_dilate_iterations << ",";
		logFile << "  gain_perception_mu1 = " << left_arm_config.gain_perception_mu1 << ",";
		logFile << "  gain_perception_mu2 = " << left_arm_config.gain_perception_mu2 << ",";
		logFile << "  gain_perception_mu3 = " << left_arm_config.gain_perception_mu3 << ",";
		logFile << "  gain_perception_mu4 = " << left_arm_config.gain_perception_mu4 << ",";
		logFile << "  weighted_perception_mu1 = " << left_arm_config.weighted_perception_mu1 << ",";
		logFile << "  weighted_perception_mu2 = " << left_arm_config.weighted_perception_mu2 << ",";
		logFile << "  weighted_perception_mu3 = " << left_arm_config.weighted_perception_mu3 << ",";
		logFile << "  weighted_perception_mu4 = " << left_arm_config.weighted_perception_mu4 << ",";
		logFile << "  perception_saturation_on = " << left_arm_config.perception_saturation_on << ",";
		logFile << "  mu1_saturation_low = " << left_arm_config.mu1_saturation_low << ",";
		logFile << "  mu1_saturation_high = " << left_arm_config.mu1_saturation_high << ",";
		logFile << "  mu2_saturation_low = " << left_arm_config.mu2_saturation_low << ",";
		logFile << "  mu2_saturation_high = " << left_arm_config.mu2_saturation_high << ",";
		logFile << "  mu3_saturation_low = " << left_arm_config.mu3_saturation_low << ",";
		logFile << "  mu3_saturation_high = " << left_arm_config.mu3_saturation_high << ",";
		logFile << "  mu4_saturation_low = " << left_arm_config.mu4_saturation_low << ",";
		logFile << "  mu4_saturation_high = " << left_arm_config.mu4_saturation_high << ",";
		//first derivative
		logFile << "  on_derivative = " << left_arm_config.on_derivative << ",";
		logFile << "  var_dd_attractor = " << left_arm_config.var_dd_attractor << ",";
		logFile << "  gain_derivative_mu1p = " << left_arm_config.gain_derivative_mu1p << ",";
		logFile << "  gain_derivative_mu2p = " << left_arm_config.gain_derivative_mu2p << ",";
		logFile << "  gain_derivative_mu3p = " << left_arm_config.gain_derivative_mu3p << ",";
		logFile << "  gain_derivative_mu4p = " << left_arm_config.gain_derivative_mu4p << ",";
		logFile << "  derivative_saturation_on = " << left_arm_config.derivative_saturation_on << ",";
		logFile << "  mu1p_saturation_low = " << left_arm_config.mu1p_saturation_low << ",";
		logFile << "  mu1p_saturation_high = " << left_arm_config.mu1p_saturation_high << ",";
		logFile << "  mu2p_saturation_low = " << left_arm_config.mu2p_saturation_low << ",";
		logFile << "  mu2p_saturation_high = " << left_arm_config.mu2p_saturation_high << ",";
		logFile << "  mu3p_saturation_low = " << left_arm_config.mu3p_saturation_low << ",";
		logFile << "  mu3p_saturation_high = " << left_arm_config.mu3p_saturation_high << ",";
		logFile << "  mu4p_saturation_low = " << left_arm_config.mu4p_saturation_low << ",";
		logFile << "  mu4p_saturation_high = " << left_arm_config.mu4p_saturation_high << ",";
		//action
		logFile << "  on_action = " << left_arm_config.on_action << ",";
		logFile << "  var_a_encoders = " << left_arm_config.var_a_encoders << ",";
		logFile << "  var_a_3d = " << left_arm_config.var_a_3d << ",";
		logFile << "  var_a_vision = " << left_arm_config.var_a_vision << ",";
		logFile << "  gain_action_q1 = " << left_arm_config.gain_action_q1 << ",";
		logFile << "  gain_action_q2 = " << left_arm_config.gain_action_q2 << ",";
		logFile << "  gain_action_q3 = " << left_arm_config.gain_action_q3 << ",";
		logFile << "  gain_action_q4 = " << left_arm_config.gain_action_q4 << ",";
		logFile << "  action_saturation_on = " << left_arm_config.action_saturation_on << ",";
		logFile << "  action_saturation_low = " << left_arm_config.action_saturation_low << ",";
		logFile << "  action_saturation_high = " << left_arm_config.action_saturation_high << ",";
		//attractor
		logFile << "  on_attractor = " << left_arm_config.on_attractor << ",";
		logFile << "  attractor_type = " << left_arm_config.attractor_type << ",";
		logFile << "  ro1_attractor = " << left_arm_config.ro1_attractor << ",";
		logFile << "  ro2_attractor = " << left_arm_config.ro2_attractor << ",";
		logFile << "  ro3_attractor = " << left_arm_config.ro3_attractor << ",";
		logFile << "  ro4_attractor = " << left_arm_config.ro4_attractor << ",";
		logFile << "  ro5_attractor = " << left_arm_config.ro5_attractor << ",";
		logFile << "  ro6_attractor = " << left_arm_config.ro6_attractor << ",";
		logFile << "  ro7_attractor = " << left_arm_config.ro7_attractor << ",";
		logFile << "  ro8_attractor = " << left_arm_config.ro8_attractor << ",";
		logFile << "  ro9_attractor = " << left_arm_config.ro9_attractor << ",";
		logFile << "  ro10_attractor = " << left_arm_config.ro10_attractor << ",";
		logFile << "  ro11_attractor = " << left_arm_config.ro11_attractor << ",";
		logFile << "  ro12_attractor = " << left_arm_config.ro12_attractor << ",";
		logFile << "  ro13_attractor = " << left_arm_config.ro13_attractor << ",";
		logFile << "  ro14_attractor = " << left_arm_config.ro14_attractor << ",";
		logFile << "  ro15_attractor = " << left_arm_config.ro15_attractor << ",";
		logFile << "  ro16_attractor = " << left_arm_config.ro16_attractor << ",";
		logFile << "  ro17_attractor = " << left_arm_config.ro17_attractor << ",";
		logFile << "  ro18_attractor = " << left_arm_config.ro18_attractor << ",";
		logFile << "  ro19_attractor = " << left_arm_config.ro19_attractor << ",";
		logFile << "  ro20_attractor = " << left_arm_config.ro20_attractor << ",";
		logFile << "  ro21_attractor = " << left_arm_config.ro21_attractor << ",";
		logFile << "  ro22_attractor = " << left_arm_config.ro22_attractor << ",";
		logFile << "  ro23_attractor = " << left_arm_config.ro23_attractor << ",";
		logFile << "  ro24_attractor = " << left_arm_config.ro24_attractor << ",";
		logFile << "  ro25_attractor = " << left_arm_config.ro25_attractor << ",";
        logFile << "  attractor_threshold = " << left_arm_config.attractor_threshold << ",";
        logFile << "  attractor_comparison = " << left_arm_config.attractor_comparison << ",";
		logFile << "  var_d_attractor = " << left_arm_config.var_d_attractor << ",";
		logFile << "  left_attractor_hue_low = " << left_arm_config.left_attractor_hue_low << ",";
		logFile << "  left_attractor_sat_low = " << left_arm_config.left_attractor_sat_low << ",";
		logFile << "  left_attractor_val_low = " << left_arm_config.left_attractor_val_low << ",";
		logFile << "  left_attractor_hue_high = " << left_arm_config.left_attractor_hue_high << ",";
		logFile << "  left_attractor_sat_high = " << left_arm_config.left_attractor_sat_high << ",";
		logFile << "  left_attractor_val_high = " << left_arm_config.left_attractor_val_high << ",";
		logFile << "  right_attractor_hue_low = " << left_arm_config.right_attractor_hue_low << ",";
		logFile << "  right_attractor_sat_low = " << left_arm_config.right_attractor_sat_low << ",";
		logFile << "  right_attractor_val_low = " << left_arm_config.right_attractor_val_low << ",";
		logFile << "  right_attractor_hue_high = " << left_arm_config.right_attractor_hue_high << ",";
		logFile << "  right_attractor_sat_high = " << left_arm_config.right_attractor_sat_high << ",";
		logFile << "  right_attractor_val_high = " << left_arm_config.right_attractor_val_high << ",";
		logFile << "  fixed_attractor = " << left_arm_config.fixed_attractor << ",";
		//touch
		logFile << "  on_touch = " << left_arm_config.on_touch << ",";
        logFile << "  touch_threshold = " << left_arm_config.touch_threshold << ",";
		logFile << "  grasping = " << left_arm_config.grasping << ",";
		//noise
		logFile << "  gaussian_noise = " << left_arm_config.gaussian_noise << ",";
		logFile << "  gaussian_mean = " << left_arm_config.gaussian_mean << ",";
		logFile << "  gaussian_std = " << left_arm_config.gaussian_std << ",";
		//other
		logFile << "  null_start = " << left_arm_config.null_start;

	}

	logFile << endl;

	//log variable relation
    if (description == "right_arm") logFile << "[VARS] t mu1 dmu1 mu2 dmu2 mu3 dmu3 mu4 dmu4 mu1p dmu1p mu2p dmu2p mu3p dmu3p mu4p dmu4p a1 da1 a2 da2 a3 da3 a4 da4 enc1(noise) enc1 enc2(noise) enc2 enc3(noise) enc3 enc4(noise) enc4 (x calc_x y calc_y z calc_z) (l_u calc_l_u l_v calc_l_v r_u calc_r_u r_v calc_r_v) (attr_x attr_y attr_z attr_l_u attr_l_v attr_r_u attr_r_v) F dt" << endl;
    if (description == "head") logFile << "[VARS] t mu1 dmu1 mu2 dmu2 mu3 dmu3 mu1p dmu1p mu2p dmu2p mu3p dmu3p a1 da1 a2 da2 a3 da3 enc1(noise) enc1 enc2(noise) enc2 enc3(noise) enc3 (attr_l_u attr_l_v) F dt" << endl;
    if (description == "left_arm") logFile << "[VARS] t mu1 dmu1 mu2 dmu2 mu3 dmu3 mu1p dmu1p mu2p dmu2p mu3p dmu3p a1 da1 a2 da2 a3 da3 enc1(noise) enc1 enc2(noise) enc2 enc3(noise) enc3 (u calc_u v calc_v) (attr_u calc_u attr_v calc_v) F dt" << endl;
		
}

void log_stats(fstream& logFile, string description)
{
	//log thread execution stats
	logFile << "[TSTATS]";

    //free-energy
    if (description == "right_arm") if (right_arm_config.activated) logFile << "  free-energy right arm: {" << feoRightArmThread->getIterations() << ", " << feoRightArmThread->getEstPeriod() << " ms}";
    if (description == "head") if (head_config.activated) logFile << "  free-energy head: {" << feoHeadThread->getIterations() << ", " << feoHeadThread->getEstPeriod() << " ms}";
    if (description == "left_arm") if (left_arm_config.activated) logFile << "  free-energy left arm: {" << feoLeftArmThread->getIterations() << ", " << feoLeftArmThread->getEstPeriod() << " ms}";

    //action
    if (description == "right_arm") if (right_arm_config.on_action && right_arm_config.activated) logFile << "  action right arm: {" << actionRightArmThread->getIterations() << ", " << actionRightArmThread->getEstPeriod() << " ms}";
    if (description == "head") if (head_config.on_action && head_config.activated) logFile << "  action head: {" << actionHeadThread->getIterations() << ", " << actionHeadThread->getEstPeriod() << " ms}";
    if (description == "left_arm") if (left_arm_config.on_action && left_arm_config.activated) logFile << "  action left arm: {" << actionLeftArmThread->getIterations() << ", " << actionLeftArmThread->getEstPeriod() << " ms}";

    //vision
    if (right_arm_config.on_vision || right_arm_config.on_attractor) logFile << "  vision: {" << visionThread->getIterations() << ", " << visionThread->getEstPeriod() << " ms}";

    //touch
    if (description == "right_arm") if (right_arm_config.on_touch && right_arm_config.activated) logFile << "  touch: {" << touchThread->getIterations() << ", " << touchThread->getEstPeriod() << " ms}";
    if (description == "left_arm") if (left_arm_config.on_touch && left_arm_config.activated) logFile << "  touch: {" << touchThread->getIterations() << ", " << touchThread->getEstPeriod() << " ms}";

    logFile << endl;
	
}

//----------------------------------------------------------------------------------
//Free energy optimization setup ---------------------------------------------------
//----------------------------------------------------------------------------------

//free energy optimization algorithm for right arm
void feo_right_arm_setup()
{

	//Derivative terms
	DerivativeTerm d_encoders, d_prior, d_pos3D, d_vision, d_attractor, dd_attractor;
	fvector realState, calcState, partialDerivativesRow;
	fmatrix partialDerivatives;
	//Action terms
	ActionTerm a_encoders, a_pos3D, a_vision;
	//Free-energy terms
	FreeEnergyTerm fe_encoders, fe_pos3D, fe_vision, fe_attractor;

	feoRightArm = new FreeEnergyOptimization();

	cout << "* Starting right arm setup..." << endl;

	//Obtaining initial state
	mu0.clear();
	a.clear(); a.push_back(0); a.push_back(0); a.push_back(0); a.push_back(0);
	if (right_arm_config.null_start)
	{
		mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); //zero position
	}else{
		mu0.push_back(read_encoders_q1(mu0, a)); mu0.push_back(read_encoders_q2(mu0, a)); 
		mu0.push_back(read_encoders_q3(mu0, a)); mu0.push_back(read_encoders_q4(mu0, a)); //current position
	}
	mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); //velocity
	initialPrior.push_back(mu0[0]); initialPrior.push_back(mu0[1]); initialPrior.push_back(mu0[2]); initialPrior.push_back(mu0[3]);
    state = mu0; //initial state
	
	//Set up parameters
	IntegrationMethod intMethod = Euler;
	//if (config.integration_method == "LocalLinearization") intMethod = LocalLinear;
	vector<double> gainPerception; 
	gainPerception.push_back(right_arm_config.gain_perception_mu1); gainPerception.push_back(right_arm_config.gain_perception_mu2); 
	gainPerception.push_back(right_arm_config.gain_perception_mu3); gainPerception.push_back(right_arm_config.gain_perception_mu4);
	gainPerception.push_back(right_arm_config.gain_derivative_mu1p); gainPerception.push_back(right_arm_config.gain_derivative_mu2p);
	gainPerception.push_back(right_arm_config.gain_derivative_mu3p); gainPerception.push_back(right_arm_config.gain_derivative_mu4p);
	vector<double> weightedPerception; 
	weightedPerception.push_back(right_arm_config.weighted_perception_mu1); weightedPerception.push_back(right_arm_config.weighted_perception_mu2);
	weightedPerception.push_back(right_arm_config.weighted_perception_mu3);	weightedPerception.push_back(right_arm_config.weighted_perception_mu4);
	vector<double> gainAction; 
	gainAction.push_back(right_arm_config.gain_action_q1); gainAction.push_back(right_arm_config.gain_action_q2);
	gainAction.push_back(right_arm_config.gain_action_q3); gainAction.push_back(right_arm_config.gain_action_q4);
	feoRightArm->setParameters("right_arm", 4, 2, mu0, main_config.dT, right_arm_config.on_action, gainPerception, weightedPerception, gainAction, intMethod, main_config.debug); //4 DOF and 2 derivatives calculated
	if (main_config.log) //logging
	{
		feoRightArm->setStartLoggingFunction(log_config);
		feoRightArm->setEndLoggingFunction(log_stats);
	}
	if (right_arm_config.perception_saturation_on) //perception saturation
	{
		vector<double> saturationLow; 
		saturationLow.push_back(right_arm_config.mu1_saturation_low*CTRL_DEG2RAD); 
		saturationLow.push_back(right_arm_config.mu2_saturation_low*CTRL_DEG2RAD); 
		saturationLow.push_back(right_arm_config.mu3_saturation_low*CTRL_DEG2RAD);
		saturationLow.push_back(right_arm_config.mu4_saturation_low*CTRL_DEG2RAD);			
		vector<double> saturationHigh; 
		saturationHigh.push_back(right_arm_config.mu1_saturation_high*CTRL_DEG2RAD); 
		saturationHigh.push_back(right_arm_config.mu2_saturation_high*CTRL_DEG2RAD);  
		saturationHigh.push_back(right_arm_config.mu3_saturation_high*CTRL_DEG2RAD); 
		saturationHigh.push_back(right_arm_config.mu4_saturation_high*CTRL_DEG2RAD);
		feoRightArm->setPerceptionSaturation(right_arm_config.perception_saturation_on, saturationLow, saturationHigh);
	}
	if (right_arm_config.derivative_saturation_on) //derivative saturation
	{
		vector<double> saturationLowD; 
		saturationLowD.push_back(right_arm_config.mu1p_saturation_low*CTRL_DEG2RAD); 
		saturationLowD.push_back(right_arm_config.mu2p_saturation_low*CTRL_DEG2RAD); 
		saturationLowD.push_back(right_arm_config.mu3p_saturation_low*CTRL_DEG2RAD);
		saturationLowD.push_back(right_arm_config.mu4p_saturation_low*CTRL_DEG2RAD);			
		vector<double> saturationHighD; 
		saturationHighD.push_back(right_arm_config.mu1p_saturation_high*CTRL_DEG2RAD); 
		saturationHighD.push_back(right_arm_config.mu2p_saturation_high*CTRL_DEG2RAD);
		saturationHighD.push_back(right_arm_config.mu3p_saturation_high*CTRL_DEG2RAD);  
		saturationHighD.push_back(right_arm_config.mu4p_saturation_high*CTRL_DEG2RAD);  
		feoRightArm->setDerivativeSaturation(right_arm_config.derivative_saturation_on, saturationLowD, saturationHighD);
	}
	if (right_arm_config.action_saturation_on) //action saturation
	{
		feoRightArm->setActionSaturation(right_arm_config.action_saturation_on, right_arm_config.action_saturation_low*CTRL_DEG2RAD, right_arm_config.action_saturation_high*CTRL_DEG2RAD);
	}
	//set action to zero when touching or reached limit positions
	feoRightArm->setZeroConditionFunction(zero_condition_right_arm);
	
	//Logging
    string log_name = "logs/log_active_inference_right_arm_" + time_str + "_" + main_config.description;
	if (main_config.log) feoRightArm->startLogging(log_name);
	
	cout << "1) First derivative terms..." << endl;
	
    if (right_arm_config.on_encoders)
    {

        //First term: encoders
        cout << "Adding encoders term..." << endl;

        realState.clear();
        calcState.clear();
        d_encoders.description = "d_encoders";
        if (right_arm_config.gaussian_noise)
        {
            realState.push_back(read_encoders_q1_noise);
            realState.push_back(read_encoders_q2_noise);
            realState.push_back(read_encoders_q3_noise);
            realState.push_back(read_encoders_q4_noise);
        }
        else
        {
            realState.push_back(read_encoders_q1);
            realState.push_back(read_encoders_q2);
            realState.push_back(read_encoders_q3);
            realState.push_back(read_encoders_q4);
        }
        calcState.push_back(internal_state_mu1);
        calcState.push_back(internal_state_mu2);
        calcState.push_back(internal_state_mu3);
        calcState.push_back(internal_state_mu4);
        d_encoders.realState = realState;
        d_encoders.calcState = calcState;
        d_encoders.variance = right_arm_config.var_d_encoders;
        d_encoders.logRealState = true;
        feoRightArm->addNewDerivativeTerm(0, d_encoders);

    }
	
	if (right_arm_config.on_3d)
	{

		//Second term: stereo vision 3d position
		cout << "Adding stereo vision 3d position term ..." << endl;

		realState.clear();
		calcState.clear();
		partialDerivatives.clear();
		partialDerivativesRow.clear();
		d_pos3D.description = "d_pos3D";
		realState.push_back(sense_3d_position_x);
		realState.push_back(sense_3d_position_y);
		realState.push_back(sense_3d_position_z);
		calcState.push_back(calc_3d_position_x);
		calcState.push_back(calc_3d_position_y);
		calcState.push_back(calc_3d_position_z);
		partialDerivativesRow.push_back(d_3d_position_x_mu1);
		partialDerivativesRow.push_back(d_3d_position_y_mu1);
		partialDerivativesRow.push_back(d_3d_position_z_mu1);
		partialDerivatives.push_back(partialDerivativesRow);
		partialDerivativesRow.clear();
		partialDerivativesRow.push_back(d_3d_position_x_mu2);
		partialDerivativesRow.push_back(d_3d_position_y_mu2);
		partialDerivativesRow.push_back(d_3d_position_z_mu2);
		partialDerivatives.push_back(partialDerivativesRow);
		partialDerivativesRow.clear();
		partialDerivativesRow.push_back(d_3d_position_x_mu3);
		partialDerivativesRow.push_back(d_3d_position_y_mu3);
		partialDerivativesRow.push_back(d_3d_position_z_mu3);
		partialDerivatives.push_back(partialDerivativesRow);		
		partialDerivativesRow.clear();
		partialDerivativesRow.push_back(d_3d_position_x_mu4);
		partialDerivativesRow.push_back(d_3d_position_y_mu4);
		partialDerivativesRow.push_back(d_3d_position_z_mu4);
		partialDerivatives.push_back(partialDerivativesRow);
		d_pos3D.realState = realState;
		d_pos3D.calcState = calcState;
		d_pos3D.partialDerivativesOn = true;
		d_pos3D.partialDerivatives = partialDerivatives;
		d_pos3D.variance = right_arm_config.var_d_3d;
        d_pos3D.logRealState = true;
        d_pos3D.logCalcState = true;
        d_pos3D.deactivateNull = true; //do not consider if sensed position is 0
		feoRightArm->addNewDerivativeTerm(0, d_pos3D);
		
	}
	
	//Normal dynamics term is 0: mup = f(mu,ro) = v + Z
		
	if (right_arm_config.on_attractor)
	{
	
		//Fourth term: dynamics with attractor
		//cout << "Adding attractor dynamics term..." << endl;

		realState.clear();
		calcState.clear();
		partialDerivatives.clear();
		partialDerivativesRow.clear();
		d_attractor.description = "d_attractor";
		realState.push_back(internal_state_mu1p);
		realState.push_back(internal_state_mu2p);
		realState.push_back(internal_state_mu3p);
		realState.push_back(internal_state_mu4p);
		
		//attractor dynamics depends on type
		switch (right_arm_config.attractor_type)
		{
			case 0: //stereo vision with hsv recognition (position x-y-z)
				cout << "Adding stereo vision 3d attractor dynamics term..." << endl;
				calcState.push_back(calc_attractor_stereo_f1);
				calcState.push_back(calc_attractor_stereo_f2);
				calcState.push_back(calc_attractor_stereo_f3);
				calcState.push_back(calc_attractor_stereo_f4);
				//partialDerivativesRow.push_back(d_attractor_3d_f1_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f2_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f3_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f4_mu1);
				//partialDerivatives.push_back(partialDerivativesRow);
				//partialDerivativesRow.clear();
				//partialDerivativesRow.push_back(d_attractor_3d_f1_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f2_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f3_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f4_mu2);
				//partialDerivatives.push_back(partialDerivativesRow);
				break;
            case 1: //3d fixed position (x-y-z)
                cout << "Adding fixed 3d attractor dynamics term..." << endl;
                calcState.push_back(calc_attractor_stereo_f1);
                calcState.push_back(calc_attractor_stereo_f2);
                calcState.push_back(calc_attractor_stereo_f3);
                calcState.push_back(calc_attractor_stereo_f4);
                break;
            case 2: //3d cube, 8 points (x-y-z)
                cout << "Adding 3d cube attractor dynamics term..." << endl;
                calcState.push_back(calc_attractor_stereo_f1);
                calcState.push_back(calc_attractor_stereo_f2);
                calcState.push_back(calc_attractor_stereo_f3);
                calcState.push_back(calc_attractor_stereo_f4);
                break;
		}

		d_attractor.realState = realState;
		d_attractor.calcState = calcState;
		//d_attractor.partialDerivativesOn = true;
		//d_attractor.partialDerivatives = partialDerivatives;
		d_attractor.sign = -1; //fix for the absence of partial derivatives
		d_attractor.variance = right_arm_config.var_d_attractor;
		d_attractor.logCalcState = false;
		feoRightArm->addNewDerivativeTerm(0, d_attractor);
	
	}
		
	if (right_arm_config.on_prior)
	{
	
		//Fifth term: prior
		cout << "Adding prior term..." << endl;

		realState.clear();
		calcState.clear();
		d_prior.description = "prior";
		realState.push_back(internal_state_mu1);
		realState.push_back(internal_state_mu2);
		realState.push_back(internal_state_mu3);
		realState.push_back(internal_state_mu4);
		calcState.push_back(initial_prior_mu1);
		calcState.push_back(initial_prior_mu2);
		calcState.push_back(initial_prior_mu3);
		calcState.push_back(initial_prior_mu4);
		d_prior.realState = realState;
		d_prior.calcState = calcState;
		d_prior.sign = -1;
		d_prior.variance = right_arm_config.var_d_prior;
		feoRightArm->addNewDerivativeTerm(0, d_prior);
	
	}
		
	cout << "Number of first derivative terms: " << feoRightArm->getNumDerivativeTerms(0) << endl;
	
	cout << "2) Second derivative terms..." << endl;

	if (right_arm_config.on_derivative)
	{

		if (right_arm_config.on_attractor)
		{
	
			//First term: dynamics with attractor
			//cout << "Adding attractor dynamics term..." << endl;

			realState.clear();
			calcState.clear();
			partialDerivatives.clear();
			partialDerivativesRow.clear();
			dd_attractor.description = "dd_attractor";
			realState.push_back(internal_state_mu1p);
			realState.push_back(internal_state_mu2p);
			realState.push_back(internal_state_mu3p);
			realState.push_back(internal_state_mu4p);
		
			//attractor dynamics depends on type
			switch (right_arm_config.attractor_type)
			{
				case 0: //stereo vision with hsv recognition (position x-y-z)
					cout << "Adding stereo vision 3d attractor dynamics term..." << endl;
					calcState.push_back(calc_attractor_stereo_f1);
					calcState.push_back(calc_attractor_stereo_f2);
					calcState.push_back(calc_attractor_stereo_f3);
					calcState.push_back(calc_attractor_stereo_f4);
					break;
                case 1: //3d fixed position (x-y-z)
                    cout << "Adding fixed 3d attractor dynamics term..." << endl;
                    calcState.push_back(calc_attractor_stereo_f1);
                    calcState.push_back(calc_attractor_stereo_f2);
                    calcState.push_back(calc_attractor_stereo_f3);
                    calcState.push_back(calc_attractor_stereo_f4);
                    break;
                case 2: //3d cube, 8 points (x-y-z)
                    cout << "Adding 3d cube attractor dynamics term..." << endl;
                    calcState.push_back(calc_attractor_stereo_f1);
                    calcState.push_back(calc_attractor_stereo_f2);
                    calcState.push_back(calc_attractor_stereo_f3);
                    calcState.push_back(calc_attractor_stereo_f4);
                    break;
			}

			dd_attractor.realState = realState;
			dd_attractor.calcState = calcState;
			dd_attractor.sign = -1;
			dd_attractor.variance = right_arm_config.var_dd_attractor;
			dd_attractor.logCalcState = false;
			feoRightArm->addNewDerivativeTerm(1, dd_attractor);
		}
	}
	
	cout << "Number of second derivative terms: " << feoRightArm->getNumDerivativeTerms(1) << endl;
	
	if (right_arm_config.on_action){
	
		cout << "3) Action terms..." << endl;

        if (right_arm_config.on_encoders)
        {

            //First term: encoders
            cout << "Adding encoders term..." << endl;

            realState.clear();
            calcState.clear();
            partialDerivatives.clear();
            partialDerivativesRow.clear();
            a_encoders.description = "a_encoders";
            if (right_arm_config.gaussian_noise)
            {
                realState.push_back(read_encoders_q1_noise);
                realState.push_back(read_encoders_q2_noise);
                realState.push_back(read_encoders_q3_noise);
                realState.push_back(read_encoders_q4_noise);
            }
            else
            {
                realState.push_back(read_encoders_q1_action);
                realState.push_back(read_encoders_q2);
                realState.push_back(read_encoders_q3);
                realState.push_back(read_encoders_q4);
            }
            calcState.push_back(internal_state_mu1);
            calcState.push_back(internal_state_mu2);
            calcState.push_back(internal_state_mu3);
            calcState.push_back(internal_state_mu4);
            partialDerivativesRow.push_back(d_encoders_q1_a1);
            partialDerivativesRow.push_back(d_encoders_q2_a1);
            partialDerivativesRow.push_back(d_encoders_q3_a1);
            partialDerivativesRow.push_back(d_encoders_q4_a1);
            partialDerivatives.push_back(partialDerivativesRow);
            partialDerivativesRow.clear();
            partialDerivativesRow.push_back(d_encoders_q1_a2);
            partialDerivativesRow.push_back(d_encoders_q2_a2);
            partialDerivativesRow.push_back(d_encoders_q3_a2);
            partialDerivativesRow.push_back(d_encoders_q4_a2);
            partialDerivatives.push_back(partialDerivativesRow);
            partialDerivativesRow.clear();
            partialDerivativesRow.push_back(d_encoders_q1_a3);
            partialDerivativesRow.push_back(d_encoders_q2_a3);
            partialDerivativesRow.push_back(d_encoders_q3_a3);
            partialDerivativesRow.push_back(d_encoders_q4_a3);
            partialDerivatives.push_back(partialDerivativesRow);
            partialDerivativesRow.clear();
            partialDerivativesRow.push_back(d_encoders_q1_a4);
            partialDerivativesRow.push_back(d_encoders_q2_a4);
            partialDerivativesRow.push_back(d_encoders_q3_a4);
            partialDerivativesRow.push_back(d_encoders_q4_a4);
            partialDerivatives.push_back(partialDerivativesRow);
            a_encoders.realState = realState;
            a_encoders.calcState = calcState;
            a_encoders.partialDerivativesOn = true;
            a_encoders.partialDerivatives = partialDerivatives;
            a_encoders.sign = -1; //negative sign
            a_encoders.variance = right_arm_config.var_a_encoders;
            feoRightArm->addNewActionTerm(a_encoders);

        }

		if (right_arm_config.on_3d)
		{

			//Second term: 3d position
			cout << "Adding 3d position term..." << endl;

			realState.clear();
			calcState.clear();
			partialDerivatives.clear();
			partialDerivativesRow.clear();
			a_pos3D.description = "a_pos3D";
			realState.push_back(sense_3d_position_x_action);
			realState.push_back(sense_3d_position_y);
			realState.push_back(sense_3d_position_z);
			calcState.push_back(calc_3d_position_x);
			calcState.push_back(calc_3d_position_y);
			calcState.push_back(calc_3d_position_z);
			partialDerivativesRow.push_back(d_3d_position_x_a1);
			partialDerivativesRow.push_back(d_3d_position_y_a1);
			partialDerivativesRow.push_back(d_3d_position_z_a1);
			partialDerivatives.push_back(partialDerivativesRow);
			partialDerivativesRow.clear();
			partialDerivativesRow.push_back(d_3d_position_x_a2);
			partialDerivativesRow.push_back(d_3d_position_y_a2);
			partialDerivativesRow.push_back(d_3d_position_z_a2);
			partialDerivatives.push_back(partialDerivativesRow);
			partialDerivativesRow.clear();
			partialDerivativesRow.push_back(d_3d_position_x_a3);
			partialDerivativesRow.push_back(d_3d_position_y_a3);
			partialDerivativesRow.push_back(d_3d_position_z_a3);
			partialDerivatives.push_back(partialDerivativesRow);
			partialDerivativesRow.clear();
			partialDerivativesRow.push_back(d_3d_position_x_a4);
			partialDerivativesRow.push_back(d_3d_position_y_a4);
			partialDerivativesRow.push_back(d_3d_position_z_a4);
			partialDerivatives.push_back(partialDerivativesRow);
			a_pos3D.realState = realState;
			a_pos3D.calcState = calcState;
			a_pos3D.partialDerivativesOn = true;
			a_pos3D.partialDerivatives = partialDerivatives;
			a_pos3D.sign = -1; //negative sign
			a_pos3D.variance = right_arm_config.var_a_3d;
            a_pos3D.deactivateNull = true; //do not consider if sensed position is 0
			feoRightArm->addNewActionTerm(a_pos3D);

        }

		cout << "Number of action terms: " << feoRightArm->getNumActionTerms() << endl;

	}
	
	
	//Free-energy terms
	cout << "4) Free-energy terms..." << endl;

    if (right_arm_config.on_encoders)
    {

        //First term: encoders
        cout << "Adding encoders term..." << endl;

        realState.clear();
        calcState.clear();
        fe_encoders.description = "fe_encoders";
        if (right_arm_config.gaussian_noise)
        {
            realState.push_back(read_encoders_q1_noise);
            realState.push_back(read_encoders_q2_noise);
            realState.push_back(read_encoders_q3_noise);
            realState.push_back(read_encoders_q4_noise);
        }
        else
        {
            realState.push_back(read_encoders_q1_action);
            realState.push_back(read_encoders_q2);
            realState.push_back(read_encoders_q3);
            realState.push_back(read_encoders_q4);
        }
        calcState.push_back(internal_state_mu1);
        calcState.push_back(internal_state_mu2);
        calcState.push_back(internal_state_mu3);
        calcState.push_back(internal_state_mu4);
        fe_encoders.realState = realState;
        fe_encoders.calcState = calcState;
        fe_encoders.sign = -1;
        fe_encoders.variance = right_arm_config.var_d_encoders;
        feoRightArm->addNewFreeEnergyTerm(fe_encoders);

    }

	if (right_arm_config.on_3d)
	{

		//Second term: 3d position
		cout << "Adding 3d position term..." << endl;

		realState.clear();
		calcState.clear();
		fe_pos3D.description = "fe_pos3D";
		realState.push_back(sense_3d_position_x_action);
		realState.push_back(sense_3d_position_y);
		realState.push_back(sense_3d_position_z);
		calcState.push_back(calc_3d_position_x);
		calcState.push_back(calc_3d_position_y);
		calcState.push_back(calc_3d_position_z);
		fe_pos3D.realState = realState;
		fe_pos3D.calcState = calcState;
		fe_pos3D.sign = -1;
		fe_pos3D.variance = right_arm_config.var_d_3d;
		fe_pos3D.deactivateNull = true; //do not consider if sensed position is 0
		feoRightArm->addNewFreeEnergyTerm(fe_pos3D);

	}
	
	//Normal dynamics term is 0: mup = f(mu,ro) = v + Z
		
	if (right_arm_config.on_attractor)
	{
	
		//Fourth term: dynamics with attractor
		//cout << "Adding attractor dynamics term..." << endl;

		realState.clear();
		calcState.clear();
		fe_attractor.description = "fe_attractor";
		realState.push_back(internal_state_mu1p);
		realState.push_back(internal_state_mu2p);
		realState.push_back(internal_state_mu3p);
		realState.push_back(internal_state_mu4p);

		//attractor dynamics depends on type
		switch (right_arm_config.attractor_type)
		{
			case 0: //stereo vision with hsv recognition (position x-y-z)
				cout << "Adding stereo vision 3d attractor dynamics term..." << endl;
				calcState.push_back(calc_attractor_stereo_f1);
				calcState.push_back(calc_attractor_stereo_f2);
				calcState.push_back(calc_attractor_stereo_f3);
				calcState.push_back(calc_attractor_stereo_f4);
				break;
            case 1: //3d fixed position (x-y-z)
                cout << "Adding fixed 3d attractor dynamics term..." << endl;
                calcState.push_back(calc_attractor_stereo_f1);
                calcState.push_back(calc_attractor_stereo_f2);
                calcState.push_back(calc_attractor_stereo_f3);
                calcState.push_back(calc_attractor_stereo_f4);
                break;
            case 2: //3d cube, 8 points (x-y-z)
                cout << "Adding 3d cube attractor dynamics term..." << endl;
                calcState.push_back(calc_attractor_stereo_f1);
                calcState.push_back(calc_attractor_stereo_f2);
                calcState.push_back(calc_attractor_stereo_f3);
                calcState.push_back(calc_attractor_stereo_f4);
                break;
		}

		fe_attractor.realState = realState;
		fe_attractor.calcState = calcState;
		fe_attractor.sign = -1;
		fe_attractor.variance = right_arm_config.var_d_attractor;
		feoRightArm->addNewFreeEnergyTerm(fe_attractor);
	
	}

	cout << "Number of free-energy terms: " << feoRightArm->getNumFreeEnergyTerms() << endl;

}

void feo_right_arm_run()
{
	//Set action thread
	if (right_arm_config.on_action){
		
		//Change to velocity control on selected joints
		iCubTorsoCtrl->changeControlMode(0, velocity); //torso_yaw
		iCubRightArmCtrl->changeControlMode(1, velocity); //r_shoulder_roll
		iCubRightArmCtrl->changeControlMode(2, velocity); //r_shoulder_yaw
		iCubRightArmCtrl->changeControlMode(3, velocity); //r_elbow
	
		//Activate action thread	
		actionRightArmThread = new YarpPeriodic(0.5*dT*1000, "action_right_arm");
		actionRightArmThread->setRun(set_action_right_arm_thread);
		actionRightArmThread->start();
	}
	
	//Get internal state
	state = feoRightArm->getInternalState();
	
	cout << "Current right arm state:";
	for (unsigned int i = 0; i < state.size(); i++)
	{
		cout << " " << state[i];
	}
	cout << endl;
	
	//Run thread
	feoRightArmThread = new YarpPeriodic(dT*1000, "free-energy_right_arm");
	feoRightArmThread->setRun(calculate_feo_right_arm);
	feoRightArmThread->start();

}

//zero condition function for right arm
bool zero_condition_right_arm(double action, int dof)
{
	//touching condition
	if (right_arm_config.on_touch) if (touching) return true;
    if (left_arm_config.on_touch && left_arm_config.activated) if (touchingLeft) return true;

	switch (dof)
	{
		case 0: //torso_yaw
			//cout << qTencoders[0]*CTRL_RAD2DEG << ", " << action << endl;
            if (qTencoders[0]*CTRL_RAD2DEG > torso_yaw_max && action > 0) return true; //-50 -> 50
            if (qTencoders[0]*CTRL_RAD2DEG < torso_yaw_min && action < 0) return true; //-50 -> 50
			break;		
		case 1: //r_shoulder_roll
			//cout << qRAencoders[1]*CTRL_RAD2DEG << ", " << action << endl;
            if (qRAencoders[1]*CTRL_RAD2DEG > shoulder_roll_max && action > 0) return true; //0 -> 160.8
            if (qRAencoders[1]*CTRL_RAD2DEG < shoulder_roll_min && action < 0) return true; //0 -> 160.8
			break;
		case 2: //r_shoulder_yaw
			//cout << qRAencoders[2]*CTRL_RAD2DEG << ", " << action << endl;
            if (qRAencoders[2]*CTRL_RAD2DEG > shoulder_yaw_max && action > 0) return true; //-37 -> 100
            if (qRAencoders[2]*CTRL_RAD2DEG < shoulder_yaw_min && action < 0) return true; //-37 -> 100
			break;
		case 3: //r_elbow
			//cout << qRAencoders[3]*CTRL_RAD2DEG << ", " << action << endl;
            if (qRAencoders[3]*CTRL_RAD2DEG > elbow_max && action > 0) return true; //5.5 -> 106
            if (qRAencoders[3]*CTRL_RAD2DEG < elbow_min && action < 0) return true; //5.5 -> 106
			break;
	}

	return false;
}

//free energy optimization algorithm for head
void feo_head_setup()
{

	//Derivative terms
	DerivativeTerm d_encoders, d_attractor, dd_attractor;
	fvector realState, calcState, partialDerivativesRow;
	fmatrix partialDerivatives;
	//Action terms
	ActionTerm a_encoders;
	//Free-energy terms
	FreeEnergyTerm fe_encoders, fe_attractor;

	feoHead = new FreeEnergyOptimization();

	cout << "* Starting head setup..." << endl;

	//Obtaining initial state
	mu0.clear();
    a.clear(); a.push_back(0); a.push_back(0); a.push_back(0);
	if (head_config.null_start)
	{
        mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); //zero position
	}else{
        mu0.push_back(read_encoders_q1e(mu0, a)); mu0.push_back(read_encoders_q2e(mu0, a));
        mu0.push_back(read_encoders_q3e(mu0, a));//current position
	}
    mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); //velocity
    //initialPrior.clear(); initialPrior.push_back(mu0[0]); initialPrior.push_back(mu0[1]);
	
	//Set up parameters
	IntegrationMethod intMethod = Euler;
	vector<double> gainPerception; 
	gainPerception.push_back(head_config.gain_perception_mu1); gainPerception.push_back(head_config.gain_perception_mu2);
    gainPerception.push_back(head_config.gain_perception_mu3);
	gainPerception.push_back(head_config.gain_derivative_mu1p); gainPerception.push_back(head_config.gain_derivative_mu2p);
    gainPerception.push_back(head_config.gain_derivative_mu3p);
	vector<double> weightedPerception; 
	weightedPerception.push_back(head_config.weighted_perception_mu1); weightedPerception.push_back(head_config.weighted_perception_mu2); 
    weightedPerception.push_back(head_config.weighted_perception_mu3);
	vector<double> gainAction; 
	gainAction.push_back(head_config.gain_action_q1); gainAction.push_back(head_config.gain_action_q2);
    gainAction.push_back(head_config.gain_action_q3);
    feoHead->setParameters("head", 3, 2, mu0, main_config.dT, head_config.on_action, gainPerception, weightedPerception, gainAction, intMethod, main_config.debug); //3 DOF and 2 derivatives calculated
	if (main_config.log) //logging
	{
		feoHead->setStartLoggingFunction(log_config);
		feoHead->setEndLoggingFunction(log_stats);
	}
	if (head_config.perception_saturation_on) //perception saturation
	{
		vector<double> saturationLow; 
		saturationLow.push_back(head_config.mu1_saturation_low*CTRL_DEG2RAD); 
        saturationLow.push_back(head_config.mu2_saturation_low*CTRL_DEG2RAD);
        saturationLow.push_back(head_config.mu3_saturation_low*CTRL_DEG2RAD);
		vector<double> saturationHigh; 
		saturationHigh.push_back(head_config.mu1_saturation_high*CTRL_DEG2RAD); 
        saturationHigh.push_back(head_config.mu2_saturation_high*CTRL_DEG2RAD);
        saturationHigh.push_back(head_config.mu3_saturation_high*CTRL_DEG2RAD);
		feoHead->setPerceptionSaturation(head_config.perception_saturation_on, saturationLow, saturationHigh);
	}
	if (head_config.derivative_saturation_on) //derivative saturation
	{
		vector<double> saturationLowD; 
		saturationLowD.push_back(head_config.mu1p_saturation_low*CTRL_DEG2RAD); 
        saturationLowD.push_back(head_config.mu2p_saturation_low*CTRL_DEG2RAD);
        saturationLowD.push_back(head_config.mu3p_saturation_low*CTRL_DEG2RAD);
		vector<double> saturationHighD; 
		saturationHighD.push_back(head_config.mu1p_saturation_high*CTRL_DEG2RAD); 
		saturationHighD.push_back(head_config.mu2p_saturation_high*CTRL_DEG2RAD);
        saturationHighD.push_back(head_config.mu3p_saturation_high*CTRL_DEG2RAD);
		feoHead->setDerivativeSaturation(head_config.derivative_saturation_on, saturationLowD, saturationHighD);
	}
	if (head_config.action_saturation_on) //action saturation
	{
		feoHead->setActionSaturation(head_config.action_saturation_on, head_config.action_saturation_low*CTRL_DEG2RAD, head_config.action_saturation_high*CTRL_DEG2RAD);
	}
	//set action to zero when touching or reached limit positions
	feoHead->setZeroConditionFunction(zero_condition_head);
	
	//Logging
    string log_name = "logs/log_active_inference_head_" + time_str + "_" + main_config.description;
	if (main_config.log) feoHead->startLogging(log_name);
	
	cout << "1) First derivative terms..." << endl;
	
	//First term: encoders
	cout << "Adding encoders term..." << endl;

	realState.clear();
	calcState.clear();
	d_encoders.description = "d_encoders";		
	realState.push_back(read_encoders_q1e);
	realState.push_back(read_encoders_q2e);
    realState.push_back(read_encoders_q3e);
	calcState.push_back(internal_state_mu1e);
	calcState.push_back(internal_state_mu2e);
    calcState.push_back(internal_state_mu3e);
	d_encoders.realState = realState;
	d_encoders.calcState = calcState;
	d_encoders.variance = head_config.var_d_encoders;
	d_encoders.logRealState = true;
	feoHead->addNewDerivativeTerm(0, d_encoders);
	
	//Normal dynamics term is 0: mup = f(mu,ro) = v + Z
		
	if (right_arm_config.on_attractor)
	{
	
		//Second term: dynamics with attractor
		//cout << "Adding attractor dynamics term..." << endl;

		realState.clear();
		calcState.clear();
		partialDerivatives.clear();
		partialDerivativesRow.clear();
		d_attractor.description = "d_attractor";
		realState.push_back(internal_state_mu1ep);
		realState.push_back(internal_state_mu2ep);
        realState.push_back(internal_state_mu3ep);
		
		//attractor dynamics depends on type
		switch (right_arm_config.attractor_type)
		{
            case 0: //stereo vision with hsv recognition (position x-y-z)
            case 1: //fixed 3d position (x-y-z)
            case 2: //3d cube, 8 points (x-y-z)
                cout << "Adding stereo vision 3d attractor dynamics term......" << endl;
                calcState.push_back(calc_attractor_head_f1);
                calcState.push_back(calc_attractor_head_f2);
                calcState.push_back(calc_attractor_head_f3);
				break;
		}

		d_attractor.realState = realState;
		d_attractor.calcState = calcState;
		//d_attractor.partialDerivativesOn = true;
		//d_attractor.partialDerivatives = partialDerivatives;
		d_attractor.sign = -1; //fix for the absence of partial derivatives
		d_attractor.variance = head_config.var_d_attractor;
		d_attractor.logCalcState = false;
		feoHead->addNewDerivativeTerm(0, d_attractor);
	
	}
		
    /*if (config.on_prior)
	{
	
		//Fifth term: prior
		cout << "Adding prior term..." << endl;

		realState.clear();
		calcState.clear();
		d_prior.description = "prior";
		realState.push_back(internal_state_mu1);
		realState.push_back(internal_state_mu2);
		calcState.push_back(initial_prior_mu1);
		calcState.push_back(initial_prior_mu2);
		d_prior.realState = realState;
		d_prior.calcState = calcState;
		d_prior.sign = -1;
		d_prior.variance = config.var_d_prior;
		feoHead->addNewDerivativeTerm(0, d_prior);
	
	}*/
		
	cout << "Number of first derivative terms: " << feoHead->getNumDerivativeTerms(0) << endl;
	
	cout << "2) Second derivative terms..." << endl;

	if (head_config.on_derivative)
	{

		if (right_arm_config.on_attractor)
		{
	
			//First term: dynamics with attractor
			//cout << "Adding attractor dynamics term..." << endl;

			realState.clear();
			calcState.clear();
			partialDerivatives.clear();
			partialDerivativesRow.clear();
			dd_attractor.description = "dd_attractor";
			realState.push_back(internal_state_mu1ep);
			realState.push_back(internal_state_mu2ep);
            realState.push_back(internal_state_mu3ep);
			//attractor dynamics depends on type
			switch (right_arm_config.attractor_type)
			{
                case 0: //stereo vision with hsv recognition (position x-y-z)
                case 1: //fixed 3d position (x-y-z)
                case 2: //3d cube, 8 points (x-y-z)
                    cout << "Adding stereo vision 3d attractor dynamics term......" << endl;
                    calcState.push_back(calc_attractor_head_f1);
                    calcState.push_back(calc_attractor_head_f2);
                    calcState.push_back(calc_attractor_head_f3);
					break;
			}

			dd_attractor.realState = realState;
			dd_attractor.calcState = calcState;
			dd_attractor.sign = -1;
			dd_attractor.variance = head_config.var_dd_attractor;
			dd_attractor.logCalcState = false;
			feoHead->addNewDerivativeTerm(1, dd_attractor);
		}
	}
	
	cout << "Number of second derivative terms: " << feoHead->getNumDerivativeTerms(1) << endl;
	
	if (head_config.on_action){
	
		cout << "3) Action terms..." << endl;

		//First term: encoders
		cout << "Adding encoders term..." << endl;

		realState.clear();
		calcState.clear();
		partialDerivatives.clear();
		partialDerivativesRow.clear();
		a_encoders.description = "a_encoders";	
		realState.push_back(read_encoders_q1e_action);
		realState.push_back(read_encoders_q2e);
        realState.push_back(read_encoders_q3e);
		calcState.push_back(internal_state_mu1e);
		calcState.push_back(internal_state_mu2e);
        calcState.push_back(internal_state_mu3e);
		partialDerivativesRow.push_back(d_encoders_q1_a1);
		partialDerivativesRow.push_back(d_encoders_q2_a1);
        partialDerivativesRow.push_back(d_encoders_q3_a1);
		partialDerivatives.push_back(partialDerivativesRow);
		partialDerivativesRow.clear();
		partialDerivativesRow.push_back(d_encoders_q1_a2);
		partialDerivativesRow.push_back(d_encoders_q2_a2);
        partialDerivativesRow.push_back(d_encoders_q3_a2);
		partialDerivatives.push_back(partialDerivativesRow);
        partialDerivativesRow.clear();
        partialDerivativesRow.push_back(d_encoders_q3_a3);
        partialDerivativesRow.push_back(d_encoders_q3_a3);
        partialDerivativesRow.push_back(d_encoders_q3_a3);
        partialDerivatives.push_back(partialDerivativesRow);
		a_encoders.realState = realState;
		a_encoders.calcState = calcState;
		a_encoders.partialDerivativesOn = true;
		a_encoders.partialDerivatives = partialDerivatives;
		a_encoders.sign = -1; //negative sign
		a_encoders.variance = head_config.var_a_encoders;
		feoHead->addNewActionTerm(a_encoders);

		cout << "Number of action terms: " << feoHead->getNumActionTerms() << endl;

	}
	
	
	//Free-energy terms
	cout << "4) Free-energy terms..." << endl;

	//First term: encoders
	cout << "Adding encoders term..." << endl;

	realState.clear();
	calcState.clear();
	fe_encoders.description = "fe_encoders";	
	realState.push_back(read_encoders_q1e_action);
	realState.push_back(read_encoders_q2e);
    realState.push_back(read_encoders_q3e);
	calcState.push_back(internal_state_mu1e);
	calcState.push_back(internal_state_mu2e);
    calcState.push_back(internal_state_mu3e);
	fe_encoders.realState = realState;
	fe_encoders.calcState = calcState;
	fe_encoders.sign = -1;
	fe_encoders.variance = head_config.var_d_encoders;
	feoHead->addNewFreeEnergyTerm(fe_encoders);
	
	//Normal dynamics term is 0: mup = f(mu,ro) = v + Z
		
	if (right_arm_config.on_attractor)
	{
	
		//Fourth term: dynamics with attractor
		//cout << "Adding attractor dynamics term..." << endl;

		realState.clear();
		calcState.clear();
		fe_attractor.description = "fe_attractor";
		realState.push_back(internal_state_mu1ep);
		realState.push_back(internal_state_mu2ep);
        realState.push_back(internal_state_mu3ep);

		//attractor dynamics depends on type
		switch (right_arm_config.attractor_type)
		{
            case 0: //stereo vision with hsv recognition (position x-y-z)
            case 1: //fixed 3d position (x-y-z)
            case 2: //3d cube, 8 points (x-y-z)
                cout << "Adding stereo vision 3d attractor dynamics term..." << endl;
                calcState.push_back(calc_attractor_head_f1);
                calcState.push_back(calc_attractor_head_f2);
                calcState.push_back(calc_attractor_head_f3);
				break;
		}

		fe_attractor.realState = realState;
		fe_attractor.calcState = calcState;
		fe_attractor.sign = -1;
		fe_attractor.variance = head_config.var_d_attractor;
		feoHead->addNewFreeEnergyTerm(fe_attractor);
	
	}

	cout << "Number of free-energy terms: " << feoHead->getNumFreeEnergyTerms() << endl;

}

void feo_head_run()
{
	//Set action thread
	if (head_config.on_action){
		
		//Change to velocity control on selected joints
        iCubHeadCtrl->changeControlMode(0, velocity); //neck_pitch
        iCubHeadCtrl->changeControlMode(2, velocity); //neck_yaw
        iCubHeadCtrl->changeControlMode(3, velocity); //eyes_tilt
	
		//Activate action thread	
		actionHeadThread = new YarpPeriodic(0.5*dT*1000, "action_head");
		actionHeadThread->setRun(set_action_head_thread);
		actionHeadThread->start();
	}
	
	//Get internal state
	state_head = feoHead->getInternalState();
	
	
	
	cout << "Current head state:";
	for (unsigned int i = 0; i < state_head.size(); i++)
	{
		cout << " " << state_head[i];
	}
	cout << endl;
	
	//Run thread
	feoHeadThread = new YarpPeriodic(dT*1000, "free-energy_head");
	feoHeadThread->setRun(calculate_feo_head);
	feoHeadThread->start();

}

//zero condition function for head
bool zero_condition_head(double action, int dof)
{
    //touching condition
    if (right_arm_config.on_touch && right_arm_config.activated) if (touching) return true;
    if (left_arm_config.on_touch && left_arm_config.activated) if (touchingLeft) return true;
	
	switch (dof)
	{
        case 0: //neck_pitch
            if (qLEencoders[0]*CTRL_RAD2DEG > neck_pitch_max && action > 0) return true; //-40 -> 30
            if (qLEencoders[0]*CTRL_RAD2DEG < neck_pitch_min && action < 0) return true; //-40 -> 30
			break;
        case 1: //neck_yaw
            if (qLEencoders[2]*CTRL_RAD2DEG > neck_yaw_max && action > 0) return true; //-55 -> 55
            if (qLEencoders[2]*CTRL_RAD2DEG < neck_yaw_min && action < 0) return true; //-55 -> 55
			break;
        case 2: //eyes_tilt
            if (qLEencoders[3]*CTRL_RAD2DEG > eyes_tilt_max && action > 0) return true; //-30 -> 30
            if (qLEencoders[3]*CTRL_RAD2DEG < eyes_tilt_min && action < 0) return true; //-30 -> 30
            break;
	}

	return false;
}

//free energy optimization algorithm for left arm
void feo_left_arm_setup()
{

    //Derivative terms
    DerivativeTerm d_encoders, d_prior, d_pos3D, d_vision, d_attractor, dd_attractor;
    fvector realState, calcState, partialDerivativesRow;
    fmatrix partialDerivatives;
    //Action terms
    ActionTerm a_encoders, a_pos3D, a_vision;
    //Free-energy terms
    FreeEnergyTerm fe_encoders, fe_pos3D, fe_vision, fe_attractor;

    feoLeftArm = new FreeEnergyOptimization();

    cout << "* Starting left arm setup..." << endl;

    //Obtaining initial state
    mu0.clear();
    a.clear(); a.push_back(0); a.push_back(0); a.push_back(0); a.push_back(0);
    if (left_arm_config.null_start)
    {
        mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); //zero position
    }else{
        mu0.push_back(read_encoders_q1b(mu0, a)); mu0.push_back(read_encoders_q2b(mu0, a));
		mu0.push_back(read_encoders_q3b(mu0, a)); mu0.push_back(read_encoders_q4b(mu0, a)); //current position
    }
    mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); mu0.push_back(0); //velocity
    initialPriorb.push_back(mu0[0]); initialPriorb.push_back(mu0[1]); initialPriorb.push_back(mu0[2]); initialPriorb.push_back(mu0[3]);

    //Set up parameters
    IntegrationMethod intMethod = Euler;
    //if (config.integration_method == "LocalLinearization") intMethod = LocalLinear;
    vector<double> gainPerception;
    gainPerception.push_back(left_arm_config.gain_perception_mu1); gainPerception.push_back(left_arm_config.gain_perception_mu2);
	gainPerception.push_back(left_arm_config.gain_perception_mu3); gainPerception.push_back(left_arm_config.gain_perception_mu4);
    gainPerception.push_back(left_arm_config.gain_derivative_mu1p); gainPerception.push_back(left_arm_config.gain_derivative_mu2p);
	gainPerception.push_back(left_arm_config.gain_derivative_mu3p); gainPerception.push_back(left_arm_config.gain_derivative_mu4p);
    vector<double> weightedPerception;
    weightedPerception.push_back(left_arm_config.weighted_perception_mu1); weightedPerception.push_back(left_arm_config.weighted_perception_mu2); 
	weightedPerception.push_back(left_arm_config.weighted_perception_mu3); weightedPerception.push_back(left_arm_config.weighted_perception_mu4);
    vector<double> gainAction;
    gainAction.push_back(left_arm_config.gain_action_q1); gainAction.push_back(left_arm_config.gain_action_q2); 
	gainAction.push_back(left_arm_config.gain_action_q3); gainAction.push_back(left_arm_config.gain_action_q4);
    feoLeftArm->setParameters("left_arm", 4, 2, mu0, main_config.dT, left_arm_config.on_action, gainPerception, weightedPerception, gainAction, intMethod, main_config.debug); //4 DOF and 2 derivatives calculated
    if (main_config.log) //logging
    {
        feoLeftArm->setStartLoggingFunction(log_config);
        feoLeftArm->setEndLoggingFunction(log_stats);
    }
    if (left_arm_config.perception_saturation_on) //perception saturation
    {
        vector<double> saturationLow;
        saturationLow.push_back(left_arm_config.mu1_saturation_low*CTRL_DEG2RAD);
        saturationLow.push_back(left_arm_config.mu2_saturation_low*CTRL_DEG2RAD);
        saturationLow.push_back(left_arm_config.mu3_saturation_low*CTRL_DEG2RAD);
        saturationLow.push_back(left_arm_config.mu4_saturation_low*CTRL_DEG2RAD);
        vector<double> saturationHigh;
        saturationHigh.push_back(left_arm_config.mu1_saturation_high*CTRL_DEG2RAD);
        saturationHigh.push_back(left_arm_config.mu2_saturation_high*CTRL_DEG2RAD);
        saturationHigh.push_back(left_arm_config.mu3_saturation_high*CTRL_DEG2RAD);
        saturationHigh.push_back(left_arm_config.mu4_saturation_high*CTRL_DEG2RAD);
        feoLeftArm->setPerceptionSaturation(left_arm_config.perception_saturation_on, saturationLow, saturationHigh);
    }
    if (left_arm_config.derivative_saturation_on) //derivative saturation
    {
        vector<double> saturationLowD;
        saturationLowD.push_back(left_arm_config.mu1p_saturation_low*CTRL_DEG2RAD);
        saturationLowD.push_back(left_arm_config.mu2p_saturation_low*CTRL_DEG2RAD);
        saturationLowD.push_back(left_arm_config.mu3p_saturation_low*CTRL_DEG2RAD);
        saturationLowD.push_back(left_arm_config.mu4p_saturation_low*CTRL_DEG2RAD);
        vector<double> saturationHighD;
        saturationHighD.push_back(left_arm_config.mu1p_saturation_high*CTRL_DEG2RAD);
        saturationHighD.push_back(left_arm_config.mu2p_saturation_high*CTRL_DEG2RAD);
        saturationHighD.push_back(left_arm_config.mu3p_saturation_high*CTRL_DEG2RAD);
        saturationHighD.push_back(left_arm_config.mu4p_saturation_high*CTRL_DEG2RAD);
        feoLeftArm->setDerivativeSaturation(left_arm_config.derivative_saturation_on, saturationLowD, saturationHighD);
    }
    if (left_arm_config.action_saturation_on) //action saturation
    {
        feoLeftArm->setActionSaturation(left_arm_config.action_saturation_on, left_arm_config.action_saturation_low*CTRL_DEG2RAD, left_arm_config.action_saturation_high*CTRL_DEG2RAD);
    }
    //set action to zero when touching or reached limit positions
    feoLeftArm->setZeroConditionFunction(zero_condition_left_arm);

    //Logging
    string log_name = "logs/log_active_inference_left_arm_" + time_str + "_" + main_config.description;
    if (main_config.log) feoLeftArm->startLogging(log_name);

    cout << "1) First derivative terms..." << endl;

    if (left_arm_config.on_encoders)
    {

        //First term: encoders
        cout << "Adding encoders term..." << endl;

        realState.clear();
        calcState.clear();
        d_encoders.description = "d_encoders";
        if (left_arm_config.gaussian_noise)
        {
            realState.push_back(read_encoders_q1b_noise);
            realState.push_back(read_encoders_q2b_noise);
            realState.push_back(read_encoders_q3b_noise);
            realState.push_back(read_encoders_q4b_noise);
        }
        else
        {
            realState.push_back(read_encoders_q1b);
            realState.push_back(read_encoders_q2b);
            realState.push_back(read_encoders_q3b);
            realState.push_back(read_encoders_q4b);
        }
        calcState.push_back(internal_state_mu1b);
        calcState.push_back(internal_state_mu2b);
        calcState.push_back(internal_state_mu3b);
        calcState.push_back(internal_state_mu4b);
        d_encoders.realState = realState;
        d_encoders.calcState = calcState;
        d_encoders.variance = left_arm_config.var_d_encoders;
        d_encoders.logRealState = true;
        feoLeftArm->addNewDerivativeTerm(0, d_encoders);

    }

    if (left_arm_config.on_3d)
    {

        //Second term: 3d position
        cout << "Adding 3d position term..." << endl;

        realState.clear();
        calcState.clear();
        partialDerivatives.clear();
        partialDerivativesRow.clear();
        d_pos3D.description = "d_pos3D";
        realState.push_back(sense_3d_position_x_b);
        realState.push_back(sense_3d_position_y_b);
        realState.push_back(sense_3d_position_z_b);
        calcState.push_back(calc_3d_position_x_b);
        calcState.push_back(calc_3d_position_y_b);
        calcState.push_back(calc_3d_position_z_b);
        partialDerivativesRow.push_back(d_3d_position_x_mu1_b);
        partialDerivativesRow.push_back(d_3d_position_y_mu1_b);
        partialDerivativesRow.push_back(d_3d_position_z_mu1_b);
        partialDerivatives.push_back(partialDerivativesRow);
        partialDerivativesRow.clear();
        partialDerivativesRow.push_back(d_3d_position_x_mu2_b);
        partialDerivativesRow.push_back(d_3d_position_y_mu2_b);
        partialDerivativesRow.push_back(d_3d_position_z_mu2_b);
        partialDerivatives.push_back(partialDerivativesRow);
        partialDerivativesRow.clear();
        partialDerivativesRow.push_back(d_3d_position_x_mu3_b);
        partialDerivativesRow.push_back(d_3d_position_y_mu3_b);
        partialDerivativesRow.push_back(d_3d_position_z_mu3_b);
        partialDerivatives.push_back(partialDerivativesRow);
		partialDerivativesRow.clear();
        partialDerivativesRow.push_back(d_3d_position_x_mu4_b);
        partialDerivativesRow.push_back(d_3d_position_y_mu4_b);
        partialDerivativesRow.push_back(d_3d_position_z_mu4_b);
        partialDerivatives.push_back(partialDerivativesRow);
        d_pos3D.realState = realState;
        d_pos3D.calcState = calcState;
        d_pos3D.partialDerivativesOn = true;
        d_pos3D.partialDerivatives = partialDerivatives;
        d_pos3D.variance = left_arm_config.var_d_3d;
        feoLeftArm->addNewDerivativeTerm(0, d_pos3D);

    }

    //Normal dynamics term is 0: mup = f(mu,ro) = v + Z

    if (left_arm_config.on_attractor)
    {

        //Fourth term: dynamics with attractor
        //cout << "Adding attractor dynamics term..." << endl;

        realState.clear();
        calcState.clear();
        partialDerivatives.clear();
        partialDerivativesRow.clear();
        d_attractor.description = "d_attractor";
        realState.push_back(internal_state_mu1bp);
        realState.push_back(internal_state_mu2bp);
        realState.push_back(internal_state_mu3bp);
        realState.push_back(internal_state_mu4bp);

        //attractor dynamics depends on type
		switch (left_arm_config.attractor_type)
		{
			case 0: //stereo vision with hsv recognition (position x-y-z)
				cout << "Adding stereo vision 3d attractor dynamics term..." << endl;
				calcState.push_back(calc_attractor_stereo_f1_b);
				calcState.push_back(calc_attractor_stereo_f2_b);
				calcState.push_back(calc_attractor_stereo_f3_b);
				calcState.push_back(calc_attractor_stereo_f4_b);
				//partialDerivativesRow.push_back(d_attractor_3d_f1_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f2_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f3_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f4_mu1);
				//partialDerivatives.push_back(partialDerivativesRow);
				//partialDerivativesRow.clear();
				//partialDerivativesRow.push_back(d_attractor_3d_f1_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f2_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f3_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f4_mu2);
				//partialDerivatives.push_back(partialDerivativesRow);
				break;
            /*case 1: //3d fixed position (x-y-z)
                cout << "Adding fixed 3d attractor dynamics term..." << endl;
                calcState.push_back(calc_attractor_stereo_f1);
                calcState.push_back(calc_attractor_stereo_f2);
                calcState.push_back(calc_attractor_stereo_f3);
                calcState.push_back(calc_attractor_stereo_f4);
                break;*/
		}

        d_attractor.realState = realState;
        d_attractor.calcState = calcState;
        //d_attractor.partialDerivativesOn = true;
        //d_attractor.partialDerivatives = partialDerivatives;
        d_attractor.sign = -1; //fix for the absence of partial derivatives
        d_attractor.variance = left_arm_config.var_d_attractor;
        d_attractor.logCalcState = false;
        feoLeftArm->addNewDerivativeTerm(0, d_attractor);

    }

    if (left_arm_config.on_prior)
    {

        //Fifth term: prior
        cout << "Adding prior term..." << endl;

        realState.clear();
        calcState.clear();
        d_prior.description = "prior";
        realState.push_back(internal_state_mu1b);
        realState.push_back(internal_state_mu2b);
        realState.push_back(internal_state_mu3b);
        realState.push_back(internal_state_mu4b);
        calcState.push_back(initial_prior_mu1b);
        calcState.push_back(initial_prior_mu2b);
        calcState.push_back(initial_prior_mu3b);
        calcState.push_back(initial_prior_mu4b);
        d_prior.realState = realState;
        d_prior.calcState = calcState;
        d_prior.sign = -1;
        d_prior.variance = left_arm_config.var_d_prior;
        feoLeftArm->addNewDerivativeTerm(0, d_prior);

    }

    cout << "Number of first derivative terms: " << feoLeftArm->getNumDerivativeTerms(0) << endl;

    cout << "2) Second derivative terms..." << endl;

    if (left_arm_config.on_derivative)
    {

        if (left_arm_config.on_attractor)
        {

            //First term: dynamics with attractor
            //cout << "Adding attractor dynamics term..." << endl;

            realState.clear();
            calcState.clear();
            partialDerivatives.clear();
            partialDerivativesRow.clear();
            dd_attractor.description = "dd_attractor";
            realState.push_back(internal_state_mu1bp);
            realState.push_back(internal_state_mu2bp);
            realState.push_back(internal_state_mu3bp);
            realState.push_back(internal_state_mu4bp);

            //attractor dynamics depends on type
			switch (left_arm_config.attractor_type)
			{
				case 0: //stereo vision with hsv recognition (position x-y-z)
					cout << "Adding stereo vision 3d attractor dynamics term..." << endl;
					calcState.push_back(calc_attractor_stereo_f1_b);
					calcState.push_back(calc_attractor_stereo_f2_b);
					calcState.push_back(calc_attractor_stereo_f3_b);
					calcState.push_back(calc_attractor_stereo_f4_b);
					//partialDerivativesRow.push_back(d_attractor_3d_f1_mu1);
					//partialDerivativesRow.push_back(d_attractor_3d_f2_mu1);
					//partialDerivativesRow.push_back(d_attractor_3d_f3_mu1);
					//partialDerivativesRow.push_back(d_attractor_3d_f4_mu1);
					//partialDerivatives.push_back(partialDerivativesRow);
					//partialDerivativesRow.clear();
					//partialDerivativesRow.push_back(d_attractor_3d_f1_mu2);
					//partialDerivativesRow.push_back(d_attractor_3d_f2_mu2);
					//partialDerivativesRow.push_back(d_attractor_3d_f3_mu2);
					//partialDerivativesRow.push_back(d_attractor_3d_f4_mu2);
					//partialDerivatives.push_back(partialDerivativesRow);
					break;
                /*case 1: //3d fixed position (x-y-z)
                    cout << "Adding fixed 3d attractor dynamics term..." << endl;
                    calcState.push_back(calc_attractor_stereo_f1);
                    calcState.push_back(calc_attractor_stereo_f2);
                    calcState.push_back(calc_attractor_stereo_f3);
                    calcState.push_back(calc_attractor_stereo_f4);
                    break;*/
			}

            dd_attractor.realState = realState;
            dd_attractor.calcState = calcState;
            dd_attractor.sign = -1;
            dd_attractor.variance = left_arm_config.var_dd_attractor;
            dd_attractor.logCalcState = false;
            feoLeftArm->addNewDerivativeTerm(1, dd_attractor);
        }
    }

    cout << "Number of second derivative terms: " << feoLeftArm->getNumDerivativeTerms(1) << endl;

    if (left_arm_config.on_action){

        cout << "3) Action terms..." << endl;

        if (left_arm_config.on_encoders)
        {

            //First term: encoders
            cout << "Adding encoders term..." << endl;

            realState.clear();
            calcState.clear();
            partialDerivatives.clear();
            partialDerivativesRow.clear();
            a_encoders.description = "a_encoders";
            if (left_arm_config.gaussian_noise)
            {
                realState.push_back(read_encoders_q1b_noise);
                realState.push_back(read_encoders_q2b_noise);
                realState.push_back(read_encoders_q3b_noise);
                realState.push_back(read_encoders_q4b_noise);
            }
            else
            {
                realState.push_back(read_encoders_q1b_action);
                realState.push_back(read_encoders_q2b);
                realState.push_back(read_encoders_q3b);
                realState.push_back(read_encoders_q4b);
            }
            calcState.push_back(internal_state_mu1b);
            calcState.push_back(internal_state_mu2b);
            calcState.push_back(internal_state_mu3b);
            calcState.push_back(internal_state_mu4b);
            partialDerivativesRow.push_back(d_encoders_q1_a1);
            partialDerivativesRow.push_back(d_encoders_q2_a1);
            partialDerivativesRow.push_back(d_encoders_q3_a1);
            partialDerivativesRow.push_back(d_encoders_q4_a1);
            partialDerivatives.push_back(partialDerivativesRow);
            partialDerivativesRow.clear();
            partialDerivativesRow.push_back(d_encoders_q1_a2);
            partialDerivativesRow.push_back(d_encoders_q2_a2);
            partialDerivativesRow.push_back(d_encoders_q3_a2);
            partialDerivativesRow.push_back(d_encoders_q4_a2);
            partialDerivatives.push_back(partialDerivativesRow);
            partialDerivativesRow.clear();
            partialDerivativesRow.push_back(d_encoders_q1_a3);
            partialDerivativesRow.push_back(d_encoders_q2_a3);
            partialDerivativesRow.push_back(d_encoders_q3_a3);
            partialDerivativesRow.push_back(d_encoders_q4_a3);
            partialDerivatives.push_back(partialDerivativesRow);
			partialDerivativesRow.clear();
            partialDerivativesRow.push_back(d_encoders_q1_a4);
            partialDerivativesRow.push_back(d_encoders_q2_a4);
            partialDerivativesRow.push_back(d_encoders_q3_a4);
            partialDerivativesRow.push_back(d_encoders_q4_a4);
            partialDerivatives.push_back(partialDerivativesRow);
            a_encoders.realState = realState;
            a_encoders.calcState = calcState;
            a_encoders.partialDerivativesOn = true;
            a_encoders.partialDerivatives = partialDerivatives;
            a_encoders.sign = -1; //negative sign
            a_encoders.variance = left_arm_config.var_a_encoders;
            feoLeftArm->addNewActionTerm(a_encoders);

        }

		if (left_arm_config.on_3d)
		{

			//Second term: 3d position
			cout << "Adding 3d position term..." << endl;

			realState.clear();
			calcState.clear();
			partialDerivatives.clear();
			partialDerivativesRow.clear();
			a_pos3D.description = "a_pos3D";
			realState.push_back(sense_3d_position_x_action_b);
			realState.push_back(sense_3d_position_y_b);
			realState.push_back(sense_3d_position_z_b);
			calcState.push_back(calc_3d_position_x_b);
			calcState.push_back(calc_3d_position_y_b);
			calcState.push_back(calc_3d_position_z_b);
			partialDerivativesRow.push_back(d_3d_position_x_a1_b);
			partialDerivativesRow.push_back(d_3d_position_y_a1_b);
			partialDerivativesRow.push_back(d_3d_position_z_a1_b);
			partialDerivatives.push_back(partialDerivativesRow);
			partialDerivativesRow.clear();
			partialDerivativesRow.push_back(d_3d_position_x_a2_b);
			partialDerivativesRow.push_back(d_3d_position_y_a2_b);
			partialDerivativesRow.push_back(d_3d_position_z_a2_b);
			partialDerivatives.push_back(partialDerivativesRow);
			partialDerivativesRow.clear();
			partialDerivativesRow.push_back(d_3d_position_x_a3_b);
			partialDerivativesRow.push_back(d_3d_position_y_a3_b);
			partialDerivativesRow.push_back(d_3d_position_z_a3_b);
			partialDerivatives.push_back(partialDerivativesRow);
			partialDerivativesRow.clear();
			partialDerivativesRow.push_back(d_3d_position_x_a4_b);
			partialDerivativesRow.push_back(d_3d_position_y_a4_b);
			partialDerivativesRow.push_back(d_3d_position_z_a4_b);
			partialDerivatives.push_back(partialDerivativesRow);
			a_pos3D.realState = realState;
			a_pos3D.calcState = calcState;
			a_pos3D.partialDerivativesOn = true;
			a_pos3D.partialDerivatives = partialDerivatives;
			a_pos3D.sign = -1; //negative sign
			a_pos3D.variance = right_arm_config.var_a_3d;
			//a_pos3D.deactivateNull = true; //do not consider if sensed position is 0
			feoRightArm->addNewActionTerm(a_pos3D);

        } 

        cout << "Number of action terms: " << feoLeftArm->getNumActionTerms() << endl;

    }


    //Free-energy terms
    cout << "4) Free-energy terms..." << endl;

    if (left_arm_config.on_encoders)
    {

        //First term: encoders
        cout << "Adding encoders term..." << endl;

        realState.clear();
        calcState.clear();
        fe_encoders.description = "fe_encoders";
        if (left_arm_config.gaussian_noise)
        {
            realState.push_back(read_encoders_q1b_noise);
            realState.push_back(read_encoders_q2b_noise);
            realState.push_back(read_encoders_q3b_noise);
            realState.push_back(read_encoders_q4b_noise);
        }
        else
        {
            realState.push_back(read_encoders_q1b_action);
            realState.push_back(read_encoders_q2b);
            realState.push_back(read_encoders_q3b);
            realState.push_back(read_encoders_q4b);
        }
        calcState.push_back(internal_state_mu1b);
        calcState.push_back(internal_state_mu2b);
        calcState.push_back(internal_state_mu3b);
        calcState.push_back(internal_state_mu4b);
        fe_encoders.realState = realState;
        fe_encoders.calcState = calcState;
        fe_encoders.sign = -1;
        fe_encoders.variance = left_arm_config.var_d_encoders;
        feoLeftArm->addNewFreeEnergyTerm(fe_encoders);

    }

    if (left_arm_config.on_3d)
    {

        //Second term: 3d position
        cout << "Adding 3d position term..." << endl;

        realState.clear();
        calcState.clear();
        fe_pos3D.description = "fe_pos3D";
        realState.push_back(sense_3d_position_x_action_b);
        realState.push_back(sense_3d_position_y_b);
        realState.push_back(sense_3d_position_z_b);
        calcState.push_back(calc_3d_position_x_b);
        calcState.push_back(calc_3d_position_y_b);
        calcState.push_back(calc_3d_position_z_b);
        fe_pos3D.realState = realState;
        fe_pos3D.calcState = calcState;
        fe_pos3D.sign = -1;
        fe_pos3D.variance = right_arm_config.var_d_3d;
        feoLeftArm->addNewFreeEnergyTerm(fe_pos3D);

    }

    //Normal dynamics term is 0: mup = f(mu,ro) = v + Z

    if (left_arm_config.on_attractor)
    {

        //Fourth term: dynamics with attractor
        //cout << "Adding attractor dynamics term..." << endl;

        realState.clear();
        calcState.clear();
        fe_attractor.description = "fe_attractor";
        realState.push_back(internal_state_mu1bp);
        realState.push_back(internal_state_mu2bp);
        realState.push_back(internal_state_mu3bp);
        realState.push_back(internal_state_mu4bp);

        //attractor dynamics depends on type
		switch (left_arm_config.attractor_type)
		{
			case 0: //stereo vision with hsv recognition (position x-y-z)
				cout << "Adding stereo vision 3d attractor dynamics term..." << endl;
				calcState.push_back(calc_attractor_stereo_f1_b);
				calcState.push_back(calc_attractor_stereo_f2_b);
				calcState.push_back(calc_attractor_stereo_f3_b);
				calcState.push_back(calc_attractor_stereo_f4_b);
				//partialDerivativesRow.push_back(d_attractor_3d_f1_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f2_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f3_mu1);
				//partialDerivativesRow.push_back(d_attractor_3d_f4_mu1);
				//partialDerivatives.push_back(partialDerivativesRow);
				//partialDerivativesRow.clear();
				//partialDerivativesRow.push_back(d_attractor_3d_f1_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f2_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f3_mu2);
				//partialDerivativesRow.push_back(d_attractor_3d_f4_mu2);
				//partialDerivatives.push_back(partialDerivativesRow);
				break;
            /*case 1: //3d fixed position (x-y-z)
                cout << "Adding fixed 3d attractor dynamics term..." << endl;
                calcState.push_back(calc_attractor_stereo_f1);
                calcState.push_back(calc_attractor_stereo_f2);
                calcState.push_back(calc_attractor_stereo_f3);
                calcState.push_back(calc_attractor_stereo_f4);
                break;*/
		}

        fe_attractor.realState = realState;
        fe_attractor.calcState = calcState;
        fe_attractor.sign = -1;
        fe_attractor.variance = left_arm_config.var_d_attractor;
        feoLeftArm->addNewFreeEnergyTerm(fe_attractor);

    }

    cout << "Number of free-energy terms: " << feoLeftArm->getNumFreeEnergyTerms() << endl;

}

void feo_left_arm_run()
{
    //Set action thread
    if (left_arm_config.on_action){

        //Change to velocity control on selected joints
		iCubTorsoCtrl->changeControlMode(0, velocity);
        iCubLeftArmCtrl->changeControlMode(1, velocity);
        iCubLeftArmCtrl->changeControlMode(2, velocity);
        iCubLeftArmCtrl->changeControlMode(3, velocity);

        //Activate action thread
        actionLeftArmThread = new YarpPeriodic(0.5*dT*1000, "action_left_arm");
        actionLeftArmThread->setRun(set_action_left_arm_thread);
        actionLeftArmThread->start();
    }

    //Get internal state
    state_b = feoLeftArm->getInternalState();

    cout << "Current left arm state:";
    for (unsigned int i = 0; i < state_b.size(); i++)
    {
        cout << " " << state_b[i];
    }
    cout << endl;

    //Run thread
    feoLeftArmThread = new YarpPeriodic(dT*1000, "free-energy_left_arm");
    feoLeftArmThread->setRun(calculate_feo_left_arm);
    feoLeftArmThread->start();

}

//zero condition function for left arm
bool zero_condition_left_arm(double action, int dof)
{
    //touching condition
    if (left_arm_config.on_touch) if (touchingLeft) return true;
    if (right_arm_config.on_touch && right_arm_config.activated) if (touching) return true;

    switch (dof)
    {
		case 0: //torso_yaw
			//cout << qTencoders[0]*CTRL_RAD2DEG << ", " << action << endl;
            if (qTencoders[0]*CTRL_RAD2DEG > torso_yaw_max && action > 0) return true; //-50 -> 50
            if (qTencoders[0]*CTRL_RAD2DEG < torso_yaw_min && action < 0) return true; //-50 -> 50
			break;
        case 1: //l_shoulder_roll
            //cout << qLAencoders[1]*CTRL_RAD2DEG << ", " << action << endl;
            if (qLAencoders[1]*CTRL_RAD2DEG > shoulder_roll_max && action > 0) return true; //0 -> 160.8
            if (qLAencoders[1]*CTRL_RAD2DEG < shoulder_roll_min && action < 0) return true; //0 -> 160.8
            break;
        case 2: //l_shoulder_yaw
            //cout << qLAencoders[2]*CTRL_RAD2DEG << ", " << action << endl;
            if (qLAencoders[2]*CTRL_RAD2DEG > shoulder_yaw_max && action > 0) return true; //-37 -> 100
            if (qLAencoders[2]*CTRL_RAD2DEG < shoulder_yaw_min && action < 0) return true; //-37 -> 100
            break;
        case 3: //l_elbow
            //cout << qLAencoders[3]*CTRL_RAD2DEG << ", " << action << endl;
            if (qLAencoders[3]*CTRL_RAD2DEG > elbow_max && action > 0) return true; //5.5 -> 106
            if (qLAencoders[3]*CTRL_RAD2DEG < elbow_min && action < 0) return true; //5.5 -> 106
            break;
    }

    return false;
}


//----------------------------------------------------------------------------------
//Threads --------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//vision
void sense_vision_thread()
{
    //right arm visual to 3D
    if (right_arm_config.activated && (right_arm_config.on_vision || right_arm_config.on_attractor))
	{
		//read image and find location of end-effector
		Scalar left_col_low = Scalar(left_hue_low, left_sat_low, left_val_low);
		Scalar left_col_high = Scalar(left_hue_high, left_sat_high, left_val_high);
		Scalar right_col_low = Scalar(right_hue_low, right_sat_low, right_val_low);
		Scalar right_col_high = Scalar(right_hue_high, right_sat_high, right_val_high);
        //re_pos = iCubImgL->findLocationCV(L, col_low, col_high, right_arm_config.erode_dilate_iterations, false);
        re_pos = iCubImgRL->get3Dfrom2D(RL, left_col_low, left_col_high, right_arm_config.left_erode_dilate_iterations,
                            right_col_low, right_col_high, right_arm_config.right_erode_dilate_iterations, 3, false);
        //cout << "3D position: " << re_pos(0) << " " << re_pos(1) << " " << re_pos(2) << endl;

        //calculate position in image
        calc_vision_l_u(state, state);
        calc_vision_l_v(state, state);
        calc_vision_r_u(state, state);
        calc_vision_r_v(state, state);

        //publish right arm values
        if (main_config.publish)
        {
            //3D position: x, y, z
            Bottle& output_x = plot_x.prepare();
            output_x.clear();
            output_x.addDouble(sense_3d_position_x(state, state)); //x
            plot_x.write();
            Bottle& output_y = plot_y.prepare();
            output_y.clear();
            output_y.addDouble(sense_3d_position_y(state, state)); //y
            plot_y.write();
            Bottle& output_z = plot_z.prepare();
            output_z.clear();
            output_z.addDouble(sense_3d_position_z(state, state)); //z
            plot_z.write();
            //left eye: u, v, size
            Bottle& output_l_u = plot_l_u.prepare();
            output_l_u.clear();
            output_l_u.addDouble(re_pos(3)); //u
            plot_l_u.write();
            Bottle& output_l_v = plot_l_v.prepare();
            output_l_v.clear();
            output_l_v.addDouble(re_pos(4)); //v
            plot_l_v.write();
            Bottle& output_l_size = plot_l_size.prepare();
            output_l_size.clear();
            output_l_size.addDouble(re_pos(5)); //size
            plot_l_size.write();
            //right eye: u, v, size
            Bottle& output_r_u = plot_r_u.prepare();
            output_r_u.clear();
            output_r_u.addDouble(re_pos(6)); //u
            plot_r_u.write();
            Bottle& output_r_v = plot_r_v.prepare();
            output_r_v.clear();
            output_r_v.addDouble(re_pos(7)); //v
            plot_r_v.write();
            Bottle& output_r_size = plot_r_size.prepare();
            output_r_size.clear();
            output_r_size.addDouble(re_pos(8)); //size
            plot_r_size.write();
            //calculated left eye: u, v
            Bottle& output_calc_l_u = plot_calc_l_u.prepare();
            output_calc_l_u.clear();
            output_calc_l_u.addDouble(calc_vision_pos_l(0)); //u
            plot_calc_l_u.write();
            Bottle& output_calc_l_v = plot_calc_l_v.prepare();
            output_calc_l_v.clear();
            output_calc_l_v.addDouble(calc_vision_pos_l(1)); //v
            plot_calc_l_v.write();
            //calculated right eye: u, v
            Bottle& output_calc_r_u = plot_calc_r_u.prepare();
            output_calc_r_u.clear();
            output_calc_r_u.addDouble(calc_vision_pos_r(0)); //u
            plot_calc_r_u.write();
            Bottle& output_calc_r_v = plot_calc_r_v.prepare();
            output_calc_r_v.clear();
            output_calc_r_v.addDouble(calc_vision_pos_r(1)); //v
            plot_calc_r_v.write();
        }

	}

    //attractor visual to 3D
    if (right_arm_config.on_attractor || left_arm_config.on_attractor)
    {

        switch (right_arm_config.attractor_type)
        {
            case 0: //stereo vision with hsv recognition (position x-y-z)
                if (right_arm_config.fixed_attractor)
                {
                    //read the first time, att_pos constant

                }else{
                    //read image and find location of attractor
                    Scalar left_att_col_low = Scalar(left_att_hue_low, left_att_sat_low, left_att_val_low);
                    Scalar left_att_col_high = Scalar(left_att_hue_high, left_att_sat_high, left_att_val_high);
                    Scalar right_att_col_low = Scalar(right_att_hue_low, right_att_sat_low, right_att_val_low);
                    Scalar right_att_col_high = Scalar(right_att_hue_high, right_att_sat_high, right_att_val_high);
                    //att_pos = iCubImgLA->findLocationCV(L, att_col_low, att_col_high, right_arm_config.erode_dilate_iterations, false);
                    att_pos = iCubImgRLA->get3Dfrom2D(RL, left_att_col_low, left_att_col_high, right_arm_config.left_erode_dilate_iterations,
										right_att_col_low, right_att_col_high, right_arm_config.right_erode_dilate_iterations, 3, false);

                    if (att_pos(0) == 0 && att_pos(1) == 0) //attractor out of sight
                    {
                        att_pos = prev_att_pos;
                    }

                    prev_att_pos = att_pos;

                    //cout << "3D position: " <<  att_pos(0) << " " << att_pos(1) << " " << att_pos(2) << endl;
					//cout << "Visual position: " <<  att_pos(3) << " " << att_pos(4) << " " << att_pos(5) << endl;
                }
                attr_3d_pos(0) = sense_3d_attractor_x(state, state); attr_3d_pos(1) = sense_3d_attractor_y(state, state); attr_3d_pos(2) = sense_3d_attractor_z(state, state);
                break;
            case 1: //3d fixed position (x-y-z)
                attr_3d_pos(0) = ro2; attr_3d_pos(1) = ro3; attr_3d_pos(2) = ro4;
                att_pos[0] = 0; att_pos[1] = 0; att_pos[2] = 0; //left eye relative position
                att_pos[3] = calc_attractor_l_u(); att_pos[4] = calc_attractor_l_v(); att_pos[5] = 0; //left eye camera position
                att_pos[6] = calc_attractor_r_u(); att_pos[7] = calc_attractor_r_v(); att_pos[8] = 0; //right eye camera position
                break;
            case 2: //3d cube (cycle)
                yarp::sig::Vector target;
                target.resize(3);
                //select attractor comparison vector
                switch (right_arm_config.attractor_comparison)
                {
                    case 0: //calculated
                        target(0) = calc_3d_position_x(state, state);
                        target(1) = calc_3d_position_y(state, state);
                        target(2) = calc_3d_position_z(state, state);
                        break;
                    case 1: //visual
                        target(0) = re_pos(0);
                        target(1) = re_pos(1);
                        target(2) = re_pos(2);
                        break;
                }

                //compare visual/calculated position to attractor
                //cout << "Attractor comparison : (" << target(0) - attr_3d_pos(0) << ", " << target(1) - attr_3d_pos(1) << ", " << target(2) - attr_3d_pos(2) << ")" << endl;
                if ( (abs(target(0) - attr_3d_pos(0)) <= right_arm_config.attractor_threshold) &&
                     (abs(target(1) - attr_3d_pos(1)) <= right_arm_config.attractor_threshold) &&
                     (abs(target(2) - attr_3d_pos(2)) <= right_arm_config.attractor_threshold) )
                {
                    switch (att_state)
                    {
                        case 0: //next: point 2
                            cout << "Cube point 1 reached" << endl;
                            attr_3d_pos(0) = ro5;
                            attr_3d_pos(1) = ro6;
							attr_3d_pos(2) = ro7;
                            att_state = 1;
                            cout << "Next point (2): (" << ro5 << ", " << ro6 << ", " << ro7 << ")" << endl;
                            break;
						case 1: //next: point 3
                            cout << "Cube point 2 reached" << endl;
                            attr_3d_pos(0) = ro8;
                            attr_3d_pos(1) = ro9;
							attr_3d_pos(2) = ro10;
                            att_state = 2;
                            cout << "Next point (3): (" << ro8 << ", " << ro9 << ", " << ro10 << ")" << endl;
                            break;
						case 2: //next: point 4
                            cout << "Cube point 3 reached" << endl;
                            attr_3d_pos(0) = ro11;
                            attr_3d_pos(1) = ro12;
							attr_3d_pos(2) = ro13;
                            att_state = 3;
                            cout << "Next point (4): (" << ro11 << ", " << ro12 << ", " << ro13 << ")" << endl;
                            break;
						case 3: //next: point 5
                            cout << "Cube point 4 reached" << endl;
                            attr_3d_pos(0) = ro14;
                            attr_3d_pos(1) = ro15;
							attr_3d_pos(2) = ro16;
                            att_state = 4;
                            cout << "Next point (5): (" << ro14 << ", " << ro15 << ", " << ro16 << ")" << endl;
                            break;
						case 4: //next: point 6
                            cout << "Cube point 5 reached" << endl;
                            attr_3d_pos(0) = ro17;
                            attr_3d_pos(1) = ro18;
							attr_3d_pos(2) = ro19;
                            att_state = 5;
                            cout << "Next point (6): (" << ro17 << ", " << ro18 << ", " << ro19 << ")" << endl;
                            break;
						case 5: //next: point 7
                            cout << "Cube point 6 reached" << endl;
                            attr_3d_pos(0) = ro20;
                            attr_3d_pos(1) = ro21;
							attr_3d_pos(2) = ro22;
                            att_state = 6;
                            cout << "Next point (7): (" << ro20 << ", " << ro21 << ", " << ro22 << ")" << endl;
                            break;
						case 6: //next: point 8
                            cout << "Cube point 7 reached" << endl;
                            attr_3d_pos(0) = ro23;
                            attr_3d_pos(1) = ro24;
							attr_3d_pos(2) = ro25;
                            att_state = 7;
                            cout << "Next point (8): (" << ro23 << ", " << ro24 << ", " << ro25 << ")" << endl;
                            break;
                        case 7:
                            cout << "Cube point 8 reached" << endl;
                            //ctrl_c_handler(0); //end execution
                            break;
                    }
                } 

                //recalculate attractor position
                att_pos[0] = 0; att_pos[1] = 0; att_pos[2] = 0; //left eye relative position
                att_pos[3] = calc_attractor_l_u(); att_pos[4] = calc_attractor_l_v(); att_pos[5] = 0; //left eye camera position
                att_pos[6] = calc_attractor_r_u(); att_pos[7] = calc_attractor_r_v(); att_pos[8] = 0; //right eye camera position

                break;
        }

        //publish attractor values
        if (main_config.publish)
        {
			//3D position: x, y, z
            Bottle& output_attr_x = plot_attr_x.prepare();
            output_attr_x.clear();
            output_attr_x.addDouble(attr_3d_pos(0)); //x
            plot_attr_x.write();
            Bottle& output_attr_y = plot_attr_y.prepare();
            output_attr_y.clear();
            output_attr_y.addDouble(attr_3d_pos(1)); //y
            plot_attr_y.write();
            Bottle& output_attr_z = plot_attr_z.prepare();
            output_attr_z.clear();
            output_attr_z.addDouble(attr_3d_pos(2)); //z
            plot_attr_z.write();
            //left eye: u, v, size
            Bottle& output_attr_l_u = plot_attr_l_u.prepare();
            output_attr_l_u.clear();
            output_attr_l_u.addDouble(att_pos(3)); //u
            plot_attr_l_u.write();
            Bottle& output_attr_l_v = plot_attr_l_v.prepare();
            output_attr_l_v.clear();
            output_attr_l_v.addDouble(att_pos(4)); //v
            plot_attr_l_v.write();
            Bottle& output_attr_l_size = plot_attr_l_size.prepare();
            output_attr_l_size.clear();
            output_attr_l_size.addDouble(att_pos(5)); //size
            plot_attr_l_size.write();
            //right eye: u, v, size
            Bottle& output_attr_r_u = plot_attr_r_u.prepare();
            output_attr_r_u.clear();
            output_attr_r_u.addDouble(att_pos(6)); //u
            plot_attr_r_u.write();
            Bottle& output_attr_r_v = plot_attr_r_v.prepare();
            output_attr_r_v.clear();
            output_attr_r_v.addDouble(att_pos(7)); //v
            plot_attr_r_v.write();
            Bottle& output_attr_r_size = plot_attr_r_size.prepare();
            output_attr_r_size.clear();
            output_attr_r_size.addDouble(att_pos(8)); //size
            plot_attr_r_size.write();
        }
    }

    //left arm visual to 3D
    if (left_arm_config.activated && (left_arm_config.on_vision)) //|| left_arm_config.on_attractor))
    {
        //read image and find location of end-effector
        Scalar left_col_low_b = Scalar(left_hue_low_b, left_sat_low_b, left_val_low_b);
        Scalar left_col_high_b = Scalar(left_hue_high_b, left_sat_high_b, left_val_high_b);
		Scalar right_col_low_b = Scalar(right_hue_low_b, right_sat_low_b, right_val_low_b);
        Scalar right_col_high_b = Scalar(right_hue_high_b, right_sat_high_b, right_val_high_b);
        //le_pos = iCubImgLb->findLocationCV(L, col_low_b, col_high_b, left_arm_config.erode_dilate_iterations, false);
        le_pos = iCubImgRLb->get3Dfrom2D(RL, left_col_low_b, left_col_high_b, left_arm_config.left_erode_dilate_iterations,
								right_col_low_b, right_col_high_b, left_arm_config.right_erode_dilate_iterations, 3, false);
    }

}

//right arm action
void set_action_right_arm_thread()
{
	double vel_q3, vel_q5, vel_q6, vel_q7;
    yarp::sig::Vector ra_encoders, t_encoders;
	
	if (right_arm_config.on_action)
	{
		//update action (velocity)
		a_mutex.lock();
		vel_q3 = CTRL_RAD2DEG*a[0]; //torso_yaw
		vel_q5 = CTRL_RAD2DEG*a[1]; //r_shoulder_roll
		vel_q6 = CTRL_RAD2DEG*a[2]; //r_shoulder_yaw
		vel_q7 = CTRL_RAD2DEG*a[3]; //r_elbow
		a_mutex.unlock();
		//velocity limitation (deg/s)
		if (vel_q3 > max_vel_arm) vel_q3 = max_vel_arm;
		if (vel_q3 < -max_vel_arm) vel_q3 = -max_vel_arm;
		if (vel_q5 > max_vel_arm) vel_q5 = max_vel_arm;
		if (vel_q5 < -max_vel_arm) vel_q5 = -max_vel_arm;
		if (vel_q6 > max_vel_arm) vel_q6 = max_vel_arm;
		if (vel_q6 < -max_vel_arm) vel_q6 = -max_vel_arm;
		if (vel_q7 > max_vel_arm) vel_q7 = max_vel_arm;
		if (vel_q7 < -max_vel_arm) vel_q7 = -max_vel_arm;
		//position limitation (stop)
		t_encoders = iCubTorsoCtrl->readEncoders();
        if (t_encoders[0] > torso_yaw_max && vel_q3 > 0) vel_q3 = 0; //-50 -> 50
        if (t_encoders[0] < torso_yaw_min && vel_q3 < 0) vel_q3 = 0; //-50 -> 50
		ra_encoders = iCubRightArmCtrl->readEncoders();
        if (ra_encoders[1] > shoulder_roll_max && vel_q5 > 0) vel_q5 = 0; //0 -> 160.8
        if (ra_encoders[1] < shoulder_roll_min && vel_q5 < 0) vel_q5 = 0; //0 -> 160.8
        if (ra_encoders[2] > shoulder_yaw_max && vel_q6 > 0) vel_q6 = 0; //-37 -> 100
        if (ra_encoders[2] < shoulder_yaw_min && vel_q6 < 0) vel_q6 = 0; //-37 -> 100
        if (ra_encoders[3] > elbow_max && vel_q7 > 0) vel_q7 = 0; //5.5 -> 106
        if (ra_encoders[3] < elbow_min && vel_q7 < 0) vel_q7 = 0; //5.5 -> 106
		//touch sensor
		/*if (config.on_touch && touching)
		{
			vel_q5 = 0;
			vel_q7 = 0;
		}*/
		//publishing
		if (main_config.publish)
		{
			Bottle& output_a1 = plot_a1.prepare();
			output_a1.clear();
			output_a1.addDouble(vel_q3);
			plot_a1.write();
			Bottle& output_a2 = plot_a2.prepare();
			output_a2.clear();
			output_a2.addDouble(vel_q5);
			plot_a2.write();
			Bottle& output_a3 = plot_a3.prepare();
			output_a3.clear();
			output_a3.addDouble(vel_q6);
			plot_a3.write();
			Bottle& output_a4 = plot_a4.prepare();
			output_a4.clear();
			output_a4.addDouble(vel_q7);
			plot_a4.write();
		}
		
		//send velocity command
		iCubTorsoCtrl->velocityMove(0, vel_q3);
		iCubRightArmCtrl->velocityMove(1, vel_q5);
		iCubRightArmCtrl->velocityMove(2, vel_q6);
		iCubRightArmCtrl->velocityMove(3, vel_q7);
		yarp::os::Time::yield(); //release the quantum
	}

}

//head action
void set_action_head_thread()
{
    double vel_q4e, vel_q6e, vel_q7e;
	yarp::sig::Vector le_encoders;
	
	if (head_config.on_action)
	{
		//update action (velocity)
		a_mutex.lock();
		vel_q4e = CTRL_RAD2DEG*a_head[0];
		vel_q6e = CTRL_RAD2DEG*a_head[1];
        vel_q7e = CTRL_RAD2DEG*a_head[2];
		a_mutex.unlock();
		//velocity limitation (deg/s)
		if (vel_q4e > max_vel_head) vel_q4e = max_vel_head;
		if (vel_q4e < -max_vel_head) vel_q4e = -max_vel_head;
		if (vel_q6e > max_vel_head) vel_q6e = max_vel_head;
		if (vel_q6e < -max_vel_head) vel_q6e = -max_vel_head;
        if (vel_q7e > max_vel_head) vel_q7e = max_vel_head;
        if (vel_q7e < -max_vel_head) vel_q7e = -max_vel_head;
		//position limitation (stop)
		le_encoders = iCubHeadCtrl->readEncoders();
        if (le_encoders[0] > neck_pitch_max && vel_q4e > 0) vel_q4e = 0; //-40 -> 30
        if (le_encoders[0] < neck_pitch_min && vel_q4e < 0) vel_q4e = 0; //-40 -> 30
        if (le_encoders[2] > neck_yaw_max && vel_q6e > 0) vel_q6e = 0; //-55 -> 55
        if (le_encoders[2] < neck_yaw_min && vel_q6e < 0) vel_q6e = 0; //-55 -> 55
        if (le_encoders[3] > eyes_tilt_max && vel_q7e > 0) vel_q7e = 0; //-30 -> 30
        if (le_encoders[3] < eyes_tilt_min && vel_q7e < 0) vel_q7e = 0; //-30 -> 30
		//publishing
		if (main_config.publish)
		{
			Bottle& output_a1e = plot_a1e.prepare();
			output_a1e.clear();
			output_a1e.addDouble(vel_q4e);
			plot_a1e.write();
			Bottle& output_a2e = plot_a2e.prepare();
			output_a2e.clear();
			output_a2e.addDouble(vel_q6e);
			plot_a2e.write();
            Bottle& output_a3e = plot_a3e.prepare();
            output_a3e.clear();
            output_a3e.addDouble(vel_q7e);
            plot_a3e.write();
		}
		
		//send velocity command
		iCubHeadCtrl->velocityMove(0, vel_q4e);
		iCubHeadCtrl->velocityMove(2, vel_q6e);
        iCubHeadCtrl->velocityMove(3, vel_q7e);
		yarp::os::Time::yield(); //release the quantum
	}

}

//left arm action
void set_action_left_arm_thread()
{
    double vel_q3, vel_q5, vel_q6, vel_q7;
    yarp::sig::Vector la_encoders, t_encoders;

    if (left_arm_config.on_action)
    {
        //update action (velocity)
        a_mutex.lock();
		vel_q3 = CTRL_RAD2DEG*a_b[0]; //torso_yaw
        vel_q5 = CTRL_RAD2DEG*a_b[1]; //l_shoulder_roll
        vel_q6 = CTRL_RAD2DEG*a_b[2]; //l_shoulder_yaw
        vel_q7 = CTRL_RAD2DEG*a_b[3]; //l_elbow
        a_mutex.unlock();
        //velocity limitation (deg/s)
        if (vel_q3 > max_vel_arm) vel_q3 = max_vel_arm;
        if (vel_q3 < -max_vel_arm) vel_q3 = -max_vel_arm;
        if (vel_q5 > max_vel_arm) vel_q5 = max_vel_arm;
        if (vel_q5 < -max_vel_arm) vel_q5 = -max_vel_arm;
        if (vel_q6 > max_vel_arm) vel_q6 = max_vel_arm;
        if (vel_q6 < -max_vel_arm) vel_q6 = -max_vel_arm;
        if (vel_q7 > max_vel_arm) vel_q7 = max_vel_arm;
        if (vel_q7 < -max_vel_arm) vel_q7 = -max_vel_arm;
        //position limitation (stop)		
		t_encoders = iCubTorsoCtrl->readEncoders();
        if (t_encoders[0] > torso_yaw_max && vel_q3 > 0) vel_q3 = 0; //-50 -> 50
        if (t_encoders[0] < torso_yaw_min && vel_q3 < 0) vel_q3 = 0; //-50 -> 50
        la_encoders = iCubLeftArmCtrl->readEncoders();
        if (la_encoders[1] > shoulder_roll_max && vel_q5 > 0) vel_q5 = 0; //0 -> 160.8
        if (la_encoders[1] < shoulder_roll_min && vel_q5 < 0) vel_q5 = 0; //0 -> 160.8
        if (la_encoders[2] > shoulder_yaw_max && vel_q6 > 0) vel_q6 = 0; //-37 -> 100
        if (la_encoders[2] < shoulder_yaw_min && vel_q6 < 0) vel_q6 = 0; //-37 -> 100
        if (la_encoders[3] > elbow_max && vel_q7 > 0) vel_q7 = 0; //5.5 -> 106
        if (la_encoders[3] < elbow_min && vel_q7 < 0) vel_q7 = 0; //5.5 -> 106
        //touch sensor
        /*if (config.on_touch && touching)
        {
            vel_q5 = 0;
            vel_q7 = 0;
        }*/
        //publishing
        if (main_config.publish)
        {
            Bottle& output_a1b = plot_a1b.prepare();
            output_a1b.clear();
            output_a1b.addDouble(vel_q3);
            plot_a1b.write();
            Bottle& output_a2b = plot_a2b.prepare();
            output_a2b.clear();
            output_a2b.addDouble(vel_q5);
            plot_a2b.write();
            Bottle& output_a3b = plot_a3b.prepare();
            output_a3b.clear();
            output_a3b.addDouble(vel_q6);
            plot_a3b.write();
            Bottle& output_a4b = plot_a4b.prepare();
            output_a4b.clear();
            output_a4b.addDouble(vel_q7);
            plot_a4b.write();
        }

        //send velocity command
		iCubTorsoCtrl->velocityMove(0, vel_q3);
        iCubLeftArmCtrl->velocityMove(1, vel_q5);
        iCubLeftArmCtrl->velocityMove(2, vel_q6);
        iCubLeftArmCtrl->velocityMove(3, vel_q7);
        yarp::os::Time::yield(); //release the quantum
    }

}

//touch
void set_touch_thread()
{
    yarp::sig::Vector rht, lht;

    if (right_arm_config.activated && right_arm_config.on_touch)
	{
		rht = iCubRightHandTouch->readTouchSensors();
        //TouchState right_hand = iCubRightHandTouch->identifyTouchSensors(rht);
		if (iCubRightHandTouch->checkTouch(rht))
		{
			touchCounter++;			
			if (touchCounter > right_arm_config.touch_threshold) //threshold touch filter
			{
				touching = true;
				cout << "Right hand in contact!" << endl;
                if (right_arm_config.grasping) rightGrasping(true); //grasp if activated
			}
		}else{
			touchCounter = 0;
			if (touching) cout << "End of right hand contact!" << endl;
			touching = false;
		}
		
	}

    if (left_arm_config.activated && left_arm_config.on_touch)
    {
        lht = iCubLeftHandTouch->readTouchSensors();
        //TouchState left_hand = iCubLeftHandTouch->identifyTouchSensors(lht);
        if (iCubLeftHandTouch->checkTouch(lht))
        {
            touchCounterLeft++;
            if (touchCounterLeft > left_arm_config.touch_threshold) //threshold touch filter
            {
                touchingLeft = true;
                cout << "Left hand in contact!" << endl;
                if (left_arm_config.grasping) leftGrasping(true); //grasp if activated
            }
        }else{
            touchCounterLeft = 0;
            if (touchingLeft) cout << "End of left hand contact!" << endl;
            touchingLeft = false;
        }

    }
}

//feo right arm calculation
void calculate_feo_right_arm()
{

	if (counter < main_config.cycles)
	{
		counter++;
		if (main_config.print) cout << "=> Updating right arm internal state... #" << counter << endl;

		auto t1 = hrClock::now();

		feoRightArm->updateInternalState();

		auto t2 = hrClock::now();

		a_mutex.lock();
		state = feoRightArm->getInternalState();
		a = feoRightArm->getAction();
		a_mutex.unlock();

		if (main_config.print)
		{

			cout << "Time taken: " 
				  << chrono::duration_cast<chrono::nanoseconds>(t2-t1).count()
				  << " nanoseconds." << endl;

			cout << "Current right arm state:";
			for (unsigned int i = 0; i < state.size(); i++)
			{
				a_mutex.lock();
				if (i < a.size()) cout << " " << state[i] << " " << a[i];
				else cout << " " << state[i];
				a_mutex.unlock();
			}
			cout << endl;

		}

	}

}

//feo head calculation
void calculate_feo_head()
{

	if (counter < main_config.cycles)
	{
		counter++;
		if (main_config.print) cout << "=> Updating head internal state... #" << counter << endl;

		auto t1 = hrClock::now();

		feoHead->updateInternalState();

		auto t2 = hrClock::now();

		a_mutex.lock();
		state_head = feoHead->getInternalState();
		a_head = feoHead->getAction();
		a_mutex.unlock();

		if (main_config.print)
		{

			cout << "Time taken: " 
				  << chrono::duration_cast<chrono::nanoseconds>(t2-t1).count()
				  << " nanoseconds." << endl;

			cout << "Current head state:";
			for (unsigned int i = 0; i < state_head.size(); i++)
			{
				a_mutex.lock();
				if (i < a_head.size()) cout << " " << state_head[i] << " " << a_head[i];
				else cout << " " << state_head[i];
				a_mutex.unlock();
			}
			cout << endl;

		}

	}

}

//feo left arm calculation
void calculate_feo_left_arm()
{

    if (counter < main_config.cycles)
    {
        counter++;
        if (main_config.print) cout << "=> Updating left arm internal state... #" << counter << endl;

        auto t1 = hrClock::now();

        feoLeftArm->updateInternalState();

        auto t2 = hrClock::now();

        a_mutex.lock();
        state_b = feoLeftArm->getInternalState();
        a_b = feoLeftArm->getAction();
        a_mutex.unlock();

        if (main_config.print)
        {

            cout << "Time taken: "
                  << chrono::duration_cast<chrono::nanoseconds>(t2-t1).count()
                  << " nanoseconds." << endl;

            cout << "Current left arm state:";
            for (unsigned int i = 0; i < state_b.size(); i++)
            {
                a_mutex.lock();
                if (i < a_b.size()) cout << " " << state_b[i] << " " << a_b[i];
                else cout << " " << state_b[i];
                a_mutex.unlock();
            }
            cout << endl;

        }

    }

}

//----------------------------------------------------------------------------------
//Motion ---------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//right hand grasp for object
void rightGrasping(bool all)
{
	//Fingers joints: 7 to 15

    cout << "Initiating right hand grasping..." << endl;

	//Set control mode to position
	iCubRightArmCtrl->changeControlMode(7, position);
	iCubRightArmCtrl->changeControlMode(8, position);
	iCubRightArmCtrl->changeControlMode(9, position);
	iCubRightArmCtrl->changeControlMode(10, position);
	iCubRightArmCtrl->changeControlMode(11, position);
	iCubRightArmCtrl->changeControlMode(12, position);
	iCubRightArmCtrl->changeControlMode(13, position);
	iCubRightArmCtrl->changeControlMode(14, position);
	iCubRightArmCtrl->changeControlMode(15, position);

	//Set reference speeds for grasping
	iCubRightArmCtrl->setPositionMotionSpeed(7, 10);
	iCubRightArmCtrl->setPositionMotionSpeed(8, 60);
	iCubRightArmCtrl->setPositionMotionSpeed(9, 60);
	iCubRightArmCtrl->setPositionMotionSpeed(10, 60);
	iCubRightArmCtrl->setPositionMotionSpeed(11, 60);
	iCubRightArmCtrl->setPositionMotionSpeed(12, 60);
	iCubRightArmCtrl->setPositionMotionSpeed(13, 60);
	iCubRightArmCtrl->setPositionMotionSpeed(14, 60);
	iCubRightArmCtrl->setPositionMotionSpeed(15, 100);

	yarp::sig::Vector q_init_ra = iCubRightArmCtrl->readEncoders();

	if (!all)
	{

		cout << "Grasping position 1." << endl;

		//Move right hand to desired position
		q_init_ra[7] = 15; //r_hand_finger
		q_init_ra[8] = 80; //r_thumb_oppose
		q_init_ra[9] = 0; //r_thumb_proximal
		q_init_ra[10] = 0; //r_thumb_distal
		q_init_ra[11] = 70; //r_index_proximal
		q_init_ra[12] = 0; //r_index_distal
		q_init_ra[13] = 70; //r_middle_proximal
		q_init_ra[14] = 0; //r_middle_distal
		q_init_ra[15] = 100; //r_pinky
		//iCubRightArmCtrl->changeControlMode(position);
		iCubRightArmCtrl->setPosition(q_init_ra, true);

		cout << "Grasping position 2." << endl;

		//Move right hand to desired position
		q_init_ra[7] = 15; //r_hand_finger
		q_init_ra[8] = 80; //r_thumb_oppose
		q_init_ra[9] = 0; //r_thumb_proximal
		q_init_ra[10] = 0; //r_thumb_distal
		q_init_ra[11] = 70; //r_index_proximal
		q_init_ra[12] = 20; //r_index_distal
		q_init_ra[13] = 70; //r_middle_proximal
		q_init_ra[14] = 20; //r_middle_distal
		q_init_ra[15] = 140; //r_pinky
		//iCubRightArmCtrl->changeControlMode(position);
		iCubRightArmCtrl->setPosition(q_init_ra, true);

		cout << "Grasping position 3." << endl;

		//Move right hand to desired position
		q_init_ra[7] = 15; //r_hand_finger
		q_init_ra[8] = 80; //r_thumb_oppose
		q_init_ra[9] = 20; //r_thumb_proximal
		q_init_ra[10] = 0; //r_thumb_distal
		q_init_ra[11] = 70; //r_index_proximal
		q_init_ra[12] = 40; //r_index_distal
		q_init_ra[13] = 70; //r_middle_proximal
		q_init_ra[14] = 40; //r_middle_distal
		q_init_ra[15] = 160; //r_pinky
		//iCubRightArmCtrl->changeControlMode(position);
		iCubRightArmCtrl->setPosition(q_init_ra, true);

		cout << "Grasping position 4." << endl;

		//Move right hand to desired position
		q_init_ra[7] = 15; //r_hand_finger
		q_init_ra[8] = 80; //r_thumb_oppose
		q_init_ra[9] = 20; //r_thumb_proximal
		q_init_ra[10] = 10; //r_thumb_distal
		q_init_ra[11] = 70; //r_index_proximal
		q_init_ra[12] = 50; //r_index_distal
		q_init_ra[13] = 70; //r_middle_proximal
		q_init_ra[14] = 50; //r_middle_distal
		q_init_ra[15] = 180; //r_pinky
		//iCubRightArmCtrl->changeControlMode(position);
		iCubRightArmCtrl->setPosition(q_init_ra, true);

	
	}else{

		cout << "Grasping position final." << endl;

		//Move left hand to desired position
		q_init_ra[7] = 15; //r_hand_finger
		q_init_ra[8] = 80; //r_thumb_oppose
		q_init_ra[9] = 20; //r_thumb_proximal
		q_init_ra[10] = 10; //r_thumb_distal
		q_init_ra[11] = 70; //r_index_proximal
		q_init_ra[12] = 50; //r_index_distal
		q_init_ra[13] = 70; //r_middle_proximal
		q_init_ra[14] = 50; //r_middle_distal
		q_init_ra[15] = 180; //r_pinky
		//iCubRightArmCtrl->changeControlMode(position);
		iCubRightArmCtrl->setPosition(q_init_ra, true);
	
	}

    cout << "Right hand grasping finished!" << endl;

    //end execution
    ctrl_c_handler(0);

}

//left hand grasp for object
void leftGrasping(bool all)
{
    //Fingers joints: 7 to 15

    cout << "Initiating left hand grasping..." << endl;

    //Set control mode to position
    iCubLeftArmCtrl->changeControlMode(7, position);
    iCubLeftArmCtrl->changeControlMode(8, position);
    iCubLeftArmCtrl->changeControlMode(9, position);
    iCubLeftArmCtrl->changeControlMode(10, position);
    iCubLeftArmCtrl->changeControlMode(11, position);
    iCubLeftArmCtrl->changeControlMode(12, position);
    iCubLeftArmCtrl->changeControlMode(13, position);
    iCubLeftArmCtrl->changeControlMode(14, position);
    iCubLeftArmCtrl->changeControlMode(15, position);

    //Set reference speeds for grasping
    iCubLeftArmCtrl->setPositionMotionSpeed(7, 10);
    iCubLeftArmCtrl->setPositionMotionSpeed(8, 60);
    iCubLeftArmCtrl->setPositionMotionSpeed(9, 60);
    iCubLeftArmCtrl->setPositionMotionSpeed(10, 60);
    iCubLeftArmCtrl->setPositionMotionSpeed(11, 60);
    iCubLeftArmCtrl->setPositionMotionSpeed(12, 60);
    iCubLeftArmCtrl->setPositionMotionSpeed(13, 60);
    iCubLeftArmCtrl->setPositionMotionSpeed(14, 60);
    iCubLeftArmCtrl->setPositionMotionSpeed(15, 100);

    yarp::sig::Vector q_init_la = iCubLeftArmCtrl->readEncoders();

    if (!all)
    {

        cout << "Grasping position 1." << endl;

        //Move left hand to desired position
        q_init_la[7] = 15; //l_hand_finger
        q_init_la[8] = 80; //l_thumb_oppose
        q_init_la[9] = 0; //l_thumb_proximal
        q_init_la[10] = 0; //l_thumb_distal
        q_init_la[11] = 70; //l_index_proximal
        q_init_la[12] = 0; //l_index_distal
        q_init_la[13] = 70; //l_middle_proximal
        q_init_la[14] = 0; //l_middle_distal
        q_init_la[15] = 100; //l_pinky
        //iCubRightArmCtrl->changeControlMode(position);
        iCubLeftArmCtrl->setPosition(q_init_la, true);

        cout << "Grasping position 2." << endl;

        //Move left hand to desired position
        q_init_la[7] = 15; //l_hand_finger
        q_init_la[8] = 80; //l_thumb_oppose
        q_init_la[9] = 0; //l_thumb_proximal
        q_init_la[10] = 0; //l_thumb_distal
        q_init_la[11] = 70; //l_index_proximal
        q_init_la[12] = 20; //l_index_distal
        q_init_la[13] = 70; //l_middle_proximal
        q_init_la[14] = 20; //l_middle_distal
        q_init_la[15] = 140; //l_pinky
        //iCubRightArmCtrl->changeControlMode(position);
        iCubLeftArmCtrl->setPosition(q_init_la, true);

        cout << "Grasping position 3." << endl;

        //Move left hand to desired position
        q_init_la[7] = 15; //l_hand_finger
        q_init_la[8] = 80; //l_thumb_oppose
        q_init_la[9] = 20; //l_thumb_proximal
        q_init_la[10] = 0; //l_thumb_distal
        q_init_la[11] = 70; //l_index_proximal
        q_init_la[12] = 40; //l_index_distal
        q_init_la[13] = 70; //l_middle_proximal
        q_init_la[14] = 40; //l_middle_distal
        q_init_la[15] = 160; //l_pinky
        //iCubRightArmCtrl->changeControlMode(position);
        iCubLeftArmCtrl->setPosition(q_init_la, true);

        cout << "Grasping position 4." << endl;

        //Move left hand to desired position
        q_init_la[7] = 15; //l_hand_finger
        q_init_la[8] = 80; //l_thumb_oppose
        q_init_la[9] = 20; //l_thumb_proximal
        q_init_la[10] = 10; //l_thumb_distal
        q_init_la[11] = 70; //l_index_proximal
        q_init_la[12] = 50; //l_index_distal
        q_init_la[13] = 70; //l_middle_proximal
        q_init_la[14] = 50; //l_middle_distal
        q_init_la[15] = 180; //l_pinky
        //iCubRightArmCtrl->changeControlMode(position);
        iCubLeftArmCtrl->setPosition(q_init_la, true);


    }else{

        cout << "Grasping position final." << endl;

        //Move left hand to desired position
        q_init_la[7] = 15; //l_hand_finger
        q_init_la[8] = 80; //l_thumb_oppose
        q_init_la[9] = 20; //l_thumb_proximal
        q_init_la[10] = 10; //l_thumb_distal
        q_init_la[11] = 70; //l_index_proximal
        q_init_la[12] = 50; //l_index_distal
        q_init_la[13] = 70; //l_middle_proximal
        q_init_la[14] = 50; //l_middle_distal
        q_init_la[15] = 180; //l_pinky
        //iCubRightArmCtrl->changeControlMode(position);
        iCubLeftArmCtrl->setPosition(q_init_la, true);

    }

    cout << "Left hand grasping finished!" << endl;

    //end execution
    ctrl_c_handler(0);

}

//----------------------------------------------------------------------------------
//Other ----------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//set ctrl+C handler
void set_interrupt_handle()
{
	struct sigaction new_action, old_action;

	//Set the handler in the new_action struct
	new_action.sa_handler = ctrl_c_handler;
	//Set to empty the sa_mask. It means that no signal is blocked
	// while the handler run.
	sigemptyset(&new_action.sa_mask);
	//Block the SEGTERM signal.
	// It means that while the handler run, the SIGTERM signal is ignored
	sigaddset(&new_action.sa_mask, SIGTERM);
	//Remove any flag from sa_flag. See documentation for flags allowed
	new_action.sa_flags = 0;

	//Register action
	sigaction(SIGINT,&new_action,NULL);

	//signal(SIGINT, ctrl_c_handler);
}

//ctrl+C handler
void ctrl_c_handler(int s)
{
	if (t_end) terminate(); //throw runtime_error("Program terminated!");
	
	switch (stage)
	{

		case read_config:
			cout << "Ending from read_config." << endl;
			t_end = true;
			break;
		case start_icub:
			cout << "Ending from start_icub." << endl;
			if (iCubRightArmCtrl != NULL) delete iCubRightArmCtrl; //control and encoders
			if (iCubHeadCtrl != NULL) delete iCubHeadCtrl;
            if (iCubLeftArmCtrl != NULL) delete iCubLeftArmCtrl;
            if (iCubTorsoCtrl != NULL) delete iCubTorsoCtrl;
			if (iCubRightArmKin != NULL) delete iCubRightArmKin; //kinematics
			if (iCubLeftEyeKin != NULL) delete iCubLeftEyeKin;
            if (iCubLeftArmKin != NULL) delete iCubLeftArmKin;
            if (iCubTorsoKin != NULL) delete iCubTorsoKin;
			t_end = true;
			break;
		case start_publish:
			cout << "Ending from start_publish." << endl;
			clean_up();
			t_end = true;
			break;
		case start_camera:
			cout << "Ending from start_camera." << endl;
			//suspend thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->suspend();
			//end thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->stop();
			clean_up();
			t_end = true;
			break;
		case start_touch:
			cout << "Ending from start_touch." << endl;
			//suspend thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->suspend();
            if ((right_arm_config.on_touch && right_arm_config.activated) || (left_arm_config.on_touch && left_arm_config.activated)) touchThread->suspend();
			//end thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->stop();
            if ((right_arm_config.on_touch && right_arm_config.activated) || (left_arm_config.on_touch && left_arm_config.activated)) touchThread->stop();
			clean_up();
			t_end = true;
			break;
		case start_noise:
			cout << "Ending from start_noise." << endl;
			//suspend thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->suspend();
            if ((right_arm_config.on_touch && right_arm_config.activated) || (left_arm_config.on_touch && left_arm_config.activated)) touchThread->suspend();
			//end thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->stop();
            if ((right_arm_config.on_touch && right_arm_config.activated) || (left_arm_config.on_touch && left_arm_config.activated)) touchThread->stop();
			clean_up();
			t_end = true;
			break;
		case start_feo:
			cout << "Ending from start_feo." << endl;
			//suspend thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->suspend();
            if ((right_arm_config.on_touch && right_arm_config.activated) || (left_arm_config.on_touch && left_arm_config.activated)) touchThread->suspend();
			//end thread execution
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->stop();
            if ((right_arm_config.on_touch && right_arm_config.activated) || (left_arm_config.on_touch && left_arm_config.activated)) touchThread->stop();
			clean_up();
			t_end = true;
			break;
		case exec_feo:
			cout << "Ending from exec_feo." << endl;
			//suspend thread execution
			if (right_arm_config.activated) feoRightArmThread->suspend();
			if (head_config.activated) feoHeadThread->suspend();
            if (left_arm_config.activated) feoLeftArmThread->suspend();
			if (right_arm_config.on_action && right_arm_config.activated) actionRightArmThread->suspend();
			if (head_config.on_action && head_config.activated) actionHeadThread->suspend();
            if (left_arm_config.on_action && left_arm_config.activated) actionLeftArmThread->suspend();
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->suspend();
            if ((left_arm_config.on_touch && left_arm_config.activated) || (right_arm_config.on_touch && right_arm_config.activated)) touchThread->suspend();
			//end thread execution
			if (right_arm_config.activated) feoRightArmThread->stop();
			if (head_config.activated) feoHeadThread->stop();
            if (left_arm_config.activated) feoLeftArmThread->stop();
			if (right_arm_config.on_action && right_arm_config.activated) actionRightArmThread->stop();
			if (head_config.on_action && head_config.activated) actionHeadThread->stop();
            if (left_arm_config.on_action && left_arm_config.activated) actionLeftArmThread->stop();
            if (left_arm_config.on_vision || right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_attractor) visionThread->stop();
            if ((left_arm_config.on_touch && left_arm_config.activated) || (right_arm_config.on_touch && right_arm_config.activated)) touchThread->stop();
			t_end = true;
			break;

	}

	return;
}

//prepare variables for extern access
void prepare_variables()
{

	//copy config values to local variables (for extern)
    dT = main_config.dT;
	publish = main_config.publish;

}

void end_right_arm()
{
	//End velocity movement
	iCubTorsoCtrl->velocityMove(0, 0);
	iCubRightArmCtrl->velocityMove(1, 0);
	iCubRightArmCtrl->velocityMove(2, 0);
	iCubRightArmCtrl->velocityMove(3, 0);

	//Change control mode to position
	iCubTorsoCtrl->changeControlMode(0, position);
	iCubRightArmCtrl->changeControlMode(1, position);
	iCubRightArmCtrl->changeControlMode(2, position);
	iCubRightArmCtrl->changeControlMode(3, position);
	
	//Print thread execution results
	cout << "Free energy optimization right arm thread ran " << feoRightArmThread->getIterations() << 
		" times with estimated period (ms): " << feoRightArmThread->getEstPeriod() << endl;
	if (right_arm_config.activated && right_arm_config.on_action) cout << "Action right arm thread ran " << actionRightArmThread->getIterations() <<
		" times with estimated period (ms): " << actionRightArmThread->getEstPeriod() << endl;
	if (right_arm_config.on_vision || right_arm_config.on_attractor) cout << "Vision thread ran " << visionThread->getIterations() <<
		" times with estimated period (ms): " << visionThread->getEstPeriod() << endl;
	if (right_arm_config.on_touch) cout << "Touch thread ran " << touchThread->getIterations() <<
		" times with estimated period (ms): " << touchThread->getEstPeriod() << endl;
	
}

void end_head()
{
	//End velocity movement
	iCubHeadCtrl->velocityMove(0, 0);
	iCubHeadCtrl->velocityMove(2, 0);
    iCubHeadCtrl->velocityMove(3, 0);

	//Change control mode to position
	iCubHeadCtrl->changeControlMode(0, position);
	iCubHeadCtrl->changeControlMode(2, position);
    iCubHeadCtrl->changeControlMode(3, position);
	
	//Print thread execution results
	cout << "Free energy optimization head thread ran " << feoHeadThread->getIterations() << 
		" times with estimated period (ms): " << feoHeadThread->getEstPeriod() << endl;
	if (head_config.activated && head_config.on_action) cout << "Action head thread ran " << actionHeadThread->getIterations() <<
		" times with estimated period (ms): " << actionHeadThread->getEstPeriod() << endl;
	if (right_arm_config.on_vision || right_arm_config.on_attractor) cout << "Vision thread ran " << visionThread->getIterations() <<
		" times with estimated period (ms): " << visionThread->getEstPeriod() << endl;
	
}

void end_left_arm()
{
    //End velocity movement
	iCubTorsoCtrl->velocityMove(0, 0);
    iCubLeftArmCtrl->velocityMove(1, 0);
    iCubLeftArmCtrl->velocityMove(2, 0);
    iCubLeftArmCtrl->velocityMove(3, 0);

    //Change control mode to position
	iCubTorsoCtrl->changeControlMode(0, position);
    iCubLeftArmCtrl->changeControlMode(1, position);
    iCubLeftArmCtrl->changeControlMode(2, position);
    iCubLeftArmCtrl->changeControlMode(3, position);

    //Print thread execution results
    cout << "Free energy optimization left arm thread ran " << feoLeftArmThread->getIterations() <<
        " times with estimated period (ms): " << feoLeftArmThread->getEstPeriod() << endl;
    if (left_arm_config.activated && left_arm_config.on_action) cout << "Action left arm thread ran " << actionLeftArmThread->getIterations() <<
        " times with estimated period (ms): " << actionLeftArmThread->getEstPeriod() << endl;
    if (left_arm_config.on_vision || left_arm_config.on_attractor) cout << "Vision thread ran " << visionThread->getIterations() <<
        " times with estimated period (ms): " << visionThread->getEstPeriod() << endl;
    if (left_arm_config.on_touch) cout << "Touch thread ran " << touchThread->getIterations() <<
        " times with estimated period (ms): " << touchThread->getEstPeriod() << endl;

}

//clean up at end of execution
void clean_up()
{
	cout << "Cleaning up..." << endl;

	//close ports
	if (main_config.publish)
	{
        //right arm ports
		if (right_arm_config.activated)
		{
			//send zeros
			Bottle& output_q1 = plot_q1.prepare();
			output_q1.clear();
			output_q1.addDouble(0.0);
			plot_q1.write();
			Bottle& output_q2 = plot_q2.prepare();
			output_q2.clear();
			output_q2.addDouble(0.0);
			plot_q2.write();
			Bottle& output_q3 = plot_q3.prepare();
			output_q3.clear();
			output_q3.addDouble(0.0);
			plot_q3.write();
			Bottle& output_q4 = plot_q4.prepare();
			output_q4.clear();
			output_q4.addDouble(0.0);
			plot_q4.write();
			Bottle& output_mu1 = plot_mu1.prepare();
			output_mu1.clear();
			output_mu1.addDouble(0.0);
			plot_mu1.write();
			Bottle& output_mu2 = plot_mu2.prepare();
			output_mu2.clear();
			output_mu2.addDouble(0.0);
			plot_mu2.write();
			Bottle& output_mu3 = plot_mu3.prepare();
			output_mu3.clear();
			output_mu3.addDouble(0.0);
			plot_mu3.write();
			Bottle& output_mu4 = plot_mu4.prepare();
			output_mu4.clear();
			output_mu4.addDouble(0.0);
			plot_mu4.write();
			Bottle& output_mu1p = plot_mu1p.prepare();
			output_mu1p.clear();
			output_mu1p.addDouble(0.0);
			plot_mu1p.write();
			Bottle& output_mu2p = plot_mu2p.prepare();
			output_mu2p.clear();
			output_mu2p.addDouble(0.0);
			plot_mu2p.write();
			Bottle& output_mu3p = plot_mu3p.prepare();
			output_mu3p.clear();
			output_mu3p.addDouble(0.0);
			plot_mu3p.write();
			Bottle& output_mu4p = plot_mu4p.prepare();
			output_mu4p.clear();
			output_mu4p.addDouble(0.0);
			plot_mu4p.write();
			Bottle& output_a1 = plot_a1.prepare();
			output_a1.clear();
			output_a1.addDouble(0.0);
			plot_a1.write();
			Bottle& output_a2 = plot_a2.prepare();
			output_a2.clear();
			output_a2.addDouble(0.0);
			plot_a2.write();
			Bottle& output_a3 = plot_a3.prepare();
			output_a3.clear();
			output_a3.addDouble(0.0);
			plot_a3.write();
			Bottle& output_a4 = plot_a4.prepare();
			output_a4.clear();
			output_a4.addDouble(0.0);
			plot_a4.write();
            //end-effector
            Bottle& output_l_u = plot_l_u.prepare();
            output_l_u.clear();
            output_l_u.addDouble(0.0);
            plot_l_u.write();
            Bottle& output_l_v = plot_l_v.prepare();
            output_l_v.clear();
            output_l_v.addDouble(0.0);
            plot_l_v.write();
            Bottle& output_l_size = plot_l_size.prepare();
            output_l_size.clear();
            output_l_size.addDouble(0.0);
            plot_l_size.write();
            Bottle& output_r_u = plot_r_u.prepare();
            output_r_u.clear();
            output_r_u.addDouble(0.0);
            plot_r_u.write();
            Bottle& output_r_v = plot_r_v.prepare();
            output_r_v.clear();
            output_r_v.addDouble(0.0);
            plot_r_v.write();
            Bottle& output_r_size = plot_r_size.prepare();
            output_r_size.clear();
            output_r_size.addDouble(0.0);
            plot_r_size.write();
            Bottle& output_x = plot_x.prepare();
            output_x.clear();
            output_x.addDouble(0.0);
            plot_x.write();
            Bottle& output_y = plot_y.prepare();
            output_y.clear();
            output_y.addDouble(0.0);
            plot_y.write();
            Bottle& output_z = plot_z.prepare();
            output_z.clear();
            output_z.addDouble(0.0);
            plot_z.write();
            Bottle& output_calc_l_u = plot_calc_l_u.prepare();
            output_calc_l_u.clear();
            output_calc_l_u.addDouble(0.0);
            plot_calc_l_u.write();
            Bottle& output_calc_l_v = plot_calc_l_v.prepare();
            output_calc_l_v.clear();
            output_calc_l_v.addDouble(0.0);
            plot_calc_l_v.write();
            Bottle& output_calc_r_u = plot_calc_r_u.prepare();
            output_calc_r_u.clear();
            output_calc_r_u.addDouble(0.0);
            plot_calc_r_u.write();
            Bottle& output_calc_r_v = plot_calc_r_v.prepare();
            output_calc_r_v.clear();
            output_calc_r_v.addDouble(0.0);
            plot_calc_r_v.write();
            Bottle& output_calc_x = plot_calc_x.prepare();
            output_calc_x.clear();
            output_calc_x.addDouble(0.0);
            plot_calc_x.write();
            Bottle& output_calc_y = plot_calc_y.prepare();
            output_calc_y.clear();
            output_calc_y.addDouble(0.0);
            plot_calc_y.write();
            Bottle& output_calc_z = plot_calc_z.prepare();
            output_calc_z.clear();
            output_calc_z.addDouble(0.0);
            plot_calc_z.write();
            //attractor
            Bottle& output_attr_l_u = plot_attr_l_u.prepare();
            output_attr_l_u.clear();
            output_attr_l_u.addDouble(0.0);
            plot_attr_l_u.write();
            Bottle& output_attr_l_v = plot_attr_l_v.prepare();
            output_attr_l_v.clear();
            output_attr_l_v.addDouble(0.0);
            plot_attr_l_v.write();
            Bottle& output_attr_l_size = plot_attr_l_size.prepare();
            output_attr_l_size.clear();
            output_attr_l_size.addDouble(0.0);
            plot_attr_l_size.write();
            Bottle& output_attr_r_u = plot_attr_r_u.prepare();
            output_attr_r_u.clear();
            output_attr_r_u.addDouble(0.0);
            plot_attr_r_u.write();
            Bottle& output_attr_r_v = plot_attr_r_v.prepare();
            output_attr_r_v.clear();
            output_attr_r_v.addDouble(0.0);
            plot_attr_r_v.write();
            Bottle& output_attr_r_size = plot_attr_r_size.prepare();
            output_attr_r_size.clear();
            output_attr_r_size.addDouble(0.0);
            plot_attr_r_size.write();
			Bottle& output_attr_x = plot_attr_x.prepare();
			output_attr_x.clear();
			output_attr_x.addDouble(0.0);
            plot_attr_x.write();
			Bottle& output_attr_y = plot_attr_y.prepare();
			output_attr_y.clear();
			output_attr_y.addDouble(0.0);
            plot_attr_y.write();
			Bottle& output_attr_z = plot_attr_z.prepare();
			output_attr_z.clear();
			output_attr_z.addDouble(0.0);
            plot_attr_z.write();
            //other
			Bottle& output_free_energy = plot_free_energy.prepare();
			output_free_energy.clear();
			output_free_energy.addDouble(0.0);
			plot_free_energy.write();

			//close ports
			plot_q1.close();
			plot_q2.close();
			plot_q3.close();
			plot_q4.close();
			plot_mu1.close();
			plot_mu2.close();
			plot_mu3.close();
			plot_mu4.close();
			plot_mu1p.close();
			plot_mu2p.close();
			plot_mu3p.close();
			plot_mu4p.close();
			plot_a1.close();
			plot_a2.close();
			plot_a3.close();
			plot_a4.close();
            //end-effector
            plot_calc_l_u.close();
            plot_calc_l_v.close();
            plot_calc_r_u.close();
            plot_calc_r_v.close();
			plot_calc_x.close();
			plot_calc_y.close();
			plot_calc_z.close();
            plot_r_u.close();
            plot_r_v.close();
            plot_r_size.close();
            plot_l_u.close();
            plot_l_v.close();
            plot_l_size.close();
            plot_x.close();
            plot_y.close();
            plot_z.close();
            //attractor
            plot_attr_r_u.close();
            plot_attr_r_v.close();
            plot_attr_r_size.close();
            plot_attr_l_u.close();
            plot_attr_l_v.close();
            plot_attr_l_size.close();
			plot_attr_x.close();
			plot_attr_y.close();
			plot_attr_z.close();
			plot_free_energy.close();
		}

        //head ports
		if (head_config.activated)
		{

			//send zeros
			Bottle& output_q1e = plot_q1e.prepare();
			output_q1e.clear();
			output_q1e.addDouble(0.0);
			plot_q1e.write();
			Bottle& output_q2e = plot_q2e.prepare();
			output_q2e.clear();
			output_q2e.addDouble(0.0);
			plot_q2e.write();
            Bottle& output_q3e = plot_q3e.prepare();
            output_q3e.clear();
            output_q3e.addDouble(0.0);
            plot_q3e.write();
			Bottle& output_mu1e = plot_mu1e.prepare();
			output_mu1e.clear();
			output_mu1e.addDouble(0.0);
			plot_mu1e.write();
			Bottle& output_mu2e = plot_mu2e.prepare();
			output_mu2e.clear();
			output_mu2e.addDouble(0.0);
			plot_mu2e.write();
            Bottle& output_mu3e = plot_mu3e.prepare();
            output_mu3e.clear();
            output_mu3e.addDouble(0.0);
            plot_mu3e.write();
			Bottle& output_mu1ep = plot_mu1ep.prepare();
			output_mu1ep.clear();
			output_mu1ep.addDouble(0.0);
			plot_mu1ep.write();
			Bottle& output_mu2ep = plot_mu2ep.prepare();
			output_mu2ep.clear();
			output_mu2ep.addDouble(0.0);
			plot_mu2ep.write();
            Bottle& output_mu3ep = plot_mu3ep.prepare();
            output_mu3ep.clear();
            output_mu3ep.addDouble(0.0);
            plot_mu3ep.write();
			Bottle& output_a1e = plot_a1e.prepare();
			output_a1e.clear();
			output_a1e.addDouble(0.0);
			plot_a1e.write();
			Bottle& output_a2e = plot_a2e.prepare();
			output_a2e.clear();
			output_a2e.addDouble(0.0);
			plot_a2e.write();
            Bottle& output_a3e = plot_a3e.prepare();
            output_a3e.clear();
            output_a3e.addDouble(0.0);
            plot_a3e.write();
            /*Bottle& output_attr_u = plot_attr_u.prepare();
			output_attr_u.clear();
			output_attr_u.addDouble(0.0);
			plot_attr_u.write();
			Bottle& output_attr_v = plot_attr_v.prepare();
			output_attr_v.clear();
			output_attr_v.addDouble(0.0);
			plot_attr_v.write();
            Bottle& output_attr_size = plot_attr_size.prepare();
            output_attr_size.clear();
            output_attr_size.addDouble(0.0);
            plot_attr_size.write();*/
			Bottle& output_center_x = plot_center_x.prepare();
			output_center_x.clear();
			output_center_x.addDouble(0.0);
			plot_center_x.write();
			Bottle& output_center_y = plot_center_y.prepare();
			output_center_y.clear();
			output_center_y.addDouble(0.0);
			plot_center_y.write();
			Bottle& output_free_energy_e = plot_free_energy_e.prepare();
			output_free_energy_e.clear();
			output_free_energy_e.addDouble(0.0);
			plot_free_energy_e.write();

			//close ports
			plot_q1e.close();
			plot_q2e.close();
            plot_q3e.close();
			plot_mu1e.close();
			plot_mu2e.close();
            plot_mu3e.close();
			plot_mu1ep.close();
			plot_mu2ep.close();
            plot_mu3ep.close();
			plot_a1e.close();
			plot_a2e.close();
            plot_a3e.close();
            /*plot_attr_u.close();
			plot_attr_v.close();
            plot_attr_size.close();*/
			plot_center_x.close();
			plot_center_y.close();
			plot_free_energy_e.close();
		}

        //left arm ports
        if (left_arm_config.activated)
        {
            //send zeros
            Bottle& output_q1b = plot_q1b.prepare();
            output_q1b.clear();
            output_q1b.addDouble(0.0);
            plot_q1b.write();
            Bottle& output_q2b = plot_q2b.prepare();
            output_q2b.clear();
            output_q2b.addDouble(0.0);
            plot_q2b.write();
            Bottle& output_q3b = plot_q3b.prepare();
            output_q3b.clear();
            output_q3b.addDouble(0.0);
            plot_q3b.write();
			Bottle& output_q4b = plot_q4b.prepare();
            output_q4b.clear();
            output_q4b.addDouble(0.0);
            plot_q4b.write();
            Bottle& output_mu1b = plot_mu1b.prepare();
            output_mu1b.clear();
            output_mu1b.addDouble(0.0);
            plot_mu1b.write();
            Bottle& output_mu2b = plot_mu2b.prepare();
            output_mu2b.clear();
            output_mu2b.addDouble(0.0);
            plot_mu2b.write();
            Bottle& output_mu3b = plot_mu3b.prepare();
            output_mu3b.clear();
            output_mu3b.addDouble(0.0);
            plot_mu3b.write();
			Bottle& output_mu4b = plot_mu4b.prepare();
            output_mu4b.clear();
            output_mu4b.addDouble(0.0);
            plot_mu4b.write();
            Bottle& output_mu1bp = plot_mu1bp.prepare();
            output_mu1bp.clear();
            output_mu1bp.addDouble(0.0);
            plot_mu1bp.write();
            Bottle& output_mu2bp = plot_mu2bp.prepare();
            output_mu2bp.clear();
            output_mu2bp.addDouble(0.0);
            plot_mu2bp.write();
            Bottle& output_mu3bp = plot_mu3bp.prepare();
            output_mu3bp.clear();
            output_mu3bp.addDouble(0.0);
            plot_mu3bp.write();
			Bottle& output_mu4bp = plot_mu4bp.prepare();
            output_mu4bp.clear();
            output_mu4bp.addDouble(0.0);
            plot_mu4bp.write();
            Bottle& output_a1b = plot_a1b.prepare();
            output_a1b.clear();
            output_a1b.addDouble(0.0);
            plot_a1b.write();
            Bottle& output_a2b = plot_a2b.prepare();
            output_a2b.clear();
            output_a2b.addDouble(0.0);
            plot_a2b.write();
            Bottle& output_a3b = plot_a3b.prepare();
            output_a3b.clear();
            output_a3b.addDouble(0.0);
            plot_a3b.write();
			Bottle& output_a4b = plot_a4b.prepare();
            output_a4b.clear();
            output_a4b.addDouble(0.0);
            plot_a4b.write();
            Bottle& output_ub = plot_ub.prepare();
            output_ub.clear();
            output_ub.addDouble(0.0);
            plot_ub.write();
            Bottle& output_vb = plot_vb.prepare();
            output_vb.clear();
            output_vb.addDouble(0.0);
            plot_vb.write();
			Bottle& output_xb = plot_xb.prepare();
            output_xb.clear();
            output_xb.addDouble(0.0);
            plot_xb.write();
			Bottle& output_yb = plot_yb.prepare();
            output_yb.clear();
            output_yb.addDouble(0.0);
            plot_yb.write();
			Bottle& output_zb = plot_zb.prepare();
            output_zb.clear();
            output_zb.addDouble(0.0);
            plot_zb.write();
            Bottle& output_calc_ub = plot_calc_ub.prepare();
            output_calc_ub.clear();
            output_calc_ub.addDouble(0.0);
            plot_calc_ub.write();
            Bottle& output_calc_vb = plot_calc_vb.prepare();
            output_calc_vb.clear();
            output_calc_vb.addDouble(0.0);
            plot_calc_vb.write();
			Bottle& output_calc_xb = plot_calc_xb.prepare();
            output_calc_xb.clear();
            output_calc_xb.addDouble(0.0);
            plot_calc_xb.write();
			Bottle& output_calc_yb = plot_calc_yb.prepare();
            output_calc_yb.clear();
            output_calc_yb.addDouble(0.0);
            plot_calc_yb.write();
			Bottle& output_calc_zb = plot_calc_zb.prepare();
            output_calc_zb.clear();
            output_calc_zb.addDouble(0.0);
            plot_calc_zb.write();
            Bottle& output_free_energy_b = plot_free_energy_b.prepare();
            output_free_energy_b.clear();
            output_free_energy_b.addDouble(0.0);
            plot_free_energy_b.write();

            //close ports
            plot_q1b.close();
            plot_q2b.close();
            plot_q3b.close();
            plot_q4b.close();
            plot_mu1b.close();
            plot_mu2b.close();
            plot_mu3b.close();
            plot_mu4b.close();
            plot_mu1bp.close();
            plot_mu2bp.close();
            plot_mu3bp.close();
            plot_mu4bp.close();
            plot_a1b.close();
            plot_a2b.close();
            plot_a3b.close();
            plot_a4b.close();
            plot_ub.close();
            plot_vb.close();
            plot_xb.close();
            plot_yb.close();
            plot_zb.close();
            plot_calc_ub.close();
            plot_calc_vb.close();
            plot_calc_xb.close();
            plot_calc_yb.close();
            plot_calc_zb.close();
            plot_free_energy_b.close();
        }


	}

	//delete objects
    if (right_arm_config.activated) delete iCubRightArmCtrl; //control and encoders
	delete iCubHeadCtrl;
    delete iCubTorsoCtrl;
    if (left_arm_config.activated) delete iCubLeftArmCtrl;
	delete iCubRightArmKin; //kinematics
	delete iCubLeftEyeKin;
    delete iCubLeftArmKin;
    delete iCubTorsoKin;
	if (right_arm_config.on_vision) delete iCubImgL; //camera
	if (right_arm_config.on_attractor) delete iCubImgLA; //attractor
    if (left_arm_config.on_vision) delete iCubImgLb; //camera
    if (right_arm_config.on_touch) delete iCubRightHandTouch; //touch
    if (left_arm_config.on_touch) delete iCubLeftHandTouch; //touch
    if (right_arm_config.activated) delete feoRightArm; //feo algorithm for right arm
	if (head_config.activated) delete feoHead; //feo algorithm for head
    if (left_arm_config.activated) delete feoLeftArm; //feo algorithm for left arm
	
	if (right_arm_config.activated) delete feoRightArmThread;
	if (head_config.activated) delete feoHeadThread;
    if (left_arm_config.activated) delete feoLeftArmThread;
	if (right_arm_config.activated) if (right_arm_config.on_action) delete actionRightArmThread;
	if (head_config.activated) if (head_config.on_action) delete actionHeadThread;
    if (left_arm_config.activated) if (left_arm_config.on_action) delete actionLeftArmThread;
    if (right_arm_config.on_vision || right_arm_config.on_attractor || left_arm_config.on_vision || left_arm_config.on_attractor) delete visionThread;
    if ((right_arm_config.on_touch && right_arm_config.activated) || (left_arm_config.on_touch && left_arm_config.activated)) delete touchThread;
    if (right_arm_config.gaussian_noise || left_arm_config.gaussian_noise) delete gen;
    if (right_arm_config.gaussian_noise || left_arm_config.gaussian_noise) delete ndis;
}

//----------------------------------------------------------------------------------
//main -----------------------------------------------------------------------------
//----------------------------------------------------------------------------------
int main()
{
	//Get date and time ---------------------------------------------------------------	

	get_date_time();	

	cout << "Active inference program started at: " << time_str << endl;

	//Read config files ---------------------------------------------------------------
	
	config_file_parser("config.ini"); //main config
	config_file_parser("ra_config.ini"); //right arm config
	config_file_parser("head_config.ini"); //head config
    config_file_parser("la_config.ini"); //left arm config
	
	print_config();
	
	prepare_variables();

	//Set interrupt handle ------------------------------------------------------------
	
	set_interrupt_handle();
	
	stage = read_config;

	while (cin.get() != 10){
		if (t_end) return 0;
	};
	
	//iCub encoders --------------------------------------------------------------------
	
	//Move to starting position and read encoders
	if (!init_encoders()) return -1; //exit if error

	//Setup robot parameters
	robot_parameters();
	
	//Kinematics library ---------------------------------------------------------------
	
	init_kinematics();
	
	stage = start_icub;
	
	while (cin.get() != 10){
		if (t_end) return 0;
	};

	//Plotting ports -------------------------------------------------------------------
	
	if (main_config.publish) init_publish();

	stage = start_publish;

	while (cin.get() != 10){
		if (t_end) return 0;
	};

	//Left eye camera ------------------------------------------------------------------
	
	if (right_arm_config.on_vision || right_arm_config.on_attractor) init_camera();

	stage = start_camera;
	
	while (cin.get() != 10){
		if (t_end) return 0;
	};
	
    //Right and left hand touch sensors ------------------------------------------------

    init_touch();

	stage = start_touch;

	while (cin.get() != 10){
		if (t_end) return 0;
	};

	//Gaussian noise generator initialization ------------------------------------------

	if (right_arm_config.gaussian_noise) init_noise();

	stage = start_noise;

	while (cin.get() != 10){
		if (t_end) return 0;
	};

	//Free energy optimization ---------------------------------------------------------
	//FEO: definitions -----------------------------------------------------------------
	
	cout << "** Setting up free energy optimization algorithm..." << endl;
	
	if (right_arm_config.activated) feo_right_arm_setup();
	if (head_config.activated) feo_head_setup();	
    if (left_arm_config.activated) feo_left_arm_setup();

	stage = start_feo;

	while (cin.get() != 10){
		if (t_end) return 0;
	};
	
	//FEO: executing -------------------------------------------------------------------
	
	cout << "** Executing free energy optimization algorithm..." << endl;
	
	stage = exec_feo;

    if (right_arm_config.activated)
    {
        cout << "Right arm free energy optimization activated!" << endl;
        feo_right_arm_run();
    }
    if (head_config.activated)
    {
        cout << "Head free energy optimization activated!" << endl;
        feo_head_run();
    }
    if (left_arm_config.activated)
    {
        cout << "Left arm free energy optimization activated!" << endl;
        feo_left_arm_run();
    }
	
	//Closure --------------------------------------------------------------------------

	//Waiting for the end of the thread cycles
	while(!t_end);
	//cin.ignore();
	
	if (right_arm_config.activated) end_right_arm();
    if (head_config.activated) end_head();
    if (left_arm_config.activated) end_left_arm();

	cout << "** Finished" << endl;
	
	clean_up();

	cout << "** End of execution" << endl;

	return 0;
}


