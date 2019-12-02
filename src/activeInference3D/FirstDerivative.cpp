/*
 * FirstDerivative.cpp
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

#include "FirstDerivative.h"

//----------------------------------------------------------------------------------
//First derivative terms -----------------------------------------------------------
//----------------------------------------------------------------------------------

//Right arm encoders ---------------------------------------------------------------
double read_encoders_q1(vector<double> mu, vector<double> a)
{
	//read only on first: arm, eye and torso
	qRAencoders = iCubRightArmCtrl->readEncoders();
	qLEencoders = iCubHeadCtrl->readEncoders();
	qTencoders = iCubTorsoCtrl->readEncoders();
	
	//convert to radians
	qRAencoders *= CTRL_DEG2RAD;
	qLEencoders *= CTRL_DEG2RAD;
	qTencoders *= CTRL_DEG2RAD;
	
	//Update values for calculations
	//Right arm
	ra_q1 = qTencoders[2]; //torso_pitch
	ra_q2 = qTencoders[1]; //torso_roll
	ra_q3 = qTencoders[0]; //torso_yaw
	ra_q4 = qRAencoders[0]; //r_shoulder_pitch
	ra_q5 = qRAencoders[1]; //r_shoulder_roll
	ra_q6 = qRAencoders[2]; //r_shoulder_yaw
	ra_q7 = qRAencoders[3]; //r_elbow
	ra_q8 = qRAencoders[4]; //r_wrist_prosup
	ra_q9 = qRAencoders[5]; //r_wrist_pitch
	ra_q10 = qRAencoders[6]; //r_wrist_yaw
	//Left eye
	le_q1 = qTencoders[2]; //torso_pitch
	le_q2 = qTencoders[1]; //torso_roll
	le_q3 = qTencoders[0]; //torso_yaw
	le_q4 = qLEencoders[0]; //neck_pitch
	le_q5 = qLEencoders[1]; //neck_roll
	le_q6 = qLEencoders[2]; //neck_yaw
	le_q7 = qLEencoders[3]; //eyes_tilt
	le_q8 = qLEencoders[4]; //eyes_version


	//update kinematic libraries
	iCubRightArmKin->setJointValues(qRAencoders);
	iCubLeftEyeKin->setJointValues(qLEencoders);
	iCubTorsoKin->setJointValues(qTencoders);
	
	if (publish)
	{
		Bottle& output_q1 = plot_q1.prepare();
		output_q1.clear();
		output_q1.addDouble(CTRL_RAD2DEG*qTencoders[0]);
		plot_q1.write();
	}

	return qTencoders[0]; //torso_yaw
}

double read_encoders_q1_noise(vector<double> mu, vector<double> a)
{
	double real = read_encoders_q1(mu, a);
	double noise = (*ndis)(*gen);

	return (real + noise); //torso_yaw + noise
}

double read_encoders_q2(vector<double> mu, vector<double> a)
{
	//read in previous function

	if (publish)
	{
		Bottle& output_q2 = plot_q2.prepare();
		output_q2.clear();
		output_q2.addDouble(CTRL_RAD2DEG*qRAencoders[1]);
		plot_q2.write();
	}	

	return qRAencoders[1]; //r_shoulder_roll
}

double read_encoders_q2_noise(vector<double> mu, vector<double> a)
{
	double real = read_encoders_q2(mu, a);
	double noise = (*ndis)(*gen);

	return (real + noise); //r_shoulder_roll + noise
}

double read_encoders_q3(vector<double> mu, vector<double> a)
{
	//read in previous function

	if (publish)
	{
		Bottle& output_q3 = plot_q3.prepare();
		output_q3.clear();
		output_q3.addDouble(CTRL_RAD2DEG*qRAencoders[2]);
		plot_q3.write();
	}	

	return qRAencoders[2]; //r_shoulder_yaw
}

double read_encoders_q3_noise(vector<double> mu, vector<double> a)
{
	double real = read_encoders_q3(mu, a);
	double noise = (*ndis)(*gen);

	return (real + noise); //r_shoulder_yaw + noise
}

double read_encoders_q4(vector<double> mu, vector<double> a)
{
	//read in previous function

	if (publish)
	{
		Bottle& output_q4 = plot_q4.prepare();
		output_q4.clear();
		output_q4.addDouble(CTRL_RAD2DEG*qRAencoders[3]);
		plot_q4.write();
	}	

	return qRAencoders[3]; //r_elbow
}

double read_encoders_q4_noise(vector<double> mu, vector<double> a)
{
	double real = read_encoders_q4(mu, a);
	double noise = (*ndis)(*gen);

	return (real + noise); //r_elbow + noise
}

//Right arm internal state ---------------------------------------------------------
double internal_state_mu1(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu1 = plot_mu1.prepare();
		output_mu1.clear();
		output_mu1.addDouble(CTRL_RAD2DEG*mu[0]);
		plot_mu1.write();
	}	

	return mu[0]; //torso_yaw
}

double internal_state_mu2(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu2 = plot_mu2.prepare();
		output_mu2.clear();
		output_mu2.addDouble(CTRL_RAD2DEG*mu[1]);
		plot_mu2.write();
	}
	
	return mu[1]; //r_shoulder_roll
}

double internal_state_mu3(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu3 = plot_mu3.prepare();
		output_mu3.clear();
		output_mu3.addDouble(CTRL_RAD2DEG*mu[2]);
		plot_mu3.write();
	}
	
	return mu[2]; //r_shoulder_yaw
}

double internal_state_mu4(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu4 = plot_mu4.prepare();
		output_mu4.clear();
		output_mu4.addDouble(CTRL_RAD2DEG*mu[3]);
		plot_mu4.write();
	}
	
	return mu[3]; //r_elbow
}

//Head encoders --------------------------------------------------------------------
double read_encoders_q1e(vector<double> mu, vector<double> a)
{
	//read only on first, arm and eye
	qLEencoders = iCubHeadCtrl->readEncoders();
	qTencoders = iCubTorsoCtrl->readEncoders();
	
	//convert to radians
	qLEencoders *= CTRL_DEG2RAD;
	qTencoders *= CTRL_DEG2RAD;
	
	//Update values for calculations
	//Left eye
	le_q1 = qTencoders[2]; //torso_pitch
	le_q2 = qTencoders[1]; //torso_roll
	le_q3 = qTencoders[0]; //torso_yaw
	le_q4 = qLEencoders[0]; //neck_pitch
	le_q5 = qLEencoders[1]; //neck_roll
	le_q6 = qLEencoders[2]; //neck_yaw
	le_q7 = qLEencoders[3]; //eyes_tilt
	le_q8 = qLEencoders[4]; //eyes_version

	//update kinematic libraries
	iCubLeftEyeKin->setJointValues(qLEencoders);
	iCubTorsoKin->setJointValues(qTencoders);
	
	if (publish)
	{
		Bottle& output_q1e = plot_q1e.prepare();
		output_q1e.clear();
		output_q1e.addDouble(CTRL_RAD2DEG*qLEencoders[0]);
		plot_q1e.write();
	}

	return qLEencoders[0]; //neck_pitch
}

double read_encoders_q2e(vector<double> mu, vector<double> a)
{
	//read in previous function

	if (publish)
	{
		Bottle& output_q2e = plot_q2e.prepare();
		output_q2e.clear();
		output_q2e.addDouble(CTRL_RAD2DEG*qLEencoders[2]);
		plot_q2e.write();
	}	

	return qLEencoders[2]; //neck_yaw
}

double read_encoders_q3e(vector<double> mu, vector<double> a)
{
    //read in previous function

    if (publish)
    {
        Bottle& output_q3e = plot_q3e.prepare();
        output_q3e.clear();
        output_q3e.addDouble(CTRL_RAD2DEG*qLEencoders[3]);
        plot_q3e.write();
    }

    return qLEencoders[3]; //eyes_tilt
}

//Head internal state ---------------------------------------------------------
double internal_state_mu1e(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu1e = plot_mu1e.prepare();
		output_mu1e.clear();
		output_mu1e.addDouble(CTRL_RAD2DEG*mu[0]);
		plot_mu1e.write();
	}	

	return mu[0]; //neck_pitch
}

double internal_state_mu2e(vector<double> mu, vector<double> a)
{
	if (publish){
		Bottle& output_mu2e = plot_mu2e.prepare();
		output_mu2e.clear();
		output_mu2e.addDouble(CTRL_RAD2DEG*mu[1]);
		plot_mu2e.write();
	}
	
	return mu[1]; //neck_yaw
}

double internal_state_mu3e(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu3e = plot_mu3e.prepare();
        output_mu3e.clear();
        output_mu3e.addDouble(CTRL_RAD2DEG*mu[2]);
        plot_mu3e.write();
    }

    return mu[2]; //eyes_tilt
}

//3D position ----------------------------------------------------------------------
//Right arm
double sense_3d_position_x(vector<double> mu, vector<double> a)
{
	//both eyes stereoscopic 3D position of end-effector
	
    //calculate left eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix(dof_le, dof_le);

    //obtain world coordinates
    Vector4d re_buf;
    re_buf[0] = re_pos[0]*k; re_buf[1] = re_pos[1]*k; re_buf[2] = re_pos[2]*k; re_buf[3] = 1;
    re_buf = le_rtm*re_buf;

    //assign
    if (re_pos[0] == 0) //if not sensed
    {
        re_xpos[0] = 0; re_xpos[1] = 0; re_xpos[2] = 0;
    }else{
        re_xpos[0] = re_buf[0]/k;
        re_xpos[1] = re_buf[1]/k;
        re_xpos[2] = re_buf[2]/k;
    }

    return re_xpos[0];
}

double sense_3d_position_y(vector<double> mu, vector<double> a)
{
    return re_xpos[1];
}

double sense_3d_position_z(vector<double> mu, vector<double> a)
{
    return re_xpos[2];
}

//Attractor
double sense_3d_attractor_x(vector<double> mu, vector<double> a)
{
    //both eyes stereoscopic 3D position of attractor

    //calculate left eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix(dof_le, dof_le);

    //obtain world coordinates
    Vector4d att_buf;
    att_buf[0] = att_pos[0]*k; att_buf[1] = att_pos[1]*k; att_buf[2] = att_pos[2]*k; att_buf[3] = 1;
    att_buf = le_rtm*att_buf;

    //assign
    att_xpos[0] = att_buf[0]/k;
    att_xpos[1] = att_buf[1]/k;
    att_xpos[2] = att_buf[2]/k;

    return att_xpos[0];
}

double sense_3d_attractor_y(vector<double> mu, vector<double> a)
{
    return att_xpos[1];
}

double sense_3d_attractor_z(vector<double> mu, vector<double> a)
{
    return att_xpos[2];
}



//Left arm
double sense_3d_position_x_b(vector<double> mu, vector<double> a)
{
	//both eyes stereoscopic 3D position of end-effector
	
    //calculate left eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix(dof_le, dof_le);

    //calculate the inverse of the left eye rototranslational matrix
    le_rtm_inv = le_rtm.inverse();

    //obtain world coordinates
    Vector4d le_buf;
    le_buf[0] = le_pos[0]*k; le_buf[1] = le_pos[1]*k; le_buf[2] = le_pos[2]*k; le_buf[3] = 1;
    le_buf = le_rtm_inv*le_buf;

    //assign
    le_xpos[0] = le_buf[0]/k;
    le_xpos[1] = le_buf[1]/k;
    le_xpos[2] = le_buf[2]/k;

    return le_xpos[0];
}

double sense_3d_position_y_b(vector<double> mu, vector<double> a)
{
    return le_xpos[1];
}

double sense_3d_position_z_b(vector<double> mu, vector<double> a)
{
    return le_xpos[2];
}

//Left arm encoders ---------------------------------------------------------------
double read_encoders_q1b(vector<double> mu, vector<double> a)
{
    //read only on first, arm and eye
    qLAencoders = iCubLeftArmCtrl->readEncoders();
    qLEencoders = iCubHeadCtrl->readEncoders();
	qTencoders = iCubTorsoCtrl->readEncoders();

    //convert to radians
    qLAencoders *= CTRL_DEG2RAD;
    qLEencoders *= CTRL_DEG2RAD;
	qTencoders *= CTRL_DEG2RAD;

    //Update values for calculations
    //Left arm
	la_q1 = qTencoders[2]; //torso_pitch
	la_q2 = qTencoders[1]; //torso_roll
	la_q3 = qTencoders[0]; //torso_yaw
    la_q4 = qLAencoders[0]; //l_shoulder_pitch
    la_q5 = qLAencoders[1]; //l_shoulder_roll
    la_q6 = qLAencoders[2]; //l_shoulder_yaw
    la_q7 = qLAencoders[3]; //l_elbow
    la_q8 = qLAencoders[4]; //l_wrist_prosup
    la_q9 = qLAencoders[5]; //l_wrist_pitch
    la_q10 = qLAencoders[6]; //l_wrist_yaw
    //Left eye
	le_q1 = qTencoders[2]; //torso_pitch
	le_q2 = qTencoders[1]; //torso_roll
	le_q3 = qTencoders[0]; //torso_yaw
	le_q4 = qLEencoders[0]; //neck_pitch
	le_q5 = qLEencoders[1]; //neck_roll
	le_q6 = qLEencoders[2]; //neck_yaw
	le_q7 = qLEencoders[3]; //eyes_tilt
	le_q8 = qLEencoders[4]; //eyes_version


    //update kinematic libraries
    iCubLeftArmKin->setJointValues(qLAencoders);
    iCubLeftEyeKin->setJointValues(qLEencoders);
	iCubTorsoKin->setJointValues(qTencoders);

    if (publish)
    {
        Bottle& output_q1b = plot_q1b.prepare();
        output_q1b.clear();
        output_q1b.addDouble(CTRL_RAD2DEG*qTencoders[0]);
        plot_q1b.write();
    }

    return qTencoders[0]; //torso_yaw
}

double read_encoders_q1b_noise(vector<double> mu, vector<double> a)
{
    double real = read_encoders_q1b(mu, a);
    double noise = (*ndis)(*gen);

    return (real + noise); //torso_yaw + noise
}

double read_encoders_q2b(vector<double> mu, vector<double> a)
{
    //read in previous function

    if (publish)
    {
        Bottle& output_q2b = plot_q2b.prepare();
        output_q2b.clear();
        output_q2b.addDouble(CTRL_RAD2DEG*qLAencoders[1]);
        plot_q2b.write();
    }

    return qLAencoders[1]; //l_shoulder_roll
}

double read_encoders_q2b_noise(vector<double> mu, vector<double> a)
{
    double real = read_encoders_q2b(mu, a);
    double noise = (*ndis)(*gen);

    return (real + noise); //l_shoulder_roll + noise
}

double read_encoders_q3b(vector<double> mu, vector<double> a)
{
    //read in previous function

    if (publish)
    {
        Bottle& output_q3b = plot_q3b.prepare();
        output_q3b.clear();
        output_q3b.addDouble(CTRL_RAD2DEG*qLAencoders[2]);
        plot_q3b.write();
    }

    return qLAencoders[2]; //l_shoulder_yaw
}

double read_encoders_q3b_noise(vector<double> mu, vector<double> a)
{
    double real = read_encoders_q3b(mu, a);
    double noise = (*ndis)(*gen);

    return (real + noise); //l_shoulder_yaw + noise
}

double read_encoders_q4b(vector<double> mu, vector<double> a)
{
    //read in previous function

    if (publish)
    {
        Bottle& output_q4b = plot_q4b.prepare();
        output_q4b.clear();
        output_q4b.addDouble(CTRL_RAD2DEG*qLAencoders[3]);
        plot_q4b.write();
    }

    return qLAencoders[3]; //l_elbow
}

double read_encoders_q4b_noise(vector<double> mu, vector<double> a)
{
    double real = read_encoders_q4b(mu, a);
    double noise = (*ndis)(*gen);

    return (real + noise); //l_elbow + noise
}

//Left arm internal state ---------------------------------------------------------
double internal_state_mu1b(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu1b = plot_mu1b.prepare();
        output_mu1b.clear();
        output_mu1b.addDouble(CTRL_RAD2DEG*mu[0]);
        plot_mu1b.write();
    }

    return mu[0]; //torso_yaw
}

double internal_state_mu2b(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu2b = plot_mu2b.prepare();
        output_mu2b.clear();
        output_mu2b.addDouble(CTRL_RAD2DEG*mu[1]);
        plot_mu2b.write();
    }

    return mu[1]; //l_shoulder_roll
}

double internal_state_mu3b(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu3b = plot_mu3b.prepare();
        output_mu3b.clear();
        output_mu3b.addDouble(CTRL_RAD2DEG*mu[2]);
        plot_mu3b.write();
    }

    return mu[2]; //l_shoulder_yaw
}

double internal_state_mu4b(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_mu4b = plot_mu4b.prepare();
        output_mu4b.clear();
        output_mu4b.addDouble(CTRL_RAD2DEG*mu[3]);
        plot_mu4b.write();
    }

    return mu[3]; //l_elbow
}

//3D right arm rototranslational matrix ---------------------------------------
void calc_3d_rototranslational_matrix(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + mu[0]; //torso_yaw
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[1]; //r_shoulder_roll
	double q6 = ra_q6b + mu[2]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[3]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11
	ra_rtm(0,0) = Sin(q10)*(-(Cos(q8)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q5)*
                  (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                       Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) \
+ (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
               Sin(q6)) + (-(Cos(q5)*
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
                  Cos(q2)*Sin(q1)*Sin(q3))) + 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7))*Sin(q9));
	
	//H21
	ra_rtm(1,0) = Sin(q10)*(-(Cos(q8)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                 Sin(q2)*Sin(q3)*Sin(q5)) + 
              (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
           (Cos(q5)*Sin(q2)*Sin(q3) + 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) + (Cos(q6)*
            (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
      Sin(q9));
	
	//H31
	ra_rtm(2,0) = Sin(q10)*(-(Cos(q8)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))
 + (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q5)*
                  (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                       Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))
+ (Cos(q1)*Cos(q4)*Sin(q2) - (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*
                  Sin(q4))*Sin(q6)) + 
           (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
                  Cos(q1)*Cos(q2)*Sin(q3))) + 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H41
	ra_rtm(3,0) = 0;
	
	//H12
	ra_rtm(0,1) = Cos(q10)*(-(Cos(q8)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q5)*
                  (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                       Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))
+ (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
               Sin(q6)) + (-(Cos(q5)*
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
                  Cos(q2)*Sin(q1)*Sin(q3))) + 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7))*Sin(q9));
	
	//H22
	ra_rtm(1,1) = Cos(q10)*(-(Cos(q8)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                 Sin(q2)*Sin(q3)*Sin(q5)) + 
              (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
           (Cos(q5)*Sin(q2)*Sin(q3) + 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) + (Cos(q6)*
            (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
      Sin(q9));
	
	//H32
	ra_rtm(2,1) = Cos(q10)*(-(Cos(q8)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))
 + (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q5)*
                  (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                       Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) 
+ (Cos(q1)*Cos(q4)*Sin(q2) - (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*
                  Sin(q4))*Sin(q6)) + 
           (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
                  Cos(q1)*Cos(q2)*Sin(q3))) + 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H42
	ra_rtm(3,1) = 0;
	
	//H13
	ra_rtm(0,2) = -(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*
               (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
            (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
               Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
       (Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))*Sin(q7))) + 
  (Cos(q8)*(Cos(q7)*(Cos(q6)*(Cos(q5)*
               (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                    Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H23
	ra_rtm(1,2) = -(Cos(q9)*(-(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
            (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
       (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5)) + 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))) + 
  (Cos(q8)*(Cos(q7)*(Cos(q6)*(Cos(q5)*
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) +
 (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8))*Sin(q9);
	
	//H33
	ra_rtm(2,2) = -(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*
               (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
            (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
               Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
       (Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
        Sin(q7))) + (Cos(q8)*(Cos(q7)*
         (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) +
 (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H43
	ra_rtm(3,2) = 0;
	
	//H14
	ra_rtm(0,3) = d2*Cos(q1) - a1*Sin(q1) - a3*Cos(q2)*Cos(q3)*Sin(q1) - d3*Sin(q1)*Sin(q2) + 
  a3*Cos(q1)*Sin(q3) + d4*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) + 
  a6*Cos(q6)*(Cos(q5)*(Cos(q4)*
         (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
  d6*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a6*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
     (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q5)*
         (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
     (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6)) + 
  a7*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
               Cos(q2)*Sin(q1)*Sin(q3))) + 
          (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)) + a10*Sin(q10)*(-(Cos(q8)*
        (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(Cos(q8)*
         (Cos(q7)*(Cos(q6)*(Cos(q5)*
                  (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                       Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) +
 (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
               Sin(q6)) + (-(Cos(q5)*
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
                  Cos(q2)*Sin(q1)*Sin(q3))) + 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7))*Sin(q9)) + 
  d10*(-(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*
                  (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
               (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
          (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))*Sin(q7))) + 
     (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q5)*(Cos(q4)*
                     (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                    Sin(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) +
 (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
               Sin(q6)) + (-(Cos(q5)*
                 (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8))*Sin(q9));
	
	//H24
	ra_rtm(1,3) = d3*Cos(q2) - a3*Cos(q3)*Sin(q2) - d4*Sin(q2)*Sin(q3) + 
  a6*Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
     Sin(q2)*Sin(q3)*Sin(q5)) + 
  d6*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
  a6*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q5)*
         (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5)) + 
     (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
  a7*(Cos(q5)*Sin(q2)*Sin(q3) + 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
          (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
     (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)) + 
  a10*Sin(q10)*(-(Cos(q8)*(Cos(q6)*
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                    Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
              (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
           (Cos(q5)*Sin(q2)*Sin(q3) + 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) + (Cos(q6)*
            (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
          (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))) +
 (Cos(q8)*(Cos(q7)*(Cos(q6)*(Cos(q5)*
                  (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                 Sin(q2)*Sin(q3)*Sin(q5)) + 
              (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
           (Cos(q5)*Sin(q2)*Sin(q3) + 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) + (Cos(q6)*
            (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8))*Sin(q9));
	
	//H34
	ra_rtm(2,3) = a1*Cos(q1) + a3*Cos(q1)*Cos(q2)*Cos(q3) + d2*Sin(q1) + d3*Cos(q1)*Sin(q2) + 
  a3*Sin(q1)*Sin(q3) + d4*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) + 
  a6*Cos(q6)*(Cos(q5)*(Cos(q4)*
         (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
  d6*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a6*(Cos(q1)*Cos(q4)*Sin(q2) - 
     (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q5)*
         (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
     (Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) + 
  a7*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
               Cos(q1)*Cos(q2)*Sin(q3))) + 
          (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)) + a10*Sin(q10)*(-(Cos(q8)*
        (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))
 + (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(Cos(q8)*
         (Cos(q7)*(Cos(q6)*(Cos(q5)*
                  (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                       Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) +
 (Cos(q1)*Cos(q4)*Sin(q2) - 
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
               Sin(q6)) + (-(Cos(q5)*
                 (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
                  Cos(q1)*Cos(q2)*Sin(q3))) + 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))
	*Sin(q7))*Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q7)*(-(Cos(q5)*
                  (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
               (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
          (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
             (Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))*Sin(q7))) + 
     (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q5)*(Cos(q4)*
                     (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                    Cos(q1)*Sin(q2)*Sin(q4)) + 
                 (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) +
 (Cos(q1)*Cos(q4)*Sin(q2) - (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*
                  Sin(q4))*Sin(q6)) + 
           (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
        (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8))*Sin(q9));
	
	ra_rtm(3,3) = 1;
}

//3D right arm rototranslational matrix derivative wrt mu1 --------------------
void calc_d_3d_rototranslational_matrix_mu1(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + mu[0]; //torso_yaw
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[1]; //r_shoulder_roll
	double q6 = ra_q6b + mu[2]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[3]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d4
	ra_d1rtm(0,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
         Sin(q7))*Sin(q8)) + Cos(q10)*
   (Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) \
- (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
           (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*
              (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                  Cos(q1)*Sin(q3))) + 
             Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) \
+ (Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H21d4
	ra_d1rtm(1,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
              Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) \
+ (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
                 Cos(q3)*Sin(q2)*Sin(q5)) - 
              Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) + 
           (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q3)*Cos(q5)*Sin(q2) + 
             Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))) + 
        (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
              Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H31d4
	ra_d1rtm(2,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))) \
+ (Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
         Sin(q7))*Sin(q8)) + Cos(q10)*
   (Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) \
+ (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
         Sin(q8)) + (-(Cos(q7)*
           (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
             Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) \
+ (Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H41d4
	ra_d1rtm(3,0) = 0;
	
	//H12d4
	ra_d1rtm(0,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
         Sin(q7))*Sin(q8)) - Sin(q10)*
   (Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) \
- (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
           (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*
              (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                  Cos(q1)*Sin(q3))) + 
             Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) \
+ (Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H22d4
	ra_d1rtm(1,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
              Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) \
+ (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
                 Cos(q3)*Sin(q2)*Sin(q5)) - 
              Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) + 
           (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q3)*Cos(q5)*Sin(q2) + 
             Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))) + 
        (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
              Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H32d4
	ra_d1rtm(2,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))) \
+ (Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
         Sin(q7))*Sin(q8)) - Sin(q10)*
   (Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) \
+ (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
         Sin(q8)) + (-(Cos(q7)*
           (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
             Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) \
+ (Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H42d4
	ra_d1rtm(3,1) = 0;
	
	//H13d4
	ra_d1rtm(0,2) = -(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
            Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
       (Cos(q6)*(Cos(q4)*Cos(q5)*
              (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
          (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6))*
        Sin(q7))) + (Cos(q8)*(Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
         Sin(q7)) + (-(Cos(q6)*
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H23d4
	ra_d1rtm(1,2) = -(Cos(q9)*(-(Cos(q7)*(Cos(q3)*Cos(q5)*Sin(q2) + 
            Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))) + 
       (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
             Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6))*
        Sin(q7))) + (Cos(q8)*(Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
              Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) \
+ (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) + 
     (-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
        (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*Sin(q6)\
)*Sin(q8))*Sin(q9);
	
	//H33d4
	ra_d1rtm(2,2) = -(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
            Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
       (Cos(q6)*(Cos(q4)*Cos(q5)*
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
          (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6))*
        Sin(q7))) + (Cos(q8)*(Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
         Sin(q7)) + (-(Cos(q6)*
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)) - 
        (Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H43d4
	ra_d1rtm(3,2) = 0;
	
	//H14d4
	ra_d1rtm(0,3) = a3*Cos(q1)*Cos(q3) + a3*Cos(q2)*Sin(q1)*Sin(q3) + 
  d4*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) + 
  a6*Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
     (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) + 
  d6*(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
     Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
  a6*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*
         (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
     (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
  a7*(-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
     Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
               Cos(q1)*Sin(q3))) + 
          Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
     (Cos(q6)*(Cos(q4)*Cos(q5)*
            (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
        (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6))*Sin(q7)\
) + a10*Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
         Sin(q7))*Sin(q8)) + a10*Cos(q10)*
   (Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) \
- (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
           (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)\
)*Sin(q7)) + (-(Cos(q6)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*
              Sin(q4)) - (Cos(q4)*Cos(q5)*
               (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                  Cos(q1)*Sin(q3))) + 
             Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) \
+ (Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + 
                 Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9)) + d10*
   (-(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
               Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))\
) + (Cos(q6)*(Cos(q4)*Cos(q5)*
                 (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) - 
             (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6))*
           Sin(q7))) + (Cos(q8)*
         (Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5)) \
- (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)*Sin(q6)) + 
           (-(Cos(q5)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*
              (Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*(Cos(q1)*Cos(q3) + Cos(q2)*Sin(q1)*Sin(q3)) + 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8))*Sin(q9));
	
	//H24d4
	ra_d1rtm(1,3) = -(d4*Cos(q3)*Sin(q2)) + a3*Sin(q2)*Sin(q3) + 
  a6*Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5)) + 
  d6*(-(Cos(q3)*Cos(q5)*Sin(q2)) - Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5)) - 
  a6*Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
        Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) + 
  a7*(Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(Cos(q3)*Cos(q5)*Sin(q2) + 
          Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))) + 
     (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
           Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6))*
      Sin(q7)) + a10*Sin(q10)*(-(Cos(q8)*
        (-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*
           Sin(q6))) + (Cos(q7)*
         (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
              Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) \
+ (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q7)*(Cos(q6)*
               (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
                 Cos(q3)*Sin(q2)*Sin(q5)) - 
              Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) + 
           (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q3)*Cos(q5)*Sin(q2) + 
             Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))) + 
        (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
              Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9)) + d10*
   (-(Cos(q9)*(-(Cos(q7)*(Cos(q3)*Cos(q5)*Sin(q2) + 
               Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))) + 
          (Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
                Cos(q3)*Sin(q2)*Sin(q5)) - Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6))*
           Sin(q7))) + (Cos(q8)*
         (Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - 
                 Cos(q3)*Sin(q2)*Sin(q5)) - 
              Sin(q2)*Sin(q3)*Sin(q4)*Sin(q6)) + 
           (Cos(q3)*Cos(q5)*Sin(q2) + Cos(q4)*Sin(q2)*Sin(q3)*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*Sin(q2)*Sin(q3)*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*Sin(q2)*Sin(q3) - Cos(q3)*Sin(q2)*Sin(q5))*
            Sin(q6))*Sin(q8))*Sin(q9));
	
	//H34d4
	ra_d1rtm(2,3) = a3*Cos(q3)*Sin(q1) - a3*Cos(q1)*Cos(q2)*Sin(q3) + 
  d4*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
  d6*(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) - 
     Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
  a6*Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
     (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
  a6*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*
         (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
     (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) + 
  a7*(-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
     Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
          Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
     (Cos(q6)*(Cos(q4)*Cos(q5)*
            (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
        (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6))*Sin(q7)\
) + a10*Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)) - 
          (Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))) \
+ (Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*
               (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) + 
        (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
           Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
         Sin(q7))*Sin(q8)) + a10*Cos(q10)*
   (Cos(q9)*(Cos(q8)*(Cos(q7)*
            (Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)\
) + (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)\
)*Sin(q7)) + (-(Cos(q6)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*
              Sin(q4)) - (Cos(q4)*Cos(q5)*
               (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)\
)*Sin(q8)) + (-(Cos(q7)*(-(Cos(q5)*
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
             Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) \
+ (Cos(q6)*(Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - 
                 Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
           (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6))*
         Sin(q7))*Sin(q9)) + d10*
   (-(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
               Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))\
) + (Cos(q6)*(Cos(q4)*Cos(q5)*
                 (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
             (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6))*
           Sin(q7))) + (Cos(q8)*
         (Cos(q7)*(Cos(q6)*(Cos(q4)*Cos(q5)*
                  (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5)) - 
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)*Sin(q6)) \
+ (-(Cos(q5)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))) + 
              Cos(q4)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q7)) + (-(Cos(q6)*
              (Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3))*Sin(q4)) - 
           (Cos(q4)*Cos(q5)*(Cos(q3)*Sin(q1) - Cos(q1)*Cos(q2)*Sin(q3)) + 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
         Sin(q8))*Sin(q9));
	
	//H44d4
	ra_d1rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu2 --------------------
void calc_d_3d_rototranslational_matrix_mu2(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + mu[0]; //torso_yaw
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[1]; //r_shoulder_roll
	double q6 = ra_q6b + mu[2]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[3]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d1
	ra_d2rtm(0,0) = Sin(q10)*(Cos(q8)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
           Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H21d1
	ra_d2rtm(1,0) = Sin(q10)*(Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H31d1
	ra_d2rtm(2,0) = Sin(q10)*(Cos(q8)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
           Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H41d1
	ra_d2rtm(3,0) = 0;
	
	//H12d1
	ra_d2rtm(0,1) = Cos(q10)*(Cos(q8)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
           Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H22d1
	ra_d2rtm(1,1) = Cos(q10)*(Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H32d1
	ra_d2rtm(2,1) = Cos(q10)*(Cos(q8)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
           Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H42d1
	ra_d2rtm(3,1) = 0;
	
	//H13d1
	ra_d2rtm(0,2) = -(Cos(q9)*(-(Cos(q7)*(Cos(q5)*
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
               Sin(q1)*Sin(q2)*Sin(q4)) + 
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
       Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
          (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
  (Cos(q8)*(Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7)) \
- (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8))*Sin(q9);
	
	//H23d1
	ra_d2rtm(1,2) = -(Cos(q9)*(-(Cos(q7)*(Cos(q5)*
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
            Sin(q2)*Sin(q3)*Sin(q5))) + 
       Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
          (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))) \
+ (Cos(q8)*(Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) - 
     (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q8))*Sin(q9);
	
	//H33d1
	ra_d2rtm(2,2) = -(Cos(q9)*(-(Cos(q7)*(Cos(q5)*
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
               Cos(q1)*Sin(q2)*Sin(q4)) + 
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
       Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
          (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
  (Cos(q8)*(Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7)) \
- (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8))*Sin(q9);
	
	//H43d1
	ra_d2rtm(3,2) = 0;
	
	//H14d1
	ra_d2rtm(0,3) = d6*(-(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
          Sin(q1)*Sin(q2)*Sin(q4))) - 
     (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
  a6*Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a7*Cos(q6)*Cos(q7)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
        Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a7*(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(Cos(q5)*(Cos(q4)*
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
     Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q8)*(Cos(q5)*
         (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9)) + 
  d10*(-(Cos(q9)*(-(Cos(q7)*(Cos(q5)*
                (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) + 
          Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
                Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
     (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8))*Sin(q9));
	
	//H24d1
	ra_d2rtm(1,3) = d6*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
     Sin(q2)*Sin(q3)*Sin(q5)) + 
  a6*Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
  a7*Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
  a7*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
     Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))) + 
     Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(Cos(q8)*
         (Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                  Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5))) + 
          Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)\
)) + (Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
         Sin(q8))*Sin(q9));
	
	//H34d1
	ra_d2rtm(2,3) = d6*(-(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
          Cos(q1)*Sin(q2)*Sin(q4))) - 
     (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
  a6*Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a7*Cos(q6)*Cos(q7)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
        Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a7*(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(Cos(q5)*(Cos(q4)*
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
     Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q8)*(Cos(q5)*
         (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9)) + 
  d10*(-(Cos(q9)*(-(Cos(q7)*(Cos(q5)*
                (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) + 
          Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
                Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
     (Cos(q8)*(Cos(q6)*Cos(q7)*
            (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q7)) - (Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8))*Sin(q9));
	
	//H44d1
	ra_d2rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu3 --------------------
void calc_d_3d_rototranslational_matrix_mu3(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + mu[0]; //torso_yaw
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[1]; //r_shoulder_roll
	double q6 = ra_q6b + mu[2]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[3]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d3
	ra_d3rtm(0,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q5)*(Cos(q4)*
                   (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))) + Cos(q7)*
      (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(Cos(q6)*
            (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q8)) + 
     (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H21d3
	ra_d3rtm(1,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
               Sin(q2)*Sin(q3)*Sin(q5))) - 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) + 
     Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(Cos(q7)*Cos(q8)*
         (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8)) + 
     (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q9));
	
	//H31d3
	ra_d3rtm(2,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q5)*(Cos(q4)*
                   (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))) \
+ Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(Cos(q6)*
            (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q8)) + (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H41d3
	ra_d3rtm(3,0) = 0;
	
	//H12d3
	ra_d3rtm(0,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q5)*(Cos(q4)*
                   (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))) + Cos(q7)*
      (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(Cos(q6)*
            (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q8)) + 
     (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H22d3
	ra_d3rtm(1,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
               Sin(q2)*Sin(q3)*Sin(q5))) - 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) + 
     Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(Cos(q7)*Cos(q8)*
         (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8)) + 
     (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q9));
	
	//H32d3
	ra_d3rtm(2,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q5)*(Cos(q4)*
                   (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))) \
+ Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(Cos(q6)*
            (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q8)) + (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H42d3
	ra_d3rtm(3,1) = 0;
	
	//H13d3
	ra_d3rtm(0,2) = -(Cos(q9)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
          (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
     Sin(q7)) + (Cos(q7)*Cos(q8)*
      (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)) \
+ (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H23d3
	ra_d3rtm(1,2) = -(Cos(q9)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
       (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)) + 
  (Cos(q7)*Cos(q8)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)) + 
     (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8))*Sin(q9);
	
	//H33d3
	ra_d3rtm(2,2) = -(Cos(q9)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
          (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
     Sin(q7)) + (Cos(q7)*Cos(q8)*
      (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)) \
+ (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H43d3
	ra_d3rtm(3,2) = 0;
	
	//H14d3
	ra_d3rtm(0,3) = a6*Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
     (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
  a6*(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)) + 
  d8*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7) + a10*Sin(q10)*(-(Cos(q8)*
        (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                   (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))) + Cos(q7)*
      (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(Cos(q6)*
            (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) \
- (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q8)) + 
     (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9)) + d10*(-(Cos(q9)*
        (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     (Cos(q7)*Cos(q8)*(Cos(q6)*
            (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q8))*Sin(q9));
	
	//H24d3
	ra_d3rtm(1,3) = a6*Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
  a6*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
     Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
     (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)) + 
  d8*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
     (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7) + 
  a10*Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
               Sin(q2)*Sin(q3)*Sin(q5))) - 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) + 
     Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(Cos(q7)*Cos(q8)*
         (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8)) + 
     (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q9)) + 
  d10*(-(Cos(q9)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)) + 
     (Cos(q7)*Cos(q8)*(Cos(q6)*
            (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8))*
      Sin(q9));
	
	//H34d3
	ra_d3rtm(2,3) = a6*Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
     (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
  a6*(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6) + 
  a7*Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)) + 
  d8*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7) + a10*Sin(q10)*(-(Cos(q8)*
        (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                   (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))) \
+ Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(Cos(q6)*
            (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) \
- (Cos(q1)*Cos(q4)*Sin(q2) - (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*
               Sin(q4))*Sin(q6))*Sin(q8)) + 
     (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9)) + d10*(-(Cos(q9)*
        (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     (Cos(q7)*Cos(q8)*(Cos(q6)*
            (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6)) + (-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q8))*Sin(q9));
	
	//H44d3
	ra_d3rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu4 --------------------
void calc_d_3d_rototranslational_matrix_mu4(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + mu[0]; //torso_yaw
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[1]; //r_shoulder_roll
	double q6 = ra_q6b + mu[2]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[3]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d2
	ra_d4rtm(0,0) = Sin(q10)*(Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
             Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6)\
)*Sin(q7))*Sin(q8) + Cos(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
                Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7)) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H21d2
	ra_d4rtm(1,0) = Sin(q10)*(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
   Sin(q8) + Cos(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H31d2
	ra_d4rtm(2,0) = Sin(q10)*(Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
             Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7))*Sin(q8) + Cos(q10)*
   (Cos(q8)*Cos(q9)*(Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7)) + (Cos(q7)*(Cos(q6)*
            (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H41d2
	ra_d4rtm(3,0) = 0;
	
	//H12d2
	ra_d4rtm(0,1) = Cos(q10)*(Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
             Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6)\
)*Sin(q7))*Sin(q8) - Sin(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
                Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7)) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H22d2
	ra_d4rtm(1,1) = Cos(q10)*(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
   Sin(q8) - Sin(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H32d2
	ra_d4rtm(2,1) = Cos(q10)*(Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
             Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7))*Sin(q8) - Sin(q10)*
   (Cos(q8)*Cos(q9)*(Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7)) + (Cos(q7)*(Cos(q6)*
            (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H42d2
	ra_d4rtm(3,1) = 0;
	
	//H13d2
	ra_d4rtm(0,2) = -(Cos(q9)*(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6)) + (-(Cos(q5)*
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
          (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
  Cos(q8)*(Cos(q7)*(-(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7))*Sin(q9);
	
	//H23d2
	ra_d4rtm(1,2) = -(Cos(q9)*(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5)) + 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
       (Cos(q5)*Sin(q2)*Sin(q3) + 
          (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))) \
+ Cos(q8)*(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*Sin(q9);
	
	//H33d2
	ra_d4rtm(2,2) = -(Cos(q9)*(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
          (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
  Cos(q8)*(Cos(q7)*(-(Cos(q5)*
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7))*Sin(q9);
	
	//H43d2
	ra_d4rtm(3,2) = 0;
	
	//H14d2
	ra_d4rtm(0,3) = a7*Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
  a7*(Cos(q6)*(Cos(q5)*(Cos(q4)*
            (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
     (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
   Sin(q7) + d8*(Cos(q7)*(Cos(q6)*
         (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q7)*(-(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6)\
)*Sin(q7))*Sin(q8) + a10*Cos(q10)*
   (Cos(q8)*Cos(q9)*(Cos(q7)*(-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7)) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                    Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9)) + 
  d10*(-(Cos(q9)*(Cos(q7)*(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
              Sin(q6)) + (-(Cos(q5)*
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
     Cos(q8)*(Cos(q7)*(-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7))*Sin(q9));
	
	//H24d2
	ra_d4rtm(1,3) = a7*Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
  a7*(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5)) + 
     (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7) + 
  d8*(Cos(q7)*(Cos(q6)*(Cos(q5)*
            (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
     (Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
   Sin(q8) + a10*Cos(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)) + 
     (Cos(q7)*(Cos(q6)*(Cos(q5)*
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (Cos(q7)*(Cos(q6)*(Cos(q5)*
                 (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                Sin(q2)*Sin(q3)*Sin(q5)) + 
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
          (Cos(q5)*Sin(q2)*Sin(q3) + 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)\
)) + Cos(q8)*(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
      Sin(q9));
	
	//H34d2
	ra_d4rtm(2,3) = a7*Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
  a7*(Cos(q6)*(Cos(q5)*(Cos(q4)*
            (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
     (Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
   Sin(q7) + d8*(Cos(q7)*(Cos(q6)*
         (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) + 
     (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q7)*(-(Cos(q5)*
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7))*Sin(q8) + a10*Cos(q10)*
   (Cos(q8)*Cos(q9)*(Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)\
)*Sin(q7)) + (Cos(q7)*(Cos(q6)*
            (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)\
) + (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9)) + 
  d10*(-(Cos(q9)*(Cos(q7)*(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
             (Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
              Sin(q6)) + (-(Cos(q5)*
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))) + 
     Cos(q8)*(Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H44d2
	ra_d4rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu1 mu1 ----------------
void calc_dd_3d_rototranslational_matrix_mu11(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + ra_q3;
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[0]; //r_shoulder_roll
	double q6 = ra_q6b + mu[1]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[2]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d11
	ra_d11rtm(0,0) = Sin(q10)*(Cos(q8)*(-(Cos(q5)*(Cos(q4)*
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H21d11
	ra_d11rtm(1,0) = Sin(q10)*(Cos(q8)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
             Cos(q2)*Sin(q4))) + Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) - (-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                Cos(q2)*Sin(q4))) + Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H31d11
	ra_d11rtm(2,0) = Sin(q10)*(Cos(q8)*(-(Cos(q5)*(Cos(q4)*
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H41d11
	ra_d11rtm(3,0) = 0;
	
	//H12d11
	ra_d11rtm(0,1) = Cos(q10)*(Cos(q8)*(-(Cos(q5)*(Cos(q4)*
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H22d11
	ra_d11rtm(1,1) = Cos(q10)*(Cos(q8)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
             Cos(q2)*Sin(q4))) + Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) - (-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                Cos(q2)*Sin(q4))) + Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H32d11
	ra_d11rtm(2,1) = Cos(q10)*(Cos(q8)*(-(Cos(q5)*(Cos(q4)*
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H42d11
	ra_d11rtm(3,1) = 0;
	
	//H13d11
	ra_d11rtm(0,2) = -(Cos(q9)*(-(Cos(q7)*(Cos(q5)*
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
            (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
               Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
       Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
               Sin(q1)*Sin(q2)*Sin(q4))) - 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))) \
+ (Cos(q8)*(Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
     (-(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)*
      Sin(q8))*Sin(q9);
	
	//H23d11
	ra_d11rtm(1,2) = -(Cos(q9)*(-(Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
            (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
       Cos(q6)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
               Cos(q2)*Sin(q4))) + Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))) + 
  (Cos(q8)*(Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) \
- (-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)*Sin(q8))*Sin(q9);
	
	//H33d11
	ra_d11rtm(2,2) = -(Cos(q9)*(-(Cos(q7)*(Cos(q5)*
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
            (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
               Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
       Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
               Cos(q1)*Sin(q2)*Sin(q4))) - 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))) \
+ (Cos(q8)*(Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
     (-(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)*
      Sin(q8))*Sin(q9);
	
	//H43d11
	ra_d11rtm(3,2) = 0;
	
	//H14d11
	ra_d11rtm(0,3) = a6*Cos(q6)*(-(Cos(q5)*(Cos(q4)*
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
          Sin(q1)*Sin(q2)*Sin(q4))) - 
     (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
  a7*Cos(q6)*Cos(q7)*(-(Cos(q5)*
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
          Sin(q1)*Sin(q2)*Sin(q4))) - 
     (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
  d6*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a7*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
          (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
     Cos(q6)*(-(Cos(q5)*(Cos(q4)*
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q8)*(-(Cos(q5)*
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(Cos(q8)*
         (Cos(q6)*Cos(q7)*(-(Cos(q5)*
                 (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                      Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q7)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - 
                  Cos(q2)*Sin(q1)*Sin(q3)) - 
               (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
          Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                   (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4))) - 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7)\
)) + (Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8))*Sin(q9));
	
	//H24d11
	ra_d11rtm(1,3) = a6*Cos(q6)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
     Sin(q2)*Sin(q3)*Sin(q5)) + 
  a7*Cos(q6)*Cos(q7)*(-(Cos(q5)*
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
     Sin(q2)*Sin(q3)*Sin(q5)) + 
  d6*(Cos(q5)*Sin(q2)*Sin(q3) + 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
  a7*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
          (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
     Cos(q6)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q8)*(-(Cos(q5)*
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) - (-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)*Sin(q8)) + 
     (-(Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                Cos(q2)*Sin(q4))) + Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
               (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))) + 
          Cos(q6)*(-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                  Cos(q2)*Sin(q4))) + Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))) + 
     (Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
              Sin(q2)*Sin(q3)*Sin(q5)) + 
           (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
            Sin(q7)) - (-(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))) + 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)*Sin(q8))*Sin(q9));
	
	//H34d11
	ra_d11rtm(2,3) = a6*Cos(q6)*(-(Cos(q5)*(Cos(q4)*
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
          Cos(q1)*Sin(q2)*Sin(q4))) - 
     (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
  a7*Cos(q6)*Cos(q7)*(-(Cos(q5)*
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
          Cos(q1)*Sin(q2)*Sin(q4))) - 
     (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
  d6*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
  a7*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(-(Cos(q7)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
          (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
     Cos(q6)*(-(Cos(q5)*(Cos(q4)*
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q8)*(-(Cos(q5)*
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))) - 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6) + 
     (Cos(q6)*Cos(q7)*(-(Cos(q5)*
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(Cos(q8)*
         (Cos(q6)*Cos(q7)*(-(Cos(q5)*
                 (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8)) + (-(Cos(q7)*
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
        Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q7)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + 
                  Cos(q1)*Cos(q2)*Sin(q3)) - 
               (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))) + 
          Cos(q6)*(-(Cos(q5)*(Cos(q4)*
                   (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4))) - 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7)\
)) + (Cos(q8)*(Cos(q6)*Cos(q7)*
            (-(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4))) - 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) - 
        (-(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))) - 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)*
         Sin(q8))*Sin(q9));
	
	//H44d11
	ra_d11rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu2 mu2 ----------------
void calc_dd_3d_rototranslational_matrix_mu22(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + ra_q3;
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[0]; //r_shoulder_roll
	double q6 = ra_q6b + mu[1]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[2]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d33
	ra_d22rtm(0,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) + 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + Cos(q7)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q6)*
              (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) + 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H21d33
	ra_d22rtm(1,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) + 
     Cos(q7)*(-(Cos(q6)*(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(Cos(q7)*Cos(q8)*
         (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)*Sin(q9));
	
	//H31d33
	ra_d22rtm(2,0) = Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q1)*Cos(q4)*Sin(q2) - 
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))) + Cos(q7)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H41d33
	ra_d22rtm(3,0) = 0;
	
	//H12d33
	ra_d22rtm(0,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) + 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + Cos(q7)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q6)*
              (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) + 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H22d33
	ra_d22rtm(1,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) + 
     Cos(q7)*(-(Cos(q6)*(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(Cos(q7)*Cos(q8)*
         (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)*Sin(q9));
	
	//H32d33
	ra_d22rtm(2,1) = Cos(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q1)*Cos(q4)*Sin(q2) - 
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))) + Cos(q7)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H42d33
	ra_d22rtm(3,1) = 0;
	
	//H13d33
	ra_d22rtm(0,2) = -(Cos(q9)*(-(Cos(q6)*(Cos(q5)*
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
               Sin(q1)*Sin(q2)*Sin(q4)) + 
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
       (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
          (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
     Sin(q7)) + (Cos(q7)*Cos(q8)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H23d33
	ra_d22rtm(1,2) = -(Cos(q9)*(-(Cos(q6)*(Cos(q5)*
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
            Sin(q2)*Sin(q3)*Sin(q5))) - 
       (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)) + 
  (Cos(q7)*Cos(q8)*(-(Cos(q6)*
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
     (-(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8))*Sin(q9);
	
	//H33d33
	ra_d22rtm(2,2) = -(Cos(q9)*(-(Cos(q6)*(Cos(q5)*
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
               Cos(q1)*Sin(q2)*Sin(q4)) + 
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
       (Cos(q1)*Cos(q4)*Sin(q2) - 
          (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
     Sin(q7)) + (Cos(q7)*Cos(q8)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) + 
     (-(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q8))*Sin(q9);
	
	//H43d33
	ra_d22rtm(3,2) = 0;
	
	//H14d33
	ra_d22rtm(0,3) = -(a6*Cos(q6)*(Cos(q5)*(Cos(q4)*
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
          Sin(q1)*Sin(q2)*Sin(q4)) + 
       (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
  a6*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
     (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6) + 
  a7*Cos(q7)*(-(Cos(q6)*(Cos(q5)*
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
     (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6)) + 
  d8*(-(Cos(q6)*(Cos(q5)*(Cos(q4)*
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
     (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
   Sin(q7) + a10*Sin(q10)*(-(Cos(q8)*
        (-(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) + 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))) + Cos(q7)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) \
- (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q6)*
              (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) \
+ (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)*Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                   (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                  Sin(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))*Sin(q7)) + 
     (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))) - 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6)) + (-(Cos(q6)*
              (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))) + 
           (Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8))*Sin(q9));
	
	//H24d33
	ra_d22rtm(1,3) = -(a6*Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
       Sin(q2)*Sin(q3)*Sin(q5))) - 
  a6*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6) + 
  a7*Cos(q7)*(-(Cos(q6)*(Cos(q5)*
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))) - 
     (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
  d8*(-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))) - 
     (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7) + 
  a10*Sin(q10)*(-(Cos(q8)*(-(Cos(q6)*
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) + 
     Cos(q7)*(-(Cos(q6)*(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(Cos(q7)*Cos(q8)*
         (-(Cos(q6)*(Cos(q5)*
                 (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))) - 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)*Sin(q9)\
) + d10*(-(Cos(q9)*(-(Cos(q6)*
             (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
               Sin(q2)*Sin(q3)*Sin(q5))) - 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)) + 
     (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                Sin(q2)*Sin(q3)*Sin(q5))) - 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))) + 
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
              Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q8))*Sin(q9));
	
	//H34d33	
	ra_d22rtm(2,3) = -(a6*Cos(q6)*(Cos(q5)*(Cos(q4)*
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
          Cos(q1)*Sin(q2)*Sin(q4)) + 
       (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
  a6*(Cos(q1)*Cos(q4)*Sin(q2) - 
     (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6) + 
  a7*Cos(q7)*(-(Cos(q6)*(Cos(q5)*
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
     (Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) + 
  d8*(-(Cos(q6)*(Cos(q5)*(Cos(q4)*
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
     (Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
   Sin(q7) + a10*Sin(q10)*(-(Cos(q8)*
        (-(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))) + Cos(q7)*
      (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (Cos(q7)*Cos(q8)*(-(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) \
- (Cos(q1)*Cos(q4)*Sin(q2) - (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*
               Sin(q4))*Sin(q6)) + 
        (-(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8)) + 
     (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)*Sin(q9)) + d10*(-(Cos(q9)*
        (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                   (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                  Cos(q1)*Sin(q2)*Sin(q4)) + 
               (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
        Sin(q7)) + (Cos(q7)*Cos(q8)*
         (-(Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))) - 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6)) \
+ (-(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))) + 
           (Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
            Sin(q6))*Sin(q8))*Sin(q9));
	
	//H44d33
	ra_d22rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu3 mu3 ----------------
void calc_dd_3d_rototranslational_matrix_mu33(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + ra_q3;
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[0]; //r_shoulder_roll
	double q6 = ra_q6b + mu[1]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[2]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d22
	ra_d33rtm(0,0) = Sin(q10)*(-(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))) - (-(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  Cos(q10)*(Cos(q8)*Cos(q9)*(-(Cos(q7)*
           (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7))*Sin(q9));
	
	//H21d22
	ra_d33rtm(1,0) = Sin(q10)*(-(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5)) + 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
     (Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q8) + Cos(q10)*(Cos(q8)*Cos(q9)*
      (-(Cos(q7)*(Cos(q6)*(Cos(q5)*
                 (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                Sin(q2)*Sin(q3)*Sin(q5)) + 
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) \
+ (Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
      Sin(q9));
	
	//H31d22
	ra_d33rtm(2,0) = Sin(q10)*(-(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))\
) - (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  Cos(q10)*(Cos(q8)*Cos(q9)*(-(Cos(q7)*
           (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
             (Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H41d22
	ra_d33rtm(3,0) = 0;
	
	//H12d22
	ra_d33rtm(0,1) = Cos(q10)*(-(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))) - (-(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) - 
  Sin(q10)*(Cos(q8)*Cos(q9)*(-(Cos(q7)*
           (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7))*Sin(q9));
	
	//H22d22
	ra_d33rtm(1,1) = Cos(q10)*(-(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5)) + 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
     (Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q8) - Sin(q10)*(Cos(q8)*Cos(q9)*
      (-(Cos(q7)*(Cos(q6)*(Cos(q5)*
                 (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                Sin(q2)*Sin(q3)*Sin(q5)) + 
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) \
+ (Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
      Sin(q9));
	
	//H32d22
	ra_d33rtm(2,1) = Cos(q10)*(-(Cos(q7)*(Cos(q6)*(Cos(q5)*
              (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))\
) - (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) - 
  Sin(q10)*(Cos(q8)*Cos(q9)*(-(Cos(q7)*
           (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
             (Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H42d22
	ra_d33rtm(3,1) = 0;
	
	//H13d22
	ra_d33rtm(0,2) = -(Cos(q9)*(Cos(q7)*(-(Cos(q5)*
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
          (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
       (Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))*Sin(q7))) + 
  Cos(q8)*(-(Cos(q7)*(Cos(q6)*
           (Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))) - (-(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9);
	
	//H23d22
	ra_d33rtm(1,2) = -(Cos(q9)*(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
          (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
       (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5)) + 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))) + 
  Cos(q8)*(-(Cos(q7)*(Cos(q6)*
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5)) + 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
     (Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q9);
	
	//H33d22
	ra_d33rtm(2,2) = -(Cos(q9)*(Cos(q7)*(-(Cos(q5)*
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
          (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
       (Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
        Sin(q7))) + Cos(q8)*(-(Cos(q7)*
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))) \
- (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9);
	
	//H43d22
	ra_d33rtm(3,2) = 0;
	
	//H14d22
	ra_d33rtm(0,3) = -(a7*Cos(q7)*(Cos(q6)*(Cos(q5)*
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
       (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
          (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))) \
- a7*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
        (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)) + a10*Sin(q10)*(-(Cos(q7)*
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
          (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
           Sin(q6))) - (-(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  d10*(-(Cos(q9)*(Cos(q7)*(-(Cos(q5)*
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
          (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))*Sin(q7))) + 
     Cos(q8)*(-(Cos(q7)*(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                   Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9)) + 
  a10*Cos(q10)*(Cos(q8)*Cos(q9)*
      (-(Cos(q7)*(Cos(q6)*(Cos(q5)*
                 (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                      Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
             (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
                (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q7)*(-(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))) + 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                 Sin(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) + 
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
              (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4))*
            Sin(q6))*Sin(q7))*Sin(q9));
	
	//H24d22
	ra_d33rtm(1,3) = -(a7*Cos(q7)*(Cos(q6)*(Cos(q5)*
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5)) + 
       (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
  a7*(Cos(q5)*Sin(q2)*Sin(q3) + 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) + 
        (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7)) + 
  a10*Sin(q10)*(-(Cos(q7)*(Cos(q6)*
           (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5)) + 
          (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
     (Cos(q5)*Sin(q2)*Sin(q3) + 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q8) + d10*(-(Cos(q9)*(Cos(q7)*
           (Cos(q5)*Sin(q2)*Sin(q3) + 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
          (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                   Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))) \
+ Cos(q8)*(-(Cos(q7)*(Cos(q6)*
              (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                Sin(q2)*Sin(q3)*Sin(q5)) + 
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9)) + a10*Cos(q10)*(Cos(q8)*Cos(q9)*
      (-(Cos(q7)*(Cos(q6)*(Cos(q5)*
                 (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
                Sin(q2)*Sin(q3)*Sin(q5)) + 
             (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))) - 
        (Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) \
+ (Cos(q7)*(Cos(q5)*Sin(q2)*Sin(q3) + 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
                 Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) + 
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4))*Sin(q6))*Sin(q7))*
      Sin(q9));
	
	//H34d22	
	ra_d33rtm(2,3) = -(a7*Cos(q7)*(Cos(q6)*(Cos(q5)*
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
       (Cos(q1)*Cos(q4)*Sin(q2) - 
          (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))) - 
  a7*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     (Cos(q6)*(Cos(q5)*(Cos(q4)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
        (Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
      Sin(q7)) + a10*Sin(q10)*(-(Cos(q7)*
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                 (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
          (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))\
) - (-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  d10*(-(Cos(q9)*(Cos(q7)*(-(Cos(q5)*
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
          (Cos(q6)*(Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
             (Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))*Sin(q7))) + 
     Cos(q8)*(-(Cos(q7)*(Cos(q6)*
              (Cos(q5)*(Cos(q4)*
                    (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
             (Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9)) + 
  a10*Cos(q10)*(Cos(q8)*Cos(q9)*
      (-(Cos(q7)*(Cos(q6)*(Cos(q5)*
                 (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                   Cos(q1)*Sin(q2)*Sin(q4)) + 
                (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
             (Cos(q1)*Cos(q4)*Sin(q2) - 
                (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*
              Sin(q6))) - (-(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q7)*(-(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))) + 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
        (Cos(q6)*(Cos(q5)*(Cos(q4)*
                  (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                 Cos(q1)*Sin(q2)*Sin(q4)) + 
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) + 
           (Cos(q1)*Cos(q4)*Sin(q2) - 
              (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4))*Sin(q6))*
         Sin(q7))*Sin(q9));
	
	//H44d22
	ra_d33rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu1 mu2 ----------------
void calc_dd_3d_rototranslational_matrix_mu12(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + ra_q3;
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[0]; //r_shoulder_roll
	double q6 = ra_q6b + mu[1]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[2]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d13
	ra_d12rtm(0,0) = Sin(q10)*(Cos(q6)*Cos(q8)*(Cos(q5)*
         (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(-(Cos(q7)*Cos(q8)*
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8)) - 
     (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7)*Sin(q9));
	
	//H21d13
	ra_d12rtm(1,0) = Sin(q10)*(Cos(q6)*Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q8)) + Cos(q10)*(Cos(q9)*
      (-(Cos(q7)*Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
           Sin(q6)) - Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q8)) \
- (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q7)*Sin(q9));
	
	//H31d13
	ra_d12rtm(2,0) = Sin(q10)*(Cos(q6)*Cos(q8)*(Cos(q5)*
         (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
  Cos(q10)*(Cos(q9)*(-(Cos(q7)*Cos(q8)*
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8)) - 
     (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7)*Sin(q9));
	
	//H41d13
	ra_d12rtm(3,0) = 0;
	
	//H12d13
	ra_d12rtm(0,1) = Cos(q10)*(Cos(q6)*Cos(q8)*(Cos(q5)*
         (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(-(Cos(q7)*Cos(q8)*
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8)) - 
     (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7)*Sin(q9));
	
	//H22d13
	ra_d12rtm(1,1) = Cos(q10)*(Cos(q6)*Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q8)) - Sin(q10)*(Cos(q9)*
      (-(Cos(q7)*Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
           Sin(q6)) - Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q8)) \
- (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q7)*Sin(q9));
	
	//H32d13
	ra_d12rtm(2,1) = Cos(q10)*(Cos(q6)*Cos(q8)*(Cos(q5)*
         (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) - 
  Sin(q10)*(Cos(q9)*(-(Cos(q7)*Cos(q8)*
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8)) - 
     (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7)*Sin(q9));
	
	//H42d13
	ra_d12rtm(3,1) = 0;
	
	//H13d13
	ra_d12rtm(0,2) = Cos(q9)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) + 
  (-(Cos(q7)*Cos(q8)*(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
          (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8))*Sin(q9);
	
	//H23d13
	ra_d12rtm(1,2) = Cos(q9)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) \
+ (-(Cos(q7)*Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
          (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)) \
- Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q8))*
   Sin(q9);
	
	//H33d13
	ra_d12rtm(2,2) = Cos(q9)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) + 
  (-(Cos(q7)*Cos(q8)*(Cos(q5)*
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
          (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8))*Sin(q9);
	
	//H43d13
	ra_d12rtm(3,2) = 0;
	
	//H14d13
	ra_d12rtm(0,3) = -(a6*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
       (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
          Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
  a7*Cos(q7)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) - 
  d8*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) + 
  a10*Sin(q10)*(Cos(q6)*Cos(q8)*
      (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(-(Cos(q7)*Cos(q8)*
           (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8)) - 
     (Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7)*Sin(q9)) + 
  d10*(Cos(q9)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) + 
     (-(Cos(q7)*Cos(q8)*(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8))*Sin(q9));
	
	//H24d13
	ra_d12rtm(1,3) = -(a6*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
       (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
  a7*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6) - 
  d8*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) \
+ a10*Sin(q10)*(Cos(q6)*Cos(q8)*
      (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q8)) + a10*Cos(q10)*(Cos(q9)*
      (-(Cos(q7)*Cos(q8)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
           Sin(q6)) - Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q8)) \
- (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q7)*Sin(q9)) + d10*(Cos(q9)*
      (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q6)*
      Sin(q7) + (-(Cos(q7)*Cos(q8)*
           (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*
           Sin(q6)) - Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q8))*
      Sin(q9));
	
	//H34d13	
	ra_d12rtm(2,3) = -(a6*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
       (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
          Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
  a7*Cos(q7)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6) - 
  d8*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) + 
  a10*Sin(q10)*(Cos(q6)*Cos(q8)*
      (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) - 
     Cos(q7)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q8)) + 
  a10*Cos(q10)*(Cos(q9)*(-(Cos(q7)*Cos(q8)*
           (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8)) - 
     (Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7)*Sin(q9)) + 
  d10*(Cos(q9)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)*Sin(q7) + 
     (-(Cos(q7)*Cos(q8)*(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q6)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q8))*Sin(q9));
	
	//H44d13
	ra_d12rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu1 mu3 ----------------
void calc_dd_3d_rototranslational_matrix_mu13(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + ra_q3;
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[0]; //r_shoulder_roll
	double q6 = ra_q6b + mu[1]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[2]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d12
	ra_d13rtm(0,0) = Sin(q10)*(Cos(q7)*(Cos(q5)*(Cos(q4)*
            (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  Cos(q10)*(Cos(q8)*Cos(q9)*(Cos(q7)*
         (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H21d12
	ra_d13rtm(1,0) = Sin(q10)*(Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
           Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) - 
     Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q8) + Cos(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) - 
        Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) \
+ (Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H31d12
	ra_d13rtm(2,0) = Sin(q10)*(Cos(q7)*(Cos(q5)*(Cos(q4)*
            (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  Cos(q10)*(Cos(q8)*Cos(q9)*(Cos(q7)*
         (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H41d12
	ra_d13rtm(3,0) = 0;
	
	//H12d12
	ra_d13rtm(0,1) = Cos(q10)*(Cos(q7)*(Cos(q5)*(Cos(q4)*
            (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) - 
  Sin(q10)*(Cos(q8)*Cos(q9)*(Cos(q7)*
         (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H22d12
	ra_d13rtm(1,1) = Cos(q10)*(Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
           Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) - 
     Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q8) - Sin(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) - 
        Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) \
+ (Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H32d12
	ra_d13rtm(2,1) = Cos(q10)*(Cos(q7)*(Cos(q5)*(Cos(q4)*
            (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) - 
  Sin(q10)*(Cos(q8)*Cos(q9)*(Cos(q7)*
         (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H42d12
	ra_d13rtm(3,1) = 0;
	
	//H13d12
	ra_d13rtm(0,2) = -(Cos(q9)*(Cos(q6)*Cos(q7)*(Cos(q5)*
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
          (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
       (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))) \
+ Cos(q8)*(Cos(q7)*(Cos(q5)*(Cos(q4)*
            (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9);
	
	//H23d12
	ra_d13rtm(1,2) = -(Cos(q9)*(Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
          (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
       (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))) + 
  Cos(q8)*(Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + 
           Cos(q2)*Sin(q4)) - Sin(q2)*Sin(q3)*Sin(q5)) - 
     Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q9);
	
	//H33d12
	ra_d13rtm(2,2) = -(Cos(q9)*(Cos(q6)*Cos(q7)*(Cos(q5)*
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
          (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
       (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))) \
+ Cos(q8)*(Cos(q7)*(Cos(q5)*(Cos(q4)*
            (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9);
	
	//H43d12
	ra_d13rtm(3,2) = 0;
	
	//H14d12
	ra_d13rtm(0,3) = a7*Cos(q7)*(Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
           Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
  a7*Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
     (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
        Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(Cos(q6)*Cos(q7)*(Cos(q5)*
         (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
     (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q7)*(Cos(q5)*
         (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
        (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  a10*Cos(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*(Cos(q4)*
               (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (Cos(q6)*Cos(q7)*(Cos(q5)*
              (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
             (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
                Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q7)\
)) + Cos(q8)*(Cos(q7)*(Cos(q5)*
            (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3)) - 
           (Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
              Sin(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H24d12
	ra_d13rtm(1,3) = a7*Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
     Sin(q2)*Sin(q3)*Sin(q5)) - 
  a7*Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
     (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
     (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q7)*(Cos(q5)*
         (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5)) - 
     Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
        (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
   Sin(q8) + a10*Cos(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) - 
        Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7)) \
+ (Cos(q6)*Cos(q7)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))*Sin(q9)) + 
  d10*(-(Cos(q9)*(Cos(q6)*Cos(q7)*
           (-(Cos(q5)*Sin(q2)*Sin(q3)) - 
             (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5)) + 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q7))) + 
     Cos(q8)*(Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5)) - 
        Cos(q6)*(-(Cos(q5)*Sin(q2)*Sin(q3)) - 
           (-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4))*Sin(q5))*Sin(q7))*
      Sin(q9));
	
	//H34d12
	ra_d13rtm(2,3) = a7*Cos(q7)*(Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4)) + 
     (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
  a7*Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
     (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
        Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7) + 
  d8*(Cos(q6)*Cos(q7)*(Cos(q5)*
         (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
     (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7)) + 
  a10*Sin(q10)*(Cos(q7)*(Cos(q5)*
         (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
     Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
        (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q8) + 
  a10*Cos(q10)*(Cos(q8)*Cos(q9)*
      (Cos(q7)*(Cos(q5)*(Cos(q4)*
               (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7)) + 
     (Cos(q6)*Cos(q7)*(Cos(q5)*
            (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7))*
      Sin(q9)) + d10*(-(Cos(q9)*
        (Cos(q6)*Cos(q7)*(Cos(q5)*
              (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
             (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5)) + 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q7)\
)) + Cos(q8)*(Cos(q7)*(Cos(q5)*
            (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5)) - 
        Cos(q6)*(Cos(q5)*(-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3)) - 
           (Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4))*Sin(q5))*Sin(q7))*Sin(q9));
	
	//H44d12
	ra_d13rtm(3,3) = 0;
}

//3D right arm rototranslational matrix derivative wrt mu2 mu3 ----------------
void calc_dd_3d_rototranslational_matrix_mu23(vector<double> mu, vector<double> a)
{
	double q1 = ra_q1b + ra_q1;	
	double q2 = ra_q2b + ra_q2;
	double q3 = ra_q3b + ra_q3;
	double q4 = ra_q4b + ra_q4;
	double q5 = ra_q5b + mu[0]; //r_shoulder_roll
	double q6 = ra_q6b + mu[1]; //r_shoulder_yaw
	double q7 = ra_q7b + mu[2]; //r_elbow
	double q8 = ra_q8b + ra_q8;
	double q9 = ra_q9b + ra_q9;
	double q10 = ra_q10b + ra_q10;
	
	//H11d23
	ra_d23rtm(0,0) = -(Sin(q10)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
          (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
     Sin(q7)*Sin(q8)) + Cos(q10)*
   (-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q9));
	
	//H21d23
	ra_d23rtm(1,0) = -(Sin(q10)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
       (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q8)) + 
  Cos(q10)*(-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q9));
	
	//H31d23
	ra_d23rtm(2,0) = -(Sin(q10)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
          (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
     Sin(q7)*Sin(q8)) + Cos(q10)*
   (-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q9));
	
	//H41d23
	ra_d23rtm(3,0) = 0;
	
	//H12d23
	ra_d23rtm(0,1) = -(Cos(q10)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
          (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
     Sin(q7)*Sin(q8)) - Sin(q10)*
   (-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q9));
	
	//H22d23
	ra_d23rtm(1,1) = -(Cos(q10)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
       (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q8)) - 
  Sin(q10)*(-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q9));
	
	//H32d23
	ra_d23rtm(2,1) = -(Cos(q10)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
          (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
     Sin(q7)*Sin(q8)) - Sin(q10)*
   (-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q9));
	
	//H42d23
	ra_d23rtm(3,1) = 0;
	
	//H13d23
	ra_d23rtm(0,2) = -(Cos(q7)*Cos(q9)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
          (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
             Sin(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))) \
- Cos(q8)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7)*Sin(q9);
	
	//H23d23
	ra_d23rtm(1,2) = -(Cos(q7)*Cos(q9)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
       (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
          Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) - 
  Cos(q8)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
     (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q9);
	
	//H33d23
	ra_d23rtm(2,2) = -(Cos(q7)*Cos(q9)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
          (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
       (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
             Cos(q1)*Sin(q2)*Sin(q4)) + 
          (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))) \
- Cos(q8)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7)*Sin(q9);
	
	//H43d23
	ra_d23rtm(3,2) = 0;
	
	//H14d23
	ra_d23rtm(0,3) = d8*Cos(q7)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)) - 
  a7*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7) - a10*Sin(q10)*(Cos(q6)*
      (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
        (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3)) - 
           Sin(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7)*Sin(q8) + a10*Cos(q10)*
   (-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q9)) + d10*(-(Cos(q7)*Cos(q9)*
        (Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
             (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                   Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6)\
)) - Cos(q8)*(Cos(q6)*(-(Cos(q4)*Sin(q1)*Sin(q2)) - 
           (-(Cos(q2)*Cos(q3)*Sin(q1)) + Cos(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(-(Cos(q2)*Cos(q3)*Sin(q1)) + 
                 Cos(q1)*Sin(q3)) - Sin(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q1)*Cos(q3)) - Cos(q2)*Sin(q1)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H24d23
	ra_d23rtm(1,3) = d8*Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
     (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6)) - 
  a7*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
     (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7) - 
  a10*Sin(q10)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
     (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
        Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q8) + 
  a10*Cos(q10)*(-(Cos(q8)*Cos(q9)*
        (Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q9)) + 
  d10*(-(Cos(q7)*Cos(q9)*(Cos(q6)*
           (Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
          (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
             Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))) - 
     Cos(q8)*(Cos(q6)*(Cos(q2)*Cos(q4) + Cos(q3)*Sin(q2)*Sin(q4)) - 
        (Cos(q5)*(-(Cos(q3)*Cos(q4)*Sin(q2)) + Cos(q2)*Sin(q4)) - 
           Sin(q2)*Sin(q3)*Sin(q5))*Sin(q6))*Sin(q7)*Sin(q9));
	
	//H34d23	
	ra_d23rtm(2,3) = d8*Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)) - 
  a7*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7) - a10*Sin(q10)*(Cos(q6)*
      (Cos(q1)*Cos(q4)*Sin(q2) - 
        (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
     (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
           Cos(q1)*Sin(q2)*Sin(q4)) + 
        (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
   Sin(q7)*Sin(q8) + a10*Cos(q10)*
   (-(Cos(q8)*Cos(q9)*(Cos(q6)*
           (Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + 
                   Sin(q1)*Sin(q3)) + Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*
           Sin(q6))*Sin(q7)) + 
     Cos(q7)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q9)) + d10*(-(Cos(q7)*Cos(q9)*
        (Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
             (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
          (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
                Cos(q1)*Sin(q2)*Sin(q4)) + 
             (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6)\
)) - Cos(q8)*(Cos(q6)*(Cos(q1)*Cos(q4)*Sin(q2) - 
           (Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3))*Sin(q4)) - 
        (Cos(q5)*(Cos(q4)*(Cos(q1)*Cos(q2)*Cos(q3) + Sin(q1)*Sin(q3)) + 
              Cos(q1)*Sin(q2)*Sin(q4)) + 
           (-(Cos(q3)*Sin(q1)) + Cos(q1)*Cos(q2)*Sin(q3))*Sin(q5))*Sin(q6))*
      Sin(q7)*Sin(q9));
	
	//H44d23
	ra_d23rtm(3,3) = 0;
}


//3D left arm rototranslational matrix ---------------------------------------
void calc_3d_rototranslational_matrix_b(vector<double> mu, vector<double> a)
{
    double q1b = la_q1b + la_q1;
    double q2b = la_q2b + la_q2;
    double q3b = la_q3b + mu[0]; //torso_yaw
    double q4b = la_q4b + la_q4;
    double q5b = la_q5b + mu[1]; //l_shoulder_roll
    double q6b = la_q6b + mu[2]; //l_shoulder_yaw
    double q7b = la_q7b + mu[3]; //l_elbow
    double q8b = la_q8b + la_q8;
    double q9b = la_q9b + la_q9;
    double q10b = la_q10b + la_q10;

    //H11b
    la_rtm(0,0) = Sin(q10b)*(-(Cos(q8b)*(Cos(q6b)*
             (Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) \
  - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))) + (Cos(q7b)*
           (Cos(q6b)*(Cos(q5b)*
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                 Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                 Sin(q4b))*Sin(q6b)) +
          (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
             (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                   Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
  )*Sin(q7b))*Sin(q8b)) + Cos(q10b)*
     (Cos(q9b)*(Cos(q8b)*(Cos(q7b)*
              (Cos(q6b)*(Cos(q5b)*
                    (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) \
  + (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
                (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                    Sin(q4b))*Sin(q6b)) +
             (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                   Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                 Sin(q5b))*Sin(q7b)) +
          (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
  ) - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
  )*Sin(q6b))*Sin(q8b)) + (-(Cos(q7b)*
             (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                Sin(q5b))) + (Cos(q6b)*
              (Cos(q5b)*(Cos(q4b)*
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                   Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
  ) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
  )*Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H21b
    la_rtm(1,0) = Sin(q10b)*(-(Cos(q8b)*(Cos(q6b)*
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
            (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
               Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))) +
       (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                 (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                Sin(q2b)*Sin(q3b)*Sin(q5b)) +
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
          (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
           Sin(q7b))*Sin(q8b)) +
    Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
           (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                    (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                   Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)\
  ) + (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)\
  )*Sin(q7b)) + (Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
                Cos(q3b)*Sin(q2b)*Sin(q4b)) -
             (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                   Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*
           Sin(q8b)) + (-(Cos(q7b)*
             (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
               (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))) \
  + (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                   Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
           Sin(q7b))*Sin(q9b));

    //H31b
    la_rtm(2,0) = Sin(q10b)*(-(Cos(q8b)*(Cos(q6b)*
             (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))) + (Cos(q7b)*
           (Cos(q6b)*(Cos(q5b)*
                 (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                      Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                 Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
              Sin(q6b)) + (Cos(q5b)*
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
             (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q8b)) +
    Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
           (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                         Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) \
  + (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
                (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                    Sin(q4b))*Sin(q6b)) +
             (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                   Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                      Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                 Sin(q5b))*Sin(q7b)) +
          (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
             (Cos(q5b)*(Cos(q4b)*
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                   Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
  )*Sin(q6b))*Sin(q8b)) + (-(Cos(q7b)*
             (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))) +
          (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                   Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
  ) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
              Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H41b
    la_rtm(3,0) = 0;

    //H12b
    la_rtm(0,1) = Cos(q10b)*(-(Cos(q8b)*(Cos(q6b)*
             (Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) \
  - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))) + (Cos(q7b)*
           (Cos(q6b)*(Cos(q5b)*
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                 Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                 Sin(q4b))*Sin(q6b)) +
          (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
             (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                   Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
  )*Sin(q7b))*Sin(q8b)) - Sin(q10b)*
     (Cos(q9b)*(Cos(q8b)*(Cos(q7b)*
              (Cos(q6b)*(Cos(q5b)*
                    (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) \
  + (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
                (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                    Sin(q4b))*Sin(q6b)) +
             (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                   Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                 Sin(q5b))*Sin(q7b)) +
          (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
  ) - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
  )*Sin(q6b))*Sin(q8b)) + (-(Cos(q7b)*
             (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                Sin(q5b))) + (Cos(q6b)*
              (Cos(q5b)*(Cos(q4b)*
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                   Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
  ) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
  )*Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H22b
    la_rtm(1,1) = Cos(q10b)*(-(Cos(q8b)*(Cos(q6b)*
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
            (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
               Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))) +
       (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                 (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                Sin(q2b)*Sin(q3b)*Sin(q5b)) +
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
          (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
           Sin(q7b))*Sin(q8b)) -
    Sin(q10b)*(Cos(q9b)*(Cos(q8b)*
           (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                    (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                   Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)\
  ) + (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)\
  )*Sin(q7b)) + (Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
                Cos(q3b)*Sin(q2b)*Sin(q4b)) -
             (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                   Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*
           Sin(q8b)) + (-(Cos(q7b)*
             (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
               (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))) \
  + (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                   Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
           Sin(q7b))*Sin(q9b));

    //H32b
    la_rtm(2,1) = Cos(q10b)*(-(Cos(q8b)*(Cos(q6b)*
             (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))) + (Cos(q7b)*
           (Cos(q6b)*(Cos(q5b)*
                 (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                      Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                 Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
              Sin(q6b)) + (Cos(q5b)*
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
             (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q8b)) -
    Sin(q10b)*(Cos(q9b)*(Cos(q8b)*
           (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                         Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) \
  + (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
                (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                    Sin(q4b))*Sin(q6b)) +
             (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                   Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                      Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                 Sin(q5b))*Sin(q7b)) +
          (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
             (Cos(q5b)*(Cos(q4b)*
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                   Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
  )*Sin(q6b))*Sin(q8b)) + (-(Cos(q7b)*
             (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))) +
          (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                   Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
  ) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
              Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H42b
    la_rtm(3,1) = 0;

    //H13b
    la_rtm(0,2) = -(Cos(q9b)*(-(Cos(q7b)*(Cos(q5b)*
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
     )) + (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                     Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) \
     + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b))*Sin(q7b))) +
       (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
                 (Cos(q5b)*(Cos(q4b)*
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                      Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                    Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                    Sin(q4b))*Sin(q6b)) +
             (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
     )*Sin(q7b)) + (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
             (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
              Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H23b
    la_rtm(1,2) = -(Cos(q9b)*(-(Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
              (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))) \
  + (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                  Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
            (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
          Sin(q7b))) + (Cos(q8b)*
        (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                 (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                Sin(q2b)*Sin(q3b)*Sin(q5b)) +
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
          (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
           Sin(q7b)) + (Cos(q6b)*
           (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
          (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
             Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H33b
    la_rtm(2,2) = -(Cos(q9b)*(-(Cos(q7b)*(Cos(q5b)*
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
             (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))) +
        (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                 Cos(q1b)*Sin(q2b)*Sin(q4b)) +
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) \
 + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
            Sin(q6b))*Sin(q7b))) +
   (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
             (Cos(q5b)*(Cos(q4b)*
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
             Sin(q6b)) + (Cos(q5b)*
             (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
            (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b)) +
      (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
            (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
         (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                  Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
            (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
          Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H43b
    la_rtm(3,2) = 0;

    //H14b
    la_rtm(0,3) = d2b*Cos(q1b) - a1b*Sin(q1b) - a3b*Cos(q2b)*Cos(q3b)*Sin(q1b) -
        d3b*Sin(q1b)*Sin(q2b) + a3b*Cos(q1b)*Sin(q3b) +
        d4b*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) +
        a6b*Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
              Sin(q1b)*Sin(q2b)*Sin(q4b)) +
           (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
        d6b*(-(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))) +
           (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
              Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
        a6b*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
           (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*Sin(q6b) \
      + a7b*Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                 Sin(q1b)*Sin(q2b)*Sin(q4b)) +
              (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
           (Cos(q4b)*Sin(q1b)*Sin(q2b) -
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
            Sin(q6b)) + a7b*(Cos(q5b)*
            (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
           (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
              Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b) +
        d8b*(-(Cos(q7b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                   Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                   Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))) +
           (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                     (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                    Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
              (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
               Sin(q6b))*Sin(q7b)) +
        a10b*Sin(q10b)*(-(Cos(q8b)*(Cos(q6b)*
                 (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) \
      - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
                 Sin(q6b))) + (Cos(q7b)*
               (Cos(q6b)*(Cos(q5b)*
                     (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                     Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                     Sin(q4b))*Sin(q6b)) +
              (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
      )*Sin(q7b))*Sin(q8b)) + a10b*Cos(q10b)*
         (Cos(q9b)*(Cos(q8b)*(Cos(q7b)*
                  (Cos(q6b)*(Cos(q5b)*
                        (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                             Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) \
      + (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
                    (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                        Sin(q4b))*Sin(q6b)) +
                 (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                       Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                    (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                     Sin(q5b))*Sin(q7b)) +
              (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                     Sin(q4b)) - (Cos(q5b)*
                     (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                     Sin(q5b))*Sin(q6b))*Sin(q8b)) +
           (-(Cos(q7b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                      Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                    Sin(q5b))) + (Cos(q6b)*
                  (Cos(q5b)*(Cos(q4b)*
                        (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                       Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                     Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                     Sin(q4b))*Sin(q6b))*Sin(q7b))*Sin(q9b)) +
        d10b*(-(Cos(q9b)*(-(Cos(q7b)*
                   (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                        Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                     (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                           Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                      Sin(q5b))) + (Cos(q6b)*
                    (Cos(q5b)*(Cos(q4b)*
                          (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) \
      + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                      (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                       Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                       Sin(q4b))*Sin(q6b))*Sin(q7b))) +
           (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
                     (Cos(q5b)*(Cos(q4b)*
                           (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                             Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) \
      + (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
                    (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                        Sin(q4b))*Sin(q6b)) +
                 (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                       Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                    (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                     Sin(q5b))*Sin(q7b)) +
              (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
      ) - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
      )*Sin(q6b))*Sin(q8b))*Sin(q9b));

    //H24b
    la_rtm(1,3) = d3b*Cos(q2b) - a3b*Cos(q3b)*Sin(q2b) + d4b*Sin(q2b)*Sin(q3b) +
        a6b*Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
              Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
        d6b*(Cos(q5b)*Sin(q2b)*Sin(q3b) +
           (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) +
        a6b*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b) +
        a7b*Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
               (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
              Sin(q2b)*Sin(q3b)*Sin(q5b)) +
           (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
        a7b*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
           (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b) \
      + d8b*(-(Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))) +
           (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                    Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
              (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
            Sin(q7b)) + a10b*Sin(q10b)*
         (-(Cos(q8b)*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
                   Cos(q3b)*Sin(q2b)*Sin(q4b)) -
                (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                   Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))) +
           (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                     (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                    Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                 (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
              (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                 (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
               Sin(q7b))*Sin(q8b)) +
        a10b*Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
               (Cos(q7b)*(Cos(q6b)*
                     (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                          Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                    (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*
                     Sin(q6b)) + (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                    (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*
                     Sin(q5b))*Sin(q7b)) +
              (Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
                 (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b)\
      )*Sin(q8b)) + (-(Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                   (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))\
      ) + (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                 (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
               Sin(q7b))*Sin(q9b)) +
        d10b*(-(Cos(q9b)*(-(Cos(q7b)*
                   (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                     (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)\
      )) + (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                         Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                   (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
                 Sin(q7b))) + (Cos(q8b)*
               (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                        (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                       Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                    (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)\
      ) + (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                    (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)\
      )*Sin(q7b)) + (Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
                    Cos(q3b)*Sin(q2b)*Sin(q4b)) -
                 (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*
               Sin(q8b))*Sin(q9b));

    //H34b
    la_rtm(2,3) = a1b*Cos(q1b) + a3b*Cos(q1b)*Cos(q2b)*Cos(q3b) + d2b*Sin(q1b) +
        d3b*Cos(q1b)*Sin(q2b) + a3b*Sin(q1b)*Sin(q3b) +
        d4b*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) +
        a6b*Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
              Cos(q1b)*Sin(q2b)*Sin(q4b)) +
           (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
        d6b*(-(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))) +
           (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
              Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
        a6b*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*Sin(q6b) +
        a7b*Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                 Cos(q1b)*Sin(q2b)*Sin(q4b)) +
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
           (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*Sin(q6b)) \
      + a7b*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
           (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
              Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b) +
        d8b*(-(Cos(q7b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                   Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                   Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))) +
           (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                     (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                    Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
              (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
               Sin(q6b))*Sin(q7b)) +
        a10b*Sin(q10b)*(-(Cos(q8b)*(Cos(q6b)*
                 (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
                (Cos(q5b)*(Cos(q4b)*
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                      Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
                 Sin(q6b))) + (Cos(q7b)*
               (Cos(q6b)*(Cos(q5b)*
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                     Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                  Sin(q6b)) + (Cos(q5b)*
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                 (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                    Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q8b)) +
        a10b*Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
               (Cos(q7b)*(Cos(q6b)*
                     (Cos(q5b)*(Cos(q4b)*
                           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) \
      - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                        Sin(q5b)) +
                    (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                        Sin(q4b))*Sin(q6b)) +
                 (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                     Sin(q5b))*Sin(q7b)) +
              (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) \
      - (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                     Sin(q5b))*Sin(q6b))*Sin(q8b)) +
           (-(Cos(q7b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                   (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                         Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                    Sin(q5b))) + (Cos(q6b)*
                  (Cos(q5b)*(Cos(q4b)*
                        (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                     Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                  Sin(q6b))*Sin(q7b))*Sin(q9b)) +
        d10b*(-(Cos(q9b)*(-(Cos(q7b)*
                   (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                        Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                           Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                      Sin(q5b))) + (Cos(q6b)*
                    (Cos(q5b)*(Cos(q4b)*
                          (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                         Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                      (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                       Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                    Sin(q6b))*Sin(q7b))) +
           (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
                     (Cos(q5b)*(Cos(q4b)*
                           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                          Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                        Sin(q5b)) +
                    (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                        Sin(q4b))*Sin(q6b)) +
                 (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                     Sin(q5b))*Sin(q7b)) +
              (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
                 (Cos(q5b)*(Cos(q4b)*
                        (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
      )*Sin(q6b))*Sin(q8b))*Sin(q9b));

    la_rtm(3,3) = 1;
}

//3D left arm rototranslational matrix derivative wrt mu1 --------------------
void calc_d_3d_rototranslational_matrix_mu1b(vector<double> mu, vector<double> a)
{
    double q1b = la_q1b + la_q1;
    double q2b = la_q2b + la_q2;
    double q3b = la_q3b + mu[0]; //torso_yaw
    double q4b = la_q4b + la_q4;
    double q5b = la_q5b + mu[1]; //l_shoulder_roll
    double q6b = la_q6b + mu[2]; //l_shoulder_yaw
    double q7b = la_q7b + mu[3]; //l_elbow
    double q8b = la_q8b + la_q8;
    double q9b = la_q9b + la_q9;
    double q10b = la_q10b + la_q10;

    //H11d4b
    la_d1rtm(0,0) = Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
             (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*(Cos(q1b)*Cos(q3b) + 
                Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
             (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b))*
           Sin(q6b))) + (Cos(q7b)*
         (Cos(q6b)*(Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
               Sin(q5b)) - (Cos(q1b)*Cos(q3b) + 
              Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b))*Sin(q8b)) + 
  Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
                  (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)\
)*Sin(q6b))*Sin(q8b)) + (-(Cos(q7b)*
           (Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
             Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
              Sin(q5b))) + (Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)\
) - (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*
         Sin(q7b))*Sin(q9b));

    //H21d4b
    la_d1rtm(1,0) = Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
             Cos(q3b)*Sin(q2b)*Sin(q5b))*Sin(q6b))) + 
     (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
           Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
        (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
           Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b))*Sin(q8b)) + 
  Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
                 Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
              Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
           (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
              Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
           (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b))*Sin(q6b))*Sin(q8b)) + 
     (-(Cos(q7b)*(-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
             Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))) + 
        (Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
           Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H31d4b
    la_d1rtm(2,0) = Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
             (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*(Cos(q3b)*Sin(q1b) - 
                Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
             (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
           Sin(q6b))) + (Cos(q7b)*
         (Cos(q6b)*(Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) \
- (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b))*Sin(q8b)) + 
  Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
                  (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
            Sin(q6b))*Sin(q8b)) + 
     (-(Cos(q7b)*(Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + 
                Sin(q1b)*Sin(q3b)) - 
             Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
              Sin(q5b))) + (Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) - 
           (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*
            Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H41d4b
    la_d1rtm(3,0) = 0;

    //H12d4b
    la_d1rtm(0,1) = Cos(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
             (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*(Cos(q1b)*Cos(q3b) + 
                Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
             (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b))*
           Sin(q6b))) + (Cos(q7b)*
         (Cos(q6b)*(Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
               Sin(q5b)) - (Cos(q1b)*Cos(q3b) + 
              Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b))*Sin(q8b)) - 
  Sin(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
                  (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)\
)*Sin(q6b))*Sin(q8b)) + (-(Cos(q7b)*
           (Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
             Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
              Sin(q5b))) + (Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)\
) - (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*
         Sin(q7b))*Sin(q9b));

    //H22d4b
    la_d1rtm(1,1) = Cos(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
             Cos(q3b)*Sin(q2b)*Sin(q5b))*Sin(q6b))) + 
     (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
           Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
        (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
           Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b))*Sin(q8b)) - 
  Sin(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
                 Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
              Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
           (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
              Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
           (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b))*Sin(q6b))*Sin(q8b)) + 
     (-(Cos(q7b)*(-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
             Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))) + 
        (Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
           Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H32d4b
    la_d1rtm(2,1) = Cos(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
             (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*(Cos(q3b)*Sin(q1b) - 
                Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
             (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
           Sin(q6b))) + (Cos(q7b)*
         (Cos(q6b)*(Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) \
- (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b))*Sin(q8b)) - 
  Sin(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
                  (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
            Sin(q6b))*Sin(q8b)) + 
     (-(Cos(q7b)*(Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + 
                Sin(q1b)*Sin(q3b)) - 
             Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
              Sin(q5b))) + (Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) - 
           (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*
            Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H42d4b
    la_d1rtm(3,1) = 0;

    //H13d4b
    la_d1rtm(0,2) = -(Cos(q9b)*(-(Cos(q7b)*(Cos(q5b)*
             (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
            Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
             Sin(q5b))) + (Cos(q6b)*
           (Cos(q4b)*Cos(q5b)*
              (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
             (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)) \
- (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*
        Sin(q7b))) + (Cos(q8b)*
      (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
               Sin(q5b)) - (Cos(q1b)*Cos(q3b) + 
              Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b)) + 
     (-(Cos(q6b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
           Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
            (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
           (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b))*
         Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H23d4b
    la_d1rtm(1,2) = -(Cos(q9b)*(-(Cos(q7b)*(-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
            Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))) + 
       (Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
             Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
          Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b))*Sin(q7b))) + 
  (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
           Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
        (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
           Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) + 
     (-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
        (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - Cos(q3b)*Sin(q2b)*Sin(q5b))*
         Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H33d4b
    la_d1rtm(2,2) = -(Cos(q9b)*(-(Cos(q7b)*(Cos(q5b)*
             (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
            Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
             Sin(q5b))) + (Cos(q6b)*
           (Cos(q4b)*Cos(q5b)*
              (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
             (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) - 
          (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*
           Sin(q6b))*Sin(q7b))) + 
  (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) \
- (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b)) + 
     (-(Cos(q6b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
           Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
            (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
         Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H43d4b
    la_d1rtm(3,2) = 0;

    //H14d4b
    la_d1rtm(0,3) = a3b*Cos(q1b)*Cos(q3b) + a3b*Cos(q2b)*Sin(q1b)*Sin(q3b) + 
  d4b*(Cos(q2b)*Cos(q3b)*Sin(q1b) - Cos(q1b)*Sin(q3b)) + 
  a6b*Cos(q6b)*(Cos(q4b)*Cos(q5b)*
      (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
     (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)) + 
  d6b*(-(Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))) + 
     Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) - 
  a6b*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b) + 
  a7b*Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
         (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
        (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)) - 
     (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
  a7b*(Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
     Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
   Sin(q7b) + d8b*(-(Cos(q7b)*(Cos(q5b)*
           (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
          Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
           Sin(q5b))) + (Cos(q6b)*
         (Cos(q4b)*Cos(q5b)*(Cos(q1b)*Cos(q3b) + 
              Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
           (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)) - 
        (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*
      Sin(q7b)) + a10b*Sin(q10b)*
   (-(Cos(q8b)*(-(Cos(q6b)*(Cos(q1b)*Cos(q3b) + 
               Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*(Cos(q1b)*Cos(q3b) + 
                Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
             (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b))*
           Sin(q6b))) + (Cos(q7b)*
         (Cos(q6b)*(Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
               Sin(q5b)) - (Cos(q1b)*Cos(q3b) + 
              Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b))*Sin(q8b)) + 
  a10b*Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*
               (Cos(q4b)*Cos(q5b)*
                  (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
               Sin(q5b))*Sin(q6b))*Sin(q8b)) + 
     (-(Cos(q7b)*(Cos(q5b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + 
                Cos(q1b)*Sin(q3b)) - 
             Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
              Sin(q5b))) + (Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
               Sin(q5b)) - (Cos(q1b)*Cos(q3b) + 
              Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*Sin(q7b))*
      Sin(q9b)) + d10b*(-(Cos(q9b)*
        (-(Cos(q7b)*(Cos(q5b)*
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
               Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
                Sin(q5b))) + (Cos(q6b)*
              (Cos(q4b)*Cos(q5b)*
                 (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                 Sin(q5b)) - (Cos(q1b)*Cos(q3b) + 
                Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*Sin(q7b))) + 
     (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
               (Cos(q4b)*Cos(q5b)*
                  (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q1b)*Cos(q3b) + Cos(q2b)*Sin(q1b)*Sin(q3b)) + 
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q5b)\
)*Sin(q6b))*Sin(q8b))*Sin(q9b));

    //H24d1b
    la_d1rtm(1,3) = d4b*Cos(q3b)*Sin(q2b) + a3b*Sin(q2b)*Sin(q3b) + 
  a6b*Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
     Cos(q3b)*Sin(q2b)*Sin(q5b)) + 
  d6b*(Cos(q3b)*Cos(q5b)*Sin(q2b) + Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b)) - 
  a6b*Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b) + 
  a7b*Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
        Cos(q3b)*Sin(q2b)*Sin(q5b)) - Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
  a7b*(-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*
   Sin(q7b) + d8b*(-(Cos(q7b)*(-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
          Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))) + 
     (Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
           Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
        Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b))*Sin(q7b)) + 
  a10b*Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
             Cos(q3b)*Sin(q2b)*Sin(q5b))*Sin(q6b))) + 
     (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
           Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
        (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
           Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b))*Sin(q8b)) + 
  a10b*Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*
               (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
                 Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
              Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
           (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
              Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
           (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b))*Sin(q6b))*Sin(q8b)) + 
     (-(Cos(q7b)*(-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
             Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))) + 
        (Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
           Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b))*Sin(q7b))*Sin(q9b)) + 
  d10b*(-(Cos(q9b)*(-(Cos(q7b)*
             (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
               Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))) + 
          (Cos(q6b)*(Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
                Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
             Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b))*Sin(q7b))) + 
     (Cos(q8b)*(Cos(q7b)*(Cos(q6b)*
               (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
                 Cos(q3b)*Sin(q2b)*Sin(q5b)) - 
              Sin(q2b)*Sin(q3b)*Sin(q4b)*Sin(q6b)) + 
           (-(Cos(q3b)*Cos(q5b)*Sin(q2b)) - 
              Cos(q4b)*Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*Sin(q2b)*Sin(q3b)*Sin(q4b)) - 
           (Cos(q4b)*Cos(q5b)*Sin(q2b)*Sin(q3b) - 
              Cos(q3b)*Sin(q2b)*Sin(q5b))*Sin(q6b))*Sin(q8b))*Sin(q9b));

    //H34d4b
    la_d1rtm(2,3) = a3b*Cos(q3b)*Sin(q1b) - a3b*Cos(q1b)*Cos(q2b)*Sin(q3b) + 
  d4b*(-(Cos(q1b)*Cos(q2b)*Cos(q3b)) - Sin(q1b)*Sin(q3b)) + 
  d6b*(-(Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))) + 
     Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) + 
  a6b*Cos(q6b)*(Cos(q4b)*Cos(q5b)*
      (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
     (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) - 
  a6b*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b) + 
  a7b*Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
         (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
        (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) - 
     (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
  a7b*(Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
     Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
   Sin(q7b) + d8b*(-(Cos(q7b)*(Cos(q5b)*
           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
          Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
           Sin(q5b))) + (Cos(q6b)*
         (Cos(q4b)*Cos(q5b)*(Cos(q3b)*Sin(q1b) - 
              Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) - 
        (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*
      Sin(q7b)) + a10b*Sin(q10b)*
   (-(Cos(q8b)*(-(Cos(q6b)*(Cos(q3b)*Sin(q1b) - 
               Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)) - 
          (Cos(q4b)*Cos(q5b)*(Cos(q3b)*Sin(q1b) - 
                Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
             (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
           Sin(q6b))) + (Cos(q7b)*
         (Cos(q6b)*(Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) \
- (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b)) + 
        (Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
           Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
            Sin(q5b))*Sin(q7b))*Sin(q8b)) + 
  a10b*Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*
               (Cos(q4b)*Cos(q5b)*
                  (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
            Sin(q6b))*Sin(q8b)) + 
     (-(Cos(q7b)*(Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + 
                Sin(q1b)*Sin(q3b)) - 
             Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
              Sin(q5b))) + (Cos(q6b)*
            (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) \
- (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*
         Sin(q7b))*Sin(q9b)) + 
  d10b*(-(Cos(q9b)*(-(Cos(q7b)*
             (Cos(q5b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
               Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
                Sin(q5b))) + (Cos(q6b)*
              (Cos(q4b)*Cos(q5b)*
                 (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b)) \
- (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*Sin(q6b))*
           Sin(q7b))) + (Cos(q8b)*
         (Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Cos(q5b)*
                  (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*
                  Sin(q5b)) - 
              (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q4b)*
               Sin(q6b)) + (Cos(q5b)*
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) - 
              Cos(q4b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
               Sin(q5b))*Sin(q7b)) + 
        (-(Cos(q6b)*(Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b))*
              Sin(q4b)) - (Cos(q4b)*Cos(q5b)*
               (Cos(q3b)*Sin(q1b) - Cos(q1b)*Cos(q2b)*Sin(q3b)) + 
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q5b))*
            Sin(q6b))*Sin(q8b))*Sin(q9b));

    //H44d1b
    la_d1rtm(3,3) = 0;
}

//3D left arm rototranslational matrix derivative wrt mu2 --------------------
void calc_d_3d_rototranslational_matrix_mu2b(vector<double> mu, vector<double> a)
{
    double q1b = la_q1b + la_q1;
    double q2b = la_q2b + la_q2;
    double q3b = la_q3b + mu[0]; //torso_yaw
    double q4b = la_q4b + la_q4;
    double q5b = la_q5b + mu[1]; //l_shoulder_roll
    double q6b = la_q6b + mu[2]; //l_shoulder_yaw
    double q7b = la_q7b + mu[3]; //l_elbow
    double q8b = la_q8b + la_q8;
    double q9b = la_q9b + la_q9;
    double q10b = la_q10b + la_q10;

    //H11d1b
    la_d2rtm(0,0) = Sin(q10b)*(Cos(q8b)*(Cos(q5b)*
             (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
            (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
               Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b) +
         (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
    ) + (-(Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q7b))*Sin(q8b)) +
      Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
             (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                  (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                        Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                   Sin(q5b)) + (-(Cos(q5b)*
                     (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
    )*Sin(q7b)) - (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                  Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*
             Sin(q6b)*Sin(q8b)) + (-(Cos(q7b)*
               (-(Cos(q5b)*(Cos(q4b)*
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                      Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))) \
    + Cos(q6b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*
             Sin(q7b))*Sin(q9b));

    //H21d1b
    la_d2rtm(1,0) = Sin(q10b)*(Cos(q8b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
           Sin(q6b) + (Cos(q6b)*Cos(q7b)*
              (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) +
             (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                     Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b))*
           Sin(q8b)) + Cos(q10b)*(Cos(q9b)*
           (Cos(q8b)*(Cos(q6b)*Cos(q7b)*
                 (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                   (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)\
     ) + (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))) +
                   Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) -
             (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
              Sin(q6b)*Sin(q8b)) + (-(Cos(q7b)*
                (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))) +
             Cos(q6b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
              Sin(q7b))*Sin(q9b));

    //H31d1b
    la_d2rtm(2,0) = Sin(q10b)*(Cos(q8b)*(Cos(q5b)*
             (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
            (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b) +
         (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
            (-(Cos(q5b)*(Cos(q4b)*
                     (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                    Cos(q1b)*Sin(q2b)*Sin(q4b))) -
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q7b))*Sin(q8b)) +
      Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
             (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                   (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                  (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                        Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                   Sin(q5b)) + (-(Cos(q5b)*
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    )*Sin(q7b)) - (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                  Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b)*Sin(q8b)) +
         (-(Cos(q7b)*(-(Cos(q5b)*(Cos(q4b)*
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                      Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                 (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))) \
    + Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q9b));

    //H41d1b
    la_d2rtm(3,0) = 0;

    //H12d1b
    la_d2rtm(0,1) = Cos(q10b)*(Cos(q8b)*(Cos(q5b)*
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                   Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b) +
             (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
        ) + (-(Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                           Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
                 Sin(q7b))*Sin(q8b)) -
          Sin(q10b)*(Cos(q9b)*(Cos(q8b)*
                 (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                       (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                      (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                            Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                       Sin(q5b)) + (-(Cos(q5b)*
                         (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                              Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                      (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
        )*Sin(q7b)) - (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                      Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*
                 Sin(q6b)*Sin(q8b)) + (-(Cos(q7b)*
                   (-(Cos(q5b)*(Cos(q4b)*
                           (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                          Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                     (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))) \
        + Cos(q6b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*
                 Sin(q7b))*Sin(q9b));

    //H22d1b
    la_d2rtm(1,1) = Cos(q10b)*(Cos(q8b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
           Sin(q6b) + (Cos(q6b)*Cos(q7b)*
              (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) +
             (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                     Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b))*
           Sin(q8b)) - Sin(q10b)*(Cos(q9b)*
           (Cos(q8b)*(Cos(q6b)*Cos(q7b)*
                 (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                   (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)\
     ) + (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))) +
                   Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) -
             (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
              Sin(q6b)*Sin(q8b)) + (-(Cos(q7b)*
                (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))) +
             Cos(q6b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
              Sin(q7b))*Sin(q9b));

    //H32d1b
    la_d2rtm(2,1) = Cos(q10b)*(Cos(q8b)*(Cos(q5b)*
             (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
            (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b) +
         (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
            (-(Cos(q5b)*(Cos(q4b)*
                     (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                    Cos(q1b)*Sin(q2b)*Sin(q4b))) -
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q7b))*Sin(q8b)) -
      Sin(q10b)*(Cos(q9b)*(Cos(q8b)*
             (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                   (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                  (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                        Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                   Sin(q5b)) + (-(Cos(q5b)*
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    )*Sin(q7b)) - (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                  Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b)*Sin(q8b)) +
         (-(Cos(q7b)*(-(Cos(q5b)*(Cos(q4b)*
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                      Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                 (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))) \
    + Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q9b));

    //H42d1b
    la_d2rtm(3,1) = 0;

    //H13d1b
    la_d2rtm(0,2) = -(Cos(q9b)*(-(Cos(q7b)*(-(Cos(q5b)*
                  (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))) \
   + Cos(q6b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
             (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))) +
     (Cos(q8b)*(Cos(q6b)*Cos(q7b)*
            (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
              (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                    Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
   ) + (-(Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                      Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
              (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
            Sin(q7b)) - (Cos(q5b)*
            (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
           (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
              Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b)*Sin(q8b))*Sin(q9b);

    //H23d1b
    la_d2rtm(1,2) = -(Cos(q9b)*(-(Cos(q7b)*(-(Cos(q5b)*
                  (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))) +
               Sin(q2b)*Sin(q3b)*Sin(q5b))) +
          Cos(q6b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
           Sin(q7b))) + (Cos(q8b)*
         (Cos(q6b)*Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
              (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) +
           (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                   Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) \
   - (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
           (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
         Sin(q6b)*Sin(q8b))*Sin(q9b);

    //H33d1b
    la_d2rtm(2,2) = -(Cos(q9b)*(-(Cos(q7b)*(-(Cos(q5b)*
                  (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                       Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))) -
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))) \
   + Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
             (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))) +
     (Cos(q8b)*(Cos(q6b)*Cos(q7b)*
            (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
              (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                 Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
           (-(Cos(q5b)*(Cos(q4b)*
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                   Cos(q1b)*Sin(q2b)*Sin(q4b))) -
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
            Sin(q7b)) - (Cos(q5b)*
            (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
           (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
              Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b)*Sin(q8b))*Sin(q9b);

    //H43d1b
    la_d2rtm(3,2) = 0;

    //H14d1b
    la_d2rtm(0,3) = d6b*(Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                 Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
           (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
        a6b*Cos(q6b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
              Cos(q2b)*Sin(q1b)*Sin(q3b)) -
           (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
              Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
        a7b*Cos(q6b)*Cos(q7b)*(Cos(q5b)*
            (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
           (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
              Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
        a7b*(-(Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                   Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
           (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*Sin(q7b) \
      + d8b*(-(Cos(q7b)*(-(Cos(q5b)*
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                        Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))) +
           Cos(q6b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                 Cos(q2b)*Sin(q1b)*Sin(q3b)) -
              (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                 Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b)) +
        a10b*Sin(q10b)*(Cos(q8b)*(Cos(q5b)*
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
              (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                 Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b) +
           (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
      ) + (-(Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
               Sin(q7b))*Sin(q8b)) +
        a10b*Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
               (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                     (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                    (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                     Sin(q5b)) + (-(Cos(q5b)*
                       (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                            Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) \
      - (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*Sin(q7b)) \
      - (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
      )*Sin(q6b)*Sin(q8b)) + (-(Cos(q7b)*
                 (-(Cos(q5b)*(Cos(q4b)*
                         (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                        Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))\
      ) + Cos(q6b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                    Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
      )*Sin(q7b))*Sin(q9b)) + d10b*(-(Cos(q9b)*
              (-(Cos(q7b)*(-(Cos(q5b)*
                        (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                             Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                     (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
      )) + Cos(q6b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                      Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                    Sin(q5b))*Sin(q7b))) +
           (Cos(q8b)*(Cos(q6b)*Cos(q7b)*
                  (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                       Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                    (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                     Sin(q5b)) + (-(Cos(q5b)*
                       (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                            Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))) -
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
      )*Sin(q7b)) - (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                    Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*
               Sin(q6b)*Sin(q8b))*Sin(q9b));

    //H24d1b
    la_d2rtm(1,3) = d6b*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
             Sin(q2b)*Sin(q3b)*Sin(q5b)) +
          a6b*Cos(q6b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) +
          a7b*Cos(q6b)*Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
             (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) +
          a7b*(-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))) +
             Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b) +
          d8b*(-(Cos(q7b)*(-(Cos(q5b)*
                     (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))) +
                  Sin(q2b)*Sin(q3b)*Sin(q5b))) +
             Cos(q6b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
              Sin(q7b)) + a10b*Sin(q10b)*
           (Cos(q8b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
              Sin(q6b) + (Cos(q6b)*Cos(q7b)*
                 (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                   (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) +
                (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                        Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b))*
              Sin(q8b)) + a10b*Cos(q10b)*
           (Cos(q9b)*(Cos(q8b)*(Cos(q6b)*Cos(q7b)*
                    (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                      (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*
                       Sin(q5b)) + (-(Cos(q5b)*
                         (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))) +
                      Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) -
                (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                   (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
                 Sin(q6b)*Sin(q8b)) + (-(Cos(q7b)*
                   (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                          Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))) +
                Cos(q6b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                   (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
                 Sin(q7b))*Sin(q9b)) +
          d10b*(-(Cos(q9b)*(-(Cos(q7b)*
                     (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                            Cos(q2b)*Sin(q4b))) + Sin(q2b)*Sin(q3b)*Sin(q5b))) +
                  Cos(q6b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                     (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
                   Sin(q7b))) + (Cos(q8b)*
                 (Cos(q6b)*Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                      (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)\
        ) + (-(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))) +
                      Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q7b)) -
                (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                   (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
                 Sin(q6b)*Sin(q8b))*Sin(q9b));

    //H34d1b
    la_d2rtm(2,3) = d6b*(Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b)) +
            (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
         a6b*Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
               Cos(q1b)*Cos(q2b)*Sin(q3b)) -
            (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
         a7b*Cos(q6b)*Cos(q7b)*(Cos(q5b)*
             (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
            (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
         a7b*(-(Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                    Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))) -
            (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*Sin(q7b) \
       + d8b*(-(Cos(q7b)*(-(Cos(q5b)*
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                      Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                 (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))) +
            Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                  Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b)) +
         a10b*Sin(q10b)*(Cos(q8b)*(Cos(q5b)*
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b) +
            (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                   (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                  (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) +
               (-(Cos(q5b)*(Cos(q4b)*
                        (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
                Sin(q7b))*Sin(q8b)) +
         a10b*Cos(q10b)*(Cos(q9b)*(Cos(q8b)*
                (Cos(q6b)*Cos(q7b)*(Cos(q5b)*
                      (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                           Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                      Sin(q5b)) + (-(Cos(q5b)*
                        (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                             Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))) \
       - (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*Sin(q7b)) \
       - (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                  (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b)*Sin(q8b)) +
            (-(Cos(q7b)*(-(Cos(q5b)*(Cos(q4b)*
                          (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                         Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))\
       ) + Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                  (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q9b)) +
         d10b*(-(Cos(q9b)*(-(Cos(q7b)*
                    (-(Cos(q5b)*(Cos(q4b)*
                            (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                           Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                      (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
       )) + Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))) +
            (Cos(q8b)*(Cos(q6b)*Cos(q7b)*
                   (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                        Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                           Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))*
                      Sin(q5b)) + (-(Cos(q5b)*
                        (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                             Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b))) -
                     (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
       )*Sin(q7b)) - (Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                  (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q6b)*Sin(q8b))*
             Sin(q9b));

    //H44d1b
    la_d2rtm(3,3) = 0;
}

//3D left arm rototranslational matrix derivative wrt mu3 --------------------
void calc_d_3d_rototranslational_matrix_mu3b(vector<double> mu, vector<double> a)
{
    double q1b = la_q1b + la_q1;
    double q2b = la_q2b + la_q2;
    double q3b = la_q3b + mu[0]; //torso_yaw
    double q4b = la_q4b + la_q4;
    double q5b = la_q5b + mu[1]; //l_shoulder_roll
    double q6b = la_q6b + mu[2]; //l_shoulder_yaw
    double q7b = la_q7b + mu[3]; //l_elbow
    double q8b = la_q8b + la_q8;
    double q9b = la_q9b + la_q9;
    double q10b = la_q10b + la_q10;

    //H11d3b
    la_d3rtm(0,0) = Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
                 (Cos(q5b)*(Cos(q4b)*
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                      Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                    Sin(q5b))) - (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
               Sin(q6b))) + Cos(q7b)*
          (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q8b)) +
      Cos(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
             (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    ) - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                        Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
    )*Sin(q6b)) + (-(Cos(q6b)*(Cos(q5b)*
                     (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                     Sin(q5b))) - (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    )*Sin(q6b))*Sin(q8b)) + (Cos(q6b)*
             (Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q7b)*Sin(q9b));

    //H21d3b
    la_d3rtm(1,0) = Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
                 (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                      Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
              (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))) +
         Cos(q7b)*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
               Cos(q3b)*Sin(q2b)*Sin(q4b)) -
            (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
               Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q8b)) +
      Cos(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
             (Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
               (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                     Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b)) \
    + (-(Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
               (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
             Sin(q8b)) + (Cos(q6b)*
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
            (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
               Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q7b)*Sin(q9b));

    //H31d3b
    la_d3rtm(2,0) = Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
                 (Cos(q5b)*(Cos(q4b)*
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                      Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                    Sin(q5b))) - (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
               Sin(q6b))) + Cos(q7b)*
          (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                     Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q8b)) +
      Cos(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
             (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
               (Cos(q5b)*(Cos(q4b)*
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    )*Sin(q6b)) + (-(Cos(q6b)*(Cos(q5b)*
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                     Sin(q5b))) - (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b))*Sin(q8b)) +
         (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                     Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q7b)*Sin(q9b));

    //H41d3b
    la_d3rtm(3,0) = 0;

    //H12d3b
    la_d3rtm(0,1) = Cos(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
                 (Cos(q5b)*(Cos(q4b)*
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                      Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                    Sin(q5b))) - (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
               Sin(q6b))) + Cos(q7b)*
          (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q8b)) -
      Sin(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
             (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    ) - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                        Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
    )*Sin(q6b)) + (-(Cos(q6b)*(Cos(q5b)*
                     (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                     Sin(q5b))) - (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    )*Sin(q6b))*Sin(q8b)) + (Cos(q6b)*
             (Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q7b)*Sin(q9b));

    //H22d3b
    la_d3rtm(1,1) = Cos(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
                 (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                      Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
              (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))) +
         Cos(q7b)*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
               Cos(q3b)*Sin(q2b)*Sin(q4b)) -
            (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
               Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q8b)) -
      Sin(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
             (Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
               (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                     Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b)) \
    + (-(Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
               (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
             Sin(q8b)) + (Cos(q6b)*
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
            (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
               Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q7b)*Sin(q9b));

    //H32d3b
    la_d3rtm(2,1) = Cos(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
                 (Cos(q5b)*(Cos(q4b)*
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                      Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                    Sin(q5b))) - (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
               Sin(q6b))) + Cos(q7b)*
          (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                     Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q8b)) -
      Sin(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
             (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
               (Cos(q5b)*(Cos(q4b)*
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    )*Sin(q6b)) + (-(Cos(q6b)*(Cos(q5b)*
                     (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                     Sin(q5b))) - (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b))*Sin(q8b)) +
         (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                     Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b))*Sin(q7b)*Sin(q9b));

    //H42d3b
    la_d3rtm(3,1) = 0;

    //H13d3b
    la_d3rtm(0,2) = -(Cos(q9b)*(Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
           (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                    Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
              (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
            Sin(q6b))*Sin(q7b)) + (Cos(q7b)*Cos(q8b)*
          (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b)) + (-(Cos(q6b)*
               (Cos(q5b)*(Cos(q4b)*
                     (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                    Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))\
    ) - (Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
             Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H23d3b
    la_d3rtm(1,2) = -(Cos(q9b)*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
            (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
               Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q7b)) +
       (Cos(q7b)*Cos(q8b)*(Cos(q6b)*
              (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
             (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b)) +
          (-(Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                     Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
             (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
           Sin(q8b))*Sin(q9b);

    //H33d3b
    la_d3rtm(2,2) = -(Cos(q9b)*(Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
           (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                    Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
            Sin(q6b))*Sin(q7b)) + (Cos(q7b)*Cos(q8b)*
          (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
            (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                     Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
             Sin(q6b)) + (-(Cos(q6b)*
               (Cos(q5b)*(Cos(q4b)*
                     (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                    Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))\
    ) - (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
             Sin(q6b))*Sin(q8b))*Sin(q9b);

    //H43d3b
    la_d3rtm(3,2) = 0;

    //H14d3b
    la_d3rtm(0,3) = a6b*Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
              (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
           a6b*(Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                    Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
              (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*Sin(q6b) \
         + a7b*Cos(q7b)*(Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
              (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
               Sin(q6b)) + d8b*(Cos(q6b)*
               (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                 (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
              (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                       Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
               Sin(q6b))*Sin(q7b) + a10b*Sin(q10b)*
            (-(Cos(q8b)*(-(Cos(q6b)*(Cos(q5b)*
                         (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                              Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                        (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                         Sin(q5b))) - (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
                    Sin(q6b))) + Cos(q7b)*
               (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
                 (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
                  Sin(q6b))*Sin(q8b)) +
           a10b*Cos(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
                  (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                        Sin(q4b)) - (Cos(q5b)*
                        (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                             Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                        Sin(q5b))*Sin(q6b)) +
                 (-(Cos(q6b)*(Cos(q5b)*
                          (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                               Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                         (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                          Sin(q5b))) -
                    (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                        Sin(q4b))*Sin(q6b))*Sin(q8b)) +
              (Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) -
                 (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
                  Sin(q6b))*Sin(q7b)*Sin(q9b)) +
           d10b*(-(Cos(q9b)*(Cos(q6b)*(Cos(q4b)*Sin(q1b)*Sin(q2b) -
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)) \
         - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                            Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                      (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b))*
                    Sin(q6b))*Sin(q7b)) +
              (Cos(q7b)*Cos(q8b)*(Cos(q6b)*
                     (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
         ) - (Cos(q5b)*(Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                             Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
         )*Sin(q6b)) + (-(Cos(q6b)*(Cos(q5b)*
                          (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                               Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                         (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                          Sin(q5b))) - (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
         )*Sin(q6b))*Sin(q8b))*Sin(q9b));

    //H24d3b
    la_d3rtm(1,3) = a6b*Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
        a6b*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
           Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b) +
        a7b*Cos(q7b)*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
              Cos(q3b)*Sin(q2b)*Sin(q4b)) -
           (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
              Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b)) +
        d8b*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
           (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
              Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q7b) +
        a10b*Sin(q10b)*(-(Cos(q8b)*(-(Cos(q6b)*
                   (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                        Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
                (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))) +
           Cos(q7b)*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
                 Cos(q3b)*Sin(q2b)*Sin(q4b)) -
              (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                 Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q8b)) +
        a10b*Cos(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
               (Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
                 (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b)\
      ) + (-(Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                         Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
                 (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
               Sin(q8b)) + (Cos(q6b)*
               (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
              (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                 Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q7b)*Sin(q9b)) +
        d10b*(-(Cos(q9b)*(Cos(q6b)*(-(Cos(q2b)*Cos(q4b)) +
                   Cos(q3b)*Sin(q2b)*Sin(q4b)) -
                (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                   Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b))*Sin(q7b)) +
           (Cos(q7b)*Cos(q8b)*(Cos(q6b)*
                  (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b)) -
                 (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))*Sin(q6b)) \
      + (-(Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                         Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b))) -
                 (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
               Sin(q8b))*Sin(q9b));

    //H34d3b
    la_d3rtm(2,3) = a6b*Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
           a6b*(Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                    Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
              (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*Sin(q6b) \
         + a7b*Cos(q7b)*(Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
              (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                       Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
               Sin(q6b)) + d8b*(Cos(q6b)*
               (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                 (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
              (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                       Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                 (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
               Sin(q6b))*Sin(q7b) + a10b*Sin(q10b)*
            (-(Cos(q8b)*(-(Cos(q6b)*(Cos(q5b)*
                         (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                              Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                        (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                         Sin(q5b))) - (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                    Sin(q6b))) + Cos(q7b)*
               (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
                 (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
                  Sin(q6b))*Sin(q8b)) +
           a10b*Cos(q10b)*(Cos(q9b)*(Cos(q7b)*Cos(q8b)*
                  (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) \
         - (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                             Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                        Sin(q5b))*Sin(q6b)) +
                 (-(Cos(q6b)*(Cos(q5b)*
                          (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                               Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                         (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                          Sin(q5b))) -
                    (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                     Sin(q6b))*Sin(q8b)) +
              (Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
                 (Cos(q5b)*(Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                          Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
                  Sin(q6b))*Sin(q7b)*Sin(q9b)) +
           d10b*(-(Cos(q9b)*(Cos(q6b)*(-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
                   (Cos(q5b)*(Cos(q4b)*
                          (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                         Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                      (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b))*
                    Sin(q6b))*Sin(q7b)) +
              (Cos(q7b)*Cos(q8b)*(Cos(q6b)*
                     (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b)) -
                    (Cos(q5b)*(Cos(q4b)*
                           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                          Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
         )*Sin(q6b)) + (-(Cos(q6b)*(Cos(q5b)*
                          (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                               Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                         (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                          Sin(q5b))) - (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                     Sin(q6b))*Sin(q8b))*Sin(q9b));

    //H44d3b
    la_d3rtm(3,3) = 0;
}

//3D left arm rototranslational matrix derivative wrt mu4 --------------------
void calc_d_3d_rototranslational_matrix_mu4b(vector<double> mu, vector<double> a)
{
    double q1b = la_q1b + la_q1;
    double q2b = la_q2b + la_q2;
    double q3b = la_q3b + mu[0]; //torso_yaw
    double q4b = la_q4b + la_q4;
    double q5b = la_q5b + mu[1]; //l_shoulder_roll
    double q6b = la_q6b + mu[2]; //l_shoulder_yaw
    double q7b = la_q7b + mu[3]; //l_elbow
    double q8b = la_q8b + la_q8;
    double q9b = la_q9b + la_q9;
    double q10b = la_q10b + la_q10;

    //H11d2b
    la_d4rtm(0,0) = Sin(q10b)*(Cos(q7b)*(Cos(q5b)*
             (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
            (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
               Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
         (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                  Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) \
    + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
             Sin(q6b))*Sin(q7b))*Sin(q8b) +
      Cos(q10b)*(Cos(q8b)*Cos(q9b)*
          (Cos(q7b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                  Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) \
    - (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                     Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
    ) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    )*Sin(q6b))*Sin(q7b)) + (Cos(q7b)*
             (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                     Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
    ) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    )*Sin(q6b)) + (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                  Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*
             Sin(q7b))*Sin(q9b));

    //H21d2
    la_d4rtm(1,0) = Sin(q10b)*(Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
         (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
      (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
               Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
         (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
       Sin(q7b))*Sin(q8b) + Cos(q10b)*
    (Cos(q8b)*Cos(q9b)*(Cos(q7b)*
          (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
            (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
         (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                  Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
            (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
          Sin(q7b)) + (Cos(q7b)*
          (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                  Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
            (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
         (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
            (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
          Sin(q7b))*Sin(q9b));

    //H31d2
    la_d4rtm(2,0) = Sin(q10b)*(Cos(q7b)*(Cos(q5b)*
             (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
            (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
         (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) \
    + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
             Sin(q6b))*Sin(q7b))*Sin(q8b) +
      Cos(q10b)*(Cos(q8b)*Cos(q9b)*
          (Cos(q7b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                  Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
            (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    ) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b))*Sin(q7b)) +
         (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                   (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                        Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    ) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b)) + (Cos(q5b)*
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q9b));

    //H41d2
    la_d4rtm(3,0) = 0;

    //H12d2
    la_d4rtm(0,1) = Cos(q10b)*(Cos(q7b)*(Cos(q5b)*
             (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
            (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
               Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
         (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                  Sin(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) \
    + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
               (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
             Sin(q6b))*Sin(q7b))*Sin(q8b) -
      Sin(q10b)*(Cos(q8b)*Cos(q9b)*
          (Cos(q7b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                  Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) \
    - (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                     Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
    ) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    )*Sin(q6b))*Sin(q7b)) + (Cos(q7b)*
             (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                     Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
    ) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
    )*Sin(q6b)) + (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                  Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                     Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*
             Sin(q7b))*Sin(q9b));

    //H22d2
    la_d4rtm(1,1) = Cos(q10b)*(Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
         (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
      (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
               Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
         (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
       Sin(q7b))*Sin(q8b) - Sin(q10b)*
    (Cos(q8b)*Cos(q9b)*(Cos(q7b)*
          (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
            (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
         (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                  Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
            (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
          Sin(q7b)) + (Cos(q7b)*
          (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                  Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
            (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
         (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
            (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
          Sin(q7b))*Sin(q9b));

    //H32d2
    la_d4rtm(2,1) = Cos(q10b)*(Cos(q7b)*(Cos(q5b)*
             (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
            (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
               Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
         (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                   (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b)) +
               (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) \
    + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
               (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
             Sin(q6b))*Sin(q7b))*Sin(q8b) -
      Sin(q10b)*(Cos(q8b)*Cos(q9b)*
          (Cos(q7b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                  Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
            (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    ) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b))*Sin(q7b)) +
         (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                   (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                        Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
    ) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b)) + (Cos(q5b)*
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q9b));

    //H42d2
    la_d4rtm(3,1) = 0;

    //H13d2
    la_d4rtm(0,2) = -(Cos(q9b)*(Cos(q7b)*(Cos(q6b)*
                  (Cos(q5b)*(Cos(q4b)*
                        (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                       Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) \
       + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
                  Sin(q6b)) + (Cos(q5b)*
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                 (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                    Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))) +
         Cos(q8b)*(Cos(q7b)*(Cos(q5b)*
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
               (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                  Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
            (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                     Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
               (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                  (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b))*Sin(q7b))*Sin(q9b);

    //H23d2
    la_d4rtm(1,2) = -(Cos(q9b)*(Cos(q7b)*(Cos(q6b)*
          (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
               Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
         (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
      (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
         (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
       Sin(q7b))) + Cos(q8b)*(Cos(q7b)*
     (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
       (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
    (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
             Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
       (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
     Sin(q7b))*Sin(q9b);

    //H33d2
    la_d4rtm(2,2) = -(Cos(q9b)*(Cos(q7b)*(Cos(q6b)*
                  (Cos(q5b)*(Cos(q4b)*
                        (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) \
       + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                  Sin(q6b)) + (Cos(q5b)*
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                 (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                    Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))) +
         Cos(q8b)*(Cos(q7b)*(Cos(q5b)*
                (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
               (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                  Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
            (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                      (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                     Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                  (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
               (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                  (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                Sin(q6b))*Sin(q7b))*Sin(q9b);

    //H43d2
    la_d4rtm(3,2) = 0;

    //H14d2
    la_d4rtm(0,3) = a7b*Cos(q7b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                Cos(q2b)*Sin(q1b)*Sin(q3b)) -
             (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
          a7b*(Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                    (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                   Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
             (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
              Sin(q6b))*Sin(q7b) + d8b*
           (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                    (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) +
                (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
                 Sin(q6b)) + (Cos(q5b)*
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                   Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b)) +
          a10b*Sin(q10b)*(Cos(q7b)*(Cos(q5b)*
                 (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                   Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
             (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                       (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                      Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                   (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)) \
        + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                   (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b))*
                 Sin(q6b))*Sin(q7b))*Sin(q8b) +
          a10b*Cos(q10b)*(Cos(q8b)*Cos(q9b)*
              (Cos(q7b)*(Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) -
                      Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
        ) - (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                          (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                         Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                      (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                       Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                       Sin(q4b))*Sin(q6b))*Sin(q7b)) +
             (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                       (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                            Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                      (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                       Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                       Sin(q4b))*Sin(q6b)) +
                (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)\
        )*Sin(q7b))*Sin(q9b)) + d10b*(-(Cos(q9b)*
                (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                         (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                              Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                        (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*
                         Sin(q5b)) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                        (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*
                         Sin(q4b))*Sin(q6b)) +
                  (Cos(q5b)*(-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                     (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                           Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*
                      Sin(q5b))*Sin(q7b))) +
             Cos(q8b)*(Cos(q7b)*(Cos(q5b)*
                    (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b)) -
                   (Cos(q4b)*(-(Cos(q2b)*Cos(q3b)*Sin(q1b)) +
                         Cos(q1b)*Sin(q3b)) + Sin(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) \
        - (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                          (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b)) +
                         Sin(q1b)*Sin(q2b)*Sin(q4b)) +
                      (-(Cos(q1b)*Cos(q3b)) - Cos(q2b)*Sin(q1b)*Sin(q3b))*Sin(q5b)\
        ) + (Cos(q4b)*Sin(q1b)*Sin(q2b) -
                      (-(Cos(q2b)*Cos(q3b)*Sin(q1b)) + Cos(q1b)*Sin(q3b))*Sin(q4b)\
        )*Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H24d2
    la_d4rtm(1,3) = a7b*Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
              (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
           a7b*(Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                    Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
              (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*Sin(q7b) \
         + d8b*(Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                     (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                    Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                 (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
              (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                 (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
               Sin(q7b)) + a10b*Sin(q10b)*
            (Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                 (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
              (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                       Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                 (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
               Sin(q7b))*Sin(q8b) + a10b*Cos(q10b)*
            (Cos(q8b)*Cos(q9b)*(Cos(q7b)*
                  (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                    (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
                 (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                          Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                    (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
                  Sin(q7b)) + (Cos(q7b)*
                  (Cos(q6b)*(Cos(q5b)*
                        (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b)) -
                       Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                    (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) +
                 (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                    (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
                  Sin(q7b))*Sin(q9b)) +
           d10b*(-(Cos(q9b)*(Cos(q7b)*(Cos(q6b)*
                       (Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                            Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                      (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b)) \
         + (-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                      (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b))*
                    Sin(q7b))) + Cos(q8b)*
               (Cos(q7b)*(-(Cos(q5b)*Sin(q2b)*Sin(q3b)) -
                    (-(Cos(q3b)*Cos(q4b)*Sin(q2b)) - Cos(q2b)*Sin(q4b))*Sin(q5b)) -
                 (Cos(q6b)*(Cos(q5b)*(-(Cos(q3b)*Cos(q4b)*Sin(q2b)) -
                          Cos(q2b)*Sin(q4b)) - Sin(q2b)*Sin(q3b)*Sin(q5b)) +
                    (-(Cos(q2b)*Cos(q4b)) + Cos(q3b)*Sin(q2b)*Sin(q4b))*Sin(q6b))*
                  Sin(q7b))*Sin(q9b));

    //H34d2
    la_d4rtm(2,3) = a7b*Cos(q7b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                    Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                 (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                    Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
              a7b*(Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                        (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                    (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
                 (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                    (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*Sin(q6b)\
            )*Sin(q7b) + d8b*(Cos(q7b)*(Cos(q6b)*
                     (Cos(q5b)*(Cos(q4b)*
                           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                          Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) +
                    (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                     Sin(q6b)) + (Cos(q5b)*
                     (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b)) +
              a10b*Sin(q10b)*(Cos(q7b)*(Cos(q5b)*
                     (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                    (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                       Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
                 (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                           (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                          Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                       (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)) \
            + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                       (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                     Sin(q6b))*Sin(q7b))*Sin(q8b) +
              a10b*Cos(q10b)*(Cos(q8b)*Cos(q9b)*
                  (Cos(q7b)*(Cos(q5b)*(-(Cos(q3b)*Sin(q1b)) +
                          Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                       (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                          Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
                    (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                             Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                          (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                           Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                          (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                        Sin(q6b))*Sin(q7b)) +
                 (Cos(q7b)*(Cos(q6b)*(Cos(q5b)*
                           (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) +
                                Sin(q1b)*Sin(q3b)) - Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                          (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                           Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                          (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                        Sin(q6b)) + (Cos(q5b)*
                        (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                       (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                          Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))*Sin(q9b)) +
              d10b*(-(Cos(q9b)*(Cos(q7b)*(Cos(q6b)*
                          (Cos(q5b)*(Cos(q4b)*
                                (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                               Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                            (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*
                             Sin(q5b)) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                            (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                          Sin(q6b)) + (Cos(q5b)*
                          (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                         (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                            Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b))*Sin(q7b))) +
                 Cos(q8b)*(Cos(q7b)*(Cos(q5b)*
                        (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b)) -
                       (Cos(q4b)*(Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                          Cos(q1b)*Sin(q2b)*Sin(q4b))*Sin(q5b)) -
                    (Cos(q6b)*(Cos(q5b)*(Cos(q4b)*
                              (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b)) -
                             Cos(q1b)*Sin(q2b)*Sin(q4b)) +
                          (-(Cos(q3b)*Sin(q1b)) + Cos(q1b)*Cos(q2b)*Sin(q3b))*Sin(q5b)\
            ) + (-(Cos(q1b)*Cos(q4b)*Sin(q2b)) -
                          (Cos(q1b)*Cos(q2b)*Cos(q3b) + Sin(q1b)*Sin(q3b))*Sin(q4b))*
                        Sin(q6b))*Sin(q7b))*Sin(q9b));

    //H44d2
    la_d4rtm(3,3) = 0;
}


//3D position -----------------------------------------------------------------
//Right arm
double calc_3d_position_x(vector<double> mu, vector<double> a)
{
	double x_pos;
	
	//calculate position vector
	calc_3d_rototranslational_matrix(mu, a);
	ra_epv = ra_rtm * ra_repv;
	
	x_pos = ra_epv(0);
	x_pos /= k; //(mm to m)
	
	return x_pos;
}

double calc_3d_position_y(vector<double> mu, vector<double> a)
{
	double y_pos;

    /*//calculate position vector
    calc_3d_rototranslational_matrix(mu, a);
    ra_epv = ra_rtm * ra_repv;*/

	y_pos = ra_epv(1);
	y_pos /= k; //(mm to m)
	
	return y_pos;
}

double calc_3d_position_z(vector<double> mu, vector<double> a)
{
	double z_pos;
	
    /*//calculate position vector
    calc_3d_rototranslational_matrix(mu, a);
    ra_epv = ra_rtm * ra_repv;*/

	z_pos = ra_epv(2);
	z_pos /= k; //(mm to m)
	
	return z_pos;
}

//Left arm
double calc_3d_position_x_b(vector<double> mu, vector<double> a)
{
    double x_pos;

    //calculate position vector
    calc_3d_rototranslational_matrix_b(mu, a);
    la_epv = la_rtm * la_repv;

    x_pos = la_epv(0);
    x_pos /= k; //(mm to m)

    return x_pos;
}

double calc_3d_position_y_b(vector<double> mu, vector<double> a)
{
    double y_pos;

    //calculate position vector
    calc_3d_rototranslational_matrix_b(mu, a);
    la_epv = la_rtm * la_repv;

    y_pos = la_epv(1);
    y_pos /= k; //(mm to m)

    return y_pos;
}

double calc_3d_position_z_b(vector<double> mu, vector<double> a)
{
    double z_pos;

    //calculate position vector
    calc_3d_rototranslational_matrix_b(mu, a);
    la_epv = la_rtm * la_repv;

    z_pos = la_epv(2);
    z_pos /= k; //(mm to m)

    return z_pos;
}

//3D position derivative ------------------------------------------------------
//Right arm
double d_3d_position_x_mu1(vector<double> mu, vector<double> a)
{
	double d_x_1 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu1(mu, a);
	ra_d1epv = ra_d1rtm * ra_repv;
	
	d_x_1 = ra_d1epv(0);
	d_x_1 /= k; //(mm to m)
		
	return d_x_1;
}

double d_3d_position_x_mu2(vector<double> mu, vector<double> a)
{
	double d_x_2 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu2(mu, a);
	ra_d2epv = ra_d2rtm * ra_repv;
	
	d_x_2 = ra_d2epv(0);
	d_x_2 /= k; //(mm to m)
		
	return d_x_2;
}

double d_3d_position_x_mu3(vector<double> mu, vector<double> a)
{
	double d_x_3 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu3(mu, a);
	ra_d3epv = ra_d3rtm * ra_repv;
	
	d_x_3 = ra_d3epv(0);
	d_x_3 /= k; //(mm to m)
		
	return d_x_3;
}

double d_3d_position_x_mu4(vector<double> mu, vector<double> a)
{
	double d_x_4 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu4(mu, a);
	ra_d4epv = ra_d4rtm * ra_repv;
	
	d_x_4 = ra_d4epv(0);
	d_x_4 /= k; //(mm to m)
		
	return d_x_4;
}

double d_3d_position_y_mu1(vector<double> mu, vector<double> a)
{
	double d_y_1 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu1(mu, a);
	//ra_d1epv = ra_d1rtm * ra_repv;
	
	d_y_1 = ra_d1epv(1);
	d_y_1 /= k; //(mm to m)
	
	return d_y_1;
}

double d_3d_position_y_mu2(vector<double> mu, vector<double> a)
{
	double d_y_2 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu2(mu, a);
	//ra_d2epv = ra_d2rtm * ra_repv;
	
	d_y_2 = ra_d2epv(1);
	d_y_2 /= k; //(mm to m)
	
	return d_y_2;
}

double d_3d_position_y_mu3(vector<double> mu, vector<double> a)
{
	double d_y_3 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu3(mu, a);
	//ra_d3epv = ra_d3rtm * ra_repv;
	
	d_y_3 = ra_d3epv(1);
	d_y_3 /= k; //(mm to m)
	
	return d_y_3;
}

double d_3d_position_y_mu4(vector<double> mu, vector<double> a)
{
	double d_y_4 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu4(mu, a);
	//ra_d4epv = ra_d4rtm * ra_repv;
	
	d_y_4 = ra_d4epv(1);
	d_y_4 /= k; //(mm to m)
	
	return d_y_4;
}

double d_3d_position_z_mu1(vector<double> mu, vector<double> a)
{
	double d_z_1 = 0;
	
	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu1(mu, a);
	//ra_d1epv = ra_d1rtm * ra_repv;

	d_z_1 = ra_d1epv(2);
	d_z_1 /= k; //(mm to m)
	
	return d_z_1;
}

double d_3d_position_z_mu2(vector<double> mu, vector<double> a)
{
	double d_z_2 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu2(mu, a);
	//ra_d2epv = ra_d2rtm * ra_repv;
	
	d_z_2 = ra_d2epv(2);
	d_z_2 /= k; //(mm to m)
	
	return d_z_2;
}

double d_3d_position_z_mu3(vector<double> mu, vector<double> a)
{
	double d_z_3 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu3(mu, a);
	//ra_d3epv = ra_d3rtm * ra_repv;
	
	d_z_3 = ra_d3epv(2);
	d_z_3 /= k; //(mm to m)
	
	return d_z_3;
}

double d_3d_position_z_mu4(vector<double> mu, vector<double> a)
{
	double d_z_4 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu4(mu, a);
	//ra_d4epv = ra_d4rtm * ra_repv;
	
	d_z_4 = ra_d4epv(2);
	d_z_4 /= k; //(mm to m)
	
	return d_z_4;
}

//Left arm
double d_3d_position_x_mu1_b(vector<double> mu, vector<double> a)
{
	double d_x_1 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu1b(mu, a);
	la_d1epv = la_d1rtm * la_repv;
	
	d_x_1 = la_d1epv(0);
	d_x_1 /= k; //(mm to m)
		
	return d_x_1;
}

double d_3d_position_x_mu2_b(vector<double> mu, vector<double> a)
{
	double d_x_2 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu2b(mu, a);
	la_d2epv = la_d2rtm * la_repv;
	
	d_x_2 = la_d2epv(0);
	d_x_2 /= k; //(mm to m)
		
	return d_x_2;
}

double d_3d_position_x_mu3_b(vector<double> mu, vector<double> a)
{
	double d_x_3 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu3b(mu, a);
	la_d3epv = la_d3rtm * la_repv;
	
	d_x_3 = la_d3epv(0);
	d_x_3 /= k; //(mm to m)
		
	return d_x_3;
}

double d_3d_position_x_mu4_b(vector<double> mu, vector<double> a)
{
	double d_x_4 = 0;
	
	//calculate position derivative vector
	calc_d_3d_rototranslational_matrix_mu4b(mu, a);
	la_d4epv = la_d4rtm * la_repv;
	
	d_x_4 = la_d4epv(0);
	d_x_4 /= k; //(mm to m)
		
	return d_x_4;
}

double d_3d_position_y_mu1_b(vector<double> mu, vector<double> a)
{
	double d_y_1 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu1b(mu, a);
	//ra_d1epv = ra_d1rtm * ra_repv;
	
	d_y_1 = la_d1epv(1);
	d_y_1 /= k; //(mm to m)
	
	return d_y_1;
}

double d_3d_position_y_mu2_b(vector<double> mu, vector<double> a)
{
	double d_y_2 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu2b(mu, a);
	//ra_d2epv = ra_d2rtm * ra_repv;
	
	d_y_2 = la_d2epv(1);
	d_y_2 /= k; //(mm to m)
	
	return d_y_2;
}

double d_3d_position_y_mu3_b(vector<double> mu, vector<double> a)
{
	double d_y_3 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu3b(mu, a);
	//ra_d3epv = ra_d3rtm * ra_repv;
	
	d_y_3 = la_d3epv(1);
	d_y_3 /= k; //(mm to m)
	
	return d_y_3;
}

double d_3d_position_y_mu4_b(vector<double> mu, vector<double> a)
{
	double d_y_4 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu4b(mu, a);
	//ra_d4epv = ra_d4rtm * ra_repv;
	
	d_y_4 = la_d4epv(1);
	d_y_4 /= k; //(mm to m)
	
	return d_y_4;
}

double d_3d_position_z_mu1_b(vector<double> mu, vector<double> a)
{
	double d_z_1 = 0;
	
	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu1b(mu, a);
	//ra_d1epv = ra_d1rtm * ra_repv;

	d_z_1 = la_d1epv(2);
	d_z_1 /= k; //(mm to m)
	
	return d_z_1;
}

double d_3d_position_z_mu2_b(vector<double> mu, vector<double> a)
{
	double d_z_2 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu2b(mu, a);
	//ra_d2epv = ra_d2rtm * ra_repv;
	
	d_z_2 = la_d2epv(2);
	d_z_2 /= k; //(mm to m)
	
	return d_z_2;
}

double d_3d_position_z_mu3_b(vector<double> mu, vector<double> a)
{
	double d_z_3 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu3b(mu, a);
	//ra_d3epv = ra_d3rtm * ra_repv;
	
	d_z_3 = la_d3epv(2);
	d_z_3 /= k; //(mm to m)
	
	return d_z_3;
}

double d_3d_position_z_mu4_b(vector<double> mu, vector<double> a)
{
	double d_z_4 = 0;

	//calculate position derivative vector
	//calc_d_3d_rototranslational_matrix_mu4b(mu, a);
	//ra_d4epv = ra_d4rtm * ra_repv;
	
	d_z_4 = la_d4epv(2);
	d_z_4 /= k; //(mm to m)
	
	return d_z_4;
}


//3D head rototranslational matrix -------------------------------------------------
void calc_le_rototranslational_matrix(vector<double> mu, vector<double> a)
{
	double q1e = le_q1b + le_q1;
	double q2e = le_q2b + le_q2;
	double q3e = le_q3b + le_q3;
	double q4e = le_q4b + mu[0]; //neck_pitch
	double q5e = le_q5b + le_q5;
	double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
	double q8e = le_q8b + le_q8;

	//L11
	le_rtm(0,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + 
                 Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) + 
        (Cos(q4e)*Sin(q1e)*Sin(q2e) - 
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) + 
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) + 
  (-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) - 
          (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) + 
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + 
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);
	
	//L21
	le_rtm(1,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) + 
           Sin(q2e)*Sin(q3e)*Sin(q5e)) + 
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) + 
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) + 
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e)) + (-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) + 
          Cos(q3e)*Sin(q2e)*Sin(q4e))) + 
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) + 
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))*Sin(q8e);
	
	//L31
	le_rtm(2,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
              Cos(q1e)*Sin(q2e)*Sin(q4e)) + 
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) + 
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) - 
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) + 
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) + 
  (-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) - 
          (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) + 
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + 
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) + 
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);
	
	//L41
	le_rtm(3,0) = 0;
	
	//L12
	le_rtm(0,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) + 
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) - 
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
           Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) + 
     (Cos(q4e)*Sin(q1e)*Sin(q2e) - 
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
      Sin(q6e))*Sin(q7e);
	
	//L22
	le_rtm(1,1) = Cos(q7e)*(-(Cos(q5e)*Sin(q2e)*Sin(q3e)) + 
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) - 
  (Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - 
           Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) + 
     (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e))*Sin(q7e);
	
	//L32
	le_rtm(2,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) + 
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) - 
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
           Cos(q1e)*Sin(q2e)*Sin(q4e)) + 
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) + 
     (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) - 
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e))*
   Sin(q7e);
	
	//L42
	le_rtm(3,1) = 0;
	
	//L13
	le_rtm(0,2) = -(Cos(q8e)*(-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) - 
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) \
+ (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + 
                Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
          (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
              Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) + 
        (Cos(q4e)*Sin(q1e)*Sin(q2e) - 
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) + 
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);
	
	//L23
	le_rtm(1,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) + 
            Cos(q3e)*Sin(q2e)*Sin(q4e))) + 
       (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) + 
          Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))) + 
  (Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) + 
           Sin(q2e)*Sin(q3e)*Sin(q5e)) + 
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) + 
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) + 
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e))*Sin(q8e);
	
	//L33
	le_rtm(2,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) - 
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) + 
       (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + 
                Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) + 
          (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
              Cos(q1e)*Sin(q2e)*Sin(q4e)) + 
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) + 
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) - 
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) + 
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);
	
	//L43
	le_rtm(3,2) = 0;
	
	//L14
	le_rtm(0,3) = d2e*Cos(q1e) - a1e*Sin(q1e) - a3e*Cos(q2e)*Cos(q3e)*Sin(q1e) - 
  d3e*Sin(q1e)*Sin(q2e) + a3e*Cos(q1e)*Sin(q3e) + 
  a4e*Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
  a4e*Sin(q1e)*Sin(q2e)*Sin(q4e) + 
  d5e*(-(Cos(q4e)*Sin(q1e)*Sin(q2e)) + 
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) + 
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
        Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
     (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) + 
  d6e*(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e)) - 
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) + 
  a6e*(Cos(q4e)*Sin(q1e)*Sin(q2e) - 
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) \
+ d7e*(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) - 
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) - 
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + 
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e));
	
	//L24
	le_rtm(1,3) = d3e*Cos(q2e) - a3e*Cos(q3e)*Sin(q2e) - a4e*Cos(q3e)*Cos(q4e)*Sin(q2e) - 
  a4e*Cos(q2e)*Sin(q4e) + d5e*(Cos(q2e)*Cos(q4e) - 
     Cos(q3e)*Sin(q2e)*Sin(q4e)) + 
  a6e*Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - 
        Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) + 
  d6e*(Cos(q5e)*Sin(q2e)*Sin(q3e) - 
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) + 
  a6e*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e) + 
  d7e*(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e)) - 
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) + 
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e));
	
	//L34
	le_rtm(2,3) = a1e*Cos(q1e) + a3e*Cos(q1e)*Cos(q2e)*Cos(q3e) + d2e*Sin(q1e) + 
  d3e*Cos(q1e)*Sin(q2e) + a3e*Sin(q1e)*Sin(q3e) + 
  a4e*Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
  a4e*Cos(q1e)*Sin(q2e)*Sin(q4e) + 
  d5e*(Cos(q1e)*Cos(q4e)*Sin(q2e) + 
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) + 
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
        Cos(q1e)*Sin(q2e)*Sin(q4e)) + 
     (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) + 
  d6e*(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e)) - 
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) + 
  a6e*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) - 
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) + 
  d7e*(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) - 
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) - 
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + 
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) + 
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e));
	
	//L44
	le_rtm(3,3) = 1;

}

void calc_re_rototranslational_matrix(vector<double> mu, vector<double> a)
{
    double q1e = le_q1b + le_q1;
    double q2e = le_q2b + le_q2;
    double q3e = le_q3b + le_q3;
    double q4e = le_q4b + mu[0]; //neck_pitch
    double q5e = le_q5b + le_q5;
    double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
    double q8e = le_q8b + le_q8;

    //L11
    re_rtm(0,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
                 Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
        (Cos(q4e)*Sin(q1e)*Sin(q2e) -
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) +
  (-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
          (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) +
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);

    //L21
    re_rtm(1,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
           Sin(q2e)*Sin(q3e)*Sin(q5e)) +
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) +
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e)) + (-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) +
          Cos(q3e)*Sin(q2e)*Sin(q4e))) +
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))*Sin(q8e);

    //L31
    re_rtm(2,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
              Cos(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) +
  (-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
          (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) +
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);

    //L41
    re_rtm(3,0) = 0;

    //L12
    re_rtm(0,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
     (Cos(q4e)*Sin(q1e)*Sin(q2e) -
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
      Sin(q6e))*Sin(q7e);

    //L22
    re_rtm(1,1) = Cos(q7e)*(-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) -
           Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) +
     (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e))*Sin(q7e);

    //L32
    re_rtm(2,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
     (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e))*
   Sin(q7e);

    //L42
    re_rtm(3,1) = 0;

    //L13
    re_rtm(0,2) = -(Cos(q8e)*(-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) \
+ (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
                Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
          (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
              Sin(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
        (Cos(q4e)*Sin(q1e)*Sin(q2e) -
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);

    //L23
    re_rtm(1,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) +
            Cos(q3e)*Sin(q2e)*Sin(q4e))) +
       (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
          Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))) +
  (Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
           Sin(q2e)*Sin(q3e)*Sin(q5e)) +
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) +
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e))*Sin(q8e);

    //L33
    re_rtm(2,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) +
       (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
                Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
          (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
              Cos(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);

    //L43
    re_rtm(3,2) = 0;

    //L14
    re_rtm(0,3) = d2e*Cos(q1e) - a1e*Sin(q1e) - a3e*Cos(q2e)*Cos(q3e)*Sin(q1e) -
  d3e*Sin(q1e)*Sin(q2e) + a3e*Cos(q1e)*Sin(q3e) +
  a4e*Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
  a4e*Sin(q1e)*Sin(q2e)*Sin(q4e) +
  d5e*(-(Cos(q4e)*Sin(q1e)*Sin(q2e)) +
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e)) +
     (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
  d6e*(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e)) -
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) \
+ d7ex*(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) -
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e));

    //L24
    re_rtm(1,3) = d3e*Cos(q2e) - a3e*Cos(q3e)*Sin(q2e) - a4e*Cos(q3e)*Cos(q4e)*Sin(q2e) -
  a4e*Cos(q2e)*Sin(q4e) + d5e*(Cos(q2e)*Cos(q4e) -
     Cos(q3e)*Sin(q2e)*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) -
        Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) +
  d6e*(Cos(q5e)*Sin(q2e)*Sin(q3e) -
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e) +
  d7ex*(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e)) -
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e));

    //L34
    re_rtm(2,3) = a1e*Cos(q1e) + a3e*Cos(q1e)*Cos(q2e)*Cos(q3e) + d2e*Sin(q1e) +
  d3e*Cos(q1e)*Sin(q2e) + a3e*Sin(q1e)*Sin(q3e) +
  a4e*Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
  a4e*Cos(q1e)*Sin(q2e)*Sin(q4e) +
  d5e*(Cos(q1e)*Cos(q4e)*Sin(q2e) +
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e)) +
     (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
  d6e*(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e)) -
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) +
  d7ex*(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) -
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e));

    //L44
    re_rtm(3,3) = 1;

}

void calc_le_rototranslational_matrix_h(vector<double> mu, vector<double> a)
{
    double q1e = le_q1b + le_q1;
    double q2e = le_q2b + le_q2;
    double q3e = le_q3b + le_q3;
    double q4e = le_q4b + mu[0]; //neck_pitch
    double q5e = le_q5b + le_q5;
    double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
    double q8e = le_q8b + le_q8;

    //L11
    le_rtm_h(0,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
                 Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
        (Cos(q4e)*Sin(q1e)*Sin(q2e) -
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) +
  (-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
          (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) +
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);

    //L21
    le_rtm_h(1,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
           Sin(q2e)*Sin(q3e)*Sin(q5e)) +
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) +
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e)) + (-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) +
          Cos(q3e)*Sin(q2e)*Sin(q4e))) +
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))*Sin(q8e);

    //L31
    le_rtm_h(2,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
              Cos(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) +
  (-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
          (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) +
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);

    //L41
    le_rtm_h(3,0) = 0;

    //L12
    le_rtm_h(0,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
     (Cos(q4e)*Sin(q1e)*Sin(q2e) -
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
      Sin(q6e))*Sin(q7e);

    //L22
    le_rtm_h(1,1) = Cos(q7e)*(-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) -
           Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) +
     (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e))*Sin(q7e);

    //L32
    le_rtm_h(2,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
     (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e))*
   Sin(q7e);

    //L42
    le_rtm_h(3,1) = 0;

    //L13
    le_rtm_h(0,2) = -(Cos(q8e)*(-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) \
+ (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
                Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
          (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
              Sin(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
        (Cos(q4e)*Sin(q1e)*Sin(q2e) -
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);

    //L23
    le_rtm_h(1,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) +
            Cos(q3e)*Sin(q2e)*Sin(q4e))) +
       (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
          Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))) +
  (Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
           Sin(q2e)*Sin(q3e)*Sin(q5e)) +
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) +
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e))*Sin(q8e);

    //L33
    le_rtm_h(2,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) +
       (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
                Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
          (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
              Cos(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);

    //L43
    le_rtm_h(3,2) = 0;

    //L14
    le_rtm_h(0,3) = d2e*Cos(q1e) - a1e*Sin(q1e) - a3e*Cos(q2e)*Cos(q3e)*Sin(q1e) -
  d3e*Sin(q1e)*Sin(q2e) + a3e*Cos(q1e)*Sin(q3e) +
  a4e*Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
  a4e*Sin(q1e)*Sin(q2e)*Sin(q4e) +
  d5e*(-(Cos(q4e)*Sin(q1e)*Sin(q2e)) +
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e)) +
     (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
  d6e*(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e)) -
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) \
+ d7e*(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) -
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e));

    //L24
    le_rtm_h(1,3) = d3e*Cos(q2e) - a3e*Cos(q3e)*Sin(q2e) - a4e*Cos(q3e)*Cos(q4e)*Sin(q2e) -
  a4e*Cos(q2e)*Sin(q4e) + d5e*(Cos(q2e)*Cos(q4e) -
     Cos(q3e)*Sin(q2e)*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) -
        Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) +
  d6e*(Cos(q5e)*Sin(q2e)*Sin(q3e) -
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e) +
  d7e*(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e)) -
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e));

    //L34
    le_rtm_h(2,3) = a1e*Cos(q1e) + a3e*Cos(q1e)*Cos(q2e)*Cos(q3e) + d2e*Sin(q1e) +
  d3e*Cos(q1e)*Sin(q2e) + a3e*Sin(q1e)*Sin(q3e) +
  a4e*Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
  a4e*Cos(q1e)*Sin(q2e)*Sin(q4e) +
  d5e*(Cos(q1e)*Cos(q4e)*Sin(q2e) +
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e)) +
     (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
  d6e*(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e)) -
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) +
  d7e*(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) -
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e));

    //L44
    le_rtm_h(3,3) = 1;

}

void calc_le_rototranslational_matrix_b(vector<double> mu, vector<double> a)
{
    double q1e = le_q1b + le_q1;
    double q2e = le_q2b + le_q2;
    double q3e = le_q3b + le_q3;
    double q4e = le_q4b + mu[0]; //neck_pitch
    double q5e = le_q5b + le_q5;
    double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
    double q8e = le_q8b + le_q8;

    //L11
    le_rtm_b(0,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
                 Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
        (Cos(q4e)*Sin(q1e)*Sin(q2e) -
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) +
  (-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
          (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) +
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);

    //L21
    le_rtm_b(1,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
           Sin(q2e)*Sin(q3e)*Sin(q5e)) +
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) +
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e)) + (-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) +
          Cos(q3e)*Sin(q2e)*Sin(q4e))) +
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))*Sin(q8e);

    //L31
    le_rtm_b(2,0) = Cos(q8e)*(Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
              Cos(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)) +
  (-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
          (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) +
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e))*
   Sin(q8e);

    //L41
    le_rtm_b(3,0) = 0;

    //L12
    le_rtm_b(0,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
     (Cos(q4e)*Sin(q1e)*Sin(q2e) -
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
      Sin(q6e))*Sin(q7e);

    //L22
    le_rtm_b(1,1) = Cos(q7e)*(-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) -
           Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) +
     (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e))*Sin(q7e);

    //L32
    le_rtm_b(2,1) = Cos(q7e)*(-(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) -
  (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
     (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e))*
   Sin(q7e);

    //L42
    le_rtm_b(3,1) = 0;

    //L13
    le_rtm_b(0,2) = -(Cos(q8e)*(-(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
            (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))) \
+ (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
                Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
          (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
              Sin(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
        (Cos(q4e)*Sin(q1e)*Sin(q2e) -
           (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))) +
        (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
           Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);

    //L23
    le_rtm_b(1,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) +
            Cos(q3e)*Sin(q2e)*Sin(q4e))) +
       (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
          Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e))) +
  (Cos(q7e)*(Cos(q6e)*(Cos(q5e)*
            (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
           Sin(q2e)*Sin(q3e)*Sin(q5e)) +
        (-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e)) +
     (-(Cos(q5e)*Sin(q2e)*Sin(q3e)) +
        (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e))*
      Sin(q7e))*Sin(q8e);

    //L33
    le_rtm_b(2,2) = -(Cos(q8e)*(-(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
            (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))) +
       (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
                Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
          (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*
        Sin(q6e))) + (Cos(q7e)*
      (Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
               (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
              Cos(q1e)*Sin(q2e)*Sin(q4e)) +
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
        (-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
           (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*
         Sin(q6e)) + (-(Cos(q5e)*
           (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))) +
        (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
           Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e))*Sin(q8e);

    //L43
    le_rtm_b(3,2) = 0;

    //L14
    le_rtm_b(0,3) = d2e*Cos(q1e) - a1e*Sin(q1e) - a3e*Cos(q2e)*Cos(q3e)*Sin(q1e) -
  d3e*Sin(q1e)*Sin(q2e) + a3e*Cos(q1e)*Sin(q3e) +
  a4e*Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
  a4e*Sin(q1e)*Sin(q2e)*Sin(q4e) +
  d5e*(-(Cos(q4e)*Sin(q1e)*Sin(q2e)) +
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e)) +
     (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)) +
  d6e*(Cos(q5e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e)) -
     (Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) +
        Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) \
+ d7e*(Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) -
        (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)) -
     (Cos(q5e)*(Cos(q4e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) +
              Cos(q1e)*Sin(q3e)) + Sin(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e))*Sin(q6e));

    //L24
    le_rtm_b(1,3) = d3e*Cos(q2e) - a3e*Cos(q3e)*Sin(q2e) - a4e*Cos(q3e)*Cos(q4e)*Sin(q2e) -
  a4e*Cos(q2e)*Sin(q4e) + d5e*(Cos(q2e)*Cos(q4e) -
     Cos(q3e)*Sin(q2e)*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) -
        Cos(q2e)*Sin(q4e)) + Sin(q2e)*Sin(q3e)*Sin(q5e)) +
  d6e*(Cos(q5e)*Sin(q2e)*Sin(q3e) -
     (-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e) +
  d7e*(Cos(q6e)*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e)) -
     (Cos(q5e)*(-(Cos(q3e)*Cos(q4e)*Sin(q2e)) - Cos(q2e)*Sin(q4e)) +
        Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e));

    //L34
    le_rtm_b(2,3) = a1e*Cos(q1e) + a3e*Cos(q1e)*Cos(q2e)*Cos(q3e) + d2e*Sin(q1e) +
  d3e*Cos(q1e)*Sin(q2e) + a3e*Sin(q1e)*Sin(q3e) +
  a4e*Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
  a4e*Cos(q1e)*Sin(q2e)*Sin(q4e) +
  d5e*(Cos(q1e)*Cos(q4e)*Sin(q2e) +
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) +
  a6e*Cos(q6e)*(Cos(q5e)*(Cos(q4e)*
         (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e)) +
     (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)) +
  d6e*(Cos(q5e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e)) -
     (Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) -
        Cos(q1e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) +
  a6e*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) +
  d7e*(Cos(q6e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)) -
        (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)) -
     (Cos(q5e)*(Cos(q4e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) +
              Sin(q1e)*Sin(q3e)) - Cos(q1e)*Sin(q2e)*Sin(q4e)) +
        (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e))*Sin(q6e));

    //L44
    le_rtm_b(3,3) = 1;

}

//3D head rototranslational matrix inverse -----------------------------------------
void calc_le_rototranslational_matrix_inv(vector<double> mu, vector<double> a)
{
	double q1e = le_q1b + le_q1;
	double q2e = le_q2b + le_q2;
	double q3e = le_q3b + le_q3;
	double q4e = le_q4b + mu[0]; //neck_pitch
	double q5e = le_q5b + le_q5;
	double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
	double q8e = le_q8b + le_q8;

	//L11i
	inv_le_rtm(0,0) = Cos(q8e)*(Cos(q7e)*(Sin(q1e)*Sin(q2e)*
         (Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
        Cos(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) + 
           Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e)) + 
        Cos(q2e)*Sin(q1e)*(Cos(q6e)*Sin(q3e)*Sin(q5e) + 
           Cos(q3e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e)))) + 
     (Cos(q1e)*(-(Cos(q3e)*Cos(q5e)) + Cos(q4e)*Sin(q3e)*Sin(q5e)) + 
        Sin(q1e)*(Sin(q2e)*Sin(q4e)*Sin(q5e) - 
           Cos(q2e)*(Cos(q5e)*Sin(q3e) + Cos(q3e)*Cos(q4e)*Sin(q5e))))*
      Sin(q7e)) + (-(Cos(q4e)*Cos(q6e)*Sin(q1e)*Sin(q2e)) + 
     Cos(q6e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*
      Sin(q4e) + Cos(q5e)*(-(Cos(q2e)*Cos(q3e)*Cos(q4e)*Sin(q1e)) + 
        Cos(q1e)*Cos(q4e)*Sin(q3e) + Sin(q1e)*Sin(q2e)*Sin(q4e))*Sin(q6e) + 
     (Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e)*Sin(q6e))*
   Sin(q8e);
	
	//L21i
	inv_le_rtm(1,0) = Cos(q7e)*(Cos(q1e)*(-(Cos(q3e)*Cos(q5e)) + Cos(q4e)*Sin(q3e)*Sin(q5e)) + 
     Sin(q1e)*(Sin(q2e)*Sin(q4e)*Sin(q5e) - 
        Cos(q2e)*(Cos(q5e)*Sin(q3e) + Cos(q3e)*Cos(q4e)*Sin(q5e)))) - 
  (Sin(q1e)*Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
     Cos(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) + 
        Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e)) + 
     Cos(q2e)*Sin(q1e)*(Cos(q6e)*Sin(q3e)*Sin(q5e) + 
        Cos(q3e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e))))*
   Sin(q7e);
	
	//L31i
	inv_le_rtm(2,0) = Cos(q6e)*Cos(q8e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) + 
     (Cos(q2e)*Cos(q3e)*Sin(q1e) - Cos(q1e)*Sin(q3e))*Sin(q4e)) - 
  Cos(q8e)*(Cos(q5e)*Sin(q1e)*Sin(q2e)*Sin(q4e) + 
     Cos(q1e)*(Cos(q4e)*Cos(q5e)*Sin(q3e) + Cos(q3e)*Sin(q5e)) + 
     Cos(q2e)*Sin(q1e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)) + Sin(q3e)*Sin(q5e)))*
   Sin(q6e) + Cos(q7e)*(Sin(q1e)*Sin(q2e)*
      (Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
     Cos(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) + 
        Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e)) + 
     Cos(q2e)*Sin(q1e)*(Cos(q6e)*Sin(q3e)*Sin(q5e) + 
        Cos(q3e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e))))*
   Sin(q8e) + (Cos(q1e)*(-(Cos(q3e)*Cos(q5e)) + 
        Cos(q4e)*Sin(q3e)*Sin(q5e)) + 
     Sin(q1e)*(Sin(q2e)*Sin(q4e)*Sin(q5e) - 
        Cos(q2e)*(Cos(q5e)*Sin(q3e) + Cos(q3e)*Cos(q4e)*Sin(q5e))))*Sin(q7e)*
   Sin(q8e);
	
	//L41i
	inv_le_rtm(3,0) = 0;
	
	//L12i
	inv_le_rtm(0,1) = Sin(q2e)*Sin(q3e)*(Cos(q6e)*Cos(q7e)*Cos(q8e)*Sin(q5e) - 
     Cos(q5e)*Cos(q8e)*Sin(q7e) + Sin(q5e)*Sin(q6e)*Sin(q8e)) - 
  Cos(q2e)*(Cos(q4e)*Cos(q7e)*Cos(q8e)*Sin(q6e) + 
     Cos(q8e)*Sin(q4e)*Sin(q5e)*Sin(q7e) - Cos(q4e)*Cos(q6e)*Sin(q8e) + 
     Cos(q5e)*Sin(q4e)*(Cos(q6e)*Cos(q7e)*Cos(q8e) + Sin(q6e)*Sin(q8e))) - 
  Cos(q3e)*Sin(q2e)*(Sin(q4e)*(-(Cos(q7e)*Cos(q8e)*Sin(q6e)) + 
        Cos(q6e)*Sin(q8e)) + Cos(q4e)*
      (Cos(q5e)*Cos(q6e)*Cos(q7e)*Cos(q8e) + Cos(q8e)*Sin(q5e)*Sin(q7e) + 
        Cos(q5e)*Sin(q6e)*Sin(q8e)));
	
	//L22i
	inv_le_rtm(1,1) = -(Cos(q7e)*(Cos(q5e)*Sin(q2e)*Sin(q3e) + 
       (Cos(q3e)*Cos(q4e)*Sin(q2e) + Cos(q2e)*Sin(q4e))*Sin(q5e))) + 
  (-(Cos(q6e)*Sin(q2e)*Sin(q3e)*Sin(q5e)) + 
     Cos(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
     Cos(q3e)*Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e)))*
   Sin(q7e);
	
	//L32i
	inv_le_rtm(2,1) = Cos(q6e)*Cos(q8e)*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e)) + 
  Cos(q8e)*(Cos(q3e)*Cos(q4e)*Cos(q5e)*Sin(q2e) + 
     Cos(q2e)*Cos(q5e)*Sin(q4e) - Sin(q2e)*Sin(q3e)*Sin(q5e))*Sin(q6e) + 
  Cos(q7e)*(Cos(q6e)*Sin(q2e)*Sin(q3e)*Sin(q5e) - 
     Cos(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
     Cos(q3e)*Sin(q2e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e)))*
   Sin(q8e) - (Cos(q5e)*Sin(q2e)*Sin(q3e) + 
     (Cos(q3e)*Cos(q4e)*Sin(q2e) + Cos(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e)*
   Sin(q8e);
	
	//L42i
	inv_le_rtm(3,1) = 0;
	
	//L13i
	inv_le_rtm(0,2) = Cos(q8e)*(Sin(q1e)*(Cos(q3e)*Cos(q6e)*Cos(q7e)*Sin(q5e) - 
        Cos(q7e)*Sin(q3e)*Sin(q4e)*Sin(q6e) - Cos(q3e)*Cos(q5e)*Sin(q7e) + 
        Cos(q4e)*Sin(q3e)*(Cos(q5e)*Cos(q6e)*Cos(q7e) + Sin(q5e)*Sin(q7e))) \
+ Cos(q1e)*(-(Sin(q2e)*(Cos(q5e)*Cos(q6e)*Cos(q7e)*Sin(q4e) + 
             Cos(q4e)*Cos(q7e)*Sin(q6e) + Sin(q4e)*Sin(q5e)*Sin(q7e))) + 
        Cos(q2e)*(Sin(q3e)*(-(Cos(q6e)*Cos(q7e)*Sin(q5e)) + 
              Cos(q5e)*Sin(q7e)) + 
           Cos(q3e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Cos(q7e) - 
              Cos(q7e)*Sin(q4e)*Sin(q6e) + Cos(q4e)*Sin(q5e)*Sin(q7e))))) + 
  (Cos(q1e)*Cos(q4e)*Cos(q6e)*Sin(q2e) + 
     Cos(q6e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e) + 
     Cos(q5e)*(Cos(q4e)*Sin(q1e)*Sin(q3e) + 
        Cos(q1e)*(Cos(q2e)*Cos(q3e)*Cos(q4e) - Sin(q2e)*Sin(q4e)))*Sin(q6e) \
+ (Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e)*Sin(q6e))*
   Sin(q8e);
	
	//L23i
	inv_le_rtm(1,2) = Cos(q7e)*(Cos(q4e)*Sin(q1e)*Sin(q3e)*Sin(q5e) + 
     Cos(q3e)*(-(Cos(q5e)*Sin(q1e)) + 
        Cos(q1e)*Cos(q2e)*Cos(q4e)*Sin(q5e)) + 
     Cos(q1e)*(Cos(q2e)*Cos(q5e)*Sin(q3e) - Sin(q2e)*Sin(q4e)*Sin(q5e))) + 
  (-(Sin(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) + 
          Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e))) + 
     Cos(q1e)*(Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
        Cos(q2e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)*Cos(q6e)) + 
           Cos(q6e)*Sin(q3e)*Sin(q5e) + Cos(q3e)*Sin(q4e)*Sin(q6e))))*
   Sin(q7e);
	
	//L33i
	inv_le_rtm(2,2) = -(Cos(q6e)*Cos(q8e)*(Sin(q1e)*Sin(q3e)*Sin(q4e) + 
       Cos(q1e)*(Cos(q4e)*Sin(q2e) + Cos(q2e)*Cos(q3e)*Sin(q4e)))) - 
  Cos(q8e)*(Sin(q1e)*(Cos(q4e)*Cos(q5e)*Sin(q3e) + Cos(q3e)*Sin(q5e)) - 
     Cos(q1e)*(Cos(q5e)*Sin(q2e)*Sin(q4e) + 
        Cos(q2e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)) + Sin(q3e)*Sin(q5e))))*
   Sin(q6e) + Cos(q7e)*(Sin(q1e)*
      (Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) + Cos(q3e)*Cos(q6e)*Sin(q5e) - 
        Sin(q3e)*Sin(q4e)*Sin(q6e)) - 
     Cos(q1e)*(Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
        Cos(q2e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)*Cos(q6e)) + 
           Cos(q6e)*Sin(q3e)*Sin(q5e) + Cos(q3e)*Sin(q4e)*Sin(q6e))))*
   Sin(q8e) + (Cos(q4e)*Sin(q1e)*Sin(q3e)*Sin(q5e) + 
     Cos(q3e)*(-(Cos(q5e)*Sin(q1e)) + 
        Cos(q1e)*Cos(q2e)*Cos(q4e)*Sin(q5e)) + 
     Cos(q1e)*(Cos(q2e)*Cos(q5e)*Sin(q3e) - Sin(q2e)*Sin(q4e)*Sin(q5e)))*
   Sin(q7e)*Sin(q8e);
	
	//L43i
	inv_le_rtm(3,2) = 0;
	
	//L14i
	inv_le_rtm(0,3) = Cos(q7e)*Cos(q8e)*(-a6e - Cos(q5e)*Cos(q6e)*
      (a4e + Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e)) - d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) + 
     a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) + d5e*Sin(q6e) + 
     d3e*Cos(q4e)*Sin(q6e) + a1e*Cos(q4e)*Sin(q2e)*Sin(q6e) + 
     a3e*Sin(q4e)*Sin(q6e) + a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q6e) + 
     d2e*Sin(q3e)*Sin(q4e)*Sin(q6e)) + 
  Cos(q8e)*(d6e - a1e*Cos(q2e)*Cos(q5e)*Sin(q3e) - a4e*Sin(q5e) - 
     a3e*Cos(q4e)*Sin(q5e) - d2e*Cos(q4e)*Sin(q3e)*Sin(q5e) + 
     d3e*Sin(q4e)*Sin(q5e) + a1e*Sin(q2e)*Sin(q4e)*Sin(q5e) + 
     Cos(q3e)*(d2e*Cos(q5e) - a1e*Cos(q2e)*Cos(q4e)*Sin(q5e)))*Sin(q7e) + 
  (d7e - Cos(q6e)*(d5e + Cos(q4e)*(d3e + a1e*Sin(q2e)) + 
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e)) - 
     Cos(q5e)*(a4e + Cos(q4e)*
         (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e))*Sin(q6e) - 
     d2e*Cos(q3e)*Sin(q5e)*Sin(q6e) + 
     a1e*Cos(q2e)*Sin(q3e)*Sin(q5e)*Sin(q6e))*Sin(q8e);
	
	//L24i
	inv_le_rtm(1,3) = Cos(q7e)*(d6e - a1e*Cos(q2e)*Cos(q5e)*Sin(q3e) - a4e*Sin(q5e) - 
     a3e*Cos(q4e)*Sin(q5e) - d2e*Cos(q4e)*Sin(q3e)*Sin(q5e) + 
     d3e*Sin(q4e)*Sin(q5e) + a1e*Sin(q2e)*Sin(q4e)*Sin(q5e) + 
     Cos(q3e)*(d2e*Cos(q5e) - a1e*Cos(q2e)*Cos(q4e)*Sin(q5e))) + 
  (a6e + Cos(q5e)*Cos(q6e)*(a4e + 
        Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e)) + d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) - 
     a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) - d5e*Sin(q6e) - 
     d3e*Cos(q4e)*Sin(q6e) - a1e*Cos(q4e)*Sin(q2e)*Sin(q6e) - 
     a3e*Sin(q4e)*Sin(q6e) - a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q6e) - 
     d2e*Sin(q3e)*Sin(q4e)*Sin(q6e))*Sin(q7e);
	
	//L34i
	inv_le_rtm(2,3) = Cos(q8e)*(-d7e + Cos(q6e)*(d5e + Cos(q4e)*(d3e + a1e*Sin(q2e)) + 
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e)) + 
     Cos(q5e)*(a4e + Cos(q4e)*
         (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e))*Sin(q6e) + 
     d2e*Cos(q3e)*Sin(q5e)*Sin(q6e) - 
     a1e*Cos(q2e)*Sin(q3e)*Sin(q5e)*Sin(q6e)) + 
  (Cos(q7e)*(-a6e - Cos(q5e)*Cos(q6e)*
         (a4e + Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
           (d3e + a1e*Sin(q2e))*Sin(q4e)) - 
        d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) + 
        a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) + d5e*Sin(q6e) + 
        d3e*Cos(q4e)*Sin(q6e) + a1e*Cos(q4e)*Sin(q2e)*Sin(q6e) + 
        a3e*Sin(q4e)*Sin(q6e) + a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q6e) + 
        d2e*Sin(q3e)*Sin(q4e)*Sin(q6e)) + 
     (d6e - a1e*Cos(q2e)*Cos(q5e)*Sin(q3e) - a4e*Sin(q5e) - 
        a3e*Cos(q4e)*Sin(q5e) - d2e*Cos(q4e)*Sin(q3e)*Sin(q5e) + 
        d3e*Sin(q4e)*Sin(q5e) + a1e*Sin(q2e)*Sin(q4e)*Sin(q5e) + 
        Cos(q3e)*(d2e*Cos(q5e) - a1e*Cos(q2e)*Cos(q4e)*Sin(q5e)))*Sin(q7e))*
   Sin(q8e);
	
	//L44i
	inv_le_rtm(3,3) = 1;

}

//3D head rototranslational matrix inverse derivative wrt mu1e ---------------------
void calc_le_rototranslational_matrix_inv_mu1e(vector<double> mu, vector<double> a)
{
	double q1e = le_q1b + le_q1;
	double q2e = le_q2b + le_q2;
	double q3e = le_q3b + le_q3;
	double q4e = le_q4b + mu[0]; //neck_pitch
	double q5e = le_q5b + le_q5;
	double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
	double q8e = le_q8b + le_q8;

	//L11id1
	d1_inv_le_rtm(0,0) = Cos(q8e)*(Cos(q7e)*(Cos(q2e)*Cos(q3e)*Sin(q1e)*
         (Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
        Cos(q1e)*(-(Cos(q5e)*Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
           Cos(q4e)*Sin(q3e)*Sin(q6e)) + 
        Sin(q1e)*Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e))) \
+ (-(Cos(q1e)*Sin(q3e)*Sin(q4e)*Sin(q5e)) + 
        Sin(q1e)*(Cos(q4e)*Sin(q2e)*Sin(q5e) + 
           Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e)))*Sin(q7e)) + 
  (Cos(q4e)*Cos(q6e)*(-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e)) + 
     Cos(q6e)*Sin(q1e)*Sin(q2e)*Sin(q4e) + 
     Cos(q5e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) + 
        Cos(q2e)*Cos(q3e)*Sin(q1e)*Sin(q4e) - Cos(q1e)*Sin(q3e)*Sin(q4e))*
      Sin(q6e))*Sin(q8e);
	
	//L21id1
	d1_inv_le_rtm(1,0) = Cos(q7e)*(-(Cos(q1e)*Sin(q3e)*Sin(q4e)*Sin(q5e)) + 
     Sin(q1e)*(Cos(q4e)*Sin(q2e)*Sin(q5e) + 
        Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e))) - 
  (Cos(q2e)*Cos(q3e)*Sin(q1e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + 
        Cos(q4e)*Sin(q6e)) + Cos(q1e)*
      (-(Cos(q5e)*Cos(q6e)*Sin(q3e)*Sin(q4e)) - Cos(q4e)*Sin(q3e)*Sin(q6e)) \
+ Sin(q1e)*Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e)))*
   Sin(q7e);
	
	//L31id1
	d1_inv_le_rtm(2,0) = Cos(q6e)*Cos(q8e)*(Cos(q4e)*(Cos(q2e)*Cos(q3e)*Sin(q1e) - 
        Cos(q1e)*Sin(q3e)) - Sin(q1e)*Sin(q2e)*Sin(q4e)) - 
  Cos(q8e)*(Cos(q4e)*Cos(q5e)*Sin(q1e)*Sin(q2e) + 
     Cos(q2e)*Cos(q3e)*Cos(q5e)*Sin(q1e)*Sin(q4e) - 
     Cos(q1e)*Cos(q5e)*Sin(q3e)*Sin(q4e))*Sin(q6e) + 
  Cos(q7e)*(Cos(q2e)*Cos(q3e)*Sin(q1e)*
      (Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) + 
     Cos(q1e)*(-(Cos(q5e)*Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
        Cos(q4e)*Sin(q3e)*Sin(q6e)) + 
     Sin(q1e)*Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e)))*
   Sin(q8e) + (-(Cos(q1e)*Sin(q3e)*Sin(q4e)*Sin(q5e)) + 
     Sin(q1e)*(Cos(q4e)*Sin(q2e)*Sin(q5e) + 
        Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e)))*Sin(q7e)*Sin(q8e);
	
	//L41id1
	d1_inv_le_rtm(3,0) = 0;
	
	//L12id1
	d1_inv_le_rtm(0,1) = -(Cos(q2e)*(-(Cos(q7e)*Cos(q8e)*Sin(q4e)*Sin(q6e)) + 
       Cos(q4e)*Cos(q8e)*Sin(q5e)*Sin(q7e) + Cos(q6e)*Sin(q4e)*Sin(q8e) + 
       Cos(q4e)*Cos(q5e)*(Cos(q6e)*Cos(q7e)*Cos(q8e) + Sin(q6e)*Sin(q8e)))) \
- Cos(q3e)*Sin(q2e)*(Cos(q4e)*(-(Cos(q7e)*Cos(q8e)*Sin(q6e)) + 
        Cos(q6e)*Sin(q8e)) - Sin(q4e)*
      (Cos(q5e)*Cos(q6e)*Cos(q7e)*Cos(q8e) + Cos(q8e)*Sin(q5e)*Sin(q7e) + 
        Cos(q5e)*Sin(q6e)*Sin(q8e)));
	
	//L22id1
	d1_inv_le_rtm(1,1) = -(Cos(q7e)*(Cos(q2e)*Cos(q4e) - Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q5e)) + 
  (Cos(q3e)*Sin(q2e)*(-(Cos(q5e)*Cos(q6e)*Sin(q4e)) - Cos(q4e)*Sin(q6e)) + 
     Cos(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e)))*Sin(q7e);
	
	//L32id1
	d1_inv_le_rtm(2,1) = Cos(q6e)*Cos(q8e)*(Cos(q3e)*Cos(q4e)*Sin(q2e) + Cos(q2e)*Sin(q4e)) + 
  Cos(q8e)*(Cos(q2e)*Cos(q4e)*Cos(q5e) - 
     Cos(q3e)*Cos(q5e)*Sin(q2e)*Sin(q4e))*Sin(q6e) + 
  Cos(q7e)*(Cos(q3e)*Sin(q2e)*
      (Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) - 
     Cos(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e)))*Sin(q8e) - 
  (Cos(q2e)*Cos(q4e) - Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q5e)*Sin(q7e)*Sin(q8e);
	
	//L42id1
	d1_inv_le_rtm(3,1) = 0;
	
	//L13id1
	d1_inv_le_rtm(0,2) = Cos(q8e)*(Sin(q1e)*(-(Cos(q4e)*Cos(q7e)*Sin(q3e)*Sin(q6e)) - 
        Sin(q3e)*Sin(q4e)*(Cos(q5e)*Cos(q6e)*Cos(q7e) + Sin(q5e)*Sin(q7e))) \
+ Cos(q1e)*(-(Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Cos(q7e) - 
             Cos(q7e)*Sin(q4e)*Sin(q6e) + Cos(q4e)*Sin(q5e)*Sin(q7e))) + 
        Cos(q2e)*Cos(q3e)*(-(Cos(q5e)*Cos(q6e)*Cos(q7e)*Sin(q4e)) - 
           Cos(q4e)*Cos(q7e)*Sin(q6e) - Sin(q4e)*Sin(q5e)*Sin(q7e)))) + 
  (Cos(q4e)*Cos(q6e)*(Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e)) - 
     Cos(q1e)*Cos(q6e)*Sin(q2e)*Sin(q4e) + 
     Cos(q5e)*(-(Sin(q1e)*Sin(q3e)*Sin(q4e)) + 
        Cos(q1e)*(-(Cos(q4e)*Sin(q2e)) - Cos(q2e)*Cos(q3e)*Sin(q4e)))*
      Sin(q6e))*Sin(q8e);
	
	//L23id1
	d1_inv_le_rtm(1,2) = Cos(q7e)*(-(Cos(q1e)*Cos(q4e)*Sin(q2e)*Sin(q5e)) - 
     Cos(q1e)*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e) - 
     Sin(q1e)*Sin(q3e)*Sin(q4e)*Sin(q5e)) + 
  (-(Sin(q1e)*(-(Cos(q5e)*Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
          Cos(q4e)*Sin(q3e)*Sin(q6e))) + 
     Cos(q1e)*(Cos(q2e)*(Cos(q3e)*Cos(q5e)*Cos(q6e)*Sin(q4e) + 
           Cos(q3e)*Cos(q4e)*Sin(q6e)) + 
        Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e))))*Sin(q7e);
	
	//L33id1
	d1_inv_le_rtm(2,2) = -(Cos(q6e)*Cos(q8e)*(Cos(q4e)*Sin(q1e)*Sin(q3e) + 
       Cos(q1e)*(Cos(q2e)*Cos(q3e)*Cos(q4e) - Sin(q2e)*Sin(q4e)))) - 
  Cos(q8e)*(-(Cos(q5e)*Sin(q1e)*Sin(q3e)*Sin(q4e)) - 
     Cos(q1e)*(Cos(q4e)*Cos(q5e)*Sin(q2e) + 
        Cos(q2e)*Cos(q3e)*Cos(q5e)*Sin(q4e)))*Sin(q6e) + 
  Cos(q7e)*(Sin(q1e)*(-(Cos(q5e)*Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
        Cos(q4e)*Sin(q3e)*Sin(q6e)) - 
     Cos(q1e)*(Cos(q2e)*(Cos(q3e)*Cos(q5e)*Cos(q6e)*Sin(q4e) + 
           Cos(q3e)*Cos(q4e)*Sin(q6e)) + 
        Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e))))*Sin(q8e) \
+ (-(Cos(q1e)*Cos(q4e)*Sin(q2e)*Sin(q5e)) - 
     Cos(q1e)*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e) - 
     Sin(q1e)*Sin(q3e)*Sin(q4e)*Sin(q5e))*Sin(q7e)*Sin(q8e);
	
	//L43id1
	d1_inv_le_rtm(3,2) = 0;
	
	//L14id1
	d1_inv_le_rtm(0,3) = Cos(q7e)*Cos(q8e)*(-(Cos(q5e)*Cos(q6e)*
        (-(Cos(q4e)*(d3e + a1e*Sin(q2e))) - 
          (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e))) + 
     a3e*Cos(q4e)*Sin(q6e) + a1e*Cos(q2e)*Cos(q3e)*Cos(q4e)*Sin(q6e) + 
     d2e*Cos(q4e)*Sin(q3e)*Sin(q6e) - d3e*Sin(q4e)*Sin(q6e) - 
     a1e*Sin(q2e)*Sin(q4e)*Sin(q6e)) + 
  Cos(q8e)*(d3e*Cos(q4e)*Sin(q5e) + a1e*Cos(q4e)*Sin(q2e)*Sin(q5e) + 
     a3e*Sin(q4e)*Sin(q5e) + a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e) + 
     d2e*Sin(q3e)*Sin(q4e)*Sin(q5e))*Sin(q7e) + 
  (-(Cos(q6e)*(Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
          (d3e + a1e*Sin(q2e))*Sin(q4e))) - 
     Cos(q5e)*(-(Cos(q4e)*(d3e + a1e*Sin(q2e))) - 
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e))*Sin(q6e))*
   Sin(q8e);
	
	//L24id1
	d1_inv_le_rtm(1,3) = Cos(q7e)*(d3e*Cos(q4e)*Sin(q5e) + a1e*Cos(q4e)*Sin(q2e)*Sin(q5e) + 
     a3e*Sin(q4e)*Sin(q5e) + a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e) + 
     d2e*Sin(q3e)*Sin(q4e)*Sin(q5e)) + 
  (Cos(q5e)*Cos(q6e)*(-(Cos(q4e)*(d3e + a1e*Sin(q2e))) - 
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e)) - 
     a3e*Cos(q4e)*Sin(q6e) - a1e*Cos(q2e)*Cos(q3e)*Cos(q4e)*Sin(q6e) - 
     d2e*Cos(q4e)*Sin(q3e)*Sin(q6e) + d3e*Sin(q4e)*Sin(q6e) + 
     a1e*Sin(q2e)*Sin(q4e)*Sin(q6e))*Sin(q7e);
	
	//L34id1
	d1_inv_le_rtm(2,3) = Cos(q8e)*(Cos(q6e)*(Cos(q4e)*
         (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e)) + 
     Cos(q5e)*(-(Cos(q4e)*(d3e + a1e*Sin(q2e))) - 
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e))*Sin(q6e)) + 
  (Cos(q7e)*(-(Cos(q5e)*Cos(q6e)*
           (-(Cos(q4e)*(d3e + a1e*Sin(q2e))) - 
             (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e))) + 
        a3e*Cos(q4e)*Sin(q6e) + a1e*Cos(q2e)*Cos(q3e)*Cos(q4e)*Sin(q6e) + 
        d2e*Cos(q4e)*Sin(q3e)*Sin(q6e) - d3e*Sin(q4e)*Sin(q6e) - 
        a1e*Sin(q2e)*Sin(q4e)*Sin(q6e)) + 
     (d3e*Cos(q4e)*Sin(q5e) + a1e*Cos(q4e)*Sin(q2e)*Sin(q5e) + 
        a3e*Sin(q4e)*Sin(q5e) + a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q5e) + 
        d2e*Sin(q3e)*Sin(q4e)*Sin(q5e))*Sin(q7e))*Sin(q8e);
	
	//L44id1
	d1_inv_le_rtm(3,3) = 0;

}

//3D head rototranslational matrix inverse derivative wrt mu2e ---------------------
void calc_le_rototranslational_matrix_inv_mu2e(vector<double> mu, vector<double> a)
{
	double q1e = le_q1b + le_q1;
	double q2e = le_q2b + le_q2;
	double q3e = le_q3b + le_q3;
	double q4e = le_q4b + mu[0]; //neck_pitch
	double q5e = le_q5b + le_q5;
	double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
	double q8e = le_q8b + le_q8;

	//L11id2
	d2_inv_le_rtm(0,0) = Cos(q7e)*Cos(q8e)*(Sin(q1e)*Sin(q2e)*
      (Cos(q4e)*Cos(q6e) - Cos(q5e)*Sin(q4e)*Sin(q6e)) + 
     Cos(q1e)*(-(Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
        Cos(q4e)*Cos(q5e)*Sin(q3e)*Sin(q6e) - Cos(q3e)*Sin(q5e)*Sin(q6e)) + 
     Cos(q2e)*Sin(q1e)*(-(Sin(q3e)*Sin(q5e)*Sin(q6e)) + 
        Cos(q3e)*(Cos(q6e)*Sin(q4e) + Cos(q4e)*Cos(q5e)*Sin(q6e)))) + 
  (Cos(q5e)*Cos(q6e)*(-(Cos(q2e)*Cos(q3e)*Cos(q4e)*Sin(q1e)) + 
        Cos(q1e)*Cos(q4e)*Sin(q3e) + Sin(q1e)*Sin(q2e)*Sin(q4e)) + 
     Cos(q6e)*(Cos(q1e)*Cos(q3e) + Cos(q2e)*Sin(q1e)*Sin(q3e))*Sin(q5e) + 
     Cos(q4e)*Sin(q1e)*Sin(q2e)*Sin(q6e) - 
     (-(Cos(q2e)*Cos(q3e)*Sin(q1e)) + Cos(q1e)*Sin(q3e))*Sin(q4e)*Sin(q6e))*
   Sin(q8e);
	
	//L21id2
	d2_inv_le_rtm(1,0) = -((Sin(q1e)*Sin(q2e)*(Cos(q4e)*Cos(q6e) - Cos(q5e)*Sin(q4e)*Sin(q6e)) + 
      Cos(q1e)*(-(Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
         Cos(q4e)*Cos(q5e)*Sin(q3e)*Sin(q6e) - Cos(q3e)*Sin(q5e)*Sin(q6e)) \
+ Cos(q2e)*Sin(q1e)*(-(Sin(q3e)*Sin(q5e)*Sin(q6e)) + 
         Cos(q3e)*(Cos(q6e)*Sin(q4e) + Cos(q4e)*Cos(q5e)*Sin(q6e))))*Sin(q7e));
	
	//L31id2
	d2_inv_le_rtm(2,0) = -(Cos(q6e)*Cos(q8e)*(Cos(q5e)*Sin(q1e)*Sin(q2e)*Sin(q4e) + 
       Cos(q1e)*(Cos(q4e)*Cos(q5e)*Sin(q3e) + Cos(q3e)*Sin(q5e)) + 
       Cos(q2e)*Sin(q1e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)) + Sin(q3e)*Sin(q5e))\
)) - Cos(q8e)*(Cos(q4e)*Sin(q1e)*Sin(q2e) + 
     (Cos(q2e)*Cos(q3e)*Sin(q1e) - Cos(q1e)*Sin(q3e))*Sin(q4e))*Sin(q6e) + 
  Cos(q7e)*(Sin(q1e)*Sin(q2e)*(Cos(q4e)*Cos(q6e) - 
        Cos(q5e)*Sin(q4e)*Sin(q6e)) + 
     Cos(q1e)*(-(Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
        Cos(q4e)*Cos(q5e)*Sin(q3e)*Sin(q6e) - Cos(q3e)*Sin(q5e)*Sin(q6e)) + 
     Cos(q2e)*Sin(q1e)*(-(Sin(q3e)*Sin(q5e)*Sin(q6e)) + 
        Cos(q3e)*(Cos(q6e)*Sin(q4e) + Cos(q4e)*Cos(q5e)*Sin(q6e))))*Sin(q8e);
	
	//L41id2
	d2_inv_le_rtm(3,0) = 0;
	
	//L12id2
	d2_inv_le_rtm(0,1) = Sin(q2e)*Sin(q3e)*(-(Cos(q7e)*Cos(q8e)*Sin(q5e)*Sin(q6e)) + 
     Cos(q6e)*Sin(q5e)*Sin(q8e)) - 
  Cos(q2e)*(Cos(q4e)*Cos(q6e)*Cos(q7e)*Cos(q8e) + 
     Cos(q4e)*Sin(q6e)*Sin(q8e) + 
     Cos(q5e)*Sin(q4e)*(-(Cos(q7e)*Cos(q8e)*Sin(q6e)) + Cos(q6e)*Sin(q8e))) \
- Cos(q3e)*Sin(q2e)*(Cos(q4e)*(-(Cos(q5e)*Cos(q7e)*Cos(q8e)*Sin(q6e)) + 
        Cos(q5e)*Cos(q6e)*Sin(q8e)) + 
     Sin(q4e)*(-(Cos(q6e)*Cos(q7e)*Cos(q8e)) - Sin(q6e)*Sin(q8e)));
	
	//L22id2
	d2_inv_le_rtm(1,1) = (Sin(q2e)*Sin(q3e)*Sin(q5e)*Sin(q6e) + 
    Cos(q3e)*Sin(q2e)*(-(Cos(q6e)*Sin(q4e)) - Cos(q4e)*Cos(q5e)*Sin(q6e)) + 
    Cos(q2e)*(Cos(q4e)*Cos(q6e) - Cos(q5e)*Sin(q4e)*Sin(q6e)))*Sin(q7e);
	
	//L32id2
	d2_inv_le_rtm(2,1) = Cos(q6e)*Cos(q8e)*(Cos(q3e)*Cos(q4e)*Cos(q5e)*Sin(q2e) + 
     Cos(q2e)*Cos(q5e)*Sin(q4e) - Sin(q2e)*Sin(q3e)*Sin(q5e)) - 
  Cos(q8e)*(-(Cos(q2e)*Cos(q4e)) + Cos(q3e)*Sin(q2e)*Sin(q4e))*Sin(q6e) + 
  Cos(q7e)*(-(Sin(q2e)*Sin(q3e)*Sin(q5e)*Sin(q6e)) + 
     Cos(q3e)*Sin(q2e)*(Cos(q6e)*Sin(q4e) + Cos(q4e)*Cos(q5e)*Sin(q6e)) - 
     Cos(q2e)*(Cos(q4e)*Cos(q6e) - Cos(q5e)*Sin(q4e)*Sin(q6e)))*Sin(q8e);
	
	//L42id2
	d2_inv_le_rtm(3,1) = 0;
	
	//L13id2
	d2_inv_le_rtm(0,2) = Cos(q8e)*(Sin(q1e)*(-(Cos(q6e)*Cos(q7e)*Sin(q3e)*Sin(q4e)) - 
        Cos(q4e)*Cos(q5e)*Cos(q7e)*Sin(q3e)*Sin(q6e) - 
        Cos(q3e)*Cos(q7e)*Sin(q5e)*Sin(q6e)) + 
     Cos(q1e)*(-(Sin(q2e)*(Cos(q4e)*Cos(q6e)*Cos(q7e) - 
             Cos(q5e)*Cos(q7e)*Sin(q4e)*Sin(q6e))) + 
        Cos(q2e)*(Cos(q7e)*Sin(q3e)*Sin(q5e)*Sin(q6e) + 
           Cos(q3e)*(-(Cos(q6e)*Cos(q7e)*Sin(q4e)) - 
              Cos(q4e)*Cos(q5e)*Cos(q7e)*Sin(q6e))))) + 
  (Cos(q5e)*Cos(q6e)*(Cos(q4e)*Sin(q1e)*Sin(q3e) + 
        Cos(q1e)*(Cos(q2e)*Cos(q3e)*Cos(q4e) - Sin(q2e)*Sin(q4e))) + 
     Cos(q6e)*(Cos(q3e)*Sin(q1e) - Cos(q1e)*Cos(q2e)*Sin(q3e))*Sin(q5e) - 
     Cos(q1e)*Cos(q4e)*Sin(q2e)*Sin(q6e) - 
     (Cos(q1e)*Cos(q2e)*Cos(q3e) + Sin(q1e)*Sin(q3e))*Sin(q4e)*Sin(q6e))*
   Sin(q8e);
	
	//L23id2
	d2_inv_le_rtm(1,2) = (-(Sin(q1e)*(-(Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
         Cos(q4e)*Cos(q5e)*Sin(q3e)*Sin(q6e) - Cos(q3e)*Sin(q5e)*Sin(q6e))) \
+ Cos(q1e)*(Sin(q2e)*(Cos(q4e)*Cos(q6e) - Cos(q5e)*Sin(q4e)*Sin(q6e)) + 
       Cos(q2e)*(Cos(q3e)*Cos(q6e)*Sin(q4e) + 
          Cos(q3e)*Cos(q4e)*Cos(q5e)*Sin(q6e) - Sin(q3e)*Sin(q5e)*Sin(q6e)))\
)*Sin(q7e);
	
	//L33id2
	d2_inv_le_rtm(2,2) = -(Cos(q6e)*Cos(q8e)*(Sin(q1e)*(Cos(q4e)*Cos(q5e)*Sin(q3e) + 
          Cos(q3e)*Sin(q5e)) - 
       Cos(q1e)*(Cos(q5e)*Sin(q2e)*Sin(q4e) + 
          Cos(q2e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)) + Sin(q3e)*Sin(q5e))))) + 
  Cos(q8e)*(Sin(q1e)*Sin(q3e)*Sin(q4e) + 
     Cos(q1e)*(Cos(q4e)*Sin(q2e) + Cos(q2e)*Cos(q3e)*Sin(q4e)))*Sin(q6e) + 
  Cos(q7e)*(Sin(q1e)*(-(Cos(q6e)*Sin(q3e)*Sin(q4e)) - 
        Cos(q4e)*Cos(q5e)*Sin(q3e)*Sin(q6e) - Cos(q3e)*Sin(q5e)*Sin(q6e)) - 
     Cos(q1e)*(Sin(q2e)*(Cos(q4e)*Cos(q6e) - Cos(q5e)*Sin(q4e)*Sin(q6e)) + 
        Cos(q2e)*(Cos(q3e)*Cos(q6e)*Sin(q4e) + 
           Cos(q3e)*Cos(q4e)*Cos(q5e)*Sin(q6e) - Sin(q3e)*Sin(q5e)*Sin(q6e))\
))*Sin(q8e);
	
	//L43id2
	d2_inv_le_rtm(3,2) = 0;
	
	//L14id2
	d2_inv_le_rtm(0,3) = Cos(q7e)*Cos(q8e)*(d5e*Cos(q6e) + d3e*Cos(q4e)*Cos(q6e) + 
     a1e*Cos(q4e)*Cos(q6e)*Sin(q2e) + a3e*Cos(q6e)*Sin(q4e) + 
     a1e*Cos(q2e)*Cos(q3e)*Cos(q6e)*Sin(q4e) + 
     d2e*Cos(q6e)*Sin(q3e)*Sin(q4e) + 
     Cos(q5e)*(a4e + Cos(q4e)*
         (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e))*Sin(q6e) + 
     d2e*Cos(q3e)*Sin(q5e)*Sin(q6e) - 
     a1e*Cos(q2e)*Sin(q3e)*Sin(q5e)*Sin(q6e)) + 
  (-(Cos(q5e)*Cos(q6e)*(a4e + Cos(q4e)*
           (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
          (d3e + a1e*Sin(q2e))*Sin(q4e))) - 
     d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) + 
     a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) + 
     (d5e + Cos(q4e)*(d3e + a1e*Sin(q2e)) + 
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e))*Sin(q6e))*
   Sin(q8e);
	
	//L24id2
	d2_inv_le_rtm(1,3) = (-(d5e*Cos(q6e)) - d3e*Cos(q4e)*Cos(q6e) - a1e*Cos(q4e)*Cos(q6e)*Sin(q2e) - 
    a3e*Cos(q6e)*Sin(q4e) - a1e*Cos(q2e)*Cos(q3e)*Cos(q6e)*Sin(q4e) - 
    d2e*Cos(q6e)*Sin(q3e)*Sin(q4e) - 
    Cos(q5e)*(a4e + Cos(q4e)*
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
       (d3e + a1e*Sin(q2e))*Sin(q4e))*Sin(q6e) - 
    d2e*Cos(q3e)*Sin(q5e)*Sin(q6e) + a1e*Cos(q2e)*Sin(q3e)*Sin(q5e)*Sin(q6e)\
)*Sin(q7e);
	
    //L34id2
	d2_inv_le_rtm(2,3) = Cos(q8e)*(Cos(q5e)*Cos(q6e)*(a4e + 
        Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e)) + d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) - 
     a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) - 
     (d5e + Cos(q4e)*(d3e + a1e*Sin(q2e)) + 
        (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e))*Sin(q4e))*Sin(q6e)) + 
  Cos(q7e)*(d5e*Cos(q6e) + d3e*Cos(q4e)*Cos(q6e) + 
     a1e*Cos(q4e)*Cos(q6e)*Sin(q2e) + a3e*Cos(q6e)*Sin(q4e) + 
     a1e*Cos(q2e)*Cos(q3e)*Cos(q6e)*Sin(q4e) + 
     d2e*Cos(q6e)*Sin(q3e)*Sin(q4e) + 
     Cos(q5e)*(a4e + Cos(q4e)*
         (a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) - 
        (d3e + a1e*Sin(q2e))*Sin(q4e))*Sin(q6e) + 
     d2e*Cos(q3e)*Sin(q5e)*Sin(q6e) - 
     a1e*Cos(q2e)*Sin(q3e)*Sin(q5e)*Sin(q6e))*Sin(q8e);
	
    //L44id2
	d2_inv_le_rtm(3,3) = 0;

}

//3D head rototranslational matrix inverse derivative wrt mu2e ---------------------
void calc_le_rototranslational_matrix_inv_mu3e(vector<double> mu, vector<double> a)
{
    double q1e = le_q1b + le_q1;
    double q2e = le_q2b + le_q2;
    double q3e = le_q3b + le_q3;
    double q4e = le_q4b + mu[0]; //neck_pitch
    double q5e = le_q5b + le_q5;
    double q6e = le_q6b + mu[1]; //neck_yaw
    double q7e = le_q7b + mu[2]; //eyes_tilt
    double q8e = le_q8b + le_q8;

    //L11id3
    d3_inv_le_rtm(0,0) = Cos(q8e)*(Cos(q7e)*(Cos(q1e)*(-(Cos(q3e)*Cos(q5e)) +
           Cos(q4e)*Sin(q3e)*Sin(q5e)) +
        Sin(q1e)*(Sin(q2e)*Sin(q4e)*Sin(q5e) -
           Cos(q2e)*(Cos(q5e)*Sin(q3e) + Cos(q3e)*Cos(q4e)*Sin(q5e)))) -
     (Sin(q1e)*Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) +
        Cos(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) +
           Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e)) +
        Cos(q2e)*Sin(q1e)*(Cos(q6e)*Sin(q3e)*Sin(q5e) +
           Cos(q3e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e))))*
      Sin(q7e));

    //L21id3
    d3_inv_le_rtm(1,0) = -(Cos(q7e)*(Sin(q1e)*Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) +
            Cos(q4e)*Sin(q6e)) +
         Cos(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) +
            Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e)) +
         Cos(q2e)*Sin(q1e)*(Cos(q6e)*Sin(q3e)*Sin(q5e) +
            Cos(q3e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e))))) -
    (Cos(q1e)*(-(Cos(q3e)*Cos(q5e)) + Cos(q4e)*Sin(q3e)*Sin(q5e)) +
       Sin(q1e)*(Sin(q2e)*Sin(q4e)*Sin(q5e) -
          Cos(q2e)*(Cos(q5e)*Sin(q3e) + Cos(q3e)*Cos(q4e)*Sin(q5e))))*Sin(q7e);

    //L31id3
    d3_inv_le_rtm(2,0) = Cos(q7e)*(Cos(q1e)*(-(Cos(q3e)*Cos(q5e)) + Cos(q4e)*Sin(q3e)*Sin(q5e)) +
           Sin(q1e)*(Sin(q2e)*Sin(q4e)*Sin(q5e) -
              Cos(q2e)*(Cos(q5e)*Sin(q3e) + Cos(q3e)*Cos(q4e)*Sin(q5e))))*Sin(q8e) \
      - (Sin(q1e)*Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) +
           Cos(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) +
              Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e)) +
           Cos(q2e)*Sin(q1e)*(Cos(q6e)*Sin(q3e)*Sin(q5e) +
              Cos(q3e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e))))*
         Sin(q7e)*Sin(q8e);

    //L41id3
    d3_inv_le_rtm(3,0) = 0;

    //L12id3
    d3_inv_le_rtm(0,1) = Sin(q2e)*Sin(q3e)*(-(Cos(q5e)*Cos(q7e)*Cos(q8e)) -
        Cos(q6e)*Cos(q8e)*Sin(q5e)*Sin(q7e)) -
     Cos(q2e)*(Cos(q7e)*Cos(q8e)*Sin(q4e)*Sin(q5e) -
        Cos(q5e)*Cos(q6e)*Cos(q8e)*Sin(q4e)*Sin(q7e) -
        Cos(q4e)*Cos(q8e)*Sin(q6e)*Sin(q7e)) -
     Cos(q3e)*Sin(q2e)*(Cos(q8e)*Sin(q4e)*Sin(q6e)*Sin(q7e) +
        Cos(q4e)*(Cos(q7e)*Cos(q8e)*Sin(q5e) -
           Cos(q5e)*Cos(q6e)*Cos(q8e)*Sin(q7e)));

    //L22id3
    d3_inv_le_rtm(1,1) = Cos(q7e)*(-(Cos(q6e)*Sin(q2e)*Sin(q3e)*Sin(q5e)) +
       Cos(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) +
       Cos(q3e)*Sin(q2e)*(Cos(q4e)*Cos(q5e)*Cos(q6e) - Sin(q4e)*Sin(q6e))) +
    (Cos(q5e)*Sin(q2e)*Sin(q3e) +
       (Cos(q3e)*Cos(q4e)*Sin(q2e) + Cos(q2e)*Sin(q4e))*Sin(q5e))*Sin(q7e);

    //L32id3
    d3_inv_le_rtm(2,1) = -(Cos(q7e)*(Cos(q5e)*Sin(q2e)*Sin(q3e) +
             (Cos(q3e)*Cos(q4e)*Sin(q2e) + Cos(q2e)*Sin(q4e))*Sin(q5e))*Sin(q8e)) \
      - (Cos(q6e)*Sin(q2e)*Sin(q3e)*Sin(q5e) -
           Cos(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) +
           Cos(q3e)*Sin(q2e)*(-(Cos(q4e)*Cos(q5e)*Cos(q6e)) + Sin(q4e)*Sin(q6e)))*
         Sin(q7e)*Sin(q8e);

    //L42id3
    d3_inv_le_rtm(3,1) = 0;

    //L13id3
    d3_inv_le_rtm(0,2) = Cos(q8e)*(Sin(q1e)*(-(Cos(q3e)*Cos(q5e)*Cos(q7e)) -
             Cos(q3e)*Cos(q6e)*Sin(q5e)*Sin(q7e) +
             Sin(q3e)*Sin(q4e)*Sin(q6e)*Sin(q7e) +
             Cos(q4e)*Sin(q3e)*(Cos(q7e)*Sin(q5e) - Cos(q5e)*Cos(q6e)*Sin(q7e))) +
          Cos(q1e)*(-(Sin(q2e)*(Cos(q7e)*Sin(q4e)*Sin(q5e) -
                  Cos(q5e)*Cos(q6e)*Sin(q4e)*Sin(q7e) - Cos(q4e)*Sin(q6e)*Sin(q7e)\
      )) + Cos(q2e)*(Sin(q3e)*(Cos(q5e)*Cos(q7e) + Cos(q6e)*Sin(q5e)*Sin(q7e)) +
                Cos(q3e)*(Cos(q4e)*Cos(q7e)*Sin(q5e) -
                   Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q7e) + Sin(q4e)*Sin(q6e)*Sin(q7e)\
      ))));

    //L23id3
    d3_inv_le_rtm(1,2) = Cos(q7e)*(-(Sin(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) +
               Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e))) +
          Cos(q1e)*(Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) +
             Cos(q2e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)*Cos(q6e)) +
                Cos(q6e)*Sin(q3e)*Sin(q5e) + Cos(q3e)*Sin(q4e)*Sin(q6e)))) -
       (Cos(q4e)*Sin(q1e)*Sin(q3e)*Sin(q5e) +
          Cos(q3e)*(-(Cos(q5e)*Sin(q1e)) +
             Cos(q1e)*Cos(q2e)*Cos(q4e)*Sin(q5e)) +
          Cos(q1e)*(Cos(q2e)*Cos(q5e)*Sin(q3e) - Sin(q2e)*Sin(q4e)*Sin(q5e)))*
        Sin(q7e);

    //L33id3
    d3_inv_le_rtm(2,2) = Cos(q7e)*(Cos(q4e)*Sin(q1e)*Sin(q3e)*Sin(q5e) +
           Cos(q3e)*(-(Cos(q5e)*Sin(q1e)) +
              Cos(q1e)*Cos(q2e)*Cos(q4e)*Sin(q5e)) +
           Cos(q1e)*(Cos(q2e)*Cos(q5e)*Sin(q3e) - Sin(q2e)*Sin(q4e)*Sin(q5e)))*
         Sin(q8e) - (Sin(q1e)*(Cos(q4e)*Cos(q5e)*Cos(q6e)*Sin(q3e) +
              Cos(q3e)*Cos(q6e)*Sin(q5e) - Sin(q3e)*Sin(q4e)*Sin(q6e)) -
           Cos(q1e)*(Sin(q2e)*(Cos(q5e)*Cos(q6e)*Sin(q4e) + Cos(q4e)*Sin(q6e)) +
              Cos(q2e)*(-(Cos(q3e)*Cos(q4e)*Cos(q5e)*Cos(q6e)) +
                 Cos(q6e)*Sin(q3e)*Sin(q5e) + Cos(q3e)*Sin(q4e)*Sin(q6e))))*
         Sin(q7e)*Sin(q8e);

    //L43id2
    d3_inv_le_rtm(3,2) = 0;

    //L14id3
    d3_inv_le_rtm(0,3) = Cos(q7e)*Cos(q8e)*(d6e - a1e*Cos(q2e)*Cos(q5e)*Sin(q3e) - a4e*Sin(q5e) -
            a3e*Cos(q4e)*Sin(q5e) - d2e*Cos(q4e)*Sin(q3e)*Sin(q5e) +
            d3e*Sin(q4e)*Sin(q5e) + a1e*Sin(q2e)*Sin(q4e)*Sin(q5e) +
            Cos(q3e)*(d2e*Cos(q5e) - a1e*Cos(q2e)*Cos(q4e)*Sin(q5e))) -
         Cos(q8e)*(-a6e - Cos(q5e)*Cos(q6e)*
             (a4e + Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) -
               (d3e + a1e*Sin(q2e))*Sin(q4e)) - d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) +
            a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) + d5e*Sin(q6e) +
            d3e*Cos(q4e)*Sin(q6e) + a1e*Cos(q4e)*Sin(q2e)*Sin(q6e) +
            a3e*Sin(q4e)*Sin(q6e) + a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q6e) +
            d2e*Sin(q3e)*Sin(q4e)*Sin(q6e))*Sin(q7e);

    //L24id3
    d3_inv_le_rtm(1,3) = Cos(q7e)*(a6e + Cos(q5e)*Cos(q6e)*
           (a4e + Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) -
             (d3e + a1e*Sin(q2e))*Sin(q4e)) + d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) -
          a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) - d5e*Sin(q6e) -
          d3e*Cos(q4e)*Sin(q6e) - a1e*Cos(q4e)*Sin(q2e)*Sin(q6e) -
          a3e*Sin(q4e)*Sin(q6e) - a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q6e) -
          d2e*Sin(q3e)*Sin(q4e)*Sin(q6e)) -
       (d6e - a1e*Cos(q2e)*Cos(q5e)*Sin(q3e) - a4e*Sin(q5e) -
          a3e*Cos(q4e)*Sin(q5e) - d2e*Cos(q4e)*Sin(q3e)*Sin(q5e) +
          d3e*Sin(q4e)*Sin(q5e) + a1e*Sin(q2e)*Sin(q4e)*Sin(q5e) +
          Cos(q3e)*(d2e*Cos(q5e) - a1e*Cos(q2e)*Cos(q4e)*Sin(q5e)))*Sin(q7e);

    //L34id3
    d3_inv_le_rtm(2,3) = (Cos(q7e)*(d6e - a1e*Cos(q2e)*Cos(q5e)*Sin(q3e) - a4e*Sin(q5e) -
            a3e*Cos(q4e)*Sin(q5e) - d2e*Cos(q4e)*Sin(q3e)*Sin(q5e) +
            d3e*Sin(q4e)*Sin(q5e) + a1e*Sin(q2e)*Sin(q4e)*Sin(q5e) +
            Cos(q3e)*(d2e*Cos(q5e) - a1e*Cos(q2e)*Cos(q4e)*Sin(q5e))) -
         (-a6e - Cos(q5e)*Cos(q6e)*
             (a4e + Cos(q4e)*(a3e + a1e*Cos(q2e)*Cos(q3e) + d2e*Sin(q3e)) -
               (d3e + a1e*Sin(q2e))*Sin(q4e)) -
            d2e*Cos(q3e)*Cos(q6e)*Sin(q5e) +
            a1e*Cos(q2e)*Cos(q6e)*Sin(q3e)*Sin(q5e) + d5e*Sin(q6e) +
            d3e*Cos(q4e)*Sin(q6e) + a1e*Cos(q4e)*Sin(q2e)*Sin(q6e) +
            a3e*Sin(q4e)*Sin(q6e) + a1e*Cos(q2e)*Cos(q3e)*Sin(q4e)*Sin(q6e) +
            d2e*Sin(q3e)*Sin(q4e)*Sin(q6e))*Sin(q7e))*Sin(q8e);

    //L44id3
    d3_inv_le_rtm(3,3) = 0;

}

//Left eye end effector position ---------------------------------------------------
double calc_le_endeffpos_x()
{
	double le_x;
	
	//calculate camera position vector
    vector<double> dof_le = {le_q4, le_q6, le_q7};
	calc_le_rototranslational_matrix(dof_le, dof_le);
	le_epv = le_rtm * le_repv;
	
	le_x = le_epv(0);
	
	le_x /= k; //(mm to m)
	
	return le_x;
}

double calc_le_endeffpos_y()
{
	double le_y;

	//calculate camera position vector
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix(dof_le, dof_le);
    le_epv = le_rtm * le_repv;
	
	le_y = le_epv(1);
	
	le_y /= k; //(mm to m)
	
	return le_y;
}

double calc_le_endeffpos_z()
{
	double le_z;

	//calculate camera position vector
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix(dof_le, dof_le);
    le_epv = le_rtm * le_repv;
	
	le_z = le_epv(2);
	
	le_z /= k; //(mm to m)
	
	return le_z;
}

//Right arm end effector position projection ---------------------------------------
void calc_le_camera_projection(vector<double> mu, vector<double> a)
{
	//calculate position projection vector
	calc_3d_rototranslational_matrix(mu, a);
	ra_epv = ra_rtm * ra_repv;
	
	//calculate left eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
	calc_le_rototranslational_matrix(dof_le, dof_le);
	
	//calculate the inverse of the left eye rototranslational matrix
	le_rtm_inv = le_rtm.inverse();
	
	//projection of the right arm end effector to the camera reference frame
	cam_projection = le_rtm_inv * ra_epv;
	
}

void calc_re_camera_projection(vector<double> mu, vector<double> a)
{
    //calculate position projection vector
    calc_3d_rototranslational_matrix(mu, a);
    ra_epv = ra_rtm * ra_repv;

    //calculate right eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_re_rototranslational_matrix(dof_le, dof_le);

    //calculate the inverse of the left eye rototranslational matrix
    re_rtm_inv = re_rtm.inverse();

    //projection of the right arm end effector to the camera reference frame
    cam_projection_R = re_rtm_inv * ra_epv;

}

//Relative world position of camera centerpoint ------------------------------------
void calc_le_camera_origin(vector<double> mu, vector<double> a)
{
	//calculate left eye rototranslational matrix
    //vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix_h(mu, mu);
	
	//calculate relative world projection of camera centerpoint
    r_le_projection = le_rtm_h * le_projection;
	
}

//Left arm end effector position projection ---------------------------------------
void calc_le_camera_projection_b(vector<double> mu, vector<double> a)
{
    //calculate position projection vector
    calc_3d_rototranslational_matrix_b(mu, a);
    la_epv = la_rtm * la_repv;

    //calculate left eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix_b(dof_le, dof_le);

    //calculate the inverse of the left eye rototranslational matrix
    le_rtm_inv_b = le_rtm_b.inverse();

    //projection of the right arm end effector to the camera reference frame
    cam_projection_b = le_rtm_inv_b * la_epv;

}

//End effector position projection derivative ----------------------------
//Right arm
void calc_d_le_camera_projection_mu1(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_d_3d_rototranslational_matrix_mu1(mu, a);
	ra_d1epv = ra_d1rtm * ra_repv;
	
	//derivative of the projection
	d1_cam_projection = le_rtm_inv * ra_d1epv;

}

void calc_d_le_camera_projection_mu2(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_d_3d_rototranslational_matrix_mu2(mu, a);
	ra_d2epv = ra_d2rtm * ra_repv;
	
	//derivative of the projection
	d2_cam_projection = le_rtm_inv * ra_d2epv;
	
}

void calc_d_le_camera_projection_mu3(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_d_3d_rototranslational_matrix_mu3(mu, a);
	ra_d3epv = ra_d3rtm * ra_repv;
	
	//derivative of the projection
	d3_cam_projection = le_rtm_inv * ra_d3epv;
	
}

//Left arm
void calc_d_le_camera_projection_mu1_b(vector<double> mu, vector<double> a)
{
    //calculate position projection derivative vector
    calc_le_camera_projection_b(mu, a);
    calc_d_3d_rototranslational_matrix_mu1b(mu, a);
    la_d1epv = la_d1rtm * la_repv;

    //derivative of the projection
    d1_cam_projection_b = le_rtm_inv_b * la_d1epv;

}

void calc_d_le_camera_projection_mu2_b(vector<double> mu, vector<double> a)
{
    //calculate position projection derivative vector
    calc_le_camera_projection_b(mu, a);
    calc_d_3d_rototranslational_matrix_mu2b(mu, a);
    la_d2epv = la_d2rtm * la_repv;

    //derivative of the projection
    d2_cam_projection_b = le_rtm_inv_b * la_d2epv;

}

void calc_d_le_camera_projection_mu3_b(vector<double> mu, vector<double> a)
{
    //calculate position projection derivative vector
    calc_le_camera_projection_b(mu, a);
    calc_d_3d_rototranslational_matrix_mu3b(mu, a);
    la_d3epv = la_d3rtm * la_repv;

    //derivative of the projection
    d3_cam_projection_b = le_rtm_inv_b * la_d3epv;

}

void calc_d_le_camera_projection_mu11(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_dd_3d_rototranslational_matrix_mu11(mu, a);
	ra_d11epv = ra_d11rtm * ra_repv;
	
	//derivative of the projection
	d11_cam_projection = le_rtm_inv * ra_d11epv;
	
}

void calc_d_le_camera_projection_mu22(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_dd_3d_rototranslational_matrix_mu22(mu, a);
	ra_d22epv = ra_d22rtm * ra_repv;
	
	//derivative of the projection
	d22_cam_projection = le_rtm_inv * ra_d22epv;
	
}

void calc_d_le_camera_projection_mu33(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_dd_3d_rototranslational_matrix_mu33(mu, a);
	ra_d33epv = ra_d33rtm * ra_repv;
	
	//derivative of the projection
	d33_cam_projection = le_rtm_inv * ra_d33epv;
	
}

void calc_d_le_camera_projection_mu12(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_dd_3d_rototranslational_matrix_mu12(mu, a);
	ra_d12epv = ra_d12rtm * ra_repv;
	
	//derivative of the projection
	d12_cam_projection = le_rtm_inv * ra_d12epv;
	
}

void calc_d_le_camera_projection_mu13(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_dd_3d_rototranslational_matrix_mu13(mu, a);
	ra_d13epv = ra_d13rtm * ra_repv;
	
	//derivative of the projection
	d13_cam_projection = le_rtm_inv * ra_d13epv;
	
}

void calc_d_le_camera_projection_mu23(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_projection(mu, a);
	calc_dd_3d_rototranslational_matrix_mu23(mu, a);
	ra_d23epv = ra_d23rtm * ra_repv;
	
	//derivative of the projection
	d23_cam_projection = le_rtm_inv * ra_d23epv;
	
}

//Head camera centerpoint projection derivative ----------------------------
void calc_d_le_camera_origin_mu1(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_origin(mu, a);
	calc_le_rototranslational_matrix_inv_mu1e(mu, a);

	//derivative of the centerpoint projection
	d1_le_projection = d1_inv_le_rtm * r_le_projection;

}

void calc_d_le_camera_origin_mu2(vector<double> mu, vector<double> a)
{
	//calculate position projection derivative vector
	calc_le_camera_origin(mu, a);
	calc_le_rototranslational_matrix_inv_mu2e(mu, a);

	//derivative of the centerpoint projection
	d2_le_projection = d2_inv_le_rtm * r_le_projection;

}

void calc_d_le_camera_origin_mu3(vector<double> mu, vector<double> a)
{
    //calculate position projection derivative vector
    calc_le_camera_origin(mu, a);
    calc_le_rototranslational_matrix_inv_mu3e(mu, a);

    //derivative of the centerpoint projection
    d3_le_projection = d3_inv_le_rtm * r_le_projection;

}

//Camera parameters ----------------------------------------------------------------
//Right arm
void calc_camera_parameters(Vector4d projection)
{
	//cam_projection OR le_projection

	double xcam, ycam, zcam;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	
	xp = xcam/zcam;
	yp = ycam/zcam;
	r2 = xp*xp + yp*yp;
	
	ck = (1 + k1*r2 + k2*pow(r2,2) + k3*pow(r2,3))/
		  (1 + k4*r2 + k5*pow(r2,2) + k6*pow(r2,3));
	
	px = 2*p1*xp*yp;
	pxx = p2*(r2 + 2*pow(xp,2));
	py = 2*p2*xp*yp;
	pyy = p1*(r2 + 2*pow(yp,2));
}

void calc_camera_parameters_L(Vector4d projection)
{

    double xcam, ycam, zcam;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);

    xp_L = xcam/zcam;
    yp_L = ycam/zcam;
    r2_L = xp_L*xp_L + yp_L*yp_L;

    ck_L = (1 + k1_L*r2_L + k2_L*pow(r2_L,2) + k3_L*pow(r2_L,3))/
          (1 + k4_L*r2_L + k5_L*pow(r2_L,2) + k6_L*pow(r2_L,3));

    px_L = 2*p1_L*xp_L*yp_L;
    pxx_L = p2_L*(r2_L + 2*pow(xp_L,2));
    py_L = 2*p2_L*xp_L*yp_L;
    pyy_L = p1_L*(r2_L + 2*pow(yp_L,2));
}

void calc_camera_parameters_R(Vector4d projection)
{

    double xcam, ycam, zcam;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);

    xp_R = xcam/zcam;
    yp_R = ycam/zcam;
    r2_R = xp_R*xp_R + yp_R*yp_R;

    ck_R = (1 + k1_R*r2_R + k2_R*pow(r2_R,2) + k3_R*pow(r2_R,3))/
          (1 + k4_R*r2_R + k5_R*pow(r2_R,2) + k6_R*pow(r2_R,3));

    px_R = 2*p1_R*xp_R*yp_R;
    pxx_R = p2_R*(r2_R + 2*pow(xp_R,2));
    py_R = 2*p2_R*xp_R*yp_R;
    pyy_R = p1_R*(r2_R + 2*pow(yp_R,2));
}

//Head
void calc_camera_parameters_h(Vector4d projection)
{
    //le_projection

    double xcam, ycam, zcam;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);

    xp_h = xcam/zcam;
    yp_h = ycam/zcam;
    r2_h = xp_h*xp_h + yp_h*yp_h;

    ck_h = (1 + k1*r2_h + k2*pow(r2_h,2) + k3*pow(r2_h,3))/
          (1 + k4*r2_h + k5*pow(r2_h,2) + k6*pow(r2_h,3));

    px_h = 2*p1*xp_h*yp_h;
    pxx_h = p2*(r2_h + 2*pow(xp_h,2));
    py_h = 2*p2*xp_h*yp_h;
    pyy_h = p1*(r2_h + 2*pow(yp_h,2));
}

//Left arm
void calc_camera_parameters_b(Vector4d projection)
{
    //cam_projection_b

    double xcam, ycam, zcam;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);

    xp_b = xcam/zcam;
    yp_b = ycam/zcam;
    r2_b = xp_b*xp_b + yp_b*yp_b;

    ck_b = (1 + k1*r2_b + k2*pow(r2_b,2) + k3*pow(r2_b,3))/
          (1 + k4*r2_b + k5*pow(r2_b,2) + k6*pow(r2_b,3));

    px_b = 2*p1*xp_b*yp_b;
    pxx_b = p2*(r2_b + 2*pow(xp_b,2));
    py_b = 2*p2*xp_b*yp_b;
    pyy_b = p1*(r2_b + 2*pow(yp_b,2));
}

//Right arm
void calc_d_camera_parameters_mu1(Vector4d projection, Vector4d d1_projection)
{
    //cam_projection, d1_cam_projection

	double xcam, ycam, zcam;
	double d1xcam, d1ycam, d1zcam;
	double dr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d1xcam = d1_projection(0);
	d1ycam = d1_projection(1);
	d1zcam = d1_projection(2);
	
	d1xp = (d1xcam*zcam - xcam*d1zcam) / pow(zcam, 2);
	d1yp = (d1ycam*zcam - ycam*d1zcam) / pow(zcam, 2);
	
	d1r2 = 2*(xp*d1xp + yp*d1yp);

	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	d1k = dr2k*d1r2;
	
	d1px = 2*p1*(d1xp*yp + xp*d1yp);
	d1pxx = 2*p2*(3*xp*d1xp + yp*d1yp);
	d1py = 2*p2*(d1xp*yp + xp*d1yp);
	d1pyy = 2*p1*(xp*d1xp + 3*yp*d1yp);
}

//Head
void calc_d_camera_parameters_mu1_h(Vector4d projection, Vector4d d1_projection)
{
    //le_projection, d1_le_projection

    double xcam, ycam, zcam;
    double d1xcam, d1ycam, d1zcam;
    double dr2k;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);
    d1xcam = d1_projection(0);
    d1ycam = d1_projection(1);
    d1zcam = d1_projection(2);

    d1xp_h = (d1xcam*zcam - xcam*d1zcam) / pow(zcam, 2);
    d1yp_h = (d1ycam*zcam - ycam*d1zcam) / pow(zcam, 2);

    d1r2_h = 2*(xp_h*d1xp_h + yp_h*d1yp_h);

    dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) +
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
        Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);

    d1k_h = dr2k*d1r2_h;

    d1px_h = 2*p1*(d1xp_h*yp_h + xp_h*d1yp_h);
    d1pxx_h = 2*p2*(3*xp_h*d1xp_h + yp_h*d1yp_h);
    d1py_h = 2*p2*(d1xp_h*yp_h + xp_h*d1yp_h);
    d1pyy_h = 2*p1*(xp_h*d1xp_h + 3*yp_h*d1yp_h);
}

//Left arm
void calc_d_camera_parameters_mu1_b(Vector4d projection, Vector4d d1_projection)
{
    //cam_projection_b, d1_cam_projection_b

    double xcam, ycam, zcam;
    double d1xcam, d1ycam, d1zcam;
    double dr2k;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);
    d1xcam = d1_projection(0);
    d1ycam = d1_projection(1);
    d1zcam = d1_projection(2);

    d1xp_b = (d1xcam*zcam - xcam*d1zcam) / pow(zcam, 2);
    d1yp_b = (d1ycam*zcam - ycam*d1zcam) / pow(zcam, 2);

    d1r2_b = 2*(xp_b*d1xp_b + yp_b*d1yp_b);

    dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) +
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
        Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);

    d1k_b = dr2k*d1r2_b;

    d1px_b = 2*p1*(d1xp_b*yp_b + xp_b*d1yp_b);
    d1pxx_b = 2*p2*(3*xp_b*d1xp_b + yp_b*d1yp_b);
    d1py_b = 2*p2*(d1xp_b*yp_b + xp_b*d1yp_b);
    d1pyy_b = 2*p1*(xp_b*d1xp_b + 3*yp_b*d1yp_b);
}

//Right arm
void calc_d_camera_parameters_mu2(Vector4d projection, Vector4d d2_projection)
{
    //cam_projection, d2_cam_projection OR le_projection, d2_le_projection

	double xcam, ycam, zcam;
	double d2xcam, d2ycam, d2zcam;
	double dr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d2xcam = d2_projection(0);
	d2ycam = d2_projection(1);
	d2zcam = d2_projection(2);
	
	d2xp = (d2xcam*zcam - xcam*d2zcam) / pow(zcam, 2);
	d2yp = (d2ycam*zcam - ycam*d2zcam) / pow(zcam, 2);
	
	d2r2 = 2*(xp*d2xp + yp*d2yp);

	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	d2k = dr2k*d2r2;
	
	d2px = 2*p1*(d2xp*yp + xp*d2yp);
	d2pxx = 2*p2*(3*xp*d2xp + yp*d2yp);
	d2py = 2*p2*(d2xp*yp + xp*d2yp);
	d2pyy = 2*p1*(xp*d2xp + 3*yp*d2yp);
}

//Head
void calc_d_camera_parameters_mu2_h(Vector4d projection, Vector4d d2_projection)
{
    //le_projection, d2_le_projection

    double xcam, ycam, zcam;
    double d2xcam, d2ycam, d2zcam;
    double dr2k;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);
    d2xcam = d2_projection(0);
    d2ycam = d2_projection(1);
    d2zcam = d2_projection(2);

    d2xp_h = (d2xcam*zcam - xcam*d2zcam) / pow(zcam, 2);
    d2yp_h = (d2ycam*zcam - ycam*d2zcam) / pow(zcam, 2);

    d2r2_h = 2*(xp_h*d2xp_h + yp_h*d2yp_h);

    dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) +
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
        Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);

    d2k_h = dr2k*d2r2_h;

    d2px_h = 2*p1*(d2xp_h*yp_h + xp_h*d2yp_h);
    d2pxx_h = 2*p2*(3*xp_h*d2xp_h + yp_h*d2yp_h);
    d2py_h = 2*p2*(d2xp_h*yp_h + xp_h*d2yp_h);
    d2pyy_h = 2*p1*(xp_h*d2xp_h + 3*yp_h*d2yp_h);
}

//Left arm
void calc_d_camera_parameters_mu2_b(Vector4d projection, Vector4d d2_projection)
{
    //cam_projection_b, d2_cam_projection_b

    double xcam, ycam, zcam;
    double d2xcam, d2ycam, d2zcam;
    double dr2k;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);
    d2xcam = d2_projection(0);
    d2ycam = d2_projection(1);
    d2zcam = d2_projection(2);

    d2xp_b = (d2xcam*zcam - xcam*d2zcam) / pow(zcam, 2);
    d2yp_b = (d2ycam*zcam - ycam*d2zcam) / pow(zcam, 2);

    d2r2_b = 2*(xp_b*d2xp_b + yp_b*d2yp_b);

    dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) +
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
        Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);

    d2k_b = dr2k*d2r2_b;

    d2px_b = 2*p1*(d2xp_b*yp_b + xp_b*d2yp_b);
    d2pxx_b = 2*p2*(3*xp_b*d2xp_b + yp_b*d2yp_b);
    d2py_b = 2*p2*(d2xp_b*yp_b + xp_b*d2yp_b);
    d2pyy_b = 2*p1*(xp_b*d2xp_b + 3*yp_b*d2yp_b);
}

//Right arm
void calc_d_camera_parameters_mu3(Vector4d projection, Vector4d d3_projection)
{
	//cam_projection, d3_cam_projection OR le_projection, d3_le_projection

	double xcam, ycam, zcam;
	double d3xcam, d3ycam, d3zcam;
	double dr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d3xcam = d3_projection(0);
	d3ycam = d3_projection(1);
	d3zcam = d3_projection(2);
	
	d3xp = (d3xcam*zcam - xcam*d3zcam) / pow(zcam, 2);
	d3yp = (d3ycam*zcam - ycam*d3zcam) / pow(zcam, 2);
	
	d3r2 = 2*(xp*d3xp + yp*d3yp);

	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	d3k = dr2k*d3r2;
	
	d3px = 2*p1*(d3xp*yp + xp*d3yp);
	d3pxx = 2*p2*(3*xp*d3xp + yp*d3yp);
	d3py = 2*p2*(d3xp*yp + xp*d3yp);
	d3pyy = 2*p1*(xp*d3xp + 3*yp*d3yp);
}

//Head
void calc_d_camera_parameters_mu3_h(Vector4d projection, Vector4d d3_projection)
{
    //le_projection, d3_le_projection

    double xcam, ycam, zcam;
    double d3xcam, d3ycam, d3zcam;
    double dr2k;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);
    d3xcam = d3_projection(0);
    d3ycam = d3_projection(1);
    d3zcam = d3_projection(2);

    d3xp_h = (d3xcam*zcam - xcam*d3zcam) / pow(zcam, 2);
    d3yp_h = (d3ycam*zcam - ycam*d3zcam) / pow(zcam, 2);

    d3r2_h = 2*(xp_h*d3xp_h + yp_h*d3yp_h);

    dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) +
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
        Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);

    d3k_h = dr2k*d3r2_h;

    d3px_h = 2*p1*(d3xp_h*yp_h + xp_h*d3yp_h);
    d3pxx_h = 2*p2*(3*xp_h*d3xp_h + yp_h*d3yp_h);
    d3py_h = 2*p2*(d3xp_h*yp_h + xp_h*d3yp_h);
    d3pyy_h = 2*p1*(xp_h*d3xp_h + 3*yp_h*d3yp_h);
}

//Left arm
void calc_d_camera_parameters_mu3_b(Vector4d projection, Vector4d d3_projection)
{
    //cam_projection_b, d3_cam_projection_b

    double xcam, ycam, zcam;
    double d3xcam, d3ycam, d3zcam;
    double dr2k;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);
    d3xcam = d3_projection(0);
    d3ycam = d3_projection(1);
    d3zcam = d3_projection(2);

    d3xp_b = (d3xcam*zcam - xcam*d3zcam) / pow(zcam, 2);
    d3yp_b = (d3ycam*zcam - ycam*d3zcam) / pow(zcam, 2);

    d3r2_b = 2*(xp_b*d3xp_b + yp_b*d3yp_b);

    dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) +
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
        Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);

    d3k_b = dr2k*d3r2_b;

    d3px_b = 2*p1*(d3xp_b*yp_b + xp_b*d3yp_b);
    d3pxx_b = 2*p2*(3*xp_b*d3xp_b + yp_b*d3yp_b);
    d3py_b = 2*p2*(d3xp_b*yp_b + xp_b*d3yp_b);
    d3pyy_b = 2*p1*(xp_b*d3xp_b + 3*yp_b*d3yp_b);
}

//Higher order (Hessian)
void calc_d_camera_parameters_mu11(Vector4d projection, Vector4d d1_projection, Vector4d d11_projection)
{
	//cam_projection, d1_cam_projection, d11_cam_projection OR le_projection, d1_le_projection, d11_le_projection

	double xcam, ycam, zcam;
	double d1xcam, d1ycam, d1zcam;
	double d11xcam, d11ycam, d11zcam;
	double dr2k, ddr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d1xcam = d1_projection(0);
	d1ycam = d1_projection(1);
	d1zcam = d1_projection(2);
	d11xcam = d11_projection(0);
	d11ycam = d11_projection(1);
	d11zcam = d11_projection(2);
	
	d11xp = (2*xcam*pow(d1zcam,2) + pow(zcam,2)*d11xcam - zcam*(2*d1xcam*d1zcam + xcam*d11zcam))*(1/pow(zcam,3));
	d11yp = (2*ycam*pow(d1zcam,2) + pow(zcam,2)*d11ycam - zcam*(2*d1ycam*d1zcam + ycam*d11zcam))*(1/pow(zcam,3));
	
	d11r2 = 2*(pow(d1xp,2) + pow(d1yp,2) + xp*d11xp + yp*d11yp);
	
	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	ddr2k = (-2*(k1 + r2*(2*k2 + 3*k3*r2))*(k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))) + 
     2*(k2 + 3*k3*r2)*Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2) + 
     (1 + r2*(k1 + r2*(k2 + k3*r2)))*(2*Power(k4 + r2*(2*k5 + 3*k6*r2),2) - 
        2*(k5 + 3*k6*r2)*(1 + r2*(k4 + r2*(k5 + k6*r2)))))/
   Power(1 + r2*(k4 + r2*(k5 + k6*r2)),3);
	
	d11k = ddr2k * pow(d1r2,2) + dr2k * d11r2;
	
	d11px = 2*p1*(d11xp*yp + 2*d1xp*d1yp + xp*d11yp);
	d11pxx = 2*p2*(3*pow(d1xp,2) + 3*xp*d11xp + pow(d1yp,2) + yp*d11yp);
	d11py = 2*p2*(d11xp*yp + 2*d1xp*d1yp + xp*d11yp);
	d11pyy = 2*p1*(pow(d1xp,2) + xp*d11xp + 3*pow(d1yp,2) + 3*yp*d11yp);
}

void calc_d_camera_parameters_mu22(Vector4d projection, Vector4d d2_projection, Vector4d d22_projection)
{
	//cam_projection, d2_cam_projection, d22_cam_projection OR le_projection, d2_le_projection, d22_le_projection

	double xcam, ycam, zcam;
	double d2xcam, d2ycam, d2zcam;
	double d22xcam, d22ycam, d22zcam;
	double dr2k, ddr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d2xcam = d2_projection(0);
	d2ycam = d2_projection(1);
	d2zcam = d2_projection(2);
	d22xcam = d22_projection(0);
	d22ycam = d22_projection(1);
	d22zcam = d22_projection(2);
	
	d22xp = (2*xcam*pow(d2zcam,2) + pow(zcam,2)*d22xcam - zcam*(2*d2xcam*d2zcam + xcam*d22zcam))*(1/pow(zcam,3));
	d22yp = (2*ycam*pow(d2zcam,2) + pow(zcam,2)*d22ycam - zcam*(2*d2ycam*d2zcam + ycam*d22zcam))*(1/pow(zcam,3));
	
	d22r2 = 2*(pow(d2xp,2) + pow(d2yp,2) + xp*d22xp + yp*d22yp);
	
	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	ddr2k = (-2*(k1 + r2*(2*k2 + 3*k3*r2))*(k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))) + 
     2*(k2 + 3*k3*r2)*Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2) + 
     (1 + r2*(k1 + r2*(k2 + k3*r2)))*(2*Power(k4 + r2*(2*k5 + 3*k6*r2),2) - 
        2*(k5 + 3*k6*r2)*(1 + r2*(k4 + r2*(k5 + k6*r2)))))/
   Power(1 + r2*(k4 + r2*(k5 + k6*r2)),3);
	
	d22k = ddr2k * pow(d2r2,2) + dr2k * d22r2;
	
	d22px = 2*p1*(d22xp*yp + 2*d2xp*d2yp + xp*d22yp);
	d22pxx = 2*p2*(3*pow(d2xp,2) + 3*xp*d22xp + pow(d2yp,2) + yp*d22yp);
	d22py = 2*p2*(d22xp*yp + 2*d2xp*d2yp + xp*d22yp);
	d22pyy = 2*p1*(pow(d2xp,2) + xp*d22xp + 3*pow(d2yp,2) + 3*yp*d22yp);	
}

void calc_d_camera_parameters_mu33(Vector4d projection, Vector4d d3_projection, Vector4d d33_projection)
{
	//cam_projection, d3_cam_projection, d33_cam_projection OR le_projection, d3_le_projection, d33_le_projection

	double xcam, ycam, zcam;
	double d3xcam, d3ycam, d3zcam;
	double d33xcam, d33ycam, d33zcam;
	double dr2k, ddr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d3xcam = d3_projection(0);
	d3ycam = d3_projection(1);
	d3zcam = d3_projection(2);
	d33xcam = d33_projection(0);
	d33ycam = d33_projection(1);
	d33zcam = d33_projection(2);
	
	d33xp = (2*xcam*pow(d3zcam,2) + pow(zcam,2)*d33xcam - zcam*(2*d3xcam*d3zcam + xcam*d33zcam))*(1/pow(zcam,3));
	d33yp = (2*ycam*pow(d3zcam,2) + pow(zcam,2)*d33ycam - zcam*(2*d3ycam*d3zcam + ycam*d33zcam))*(1/pow(zcam,3));
	
	d33r2 = 2*(pow(d3xp,2) + pow(d3yp,2) + xp*d33xp + yp*d33yp);
	
	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	ddr2k = (-2*(k1 + r2*(2*k2 + 3*k3*r2))*(k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))) + 
     2*(k2 + 3*k3*r2)*Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2) + 
     (1 + r2*(k1 + r2*(k2 + k3*r2)))*(2*Power(k4 + r2*(2*k5 + 3*k6*r2),2) - 
        2*(k5 + 3*k6*r2)*(1 + r2*(k4 + r2*(k5 + k6*r2)))))/
   Power(1 + r2*(k4 + r2*(k5 + k6*r2)),3);
	
	d33k = ddr2k * pow(d3r2,2) + dr2k * d33r2;
	
	d33px = 2*p1*(d33xp*yp + 2*d3xp*d3yp + xp*d33yp);
	d33pxx = 2*p2*(3*pow(d3xp,2) + 3*xp*d33xp + pow(d3yp,2) + yp*d33yp);
	d33py = 2*p2*(d33xp*yp + 2*d3xp*d3yp + xp*d33yp);
	d33pyy = 2*p1*(pow(d3xp,2) + xp*d33xp + 3*pow(d3yp,2) + 3*yp*d33yp);	
}

void calc_d_camera_parameters_mu12(Vector4d projection, Vector4d d1_projection, Vector4d d2_projection, Vector4d d12_projection)
{
	//cam_projection, d1_cam_projection, d2_cam_projection, d12_cam_projection OR
	//le_projection, d1_le_projection, d2_le_projection, d12_le_projection

	double xcam, ycam, zcam;
	double d1xcam, d1ycam, d1zcam;
	double d2xcam, d2ycam, d2zcam;
	double d12xcam, d12ycam, d12zcam;
	double dr2k, ddr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d1xcam = d1_projection(0);
	d1ycam = d1_projection(1);
	d1zcam = d1_projection(2);
	d2xcam = d2_projection(0);
	d2ycam = d2_projection(1);
	d2zcam = d2_projection(2);
	d12xcam = d12_projection(0);
	d12ycam = d12_projection(1);
	d12zcam = d12_projection(2);
	
	d12xp = (2*xcam*d1zcam*d2zcam + pow(zcam,2)*d12xcam - zcam*(d1xcam*d2zcam + d2xcam*d1zcam + xcam*d12zcam))*(1/pow(zcam,3));
	d12yp = (2*ycam*d1zcam*d2zcam + pow(zcam,2)*d12ycam - zcam*(d1ycam*d2zcam + d2ycam*d1zcam + ycam*d12zcam))*(1/pow(zcam,3));
	
	d12r2 = 2*(d1xp*d2xp + d1yp*d2yp + xp*d12xp + yp*d12yp);
	
	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	ddr2k = (-2*(k1 + r2*(2*k2 + 3*k3*r2))*(k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))) + 
     2*(k2 + 3*k3*r2)*Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2) + 
     (1 + r2*(k1 + r2*(k2 + k3*r2)))*(2*Power(k4 + r2*(2*k5 + 3*k6*r2),2) - 
        2*(k5 + 3*k6*r2)*(1 + r2*(k4 + r2*(k5 + k6*r2)))))/
   Power(1 + r2*(k4 + r2*(k5 + k6*r2)),3);
	
	d12k = ddr2k * d1r2 * d2r2 + dr2k * d12r2;
	
	d12px = 2*p1*(d12xp*yp + d1xp*d2yp + d2xp*d1yp + xp*d12yp);
	d12pxx = 2*p2*(3*d1xp*d2xp + 3*xp*d12xp + d1yp*d2yp + yp*d12yp);
	d12py = 2*p2*(d12xp*yp + d1xp*d2yp + d2xp*d1yp + xp*d12yp);
	d12pyy = 2*p1*(d1xp*d2xp + xp*d12xp + 3*d1yp*d2yp + 3*yp*d12yp);	
}

void calc_d_camera_parameters_mu13(Vector4d projection, Vector4d d1_projection, Vector4d d3_projection, Vector4d d13_projection)
{
	//cam_projection, d1_cam_projection, d3_cam_projection, d13_cam_projection OR
	//le_projection, d1_le_projection, d3_le_projection, d13_le_projection

	double xcam, ycam, zcam;
	double d1xcam, d1ycam, d1zcam;
	double d3xcam, d3ycam, d3zcam;
	double d13xcam, d13ycam, d13zcam;
	double dr2k, ddr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d1xcam = d1_projection(0);
	d1ycam = d1_projection(1);
	d1zcam = d1_projection(2);
	d3xcam = d3_projection(0);
	d3ycam = d3_projection(1);
	d3zcam = d3_projection(2);
	d13xcam = d13_projection(0);
	d13ycam = d13_projection(1);
	d13zcam = d13_projection(2);
	
	d13xp = (2*xcam*d1zcam*d3zcam + pow(zcam,2)*d13xcam - zcam*(d1xcam*d3zcam + d3xcam*d1zcam + xcam*d13zcam))*(1/pow(zcam,3));
	d13yp = (2*ycam*d1zcam*d3zcam + pow(zcam,2)*d13ycam - zcam*(d1ycam*d3zcam + d3ycam*d1zcam + ycam*d13zcam))*(1/pow(zcam,3));
	
	d13r2 = 2*(d1xp*d3xp + d1yp*d3yp + xp*d13xp + yp*d13yp);
	
	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	ddr2k = (-2*(k1 + r2*(2*k2 + 3*k3*r2))*(k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))) + 
     2*(k2 + 3*k3*r2)*Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2) + 
     (1 + r2*(k1 + r2*(k2 + k3*r2)))*(2*Power(k4 + r2*(2*k5 + 3*k6*r2),2) - 
        2*(k5 + 3*k6*r2)*(1 + r2*(k4 + r2*(k5 + k6*r2)))))/
   Power(1 + r2*(k4 + r2*(k5 + k6*r2)),3);
	
	d13k = ddr2k * d1r2 * d3r2 + dr2k * d13r2;
	
	d13px = 2*p1*(d13xp*yp + d1xp*d3yp + d3xp*d1yp + xp*d13yp);
	d13pxx = 2*p2*(3*d1xp*d3xp + 3*xp*d13xp + d1yp*d3yp + yp*d13yp);
	d13py = 2*p2*(d13xp*yp + d1xp*d3yp + d3xp*d1yp + xp*d13yp);
	d13pyy = 2*p1*(d1xp*d3xp + xp*d13xp + 3*d1yp*d3yp + 3*yp*d13yp);	
}

void calc_d_camera_parameters_mu23(Vector4d projection, Vector4d d2_projection, Vector4d d3_projection, Vector4d d23_projection)
{
	//cam_projection, d2_cam_projection, d3_cam_projection, d23_cam_projection OR
	//le_projection, d2_le_projection, d3_le_projection, d23_le_projection

	double xcam, ycam, zcam;
	double d2xcam, d2ycam, d2zcam;
	double d3xcam, d3ycam, d3zcam;
	double d23xcam, d23ycam, d23zcam;
	double dr2k, ddr2k;
	
	xcam = projection(0);
	ycam = projection(1);
	zcam = projection(2);
	d2xcam = d2_projection(0);
	d2ycam = d2_projection(1);
	d2zcam = d2_projection(2);
	d3xcam = d3_projection(0);
	d3ycam = d3_projection(1);
	d3zcam = d3_projection(2);
	d23xcam = d23_projection(0);
	d23ycam = d23_projection(1);
	d23zcam = d23_projection(2);
	
	d23xp = (2*xcam*d2zcam*d3zcam + pow(zcam,2)*d23xcam - zcam*(d2xcam*d3zcam + d3xcam*d2zcam + xcam*d23zcam))*(1/pow(zcam,3));
	d23yp = (2*ycam*d2zcam*d3zcam + pow(zcam,2)*d23ycam - zcam*(d2ycam*d3zcam + d3ycam*d2zcam + ycam*d23zcam))*(1/pow(zcam,3));
	
	d23r2 = 2*(d2xp*d3xp + d2yp*d3yp + xp*d23xp + yp*d23yp);
	
	dr2k = (-((k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k1 + r2*(k2 + k3*r2)))) + 
     (k1 + r2*(2*k2 + 3*k3*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))))/
   		Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2);
	
	ddr2k = (-2*(k1 + r2*(2*k2 + 3*k3*r2))*(k4 + r2*(2*k5 + 3*k6*r2))*(1 + r2*(k4 + r2*(k5 + k6*r2))) + 
     2*(k2 + 3*k3*r2)*Power(1 + r2*(k4 + r2*(k5 + k6*r2)),2) + 
     (1 + r2*(k1 + r2*(k2 + k3*r2)))*(2*Power(k4 + r2*(2*k5 + 3*k6*r2),2) - 
        2*(k5 + 3*k6*r2)*(1 + r2*(k4 + r2*(k5 + k6*r2)))))/
   Power(1 + r2*(k4 + r2*(k5 + k6*r2)),3);
	
	d23k = ddr2k * d2r2 * d3r2 + dr2k * d23r2;
	
	d23px = 2*p1*(d23xp*yp + d2xp*d3yp + d3xp*d2yp + xp*d23yp);
	d23pxx = 2*p2*(3*d2xp*d3xp + 3*xp*d23xp + d2yp*d3yp + yp*d23yp);
	d23py = 2*p2*(d23xp*yp + d2xp*d3yp + d3xp*d2yp + xp*d23yp);
	d23pyy = 2*p1*(d2xp*d3xp + xp*d23xp + 3*d2yp*d3yp + 3*yp*d23yp);	
}

//Right arm
//Left eye projection
double sense_vision_l_u(vector<double> mu, vector<double> a)
{
    //read by vision thread
	
	if (publish){
        Bottle& output_l_u = plot_l_u.prepare();
        output_l_u.clear();
        output_l_u.addDouble(re_pos[3]);
        plot_l_u.write();
	}

    return re_pos[3];
}

double sense_vision_l_v(vector<double> mu, vector<double> a)
{
	if (publish){
        Bottle& output_l_v = plot_l_v.prepare();
        output_l_v.clear();
        output_l_v.addDouble(re_pos[4]);
        plot_l_v.write();
	}

    return re_pos[4];
}

double calc_vision_l_u(vector<double> mu, vector<double> a)
{
    calc_le_camera_projection(mu, a);
    calc_camera_parameters_L(cam_projection);
	
    double vision_u = fx_L*(xp_L*ck_L + px_L + pxx_L) + cx_L;
	
	if (publish){
        Bottle& output_calc_l_u = plot_calc_l_u.prepare();
        output_calc_l_u.clear();
        output_calc_l_u.addDouble(vision_u);
        plot_calc_l_u.write();
	}

    calc_vision_pos_l(0) = vision_u;

	return vision_u;
}

double calc_vision_l_v(vector<double> mu, vector<double> a)
{
	//calc_le_camera_projection(mu, a);
	//calc_camera_parameters(cam_projection);

    double vision_v = fy_L*(yp_L*ck_L + py_L + pyy_L) + cy_L;
	
	if (publish){
        Bottle& output_calc_l_v = plot_calc_l_v.prepare();
        output_calc_l_v.clear();
        output_calc_l_v.addDouble(vision_v);
        plot_calc_l_v.write();
	}

    calc_vision_pos_l(1) = vision_v;

	return vision_v;
}

//Right eye projection
double sense_vision_r_u(vector<double> mu, vector<double> a)
{
    //read by vision thread

    if (publish){
        Bottle& output_r_u = plot_r_u.prepare();
        output_r_u.clear();
        output_r_u.addDouble(re_pos[6]);
        plot_r_u.write();
    }

    return re_pos[6];
}

double sense_vision_r_v(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_r_v = plot_r_v.prepare();
        output_r_v.clear();
        output_r_v.addDouble(re_pos[7]);
        plot_r_v.write();
    }

    return re_pos[7];
}

double calc_vision_r_u(vector<double> mu, vector<double> a)
{
    calc_re_camera_projection(mu, a);
    calc_camera_parameters_R(cam_projection_R);

    double vision_u = fx_R*(xp_R*ck_R + px_R + pxx_R) + cx_R;

    if (publish){
        Bottle& output_calc_r_u = plot_calc_r_u.prepare();
        output_calc_r_u.clear();
        output_calc_r_u.addDouble(vision_u);
        plot_calc_r_u.write();
    }

    calc_vision_pos_r(0) = vision_u;

    return vision_u;
}

double calc_vision_r_v(vector<double> mu, vector<double> a)
{
    //calc_le_camera_projection(mu, a);
    //calc_camera_parameters(cam_projection);

    double vision_v = fy_R*(yp_R*ck_R + py_R + pyy_R) + cy_R;

    if (publish){
        Bottle& output_calc_r_v = plot_calc_r_v.prepare();
        output_calc_r_v.clear();
        output_calc_r_v.addDouble(vision_v);
        plot_calc_r_v.write();
    }

    calc_vision_pos_r(1) = vision_v;

    return vision_v;
}

//Attractor
void calc_le_attractor_projection()
{
    //calculate position projection vector
    Vector4d attr_pos_3d;
    attr_pos_3d(0) = attr_3d_pos(0)*k;
    attr_pos_3d(1) = attr_3d_pos(1)*k;
    attr_pos_3d(2) = attr_3d_pos(2)*k;
    attr_pos_3d(3) = 1;

    //calculate left eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_le_rototranslational_matrix(dof_le, dof_le);

    //calculate the inverse of the left eye rototranslational matrix
    le_rtm_inv = le_rtm.inverse();

    //projection of the right arm end effector to the camera reference frame
    attr_projection_L = le_rtm_inv * attr_pos_3d;

}

void calc_re_attractor_projection()
{
    //calculate position projection vector
    Vector4d attr_pos_3d;
    attr_pos_3d(0) = attr_3d_pos(0)*k;
    attr_pos_3d(1) = attr_3d_pos(1)*k;
    attr_pos_3d(2) = attr_3d_pos(2)*k;
    attr_pos_3d(3) = 1;

    //calculate right eye rototranslational matrix
    vector<double> dof_le = {le_q4, le_q6, le_q7};
    calc_re_rototranslational_matrix(dof_le, dof_le);

    //calculate the inverse of the left eye rototranslational matrix
    re_rtm_inv = re_rtm.inverse();

    //projection of the right arm end effector to the camera reference frame
    attr_projection_R = re_rtm_inv * attr_pos_3d;

}

void calc_camera_parameters_attractor_L(Vector4d projection)
{

    double xcam, ycam, zcam;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);

    xp_attr_L = xcam/zcam;
    yp_attr_L = ycam/zcam;
    r2_attr_L = xp_attr_L*xp_attr_L + yp_attr_L*yp_attr_L;

    ck_attr_L = (1 + k1_L*r2_attr_L + k2_L*pow(r2_attr_L,2) + k3_L*pow(r2_attr_L,3))/
          (1 + k4_L*r2_attr_L + k5_L*pow(r2_attr_L,2) + k6_L*pow(r2_attr_L,3));

    px_attr_L = 2*p1_L*xp_attr_L*yp_attr_L;
    pxx_attr_L = p2_L*(r2_attr_L + 2*pow(xp_attr_L,2));
    py_attr_L = 2*p2_L*xp_attr_L*yp_attr_L;
    pyy_attr_L = p1_L*(r2_attr_L + 2*pow(yp_attr_L,2));
}

void calc_camera_parameters_attractor_R(Vector4d projection)
{

    double xcam, ycam, zcam;

    xcam = projection(0);
    ycam = projection(1);
    zcam = projection(2);

    xp_attr_R = xcam/zcam;
    yp_attr_R = ycam/zcam;
    r2_attr_R = xp_attr_R*xp_attr_R + yp_attr_R*yp_attr_R;

    ck_attr_R = (1 + k1_R*r2_attr_R + k2_R*pow(r2_attr_R,2) + k3_R*pow(r2_attr_R,3))/
          (1 + k4_R*r2_attr_R + k5_R*pow(r2_attr_R,2) + k6_R*pow(r2_attr_R,3));

    px_attr_R = 2*p1_R*xp_attr_R*yp_attr_R;
    pxx_attr_R = p2_R*(r2_attr_R + 2*pow(xp_attr_R,2));
    py_attr_R = 2*p2_R*xp_attr_R*yp_attr_R;
    pyy_attr_R = p1_R*(r2_attr_R + 2*pow(yp_attr_R,2));
}

double calc_attractor_l_u()
{
    calc_le_attractor_projection();
    calc_camera_parameters_attractor_L(attr_projection_L);

    double vision_u = fx_L*(xp_attr_L*ck_attr_L + px_attr_L + pxx_attr_L) + cx_L;

    return vision_u;
}

double calc_attractor_l_v()
{
    //calc_le_attractor_projection();
    //calc_camera_parameters_attractor_L(attr_projection_L);

    double vision_v = fy_L*(yp_attr_L*ck_attr_L + py_attr_L + pyy_attr_L) + cy_L;

    return vision_v;
}

double calc_attractor_r_u()
{
    calc_re_attractor_projection();
    calc_camera_parameters_attractor_R(attr_projection_R);

    double vision_u = fx_R*(xp_attr_R*ck_attr_R + px_attr_R + pxx_attr_R) + cx_R;

    return vision_u;
}

double calc_attractor_r_v()
{
    //calc_le_attractor_projection();
    //calc_camera_parameters_attractor_R(attr_projection_R);

    double vision_v = fy_R*(yp_attr_R*ck_attr_R + py_attr_R + pyy_attr_R) + cy_R;

    return vision_v;
}

//Left arm
double sense_vision_u_b(vector<double> mu, vector<double> a)
{
    //read by vision thread

    if (publish){
        Bottle& output_ub = plot_ub.prepare();
        output_ub.clear();
        output_ub.addDouble(le_pos[0]);
        plot_ub.write();
    }

    return le_pos[0];
}

double sense_vision_v_b(vector<double> mu, vector<double> a)
{
    if (publish){
        Bottle& output_vb = plot_vb.prepare();
        output_vb.clear();
        output_vb.addDouble(le_pos[1]);
        plot_vb.write();
    }

    return le_pos[1];
}

double calc_vision_u_b(vector<double> mu, vector<double> a)
{
    calc_le_camera_projection_b(mu, a);
    calc_camera_parameters_b(cam_projection_b);

    double vision_u = fx*(xp_b*ck_b + px_b + pxx_b) + cx;

    if (publish){
        Bottle& output_calc_ub = plot_calc_ub.prepare();
        output_calc_ub.clear();
        output_calc_ub.addDouble(vision_u);
        plot_calc_ub.write();
    }

    calc_vision_pos_b(0) = vision_u;

    return vision_u;

    return 0;
}

double calc_vision_v_b(vector<double> mu, vector<double> a)
{
    //calc_le_camera_projection_b(mu, a);
    //calc_camera_parameters_b(cam_projection);

    double vision_v = fy*(yp_b*ck_b + py_b + pyy_b) + cy;

    if (publish){
        Bottle& output_calc_vb = plot_calc_vb.prepare();
        output_calc_vb.clear();
        output_calc_vb.addDouble(vision_v);
        plot_calc_vb.write();
    }

    calc_vision_pos_b(1) = vision_v;

    return vision_v;

    return 0;
}

//Right arm
double d_vision_u_mu1(vector<double> mu, vector<double> a)
{	
	calc_d_le_camera_projection_mu1(mu, a);
	calc_d_camera_parameters_mu1(cam_projection, d1_cam_projection);
	
	double d_u_1 = fx*(d1xp*ck + xp*d1k + d1px + d1pxx);
	
	return d_u_1;
}

double d_vision_u_mu2(vector<double> mu, vector<double> a)
{
	calc_d_le_camera_projection_mu2(mu, a);
	calc_d_camera_parameters_mu2(cam_projection, d2_cam_projection);
	
	double d_u_2 = fx*(d2xp*ck + xp*d2k + d2px + d2pxx);
		
	return d_u_2;
}

double d_vision_u_mu3(vector<double> mu, vector<double> a)
{
	calc_d_le_camera_projection_mu3(mu, a);
	calc_d_camera_parameters_mu3(cam_projection, d3_cam_projection);
	
	double d_u_3 = fx*(d3xp*ck + xp*d3k + d3px + d3pxx);
		
	return d_u_3;
}

double d_vision_v_mu1(vector<double> mu, vector<double> a)
{
	//calc_d_le_camera_projection_mu1(mu, a);
	//calc_d_camera_parameters_mu1(cam_projection, d1_cam_projection);
	
	double d_v_1 = fy*(d1yp*ck + yp*d1k + d1py + d1pyy);
	
	return d_v_1;
}

double d_vision_v_mu2(vector<double> mu, vector<double> a)
{
	//calc_d_le_camera_projection_mu2(mu, a);
	//calc_d_camera_parameters_mu2(cam_projection, d2_cam_projection);
	
	double d_v_2 = fy*(d2yp*ck + yp*d2k + d2py + d2pyy);
	
	return d_v_2;
}

double d_vision_v_mu3(vector<double> mu, vector<double> a)
{
	//calc_d_le_camera_projection_mu3(mu, a);
	//calc_d_camera_parameters_mu3(cam_projection, d3_cam_projection);
	
	double d_v_3 = fy*(d3yp*ck + yp*d3k + d3py + d3pyy);
	
	return d_v_3;
}

//Head
double d_vision_u_mu1_head(vector<double> mu, vector<double> a)
{	
    calc_camera_parameters_h(le_projection);
    calc_d_le_camera_origin_mu1(mu, a);
    calc_d_camera_parameters_mu1_h(le_projection, d1_le_projection);
	
    double d_u_1 = fx*(d1xp_h*ck_h + xp_h*d1k_h + d1px_h + d1pxx_h);
	
	return d_u_1;
}

double d_vision_u_mu2_head(vector<double> mu, vector<double> a)
{
    calc_camera_parameters_h(le_projection);
    calc_d_le_camera_origin_mu2(mu, a);
    calc_d_camera_parameters_mu2_h(le_projection, d2_le_projection);
	
    double d_u_2 = fx*(d2xp_h*ck_h + xp_h*d2k_h + d2px_h + d2pxx_h);
		
	return d_u_2;
}

double d_vision_u_mu3_head(vector<double> mu, vector<double> a)
{
    calc_camera_parameters_h(le_projection);
    calc_d_le_camera_origin_mu3(mu, a);
    calc_d_camera_parameters_mu3_h(le_projection, d3_le_projection);

    double d_u_3 = fx*(d3xp_h*ck_h + xp_h*d3k_h + d3px_h + d3pxx_h);

    return d_u_3;
}

double d_vision_v_mu1_head(vector<double> mu, vector<double> a)
{
    //calc_d_le_camera_origin_mu1(mu, a);
    //calc_camera_parameters_h(le_projection);
    //calc_d_camera_parameters_mu1_h(le_projection, d1_le_projection);
	
    double d_v_1 = fy*(d1yp_h*ck_h + yp_h*d1k_h + d1py_h + d1pyy_h);
	
	return d_v_1;
}

double d_vision_v_mu2_head(vector<double> mu, vector<double> a)
{
    //calc_d_le_camera_origin_mu2(mu, a);
    //calc_d_camera_parameters_mu2_h(le_projection, d2_le_projection);
	
    double d_v_2 = fy*(d2yp_h*ck_h + yp_h*d2k_h + d2py_h + d2pyy_h);
	
	return d_v_2;
}

double d_vision_v_mu3_head(vector<double> mu, vector<double> a)
{
    //calc_d_le_camera_origin_mu3(mu, a);
    //calc_d_camera_parameters_mu3_h(le_projection, d3_le_projection);

    double d_v_3 = fy*(d3yp_h*ck_h + yp_h*d3k_h + d3py_h + d3pyy_h);

    return d_v_3;
}

//Left arm
double d_vision_u_mu1_b(vector<double> mu, vector<double> a)
{
    calc_d_le_camera_projection_mu1_b(mu, a);
    calc_d_camera_parameters_mu1_b(cam_projection_b, d1_cam_projection_b);

    double d_u_1 = fx*(d1xp_b*ck_b + xp_b*d1k_b + d1px_b + d1pxx_b);

    return d_u_1;

    return 0;
}

double d_vision_u_mu2_b(vector<double> mu, vector<double> a)
{
    calc_d_le_camera_projection_mu2_b(mu, a);
    calc_d_camera_parameters_mu2_b(cam_projection_b, d2_cam_projection_b);

    double d_u_2 = fx*(d2xp_b*ck_b + xp_b*d2k_b + d2px_b + d2pxx_b);

    return d_u_2;

    return 0;
}

double d_vision_u_mu3_b(vector<double> mu, vector<double> a)
{
    calc_d_le_camera_projection_mu3_b(mu, a);
    calc_d_camera_parameters_mu3_b(cam_projection_b, d3_cam_projection_b);

    double d_u_3 = fx*(d3xp_b*ck_b + xp_b*d3k_b + d3px_b + d3pxx_b);

    return d_u_3;

    return 0;
}

double d_vision_v_mu1_b(vector<double> mu, vector<double> a)
{
    //calc_d_le_camera_projection_mu1_b(mu, a);
    //calc_d_camera_parameters_mu1_b(cam_projection_b, d1_cam_projection_b);

    double d_v_1 = fy*(d1yp_b*ck_b + yp_b*d1k_b + d1py_b + d1pyy_b);

    return d_v_1;

    return 0;
}

double d_vision_v_mu2_b(vector<double> mu, vector<double> a)
{
    //calc_d_le_camera_projection_mu2_b(mu, a);
    //calc_d_camera_parameters_mu2_b(cam_projection_b, d2_cam_projection_b);

    double d_v_2 = fy*(d2yp_b*ck_b + yp_b*d2k_b + d2py_b + d2pyy_b);

    return d_v_2;

    return 0;
}

double d_vision_v_mu3_b(vector<double> mu, vector<double> a)
{
    //calc_d_le_camera_projection_mu3_b(mu, a);
    //calc_d_camera_parameters_mu3_b(cam_projection_b, d3_cam_projection_b);

    double d_v_3 = fy*(d3yp_b*ck_b + yp_b*d3k_b + d3py_b + d3pyy_b);

    return d_v_3;

    return 0;
}

//Prior ----------------------------------------------------------------------------
//Right arm
double initial_prior_mu1(vector<double> mu, vector<double> a)
{
	return initialPrior[0];
}

double initial_prior_mu2(vector<double> mu, vector<double> a)
{
	return initialPrior[1];
}

double initial_prior_mu3(vector<double> mu, vector<double> a)
{
	return initialPrior[2];
}

double initial_prior_mu4(vector<double> mu, vector<double> a)
{
	return initialPrior[3];
}

//Left arm
double initial_prior_mu1b(vector<double> mu, vector<double> a)
{
    return initialPriorb[0];
}

double initial_prior_mu2b(vector<double> mu, vector<double> a)
{
    return initialPriorb[1];
}

double initial_prior_mu3b(vector<double> mu, vector<double> a)
{
    return initialPriorb[2];
}

double initial_prior_mu4b(vector<double> mu, vector<double> a)
{
    return initialPriorb[3];
}
