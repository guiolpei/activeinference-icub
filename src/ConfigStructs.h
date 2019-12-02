/*
 * ConfigStructs.h
 *
 * Description: Configuration structure definitions for 3D implementation
 *
 *  Created on: Jul 07, 2019
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

struct mainConfig { //config variables
	string robot = "null"; //robot name
	bool debug = false; //maximum debug messages in all classes
	bool print = false; //print updated states
	double dT = 0.01; //sampling time
	string integration_method = "Euler"; //integration method for variable update
	bool log = false; //create log
    string description = ""; //description of experiment
	bool publish = false; //publish data
	unsigned int cycles = 1000; //number of run cycles
};

struct armConfig { //right/left arm config variables
	bool activated = 0; //FEO applied to arm
	//initial position
    double torso_yaw = 0; //torso_yaw starting position (degrees)
	double shoulder_roll = 0; //shoulder_roll starting position (degrees)
	double shoulder_yaw = 0; //shoulder_yaw starting position (degrees)
	double elbow = 0; //elbow starting position (degrees)
	//internal state
    bool on_encoders = 1; //turn on encoders
	double var_d_encoders = 1; //first derivative encoder variance
	bool on_3d = true; //turn on 3d position
	double var_d_3d = 1; //first derivative 3d position variance
	bool on_vision = false; //turn on vision
	double var_d_vision = 1000; //first derivative vision variance
	int left_vision_hue_low = 0; //left eye vision hue lower bound
	int left_vision_sat_low = 0; //left eye vision saturation lower bound
	int left_vision_val_low = 0; //left eye vision value lower bound
	int left_vision_hue_high = 1; //left eye vision hue upper bound
	int left_vision_sat_high = 1; //left eye vision saturation upper bound
	int left_vision_val_high = 1; //left eye vision value upper bound
	int right_vision_hue_low = 0; //right eye vision hue lower bound
	int right_vision_sat_low = 0; //right eye vision saturation lower bound
	int right_vision_val_low = 0; //right eye vision value lower bound
	int right_vision_hue_high = 1; //right eye vision hue upper bound
	int right_vision_sat_high = 1; //right eye vision saturation upper bound
	int right_vision_val_high = 1; //right eye vision value upper bound
	int left_erode_dilate_iterations = 2; //left eye iterations for erode-dilate in object detection
	int right_erode_dilate_iterations = 2; //right eye iterations for erode-dilate in object detection
	double gain_perception_mu1 = 1; //perception proportional gain for integration of mu1
	double gain_perception_mu2 = 1; //perception proportional gain for integration of mu2
	double gain_perception_mu3 = 1; //perception proportional gain for integration of mu3
	double gain_perception_mu4 = 1; //perception proportional gain for integration of mu4
	double weighted_perception_mu1 = 1; //weighted perception parameter for mu1
	double weighted_perception_mu2 = 1; //weighted perception parameter for mu2
	double weighted_perception_mu3 = 1; //weighted perception parameter for mu3
	double weighted_perception_mu4 = 1; //weighted perception parameter for mu4
	bool perception_saturation_on = false; //perception saturation activated
	double mu1_saturation_low = 0; //mu1 saturation minimum value (degrees)
	double mu1_saturation_high = 180; //mu1 saturation maximum value (degrees)
	double mu2_saturation_low = 0; //mu2 saturation minimum value (degrees)
	double mu2_saturation_high = 180; //mu2 saturation maximum value (degrees)
	double mu3_saturation_low = 0; //mu3 saturation minimum value (degrees)
	double mu3_saturation_high = 180; //mu3 saturation maximum value (degrees)
	double mu4_saturation_low = 0; //mu3 saturation minimum value (degrees)
	double mu4_saturation_high = 180; //mu3 saturation maximum value (degrees)
	//prior
	bool on_prior = false; //turn on prior
	double var_d_prior = 1; //first derivative prior variance
	//first derivative
	bool on_derivative = 0; //right arm second derivative calculation
	double var_dd_attractor = 1; //first derivative attractor variance
	double gain_derivative_mu1p = 1; //perception proportional gain for integration of mu1p
	double gain_derivative_mu2p = 1; //perception proportional gain for integration of mu2p
	double gain_derivative_mu3p = 1; //perception proportional gain for integration of mu3p
	double gain_derivative_mu4p = 1; //perception proportional gain for integration of mu4p
	bool derivative_saturation_on = false; //perception saturation activated
	double mu1p_saturation_low = -2; //mu1p saturation minimum value (degrees/s)
	double mu1p_saturation_high = 2; //mu1p saturation maximum value (degrees/s)
	double mu2p_saturation_low = -2; //mu2p saturation minimum value (degrees/s)
	double mu2p_saturation_high = 2; //mu2p saturation maximum value (degrees/s)
	double mu3p_saturation_low = -2; //mu3p saturation minimum value (degrees/s)
	double mu3p_saturation_high = 2; //mu3p saturation maximum value (degrees/s)
	double mu4p_saturation_low = -2; //mu4p saturation minimum value (degrees/s)
	double mu4p_saturation_high = 2; //mu4p saturation maximum value (degrees/s)
	//action
	bool on_action = false; //turn on action
	double var_a_encoders = 1; //action encoder variance
	double var_a_3d = 1; //action 3d position variance
	double var_a_vision = 1000; //action vision variance
	double gain_action_q1 = 1; //action proportional gain for integration of a1
	double gain_action_q2 = 1; //action proportional gain for integration of a2
	double gain_action_q3 = 1; //action proportional gain for integration of a3
	double gain_action_q4 = 1; //action proportional gain for integration of a4
	bool action_saturation_on = true; //action saturation activated
	double action_saturation_low = -1; //action saturation minimum value (degrees/s)
	double action_saturation_high = 1; //action saturation maximum value (degrees/s)
	//attractor
	bool on_attractor = false; //turn on attractor
    int attractor_type = 0; //attractor type. 0: 3d stereo vision with hsv recognition (hsv), 1: 3d fixed position (x-y-z), 2: 3d cube (cycle)
	double ro1_attractor = 1; //attractor ro1 parameter
	double ro2_attractor = 0; //attractor ro2 parameter, point 1, x coordinate
	double ro3_attractor = 0; //attractor ro3 parameter, point 1, y coordinate
	double ro4_attractor = 0; //attractor ro4 parameter, point 1, z coordinate
    double ro5_attractor = 0; //attractor ro5 parameter, point 2, x coordinate
    double ro6_attractor = 0; //attractor ro6 parameter, point 2, y coordinate
    double ro7_attractor = 0; //attractor ro7 parameter, point 2, z coordinate
    double ro8_attractor = 0; //attractor ro8 parameter, point 3, x coordinate
    double ro9_attractor = 0; //attractor ro9 parameter, point 3, y coordinate
    double ro10_attractor = 0; //attractor ro10 parameter, point 3, z coordinate
    double ro11_attractor = 0; //attractor ro11 parameter, point 4, x coordinate
    double ro12_attractor = 0; //attractor ro12 parameter, point 4, y coordinate
    double ro13_attractor = 0; //attractor ro13 parameter, point 4, z coordinate
    double ro14_attractor = 0; //attractor ro14 parameter, point 5, x coordinate
    double ro15_attractor = 0; //attractor ro15 parameter, point 5, y coordinate
    double ro16_attractor = 0; //attractor ro16 parameter, point 5, z coordinate
    double ro17_attractor = 0; //attractor ro17 parameter, point 6, x coordinate
    double ro18_attractor = 0; //attractor ro18 parameter, point 6, y coordinate
    double ro19_attractor = 0; //attractor ro19 parameter, point 6, z coordinate
    double ro20_attractor = 0; //attractor ro20 parameter, point 7, x coordinate
    double ro21_attractor = 0; //attractor ro21 parameter, point 7, y coordinate
    double ro22_attractor = 0; //attractor ro22 parameter, point 7, z coordinate
    double ro23_attractor = 0; //attractor ro23 parameter, point 8, x coordinate
    double ro24_attractor = 0; //attractor ro24 parameter, point 8, y coordinate
    double ro25_attractor = 0; //attractor ro25 parameter, point 8, z coordinate
    double attractor_threshold = 0.01; //meter error to change attractor
    int attractor_comparison = 0; //0: compare with calculated, 1: compare with visual
	double var_d_attractor = 1; //first derivative attractor variance
	int left_attractor_hue_low = 0; //left eye attractor hue lower bound
	int left_attractor_sat_low = 0; //left eye attractor saturation lower bound
	int left_attractor_val_low = 0; //left eye attractor value lower bound
	int left_attractor_hue_high = 1; //left eye attractor hue upper bound
	int left_attractor_sat_high = 1; //left eye attractor saturation upper bound
	int left_attractor_val_high = 1; //left eye attractor value upper bound
	int right_attractor_hue_low = 0; //right eye attractor hue lower bound
	int right_attractor_sat_low = 0; //right eye attractor saturation lower bound
	int right_attractor_val_low = 0; //right eye attractor value lower bound
	int right_attractor_hue_high = 1; //right eye attractor hue upper bound
	int right_attractor_sat_high = 1; //right eye attractor saturation upper bound
	int right_attractor_val_high = 1; //right eye attractor value upper bound	
	bool fixed_attractor = 0; //fixed attractor, read at start
	//touch
	bool on_touch = 0; //activate touch sensor
    int touch_threshold = 5; //threshold to activate touch
	bool grasping = false;
	//noise
	bool gaussian_noise = false; //gaussian noise for encoder value
	double gaussian_mean = 0; //mean of the gaussian noise (degrees)
	double gaussian_std = 0; //variance of the gaussian noise (degrees)
	//other
	bool null_start = false; //start from zero position (mu1,mu2,mu3 = 0)
};

struct headConfig { //head config variables
	bool activated = 0; //FEO applied to head
	//initial position
	double neck_pitch = 0; //neck_pitch starting position (degrees)
	double neck_yaw = 0; //neck_yaw starting position (degrees)
    double eyes_tilt = 0; //eyes_tilt starting position (degrees)
	//internal state
	double var_d_encoders = 1; //first derivative encoder variance
	double gain_perception_mu1 = 1; //perception proportional gain for integration of mu1
	double gain_perception_mu2 = 1; //perception proportional gain for integration of mu2
    double gain_perception_mu3 = 1; //perception proportional gain for integration of mu3
	double weighted_perception_mu1 = 1; //weighted perception parameter for mu1
	double weighted_perception_mu2 = 1; //weighted perception parameter for mu2
    double weighted_perception_mu3 = 1; //weighted perception parameter for mu2
	bool perception_saturation_on = false; //perception saturation activated
    double mu1_saturation_low = 0; //mu1 saturation minimum value (degrees)
	double mu1_saturation_high = 180; //mu1 saturation maximum value (degrees)
	double mu2_saturation_low = 0; //mu2 saturation minimum value (degrees)
	double mu2_saturation_high = 180; //mu2 saturation maximum value (degrees)
    double mu3_saturation_low = 0; //mu3 saturation minimum value (degrees)
    double mu3_saturation_high = 180; //mu3 saturation maximum value (degrees)
	//first derivative
	bool on_derivative = 0; //head second derivative calculation
	double var_dd_attractor = 1; //first derivative attractor variance
	double gain_derivative_mu1p = 1; //perception proportional gain for integration of mu1p
	double gain_derivative_mu2p = 1; //perception proportional gain for integration of mu2p
    double gain_derivative_mu3p = 1; //perception proportional gain for integration of mu3p
	bool derivative_saturation_on = false; //perception saturation activated
	double mu1p_saturation_low = -2; //mu1p saturation minimum value (degrees/s)
	double mu1p_saturation_high = 2; //mu1p saturation maximum value (degrees/s)
	double mu2p_saturation_low = -2; //mu2p saturation minimum value (degrees/s)
	double mu2p_saturation_high = 2; //mu2p saturation maximum value (degrees/s)
    double mu3p_saturation_low = -2; //mu2p saturation minimum value (degrees/s)
    double mu3p_saturation_high = 2; //mu2p saturation maximum value (degrees/s)
	//action
	bool on_action = false; //turn on action
	double var_a_encoders = 1; //action encoder variance
	double gain_action_q1 = 1; //action proportional gain for integration of a1
	double gain_action_q2 = 1; //action proportional gain for integration of a2
    double gain_action_q3 = 1; //action proportional gain for integration of a3
	bool action_saturation_on = true; //action saturation activated
	double action_saturation_low = -1; //action saturation minimum value (degrees/s)
	double action_saturation_high = 1; //action saturation maximum value (degrees/s)
	//attractor
	double ro1_attractor = 1; //attractor ro1 parameter
	double var_d_attractor = 1; //first derivative attractor variance
	//other
	bool null_start = false; //start from zero position (mu1,mu2 = 0)
};

