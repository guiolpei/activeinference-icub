# ActiveInference-iCub

This is the repository of body perception and action via active inference for the iCub humanoid robot.

<img src="/media/icubalgo.png" width="600">

Explanation of the approach can be found in the following papers:

[1] Oliver, G., Lanillos, P., & Cheng, G. (2019). Active inference body perception and action for humanoid robots. arXiv preprint arXiv:1906.03022.

[2] Lanillos, P., & Cheng, G. (2018, October). Adaptive robot body learning and estimation through predictive coding. In 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 4083-4090). IEEE.

## Requirements

*  iCub humanoid robot v1.4.
*  Ubuntu 16.04 or higher.
*  YARP <3.0.0.

## Usage

There are two different implementations of the active inference method: the first one accounts for 2D visual servoing using monocular vision (activeInference2D) and the second one is an improved version of the former working in 3D space using binocular vision (activeInference3D).

### Parameters

There are four config files, shared between both implementations. The are three kinematic chains used in this implementation: head, right arm and left arm.

2D implementation works with monocular vision with 3 DOF for each one of the arms (shoulder_roll, shoulder_yaw and elbow) and 2 DOF for head movement (neck_pitch and neck_yaw).

3D implementation works with binocular vision with 1 DOF for the torso (torso_yaw), 3 DOF for each one of the arms (shoulder_roll, shoulder_yaw and elbow) and 3 DOF for head movement (eyes_tilt, neck_pitch and neck_yaw).

#### 2D implementation parameters

*  *config.ini*: Main config file. 
```
[MAIN CONFIG FILE]
robot = icub %robot name in YARP network
debug = 0 %print debug info
print = 0 %print current state for each iteration
dT = 0.02 %time step
integration = Euler %type of integration (only Euler currently available)
log = 1 %write log file
description = demo %name for the log file (added to the timestamp filename)
publish = 1 %publish variables for plotting
cycles = 100000 %number of cycles to run
```
*  *ra_config.ini*: Right arm config file. 

```
[RIGHT ARM CONFIG FILE]
right_arm_activated = 1 %turn on active inference algorithm for right arm
;initial position
shoulder_roll = 40 %initial DOF position (degrees)
shoulder_yaw = -20 %initial DOF position (degrees)
elbow = 60 %initial DOF position (degrees)
;internal state
on_encoders = 1 %turn on encodeer data reading
var_d_encoders = 0.1 %perception variance value for encoders (radians)
on_3d = 0 %turn on 3d perception (not implemented)
var_d_3d = 1 %perception variance value for 3d (not implemented)
on_vision = 1 %turn on 2d vision perception
var_d_vision = 10000 %perception variance for 2d vision (pixels/radians)
vision_hue_low = 15 %hue low value for end-effector vision recognition
vision_sat_low = 101 %saturation low value for end-effector vision recognition
vision_val_low = 144 %value low value for end-effector vision recognition
vision_hue_high = 34 %hue high value for end-effector vision recognition
vision_sat_high = 231 %saturation high value for end-effector vision recognition
vision_val_high = 255 %value high value for end-effector vision recognition
erode_dilate_iterations = 2 %erode-dilate iterations for vision recognition
gain_perception_mu1 = 1 %gain for perception integration
gain_perception_mu2 = 1 %gain for perception integration
gain_perception_mu3 = 1 %gain for perception integration
weighted_perception_mu1 = 1 %weight for perception integration
weighted_perception_mu2 = 1 %weight for perception integration
weighted_perception_mu3 = 1 %weight for perception integration
perception_saturation_on = 1 %turn on perception saturation
mu1_saturation_low = 10 %minimum value for perception
mu1_saturation_high = 140 %maximum value for perception
mu2_saturation_low = -30 %minimum value for perception
mu2_saturation_high = 90 %maximum value for perception
mu3_saturation_low = 20 %minimum value for perception
mu3_saturation_high = 80 %maximum value for perception
;prior
on_prior = 0 %turn on prior
var_d_prior = 1 %prior variance (degrees)
;first derivative
on_derivative = 1 %turn on first derivative calculation
var_dd_attractor = 0.1 %attractor variance for first derivative (radians)
gain_derivative_mu1p = 1 %gain for first derivative integration
gain_derivative_mu2p = 1 %gain for first derivative integration
gain_derivative_mu3p = 1 %gain for first derivative integration
derivative_saturation_on = 1 %turn on first derivative integration
mu1p_saturation_low = -5 %minimum value for first derivative
mu1p_saturation_high = 5 %maximum value for first derivative
mu2p_saturation_low = -5 %minimum value for first derivative
mu2p_saturation_high = 5 %maximum value for first derivative
mu3p_saturation_low = -5 %minimum value for first derivative
mu3p_saturation_high = 5 %maximum value for first derivative
;action
on_action = 0 %turn on action
var_a_encoders = 0.1 %action variance for encoders (radians)
var_a_3d = 1 %action variance for 3d (not implemented)
var_a_vision = 5000 %action variance for 2d vision
gain_action_q1 = 5 %action gain for integration
gain_action_q2 = 5 %action gain for integration
gain_action_q3 = 5 %action gain for integration
action_saturation_on = 1 %turn on action saturation
action_saturation_low = -5 %minimum value for action saturation
action_saturation_high = 5 %maximum value for action saturation
;attractor
on_attractor = 1 %turn on attractor dynamics
attractor_type = 2 %type of attractor. 0: 3d (4), 1: vision (3), 2: vision (hsv), 3: vision (cycle), 4: vision (face)
ro1_attractor = 1 %attractor weight
ro2_attractor = 150 %attractor parameters
ro3_attractor = 225 %attractor parameters
ro4_attractor = 198.5 %attractor parameters
ro5_attractor = 222.5 %attractor parameters
ro6_attractor = 263 %attractor parameters
ro7_attractor = 196 %attractor parameters
ro8_attractor = 290 %attractor parameters
ro9_attractor = 120 %attractor parameters
var_d_attractor = 1 %perception attractor variance
attractor_hue_low = 90 %hue low value for attractor vision recognition
attractor_sat_low = 75 %saturation low value for attractor vision recognition
attractor_val_low = 0 %value low value for attractor vision recognition
attractor_hue_high = 105 %hue high value for attractor vision recognition
attractor_sat_high = 255 %saturation high value for attractor vision recognition
attractor_val_high = 255 %value high value for attractor vision recognition
fixed_attractor = 0 %set fixed attractor
;touch
on_touch = 1 %turn touch on
touch_threshold = 5 %touch threshold
grasping = 1 %turn on grasping
;noise
gaussian_noise = 0 %include gaussian noise on encoders
gaussian_mean = 0 %mean value of noise
gaussian_std = 40 %standard deviation of noise (degrees)
;other
null_start = 0 %start with all internal values at 0
```


*  *la_config.ini*: Same as right arm config file, for the left arm.

*  *head_config.ini*:
```
[HEAD CONFIG FILE]
head_activated = 1 %turn on active inference algorithm for head
;initial position
neck_pitch = -30 %initial DOF position (degrees)
neck_yaw = -25 %initial DOF position (degrees)
;internal state
var_d_encoders = 0.1 %perception variance value for encoders (radians)
gain_perception_mu1 = 1 %gain for perception integration
gain_perception_mu2 = 1 %gain for perception integration
weighted_perception_mu1 = 1 %weight for perception integration
weighted_perception_mu2 = 1 %weight for perception integration
perception_saturation_on = 1 %turn on perception saturation
mu1_saturation_low = -35 %minimum value for perception
mu1_saturation_high = 25 %maximum value for perception
mu2_saturation_low = -50 %minimum value for perception
mu2_saturation_high = 50 %maximum value for perception
;first derivative
on_derivative = 0 %turn on first derivative calculation
var_dd_attractor = 0.2 %attractor variance for first derivative (radians)
gain_derivative_mu1p = 1 %gain for first derivative integration
gain_derivative_mu2p = 1 %gain for first derivative integration
derivative_saturation_on = 1 %turn on first derivative integration
mu1p_saturation_low = -5 %minimum value for first derivative
mu1p_saturation_high = 5 %maximum value for first derivative
mu2p_saturation_low = -5 %minimum value for first derivative
mu2p_saturation_high = 5 %maximum value for first derivative
;action
on_action = 0 %turn on action
var_a_encoders = 0.1 %action variance for encoders (radians)
gain_action_q1 = 50 %action gain for integration
gain_action_q2 = 100 %action gain for integration
action_saturation_on = 1 %turn on action saturation
action_saturation_low = -5 %minimum value for action saturation
action_saturation_high = 5 %maximum value for action saturation
;attractor
ro1_attractor = 20 %attractor weight
var_d_attractor = 0.5 %perception attractor variance
;other
null_start = 0 %start with all internal values at 0
```

#### 3D implementation parameters

*  *config.ini*: Main config file. 
```
[MAIN CONFIG FILE]
robot = icub %robot name in YARP network
debug = 0 %print debug info
print = 0 %print current state for each iteration
dT = 0.02 %time step
integration = Euler %type of integration (only Euler currently available)
log = 1 %write log file
description = test %name for the log file (added to the timestamp filename)
publish = 1 %publish variables for plotting
cycles = 100000 %number of cycles to run
```
*  *ra_config.ini*: Right arm config file. 

```
[RIGHT ARM CONFIG FILE]
right_arm_activated = 1 %turn on active inference algorithm for right arm
;initial position
torso_yaw = -10 %initial DOF position (degrees)
shoulder_roll = 40 %initial DOF position (degrees)
shoulder_yaw = -20 %initial DOF position (degrees)
elbow = 60 %initial DOF position (degrees)
;internal state
on_encoders = 1 %turn on encodeer data reading
var_d_encoders = 0.1 %perception variance value for encoders (radians)
on_3d = 1 %turn on 3d perception
var_d_3d = 0.005 %perception variance value for 3d (meters)
left_vision_hue_low = 15 %hue low value for end-effector vision recognition with left eye
left_vision_sat_low = 101 %saturation low value for end-effector vision recognition with left eye
left_vision_val_low = 144 %value low value for end-effector vision recognition with left eye
left_vision_hue_high = 34 %hue high value for end-effector vision recognition with left eye
left_vision_sat_high = 231 %saturation high value for end-effector vision recognition with left eye
left_vision_val_high = 255 %value high value for end-effector vision recognition with left eye
right_vision_hue_low = 15 %hue low value for end-effector vision recognition with right eye
right_vision_sat_low = 101 %saturation low value for end-effector vision recognition with right eye
right_vision_val_low = 144 %value low value for end-effector vision recognition with right eye
right_vision_hue_low = 34 %hue high value for end-effector vision recognition with right eye
right_vision_sat_low = 231 %saturation high value for end-effector vision recognition with right eye
right_vision_val_low = 255 %value high value for end-effector vision recognition with right eye
left_erode_dilate_iterations = 2 %erode-dilate iterations for vision recognition with left eye
right_erode_dilate_iterations = 2 %erode-dilate iterations for vision recognition with right eye
gain_perception_mu1 = 1 %gain for perception integration
gain_perception_mu2 = 1 %gain for perception integration
gain_perception_mu3 = 1 %gain for perception integration
gain_perception_mu4 = 1 %gain for perception integration
weighted_perception_mu1 = 1 %weight for perception integration
weighted_perception_mu2 = 1 %weight for perception integration
weighted_perception_mu3 = 1 %weight for perception integration
weighted_perception_mu4 = 1 %weight for perception integration
perception_saturation_on = 1 %turn on perception saturation
mu1_saturation_low = -50 %minimum value for perception
mu1_saturation_high = 45 %maximum value for perception
mu2_saturation_low = 10 %minimum value for perception
mu2_saturation_high = 150 %maximum value for perception
mu3_saturation_low = -30 %minimum value for perception
mu3_saturation_high = 70 %maximum value for perception
mu4_saturation_low = 15 %minimum value for perception
mu4_saturation_high = 100 %maximum value for perception
;prior
on_prior = 0 %turn on prior
var_d_prior = 1 %prior variance (degrees)
;first derivative
on_derivative = 1 %turn on first derivative calculation
var_dd_attractor = 0.1 %attractor variance for first derivative (radians)
gain_derivative_mu1p = 1 %gain for first derivative integration
gain_derivative_mu2p = 1 %gain for first derivative integration
gain_derivative_mu3p = 1 %gain for first derivative integration
gain_derivative_mu4p = 1 %gain for first derivative integration
derivative_saturation_on = 1 %turn on first derivative integration
mu1p_saturation_low = -5 %minimum value for first derivative
mu1p_saturation_high = 5 %maximum value for first derivative
mu2p_saturation_low = -5 %minimum value for first derivative
mu2p_saturation_high = 5 %maximum value for first derivative
mu3p_saturation_low = -5 %minimum value for first derivative
mu3p_saturation_high = 5 %maximum value for first derivative
mu4p_saturation_low = -5 %minimum value for first derivative
mu4p_saturation_high = 5 %maximum value for first derivative
;action
on_action = 0 %turn on action
var_a_encoders = 0.1 %action variance for encoders (radians)
var_a_3d = 0.005 %action variance for 3d (meters)
gain_action_q1 = 2 %action gain for integration
gain_action_q2 = 5 %action gain for integration
gain_action_q3 = 5 %action gain for integration
gain_action_q4 = 5 %action gain for integration
action_saturation_on = 1 %turn on action saturation
action_saturation_low = -5 %minimum value for action saturation
action_saturation_high = 5 %maximum value for action saturation
;attractor
on_attractor = 1 %turn on attractor dynamics
attractor_type = 2 %type of attractor. 0: stereo vision (hsv recognition), 1: fixed 3D position, 2: 3D cube (8x 3D points)
ro1_attractor = 1 %attractor weight
ro2_attractor = -0.325 %attractor parameters
ro3_attractor = 0.1 %attractor parameters
ro4_attractor = 0.05 %attractor parameters
ro5_attractor = -0.325 %attractor parameters
ro6_attractor = 0 %attractor parameters
ro7_attractor = 0.05 %attractor parameters
ro8_attractor = -0.4 %attractor parameters
ro9_attractor = 0 %attractor parameters
ro10_attractor = 0.2 %attractor parameters
ro11_attractor = -0.325 %attractor parameters
ro12_attractor = 0.1 %attractor parameters
ro13_attractor = 0.2 %attractor parameters
ro14_attractor = -0.4 %attractor parameters
ro15_attractor = 0 %attractor parameters
ro16_attractor = 0.05 %attractor parameters
ro17_attractor = -0.4 %attractor parameters
ro18_attractor = 0.1 %attractor parameters
ro19_attractor = 0.2 %attractor parameters
ro20_attractor = -0.325 %attractor parameters
ro21_attractor = 0.1 %attractor parameters
ro22_attractor = 0.05 %attractor parameters
ro23_attractor = -0.325 %attractor parameters
ro24_attractor = 0 %attractor parameters
ro25_attractor = 0.2 %attractor parameters
attractor_threshold = 0.005 %threshold for attractor position reached
attractor_comparison = 0 %compare attractor with calculated (0) or real (1) end-effector position
var_d_attractor = 0.1 %perception attractor variance
left_attractor_hue_low = 90 %hue low value for attractor vision recognition with left eye
left_attractor_sat_low = 75 %saturation low value for attractor vision recognition with left eye
left_attractor_val_low = 0 %value low value for attractor vision recognition with left eye
left_attractor_hue_high = 105 %hue high value for attractor vision recognition with left eye
left_attractor_sat_high = 255 %saturation high value for attractor vision recognition with left eye
left_attractor_val_high = 255 %value high value for attractor vision recognition with left eye
right_attractor_hue_low = 90 %hue low value for attractor vision recognition with right eye
right_attractor_sat_low = 75 %saturation low value for attractor vision recognition with right eye
right_attractor_val_low = 0 %value low value for attractor vision recognition with right eye
right_attractor_hue_high = 105 %hue high value for attractor vision recognition with right eye
right_attractor_sat_high = 255 %saturation high value for attractor vision recognition with right eye
right_attractor_val_high = 255 %value high value for attractor vision recognition with right eye
fixed_attractor = 0 %set fixed attractor
;touch
on_touch = 1 %turn touch on
touch_threshold = 5 %touch threshold
grasping = 1 %turn on grasping
;noise
gaussian_noise = 0 %include gaussian noise on encoders
gaussian_mean = 0 %mean value of noise
gaussian_std = 40 %standard deviation of noise (degrees)
;other
null_start = 0 %start with all internal values at 0
```


*  *la_config.ini*: Same as right arm config file, for the left arm. Torso DOF is driven by right arm, not present in left arm config file.

*  *head_config.ini*:
```
[HEAD CONFIG FILE]
head_activated = 1 %turn on active inference algorithm for head
;initial position
neck_pitch = -30 %initial DOF position (degrees)
neck_yaw = -25 %initial DOF position (degrees)
eyes_tilt = -20 %initial DOF position (degrees)
;internal state
var_d_encoders = 0.1 %perception variance value for encoders (radians)
gain_perception_mu1 = 1 %gain for perception integration
gain_perception_mu2 = 1 %gain for perception integration
gain_perception_mu3 = 1 %gain for perception integration
weighted_perception_mu1 = 1 %weight for perception integration
weighted_perception_mu2 = 1 %weight for perception integration
weighted_perception_mu3 = 1 %weight for perception integration
perception_saturation_on = 1 %turn on perception saturation
mu1_saturation_low = -35 %minimum value for perception
mu1_saturation_high = 25 %maximum value for perception
mu2_saturation_low = -50 %minimum value for perception
mu2_saturation_high = 50 %maximum value for perception
mu3_saturation_low = -20 %minimum value for perception
mu3_saturation_high = 5 %maximum value for perception
;first derivative
on_derivative = 0 %turn on first derivative calculation
var_dd_attractor = 0.2 %attractor variance for first derivative (radians)
gain_derivative_mu1p = 1 %gain for first derivative integration
gain_derivative_mu2p = 1 %gain for first derivative integration
gain_derivative_mu3p = 1 %gain for first derivative integration
derivative_saturation_on = 1 %turn on first derivative integration
mu1p_saturation_low = -5 %minimum value for first derivative
mu1p_saturation_high = 5 %maximum value for first derivative
mu2p_saturation_low = -5 %minimum value for first derivative
mu2p_saturation_high = 5 %maximum value for first derivative
mu3p_saturation_low = -5 %minimum value for first derivative
mu3p_saturation_high = 5 %maximum value for first derivative
;action
on_action = 0 %turn on action
var_a_encoders = 0.1 %action variance for encoders (radians)
gain_action_q1 = 30 %action gain for integration
gain_action_q2 = 100 %action gain for integration
gain_action_q2 = 80 %action gain for integration
action_saturation_on = 1 %turn on action saturation
action_saturation_low = -5 %minimum value for action saturation
action_saturation_high = 5 %maximum value for action saturation
;attractor
ro1_attractor = 20 %attractor weight
var_d_attractor = 0.5 %perception attractor variance
;other
null_start = 0 %start with all internal values at 0
```

### Running

Once all the config files are correctly set, the algorithm can be runned using:

`./activeInference`

The flow of the program execution is as follows:

1.  Program reads the config files and prints on screen the values, any field not found will be set to its default value.
2.  Robot is driven to its initial state, this applies for all the arms that are activated (right/left arm) and always for the head.
3.  Kinematics library is loaded and all the calculations are printed on screen. The values can be checked for incongruencies.
4.  If publishing is activated, all the ports are set.
5.  If vision is activated, the vision thread starts processing images from camera.
6.  If touch is activated, the touch thread starts reading touch sensor data.
7.  If noise is activated, the noise generator is started.
8.  All the terms (percepetion and action) for the free-energy optimization algorithm are defined.
9.  The free-energy optimization algorithm starts running. Its execution will end when the number of cycles is reached or when Ctrl-C is pressed.

Tip: If vision and/or publishing is activated, the published values can be graphed in real time using *yarpscope* and the camera vision can be shown (locations read from published values printed on camera image). 
