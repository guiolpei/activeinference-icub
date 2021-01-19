/*
 * ICubImage.h
 * 
 * Description: Dedicated class to process the image information of the robot.
 *
 *  Created on: Nov 13, 2018
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

#ifndef ICUBIMAGE_H_
#define ICUBIMAGE_H_

//Includes
//Standard Library
#include <iostream> //cout
#include <vector> //vectors
#include <string> //strings

//OpenCV
#include <opencv/cv.h> //legacy code (cvCvtColor, IplImage to mat)
#include <opencv2/core/core.hpp> //OpenCV core
//#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp> //BlobDetector
#include <opencv2/highgui/highgui.hpp> //cvtColor, cvNamedWindow, cvShowImage
#include <opencv2/objdetect.hpp> //CascadeClassifier
#include <opencv2/calib3d.hpp>

//Yarp
#include <yarp/os/all.h> //OS
#include <yarp/sig/all.h> //Signal processing

//Namespaces
using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

#ifndef enum_side
#define enum_side
enum Side { R, L, RL };
#endif

class ICubImage
{
public:
	ICubImage(string name, bool debug); //Constructor
	~ICubImage(); //Destructor

    //Setup
	void setupImageRead(vector<string> imgPort, Side sel); //Open ports for image processing

    //Object detection
    yarp::sig::Vector findLocationCV(Side sel, Scalar col_low, Scalar col_high, int iterations, bool show); //Find specific location using OpenCV
    bool initFaceDetection(string cascade_name);
    yarp::sig::Vector detectFace(Side sel, bool show);

    //Object detection to 3D
    bool setParameters3D(Mat matrixCam1, Mat distCam1, Mat matrixCam2, Mat distCam2, Mat matrixRot, Mat matrixTrans);
    yarp::sig::Vector get3Dfrom2D(Side sel, Scalar left_col_low, Scalar left_col_high, int left_iterations, Scalar right_col_low, Scalar right_col_high, int right_iterations, int get2D, bool show);
private:
	
	Network yarp; //Setup yarp
	BufferedPort<ImageOf<PixelRgb>> imagePortR; //Port for reading images (right)
	BufferedPort<ImageOf<PixelRgb>> imagePortL; //Port for reading images (left)
    CascadeClassifier face_cascade; //Cascade face detector

	//Internal status
	string robot_name;
	bool port_status;
	Side port_selection;
	bool debug;
    bool cascade_status;

    //Internal camera parameters
    Mat M1, D1, M2, D2, Rot, Trans; //camera matrices, distortion parameters and rototranslational matrices
    Mat R1, P1, R2, P2; //rectification and rectified projection matrices
    Mat Q; //disparity-to-depth mapping matrix
};

#endif /* ICUBIMAGE_H_ */
