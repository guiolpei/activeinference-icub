/*
 * ICubImage.cpp
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

#include "ICubImage.h"

ICubImage::ICubImage(string name, bool debug)
{
	//Constructor
	this->debug = debug;
	robot_name = name; //robot name: icub/icubSim
	port_status = false;
    cascade_status = false;
}

ICubImage::~ICubImage()
{
	//Destructor
	imagePortR.close();
	imagePortL.close();
}

void ICubImage::setupImageRead(vector<string> imgPort, Side sel)
{
	//Setup image reading
	
	if ( imgPort.empty() ){
		cout << "Empty string vectors for port configuration." << endl;
		return;
	}
	
	
	switch (sel)
	{
		case R:
			if (debug) cout << "Setting up right eye image reading..." << endl;
			imagePortR.open(imgPort[0]);
			//Network::connect("/" + robot_name + "/camcalib/right/out",imgPort[0]);
			Network::connect("/" + robot_name + "/cam/right",imgPort[0]);
			break;
		case L:
			if (debug) cout << "Setting up left eye image reading..." << endl;
			imagePortL.open(imgPort[0]);
			//Network::connect("/" + robot_name + "/camcalib/left/out",imgPort[0]);
			Network::connect("/" + robot_name + "/cam/left",imgPort[0]);
			break;
		case RL:
			if (debug) cout << "Setting up right and left eye image reading..." << endl;
			if ( (imgPort.size() != 2) ){
				cout << "Invalid string vectors for port configuration." << endl;
				return;
			}
			imagePortR.open(imgPort[0]);
			imagePortL.open(imgPort[1]);
			//Network::connect("/" + robot_name + "/camcalib/right/out",imgPort[0]);
			//Network::connect("/" + robot_name + "/camcalib/left/out",imgPort[1]);
			Network::connect("/" + robot_name + "/cam/right",imgPort[0]);
            Network::connect("/" + robot_name + "/cam/left",imgPort[1]);
			break;
	}
	
	port_status = true;
	port_selection = sel;
}

yarp::sig::Vector ICubImage::findLocationCV(Side sel, Scalar col_low, Scalar col_high, int iterations, bool show)
{
	yarp::sig::Vector pos(3); pos.zero();
	ImageOf<PixelRgb> *yarpImage;
	IplImage *cvImage;
	
	if (!port_status)
	{
		cout << "Ports must be opened before attempting read!" << endl;
		return pos;
	}
	
	if (sel != port_selection)
	{
		cout << "Image reading must be done from the same ports that were opened!" << endl;
		return pos;
	}
	
	//Read from port
	switch (sel)
	{
		case R:
			yarpImage = imagePortR.read(true);
			break;
		case L:
			yarpImage = imagePortL.read(true);
			break;
	}


	if (debug) cout << "Read image of width: " << yarpImage->width() << ", height: " << yarpImage->height() << endl;

	//Yarp to OpenCV conversion (RGB -> BGR)
    cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
    cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);

	//Convert to new format and show
	Mat m = cvarrToMat(cvImage);
	if (show)
	{ 
		imshow("opencv", m);
		waitKey(3000);
		cvDestroyWindow("opencv");
	}
	
	//Convert to HSV format
	Mat hsv;
	cvtColor(m, hsv, COLOR_BGR2HSV);  
	if (show)
	{ 
		imshow("hsv", hsv);	
		waitKey(3000);
		cvDestroyWindow("hsv");
	}

	//Apply mask
	Mat mask, mask1, mask2;
	inRange(hsv, col_low, col_high, mask);
   	//mask = mask1 | mask2;
	if (show)
	{ 	
		imshow("mask", mask);	
		waitKey(3000);
		cvDestroyWindow("mask");
	}
	
	//Erode and dilate
	Mat mask3;
    erode(mask, mask3, Mat(), Point(-1, -1), iterations);
    dilate(mask3, mask, Mat(), Point(-1, -1), iterations);
	if (show)
	{ 	
		imshow("erode and dilate", mask);
		waitKey(3000);
		cvDestroyWindow("erode and dilate");
	}
	
	//Detect edges
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours( mask, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<vector<Point> > contours_poly( contours.size() );
	//vector<Rect> boundRect( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );
	//cout << "Found " << contours.size() << " contours..." << endl;
	for( size_t i = 0; i < contours.size(); i++ )
	{
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		//boundRect[i] = boundingRect( Mat(contours_poly[i]) );
		minEnclosingCircle( contours_poly[i], center[i], radius[i] );
        pos[0] = center[i].x; //x position of object
        pos[1] = center[i].y; //y position of object
        pos[2] = radius[i]; //size
		//cout << "The center of contour " << i << " is at: " << center[i].x << " " << center[i].y << endl;

        //if (i > 0) break; //only first
		
	}

	//Draw contours over image
	Mat drawing = m; //Mat::zeros( m.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rand()&255, rand()&255, rand()&255 );
		//drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		//rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
		circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
	}

    if (show)
    {
		cvNamedWindow("contours", WINDOW_AUTOSIZE);
		imshow("contours", drawing);
		//waitKey(3000);	
		//cvDestroyWindow("contours");
    }

	//cycle
	//waitKey(100);
	//cvDestroyWindow("contours");	

	/*//wait for window to be closed
	try{
		while (getWindowProperty("contours", 0) != -1){
			waitKey(50);
		}
	}
	catch (Exception)
	{

	}*/

	

	//Cleanup
	cvReleaseImage(&cvImage);	
	//imagePort.close();
	
	return pos;
}

bool ICubImage::initFaceDetection(string cascade_name)
{
    //load cascade files
    if(!face_cascade.load(cascade_name))
    {
        if (debug) cout << "Error loading face cascade!" << endl;
        cascade_status = false;
        return false;
    }

    cascade_status = true;
    return true;

}

yarp::sig::Vector ICubImage::detectFace(Side sel, bool show)
{
    yarp::sig::Vector pos(3); pos.zero();
    ImageOf<PixelRgb> *yarpImage;
    IplImage *cvImage;

    if (!port_status)
    {
        cout << "Ports must be opened before attempting read!" << endl;
        return pos;
    }

    if (sel != port_selection)
    {
        cout << "Image reading must be done from the same ports that were opened!" << endl;
        return pos;
    }

    if (!cascade_status)
    {
        cout << "Cascade file must be loaded before running face detection!" << endl;
        return pos;
    }

    //Read from port
    switch (sel)
    {
        case R:
            yarpImage = imagePortR.read(true);
            break;
        case L:
            yarpImage = imagePortL.read(true);
            break;
    }


    if (debug) cout << "Read image of width: " << yarpImage->width() << ", height: " << yarpImage->height() << endl;

    //Yarp to OpenCV conversion (RGB -> BGR)
    cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
    cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);

    //Convert to new format and show
    Mat m = cvarrToMat(cvImage);
    if (show)
    {
        imshow("opencv", m);
        waitKey(3000);
        cvDestroyWindow("opencv");
    }

    //Convert to grayscale
    Mat gray;
    cvtColor(m, gray, COLOR_BGR2GRAY);
    equalizeHist(gray, gray);
    if (show)
    {
        imshow("grayscale", gray);
        waitKey(3000);
        cvDestroyWindow("grayscale");
    }

    //Detect faces
    vector<Rect> faces;

    face_cascade.detectMultiScale(gray, faces, 1.5, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30), Size(100, 100));

    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( m, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

        pos[0] = center.x; //x position of face
        pos[1] = center.y; //y position of face
        pos[2] = (faces[i].width/2 < faces[i].height/2) ? faces[i].height/2 : faces[i].width/2; //maximum of width and height

        if (i == 0) break; //only first

        /*Mat faceROI = frame_gray( faces[i] );
        std::vector<Rect> eyes;

        //-- In each face, detect eyes
        eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(30, 30) );

        for( size_t j = 0; j < eyes.size(); j++ )
        {
            Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            circle( frame, eye_center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
        }*/
    }

    if (show)
    {
        //show
        imshow("face detection", m );
    }


    //Cleanup
    cvReleaseImage(&cvImage);
    //imagePort.close();

    return pos;


}

bool ICubImage::setParameters3D(Mat matrixCam1, Mat distCam1, Mat matrixCam2, Mat distCam2, Mat matrixRot, Mat matrixTrans)
{
    //Set camera parameters
    M1 = matrixCam1;
    D1 = distCam1;
    M2 = matrixCam2;
    D2 = distCam2;
    Rot = matrixRot;
    Trans = matrixTrans;

    R1 = (Mat_<double>(3,3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
    R2 = (Mat_<double>(3,3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
    P1 = (Mat_<double>(3,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    P2 = (Mat_<double>(3,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    Q = (Mat_<double>(4,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    return true;

}

yarp::sig::Vector ICubImage::get3Dfrom2D(Side sel, Scalar left_col_low, Scalar left_col_high, int left_iterations, Scalar right_col_low, Scalar right_col_high, int right_iterations, int get2D, bool show)
{
    yarp::sig::Vector pos_R(3), pos_L(3); pos_R.zero(); pos_L.zero();
    yarp::sig::Vector pos_3D(9); pos_3D.zero();
    ImageOf<PixelRgb> *yarpImageR, *yarpImageL;
    IplImage *cvImageR,*cvImageL;

    if (!port_status)
    {
        cout << "Ports must be opened before attempting read!" << endl;
        return pos_3D;
    }

    if (sel != port_selection)
    {
        cout << "Image reading must be done from the same ports that were opened!" << endl;
        return pos_3D;
    }

    //Read from port
    switch (sel)
    {
        case R:
        case L:
            cout << "Must be done with both cameras!" << endl;
            return pos_3D;
        case RL:
            yarpImageL = imagePortL.read(true);
            yarpImageR = imagePortR.read(true);
            break;
    }


    if (debug) cout << "Read right eye image of width: " << yarpImageR->width() << ", height: " << yarpImageR->height() << endl;
    if (debug) cout << "Read left eye image of width: " << yarpImageL->width() << ", height: " << yarpImageL->height() << endl;

    //Yarp to OpenCV conversion (RGB -> BGR)
    cvImageR = cvCreateImage(cvSize(yarpImageR->width(), yarpImageR->height()), IPL_DEPTH_8U, 3);
    cvCvtColor((IplImage*)yarpImageR->getIplImage(), cvImageR, CV_RGB2BGR);
    cvImageL = cvCreateImage(cvSize(yarpImageL->width(), yarpImageL->height()), IPL_DEPTH_8U, 3);
    cvCvtColor((IplImage*)yarpImageL->getIplImage(), cvImageL, CV_RGB2BGR);

    //Convert to new format and show
    Mat m_R = cvarrToMat(cvImageR);
    Mat m_L = cvarrToMat(cvImageL);

    //Convert to HSV format
    Mat hsv_R, hsv_L;
    cvtColor(m_R, hsv_R, COLOR_BGR2HSV);
    cvtColor(m_L, hsv_L, COLOR_BGR2HSV);

    //Apply mask
    Mat mask_R, mask_L;
    inRange(hsv_R, right_col_low, right_col_high, mask_R);
    inRange(hsv_L, left_col_low, left_col_high, mask_L);

    //Erode and dilate
    Mat mask3_R, mask3_L;
    erode(mask_R, mask3_R, Mat(), Point(-1, -1), right_iterations);
    dilate(mask3_R, mask_R, Mat(), Point(-1, -1), right_iterations);
    erode(mask_L, mask3_L, Mat(), Point(-1, -1), left_iterations);
    dilate(mask3_L, mask_L, Mat(), Point(-1, -1), left_iterations);

    //Detect edges
    vector<vector<Point>> contours_R, contours_L;
    vector<Vec4i> hierarchy;
    findContours( mask_R, contours_R, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( mask_L, contours_L, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<vector<Point> > contours_poly_R( contours_R.size() );
    vector<vector<Point> > contours_poly_L( contours_L.size() );
    //vector<Rect> boundRect( contours.size() );
    vector<Point2f>center_R( contours_R.size() );
    vector<Point2f>center_L( contours_L.size() );
    vector<float>radius_R( contours_R.size() );
    vector<float>radius_L( contours_L.size() );
    //cout << "Found " << contours.size() << " contours..." << endl;

    //Right eye
    for( size_t i = 0; i < contours_R.size(); i++ )
    {
        approxPolyDP( Mat(contours_R[i]), contours_poly_R[i], 3, true );
        //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( contours_poly_R[i], center_R[i], radius_R[i] );
        pos_R[0] = center_R[i].x; //x position of object
        pos_R[1] = center_R[i].y; //y position of object
        pos_R[2] = radius_R[i]; //size
        //cout << "The center of contour " << i << " is at: " << center[i].x << " " << center[i].y << endl;

        //if (i > 0) break; //only first

    }

    if (debug) cout << "Object detected at right eye position: (" << pos_R[0] << ", " << pos_R[1] << ")" << endl; //, with size: " << pos_R[2] << endl;

    //Left eye
    for( size_t i = 0; i < contours_L.size(); i++ )
    {
        approxPolyDP( Mat(contours_L[i]), contours_poly_L[i], 3, true );
        //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( contours_poly_L[i], center_L[i], radius_L[i] );
        pos_L[0] = center_L[i].x; //x position of object
        pos_L[1] = center_L[i].y; //y position of object
        pos_L[2] = radius_L[i]; //size
        //cout << "The center of contour " << i << " is at: " << center[i].x << " " << center[i].y << endl;

        //if (i > 0) break; //only first

    }

    if (debug) cout << "Object detected at left eye position: (" << pos_L[0] << ", " << pos_L[1] << ")" << endl; //, with size: " << pos_L[2] << endl;

    //Check for detection
    if ((pos_L[0] == 0 && pos_L[1] == 0) || (pos_R[0] == 0 && pos_R[1] == 0))
    {
        if (debug) cout << "Object was not detected by both cameras." << endl;
        return pos_3D;
    }


    //Compute rectification transforms for each head of a calibrated stereo camera
    if (debug) cout << "Using stereoRectify..." << endl;
    Size img_size = cvSize(yarpImageR->width(), yarpImageR->height()); //size of image
    Rect roi1, roi2;
    if (debug) cout << M1 << endl << M2 << endl << D1 << endl << D2 << endl << Rot << endl << Trans << endl;
    stereoRectify(M1, D1, M2, D2, img_size, Rot, Trans, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2);
    if (debug) cout << "stereoRectify results:" << endl;
    if (debug) cout << R1 << endl << R2 << endl << P1 << endl << P2 << endl << Q << endl;

    //Undistort points from both cameras using rectification transforms
    if (debug) cout << "Using undistortPoints..." << endl;
    Mat_<Point2d> dpos_L(1,1), dpos_R(1,1);
    Mat_<Point2d> upos_L(1,1), upos_R(1,1);
    dpos_L(0) = Point2d(pos_L[0],pos_L[1]);
    dpos_R(0) = Point2d(pos_R[0],pos_R[1]);
    if (debug) cout << dpos_L << endl << dpos_R << endl;
    undistortPoints(dpos_L, upos_L, M1, D1, R1, P1);
    undistortPoints(dpos_R, upos_R, M2, D2, R2, P2);
    if (debug) cout << "undistortPoints results:" << endl;
    if (debug) cout << upos_L << endl << upos_R << endl;

    //Obtain 3D point using perspective matrix transformation
    if (debug) cout << "Using perspectiveTransform..." << endl;
    double x_left = upos_L.at<Point2d>(0).x;
    double y_left = upos_L.at<Point2d>(0).y;
    double x_right = upos_R.at<double>(0,0);
    //cout << x_left << " " << y_left << " " << x_right << endl;
    Mat inputPos = Mat(1, 1, CV_64FC3);
    Mat outputPos = Mat(1, 1, CV_64FC3);
    inputPos.at<Vec3d>(0,0) = Vec3d(x_left,y_left,x_left-x_right);
    if (debug) cout << inputPos << endl;
    perspectiveTransform(inputPos, outputPos, Q);
    if (debug) cout << "perspectiveTransform results:" << endl;
    if (debug) cout << outputPos << endl;

    if (debug) cout << "Finishing..." << endl;
    
	//Assign 3D to output
    pos_3D[0] = outputPos.at<double>(0,0);
    pos_3D[1] = outputPos.at<double>(0,1);
    pos_3D[2] = outputPos.at<double>(0,2);

	//Assign 2D to output
	switch (get2D)
	{
		case 0:
			pos_3D[3] = 0;
			pos_3D[4] = 0;
			pos_3D[5] = 0;
			break;
		case 1: //left eye
			pos_3D[3] = pos_L[0]; //u
			pos_3D[4] = pos_L[1]; //v
			pos_3D[5] = pos_L[2]; //size
			break;
		case 2: //right eye
			pos_3D[3] = pos_R[0]; //u
			pos_3D[4] = pos_R[1]; //v
            pos_3D[5] = pos_R[2]; //size
            break;
        case 3:
            pos_3D[3] = pos_L[0]; //u
            pos_3D[4] = pos_L[1]; //v
            pos_3D[5] = pos_L[2]; //size
            pos_3D[6] = pos_R[0]; //u
            pos_3D[7] = pos_R[1]; //v
            pos_3D[8] = pos_R[2]; //size
			break;
		
	}

    //Cleanup
    cvReleaseImage(&cvImageR);
    cvReleaseImage(&cvImageL);

    return pos_3D;


}
