/*

 * kinect2_calib.h
 *
 *  Created on: 28.08.2015
 *      Author: krusche
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <string>

#include <signal.h>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

#define K2_CALIB_CAMERA_MATRIX "cameraMatrix"
#define K2_CALIB_DISTORTION    "distortionCoefficients"
#define K2_CALIB_ROTATION      "rotation"
#define K2_CALIB_PROJECTION    "projection"
#define K2_CALIB_TRANSLATION   "translation"
#define K2_CALIB_ESSENTIAL     "essential"
#define K2_CALIB_FUNDAMENTAL   "fundamental"
#define K2_CALIB_DEPTH_SHIFT   "depthShift"

#define K2_CALIB_COLOR         "calib_color.yaml"
#define K2_CALIB_IR            "calib_ir.yaml"
#define K2_CALIB_POSE          "calib_pose.yaml"
#define K2_CALIB_DEPTH         "calib_depth.yaml"

using namespace std;
using namespace cv;

/**
Class to get all internal Parameters for Kinect2
*/
class kinect2Calib
{
public:

	bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion);
	bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation);
	bool loadCalibrationDepthFile(const std::string &filename, double &depthShift);
};



